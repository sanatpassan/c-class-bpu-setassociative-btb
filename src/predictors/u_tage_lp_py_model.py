import math

class GShareBPU:
    def __init__(self, vaddr=64, btb_depth=32, bht_depth=512, 
                 hist_lens=[0, 10, 20, 40], statesize=2, bhtcols=2, 
                 ras_depth=8, ignore=2, debug=False):
        
        self.VADDR = vaddr
        self.BTB_DEPTH = btb_depth
        self.BHT_DEPTH = bht_depth
        
        self.HIST_LENS = hist_lens
        self.MAX_HISTLEN = max(hist_lens)
        
        self.STATESIZE = statesize
        self.BHTCOLS = bhtcols
        self.RAS_DEPTH = ras_depth
        self.IGNORE = ignore
        self.compressed_enabled = True 
        self.debug = debug
        self.TAG_WIDTHS = [0, 9, 11, 13]

        self.BTB_WAYS = 16
        self.BTB_SETS = self.BTB_DEPTH // self.BTB_WAYS
        self.BTB_SET_BITS = (self.BTB_SETS - 1).bit_length()

        # --- LOOP PREDICTOR STATE ---
        self.LOOP_ENTRIES = 16
        self.LOOP_CONF_MAX = 7
        self.loop_table = [
            {
                "valid": False, "tag": 0, "trip_count": 0,
                "iter_count": 0, "confidence": 0, "direction": 0
            } for _ in range(self.LOOP_ENTRIES)
        ]

        # BTB Structure
        self.btb_entry = [[{"target": 0, "ci": "Branch", "instr16": False, "hi": False} 
                          for _ in range(self.BTB_WAYS)] for _ in range(self.BTB_SETS)]
        self.btb_tag = [[{"tag": 0, "valid": False} 
                        for _ in range(self.BTB_WAYS)] for _ in range(self.BTB_SETS)]
        self.allocate_ptr = [0] * self.BTB_SETS

        # BHT Initialized to 1 (Weakly Not-Taken)
        self.bht_base = [[1 for _ in range(bht_depth // bhtcols)] for _ in range(bhtcols)]
        self.bht_tagged = [
            [[{"state": 1, "tag": 0, "valid": False} for _ in range(bht_depth // bhtcols)] for _ in range(bhtcols)]
            for _ in range(3)
        ]
        
        self.ghr = [0, 0]
        self.ras = []
        self.bpu_enable = True

    def fn_fold(self, h, h_len, sz):
        mask = (1 << h_len) - 1
        masked_h = h & mask
        folded = 0
        for i in range(0, h_len, sz):
            shifted = masked_h >> i
            folded ^= (shifted & ((1 << sz) - 1))
        return folded

    def fn_get_index(self, h, pc, hist_l):
        index_bits = (self.BHT_DEPTH // self.BHTCOLS).bit_length() - 1
        folded_h = self.fn_fold(h, hist_l, index_bits)
        shifted_pc = pc >> self.IGNORE
        return (shifted_pc ^ folded_h) & ((1 << index_bits) - 1)

    def fn_get_tag(self, h, pc, hist_l, table_idx):
        tag_sz = self.TAG_WIDTHS[table_idx]
        folded_h = self.fn_fold(h, hist_l, tag_sz)
        shifted_pc = pc >> (self.IGNORE + 1)
        return (shifted_pc ^ folded_h) & ((1 << tag_sz) - 1)

    def predict(self, pc, ilen, fence=False, discard=False):
        target = pc + ilen
        prediction = 1
        hit, instr16, hi = False, False, False
        old_ghr = self.ghr[0] 
        
        lv_lp_hist = {"idx": -1, "count": 0}

        if not fence and self.bpu_enable:
            # 1. TAGE Direction Lookup occurs in PARALLEL with BTB (unconditional)
            idx0 = self.fn_get_index(old_ghr, pc, 0)
            idx1 = self.fn_get_index(old_ghr, pc, 10)
            idx2 = self.fn_get_index(old_ghr, pc, 20)
            idx3 = self.fn_get_index(old_ghr, pc, 40)

            t1 = self.fn_get_tag(old_ghr, pc, 10, 1)
            t2 = self.fn_get_tag(old_ghr, pc, 20, 2)
            t3 = self.fn_get_tag(old_ghr, pc, 40, 3)

            hi_val = (pc >> 1) & 0x1
            bank = int(hi_val)
            
            ent3 = self.bht_tagged[2][bank][idx3]
            ent2 = self.bht_tagged[1][bank][idx2]
            ent1 = self.bht_tagged[0][bank][idx1]

            if ent3["valid"] and ent3["tag"] == t3:
                prediction = ent3["state"]
            elif ent2["valid"] and ent2["tag"] == t2:
                prediction = ent2["state"]
            elif ent1["valid"] and ent1["tag"] == t1:
                prediction = ent1["state"]
            else:
                prediction = self.bht_base[bank][idx0]

            tage_taken = (prediction >> (self.STATESIZE - 1)) & 1

            # 2. Parallel Loop Predictor Lookup
            lp_tag = pc >> self.IGNORE
            for i in range(self.LOOP_ENTRIES):
                ent = self.loop_table[i]
                if ent["valid"] and ent["tag"] == lp_tag:
                    lv_lp_hist["idx"] = i
                    lv_lp_hist["count"] = ent["iter_count"]
                    
                    if ent["trip_count"] > 0 and ent["confidence"] >= 2:
                        if ent["confidence"] == self.LOOP_CONF_MAX:
                            if ent["iter_count"] == ent["trip_count"] - 1:
                                lp_taken = 1 - ent["direction"]
                            else:
                                lp_taken = ent["direction"]
                            prediction = (lp_taken << (self.STATESIZE - 1)) | 1
                            tage_taken = lp_taken

            # 3. BTB Lookup Structure
            shifted_pc = pc >> self.IGNORE
            set_idx = shifted_pc & (self.BTB_SETS - 1)
            btb_tag = pc >> (self.IGNORE + self.BTB_SET_BITS)

            for way in range(self.BTB_WAYS):
                if self.btb_tag[set_idx][way]["valid"] and self.btb_tag[set_idx][way]["tag"] == btb_tag:
                    hit = True
                    entry = self.btb_entry[set_idx][way]
                    break

            if hit:
                hi = entry["hi"]
                instr16 = entry["instr16"]
                
                if entry["ci"] == "Call":
                    if len(self.ras) < self.RAS_DEPTH: self.ras.append(pc + (2 if instr16 else 4))
                    target = entry["target"]
                elif entry["ci"] == "Ret": 
                    target = self.ras.pop() if self.ras else entry["target"]
                elif entry["ci"] == "Branch":
                    target = entry["target"] if tage_taken else (pc + (2 if instr16 else 4))
                else: 
                    target = entry["target"]
            else:
                # BTB Miss defaults to normal fall-through pipeline stream target
                target = pc + ilen

        return {
            "nextpc": target, "btbhit": hit, "prediction": prediction,
            "history": old_ghr, "hi": hi, "instr16": instr16,
            "lp_hist": lv_lp_hist
        }

    def train(self, pc, target, ci, history, actual_taken, btbhit, instr16, mispred, lp_hist):
        if not self.bpu_enable: return
        
        shifted_pc = pc >> self.IGNORE
        set_idx = shifted_pc & (self.BTB_SETS - 1)
        btb_tag = pc >> (self.IGNORE + self.BTB_SET_BITS)
        hi_val = (pc >> 1) & 0x1
        
        hit_way = None
        for way in range(self.BTB_WAYS):
            if self.btb_tag[set_idx][way]["valid"] and self.btb_tag[set_idx][way]["tag"] == btb_tag:
                hit_way = way
                break
                
        if hit_way is not None:
            self.btb_entry[set_idx][hit_way] = {"target": target, "ci": ci, "instr16": instr16, "hi": hi_val}
        else:
            way = self.allocate_ptr[set_idx]
            self.btb_entry[set_idx][way] = {"target": target, "ci": ci, "instr16": instr16, "hi": hi_val}
            self.btb_tag[set_idx][way] = {"tag": btb_tag, "valid": True}
            self.allocate_ptr[set_idx] = (way + 1) % self.BTB_WAYS

        if ci == "Branch":
            hi_bank = int(hi_val)

            # --- LOOP PREDICTOR TRAINING ---
            lp_tag = pc >> self.IGNORE
            maybe_lp_idx = None
            for i in range(self.LOOP_ENTRIES):
                if self.loop_table[i]["valid"] and self.loop_table[i]["tag"] == lp_tag:
                    maybe_lp_idx = i
                    break

            if maybe_lp_idx is not None:
                ent = self.loop_table[maybe_lp_idx]
                history_valid = (lp_hist["idx"] >= 0)
                
                if int(actual_taken) != ent["direction"]: 
                    if history_valid and (lp_hist["count"] + 1 == ent["trip_count"]):
                        ent["confidence"] = min(ent["confidence"] + 1, self.LOOP_CONF_MAX)
                    elif history_valid:
                        ent["trip_count"] = lp_hist["count"] + 1
                        ent["confidence"] = max(0, ent["confidence"] - 1)
                    ent["iter_count"] = 0
                else: 
                    if history_valid and ent["trip_count"] > 0 and lp_hist["count"] >= ent["trip_count"]:
                        ent["confidence"] = max(0, ent["confidence"] - 1)

                if int(actual_taken) != ent["direction"]:
                    ent["iter_count"] = 0
                else:
                    if mispred:
                        ent["iter_count"] = lp_hist["count"] + 1
            elif mispred:
                victim_idx = None
                for i in range(self.LOOP_ENTRIES):
                    if not self.loop_table[i]["valid"] or self.loop_table[i]["confidence"] == 0:
                        victim_idx = i
                        break
                
                if victim_idx is not None:
                    body_dir = 1 if target < pc else 0 
                    self.loop_table[victim_idx] = {
                        "tag": lp_tag, "trip_count": 0, "iter_count": 0, 
                        "confidence": 0, "direction": body_dir, "valid": True
                    }
                else:
                    for i in range(self.LOOP_ENTRIES):
                        if self.loop_table[i]["confidence"] > 0:
                            self.loop_table[i]["confidence"] -= 1

            # --- TAGE CORE UPDATE LOGIC (UNCONDITIONAL) ---
            idx0 = self.fn_get_index(history, pc, 0)
            idx1 = self.fn_get_index(history, pc, 10)
            idx2 = self.fn_get_index(history, pc, 20)
            idx3 = self.fn_get_index(history, pc, 40)

            t1 = self.fn_get_tag(history, pc, 10, 1)
            t2 = self.fn_get_tag(history, pc, 20, 2)
            t3 = self.fn_get_tag(history, pc, 40, 3)

            ent3 = self.bht_tagged[2][hi_bank][idx3]
            ent2 = self.bht_tagged[1][hi_bank][idx2]
            ent1 = self.bht_tagged[0][hi_bank][idx1]

            provider = 0
            original_provider_state = self.bht_base[hi_bank][idx0]

            if ent3["valid"] and ent3["tag"] == t3:
                provider = 3
                original_provider_state = ent3["state"]
            elif ent2["valid"] and ent2["tag"] == t2:
                provider = 2
                original_provider_state = ent2["state"]
            elif ent1["valid"] and ent1["tag"] == t1:
                provider = 1
                original_provider_state = ent1["state"]

            if actual_taken:
                next_tage_state = 3 if original_provider_state == 3 else original_provider_state + 1
            else:
                next_tage_state = 0 if original_provider_state == 0 else original_provider_state - 1

            if provider == 3:
                self.bht_tagged[2][hi_bank][idx3]["state"] = next_tage_state
            elif provider == 2:
                self.bht_tagged[1][hi_bank][idx2]["state"] = next_tage_state
            elif provider == 1:
                self.bht_tagged[0][hi_bank][idx1]["state"] = next_tage_state
            else:
                self.bht_base[hi_bank][idx0] = next_tage_state

            if mispred:
                init_st = 2 if actual_taken else 1
                if provider == 0:
                    self.bht_tagged[0][hi_bank][idx1] = {"state": init_st, "tag": t1, "valid": True}
                elif provider == 1:
                    self.bht_tagged[1][hi_bank][idx2] = {"state": init_st, "tag": t2, "valid": True}
                elif provider == 2:
                    self.bht_tagged[2][hi_bank][idx3] = {"state": init_st, "tag": t3, "valid": True}

    def mispredict(self, btbhit_and_branch, ghr_value):
        if btbhit_and_branch:
            ghr_value ^= (1 << (self.MAX_HISTLEN - 1))
        self.ghr[0] = ghr_value & ((1 << self.MAX_HISTLEN) - 1)