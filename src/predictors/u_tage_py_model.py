import math

class GShareBPU:
    def __init__(self, vaddr=64, btb_depth=32, bht_depth=512, 
                 hist_lens=[0, 10, 20, 40], statesize=2, bhtcols=2, 
                 ras_depth=8, ignore=2, debug=False): # Note: ignore defaults to 0 now!
        
        self.VADDR = vaddr
        self.BTB_DEPTH = btb_depth
        self.BHT_DEPTH = bht_depth
        
        # History lengths for [Base, Tagged1, Tagged2, Tagged3]
        self.HIST_LENS = hist_lens
        self.MAX_HISTLEN = max(hist_lens) # Track enough history for the longest table
        
        self.STATESIZE = statesize
        self.BHTCOLS = bhtcols
        self.RAS_DEPTH = ras_depth
        self.IGNORE = ignore
        self.BTB_INDEX_BITS = self.BTB_DEPTH.bit_length() - 1
        self.compressed_enabled = True 
        self.debug = debug
        # self.TAG_WIDTH = 13 # Standardized tag width
        self.TAG_WIDTHS = [0, 9, 11, 13]

        # BTB (16-way set associative)
        self.BTB_WAYS = 16
        self.BTB_SETS = self.BTB_DEPTH // self.BTB_WAYS
        self.BTB_SET_BITS = (self.BTB_SETS - 1).bit_length()

        self.btb_entry = [[{"target": 0, "ci": "Branch", "instr16": False, "hi": False} 
                          for _ in range(self.BTB_WAYS)] for _ in range(self.BTB_SETS)]
        self.btb_tag = [[{"tag": 0, "valid": False} 
                        for _ in range(self.BTB_WAYS)] for _ in range(self.BTB_SETS)]
        self.allocate_ptr = [0] * self.BTB_SETS

        # Table 0: Base BHT (Banked, Untagged)
        self.bht_base = [[2 for _ in range(bht_depth // bhtcols)] for _ in range(bhtcols)]
        
        # Tables 1, 2, 3: Tagged BHTs (Banked)
        self.bht_tagged = [
            [[{"state": 2, "tag": 0, "valid": False} for _ in range(bht_depth // bhtcols)] for _ in range(bhtcols)]
            for _ in range(3)
        ]
        
        self.ghr = [0, 0]
        self.ras = []
        self.bpu_enable = True

    def btb_index(self, pc):
        return (pc >> 2) & (self.BTB_SETS - 1)

    def btb_tag_value(self, pc):
        return pc >> (2 + self.BTB_SET_BITS)
    
    def get_index_tag(self, history, pc, table_idx):
        hist_len = self.HIST_LENS[table_idx]
        tag_width = self.TAG_WIDTHS[table_idx] # Get the specific width for this table
        
        index_bits = (self.BHT_DEPTH // self.BHTCOLS).bit_length() - 1
        index_mask = (1 << index_bits) - 1
        
        masked_hist = history & ((1 << hist_len) - 1) if hist_len > 0 else 0
        
        # --- Index Folding (9-bit segments) ---
        folded_hist = 0
        temp_hist = masked_hist
        while temp_hist > 0:
            folded_hist ^= (temp_hist & index_mask)
            temp_hist >>= index_bits
        index = ((pc >> self.IGNORE) ^ folded_hist) & index_mask
        
        if table_idx == 0:
            return index, None
        
        # --- Tag Folding (Table-specific width segments) ---
        tag_mask = (1 << tag_width) - 1
        folded_tag = 0
        temp_tag_hist = masked_hist
        
        # We use a different "fold" shift (tag_width - 1) to ensure the 
        # bits used for the tag aren't the exact same segments used for the index.
        while temp_tag_hist > 0:
            folded_tag ^= (temp_tag_hist & tag_mask)
            temp_tag_hist >>= (tag_width - 1) 
            
        # XORing with a shifted PC further helps avoid index/tag aliasing
        tag = ((pc >> (self.IGNORE + 1)) ^ folded_tag) & tag_mask
        
        return index, tag

    def predict(self, pc, ilen, fence=False, discard=False):
        # Compute indices and tags for all 4 tables
        indices = []
        tags = []
        for i in range(4):
            idx, tag = self.get_index_tag(self.ghr[0], pc, i)
            indices.append(idx)
            tags.append(tag)

        target = pc + ilen
        prediction = 1
        hit = False
        instr16 = False
        hi = False
        lv_ghr = self.ghr[0]

        if not fence and self.bpu_enable:
            set_idx = self.btb_index(pc)
            btb_tag = self.btb_tag_value(pc)
            entry = None

            for way in range(self.BTB_WAYS):
                if self.btb_tag[set_idx][way]["valid"] and self.btb_tag[set_idx][way]["tag"] == btb_tag:
                    hit = True
                    entry = self.btb_entry[set_idx][way]
                    break

            if hit:
                hi = entry["hi"]
                instr16 = entry["instr16"]
                target = pc + (2 if instr16 else 4)

                # RAS logic...
                if self.compressed_enabled:
                    if hi: ras_push_offset = 2 if discard else (4 if instr16 else 6)
                    else: ras_push_offset = 2 if instr16 else 4
                else: ras_push_offset = 4

                if (not self.compressed_enabled) or (hi or not discard):
                    if entry["ci"] == "Call":
                        if len(self.ras) < self.RAS_DEPTH:
                            self.ras.append(pc + ras_push_offset)
                    if entry["ci"] == "Ret":
                        target = self.ras.pop() if self.ras else entry["target"]
                    else:
                        target = entry["target"]
                    
                    if entry["ci"] in ["Ret", "JAL", "Call"]:
                        prediction = 3
                    elif entry["ci"] == "Branch":
                        bank = int(hi)
                        
                        # MULTI-TABLE LOOKUP LOGIC
                        # Check tagged tables from longest (3) to shortest (1)
                        provider_id = 0
                        for i in range(3, 0, -1):
                            idx = indices[i]
                            tag = tags[i]
                            bht_entry = self.bht_tagged[i-1][bank][idx]
                            
                            if bht_entry["valid"] and bht_entry["tag"] == tag:
                                prediction = bht_entry["state"]
                                provider_id = i
                                break
                        
                        # If no tag hit, fall back to Base Table (0)
                        if provider_id == 0:
                            prediction = self.bht_base[bank][indices[0]]
                        
                        taken = (prediction >> (self.STATESIZE - 1)) & 1
                        if taken == 0:
                            target = pc + ilen
                        
                        # Speculative GHR update (using MAX_HISTLEN)
                        old_ghr_lsb_dropped = self.ghr[0] >> 1
                        lv_ghr = (taken << (self.MAX_HISTLEN - 1)) | old_ghr_lsb_dropped

            self.ghr[0] = lv_ghr

        return {
            "nextpc": target, "btbhit": hit, "prediction": prediction,
            "history": lv_ghr, "hi": hi, "instr16": instr16
        }

    def train(self, pc, target, ci, history, actual_taken, btbhit, instr16=False):
        if not self.bpu_enable: return
        
        # ... BTB Allocation logic ...
        set_idx = self.btb_index(pc)
        btb_tag = self.btb_tag_value(pc)
        hi_val = (pc >> 1) & 0x1
        
        hit_way = None
        for way in range(self.BTB_WAYS):
            if self.btb_tag[set_idx][way]["valid"] and self.btb_tag[set_idx][way]["tag"] == btb_tag:
                hit_way = way
                break

        if hit_way is not None:
            way = hit_way
            self.btb_entry[set_idx][way] = {"target": target, "ci": ci, "instr16": instr16, "hi": hi_val}
        else:
            way = self.allocate_ptr[set_idx]
            self.btb_entry[set_idx][way] = {"target": target, "ci": ci, "instr16": instr16, "hi": hi_val}
            self.btb_tag[set_idx][way] = {"tag": btb_tag, "valid": True}
            self.allocate_ptr[set_idx] = (way + 1) % self.BTB_WAYS

        # Train BHT for conditional branches
        if ci == "Branch" and btbhit:
            # Shift the OLD history forward exactly as predict did to reconstruct indices
            shifted_history = (history << 1) & ((1 << self.MAX_HISTLEN) - 1)
            
            indices = []
            tags = []
            for i in range(4):
                idx, tag = self.get_index_tag(shifted_history, pc, i)
                indices.append(idx)
                tags.append(tag)
                
            bank = int(hi_val)
            
            # 1. FIND THE PROVIDER AGAIN
            provider_id = 0
            predicted_state = self.bht_base[bank][indices[0]]
            
            for i in range(3, 0, -1):
                idx = indices[i]
                tag = tags[i]
                bht_entry = self.bht_tagged[i-1][bank][idx]
                if bht_entry["valid"] and bht_entry["tag"] == tag:
                    predicted_state = bht_entry["state"]
                    provider_id = i
                    break
                    
            predicted_taken = (predicted_state >> (self.STATESIZE - 1)) & 1

            # 2. UPDATE THE PROVIDER
            if provider_id == 0:
                old_counter = self.bht_base[bank][indices[0]]
                self.bht_base[bank][indices[0]] = min(old_counter + 1, 3) if actual_taken else max(old_counter - 1, 0)
            else:
                entry = self.bht_tagged[provider_id-1][bank][indices[provider_id]]
                old_counter = entry["state"]
                entry["state"] = min(old_counter + 1, 3) if actual_taken else max(old_counter - 1, 0)

            # 3. ON MISPREDICTION: ALLOCATE IN THE NEXT LONGEST TABLE
            if predicted_taken != actual_taken and provider_id < 3:
                alloc_id = provider_id + 1 # Pick the next table up
                alloc_idx = indices[alloc_id]
                alloc_tag = tags[alloc_id]
                alloc_entry = self.bht_tagged[alloc_id-1][bank][alloc_idx]
                
                # Overwrite entry in the new table
                alloc_entry["valid"] = True
                alloc_entry["tag"] = alloc_tag
                alloc_entry["state"] = 2 if actual_taken else 1 # Weakly taken/not-taken

    def mispredict(self, btbhit, ghr_value):
        if btbhit:
            ghr_value ^= (1 << (self.MAX_HISTLEN - 1))

        mask = (1 << self.MAX_HISTLEN) - 1
        ghr_value &= mask

        # Direct overwrite (no CReg modeling)
        self.ghr[0] = ghr_value