import math

class GShareBPU:
    def __init__(self, vaddr=64, btb_depth=32, bht_depth=512, histlen=8, 
                 histbits=5, statesize=2, bhtcols=2, ras_depth=8, ignore=2, debug=False):
        self.VADDR = vaddr
        self.BTB_DEPTH = btb_depth
        self.BHT_DEPTH = bht_depth
        self.HISTLEN = histlen
        self.HISTBITS = histbits
        self.STATESIZE = statesize
        self.BHTCOLS = bhtcols
        self.RAS_DEPTH = ras_depth
        self.IGNORE = ignore
        self.BTB_INDEX_BITS = self.BTB_DEPTH.bit_length() - 1
        self.compressed_enabled = True 
        self.debug = debug

        # BTB (16-way set associative)
        self.BTB_WAYS = 16
        self.BTB_SETS = self.BTB_DEPTH // self.BTB_WAYS
        self.BTB_SET_BITS = (self.BTB_SETS - 1).bit_length()

        self.btb_entry = [[{"target": 0, "ci": "Branch", "instr16": False, "hi": False} 
                          for _ in range(self.BTB_WAYS)] for _ in range(self.BTB_SETS)]
        self.btb_tag = [[{"tag": 0, "valid": False} 
                        for _ in range(self.BTB_WAYS)] for _ in range(self.BTB_SETS)]
        self.allocate_ptr = [0] * self.BTB_SETS

        # BHT (banked)
        self.bht = [[2 for _ in range(bht_depth // bhtcols)] for _ in range(bhtcols)]
        self.ghr = [0, 0]
        self.ras = []
        self.bpu_enable = True

    def btb_index(self, pc):
        return (pc >> 2) & (self.BTB_SETS - 1)

    def btb_tag_value(self, pc):
        return pc >> (2 + self.BTB_SET_BITS)

    def fn_hash(self, history, pc):
        pc = pc & 0xFFFFFFFFFFFFFFFF
        index_bits = int(math.log2(self.BHT_DEPTH // self.BHTCOLS))
        index_mask = (1 << index_bits) - 1
        pc_trunc = (pc >> self.IGNORE) & index_mask
        pc_high_2 = (pc >> (self.IGNORE + index_bits)) & 0b11
        pc_hash = pc_trunc ^ pc_high_2
        
        shift = self.HISTLEN - self.HISTBITS
        _h = history >> shift
        _h &= (1 << self.HISTBITS) - 1
        shifted = (_h << (int(math.log2(self.BHT_DEPTH)) - self.HISTBITS)) & ((1 << self.HISTBITS) - 1)
        hist_hash = shifted
        return (pc_hash ^ hist_hash) & index_mask
    
    def fn_hash_basic(self, history, pc):
        index_bits = (self.BHT_DEPTH // self.BHTCOLS).bit_length() - 1
        index_mask = (1 << index_bits) - 1

        _hist = history >> 1
        index = ((pc >> self.IGNORE) ^ _hist) & index_mask
        return index

    def predict(self, pc, ilen, fence=False, discard=False):
        bht_index = self.fn_hash(self.ghr[0], pc)

        if self.debug:
            print(f"BPU : Received Request: pc: {hex(pc)} ghr:{self.ghr[0]}")
            print(f"BPU : BHTindex_:{bht_index}")

        # Default sequential PC
        target = pc + ilen
        prediction = 1
        hit = False
        instr16 = False
        hi = False
        lv_ghr = self.ghr[0]

        if not fence and self.bpu_enable:
            set_idx = self.btb_index(pc)
            tag = self.btb_tag_value(pc)
            entry = None

            for way in range(self.BTB_WAYS):
                if self.btb_tag[set_idx][way]["valid"] and self.btb_tag[set_idx][way]["tag"] == tag:
                    hit = True
                    entry = self.btb_entry[set_idx][way]
                    break

            if hit:
                hi = entry["hi"]
                instr16 = entry["instr16"]
                
                # Update sequential target based on detected instruction size
                target = pc + (2 if instr16 else 4)

                # RAS logic
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
                        prediction = self.bht[int(hi)][bht_index]
                        taken = (prediction >> (self.STATESIZE - 1)) & 1
                        
                        if taken == 0:
                            target = pc + ilen
                        
                        # Speculative GHR update
                        old_ghr_lsb_dropped = self.ghr[0] >> 1
                        lv_ghr = (taken << (self.HISTLEN - 1)) | old_ghr_lsb_dropped

            if self.debug:
                print(f"BPU : Target:{hex(target)} Pred:{prediction}")

            self.ghr[0] = lv_ghr

        return {
            "nextpc": target,
            "btbhit": hit,
            "prediction": prediction,
            "history": lv_ghr,
            "hi": hi,
            "instr16": instr16
        }

    def train(self, pc, target, ci, history, actual_taken, btbhit, instr16=False):
        if not self.bpu_enable: return
        
        set_idx = self.btb_index(pc)
        tag = self.btb_tag_value(pc)
        hi_val = (pc >> 1) & 0x1
        
        # Find or allocate BTB entry
        hit_way = None
        for way in range(self.BTB_WAYS):
            if self.btb_tag[set_idx][way]["valid"] and self.btb_tag[set_idx][way]["tag"] == tag:
                hit_way = way
                break

        if hit_way is not None:
            way = hit_way
            self.btb_entry[set_idx][way] = {"target": target, "ci": ci, "instr16": instr16, "hi": hi_val}
        else:
            way = self.allocate_ptr[set_idx]
            self.btb_entry[set_idx][way] = {"target": target, "ci": ci, "instr16": instr16, "hi": hi_val}
            self.btb_tag[set_idx][way] = {"tag": tag, "valid": True}
            self.allocate_ptr[set_idx] = (way + 1) % self.BTB_WAYS

        if self.debug:
            print("In train, btbhit : ", btbhit)

        # Train BHT for conditional branches
        if ci == "Branch" and btbhit:
            shifted_history = (history << 1) & ((1 << self.HISTLEN) - 1)
            bht_index = self.fn_hash(shifted_history, pc)

            if self.debug:
                print("In train, bht_index : ",bht_index, " pc :", hex(pc)," ghr: ", hex(self.ghr[0]))
            
            bank = int(hi_val)
            old_counter = self.bht[bank][bht_index]
            
            # 2-bit saturating update
            if actual_taken:
                state = min(old_counter + 1, 3)
            else:
                state = max(old_counter - 1, 0)
            
            self.bht[bank][bht_index] = state

    def mispredict(self, btbhit, ghr_value):
        
        if btbhit:
            ghr_value ^= (1 << (self.HISTLEN - 1))

        mask = (1 << self.HISTLEN) - 1
        ghr_value &= mask

        # Direct overwrite (no CReg modeling)
        self.ghr[0] = ghr_value