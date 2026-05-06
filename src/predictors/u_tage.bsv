// See LICENSE.iitm for license details
/*

Author : IIT Madras
Details: This module implements a set-associative MICRO TAGE branch predictor with
Return-Address-Stack support. It basically has the prediction and training phase. The comments on
the respective methods describe their operations.

--------------------------------------------------------------------------------------------------
*/
package u_tage;

  // -- library imports
  import Assert :: *;
  import ConfigReg :: * ;
  import Vector :: * ;
  import OInt :: * ;
  import RegFile :: * ;

  // -- project imports
  `include "Logger.bsv"
  `include "ccore_params.defines"
  import ccore_types :: *;
`ifdef bpu_ras
  import stack :: * ;
`endif

  `define ignore 2
  `define WAYS 16
  typedef TDiv#(`btbdepth, `WAYS) NUM_SETS;

  // the following macro describes the number of banks the bht array is split into
  `ifdef compressed
    `define bhtcols 2
  `else
    `define bhtcols 1
  `endif

  /*doc:struct: This struct defines the fields of each entry in the Branch-Target-Buffer*/
  typedef struct{
    Bit#(`vaddr)  target;               // full target virtual address
    ControlInsn   ci;                   // indicate the type of entry. Branch, JAL, Call, Ret.
  `ifdef compressed
    Bool instr16 ;                      // when true indicates a 32-bit Ci at non-4-byte address
    Bool hi;                            // when true indicates higher 16-bit is a Ci
  `endif
  } BTBEntry deriving(Bits, Eq, FShow);

  /*doc:macro: Definitions for Tag Widths per Table for the TAGE predictor.*/
  `define TAG_1 9
  `define TAG_2 11
  `define TAG_3 13

  /*doc:struct: This struct holds the state, tag, and valid bit for entries in the tagged BHT tables.*/
  typedef struct {
    Bit#(`statesize) state;
    Bit#(tag_w) tag;
    Bool valid;
  } TaggedEntry#(numeric type tag_w) deriving (Bits, Eq, FShow);

  /*doc:struct: This struct holds the tag and valid bit of each BTB entry.
  Each entry corresponds to a tag for a 4-byte aligned address. This means that this tag can be a
  hit of at-most 2 instructions when compressed is supported. To distinguish between the 2
  instructions we have provided a 'hi' field in BTBEntry, which when true indicates that the higher
  instruction within the 4-byte address is a hit/trained */
  typedef struct{
    Bit#(TSub#(`vaddr, TAdd#(TLog#(NUM_SETS), `ignore))) tag;
    Bool valid;
  } BTBTag deriving(Bits, Eq, FShow);


  /*doc : func : function to calculate the hash address to access the branch-history-table.
  1. Here the lower 2 bits of the pc are ignored since they are always going to be 0 for a
  non-compressed supported system.
  2. We take lower (extrahist+histlen) bit of pc and XOR with the next set of (extrahist+histlen)
  bits of pc and with the GHR register.
  Note here that the ghr is right shifted i.e. the speculation is inserted at the MSB and hence the
  usage of truncateLSB.
  This has proven to be a better hash function, not so costly. It is completely empirical and better
  hash functions could exist and can replace this function with that to evaluate what works best.
  */
  /*function Bit#(TLog#(TDiv#(`bhtdepth,`bhtcols))) fn_hash (
                                      Bit#(`histlen) history, Bit#(`vaddr) pc);
    return truncate(pc >> `ignore) ^ truncate(pc >> (`ignore +
                                                     valueOf(TLog#(TDiv#(`bhtdepth,`bhtcols)))))
                                   ^ truncateLSB(history);
  endfunction*/

  /*doc:func: This function performs static folding of the global history buffer.
  It computes the folded history to a target size `sz` using explicit truncation. 
  Provisos are included to mathematically guarantee to the compiler that the truncation is valid.*/
  function Bit#(sz) fn_fold(Bit#(`histlen) h, Integer h_len)
    provisos(Add#(a__, sz, `histlen)); 

    Bit#(sz) folded = 0;
    Bit#(`histlen) mask = (1 << h_len) - 1;
    Bit#(`histlen) masked_h = h & mask;
    
    for(Integer i=0; i < h_len; i = i + valueOf(sz)) begin
       Bit#(`histlen) shifted = masked_h >> i;
       folded = folded ^ truncate(shifted); 
    end
    return folded;
  endfunction

  /*doc:func: This function acts as a dynamically sized Index generator.
  It dynamically generates an index for the tagged tables by folding the global history 
  and XORing it with the truncated PC.*/
  function Bit#(idx_sz) fn_get_index(Bit#(`histlen) h, Bit#(`vaddr) pc, Integer hist_l)
    provisos(
        Add#(a__, idx_sz, `histlen), 
        Add#(b__, idx_sz, `vaddr)   
    );
    
    Bit#(idx_sz) folded_h = fn_fold(h, hist_l); 
    return truncate(pc >> `ignore) ^ folded_h;
  endfunction

  /*doc:func: This function acts as a dynamically sized Tag generator.
  It dynamically generates a tag for the BTB entries by folding the history 
  and XORing it with the shifted PC.*/
  function Bit#(tag_sz) fn_get_tag(Bit#(`histlen) h, Bit#(`vaddr) pc, Integer hist_l)
    provisos(
        Add#(a__, tag_sz, `histlen), 
        Add#(b__, tag_sz, `vaddr)    
    );
    
    Bit#(tag_sz) folded_h = fn_fold(h, hist_l);
    return truncate(pc >> (`ignore + 1)) ^ folded_h;
  endfunction

  interface Ifc_bpu;
    /*doc : method : receive the request of new pc and return the next pc in case of hit. */
    method ActionValue#(PredictionResponse) mav_prediction_response (PredictionRequest r);

    /*doc : method : method to train the BTB and BHT tables based on the evaluation from execute
    stage*/
	  method Action ma_train_bpu (Training_data td);

    /*doc : method : This method is fired when there is a misprediction.
    It received 2 fields. A boolean indicating if the instructin was a conditional branch or not.
    The second field contains the GHR value after predicting the same instruction. In case
    of a conditional branch the LSB bit of this GHR is negated and this value is restored in the
    rg_ghr register. Otherwise, the GHR directly written to the rg_ghr register. */
    method Action ma_mispredict (Tuple2#(Bool, Bit#(`histlen)) g);

    /*doc : method : This method captures if the bpu is enabled through csr or not*/
    method Action ma_bpu_enable (Bool e);
  endinterface

`ifdef bpu_noinline
`ifdef core_clkgate
  (*synthesize,gate_all_clocks*)
`else
  (*synthesize*)
`endif
`endif
  module mkbpu#(parameter Bit#(`xlen) hartid) (Ifc_bpu);

    String bpu = "";

  `ifdef bpu_ras
    Ifc_stack#(`vaddr, `rasdepth) ras_stack <- mkstack;
  `endif

    /*doc : vec : This vector of register holds the BTB entries. We use vector instead of array
    to leverage the select function provided by bluespec*/
    Vector#(NUM_SETS, Vector#(`WAYS, Reg#(BTBEntry))) v_reg_btb_entry <-
                                                  replicateM(replicateM(mkReg(BTBEntry{target: ?, ci : Branch
                                           `ifdef compressed ,instr16: False, hi:False `endif })));

    /*doc : vec : This vector holds the BTB tags and the respecitve valid bits. This has been split
    from the BTB entries for better hw of CAM look-ups and index retrieval */
    Vector#(NUM_SETS, Vector#(`WAYS, Reg#(BTBTag))) v_reg_btb_tag <- replicateM(replicateM(mkReg(unpack(0))));

    /*doc : reg : This array holds the branch history table. The bht table banked into `bhtcols
    banks. In case of compressed `bhtcols is 2 else 1. By banking it becomes easy to access the bht
    in case of compressed support since we are storing only one BTB per 4-byte align addresses.
    Each entry is `statesize-bits wide and represents a up/down saturated counter.
    The reset value is set to 1 */
    /*Reg#(Bit#(`statesize)) rg_bht_arr[`bhtcols][`bhtdepth/`bhtcols];
    for(Integer i = 0; i < `bhtcols; i =  i + 1)
      for(Integer j = 0; j < `bhtdepth/`bhtcols ; j =  j + 1)
        rg_bht_arr[i][j] <- mkReg(1);*/
    
    /*doc:reg: Table 0: This register file represents the Base Branch History Table (BHT). 
    It is untagged and acts as the default fallback predictor.*/
    RegFile#(Bit#(TLog#(TDiv#(`bhtdepth,`bhtcols))), Bit#(`statesize)) rg_bht_base[`bhtcols];
    
    /*doc:reg: Tables 1, 2, and 3: These register files represent the tagged BHT tables.
    They are used to provide predictions based on varying, longer history lengths.*/
    RegFile#(Bit#(TLog#(TDiv#(`bhtdepth,`bhtcols))), TaggedEntry#(`TAG_1)) rf_tagged_1[`bhtcols];
    RegFile#(Bit#(TLog#(TDiv#(`bhtdepth,`bhtcols))), TaggedEntry#(`TAG_2)) rf_tagged_2[`bhtcols];
    RegFile#(Bit#(TLog#(TDiv#(`bhtdepth,`bhtcols))), TaggedEntry#(`TAG_3)) rf_tagged_3[`bhtcols];

    for (Integer i = 0; i < `bhtcols; i = i + 1) begin
      rg_bht_base[i] <- mkRegFileWCF(0, fromInteger(valueOf(TDiv#(`bhtdepth,`bhtcols))-1));
      rf_tagged_1[i] <- mkRegFileWCF(0, fromInteger(valueOf(TDiv#(`bhtdepth,`bhtcols))-1));
      rf_tagged_2[i] <- mkRegFileWCF(0, fromInteger(valueOf(TDiv#(`bhtdepth,`bhtcols))-1));
      rf_tagged_3[i] <- mkRegFileWCF(0, fromInteger(valueOf(TDiv#(`bhtdepth,`bhtcols))-1));
    end

    /*doc:reg: */
    Reg#(Bit#(TLog#(TDiv#(`bhtdepth, `bhtcols)))) rg_bht_index <- mkReg(0);
    /*doc : reg : This register points to the next entry in the Fully associative BTB that should
    be allocated for a new entry */
    Vector#(NUM_SETS, Reg#(Bit#(TLog#(`WAYS)))) rg_allocate <- replicateM(mkReg(0));

    /*doc : reg : This register holds the global history buffer. There are two methods which can
    update this register: mav_prediction_response and ma_mispredict. The former is called every
    time a new pc is generted and updates the regiser speculatively for conditional branches
    which are a hit in the BTB. The later method called when a mis-prediction occurs and restores
    the register with the non-speculative version.
    Both of these method are in conflict with each other. One way to resolve this would be create a
    preempts attribute given ma_mispredict a higher priority since it doesn't make sense to provide
    a prediction knowing the pipe has flushed. However, this solution would create a path from the
    ma_mispredict enable method to the mav_prediction_response output ready signal making it the
    critical path.
    Alternate to that is to implement this register as a CReg where the ma_mispredict value shadows
    the value updated by the mav_prediction_response method. This remove the above critical path.
    */
    Reg#(Bit#(`histlen)) rg_ghr[2] <- mkCReg(2, 0);

    /*doc : wire : This wire indicates if the predictor is enabled or disabled by the csr config*/
    Wire#(Bool) wr_bpu_enable <- mkWire();

  `ifdef ifence
    /*doc : reg : When true this register flushes all the entries and cleans up the btb*/
    ConfigReg#(Bool) rg_initialize <- mkConfigReg(False);

    /*doc : rule : This rule flushes the btb and puts it back to the initial reset state.
    This rule would be called each time a fence.i is being performed. This rule will also reset the
    ghr and rg_allocate register*/
    rule rl_initialize (rg_initialize);
      for(Integer i = 0; i < valueOf(NUM_SETS); i = i + 1)
        for(Integer w = 0; w < `WAYS; w = w + 1)
          v_reg_btb_tag[i][w] <= unpack(0);
      for(Integer i = 0; i < `bhtcols ; i = i + 1) begin
        rg_bht_base[i].upd(rg_bht_index, 1);
        rf_tagged_1[i].upd(rg_bht_index, TaggedEntry{state: 1, tag: 0, valid: False});
        rf_tagged_2[i].upd(rg_bht_index, TaggedEntry{state: 1, tag: 0, valid: False});
        rf_tagged_3[i].upd(rg_bht_index, TaggedEntry{state: 1, tag: 0, valid: False});
      end
  
      if (rg_bht_index == fromInteger(valueOf(TDiv#(`bhtdepth,`bhtcols))-1))
        rg_initialize <= False;
      rg_bht_index <= rg_bht_index + 1;
      rg_ghr[1] <= 0;
      for(Integer s = 0; s < valueOf(NUM_SETS); s = s + 1)
        rg_allocate[s] <= 0;
    `ifdef bpu_ras
      ras_stack.clear;
    `endif
    endrule

    /*doc:method: This method provides prediction for a requested PC.
    If a fence.i is requested, then the rg_initialize register is set to true.

    The index of the bht is obtained using the hash function above on the pc and the current value
    of GHR. This index is then used to find the entry in the BHT.

    We then perform a set-associative look-up on the BTB. The PC is first split into index bits (select 
    the BTB set) and tag bits (compared within the selected set). Using the index, we access one BTB set 
    and perform parallel tag comparison across all WAYS of that set. Tag comparison is performed across 
    all WAYS of the indexed set. The match_ vector indicates which way matches the tag. By nature of how 
    training and prediction is performed, we expect match_ vector to be a one-hot vector within the 
    selected set i.e. only one entry is a hit in the BTB set. Multiple entries can't be a hit since 
    update comes from only one source.

    Using the match_ vector and BSV's select function, the matching way inside the indexed set is chosen.

    Depending on the ci type the prediction variable is set either to 3 or the value in the BHT
    entry that we indexed earlier.

    In case of a BTB hit and the ci being a Branch, the ghr is left-shifted and the lsb is set to 1
    if predicted taken else 0. This ghr is also sent back out along with the prediction and target
    address.

    We also send out a boolean value indicating if the pc caused a hit in the BTB or not.

    Fence: This feature is required for self-modifying codes. Software is required to conduct an
    fence.i each time the text-section is modified by the software. When this happens we need to
    flush the branch predictor as well else non-branch instructions could be treated as predicted
    taken leading to wrong behavior

    Working of RAS: Earlier versions of the predictor included a separate method which would push
    the return address onto the RAS. This address came from the execute stage which when a Call
    instruction was detected. If the BTB was a hit for a pc and it detected a Ret type ci then the
    RAS popped. However with this architecture you could have a push happening from an execute stage
    and a ret being detected in the predictor, this return would never see this latest push and thus
    would pick the wrong address from the RAS. So essentially the RAS would work only if the
    call-ret are a few number of instructions apart, for smaller functions the RAS would fail
    consistently.

    To fix this problem, we push and pop with the predictor itself. If a pc is a btb hit and is a
    Call type ci, the pc+4 value if pushed on the Stack. If the subsequent pc was a btb hit and a
    Ret type ci, it would immediately pick up the RAS top which would be correct. Thus, an empty
    function would also benefit from this mechanism.
    */
    method ActionValue#(PredictionResponse) mav_prediction_response (PredictionRequest r)
                                                         `ifdef ifence if(!rg_initialize) `endif ;
      `logLevel( bpu, 0, $format("[%2d]BPU : Received Request: ",hartid, fshow(r),
                                 " ghr:%h",hartid,rg_ghr[0]))
    `ifdef ifence
      if( r.fence && wr_bpu_enable)
        rg_initialize <= True;
    `endif
      Bit#(`statesize) branch_state_ [`bhtcols];

      Bit#(`statesize) prediction_ = 1;
      Bit#(`vaddr) target_ = r.pc;
      Bool hit = False;
      Bit#(`histlen) lv_ghr = rg_ghr[0];
      Bool hi = False;
    `ifdef compressed
      Bool instr16 = False;
    `endif

      if(!r.fence && wr_bpu_enable) begin
        let shifted_pc = r.pc >> `ignore;
        Bit#(TLog#(NUM_SETS)) index = shifted_pc[valueOf(TLog#(NUM_SETS))-1:0];
        Bit#(TSub#(`vaddr, TAdd#(TLog#(NUM_SETS), `ignore))) tag = truncateLSB(r.pc);

        Vector#(`WAYS, Bool) match_;
        for(Integer i = 0; i < `WAYS; i = i + 1)
          match_[i] = (v_reg_btb_tag[index][i].tag == tag && v_reg_btb_tag[index][i].valid);

        let match_bits = pack(match_);
        hit = unpack(|match_bits);
        let hit_entry = select(readVReg(v_reg_btb_entry[index]), unpack(match_bits));

        if(hit) begin
          `logLevel( bpu, 1, $format("[%2d]BPU : BTB Hit: ",hartid,fshow(hit_entry)))
        end

        `ifdef compressed
          instr16 = hit_entry.instr16;
          hi = hit_entry.hi;
        `endif

        if(hit) begin
        `ifdef bpu_ras
          `ifdef compressed
            Bit#(`vaddr) ras_push_offset = hit_entry.hi? hit_entry.instr16? r.discard? 2: 4
                                                                          : r.discard? 4: 6
                                                       : hit_entry.instr16? 2: 4;
          `else
            Bit#(`vaddr) ras_push_offset = 4;
          `endif
        if(True `ifdef compressed && ( hit_entry.hi || !r.discard ) `endif ) begin
          if(hit_entry.ci == Call)begin // push to ras in case of Call instructions
            Bit#(`vaddr) push_pc = r.pc + ras_push_offset;
            `logLevel( bpu, 1, $format("[%2d]BPU: Pushing1 to RAS:%h",hartid,(push_pc)))
            ras_stack.push(push_pc);
          end

          if(hit_entry.ci == Ret) begin // pop from ras in case of Ret instructions
            target_ = ras_stack.top;
            ras_stack.pop;
            `logLevel( bpu, 1, $format("[%2d]BPU: Choosing from top RAS:%h",hartid,target_))
          end
          else
        `endif
          target_ = hit_entry.target;

          // update only if hi is True or if discard is false. No point in predicting dicarded inst.
            if(hit_entry.ci == Ret ||  hit_entry.ci == Call || hit_entry.ci == JAL )
               prediction_ = 3;

            if(hit_entry.ci == Branch) begin
              // Calculate all possible hits
              let idx0 = fn_get_index(rg_ghr[0], r.pc, 0);
              let idx1 = fn_get_index(rg_ghr[0], r.pc, 10);
              let idx2 = fn_get_index(rg_ghr[0], r.pc, 20);
              let idx3 = fn_get_index(rg_ghr[0], r.pc, 40);

              let tag1 = fn_get_tag(rg_ghr[0], r.pc, 10);
              let tag2 = fn_get_tag(rg_ghr[0], r.pc, 20);
              let tag3 = fn_get_tag(rg_ghr[0], r.pc, 40);

              let ent1 = rf_tagged_1[pack(hi)].sub(idx1);
              let ent2 = rf_tagged_2[pack(hi)].sub(idx2);
              let ent3 = rf_tagged_3[pack(hi)].sub(idx3);

              // Priority Lookup (Longest history first)
              if(ent3.valid && ent3.tag == tag3)
                prediction_ = ent3.state;
              else if(ent2.valid && ent2.tag == tag2)
                prediction_ = ent2.state;
              else if(ent1.valid && ent1.tag == tag1)
                prediction_ = ent1.state;
              else
                prediction_ = rg_bht_base[pack(hi)].sub(idx0);
                
              lv_ghr = {prediction_[`statesize - 1], truncateLSB(rg_ghr[0])};
              `logLevel( bpu, 0, $format("[%2d]BPU : New GHR:%h",hartid, lv_ghr))
            end
          end

        end
      `ifdef ifence if(!r.fence) `endif
          rg_ghr[0] <= lv_ghr;

        `ifdef ASSERT
          dynamicAssert(countOnes(match_bits) < 2, "Multiple Matches in BTB");
        `endif
      end

      let btbresponse = BTBResponse{prediction: prediction_, btbhit: hit
                        `ifdef compressed , hi: hi `endif
                        `ifdef gshare , history : lv_ghr`endif };

      return PredictionResponse{ nextpc : target_, btbresponse: btbresponse
                                `ifdef compressed ,instr16 : instr16 `endif };
    endmethod

    /*doc:method: This method is called for all unconditional and conditional jumps.
    Using the pc of the instruction we first compute the BTB index and tag. The index selects the 
    BTB set and the tag is used to match entries within that set. If an entry already exists in the 
    indexed set (tag match and valid bit set), then the entry is updated with a new/same target 
    from the execute stage.

    If the entry does not exist, then a new entry is allotted in the BTB set depending on the 
    rg_allocate[set_idx] value. This acts as a per-set allocation pointer for replacement among the WAYS.

    Additionally in case of conditional branches, the bht is again indexed using the pc and the ghr.
    This entry is updated only if the BTB was a hit during prediction i.e. only on the second
    instance of the branch the bht gets updated.

    It was first thought to be better to send the btbindex along the pipe to reduce the additional
    look-up hw here. However, for really small loops its possible that while training for an entry
    in a cycle, the same instruction is getting predicted again. This will cause a miss in the
    prediction and the training of the second instance would lead to allocating a new entry. This
    would lead to duplicates and thus would require zapping them - another simultaneous look-up. It
    seems the current approach does to seem close on required frequencies.
    */
    method Action ma_train_bpu (Training_data d) if(wr_bpu_enable
                                                          `ifdef ifence && !rg_initialize `endif );
      `logLevel( bpu, 4, $format("[%2d]BPU : Received Training: ",hartid,fshow(d)))

      let shifted_pc = d.pc >> `ignore;
      Bit#(TLog#(NUM_SETS)) set_idx = shifted_pc[valueOf(TLog#(NUM_SETS))-1:0];
      Bit#(TSub#(`vaddr, TAdd#(TLog#(NUM_SETS), `ignore))) tag = truncateLSB(d.pc);
      function Bool fn_way_match (BTBTag a);
        return (a.tag == tag && a.valid);
      endfunction

      let hit_index_ = findIndex(fn_way_match, readVReg(v_reg_btb_tag[set_idx]));

      if(hit_index_ matches tagged Valid .h) begin
        v_reg_btb_entry[set_idx][h] <= BTBEntry{ target : d.target, ci : d.ci
                            `ifdef compressed ,instr16: d.instr16, hi:unpack(d.pc[1]) `endif };
        `logLevel( bpu, 4, $format("[%2d]BPU : Training existing Entry index: %d",hartid,h))
      end
      else begin
        `logLevel( bpu, 4, $format("[%2d]BPU : Allocating new index: %d",hartid,rg_allocate[set_idx]))
        Bit#(TLog#(`WAYS)) way = rg_allocate[set_idx];
        v_reg_btb_entry[set_idx][way] <= BTBEntry{ target : d.target, ci : d.ci
                            `ifdef compressed ,instr16: d.instr16, hi:unpack(d.pc[1]) `endif };
        v_reg_btb_tag[set_idx][way] <= BTBTag{tag: tag, valid: True};
        rg_allocate[set_idx] <= rg_allocate[set_idx] + 1;
        if(v_reg_btb_tag[set_idx][way].valid)
          `logLevel( bpu, 4, $format("[%2d]BPU : Conflict Detected",hartid))
      end

      // we use the ghr version before the prediction to train the BHT
      if(d.ci == Branch && d.btbhit) begin
        Bit#(`histlen) old_ghr = d.history << 1; 
        let hi_bank = d.pc[1];

        // Re-identify the provider table that supplied the original prediction
        let idx0 = fn_get_index(old_ghr, d.pc, 0);
        let idx1 = fn_get_index(old_ghr, d.pc, 10);
        let idx2 = fn_get_index(old_ghr, d.pc, 20);
        let idx3 = fn_get_index(old_ghr, d.pc, 40);

        let t1 = fn_get_tag(old_ghr, d.pc, 10);
        let t2 = fn_get_tag(old_ghr, d.pc, 20);
        let t3 = fn_get_tag(old_ghr, d.pc, 40);

        let ent1 = rf_tagged_1[hi_bank].sub(idx1);
        let ent2 = rf_tagged_2[hi_bank].sub(idx2);
        let ent3 = rf_tagged_3[hi_bank].sub(idx3);

        Integer provider = 0;
        Bit#(`statesize) original_provider_state = rg_bht_base[hi_bank].sub(idx0); 
        
        if (ent3.valid && ent3.tag == t3) begin
            provider = 3;
            original_provider_state = ent3.state;
        end else if (ent2.valid && ent2.tag == t2) begin
            provider = 2;
            original_provider_state = ent2.state;
        end else if (ent1.valid && ent1.tag == t1) begin
            provider = 1;
            original_provider_state = ent1.state;
        end

        // Update the state of the provider table with the actual state computed by the Execute stage
        if(provider == 3) rf_tagged_3[hi_bank].upd(idx3, TaggedEntry{state: d.state, tag: t3, valid: True});
        else if(provider == 2) rf_tagged_2[hi_bank].upd(idx2, TaggedEntry{state: d.state, tag: t2, valid: True});
        else if(provider == 1) rf_tagged_1[hi_bank].upd(idx1, TaggedEntry{state: d.state, tag: t1, valid: True});
        else rg_bht_base[hi_bank].upd(idx0, d.state);

        // Allocate a new entry on a misprediction
        Bool actual_taken = (d.state > original_provider_state) || (d.state == 3);
        Bool predicted_taken = unpack(original_provider_state[`statesize-1]);
        
        if(predicted_taken != actual_taken && provider < 3) begin
           // Initialize new entry to weakly taken (2) or weakly not-taken (1)
           Bit#(`statesize) init_st = actual_taken ? 2 : 1;
           
           if(provider == 0) rf_tagged_1[hi_bank].upd(idx1, TaggedEntry{state: init_st, tag: t1, valid: True});
           else if(provider == 1) rf_tagged_2[hi_bank].upd(idx2, TaggedEntry{state: init_st, tag: t2, valid: True});
           else if(provider == 2) rf_tagged_3[hi_bank].upd(idx3, TaggedEntry{state: init_st, tag: t3, valid: True});
        end
      end
    endmethod

    /*doc:method: This method is called each time the evaluation stage detects a mis-prediction. If
    the misprediction was due to a conditional branch then the ghr is fixed by flipping the lsb
    and then writing it to the rg_ghr.
    */
    method Action ma_mispredict (Tuple2#(Bool, Bit#(`histlen)) g)
                                                         `ifdef ifence if(!rg_initialize) `endif ;
      let {btbhit_and_branch, ghr} = g;
      if(btbhit_and_branch)
        ghr[`histlen-1] = ~ghr[`histlen-1];
      `logLevel( bpu, 4, $format("[%2d]BPU : Misprediction fired. Restoring ghr:%h",hartid,
                                                                                              ghr))
      rg_ghr[1] <= ghr;
    endmethod

    method Action ma_bpu_enable (Bool e);
      wr_bpu_enable <= e;
    endmethod

  endmodule
endpackage


