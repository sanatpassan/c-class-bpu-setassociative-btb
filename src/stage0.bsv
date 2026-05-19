// See LICENSE.iitm for license details
/*

Author: IIT Madras
*/
/*doc:overview:
This module implements the pc-gen functionality. It incorporates the branch predictor as
well. Based on the outputs of the branch predictor (if enabled), pc+4 and any flushes in the same
cycle, this module decides what the next pc to the cache and stage1 should be.

On Reset
^^^^^^^^
Once reset is de-asserted, the rg_pc register is assigned the value of the resetpc input. All other
functionality only takes action after this initialization done.

Handling Flush/Re-direction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A flush signal is received in this module under one of the following conditions:

1. A fence instruction has reached the write-back stage (if icache or bpu are implemented).
2. A sfence instruction has reached the write-back stage (if supervisor is present)
3. A misprediction occurred in the execute-stage
4. A trap or csr operation has reached the write-back stage.

Under any of the above conditions, the register rg_pc is set to the new pc that needs to be fetched.

If a flush occurs due to a fence or sfence, then in the subsequent cycle the instruction memory
subsystem receives the fence/sfence operation while the branch predictor is not accessed. In the next
cycle, the new pc with fence/sfence disabled is then sent to both the branch predictor and the
instruction memory subsystem.

Branch Prediction
^^^^^^^^^^^^^^^^^

Currently it is expected for the branch predictor to respond with the prediction in the same cycle.

If compressed is supported, the working is slightly more complex. The predictor is expected to
respond with predictions for both ``pc`` and ``pc+2``. There is also a special ``edgecase`` which
needs to be handled and has explained in detail in the description of the rule: ``rl_gen_next_pc``.
*/
package stage0;

  // -- library imports
  import FIFO           :: * ;
  import FIFOF          :: * ;
`ifdef async_rst
  import SpecialFIFOs_Modified :: * ;
`else
  import SpecialFIFOs :: * ;
`endif
  import GetPut         :: * ;
  import TxRx           :: * ;
  import icache_types   :: * ;
  import pipe_ifcs      :: * ;

  // -- project imports
  `include "Logger.bsv"
  `include "ccore_params.defines"
  import ccore_types :: * ;
`ifdef bpu
  import u_tage_lp :: * ;
`endif

  interface Ifc_stage0;
    interface Ifc_s0_common common;
    interface Ifc_s0_icache icache;
    interface Ifc_s0_tx tx;
  `ifdef bpu
    interface Ifc_s0_bpu s0_bpu;
  `endif
  endinterface: Ifc_stage0

`ifdef stage0_noinline
`ifdef core_clkgate
  (*synthesize,gate_all_clocks*)
`else
  (*synthesize*)
`endif
`endif
  module mkstage0#(Bit#(`vaddr) resetpc, parameter Bit#(`xlen) hartid) (Ifc_stage0);
    String stage0 = "";
  `ifdef bpu
    Ifc_bpu bpu <- mkbpu(hartid);
  `endif

    /*doc:fifo: This fifo holds the request to be sent to the I-cache/I-Mem subsystem. This is a
    * bypass fifo because we expect the i-cache to have a LFIFO/SizedFIFO at its end*/
    FIFOF#(IMem_core_request#(`vaddr, `iesize)) ff_to_cache <- mkBypassFIFOF;

    // holds the info to be send to the stage1
    TX#(Stage0PC#(`vaddr)) tx_tostage1 <- mkTX;

    /*doc:reg: holds the program counter*/
    Reg#(Bit#(`vaddr)) rg_pc[2] <- mkCReg(2, 'h1000);

    /*doc:reg: register to maintain the epoch in sync with execute stage*/
    Reg#(Bit#(1)) rg_eEpoch <- mkReg(0);

    /*doc:reg: register to maintain the epoch in sync with execute stage*/
    Reg#(Bit#(1)) rg_wEpoch <- mkReg(0);

    /*doc:reg: This register is used in the initializing the pc with reset-pc being driven by SoC.*/
    Reg#(Bool) rg_initialize <- mkReg(True);

    /*doc:wire: captures the condition when the reset sequence is done*/
    Wire#(Bool) wr_reset_sequence_done <- mkWire();

  `ifdef ifence
    /*doc:reg: When true indicates that the flush occurred due to a fence*/
    Reg#(Bool) rg_fence[2] <- mkCReg(2, False);
  `endif

  `ifdef supervisor
    /*doc:reg: When true indicates that the flush occurred due to a sfence*/
    Reg#(Bool) rg_sfence[2] <- mkCReg(2, False);
  `endif

  `ifdef hypervisor
    /*doc:reg: When true indicates that the flush occurred due to a hfence*/
    Reg#(Bool) rg_hfence[2] <- mkCReg(2, False);
  `endif

`ifdef bpu
  `ifdef compressed
    /*doc:reg: This register when Valid indicates a 32-bit control instruction on a
    2-byte boundary is predicted taken. Thus the cache is send the redirect after the upper-16 bits
    of the instruction have been fetched.*/
    Reg#(Maybe#(Bit#(`vaddr))) rg_delayed_redirect[2] <- mkCReg(2, tagged Invalid);
  `endif
`endif

    // local variable to hold the next+4 pc value. Ensure only a single adder is used.
    let curr_epoch = {rg_eEpoch, rg_wEpoch};

    /*doc:rule: This rule will fire only once immediately after reset is de-asserted. The rg_pc is
    initialized with the resetpc argument*/
    rule rl_initialize (rg_initialize && wr_reset_sequence_done);
      rg_initialize <= False;
      rg_pc[1] <= resetpc;
      `logLevel( stage0, 0, $format("STAGE0: Setting PC:%h",resetpc))
    endrule

    /*doc:rule: This rule muxes between pc+4 and the prediction provided by the bpu.
    When the rg_fence is True the request is sent only to the cache and not to stage1.
    The same is the case with rg_sfence. This is because the i-cache does not respond on a fence
    request to stage1. When either rg_fence or rg_sfence are set to True, the rg_pc is not
    updated. It re-used as a valid request in the next cycle by which the rg_fence and rg_sfence
    have been made false.

    The pc sent to the stage and the i-cache has to always be 4-byte aligned. This is because the
    i-cache only responds with 32-bits at a time

    When compressed is enabled, things get a bit more complicated. Consider the following example:

    ```
       0x100a : blt a0, a1, 0x8000000
    ```
    Now when the bpu predicts this branch to be taken it will suggest the next pc should be
    `0x80000000`. However, when this pc was requested to the bpu the pc sent to the cache was
    `0x1008`. So we send another pc of `0x100c` to the cache to get upper 16-bits of this branch and
    only then send the new target pc of `0x80000000`.

    We refer to this scenario as an edge case and the register rg_delayed_redirect ensures correct
    pc sequences are sent to the cache and stage1

    */
    rule rl_gen_next_pc (tx_tostage1.u.notFull && !rg_initialize && wr_reset_sequence_done);
      `ifdef bpu
        PredictionResponse bpu_resp = ?;
      `endif

        let nextpc = (rg_pc[0] & signExtend(3'b100)) + 4;
        `logLevel( stage0, 0, $format("STAGE0: nextpc: %h ",nextpc `ifdef ifence ," fencei:%b",rg_fence[0] `endif ))

      `ifdef bpu
        // bpu is flushed in case of ifence and not for sfence
        if( `ifdef supervisor !rg_sfence[0] && `endif 
            `ifdef hypervisor !rg_hfence[0] && `endif True) begin
          let bpuresp <- bpu.mav_prediction_response(PredictionRequest{pc: rg_pc[0]
                                    `ifdef ifence     ,fence: rg_fence[0] `endif
                                    `ifdef compressed , discard: (rg_pc[0][1]==1) `endif });
        `ifdef compressed
          // check for edge case
          Bool edgecase = bpuresp.btbresponse.hi && !bpuresp.instr16;
        `endif
          if (bpuresp.btbresponse.prediction[`statesize - 1] == 1 && bpuresp.btbresponse.btbhit 
                                    `ifdef compressed && !edgecase `endif )
            nextpc = bpuresp.nextpc;
        `ifdef compressed
          // send new target from previously predicted edgecase Ci
          if(rg_delayed_redirect[0] matches tagged Valid .rpc) begin
            nextpc = rpc;
            rg_delayed_redirect[0] <= tagged Invalid;
          end
          // send pc+4 and store the target for the next round
          else if(edgecase && bpuresp.btbresponse.prediction > 1 && !rg_fence[0])
            rg_delayed_redirect[0] <= tagged Valid bpuresp.nextpc;
        `endif

          bpu_resp = bpuresp;
          `logLevel( stage0, 0, $format("[%2d]STAGE0: BPU response:",hartid,fshow(bpu_resp)))
        end
      `endif
      `logLevel( stage0, 0, $format("STAGE0: nextpc1: %h ",nextpc))

      // don't update the pc in case fence or sfence instruction
      if( `ifdef ifence !rg_fence[0] && `endif 
          `ifdef supervisor !rg_sfence[0] && `endif
          `ifdef hypervisor !rg_hfence[0] && `endif True)
        rg_pc[0] <= nextpc ;

      `ifdef ifence
        if(rg_fence[0])
          rg_fence[0] <= False;
      `endif

      `ifdef supervisor
        if(rg_sfence[0])
          rg_sfence[0] <= False;
      `endif

      `ifdef hypervisor
        if(rg_hfence[0])
          rg_hfence[0] <= False;
      `endif

        `logLevel( stage0, 0, $format("[%2d]STAGE0: Sending PC:%h to I$. ",hartid, rg_pc[0] & signExtend(3'b100)))
        ff_to_cache.enq(IMem_core_request{address  : rg_pc[0] & signExtend(3'b100),
                                        epochs  : curr_epoch
                  `ifdef supervisor    ,sfence  : rg_sfence[0]    `endif
                  `ifdef hypervisor    ,hfence  : rg_hfence[0]    `endif
                  `ifdef ifence        ,fence   : rg_fence[0]     `endif });

        if( `ifdef ifence !rg_fence[0] && `endif 
            `ifdef supervisor !rg_sfence[0] && `endif 
            `ifdef hypervisor !rg_hfence[0] && `endif True) begin
          tx_tostage1.u.enq(Stage0PC{   address      : rg_pc[0] & signExtend(3'b100)
                    `ifdef compressed   ,discard     : rg_pc[0][1]==1        `endif
                    `ifdef bpu          ,btbresponse : bpu_resp.btbresponse, lp_hist : bpu_resp.lp_hist `endif  });
          `logLevel( stage0, 0, $format("[%2d]STAGE0: Sending PC:%h to Stage1",hartid, rg_pc[0]))
        end
    endrule

    interface icache = interface Ifc_s0_icache
      interface to_icache = toGet(ff_to_cache);
    endinterface;

    interface tx = interface Ifc_s0_tx
      interface tx_to_stage1 = tx_tostage1.e;
    endinterface;

    interface common = interface Ifc_s0_common
      method Action ma_update_eEpoch ();
        rg_eEpoch <= ~rg_eEpoch;
      endmethod
  
      method Action ma_update_wEpoch ();
        rg_wEpoch <= ~rg_wEpoch;
      endmethod
      method Action ma_reset_done(Bool _done);
        wr_reset_sequence_done <= _done;
      endmethod:ma_reset_done

      method Action ma_flush (Stage0Flush fl) if(!rg_initialize && wr_reset_sequence_done);
        `logLevel( stage0, 1, $format("[%2d]STAGE0: Recieved Flush:",hartid,fshow(fl)))
      `ifdef ifence
        rg_fence[1] <= fl.fence;
      `endif
      `ifdef supervisor
        rg_sfence[1] <= fl.sfence;
      `endif
    `ifdef hypervisor
      rg_hfence[1] <= fl.hfence;
    `endif
        rg_pc[1] <= fl.pc;
    `ifdef bpu
      `ifdef compressed
        // reset any delayed-redirect
        rg_delayed_redirect[1] <= tagged Invalid;
      `endif
    `endif
      endmethod
    endinterface;

`ifdef bpu
    interface s0_bpu = interface Ifc_s0_bpu
      method ma_train_bpu   = bpu.ma_train_bpu;
    `ifdef gshare
      method ma_mispredict  = bpu.ma_mispredict;
    `endif
      method ma_bpu_enable  = bpu.ma_bpu_enable;
    endinterface;
`endif

  endmodule: mkstage0
endpackage: stage0

