// See LICENSE.iitm for license details
/*
Author: IIT Madras
Created on: Friday 18 June 2021 05:13:19 PM

*/
package pipe_ifcs ;
import FIFOF        :: * ;
import Vector       :: * ;
`ifdef async_rst
import SpecialFIFOs_Modified :: * ;
`else
import SpecialFIFOs :: * ;
`endif
import FIFOF        :: * ;
import GetPut       :: * ;
import Connectable  :: * ;

import ccore_types  :: * ;
import dcache_types :: * ;
import icache_types :: * ;
import TxRx         :: * ;
`ifdef muldiv
import mbox         :: * ;
`endif
`ifdef rtldump
import csrbox       :: * ;
`endif

`include "Logger.bsv"


// --------------------- stage0 interfaces -------------------------------------------------------
interface Ifc_s0_common;
  /*doc:method: this method will be enabled to indicate a flush from the execute stage*/
  method Action ma_update_eEpoch ();

  /*doc:method: this method will be enabled to indicate a flush from the write-back stage*/
  method Action ma_update_wEpoch ();

  /*doc:method: This method is fired either from a mis-prediction from the execute stage or a trap
  from the write-back stage or due to an sfence or fence being committed. */
  method Action ma_flush (Stage0Flush fl);

  /*doc:method: Method indicates that the reset sequence is done*/
  method Action ma_reset_done(Bool _done);
endinterface:Ifc_s0_common

`ifdef bpu
interface Ifc_s0_bpu;
  /*doc : method : method to training the BTB and BHT tables*/
  method Action ma_train_bpu (Training_data td);
`ifdef gshare
  /*doc : method: This method is fired when there is a conditional misprediction */
  method Action ma_mispredict (Tuple2#(Bool, Bit#(`histlen)) g);
`endif
  /*doc : method: This method captures if the bpu is enabled through csr or not*/
  method Action ma_bpu_enable (Bool e);
endinterface:Ifc_s0_bpu
`endif

interface Ifc_s0_icache;
  /*doc:subifc: This interface defines the request sent out from stage0 to the cache and stage1.*/
  interface Get#(IMem_core_request#(`vaddr, `iesize)) to_icache;
endinterface:Ifc_s0_icache

interface Ifc_s0_tx;
  /*doc:subifc: interface to send info to stage 1 about the next pc*/
  interface TXe#(Stage0PC#(`vaddr)) tx_to_stage1;
endinterface:Ifc_s0_tx
// -----------------------------------------------------------------------------------------------

// --------------------- stage1 interfaces -------------------------------------------------------
interface Ifc_s1_rx;
 // this interface receives the prediction response from the branch prediction unit
  interface RXe#(Stage0PC#(`vaddr)) rx_from_stage0;
endinterface:Ifc_s1_rx

interface Ifc_s1_tx;
  // instruction along with other results to be sent to the next stage
  interface TXe#(PIPE1) tx_to_stage2;
  `ifdef etrace_support
  interface TXe#(Bit#(1)) tx_ingress_opcode;   //v
   `endif
`ifdef rtldump
  interface TXe#(CommitLogPacket) tx_commitlog;
`endif
endinterface: Ifc_s1_tx

interface Ifc_s1_common;
  // method to update epochs on redirection from execute stage
	method Action ma_update_eEpoch;

    // method to update epochs on redirection from writeback stage
	method Action ma_update_wEpoch;

  // csrs from the csrfile.
  method Action ma_csr_misa_c (Bit#(1) c);

`ifdef triggers
  method Action trigger_data1(Vector#(`trigger_num, TriggerData) t);
  method Action trigger_data2(Vector#(`trigger_num, Bit#(`xlen)) t);
  method Action trigger_enable(Vector#(`trigger_num, Bool) t);
`endif
endinterface:Ifc_s1_common

interface Ifc_s1_icache;
  // instruction response from the memory subsytem or the memory bus
  interface Put#(IMem_core_response#(32, `iesize)) inst_response;
endinterface:Ifc_s1_icache
// -----------------------------------------------------------------------------------------------


// --------------------- stage3 interfaces -------------------------------------------------------
interface Ifc_s3_rx;
  /*doc:subifc: interface to receive the meta information from the decode stage*/
  interface RXe#(Stage3Meta)          rx_meta_from_stage2;
  /*doc: subifc: interface to receive the mtval value from stage2 incase of a trap */
  interface RXe#(Bit#(`xlen))          rx_mtval_from_stage2;
  /*doc: subifc: interface to receive the mtval value from stage2 incase of a trap */
  interface RXe#(Instruction_type)    rx_instrtype_from_stage2;
  /*doc: subifc: interface to receive the operand metadata value from stage2*/
  interface RXe#(OpMeta)              rx_opmeta_from_stage2;
  `ifdef etrace_support
  interface RXe#(Bit#(8))             rx_ingress_opcode;   //v
   `endif
`ifdef rtldump 
  // interface to receive the instruction sequence for the rtl dump feature
  interface RXe#(CommitLogPacket)     rx_commitlog;
`endif
endinterface:Ifc_s3_rx

interface Ifc_s3_tx;
  /*doc:subifc: Interface to send out the results for base and control ops*/
  interface TXe#(BaseOut)             tx_baseout_to_stage4;
  /*doc:subifc: interface to send out the results caused due to a trap*/
  interface TXe#(TrapOut)             tx_trapout_to_stage4;
  /*doc:subifc: interface to bypass the system operations */
  interface TXe#(SystemOut)           tx_systemout_to_stage4;
  /*doc:subifc: interface to send out the results of valid memory operations*/
  interface TXe#(MemoryOut)           tx_memoryout_to_stage4;
  /*doc:subifc: interface to send common meta information to the memory stage.*/
	interface TXe#(FUid)        tx_fuid_to_stage4;
	`ifdef etrace_support
	
	interface TXe#(Bit#(14))     tx_ingress_opcode;  //v
	 `endif
`ifdef rtldump
  // interface to send the instruction sequence for the rtl dump feature
  interface TXe#(CommitLogPacket)     tx_commitlog;
`endif
endinterface: Ifc_s3_tx

interface Ifc_s3_rf;
  /*doc:method: receive op1 and its meta info from previous stage (stage2/decode)*/
`ifndef core_clkgate
  (*always_enabled, always_ready*)
`endif
  method Action ma_op1 (FwdType i);
  /*doc:method: receive op2 and its meta info from previous stage (stage2/decode)*/
`ifndef core_clkgate
  (*always_enabled, always_ready*)
`endif
  method Action ma_op2 (FwdType i);
  /*doc:method: receive op2 and its meta info from previous stage (stage2/decode)*/
`ifndef core_clkgate
  (*always_enabled, always_ready*)
`endif
  method Action ma_op3 (FwdType i);
endinterface: Ifc_s3_rf

interface Ifc_s3_cache;
  // interface to send memory requests to the dmem subsystem
  interface Get#(DMem_request#(`vaddr, `elen, 1)) mv_memory_request;
`ifndef core_clkgate
  (*always_enabled*)
`endif
  method Action ma_cache_is_available(Bool avail);
endinterface:Ifc_s3_cache

interface Ifc_s3_bypass;
  // methods to receive the operands from the proceeding stages. A max of three instructions can
  // exist in the pipe that will require to be looked up for bypass.
`ifndef core_clkgate
  (*always_enabled, always_ready*)
`endif
  method Action ma_bypass (Vector#(`bypass_sources, FwdType) fwd);
endinterface: Ifc_s3_bypass

`ifdef bpu
interface Ifc_s3_bpu;
  // this method receives the pc of the next instruction present in the pipe. This is driven by
  // the decode stage.
  method Action ma_next_pc (Bit#(`vaddr) npc);
  // This method sends out the training information to the BTB for conditional branches.
  method Training_data mv_train_bpu;
  // This method sends out the return - address to be pushed on top of the stack.
`ifdef gshare
  method Tuple2#(Bool, Bit#(`histlen)) mv_mispredict;
`endif
endinterface:Ifc_s3_bpu
`endif

`ifdef muldiv
interface Ifc_s3_muldiv;
  /*doc:method: this method send out the inputs required to the mbox unit*/
  method MBoxIn mv_mbox_inputs;
  
`ifndef core_clkgate
  (*always_ready, always_enabled*)
`endif
  /*doc:method: This method captures the ready signals from the mbox unit*/
  method Action ma_mbox_ready(MBoxRdy rdy);
endinterface: Ifc_s3_muldiv
`endif

`ifdef spfpu
interface Ifc_s3_float;
  method Input_Packet mv_fbox_inputs;
`ifndef core_clkgate
  (*always_ready, always_enabled*)
`endif
  /*doc:method: This method captures the ready signals from the fbox unit*/
  method Action ma_fbox_ready(Bit#(1) rdy);
    
endinterface: Ifc_s3_float
`endif

`ifdef perfmonitors
interface Ifc_s3_perfmonitors;
  /*doc:method: */
`ifdef spfpu
`ifndef core_clkgate
  (*always_ready*)
`endif
  method Bit#(1) mv_count_floats;
`endif
`ifdef muldiv
`ifndef core_clkgate
  (*always_ready*)
`endif
  method Bit#(1) mv_count_muldiv;
`endif
`ifndef core_clkgate
  (*always_ready*)
`endif
  method Bit#(1) mv_count_jumps;
`ifndef core_clkgate
  (*always_ready*)
`endif
  method Bit#(1) mv_count_branches;
`ifndef core_clkgate
  (*always_ready*)
`endif
  method Bit#(1) mv_count_rawstalls ;
`ifndef core_clkgate
  (*always_ready*)
`endif
  method Bit#(1) mv_count_exestalls ;
endinterface: Ifc_s3_perfmonitors
`endif

interface Ifc_s3_common;
`ifndef core_clkgate
  (*always_ready*)
`endif
  // method to update epochs on redirection from write - back stage
  method Action ma_update_wEpoch;
  // this method will send out the redirection caused by branches / jumps to all previous stages.
`ifndef core_clkgate
  (*always_ready*)
`endif
  method Tuple2#(Bool, Bit#(`vaddr)) mv_flush;
  // method to receive the current status of the misa_c bit
`ifndef core_clkgate
  (*always_enabled, always_ready*)
`endif
  method Action ma_csr_misa_c (Bit#(1) m);
  
  method Action ma_sb_release(CommitData commit);

`ifndef core_clkgate
  (*always_enabled, always_ready*)
`endif
  method Action ma_priv (Bit#(2) priv);
  
`ifndef core_clkgate
  (*always_enabled, always_ready*)
`endif
  method Action ma_mstatus (Bit#(`xlen) mstatus);

`ifdef hypervisor
`ifndef core_clkgate
  (*always_enabled, always_ready*)
`endif
  method Action ma_vs_mode (Bit#(1) vs);

`ifndef core_clkgate
  (*always_enabled, always_ready*)
`endif
  method Action ma_hstatus (Bit#(`xlen) hstatus);
`endif

`ifdef arith_trap
  method  Action ma_arith_trap_en(Bit#(1) en);
`endif
endinterface: Ifc_s3_common
// -----------------------------------------------------------------------------------------------

// --------------------- stage2 interfaces -------------------------------------------------------
interface Ifc_s2_rx;
  /*doc:subifc: recieve instruction and pc packet from stage1*/
	interface RXe#(PIPE1) rx_from_stage1;
	`ifdef etrace_support
	interface RXe#(Bit#(1)) rx_ingress_opcode;   //v
	 `endif
`ifdef rtldump
  /*doc:subifc: receive instruction of trace from previous stage */
  interface RXe#(CommitLogPacket) rx_commitlog;
`endif
endinterface

interface Ifc_s2_tx;
  /*doc:subifc: send instruction meta data to stage3 */
  interface TXe#(Stage3Meta)    tx_meta_to_stage3;

  /*doc:subifc: send bad-address information to stage3 in case of TRAPs*/
  interface TXe#(Bit#(`xlen))    tx_mtval_to_stage3;

  /*doc:subifc: send instruction type to stage3*/
  interface TXe#(Instruction_type) tx_instrtype_to_stage3;

  interface TXe#(OpMeta) tx_opmeta_to_stage3;
  `ifdef etrace_support
 
  interface TXe#(Bit#(8)) tx_ingress_opcode;     //v
  `endif
`ifdef rtldump
  /*doc:subifc: send instruction trace to next stage */
  interface TXe#(CommitLogPacket) tx_commitlog;
`endif
endinterface: Ifc_s2_tx

interface Ifc_s2_rf;
`ifndef core_clkgate
  (*always_ready*)
`endif
  /*doc:method: Latest value of operand1 from rf*/
  method FwdType mv_op1;

`ifndef core_clkgate
  (*always_ready*)
`endif
  /*doc:method: Latest value of operand2 from rf*/
  method FwdType mv_op2;

`ifndef core_clkgate
  (*always_ready*)
`endif
  /*doc:method: Latest value of operand3 from rf*/
  method FwdType mv_op3;

endinterface:Ifc_s2_rf

interface Ifc_s2_common;
  /*doc:method: input from commit stage (stage5) to update the regfile on instruction retirement*/
  method Action ma_commit_rd (CommitData commit);

`ifndef core_clkgate
  (*always_ready*)
`endif
  /*doc:method: method to update epochs on redirection from execute stage*/
	method Action ma_update_eEpoch;

`ifndef core_clkgate
  (*always_ready*)
`endif
  /*doc:method method to update epochs on redirection from write - back stage*/
	method Action ma_update_wEpoch;

`ifndef core_clkgate
  (*always_ready*)
`endif
  /*doc:method: input from the csr file containing all the required csrs to capture exceptions.*/
  method Action ma_csrs (CSRtoDecode csr);

  /*doc:method: this method indicates that a flush has been generated from the exe / wb stage and
  thus the current stage can quit the stall that was initiated due to an exception generation */
  method Action ma_clear_stall (Bool upd);

`ifndef core_clkgate
	(*always_ready*)
`endif
	/*doc:method: method to indicate if the hart whould resume from a WFI*/
	method Action ma_resume_wfi (Bool w);

endinterface:Ifc_s2_common

`ifdef debug
interface Ifc_s2_debug;
`ifndef core_clkgate
  (*always_enabled, always_ready*)
`endif
  /*doc:method debug related info checking interrupts */
  method Action debug_status (DebugStatus status);
endinterface:Ifc_s2_debug
`endif
// -----------------------------------------------------------------------------------------------
// -------------------------------- stage4 interfaces --------------------------------------------

  interface Ifc_s4_rx;
    /*doc:subifc: Interface to send out the results for base and control ops*/
    interface RXe#(BaseOut)             rx_baseout_from_stage3;
    /*doc:subifc: interface from send out the results caused due from a trap*/
    interface RXe#(TrapOut)             rx_trapout_from_stage3;
    /*doc:subifc: interface from bypass the system operations */
    interface RXe#(SystemOut)           rx_systemout_from_stage3;
    /*doc:subifc: interface from send out the results of valid memory operations*/
    interface RXe#(MemoryOut)           rx_memoryout_from_stage3;
    /*doc:subifc: interface from send common meta information from the memory stage.*/
  	interface RXe#(FUid)        rx_fuid_from_stage3;
  	
  	`ifdef etrace_support
  	interface RXe#(Bit#(14))     rx_ingress_opcode;
  	 `endif
  `ifdef rtldump
    // interface to send the instruction sequence for the rtl dump feature
    interface RXe#(CommitLogPacket)     rx_commitlog;
  `endif
  endinterface:Ifc_s4_rx

  interface Ifc_s4_tx;
    interface TXe#(SystemOut)       tx_systemout_to_stage5;
    interface TXe#(TrapOut)         tx_trapout_to_stage5;
    interface TXe#(BaseOut)         tx_baseout_to_stage5;
    interface TXe#(WBMemop)         tx_memio_to_stage5;
    interface TXe#(CUid)            tx_fuid_to_stage5;
    `ifdef etrace_support
    
     interface TXe#(Bit#(14))       tx_ingress_opcode;
    
     `endif
  `ifdef rtldump
    interface TXe#(CommitLogPacket) tx_commitlog;
  `endif
  endinterface:Ifc_s4_tx

  interface Ifc_s4_cache;
    // interface to receive the response from dmem memory sub system
    interface Put#(DMem_core_response#(`elen,1)) memory_response;
  endinterface:Ifc_s4_cache

`ifdef muldiv
  interface Ifc_s4_muldiv;
    interface RXe#(Bit#(`xlen)) rx_mbox_output;
    `ifdef arith_trap
      interface RXe#(Tuple2#(Bool, Bit#(`causesize))) rx_mbox_arith_trap_output;
    `endif
  endinterface:Ifc_s4_muldiv
`endif

`ifdef spfpu
  interface Ifc_s4_float;
    interface RXe#(XBoxOutput) rx_fbox_output;
  endinterface:Ifc_s4_float
`endif
// -----------------------------------------------------------------------------------------------
// ------------------------------------stage5 interfaces -----------------------------------------
interface Ifc_s5_rx;
  interface RXe#(SystemOut)       rx_systemout_from_stage4;
  interface RXe#(TrapOut)         rx_trapout_from_stage4;
  interface RXe#(BaseOut)         rx_baseout_from_stage4;
  interface RXe#(WBMemop)         rx_memio_from_stage4;
  interface RXe#(CUid)            rx_fuid_from_stage4;
  `ifdef etrace_support
  
   interface RXe#( Bit#(14)) rx_ingress_opcode;
   `endif
  
`ifdef rtldump
  interface RXe#(CommitLogPacket) rx_commitlog;
`endif
endinterface:Ifc_s5_rx


 `ifdef etrace_support
interface Ifc_s5_etrace; 
    interface Bit#(3)  itype     ;
    interface Bit#(4) cause     ;
    interface Bit#(64) tval      ;
    interface Bit#(2) priv      ;
    interface Bit#(64) iaddr     ;
    interface Bit#(1) iretire   ;
    interface Bit#(1) ilastsize ;
 endinterface
`endif

interface Ifc_s5_interrupts;
  method Action ma_clint_msip (Bit#(1) intrpt);
  method Action ma_clint_mtip (Bit#(1) intrpt);
  method Action ma_clint_mtime (Bit#(64) mtime);
  method Action ma_plic_meip (Bit#(1) intrpt);
`ifdef supervisor
  method Action ma_plic_seip (Bit#(1) intrpt);
`endif
`ifdef usertraps
  method Action ma_plic_ueip (Bit#(1) intrpt);
`endif
`ifdef hypervisor
	method Action ma_plic_vseip(Bit#(1) ex_i);
`endif
endinterface:Ifc_s5_interrupts

`ifdef debug
interface Ifc_s5_debug;
  method Bit#(`xlen) mv_csr_dcsr;
  method Action ma_debug_interrupt (Bit#(1) _int);
  method Bit#(1) mv_debug_mode;
  method Bit#(1) mv_core_debugenable;
  method Bit#(1) mv_stop_timer;
  method Bit#(1) mv_stop_count;
endinterface:Ifc_s5_debug
`endif

interface Ifc_s5_common;
  method CommitData mv_commit_rd;
  method WBFlush mv_flush;
`ifdef rtldump
  method Maybe#(CommitLogPacket) mv_commit_log;
`endif
endinterface:Ifc_s5_common

interface Ifc_s5_cache;
  method Tuple2#(Bit#(1), Bit#(TLog#(`dsbsize))) mv_initiate_store;
  method Bit#(1) mv_initiate_ioop;
  method Action ma_io_response(Maybe#(DMem_core_response#(TMul#(`dwords,8),`desize)) r);
endinterface:Ifc_s5_cache

interface Ifc_s5_csrs;
  method Bit#(1) mv_csr_misa_c;
  method Bit#(6) mv_cacheenable;
  method Bit#(2) mv_curr_priv;
  method Bit#(`xlen) mv_csr_mstatus;
  method CSRtoDecode mv_csrs_to_decode;
  method Bool mv_resume_wfi;
`ifdef hypervisor
	`ifdef RV32
	method Bit#(`xlen) mv_csr_mstatush;
	`endif
	method Bit#(`xlen) mv_csr_hstatus;
	method Bit#(`xlen) mv_csr_vsstatus;				
	method Bit#(`xlen) mv_csr_vsatp;
	method Bit#(`xlen) mv_csr_hgatp;
	method Bit#(1) mv_vs_bit;
`endif
`ifdef supervisor
	method Bit#(`xlen) mv_csr_satp;
`endif
`ifdef pmp
  method Vector#(`pmpentries, Bit#(8)) mv_pmp_cfg;
  method Vector#(`pmpentries, Bit#(`paddr)) mv_pmp_addr;
`endif
`ifdef rtldump
  interface Sbread sbread;
`endif
endinterface:Ifc_s5_csrs

`ifdef perfmonitors
interface Ifc_s5_perfmonitors;
  method Action ma_events (Bit#(`mhpm_eventcount) e);
 	/*doc:method: */
  method Bit#(1) mv_count_exceptions;
  method Bit#(1) mv_count_interrupts;
  method Bit#(1) mv_count_csrops;
  method Bit#(1) mv_count_microtraps;
endinterface:Ifc_s5_perfmonitors
`endif
// -----------------------------------------------------------------------------------------------

  module mkPipe_s0_s1#(Ifc_s0_tx s0, Ifc_s1_rx s1)(Bool);
    FIFOF#(Stage0PC#(`vaddr)) ff_pipe0 <- mkSizedFIFOF( `isb_s0s1 );

    mkConnection(s0.tx_to_stage1 , ff_pipe0);
    mkConnection(ff_pipe0, s1.rx_from_stage0);
    return ff_pipe0.notEmpty;
  endmodule:mkPipe_s0_s1

  module mkPipe_s1_s2#(Ifc_s1_tx s1, Ifc_s2_rx s2)(Tuple2#(Bool,FIFOF#(PIPE1)));
    FIFOF#(PIPE1) ff_pipe1 <- mkSizedFIFOF( `isb_s1s2 );
    `ifdef etrace_support
    FIFOF#(Bit#(1)) ff_ingress <- mkSizedFIFOF( `isb_s1s2 );
     `endif
  `ifdef rtldump 
    FIFOF#(CommitLogPacket) ff_commitlog <- mkSizedFIFOF( `isb_s1s2 );
  `endif
    Empty s1_pipe1 <- mkConnection(s1.tx_to_stage2, ff_pipe1);
    Empty s2_pipe1 <- mkConnection(ff_pipe1, s2.rx_from_stage1);
  `ifdef rtldump
    mkConnection(s1.tx_commitlog, ff_commitlog);
    mkConnection(ff_commitlog, s2.rx_commitlog);
  `endif
  
  `ifdef etrace_support
    mkConnection(s1.tx_ingress_opcode, ff_ingress); // 
    mkConnection(ff_ingress, s2.rx_ingress_opcode); // 
     `endif
    
  return tuple2(ff_pipe1.notEmpty, ff_pipe1);
  endmodule:mkPipe_s1_s2

  module mkPipe_s2_s3#(Ifc_s2_tx s2, Ifc_s3_rx s3)(Bool);
    FIFOF#(Stage3Meta) ff_meta <- mkLFIFOF();
    FIFOF#(Bit#(`xlen)) ff_mtval <- mkLFIFOF();
    FIFOF#(Instruction_type) ff_insttype <- mkLFIFOF();
    FIFOF#(OpMeta) ff_opmeta <- mkLFIFOF();
    `ifdef etrace_support
    FIFOF#(Bit#(8)) ff_ingress <- mkLFIFOF();
     `endif
  `ifdef rtldump
    FIFOF#(CommitLogPacket) ff_commitlog <- mkLFIFOF();
  `endif
    let _x1 <- mkConnection(s2.tx_meta_to_stage3, ff_meta);
    let _x2 <- mkConnection(ff_meta, s3.rx_meta_from_stage2);

    let _x3 <- mkConnection(s2.tx_mtval_to_stage3, ff_mtval);
    let _x4 <- mkConnection(ff_mtval, s3.rx_mtval_from_stage2);

    let _x5 <- mkConnection(s2.tx_instrtype_to_stage3, ff_insttype);
    let _x6 <- mkConnection(ff_insttype, s3.rx_instrtype_from_stage2);
    let _x7 <- mkConnection(s2.tx_opmeta_to_stage3, ff_opmeta);
    let _x8 <- mkConnection(ff_opmeta, s3.rx_opmeta_from_stage2);
  `ifdef rtldump
    mkConnection(s2.tx_commitlog, ff_commitlog);
    mkConnection(ff_commitlog, s3.rx_commitlog);
  `endif
        
        `ifdef etrace_support
    mkConnection(s2.tx_ingress_opcode, ff_ingress); // 
    mkConnection(ff_ingress, s3.rx_ingress_opcode); // 
           `endif
    return ff_meta.notEmpty;
  endmodule:mkPipe_s2_s3

  instance Connectable#(Ifc_s2_rf, Ifc_s3_rf);
    module mkConnection#(Ifc_s2_rf s2, Ifc_s3_rf s3)(Empty);
      mkConnection (s2.mv_op1, s3.ma_op1);
      mkConnection (s2.mv_op2, s3.ma_op2);
      mkConnection (s2.mv_op3, s3.ma_op3);
    endmodule
  endinstance

  module mkPipe_s3_s4#(Ifc_s3_tx s3, Ifc_s4_rx s4)(Tuple2#(Bool, FwdType));
    FIFOF#(BaseOut)             ff_baseout <- mkSizedFIFOF( `isb_s3s4 );
    FIFOF#(TrapOut)             ff_trapout <- mkSizedFIFOF( `isb_s3s4 );
    FIFOF#(SystemOut)           ff_systemout <- mkSizedFIFOF( `isb_s3s4 );
    FIFOF#(MemoryOut)           ff_memoryout <- mkSizedFIFOF( `isb_s3s4 );
  	FIFOF#(FUid)                ff_fuid <- mkSizedFIFOF( `isb_s3s4 );
  	`ifdef etrace_support
  	FIFOF#(Bit#(14))     ff_ingress <- mkSizedFIFOF( `isb_s3s4 );
  	 `endif
  `ifdef rtldump
    FIFOF#(CommitLogPacket)     ff_commitlog <- mkSizedFIFOF( `isb_s3s4 );
  `endif
    Wire#(FwdType) wr_bypass <- mkDWire(FwdType{valid: False, addr: ?, data: ?, epochs: ?
                                        `ifdef no_wawstalls ,id: ? `endif
                                        `ifdef spfpu , rdtype: ? `endif });
  
    /*doc:rule: */
    rule rl_gen_bypass;
      Bool valid = ff_baseout.notEmpty && 
          !(ff_baseout.first.rd==0  `ifdef spfpu && ff_baseout.first.rdtype == IRF `endif );
      FwdType lv_bypass = FwdType{valid: valid , addr: ff_baseout.first.rd , 
                                  data: ff_baseout.first.rdvalue, 
                                  epochs: ff_baseout.first.epochs
                    `ifdef no_wawstalls ,id: ff_baseout.first.id `endif
                    `ifdef spfpu , rdtype: ff_baseout.first.rdtype `endif };
      wr_bypass <= lv_bypass;
    endrule:rl_gen_bypass
  
    mkConnection(s3.tx_baseout_to_stage4, ff_baseout);
    mkConnection(ff_baseout, s4.rx_baseout_from_stage3);
  
    mkConnection(s3.tx_trapout_to_stage4, ff_trapout);
    mkConnection(ff_trapout, s4.rx_trapout_from_stage3);

    mkConnection(s3.tx_systemout_to_stage4, ff_systemout);
    mkConnection(ff_systemout, s4.rx_systemout_from_stage3);

    mkConnection(s3.tx_memoryout_to_stage4, ff_memoryout);
    mkConnection(ff_memoryout, s4.rx_memoryout_from_stage3);

    mkConnection(s3.tx_fuid_to_stage4, ff_fuid);
    mkConnection(ff_fuid, s4.rx_fuid_from_stage3);
    

  `ifdef rtldump
    mkConnection(s3.tx_commitlog, ff_commitlog);
    mkConnection(ff_commitlog, s4.rx_commitlog);
  `endif
    
    `ifdef etrace_support
    mkConnection(s3.tx_ingress_opcode, ff_ingress); // 
    mkConnection(ff_ingress, s4.rx_ingress_opcode); // 
     `endif
  
    return tuple2(ff_fuid.notEmpty, wr_bypass);
  endmodule:mkPipe_s3_s4

  module mkPipe_s4_s5# (Ifc_s4_tx s4, Ifc_s5_rx s5)(Tuple2#(Bool, FwdType));
    FIFOF#(SystemOut) ff_systemout <- mkSizedFIFOF( `isb_s4s5 );
    FIFOF#(TrapOut) ff_trapout <- mkSizedFIFOF( `isb_s4s5 );
    FIFOF#(BaseOut) ff_baseout <- mkSizedFIFOF( `isb_s4s5 );
    FIFOF#(WBMemop) ff_wbmemop <- mkSizedFIFOF( `isb_s4s5 );
    FIFOF#(CUid) ff_fuid <- mkSizedFIFOF( `isb_s4s5 );
    
    `ifdef etrace_support
     FIFOF#(Bit#(14)) ff_ingress <- mkSizedFIFOF( `isb_s4s5 ); 
      `endif
  `ifdef rtldump
    FIFOF#(CommitLogPacket) ff_commitlog <- mkSizedFIFOF( `isb_s4s5 );
  `endif
    Wire#(FwdType) wr_bypass <- mkDWire(FwdType{valid: False, addr: ?, data: ?, epochs: ?
                                        `ifdef no_wawstalls ,id: ? `endif
                                        `ifdef spfpu , rdtype: ? `endif });
  
    /*doc:rule: */
    rule rl_gen_bypass;
      Bool valid = ff_baseout.notEmpty && 
          !(ff_baseout.first.rd==0  `ifdef spfpu && ff_baseout.first.rdtype == IRF `endif );
      FwdType lv_bypass = FwdType{valid: valid , addr: ff_baseout.first.rd , 
                                  data: ff_baseout.first.rdvalue, 
                                  epochs: ff_baseout.first.epochs
          `ifdef no_wawstalls ,id: ff_baseout.first.id `endif
          `ifdef spfpu , rdtype: ff_baseout.first.rdtype `endif };
      wr_bypass <= lv_bypass;
    endrule:rl_gen_bypass

    mkConnection(s4.tx_systemout_to_stage5, ff_systemout);
    mkConnection(ff_systemout, s5.rx_systemout_from_stage4);

    mkConnection(s4.tx_trapout_to_stage5, ff_trapout);
    mkConnection(ff_trapout, s5.rx_trapout_from_stage4);

    mkConnection(s4.tx_baseout_to_stage5, ff_baseout);
    mkConnection(ff_baseout, s5.rx_baseout_from_stage4);

    mkConnection(s4.tx_memio_to_stage5, ff_wbmemop);
    mkConnection(ff_wbmemop, s5.rx_memio_from_stage4);

    mkConnection(s4.tx_fuid_to_stage5, ff_fuid);
    mkConnection(ff_fuid, s5.rx_fuid_from_stage4);

  `ifdef rtldump
    mkConnection(s4.tx_commitlog, ff_commitlog);
    mkConnection(ff_commitlog, s5.rx_commitlog);
  `endif
  
  `ifdef etrace_support
  
    mkConnection(s4.tx_ingress_opcode, ff_ingress); // 
    mkConnection(ff_ingress, s5.rx_ingress_opcode); // 
    
    `endif
    return tuple2(ff_fuid.notEmpty, wr_bypass);
  endmodule:mkPipe_s4_s5

  
endpackage: pipe_ifcs

