// See LICENSE.iitm for license details
/*

Author : IIT Madras
Details:
1.  This module decodes the instructions fetched from the previous stage and also fetches the
    operands from the registerfile.
2.  If a csr operation is being decoded, then the next instruction is stalled untill the csr
    completes and commits the instruction.

NOTE0 : Handling flushes
  Flushes in this stage are handled by 2 epoch registers : eEpoch and wEpoch. This bits are compared
  to the epochs bits from the fetch unit (status under which they were fetched). If they do not
  match then the instruction is dropped. The reason for having 2 epoch registers is because both:
  the execute and the writeback stage can generate a flush of the pipe,  causing instructions to be
  dropped all over.

NOTE1 : Handling Traps
  By handling trap and flushing fetch to jump to the trap routine in this stage saves cycle. One
  might also consider that PC no longer needs to be sent to the subsequent stages. However,  note that
  the load / store exceptions are only captured in the next staged. Including pagefaults. So you will
  any how need to handle a trap in that stage as well.

  Additionally, if you have 2 stages handling traps,  you will have prioritize on over the other.
  Suppose you take a trap from the decode stage but there exists an instruction in the subsequent
  pipeline buffers which will generate a memory exception. While taking the trap in the decode stage
  you have corrupted the csrs and this will screw up all further exception handling.

NOTE2 : Handling WFI.
  WFI is also handled in this stage. If a wfi instruction is encountered is treated as a NOP and
  simply dropped. Simultaenously a register is set. When the instruction requests to be decoded and
  the register is set,  the instruction will only progress if an interrupt has arrived. This will
  ensure that the interrupt is taken on the next instructions as required by the spec. When this
  interrupt is taken (under stall mode) then the register is reset and normal functionality resumes.

  If there are n - continous "wfi" instructions,  then n - interrupts will have to be serviced to resume
  the core.

  If a wfi instruction is decoded and an interrupt is pending, then the WFI instruction is taken. In
  WFI mode, the interrupt is detected in the next cycle and wfi-wait ends. 

NOTE3 : When an illegal exception is taken you have to store the instruction in the mtval register.
This enables trap handlers to quickly emulate the instruction in software. To do this, in case of an
illegal exception we send the instruction as rs1 from stage2 and then pass it as the rd value after
exection stage.

--------------------------------------------------------------------------------------------------
*/
package stage2;
// -- package imports --//
import FIFOF        :: * ;
import TxRx         :: * ;
import DReg         :: * ;
import Connectable  :: * ;
import GetPut       :: * ;
import ConfigReg    :: * ;
import OInt         :: * ;

// -- project imports --//
import registerfile :: * ;        // for instantiating the registerfile
import decoder      :: * ;        // for the decode functions.
import ccore_types  :: * ;        // for pipe - line types
import pipe_ifcs    :: * ;
`include "ccore_params.defines"   // for core parameters
`include "trap.defines"
`include "Logger.bsv"             // for logging display statements.

interface Ifc_stage2;

  interface Ifc_s2_rx rx;
  interface Ifc_s2_tx tx;
  interface Ifc_s2_rf rf;
  interface Ifc_s2_common  common;
`ifdef debug
  interface Ifc_s2_debug debug;
`endif
  method Bool mv_wfi_detected;
endinterface : Ifc_stage2

function Fmt fstage2(Bit#(`xlen) hartid, FwdType op1, Op1type op1type, FwdType op2, Op2type op2type, 
                        FwdType op3, Instruction_type insttype, Stage3Meta meta, Bit#(`xlen) mtval );
  Fmt result = $format("[%2d]STAGE2 : ",hartid);
  Fmt op1_addr = ?;
  if (op1type == IntegerRF)
    op1_addr = $format(" RS1=") + op1_addr + $format("X[%2d][%h]",op1.addr,op1.data);
  else
`ifdef spfpu
  if(op1type == FloatingRF)
    op1_addr = $format(" RS1=") + op1_addr + $format("F[%2d][%h]",op1.addr,op1.data);
  else
`endif
    op1_addr = $format(" RS1=") + op1_addr + $format("PC[%h]",meta.pc);

  Fmt op2_addr = ?; 
  if (op2type == IntegerRF)
    op2_addr = $format(" RS2=") + op2_addr + $format("X[%2d][%h]",op2.addr,op2.data);
  else
`ifdef spfpu
  if (op2type == FloatingRF)
    op2_addr = $format(" RS2=") + op2_addr + $format("F[%2d][%h]",op2.addr,op2.data);
  else
`endif
`ifdef compressed
  if (op2type == Constant2)
    op2_addr = $format(" RS2=") + op2_addr + $format("Immediate['h2]");
  else
`endif
  if (op2type == Immediate)
    op2_addr = $format(" RS2=") + op2_addr + $format("Immediate[%h]",op2.addr,op2.data);
  else
    op2_addr = $format(" RS2=") + op2_addr + $format(" Immediate['h4]");

  Fmt op_rd = ?; 
`ifdef spfpu 
  if(meta.rdtype == FRF)
    op_rd = $format(" RD=") + op_rd + $format("F[%2d]",meta.rd);
  else
`endif
    op_rd = $format(" RD=") + op_rd + $format("X[%2d]",meta.rd);

  Fmt op3_addr = $format(" Immediate[%h]",op3.data);
  Fmt offset = $format(" Offset[%h]",op3.data);
  Fmt csr_addr = $format(" CSRADDR[%h]",op3.data[11:0]);
  case (insttype)
    ALU: result = result + $format("ALU -") + op1_addr + op2_addr + op_rd;
    BRANCH: result = result + $format("Branch -") + op1_addr + op2_addr + offset;
    JAL: result = result + $format("JAL -") + offset + op_rd;
    JALR: result = result + $format("JAL -") + op1_addr + offset + op_rd;
    TRAP: result = result + $format("TRAP - Cause:%3d Microtrap:%b Mtval:%h",meta.funct, meta.is_microtrap,mtval);
    WFI: result = result + $format("WFI");
    SYSTEM_INSTR: result = result + $format("SYSTEM -") + op1_addr +csr_addr +op_rd;
    MEMORY: begin
      result = result + $format("MEMORY - ");
      if (meta.memaccess == Load)
        result = result + $format("LOAD") + op1_addr + op_rd + offset;
      else if (meta.memaccess == Store)
        result = result + $format("STORE") + op1_addr + op2_addr + offset;
    `ifdef atomic
      else if (meta.memaccess == Atomic)
        result = result + $format("Atomic") + op1_addr + op2_addr + op_rd;
    `endif
      else
        result = result + fshow(meta.memaccess);
    end
  `ifdef muldiv
    MULDIV: result = result + $format("MULDIV -") + op1_addr + op2_addr + op_rd;
  `endif
  `ifdef spfpu
    FLOAT: result = result + $format("FLOAT -") + op1_addr + op2_addr + op3_addr + op_rd;
  `endif
  endcase
  return result;
endfunction:fstage2

`ifdef stage2_noinline
`ifdef core_clkgate
(*synthesize,gate_all_clocks*)
`else
(*synthesize*)
`endif
`endif
module mkstage2#(parameter Bit#(`xlen) hartid) (Ifc_stage2);

  String stage2=""; // defined for logger

  // --------------------- Start instantiations ------------------------//

  /*doc:mod: instantiation of the registerfile module */
  Ifc_registerfile registerfile <- mkregisterfile(hartid);

  /*doc:mod FIFO to interface with stage0 and receive fetched instruction */
	RX#(PIPE1) rx_pipe1 <- mkRX;

  /*doc:mod FIFO interface to send the decoded information to the next stage.*/
  TX#(Stage3Meta)   tx_meta   <- mkTX;

  /*doc:mod FIFO interface to send the bad-address information to the next stage.*/
  TX#(Bit#(`xlen))   tx_mtval   <- mkTX;

  /*doc:mod FIFO interface to send the bad-address information to the next stage.*/
  TX#(Instruction_type)   tx_instrtype <- mkTX;

  /*doc:mod FIFO interface to send the operands meta data to the next stage.*/
  
  TX#(OpMeta)   tx_opmeta <- mkTX;
  
`ifdef etrace_support
  TX#(Bit#(8))   tx_ingress_opcode <- mkTX;
  RX#(Bit#(1))   rx_ingress_opcode <- mkRX;
 `endif 
  
`ifdef rtldump
  // fifo interface used to transmit the trace of the instruction for rtl.dump generation
  TX#(CommitLogPacket) tx_commitlog <- mkTX;
  RX#(CommitLogPacket) rx_commitlog <- mkRX;
`endif

  /*doc:wire: wire to capture the latest csr values from csr-file*/
  Wire#(CSRtoDecode) wr_csrs <- mkWire();

  /*doc:reg: this register maintains the epoch value modified by the execute stage*/
	Reg#(Bit#(1)) eEpoch <- mkConfigReg(0);

  /*doc:reg: this register maintains the epoch value modified by the write-back stage*/
	Reg#(Bit#(1)) wEpoch <- mkConfigReg(0);

  /*doc:reg:
    this register is used to stall the current stage from processing any new instructions until a
    redirection from execute / write - back is received. The stall is generated when an trap is
    detected in this stage for the current instruction being processed. This prevents flooding
    the pipe with un - necessary instructions since a redirection is expected.*/
  Reg#(Bool) rg_stall <- mkReg(False);

  /*doc:reg:
    this register when True indicates the current stage is waiting for interrupts before
    sending any new info to the next stage*/
  Reg#(Bool) rg_wfi   <- mkReg(False);

  /*doc:wire: This wire indicates if any locally enabled interrupt is pending irrespective of the
  * global status of interrupt-enable or delegation. It simply carries mie&mip*/
  Wire#(Bool) wr_resume_wfi <- mkDWire(False);

  /*doc:reg:
    This register when set to true indicates that the current instruction being processed will
    have to be re - fetched and executed since the previous instruction was a CSR operation.*/
  Reg#(Bool) rg_microtrap <- mkReg(False);

  /*doc:reg: This register stores the micro-trap cause values*/
  Reg#(Bit#(`causesize)) rg_microtrap_cause <- mkReg(0);

  /*doc:wire:
    the following wires are used to ensure that rg_microtrap, rg_wfi and rg_stall are not set in the cycle a
    redirection from the exe / wb stage is received.*/
  Wire#(Bool) wr_flush_from_exe <- mkDWire(False);
  Wire#(Bool) wr_flush_from_wb  <- mkDWire(False);

`ifdef debug
  /*doc:wire: This wire will capture info about the current debug state of the core*/
  Wire#(DebugStatus) wr_debug_info <- mkWire();

  // This register indicates when an instruction passed the decode stage after a resume request is
  // received while is step is set.
  Reg#(Bool) rg_step_done <- mkReg(False);
`endif

  /*doc:reg:
    This register holds the latest value of operand1 from the RF. This will get updated
    every time a retirement to the same register occurs.*/
  Reg#(FwdType) rg_op1[2] <- mkCReg(2, unpack(0));

  /*doc:reg:
    This register holds the latest value of operand2 from the RF. This will get updated
    every time a retirement to the same register occurs.*/
  Reg#(FwdType) rg_op2[2] <- mkCReg(2, unpack(0));

  Reg#(Op2type) rg_op2type[2] <- mkCReg(2, IntegerRF);
  /*doc:reg:
    This register holds the latest value of operand3 from the RF. This will get updated
    every time a retirement to the same register occurs.*/
  Reg#(FwdType) rg_op3[2] <- mkCReg(2, unpack(0));

  // ---------------------- End Instatiations --------------------------//

  // ---------------------- Start Rules -------------------------------//

  // RuleName : decode_and_opfetch
  // Explicit Conditions : rg_stall == False
  // Implicit Conditions : rx_pipe1.notEmpty and all tx fifos are not full
  // Description : This rule decodes the current fetched instruction, fetches the operands from the
  // registerfile and sends the required struct to the next stage.
  rule decode_and_opfetch(!rg_stall && rx_pipe1.u.notEmpty && tx_instrtype.u.notFull && !rg_wfi);
   
    // --- extract the fields from the packet received from the stage1 ---- //
    let pc = rx_pipe1.u.first.program_counter;
    let inst = rx_pipe1.u.first.instruction; //bring to stage 3 for etrace
    let epochs = rx_pipe1.u.first.epochs;
    let trap = rx_pipe1.u.first.trap;
    let trapcause = rx_pipe1.u.first.cause;
  `ifdef compressed
    let highbyte_err = rx_pipe1.u.first.upper_err;
    let compressed = rx_pipe1.u.first.compressed ;
  `endif
  `ifdef bpu
    let btbresponse = rx_pipe1.u.first.btbresponse;
  `endif
    // ---------------------------------------------------------------------------------------- //

    `logLevel( stage2, 0, $format("[%2d]STAGE2: csrs:",hartid,fshow(wr_csrs)))

    // ----------------------------- perform decode ------------------------ //
    let decoded <- decoder_func(inst, trap, 
                                trapcause, wr_csrs,
                                rg_microtrap, rg_microtrap_cause
                                `ifdef compressed ,compressed `endif
                                `ifdef debug ,wr_debug_info, rg_step_done `endif );
    let imm = decoded.meta.immediate;
    let func_cause = decoded.meta.funct_cause;
    let instrType = decoded.meta.inst_type;
    let word32 = decode_word32(inst,wr_csrs.csr_misa[2]);
  `ifdef spfpu
    RFType rf1type = `ifdef spfpu decoded.op_type.rs1type == FloatingRF ? FRF : `endif IRF;
    RFType rf2type = `ifdef spfpu decoded.op_type.rs2type == FloatingRF ? FRF : `endif IRF;
  `endif
    `logLevel( stage2, 0, $format("[%2d]STAGE2 : PC:%h Instruction:%h",hartid,pc, inst))
    // ---------------------------------------------------------------------------------------- //
    // ---------------------- generate bad-address value in case of traps --------------------- //
    Bit#(`xlen) mtval = 0;
    if(func_cause == `Illegal_inst )
      mtval = zeroExtend(inst); // for mtval
    else if(func_cause == `Breakpoint )
      mtval = zeroExtend(pc); // for mtval
`ifdef supervisor
  `ifdef compressed
    else if(func_cause == `Inst_pagefault && highbyte_err)
      mtval = zeroExtend(pc) + 2;
  `endif
    else if(func_cause == `Inst_pagefault)
      mtval = zeroExtend(pc);
`endif
    // ---------------------------------------------------------------------------------------- //
    // ---------------------------- read operands from the registerfile ----------------------//
    let rs1_from_rf <- registerfile.read_rs1(decoded.op_addr.rs1addr
                        `ifdef spfpu ,rf1type `endif );
    let rs2_from_rf <- registerfile.read_rs2(decoded.op_addr.rs2addr
                        `ifdef spfpu ,rf2type `endif );
  `ifdef spfpu
    let rs3 <- registerfile.read_rs3(inst[31:27]);
  `endif
    // -------------------------------------------------------------------------------------- //
    
    // ------------------------ modify operand values before enquing to next stage -----------//
    Bit#(`elen) op1 =  rs1_from_rf;
    Bit#(`elen) op2 =  (decoded.op_type.rs2type == Constant2) ? 'd2: // constant2 only is C enabled.
                      (decoded.op_type.rs2type == Constant4) ? 'd4:
                      (decoded.op_type.rs2type == Immediate) ? signExtend(imm) : rs2_from_rf;
  `ifdef spfpu
    Bit#(`flen) op4 = (decoded.op_type.rs3type == FRF) ? rs3 : signExtend(imm);
  `else
    Bit#(`flen) op4 = signExtend(imm);
  `endif
    // -------------------------------------------------------------------------------------- //
    let stage3meta = Stage3Meta{funct : func_cause, memaccess : decoded.meta.memaccess,
                                pc : pc, epochs : epochs, rd: decoded.op_addr.rd,
                                is_microtrap: rg_microtrap
                  `ifdef hypervisor ,hlvx : decoded.meta.hlvx
                                    ,hvm_loadstore : decoded.meta.hvm_loadstore
                  `endif
                  `ifdef spfpu ,rdtype      : decoded.op_type.rdtype `endif
                  `ifdef RV64  , word32     :     word32
                  `elsif dpfpu , word32     :     word32 `endif
                            `ifdef bpu                , btbresponse:  btbresponse, lp_hist : rx_pipe1.u.first.lp_hist
                                `ifdef compressed     , compressed : compressed `endif
                            `endif };


    if({eEpoch, wEpoch} != epochs)begin
      `logLevel( stage2, 0, $format("[%2d]STAGE2 : Dropping Instruction due to epoch mis - match",hartid))
      rx_pipe1.u.deq;
    `ifdef rtldump
      rx_commitlog.u.deq;
    `endif
    
    `ifdef etrace_support    
      rx_ingress_opcode.u.deq;
       `endif
    end
    else begin
      if (instrType == WFI) begin
        if(!wr_flush_from_exe && !wr_flush_from_wb) begin
          `logLevel( stage2, 0, $format("[%2d]STAGE2 : Encountered WFI",hartid))
          rg_wfi <= True;
        end
        else
          `logLevel( stage2, 0, $format("[%2d]STAGE2 : Dropping WFI",hartid))
        instrType = ALU;
      end
      // The following logic is used to ensure correct step functionality. When the core is halted
      // or free-running rg_step_done is set to false. When the step bit in dcsr is set and resume
      // request is received, the very next instruction (matching epochs) will set rg_step_done to
      // True. The core needs to halt after this instruction commit. Thus, the 2nd instruction in
      // the stream (which matches the epochs which could have changed if the first instruction is a
      // branch/jump) is tagged as a Trap with HaltStep cause code, thus causing the core to go back
      // to the halted stage. When the core is again halted then, rg_step_done is reset to False.
      `ifdef debug
        `logLevel( stage2, 0, $format("[%2d]STAGE2: step_done:%b rerun:%b",hartid,rg_step_done,rg_microtrap))
        if(rg_step_done && wr_debug_info.debug_mode)
          rg_step_done<=False;
        else if(!rg_microtrap)
          rg_step_done <= !wr_debug_info.debug_mode && wr_debug_info.step_set
                                                      && wr_debug_info.debugger_available;
      `endif
  
        rg_microtrap <= decoded.meta.microtrap && !wr_flush_from_exe && !wr_flush_from_wb;
        rg_microtrap_cause <= (decoded.meta.inst_type== SYSTEM_INSTR )? `CSR_rerun :
`ifdef hypervisor (decoded.meta.memaccess == HFence_GVMA || decoded.meta.memaccess== HFence_VVMA)? `Hfence_rerun: `endif
                             (decoded.meta.memaccess == FenceI)?`FenceI_rerun : `Sfence_rerun ;
  
        // -------------------------- Enque relevant data to the next stage -------------------- //
        Bit#(3) _op1_id = ?;
        Bit#(3) _op2_id = ?;
        Bit#(3) _op1_wid = ?;
        Bit#(3) _op2_wid = ?;
        if(instrType == TRAP)
          rg_stall <= True && !wr_flush_from_exe && !wr_flush_from_wb;
        let opmeta = OpMeta { rs1addr: decoded.op_addr.rs1addr,
                              rs2addr: decoded.op_addr.rs2addr,
                              rs1type: decoded.op_type.rs1type,
                              rs2type: decoded.op_type.rs2type
                            `ifdef spfpu
                              ,rs3type: decoded.op_type.rs3type
                              ,rs3addr: decoded.op_addr.rs3addr
                            `endif
                            };
        tx_meta.u.enq(stage3meta);
        tx_mtval.u.enq(mtval);
        tx_instrtype.u.enq(instrType);
        tx_opmeta.u.enq(opmeta);
      `ifdef rtldump
        let clogpkt = rx_commitlog.u.first;
        clogpkt.inst_type = tagged REG (CommitLogReg{wdata:?, rd: stage3meta.rd, 
                            irf: `ifdef spfpu stage3meta.rdtype==IRF `else True `endif });
        if (instrType == SYSTEM_INSTR) begin
          clogpkt.inst_type = tagged CSR (CommitLogCSR{csr_address : truncate(imm),
              rd: stage3meta.rd, rdata:?, wdata:?, op:truncate(func_cause)} );
        end
        else if (instrType == MEMORY) begin
          clogpkt.inst_type = tagged MEM (CommitLogMem{access: stage3meta.memaccess, 
                  rd: stage3meta.rd, 
                  size: truncate(func_cause), address: ?, data: ?, commit_data:?,
                  irf: `ifdef spfpu stage3meta.rdtype==IRF `else True `endif });
        end
        tx_commitlog.u.enq(clogpkt);
      `endif
      `ifdef etrace_support
       let lv_compressed = rx_ingress_opcode.u.first;
       tx_ingress_opcode.u.enq({lv_compressed,inst[6:0]});
        `endif
       
        let _op1 = FwdType{ valid: True, addr: decoded.op_addr.rs1addr, data: op1, epochs: wEpoch
                          `ifdef no_wawstalls ,id: ? `endif
                          `ifdef spfpu ,rdtype: (decoded.op_type.rs1type==FloatingRF)?FRF:IRF `endif
                          };
        let _op2 = FwdType{ valid: True, addr: decoded.op_addr.rs2addr, data: op2, epochs: wEpoch
                          `ifdef no_wawstalls ,id: ? `endif
                          `ifdef spfpu ,rdtype: (decoded.op_type.rs2type==FloatingRF)?FRF:IRF `endif
                          };
        let _op3 = FwdType{ valid: True, 
                            addr: `ifdef spfpu decoded.op_addr.rs3addr `else 0 `endif ,
                            data: signExtend(op4),
                            epochs: wEpoch
                          `ifdef spfpu ,rdtype: decoded.op_type.rs3type `endif 
                          `ifdef no_wawstalls ,id : ? `endif };
                            
        rg_op1[0] <= _op1;
        rg_op2[0] <= _op2;
        rg_op2type[0] <= decoded.op_type.rs2type;
        rg_op3[0] <= _op3;

        `logLevel( stage2, 0, fstage2( hartid, _op1, decoded.op_type.rs1type, 
                  _op2, decoded.op_type.rs2type, _op3, instrType, stage3meta, mtval ))
        // -------------------------------------------------------------------------------------- //
      rx_pipe1.u.deq;
    `ifdef rtldump
      rx_commitlog.u.deq;
    `endif
    `ifdef etrace_support
    rx_ingress_opcode.u.deq;
     `endif
    end
  endrule

  /*doc:rule: */
  rule rl_wait_for_interrupt(rg_wfi);
    if(wr_resume_wfi || wr_flush_from_wb || wr_flush_from_exe)
      rg_wfi <= False;
  endrule


  // interface to send decoded structs to the next stage.
  interface tx = interface Ifc_s2_tx
    interface tx_meta_to_stage3   = tx_meta.e;
    interface tx_mtval_to_stage3  = tx_mtval.e;
    interface tx_instrtype_to_stage3 = tx_instrtype.e;
    interface tx_opmeta_to_stage3= tx_opmeta.e;
    `ifdef etrace_support
    interface tx_ingress_opcode= tx_ingress_opcode.e;
     `endif
  `ifdef rtldump
    interface tx_commitlog= tx_commitlog.e;
  `endif
  endinterface;

  interface rx = interface Ifc_s2_rx
  `ifdef rtldump
    interface rx_commitlog = rx_commitlog.e;
  `endif
	  interface rx_from_stage1 = rx_pipe1.e;
	  `ifdef etrace_support
	  interface rx_ingress_opcode =  rx_ingress_opcode.e;
	   `endif
	endinterface;

  interface common = interface Ifc_s2_common
    method Action ma_csrs (CSRtoDecode csr);
      wr_csrs <= csr;
    endmethod
  
    /*doc:method: This method is use to perform the commit to the registerfile. This method is also
    * used to update the operands presented by the registerfile to the subsequent stage. This update
    * could occur either in the same cycle as the operands are being read from the RF or at a later
    * stage. This in some sense mimics a bypass registerfile implementation. 
    * One might expect that the stage3 shall capture all commits that the current instruction depends
    * on. However, stage3 can be stalled and may be prevented from firing for multiple reasons. Under
    * those scenarios its necessary that the operands from this stage are updated. This allows us to
    * maintain a single register state for each operand source.
    */
  	method Action ma_commit_rd (CommitData commit);
      `logLevel( stage2, 0, $format("[%2d]STAGE2: ",hartid,fshow(commit)))
      if (!commit.unlock_only) begin
        registerfile.commit_rd(commit);
    
      `ifdef spfpu
        if(commit.addr == rg_op1[1].addr && commit.rdtype == rg_op1[1].rdtype)begin
          let _x = rg_op1[1];
          if(commit.rdtype == FRF || rg_op1[1].addr!=0)
            _x.data=commit.data;
          rg_op1[1] <= _x;
        end
    
        if(commit.addr == rg_op2[1].addr && commit.rdtype == rg_op2[1].rdtype)begin
          let _x = rg_op2[1];
          if(commit.rdtype == FRF || (rg_op2[1].addr != 0 && rg_op2type[1] == IntegerRF))
            _x.data=commit.data;
          rg_op2[1] <= _x;
        end
    
        if(rg_op3[1].addr == commit.addr && rg_op3[1].rdtype == FRF &&  commit.rdtype == FRF)
          rg_op3[1].data <= commit.data;
    
      `else
        let _x = rg_op1[1];
        let _y = rg_op2[1];
        if(rg_op1[1].addr == commit.addr && rg_op1[1].addr!=0) begin
          _x.data = commit.data;
          rg_op1[1] <= _x;
        end
    
        if(rg_op2[1].addr == commit.addr && rg_op2[1].addr!=0 && rg_op2type[1] == IntegerRF)
          _y.data = commit.data;
          rg_op2[1] <= _y;
      `endif
    end
    endmethod

    // This method will get activated when there is a flush from the execute stage
  	method Action ma_update_eEpoch;
      wr_flush_from_exe <= True;
  		eEpoch<=~eEpoch;
  	endmethod
  
    // This method gets activated when there is a flush from the write - back stage.
  	method Action ma_update_wEpoch;
      wr_flush_from_wb <= True;
  		wEpoch<=~wEpoch;
  	endmethod
  
    method Action ma_clear_stall (Bool upd) if(rg_stall);
      if(upd) begin
        rg_stall <= False;
        rg_microtrap <= False;
      end
    endmethod
  
    /*doc:method: */
    method Action ma_resume_wfi (Bool w);
      wr_resume_wfi <= w;
    endmethod
  endinterface;

  interface rf = interface Ifc_s2_rf
    method mv_op1 = rg_op1[0];
    method mv_op2 = rg_op2[0];
    method mv_op3 = rg_op3[0];
  endinterface;
  method mv_wfi_detected = rg_wfi;
	

`ifdef debug
  interface debug = interface Ifc_s2_debug
    method Action debug_status (DebugStatus status);
      wr_debug_info <= status;
    endmethod
  endinterface;
`endif
endmodule
endpackage
