// See LICENSE.iitm for license details

package TbSoc;
  import Soc:: *;
  import Clocks::*;
  import GetPut:: *;
	import Semi_FIFOF:: *;
	import AXI4_Types:: *;
	import AXI4_Fabric:: *;
  import uart::*;
	import ccore_types::*;
	import csr_types :: *;
	import csrbox_decoder :: * ;
  import csrbox :: * ;
  import Vector :: * ;
  `include "ccore_params.defines"
  `include "Logger.bsv"
  `include "Soc.defines"
  `include "csrbox.defines"
  import device_common::*;
  import DReg :: *;
  import Connectable :: *;
`ifdef debug
  import DebugSoc     :: * ;
`endif

`define limit 'd10000000

`ifdef openocd
  import "BDPI" function ActionValue #(int) init_rbb_jtag(Bit#(1) dummy);
  import "BDPI" function ActionValue #(Bit #(8))get_frame(int client_fd);
  import "BDPI" function Action send_tdo(Bit #(1) tdo , int client_fd);
`endif
    function Bit#(`xlen) fn_atomic_op (Bit#(5) op,  Bit#(`xlen) rs2,  Bit#(`xlen) loaded);
      Bit#(`xlen) op1 = loaded;
      Bit#(`xlen) op2 = rs2;
    `ifdef RV64
      if(op[4]==0)begin
	  		op1=signExtend(loaded[31:0]);
        op2= signExtend(rs2[31:0]);
      end
    `endif
      Int#(`xlen) s_op1 = unpack(op1);
	  	Int#(`xlen) s_op2 = unpack(op2);

      case (op[3:0])
	  			'b0011:return op2;
	  			'b0000:return (op1+op2);
	  			'b0010:return (op1^op2);
	  			'b0110:return (op1&op2);
	  			'b0100:return (op1|op2);
	  			'b1100:return min(op1,op2);
	  			'b1110:return max(op1,op2);
	  			'b1000:return pack(min(s_op1,s_op2));
	  			'b1010:return pack(max(s_op1,s_op2));
	  			default:return op1;
	  		endcase
    endfunction

    `ifdef spfpu
    function Bool fn_fflags_print (Bit#(7) funct7,Bit#(7) opcode, Bool irf);
      if ((!irf ||
          funct7[6:1] == 'b110000 ||  //fcvt.w.s/d instr
          funct7[6:1] == 'b101000) && //fcmp
          !(funct7[6:1] == 'b111000 && opcode == 'b1010011)&&  //fmv.x.w and fmv.x.d
          !(funct7[6:1] == 'b111100 && opcode == 'b1010011) &&  //fmv.w.x and fmv.w.d
          !(funct7[6:1] == 'b001000 && opcode == 'b1010011))    //fsgnj*
        return True;
      else
        return False;
    endfunction
  `endif
  (*synthesize*)
  module mkTbSoc(Empty);

    let def_clk <- exposeCurrentClock;
    let def_rst <- exposeCurrentReset;

    MakeClockIfc#(Bit#(1)) tck_clk <-mkUngatedClock(1);
    MakeResetIfc trst <- mkReset(0,False,tck_clk.new_clk);

  `ifdef debug
    Ifc_DebugSoc soc <- mkDebugSoc(tck_clk.new_clk,trst.new_rst);
  `else
    Ifc_Soc soc <- mkSoc();
  `endif
  
  `ifdef rtldump
    `include "csr_probe.bsv"
  `endif

    UserInterface#(`paddr,`buswidth,16) uart <- mkuart_user(5, 0, 0);
    Reg#(Bool) rg_read_rx<- mkDReg(False);

    Reg#(Bit#(5)) rg_cnt <-mkReg(0);

    Vector#(100, String) string_field;
    for (Integer i = 0; i<100; i = i + 1) begin
      string_field[i] = integerToString(i);
    end

    /*doc:reg: */
    Reg#(Bit#(32)) rg_inst_count <- mkReg(0);

    rule display_eol;
	    let timeval <- $time;
      `logLevel( tb, 0, $format("\n[%10d]", timeval))
    endrule

  `ifdef rtldump
  Reg#(Bit#(`xlen)) rg_prev_mstatus <- mkReg(0);
  Reg#(Bool) rg_prev_mstatus_valid <- mkReg(False);

  Bit#(XLEN) lv_misa_init = 0;
  `ifdef RV64
    lv_misa_init[63:62] = 2'b10;
  `else
    lv_misa_init[31:30] = 2'b01;
  `endif
  `ifdef atomic
    lv_misa_init[0] = 1;
  `endif
  `ifdef compressed
    lv_misa_init[2] = 1;
  `endif
  `ifdef dpfpu
    lv_misa_init[3] = 1;
  `endif
  `ifdef spfpu
    lv_misa_init[5] = 1;
  `endif
  lv_misa_init[8] = 1;  //Base I isa.
  `ifdef muldiv
    lv_misa_init[12] = 1;
  `endif
  `ifdef usertraps
    lv_misa_init[13] = 1;
  `endif
  `ifdef supervisor
    lv_misa_init[18] = 1;
  `endif
  `ifdef user
    lv_misa_init[20] = 1;
  `endif
  Reg#(Bit#(XLEN)) rg_prev_misa <- mkReg(lv_misa_init);
 	  let dump <- mkReg(InvalidFile) ;
    rule open_file_rtldump(rg_cnt<1);
      let generate_dump <- $test$plusargs("rtldump");
      if(generate_dump) begin
        String dumpFile = "rtl.dump" ;
    	  File lfh <- $fopen( dumpFile, "w" ) ;
    	  if ( lfh == InvalidFile )begin
    	    `logLevel( tb, 0, $format("TB: cannot open %s", dumpFile))
    	    $finish(0);
    	  end
    	  dump <= lfh ;
      end
    endrule
  `endif

 	  let dump1 <- mkRegA(InvalidFile) ;
    rule open_file_app(rg_cnt<5);
      String dumpFile1 = "app_log" ;
    	File lfh1 <- $fopen( dumpFile1, "w" ) ;
    	if (lfh1==InvalidFile )begin
    	  `logLevel( tb, 0, $format("TB: cannot open %s", dumpFile1))
    	  $finish(0);
    	end
      dump1 <= lfh1;
    	rg_cnt <= rg_cnt+1 ;
    endrule

    rule connect_uart_out;
      soc.uart_io.sin(uart.io.sout);
    endrule
    rule connect_uart_in;
      uart.io.sin(soc.uart_io.sout);
    endrule

    rule check_if_character_present(!rg_read_rx);
      let {data,err}<- uart.read_req('hc,HWord);
      if (data[2]==1) // character present
        rg_read_rx<=True;
    endrule

    rule write_received_character(rg_cnt>=1 && rg_read_rx);
      let {data,err}<-uart.read_req('h8,Byte);
      $fwrite(dump1,"%c",data);
    endrule

  `ifdef rtldump

    rule write_dump_file(rg_cnt >= 1);

      let generate_dump <- $test$plusargs("rtldump");
      let stime <- $stime;
      if (soc.soc_sb.commitlog matches tagged Valid .idump) begin
    `ifndef openocd `ifndef cocotb_sim
      if(idump.instruction=='h00006f||idump.instruction =='h00a001)
        $finish(0);
      else
    `endif `endif
      if(generate_dump) begin
        if (rg_inst_count % `limit == 0 && rg_inst_count != 0) begin
          File lfh1 <- $fopen( "rtl"+string_field[rg_inst_count/`limit ]+".dump","w");
          dump <= lfh1;
        end
        rg_inst_count <= rg_inst_count + 1;

        if (idump.instruction[1:0] == 'b11)
        	$fwrite(dump, $time, " core   0: ", idump.mode, `ifdef hypervisor " %1d", idump.v, `endif `ifdef RV32 " 0x%8h" `else " 0x%16h" `endif , idump.pc, " (0x%8h", idump.instruction, ")");
        else
          $fwrite(dump, $time , " core   0: ", idump.mode, `ifdef hypervisor " %1d", idump.v, `endif `ifdef RV32 " 0x%8h" `else " 0x%16h" `endif , idump.pc, " (0x%4h", idump.instruction[15:0], ")");

        if (idump.inst_type matches tagged REG .d) begin

        `ifdef spfpu
        if (!(idump.instruction[31:25] =='b0001001 && idump.instruction[14:0] == 'b000000001110011)) begin
          Bit#(`xlen) wdata_fflags = fn_probe_csr(`FFLAGS);
          Bit#(`xlen) flags = zeroExtend(d.fflags);
        // !flags &&
        if( flags!=0 && fn_fflags_print(idump.instruction[31:25],idump.instruction[6:0], d.irf)) begin 
          //Flags not zero. Destination reg is FRF.
          if (valueOf(`flen) == 64) begin
            $fwrite(dump, " ", fn_csr_to_str(`FFLAGS), " 0x%16h", wdata_fflags);
          end
          if (valueOf(`flen) == 32) begin
            $fwrite(dump, " ", fn_csr_to_str(`FFLAGS), " 0x%8h", wdata_fflags);
          end
        end
        end
      `endif
          // let csr_address = `FFLAGS; // mstatus
          // Bit#(`xlen) wdata = fn_probe_csr(`ifdef hypervisor fn_address_virtual(csr_address,idump.v) `else csr_address `endif );
          if (!((idump.instruction[31:25] =='b0001001 || idump.instruction[31:25]== 'b0010001 || 
               idump.instruction[31:25] =='b0110001)&& idump.instruction[14:0] == 'b000000001110011)) begin
            if (d.irf && valueOf(`xlen) == 64 && d.rd != 0)
              $fwrite(dump, " x%0d", d.rd, " 0x%16h", d.wdata);
            if (d.irf && valueOf(`xlen) == 32 && d.rd != 0)
              $fwrite(dump, " x%0d", d.rd, " 0x%8h", d.wdata);
            if (!d.irf && valueOf(`flen) == 64) begin
              // $fwrite(dump, " ", fn_csr_to_str(`ifdef hypervisor fn_address_virtual(csr_address,idump.v) `else csr_address `endif ), " 0x%16h", wdata);
              $fwrite(dump, " f%0d", d.rd, " 0x%16h", d.wdata);
            end
            if (!d.irf && valueOf(`flen) == 32) begin
              // $fwrite(dump, " " , fn_csr_to_str(`ifdef hypervisor fn_address_virtual(csr_address,idump.v) `else csr_address `endif ), " 0x%16h", wdata);
              $fwrite(dump, " f%0d", d.rd, " 0x%8h", d.wdata);
            end
          end
        end

        if (idump.inst_type matches tagged CSR .d) begin
          let csr_address = d.csr_address;
          csr_address = `ifdef hypervisor fn_address_virtual(csr_address, idump.v) `else csr_address `endif ;
        `ifdef supervisor
          `ifdef hypervisor if (idump.v == 0) `endif
            if (csr_address == `SSTATUS || csr_address == `SIE || csr_address == `SIP) begin
              csr_address = csr_address + 'h200;
            end
        `endif
        `ifdef hypervisor
          if (csr_address == `HIE || csr_address == `HIP)
            csr_address = csr_address - 'h300; // convert to MIE/MIP
          else if (csr_address == `VSIE || csr_address == `VSIP)
            csr_address = csr_address + 'h100; // convert to MIE/MIP
          else if (csr_address == `HVIP)
            csr_address = `MIP; // convert to MIP
        `endif
          if (valueOf(`xlen) == 64 && d.rd != 0)
            $fwrite(dump, " x%0d", d.rd, " 0x%16h", d.rdata);
          if (valueOf(`xlen) == 32 && d.rd != 0)
            $fwrite(dump, " x%0d", d.rd, " 0x%8h", d.rdata);
          Bit#(`xlen) wdata = fn_probe_csr(csr_address);
          if (!(d.op==2'b10 && idump.instruction[19:15] == 0)) begin
            //$display("Tb: Dumping instruction: %h", idump.instruction);
           `logLevel( tb, 0, $format("\n %h", idump.instruction))
           `ifdef hypervisor
            if(idump.instruction=='h10200073 && csr_address== 'h300) begin //sret
              Bit#(`xlen) hstatus = fn_probe_csr('h600);
              //$display("Tb: %h", hstatus);
              `logLevel( tb, 0, $format("\n hstatus: %h", hstatus))
              $fwrite(dump, " c1536_hstatus 0x%16h", hstatus);
            end
            `endif
            if ( `ifdef spfpu csr_address != `FCSR && `endif csr_address != `MISA ) begin
            if (valueOf(`xlen) == 64) 
              $fwrite(dump, " ", fn_csr_to_str(csr_address), " 0x%16h", wdata);
            if (valueOf(`xlen) == 32) begin
	     `ifdef RV32
                if (csr_address == `MSTATUS && idump.instruction == 'h30200073 ) begin //mret
		Bit#(`xlen) wdata1 = fn_probe_csr(`MSTATUSH);
		$fwrite(dump, " " , fn_csr_to_str(`MSTATUSH), " 0x%8h", wdata1);
	        end
      	     `endif
              $fwrite(dump, " " , fn_csr_to_str(csr_address), " 0x%8h", wdata);
		end

            if (csr_address == `MSTATUS) begin
              rg_prev_mstatus <= wdata;
              rg_prev_mstatus_valid <= True;
            end

           

            end
            if( csr_address == `MISA) begin
            if (wdata != rg_prev_misa) begin
              if (valueOf(`xlen) == 64) 
                $fwrite(dump, " ", fn_csr_to_str(csr_address), " 0x%16h", wdata);
              if (valueOf(`xlen) == 32)
                $fwrite(dump, " " , fn_csr_to_str(csr_address), " 0x%8h", wdata);
              rg_prev_misa <= wdata;
            end
          end
          end
        `ifdef spfpu
        if (csr_address == `FCSR) begin
          Bit#(`xlen) wdata_fflags = fn_probe_csr(`FFLAGS);
          Bit#(`xlen) wdata_frm = fn_probe_csr(`FRM);
          if (!(d.op==2'b10 && idump.instruction[19:15] == 0)) begin
          if (valueOf(`xlen) == 64) begin
                  $fwrite(dump, " ", fn_csr_to_str(`FFLAGS), " 0x%16h", wdata_fflags);
                  // if(lv_fssr_print)                     
                  //   $fwrite(dump, " x%0d", d.rd, " 0x%16h", d.rdata);
                  $fwrite(dump, " ", fn_csr_to_str(`FRM), " 0x%16h", wdata_frm);
                end
          if (valueOf(`xlen) == 32) begin
            $fwrite(dump, " ", fn_csr_to_str(`FFLAGS), " 0x%8h", wdata_fflags);
            // if(lv_fssr_print)
            //   $fwrite(dump, " x%0d", d.rd, " 0x%8h", d.rdata);
            $fwrite(dump, " ", fn_csr_to_str(`FRM), " 0x%8h", wdata_frm);
          end
          end
        end
          if (csr_address == `FCSR || csr_address == `FRM || csr_address == `FFLAGS)begin
            csr_address = `MSTATUS;
            Bit#(`xlen) wdata = fn_probe_csr(csr_address);
            if ( !rg_prev_mstatus_valid || (wdata!=rg_prev_mstatus)   ) begin 
            if (!(d.op==2'b10 && idump.instruction[19:15] == 0)) begin
              if (valueOf(`xlen) == 64) 
                $fwrite(dump, " " , fn_csr_to_str(csr_address), " 0x%16h", wdata);
              if (valueOf(`xlen) == 32)
                $fwrite(dump, " " , fn_csr_to_str(csr_address), " 0x%8h", wdata);

              rg_prev_mstatus <= wdata;
              rg_prev_mstatus_valid <= True;

            end
            end
          end
        `endif
        end

        if (idump.inst_type matches tagged MEM .d) begin
          let store_data = d.data;
        `ifdef atomic
          if (d.access == Atomic && d.atomic_op[3:0] != 5 && d.atomic_op[3:0] != 7) begin
            store_data = fn_atomic_op(d.atomic_op,d.data, d.commit_data);
          end
        `endif
          if (d.access == Load `ifdef atomic || d.access == Atomic `endif ) begin
            if (d.irf && valueOf(`xlen) == 64 && d.rd != 0)
              $fwrite(dump, " x%0d", d.rd, " 0x%16h", d.commit_data);
            if (d.irf && valueOf(`xlen) == 32 && d.rd != 0)
              $fwrite(dump, " x%0d", d.rd, " 0x%8h", d.commit_data);
            if (!d.irf && valueOf(`flen) == 64 )
              $fwrite(dump, " f%0d", d.rd, " 0x%16h", d.commit_data);
            if (!d.irf && valueOf(`flen) == 32 )
              $fwrite(dump, " f%0d", d.rd, " 0x%8h", d.commit_data);
          end

          if(valueOf(`xlen) ==64 && d.access != Fence && d.access != FenceI)
            $fwrite(dump, " mem 0x%16h", d.address);
          if(valueOf(`xlen) ==32&& d.access != Fence && d.access != FenceI)
            $fwrite(dump, " mem 0x%8h", d.address);

        `ifdef atomic
          if (d.access == Atomic && d.atomic_op[3:0] != 5 && d.atomic_op[3:0] != 7) begin
            if(valueOf(`xlen) ==64)
              $fwrite(dump, " mem 0x%16h", d.address);
            if(valueOf(`xlen) ==32)
              $fwrite(dump, " mem 0x%8h", d.address);
          end
        `endif

          if (d.access == Store  `ifdef atomic || (d.access == Atomic && d.atomic_op[3:0] != 5) `endif ) begin
            if (d.size == 0) begin
                $fwrite(dump, " 0x%2h", store_data[7:0]);
            end
            if (d.size == 1)
              $fwrite(dump, " 0x%4h", store_data[15:0]);
            if (d.size == 2)
              $fwrite(dump, " 0x%8h", store_data[31:0]);
            if (d.size == 3)
              $fwrite(dump, " 0x%16h", store_data);
          end
        end
          $fwrite(dump, "\n");
      end

      end
    endrule
  `endif

  `ifdef debug
    Wire#(Bit#(1)) wr_tdi <-mkWire();
    Wire#(Bit#(1)) wr_tms <-mkWire();
    rule connect_jtag_io;
      soc.wire_tdi(wr_tdi);
      soc.wire_tms(wr_tms);
    endrule
  `endif
  `ifdef openocd
    Wire#(Bit#(1)) wr_tdo <-mkWire();
    Wire#(Bit#(1)) wr_tck <-mkWire();
    Wire#(Bit#(1)) wr_trst <-mkWire();
    rule rl_wr_tdo;
      wr_tdo <= soc.wire_tdo();
    endrule
    Reg#(Bit#(1)) rg_initial <- mkRegA(0);
    Reg#(Bit#(1)) rg_end_sim <- mkRegA(0);
    Reg#(int) rg_client_fd <- mkRegA(32'hffffffff);
    Reg#(Bit#(5)) delayed_actor <- mkReg(0);
    Reg#(Bit#(5)) delayed_actor2 <- mkReg(0);
    Reg#(Bit#(5)) delayed_actor3 <- mkReg(0);
    Reg#(Bit#(5)) delayed_actor4 <- mkReg(0);
    Reg#(Bit#(5)) delayed_actor5 <- mkReg(0);
    rule rl_initial(rg_initial == 0);
      let x <- init_rbb_jtag(0);
      if(x != 32'hffffffff)begin
        rg_initial <= 1'b1;
        rg_client_fd <= x;
      end
    endrule
    rule rl_get_frame((rg_initial == 1'b1));
      let x <- get_frame(rg_client_fd);
      delayed_actor <= truncate(x);
      delayed_actor2 <= delayed_actor;
      delayed_actor3 <= delayed_actor2;
      delayed_actor4 <= delayed_actor3;
      delayed_actor5 <= delayed_actor4;
      tck_clk.setClockValue(delayed_actor2[2]);
      if(delayed_actor2[4] == 1)
        trst.assertReset();
      if(delayed_actor5[3] == 1 )
        send_tdo(wr_tdo,rg_client_fd);
      wr_tdi <= delayed_actor[0];
      wr_tms <= delayed_actor[1];
      if( x[5] == 1)begin
        $display("OpenOcd Exit");
        $finish();
      end
    endrule
  `endif
  endmodule
endpackage: TbSoc
