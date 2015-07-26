/*
** -----------------------------------------------------------------------------**
** sensorpads353.v
**
** I/O pads related circuitry for the sensor board connectorfs_composite
**
** Copyright (C) 2002-2010 Elphel, Inc.
**
** -----------------------------------------------------------------------------**
**  This file is part of X353
**  X353 is free software - hardware description language (HDL) code.
** 
**  This program is free software: you can redistribute it and/or modify
**  it under the terms of the GNU General Public License as published by
**  the Free Software Foundation, either version 3 of the License, or
**  (at your option) any later version.
**
**  This program is distributed in the hope that it will be useful,
**  but WITHOUT ANY WARRANTY; without even the implied warranty of
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**  GNU General Public License for more details.
**
**  You should have received a copy of the GNU General Public License
**  along with this program.  If not, see <http://www.gnu.org/licenses/>.
** -----------------------------------------------------------------------------**
**
*/



module	sensorpads (/// interface to DCM
                     sclk,       // system clock, @negedge
                     cmd,        // [6:0] command for phase adjustment @ negedge (sclk) MSB - reset pclk2x DCM
							wcmd,       // write command@ negedge (slck)
                     dcm_done,   // DCM command done
                     dcm_status, // [7:0] output dcm status (bit 1 - dcm clkin stopped)
                     dcm_locked, //   DCM locked
							clk_sel,    // 0 - use clk, 1 - sensor dclk (bpf pad) for DCM input (if dclkmode - use clk if 0)
							hact_length,// [13:0] WOI width-1 (to overwrite sensor HACT duration)
                     hact_regen, // 0 - use hact from sensor, 1 - regenerate using hact_lengh
                     clk,        //pixel clock, posedge
                     pclk2x,     // output - twice pixel clock
                     vact,       // VACT pad, inout
							hact,       // HACT pad, inout
							bpf,        // BPF  pad, inout
							pxd,        // [11:0] pads {PXD [9:0],CNVCLK, CNVSYNC} inout
							mrst,       // MRST pad, inoput (input in JTAG external FPGA programming mode)
							arst,       // ARST pad, output (output in JTAG external FPGA programming mode)
							aro,        // ARO pad, output
                     dclkmode,   // input 0 - DCLK is clock to sensor, 1 - combined sync from sensor (like 10347)
                     pxd14,      // input 1 - use {vact,hact} as 2 LSB in 14-bit data
                     debug, // 2-bit debug mode input
                     dclk,       // DCLK pad (inout)
                     en_vacts,   // disable processing second vact after the trigger in triggered mode
							vacts,      // output, single cycle
							ihact,      // output, iihact - posedge clk, latency = 2
							sens_clk,   // output - clock from sensor to be used as global clock after mux (directly from BPF)
							ipxd,       // [15:0] output, ipxd - posedge clk, latency = 2
							imrst,      // input (non-verting)
							iarst,      // input (non-verting)
							iaro,       // input (non-verting)
							cnvctl,     // [1:0] input {icnvclk, icnvsync} - used in 1/2/3 10318 to generate 3.3VDC-> 5.0VDC (and then linear 3.3V)
							cnven,      // input - 1 - enable converter outputs (0 - use them at 2LSBs of data)
                     senspgm,    // SENSPGM I/O pin
                     senspgmin,  // state of the SENSPGM I/O pin (read)
                     xpgmen,     // enable programming mode for an external FPGA
                     xfpgaprog,  // PROG_B to be sent to an external FPGA
                     xfpgadone,  // state of the MRST pin ("DONE" pin on an external FPGA)
                     xfpgatck,   // TCK to be sent to an external FPGA
                     xfpgatms,   // TMS to be sent to an external FPGA
                     xfpgatdi,   // TDI to be sent to an external FPGA
                     xfpgatdo    // TDO read from an external FPGA
//                     ,pherr       //[1:0] phase error (sync to posedge pclk) {too_early, too_late}
                     );
   input          clk_sel;    // 0 - use clk, 1 - dclk for DCM input (if dclkmode - use bpf when 1, clk if 0)
   input          sclk;       // system clock, @negedge
   input    [6:0] cmd;        // [5:0] command for phase adjustment @ negedge (slck)
   input          wcmd;       // write command@ negedge (slck)
   output         dcm_done;   // DCM command done
   output  [7:0]  dcm_status; // dcm status (bit 1 - dcm clkin stopped)
   output         dcm_locked; // DCM locked
   input   [13:0] hact_length;// [13:0] WOI width-1 (to overwrite sensor HACT duration)
   input          hact_regen; // 0 - use hact from sensor, 1 - regenerate using hact_lengh
							
	input				clk;
	output         pclk2x;
	input				vact;
	input				hact; //output in fillfactory mode
	inout				bpf;  // output in fillfactory mode
	inout	  [11:0]	pxd; //actually only 2 LSBs are inouts
	inout          mrst;
	output			arst;
	output			aro;
   input          dclkmode;
   input          pxd14;     // use {vact,hact} as 2 LSB in 14-bit data
   input [1:0]    debug; // 2-bit debug mode input
	inout	         dclk;
   input          en_vacts;   // disable processing second vact after the trigger in triggered mode
	output			vacts;	// 1 cycle long
	output			ihact;
   output         sens_clk; // ibpf before FD
	output  [15:0]	ipxd;
	input				imrst;
	input				iarst;
	input				iaro;
	input    [1:0] cnvctl;      // 2-bits for converter control (use the same pins as lower bits in 12-bit sensors)
	input          cnven;       // 0 - use 12-bit sensor data, 1 - use converter output pins

   inout          senspgm;    // SENSPGM I/O pin
   output         senspgmin;  // state of the SENSPGM I/O pin (read)
   input          xpgmen;     // enable programming mode for external FPGA
   input          xfpgaprog;  // PROG_B to be sent to an external FPGA
   output         xfpgadone;  // state of the MRST pin ("DONE" pin on external FPGA)
   input          xfpgatck;   // TCK to be sent to external FPGA
   input          xfpgatms;   // TMS to be sent to external FPGA
   input          xfpgatdi;   // TDI to be sent to external FPGA
   output         xfpgatdo;   // TDO read from external FPGA

   wire     [7:0] dcm_status; // dcm status (bit 1 - dcm clkin stopped)
	wire    [15:0]	ipxd;
   assign         ipxd[1:0]=2'h0;


  wire      [1:0] cnvctl;
  wire            cnven;

  wire				mrst, arst,aro,dclk, idclk;

  wire           dcm_rst_cmd;
  reg     [2:0]  dcm_drst;
  reg            dcm_rst;
	
  reg [1:0] xpgmen_d;
  wire      force_senspgm;

  wire          fifo_clkin;
  wire  [11:0]  pxdi;
//Automatic clock placement failed. Please attempt to analyze  the global clocking required for this design and either lock the clock...
   assign fifo_clkin=(clk_sel && !dclkmode)?sens_clk:clk;
sensor_phase353
     i_sensor_phase (.cclk(!sclk),       // command clock (posedge, invert on input if needed)
                     .wcmd(wcmd),   // write command
                     .cmd(cmd[5:0]),// CPU write data [5:0]
                                    //       0 - nop, just reset status data
                                    //       1 - increase phase shift
                                    //       2 - decrease phase shift
                                    //       3 - reset phase shift to default (preprogrammed in FPGA configuration)
                                    //       c - reset phase90
                                    //       4 - incr pahse90
                                    //       8 - decrease phase90
                                    //       10 - increase hact/vact phase
                                    //       20 - decrease hact/vact phase
                                    //       30 - reset hact/vact phase
                     .HACT(hact),                    //   sensor HACT I/O pin (input), used to reset FIFO
                     .VACT(vact),                    //   sensor VACT I/O pin (input)
//                     .DI({pxdi[11:2],pxd[1:0]}),   //   sensor D[DATA_WIDTH-1:0] i/o pins (input), strobed @posedge gclk_idata and en_idata
                     .DI(pxdi[11:0]),                //   sensor D[DATA_WIDTH-1:0] i/o pins (input), strobed @posedge gclk_idata and en_idata
                     .debug(debug[1:0]),             // 2-bit debug mode input
							.hact_length(hact_length[13:0]),// [13:0] WOI width-1 (to overwrite sensor HACT duration)
                     .hact_regen(hact_regen),        // 0 - use hact from sensor, 1 - regenerate using hact_lengh
                     .mode_12bits(!cnven),           // input, 1 -  enable 12/14 bit mode, 0 - 10 bit mode
                     .mode_14bits(pxd14),            // input, 1 -  enable 14 bit mode, 0 - 12/10 bit mode
                     .mode_alt(dclkmode),            //   enable alternative vact/hact input (sync to data)
                     .sync_alt(idclk),               //   alternative HACT/VACT input pad (10347) (VACT - 1 clock, HACT >1)
                     .iclk(fifo_clkin),              //   DCM input clock (GCLK) - sensor clock out or sensor clock in
                     .sclk(clk),                     //   global FIFO output clock (posedge)
                     .shact(ihact),                  //   hact - sync to sclk
                     .en_svact(en_vacts),            // disable processing second vact after the trigger in triggered mode

                     .svact(vacts),                  //   vact - sync to sclk (single cycle)
                     .sdo(ipxd[15:2]),               //   data output[DATA_WIDTH-1:0], sync to sclk
                     .dcm_done(dcm_done),            //   DCM command done
                     .status(dcm_status[7:0]),       // dcm status (bit 1 - dcm clkin stopped)
                     .locked(dcm_locked));           //   DCM locked




   IOBUF	 i_mrst	(.I(imrst), .IO(mrst), .T(xpgmen), .O(xfpgadone));
   OBUF	 i_arst	(.I(xpgmen? xfpgatms : iarst), .O(arst));
   OBUF	 i_aro	(.I(xpgmen? xfpgatck : iaro),  .O(aro ));
	
   IOBUF	 i_dclk  (.I(clk), .IO(dclk), .T(dclkmode), .O(idclk));
   
   IBUF	 i_bpf	(.I(bpf), .O(sens_clk));

   IOBUF	 i_pxd0  (.IO(pxd[ 0]), .I(xpgmen?xfpgatdi:cnvctl[0]),  .T(~(cnven | xpgmen)), .O(pxdi[0]));
   IOBUF	 i_pxd1  (.IO(pxd[ 1]), .I(                cnvctl[1]),  .T(~ cnven | xpgmen ), .O(pxdi[1]));
   IBUF	 i_pxd2	(.I (pxd[ 2]), .O(pxdi[ 2]));
   IBUF	 i_pxd3	(.I (pxd[ 3]), .O(pxdi[ 3]));
   IBUF	 i_pxd4	(.I (pxd[ 4]), .O(pxdi[ 4]));
   IBUF	 i_pxd5	(.I (pxd[ 5]), .O(pxdi[ 5]));
   IBUF	 i_pxd6	(.I (pxd[ 6]), .O(pxdi[ 6]));
   IBUF	 i_pxd7	(.I (pxd[ 7]), .O(pxdi[ 7]));
   IBUF	 i_pxd8	(.I (pxd[ 8]), .O(pxdi[ 8]));
   IBUF	 i_pxd9	(.I (pxd[ 9]), .O(pxdi[ 9]));
   IBUF	 i_pxd10 (.I (pxd[10]), .O(pxdi[10]));
   IBUF	 i_pxd11 (.I (pxd[11]), .O(pxdi[11]));

   assign xfpgatdo=pxdi[1];
 //pxdi[11:0]
 
   PULLUP i_PU_mrst   (.O(mrst));
   PULLUP i_PU_senspgm(.O(senspgm));
// to reduce noise on pulled-up senspgm it is possible to force it high when xpgmen goes from high to low
   always @ (posedge clk) xpgmen_d[1:0] <= {xpgmen_d[0],xpgmen};
// will latch the state of the senspgm pin after xpgmen is over - that will protect from driving senspgm pin
// high if it is grounded on the sensor board
   FDCE i_force_senspgm (.Q(force_senspgm), .D(senspgmin), .C(clk), .CE(xpgmen_d[1:0]==2'b10), .CLR(xpgmen));
   IOBUF	 i_senspgm	 (.I(xpgmen?(~xfpgaprog):force_senspgm), .IO(senspgm), .T(~(xpgmen || force_senspgm)), .O(senspgmin));


/// multiplicating pclk by 2, need a way to reset DCM - should be done after source of the pclk is changed
//  extending DCM reset command, synchronizing to DCM input clock
   FD_1 i_dcm_rst_cmd(.Q(dcm_rst_cmd), .D((wcmd && cmd[6]) || (dcm_rst_cmd && !dcm_drst[2])), .C(sclk)) ;
   always @ (posedge clk) begin
     dcm_drst[2:0] <= dcm_drst[2]? 3'b0:{dcm_drst[1], dcm_drst[0], dcm_rst_cmd};
     dcm_rst    <= dcm_drst[0]  || dcm_drst[1]   || dcm_drst[2] ;
   end

   wire      pclk2x, pclk2xi;
BUFG      i_pclk2x  (.I(pclk2xi), .O(pclk2x));
DCM #(
     .CLKIN_DIVIDE_BY_2("FALSE"),
     .CLKIN_PERIOD(10.0),
     .CLKOUT_PHASE_SHIFT("FIXED"),
     .CLK_FEEDBACK("2X"),
     .DESKEW_ADJUST("SYSTEM_SYNCHRONOUS"),
     .DFS_FREQUENCY_MODE("LOW"),
     .DUTY_CYCLE_CORRECTION("TRUE")
) i_dcm4(
    .CLKIN    (clk),
    .CLKFB    (pclk2x),
    .RST      (dcm_rst),
    .PSEN     (1'b0),
    .PSINCDEC (1'b0),
    .PSCLK    (1'b0),
    .DSSEN    (1'b0),
    .CLK0     (),
    .CLK90    (),
    .CLK180   (),
    .CLK270   (),
    .CLKDV    (),
    .CLK2X    (pclk2xi),
    .CLK2X180 (),
    .CLKFX    (),
    .CLKFX180 (),
    .STATUS   (),
    .LOCKED   (),
    .PSDONE   ());
    
// If needed - add positive PHASE_SHIFT - then posedge pclk2x will be earlier than posedge pclk by   PHASE_SHIFT/256*period(pclk)  
endmodule
