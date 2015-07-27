/*
** -----------------------------------------------------------------------------**
** cmd_sequencer.v
**
** command sequencer  with FIFO
**
** Copyright (C) 2008 Elphel, Inc
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
// This command sequencer is designed (together with i2c sequencer) to provide
// pipelined operation of the sensor, FPGA pre-processor and compressor, to avoid
// requirement of resetting the circuitry and loosing several frames when the sensor
// acquisition parameters are changed (especially geometry - WOI, decimation).
// It also reduces real-time requirements to the software, as it is possible to
// program parameters related to the events several frames in the future.
//
// Commands related to the particular frames go to one of the 8 FIFO buffers (64 commands deep each),
// each command is 32 bits wide, with 8MSBs being register address, and the remaining 24 bits - data
// to be written.
// That limits data that can be written to the FPGA registers compared to the direct register writes
// (8MSBs are always zero), but all the relevant information can be written with just 24 data bits.
//
// 
// Controller is programmed through 16 locations (currently mapped to 0x60..0x6f):
// 0x0..0x7 write directly to the frame number [2:0] modulo 8, except if you write to the frame
//          "just missed" - in that case data will go to the current frame.
// 0x8 - write seq commands to be sent ASAP
// 0x9 - write seq commands to be sent after the next frame starts 
//
// 0xe - write seq commands to be sent after the next 6 frames start
// 0xf - control register:
//     [14] -   reset all FIFO (takes 16 clock pulses), also - stops seq until run command
//     [13:12] - 3 - run seq, 2 - stop seq , 1,0 - no change to run state

module cmd_sequencer    (sclk,     // @negedge
                         wen,      // sync to address and d[0:15]
                         wa,       // [3:0] 0..7 - absolute data, 8..0x0e relative data, 0x0f - command
                         di,       // [15:0] data in
                         sync,     // frame sync (used to synchronize data), vacts_sclk @negedge sclk
//                         condition,// condition to wait
                         seq_rq,             // request from the sequencer
                         seq_ack,            // sequencer acknowledge
                         seq_a,              // address from the sequencer
                         seq_d,               // data from the sequencer
                         frame_no);// [2:0] current frame modulo 8
    input          sclk;
    input          wen;
    input   [ 3:0] wa;
    input   [15:0] di;
	 input          sync;
//    output  [ 7:0] condition;
    output         seq_rq;             // request from the sequencer
    input          seq_ack;            // sequencer acknowledge
    output  [ 7:0] seq_a;              // address from the sequencer
    output  [23:0] seq_d;              // data from the sequencer
	 output   [2:0] frame_no;

//    reg    [4:0]  wen_d; // [0] - not just fifo, but any PIO writes, [1] and next - filtered for FIFO only
    reg    [3:0]  wen_d; // [0] - not just fifo, but any PIO writes, [1] and next - filtered for FIFO only
    reg    [3:0]  wad; 
	 reg   [15:0]  di_1;
	 reg   [15:0]  di_2;
	 reg   [15:0]  di_3;
	 reg    [2:0]  wpage0;     // FIFO page were ASAP writes go
	 reg    [2:0]  wpage7;     // unused page, currently being cleared
	 reg    [2:0]  page_r;     // FIFO page were current write goes (reading)
	 reg    [2:0]  wpage_w;    // FIFO page were current write goes (reading)
	 reg           wpage0_inc; // increment wpage0 (after frame syn or during reset
	 reg           wpage0_inc_d; // next cycle after wpage0_inc
	 reg           reset_cmd;
	 reg           run_cmd;
	 reg           reset_on;   // reset FIFO in progress
	 wire          seq_enrun;     // enable seq
	 reg           we_fifo_wp; // enable writing to fifo write pointer memory
	 reg           req_clr;    // request for clearing fifo_wp
	 wire          is_ctl= (wad[3:0]==4'hf);
//	 wire          is_rel= ((wad[3]==1) && (wad[2:0]!=3'h7));
	 wire          is_abs= (wad[3]==0);
	 wire          pre_wpage0_inc;
    wire   [2:0]  frame_no=wpage0[2:0];
//fifo write pointers (dual port distributed RAM)
    reg	  [5:0]	fifo_wr_pointers [0:7];
	 wire   [5:0]  fifo_wr_pointers_outw=fifo_wr_pointers[wpage_w[2:0]];
	 wire   [5:0]  fifo_wr_pointers_outr=fifo_wr_pointers[page_r[2:0]];

	 reg    [5:0]  fifo_wr_pointers_outw_r;
	 reg    [5:0]  fifo_wr_pointers_outr_r;
// command seq fifo (RAMB16_S9_S18)
	 reg    [9:0]  seq_cmd_wa; // wite address for the current pair of 16-bit data words
	                           // {page[2:0],word[5:0],MSW[0]}
    reg           seq_cmd_we; // write enable to blockRAM

	 reg    [1:0]  page_r_inc; // increment page_r[2:0]; - signal and delayed version
	 reg    [5:0]  rpointer;    // FIFO read pointer for current page
//	 wire          word_number; // BlockRAM output word (0 - low, 1 - high)

    reg           seq_rq;             // request from the sequencer
    reg    [15:0] seq_dh;             // address from the sequencer
    wire   [15:0] seq_dl;             // sequencer data as read from BlockRAM
    reg           seq_re;             // read enable from BlockRAM
    reg           seq_re_last;        // second clock of seq_re
	 wire    [1:0] por;                //power on reset
    wire          initialized;        // command fifo initialized
    assign        seq_a[ 7:0]= seq_dh[15:8];
    assign        seq_d[23:0]= {seq_dh[7:0],seq_dl[15:0]};
    assign        pre_wpage0_inc = (!wen_d[0] && !wen_d[1] && !wpage0_inc) && ((req_clr && initialized)|| reset_on) ;
	 
//    input          seq_ack;            // sequencer acknowledge
// setting control parameters		
//		if (reset_cmd || (run_cmd && !di_2[12])) seq_enrun <= 1'b0;
//		else if (run_cmd && di_2[12]) seq_enrun <= 1'b1;
FD_1 i_seq_enrun (.Q(seq_enrun), .C(sclk),.D( !(reset_cmd || (run_cmd && !di_2[12])) &&
                                                ((run_cmd && di_2[12]) || seq_enrun )));
FD_1 i_por0 (.Q(por[0]), .C(sclk),.D( 1'b1));
FD_1 i_por1 (.Q(por[1]), .C(sclk),.D(por[0]));
FD_1 i_initialized ( .Q(initialized), .C(sclk), .D(por[1] && ( initialized || (reset_on && !(wpage0_inc && ( wpage0[2:0]==3'h7)))))   );

    always @ (negedge sclk) begin
// signals related to writing to seq FIFO
// delayed versions of address, data write strobe
	   if (wen) wad [ 3:0] <= wa[ 3:0];
	   if (wen || wen_d[0]) di_1[15:0] <= di[15:0];
	   di_2[15:0] <= di_1[15:0];
	   di_3[15:0] <= di_2[15:0];
//		wen_d[4:0] <= {wen_d[3:1],wen_d[0] && !is_ctl,wen};
		wen_d[3:0] <= {wen_d[2:1],wen_d[0] && !is_ctl, !reset_on && wen};
// decoded commands		
		reset_cmd <=  (!reset_on & wen_d[0] && is_ctl && di_1[14]) || (por[0] && !por[1]);
		run_cmd   <=  wen_d[0] && is_ctl && di_1[13];
// write pointer memory
      wpage0_inc <= pre_wpage0_inc && &por[1:0];
		wpage0_inc_d <= wpage0_inc   && &por[1:0];
      if (reset_cmd || !por[1]) wpage0[2:0]<=3'h0;
		else if (wpage0_inc)      wpage0[2:0]<=wpage0[2:0]+1;
      if (reset_cmd || !por[1]) wpage7[2:0]<=3'h7;
		else if (wpage0_inc)      wpage7[2:0]<=wpage0[2:0];
		reset_on <= por[1] && (reset_cmd || (reset_on && !(wpage0_inc && ( wpage0[2:0]==3'h7))));
		req_clr  <= sync || (req_clr && !wpage0_inc);

		if      (wen_d[0])   wpage_w[2:0] <= is_abs?((wad[2:0]==wpage7[2:0])? wpage0[2:0] : wad[2:0]):(wpage0[2:0]+wad[2:0]);
		else if (wpage0_inc) wpage_w[2:0] <= wpage7[2:0];
      we_fifo_wp <= wen_d[1] || wpage0_inc;
	   if (wen_d[1])  fifo_wr_pointers_outw_r[5:0] <= fifo_wr_pointers_outw[5:0];

	   if (we_fifo_wp) fifo_wr_pointers[wpage_w[2:0]] <= wpage0_inc_d? 6'h0:(fifo_wr_pointers_outw_r[5:0]+1); 
		
		fifo_wr_pointers_outr_r[5:0] <= fifo_wr_pointers_outr[5:0]; // just register distri
// command seq fifo (RAMB16_S9_S18)
      if (wen_d[1]) seq_cmd_wa[9:1] <= {wpage_w[2:0],fifo_wr_pointers_outw[5:0]};
		seq_cmd_wa[0] <= !wen_d[1]; // 0 for the first in a pair, 1 - for the second
      seq_cmd_we    <=  !reset_cmd && (wen_d[1]  || (seq_cmd_we && !wen_d[3])); //reset_cmd added to keep simulator happy
		
// signals related to reading from seq FIFO
		seq_re_last<=seq_re && !seq_re_last;							
		if (seq_re_last) seq_dh[15:0]<=seq_dl[15:0];
      if      (reset_cmd || page_r_inc[0])  rpointer[5:0] <= 6'h0;
		else if (seq_re_last)                 rpointer[5:0] <= rpointer[5:0] + 1;
		seq_re <=   seq_enrun &&
		            !seq_re_last &&
		            !reset_on &&
						!page_r_inc[1] &&
						!page_r_inc[0] &&
            	 (((rpointer[5:0]!= fifo_wr_pointers_outr_r[5:0]) &&
					   !(seq_rq && !seq_ack)) ||
						 seq_re);
		seq_rq <= seq_enrun && (seq_re_last || (seq_rq && !seq_ack));
      page_r_inc[1:0] <= {page_r_inc[0] && !(seq_re && !seq_re_last),
		                    !(page_r_inc[0] && !(seq_re && !seq_re_last)) &&
                          (rpointer[5:0] == fifo_wr_pointers_outr_r[5:0]) &&
                           (page_r[2:0]!=wpage0[2:0])};

      if      (reset_on)                                   page_r[2:0]<=3'h0;
		else if (page_r_inc[0] && !(seq_re && !seq_re_last)) page_r[2:0]<=page_r[2:0]+1;
		
    end
   RAMB16_S18_S18 i_fifo (
                          .DOA(seq_dl[15:0]),    // Port A 16-bit Data Output
                          .DOPA(),                 // Port A 2-bit Parity Output
                          .ADDRA({page_r[2:0],
								          rpointer[5:0],
											 ~seq_re_last}),       // Port A 10-bit Address Input
                          .CLKA(!sclk),            // Port A Clock
                          .DIA(16'h0),             // Port A 16-bit Data Input
                          .DIPA(2'b0),             // Port A 2-bit parity Input
                          .ENA(seq_re),            // Port A RAM Enable Input
                          .SSRA(1'b0),             // Port A Synchronous Set/Reset Input
                          .WEA(1'b0),              // Port A Write Enable Input

                          .DOB(),                  // Port B 16-bit Data Output
                          .DOPB(),                 // Port B 2-bit Parity Output
                          .ADDRB(seq_cmd_wa[9:0]), // Port B 10-bit Address Input
                          .CLKB(!sclk),            // Port B Clock
                          .DIB(di_3[15:0]),        // Port B 16-bit Data Input
                          .DIPB(2'b0),             // Port-B 2-bit parity Input
                          .ENB(seq_cmd_we),        // PortB RAM Enable Input
                          .SSRB(1'b0),             // Port B Synchronous Set/Reset Input
                          .WEB(1'b1)               // Port B Write Enable Input
                          );
	 
endmodule
