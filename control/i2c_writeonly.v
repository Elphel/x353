/*
** -----------------------------------------------------------------------------**
** i2c_writenly.v
**
** i2c (master, write only) controller with FIFO
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
// This i2c controller is designed to reduce CPU load while programming sensor over the
// i2c interface. Only write registers operation is implemented,
// registers are read out using direct pin contol by the software - it is needed only
// currently sensor register readout is only needed for debug purposes (that is not true
// for non-sensor boards connected to the sensor port)
//
// Each register is written in a separate i2c access, fetched by the controller from the
// 32-bit wide FIFO, Each long word is sent over i2c starting from the MSB ( slave address+op),
// then 0..3 of the next bytes (3bytes for 16-bit registers like in Micron sensors).
// Bytes are sent starting from the MSB, so to send 8-bit data 0x12 to register 0x34 of slave 0x56
// length should be set to 3 and data word should be 0x56341200
//
// i2C FIFO is split in 8 (only 7 are used) pages, corresponding to frames (separated by start of 
// frame sync pulses), each pages being able to hold up to 64 commands (register writes)
//
// If frame sync happens before all the commands for current frame were sent out, they stiil will be sent
// before controller proceeds to the next page ones.
//
// If sensor ir running with very small frames/high FPS and controller tries to send too many commands - it
// is possible the FIFO will be overrun. It is up to the software to avoid such (practically very unlikely)
// conditions
// 
// Controller is programmed through 16 locations (currently mapped to 0x50..0x5f):
// 0x0..0x7 write directly to the frame number [2:0] modulo 8, except if you write to the frame
//          "just missed" - in that case data will go to the current frame.
// 0x8 - write i2c commands to be sent ASAP
// 0x9 - write i2c commands to be sent after the next frame starts 
//
// 0xe - write i2c commands to be sent after the next 6 frames start
// 0xf - control register:
//     [14] -   reset all FIFO (takes 16 clock pulses), also - stops i2c until run command
//     [13:12] - 3 - run i2c, 2 - stop i2c (needed before software i2c), 1,0 - no change to run state
//     [11] -   if 1, use [10:9] to set command bytes to send after slave address (0..3)
//     [10:9] - number of bytes to send, valid if [11] is set
//     [8]    - set duration of quarter i2c cycle in system clock cycles - nominal value 100 (0x64)
//     [7:0]  - duration of quater i2c cycle (applied if [8] is set)

module i2c_writeonly     (sclk,    // @negedge
                         wen,     // sync to address and d[0:15]
                         wa,      // [3:0] 0..7 - absolute data, 8..0x0e relative data, 0x0f - command
                         di,      // [15:0] data in
								 sync,    // frame sync (used to synchronize data), vacts_sclk @negedge sclk
                         busy,    // busy (do not use software i2i)
                         scl,     // i2c SCL
                         sda,     // i2c SDA
                         scl_en,  // switch i2c control to i2c_writeonly (from software direct bit control PIO)
                         sda_en,  // enable SDA output
								 frame_no);   // [2:0] current frame modulo 8
    input         sclk;
    input         wen;
    input  [ 3:0] wa;
    input  [15:0] di;
	 input         sync;
    output        busy;
    output        scl;
    output        sda;
    output        scl_en;
    output        sda_en;
	 output  [2:0] frame_no;

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
	 reg           dly_cmd;
	 reg           bytes_cmd;
	 reg           run_cmd;
	 reg           reset_on;   // reset FIFO in progress
	 reg    [1:0]  i2c_bytes;
	 reg    [7:0]  i2c_dly;
	 reg           i2c_enrun;     // enable i2c
	 reg           we_fifo_wp; // enable writing to fifo write pointer memory
	 reg           req_clr;    // request for clearing fifo_wp
	 wire          is_ctl= (wad[3:0]==4'hf);
//	 wire          is_rel= ((wad[3]==1) && (wad[2:0]!=3'h7));
	 wire          is_abs= (wad[3]==0);
	 wire          pre_wpage0_inc = (!wen_d[0] && !wen_d[1] && !wpage0_inc) && (req_clr || reset_on) ;
    wire   [2:0]  frame_no=wpage0[2:0];
//fifo write pointers (dual port distributed RAM)
    reg	  [5:0]	fifo_wr_pointers [0:7];
	 wire   [5:0]  fifo_wr_pointers_outw=fifo_wr_pointers[wpage_w[2:0]];
	 wire   [5:0]  fifo_wr_pointers_outr=fifo_wr_pointers[page_r[2:0]];

	 reg    [5:0]  fifo_wr_pointers_outw_r;
	 reg    [5:0]  fifo_wr_pointers_outr_r;
// command i2c fifo (RAMB16_S9_S18)
	 reg    [9:0]  i2c_cmd_wa; // wite address for the current pair of 16-bit data words
	                           // {page[2:0],word[5:0],MSW[0]}
    reg           i2c_cmd_we; // write enable to blockRAM

	 reg    [1:0]  page_r_inc; // increment page_r[2:0]; - signal and delayed version
	 reg    [5:0]  rpointer;    // FIFO read pointer for current page

    reg           i2c_start; // initiate i2c register write sequence
	 reg           i2c_run;   // i2c sequence is in progress
	 reg           i2c_done;  // i2c sequence is over
	 reg    [1:0]  bytes_left; // bytes left to send after this one
	 reg    [1:0]  byte_number;  // byte number to send next (3-2-1-0)
	 reg    [1:0]  byte_sending; // byte number currently sending (3-2-1-0)
	 reg    [5:0]  i2c_state;  // 0x2b..0x28 - sending start, 0x27..0x24 - stop, 0x23..0x4 - data, 0x03..0x00 - ACKN
	 reg    [7:0]  dly_cntr;   // bit delay down counter


	 reg           scl_hard;
	 reg           sda_hard;
	 reg           sda_en_hard;
	 reg           wen_i2c_soft; // write software-contrlolles SDA, SCL state
	 reg           scl_en_soft;  // software i2c control signals (used when i2c controller is disabled)
	 reg           scl_soft;
	 reg           sda_en_soft;
	 reg           sda_soft;
	 wire          scl=i2c_run?    scl_hard:    scl_soft ;
	 wire          sda=i2c_run?    sda_hard:    sda_soft ;
	 wire          scl_en=i2c_run? 1'b1:        scl_en_soft  ;
	 wire          sda_en=i2c_run? sda_en_hard: sda_en_soft ;

    wire   [7:0]  i2c_data;
	 reg    [8:0]  i2c_sr;
	 reg           i2c_dly_pre_over; 
	 wire          i2c_dly_pre2_over;
	 reg           i2c_dly_over;
	 wire          i2c_startseq_last=(i2c_state[5:0]==6'h28);
	 wire          i2c_stopseq_last= (i2c_state[5:0]==6'h24);
	 wire          i2c_dataseq_last= (i2c_state[5:0]==6'h00);
	 wire          i2c_bit_last    = (i2c_state[1:0]==2'h0);
	 wire          i2c_is_ackn     = (i2c_state[5:2]==4'h0);
	 wire          i2c_is_start    = i2c_state[5] && i2c_state[3];
	 wire          i2c_is_stop     = i2c_state[5] && i2c_state[2];
	 wire          i2c_is_data     = !i2c_state[5] || (!i2c_state[3] && !i2c_state[2]); // including ackn

//	 reg           i2c_startseq_done; // last cycle of start sequence
	 reg           i2c_dataseq_done;  // last cycle of each byte sequence
//	 reg           i2c_dataseq_all_done; // last cycle of the last byte sequence
	 reg    [1:0]  i2c_byte_start;
	 reg           i2c_sr_shift;
	 reg           i2c_stop_start;
	 reg           sda_0;
	 reg           scl_0;
	 reg           busy;
	 reg    [2:0]  busy_cntr;
	 assign  i2c_dly_pre2_over=(dly_cntr[7:0]==8'h2);
    always @ (negedge sclk) begin
// signals related to writing to i2c FIFO
// delayed versions of address, data write strobe
	   if (wen) wad [ 3:0] <= wa[ 3:0];
	   if (wen || wen_d[0]) di_1[15:0] <= di[15:0];
	   di_2[15:0] <= di_1[15:0];
	   di_3[15:0] <= di_2[15:0];
//		wen_d[4:0] <= {wen_d[3:1],wen_d[0] && !is_ctl,wen};
		wen_d[3:0] <= {wen_d[2:1],wen_d[0] && !is_ctl,wen};
// software i2c signals		
		wen_i2c_soft <= wen_d[0] && is_ctl;
		if      (i2c_run)                   scl_en_soft <= 1'b0;
		else if (wen_i2c_soft & |di_1[1:0]) scl_en_soft <= (di_1[1:0]!=2'h3); 
		if      (i2c_run)                   scl_soft <= 1'b0;
		else if (wen_i2c_soft & |di_1[1:0]) scl_soft <= (di_1[1:0]==2'h2); 
		if      (i2c_run)                   sda_en_soft <= 1'b0;
		else if (wen_i2c_soft & |di_1[3:2]) sda_en_soft <= (di_1[3:2]!=2'h3); 
		if      (i2c_run)                   sda_soft <= 1'b0;
		else if (wen_i2c_soft & |di_1[3:2]) sda_soft <= (di_1[3:2]==2'h2); 
	
// decoded commands		
		reset_cmd <=  wen_d[0] && is_ctl && di_1[14];
		run_cmd   <=  wen_d[0] && is_ctl && di_1[13];
		bytes_cmd <=  wen_d[0] && is_ctl && di_1[11];
		dly_cmd   <=  wen_d[0] && is_ctl && di_1[ 8];
// setting control parameters		
		if (bytes_cmd) i2c_bytes[1:0] <= di_2[10:9];
		if (dly_cmd)   i2c_dly[7:0]   <= di_2[ 7:0];
		if (reset_cmd || (run_cmd && !di_2[12])) i2c_enrun <= 1'b0;
		else if (run_cmd && di_2[12]) i2c_enrun <= 1'b1;
// write pointer memory
      wpage0_inc <= pre_wpage0_inc;
		wpage0_inc_d <= wpage0_inc;
      if (reset_cmd)       wpage0[2:0]<=3'h0;
		else if (wpage0_inc) wpage0[2:0]<=wpage0[2:0]+1;
      if (reset_cmd)       wpage7[2:0]<=3'h7;
		else if (wpage0_inc) wpage7[2:0]<=wpage0[2:0];
		reset_on <= reset_cmd || (reset_on && !(wpage0_inc && ( wpage0[2:0]==3'h7)));
		req_clr  <= sync || (req_clr && !wpage0_inc);

		if      (wen_d[0])   wpage_w[2:0] <= is_abs?((wad[2:0]==wpage7[2:0])? wpage0[2:0] : wad[2:0]):(wpage0[2:0]+wad[2:0]);
		else if (wpage0_inc) wpage_w[2:0] <= wpage7[2:0];
      we_fifo_wp <= wen_d[1] || wpage0_inc;
	   if (wen_d[1])  fifo_wr_pointers_outw_r[5:0] <= fifo_wr_pointers_outw[5:0];

	   if (we_fifo_wp) fifo_wr_pointers[wpage_w[2:0]] <= wpage0_inc_d? 6'h0:(fifo_wr_pointers_outw_r[5:0]+1); 
		
		fifo_wr_pointers_outr_r[5:0] <= fifo_wr_pointers_outr[5:0]; // just register distri
// command i2c fifo (RAMB16_S9_S18)
      if (wen_d[1]) i2c_cmd_wa[9:1] <= {wpage_w[2:0],fifo_wr_pointers_outw[5:0]};
		i2c_cmd_wa[0] <= !wen_d[1]; // 0 for the first in a pair, 1 - for the second
      i2c_cmd_we    <=  !reset_cmd && (wen_d[1]  || (i2c_cmd_we && !wen_d[3])); //reset_cmd added to keep simulator happy
		
// signals related to reading from i2c FIFO
      if      (reset_on)  page_r[2:0]<=3'h0;
		else if (page_r_inc[0]) page_r[2:0]<=page_r[2:0]+1;


      if      (reset_cmd || page_r_inc[0])  rpointer[5:0] <= 6'h0;
		else if (i2c_done) rpointer[5:0] <= rpointer[5:0] + 1;

      i2c_run <= !reset_cmd && (i2c_start || (i2c_run && !i2c_done));
      i2c_start <= i2c_enrun && !i2c_run && !i2c_start && (rpointer[5:0]!= fifo_wr_pointers_outr_r[5:0]) && !page_r_inc[1] && !page_r_inc[0];
      page_r_inc[1:0] <= {page_r_inc[0], !i2c_run &&
		                    !page_r_inc[0] &&
                          (rpointer[5:0] == fifo_wr_pointers_outr_r[5:0]) &&
                           (page_r[2:0]!=wpage0[2:0])};
//i2c sequence generation		
		if      (!i2c_run)         bytes_left[1:0] <= i2c_bytes[1:0];
		else if (i2c_dataseq_done) bytes_left[1:0] <= bytes_left[1:0] -1;

		if (!i2c_run)              byte_sending[1:0] <= 2'h3;
		else if (i2c_dataseq_done) byte_sending[1:0] <= byte_sending[1:0] + 1;

		if (!i2c_run)              byte_number[1:0] <= 2'h3;
		else if (i2c_byte_start[1])byte_number[1:0] <= byte_number[1:0] - 1;

		if (!i2c_run || i2c_dly_over) dly_cntr[7:0] <= i2c_dly[7:0];
		else dly_cntr[7:0] <= dly_cntr[7:0] - 1;
//		i2c_dly_pre_over <= (dly_cntr[7:0]==8'h2); // period = 3..258
		i2c_dly_pre_over <= i2c_dly_pre2_over; // period = 3..258
		i2c_dly_over <=i2c_dly_pre_over;
		
//		i2c_startseq_done    <= i2c_startseq_last && i2c_dly_pre_over;
		i2c_dataseq_done     <= i2c_dataseq_last &&  i2c_dly_pre_over;
//		i2c_dataseq_all_done <= i2c_dataseq_last &&  i2c_dly_pre_over && (bytes_left == 2'h0) ;

		i2c_byte_start[1:0]  <= {i2c_byte_start[0],
                   		(i2c_startseq_last || (i2c_dataseq_last && (bytes_left[1:0] != 2'h0))) && i2c_dly_pre2_over };
		i2c_sr_shift    <=   i2c_bit_last && !(i2c_dataseq_last) && i2c_dly_pre_over;
      i2c_stop_start  <=   i2c_dataseq_last && (bytes_left[1:0] == 2'h0) && i2c_dly_pre_over ;

		i2c_done     <=  i2c_stopseq_last && i2c_dly_pre_over;
		if    (i2c_byte_start[1]) i2c_sr[8:0] <= {i2c_data[7:0], 1'b1};
		else if (i2c_sr_shift)    i2c_sr[8:0] <= {i2c_sr[7:0],   1'b1};
		
		if      (!i2c_run)          i2c_state[5:0] <= 6'h2a; // start of start seq
		else if (i2c_stop_start)    i2c_state[5:0] <= 6'h26; // start of stop  seq
		else if (i2c_byte_start[1]) i2c_state[5:0] <= 6'h23; // start of data  seq
		else if (i2c_dly_over)      i2c_state[5:0] <= i2c_state[5:0] - 1;
// now creating output signals
      scl_0 <= (i2c_is_start && (i2c_state[1:0]!=2'h0)) ||
		         (i2c_is_stop  && !i2c_state[1]) ||
				   (i2c_is_data  && (i2c_state[1] ^i2c_state[0])) ||
					 !i2c_run;
	   sda_0 <= (i2c_is_start &&  i2c_state[1]) ||
		         (i2c_is_stop  && (i2c_state[1:0]==2'h0)) ||
				   (i2c_is_data  && i2c_sr[8]) ||
               !i2c_run;
		sda_hard <= sda_0;
      scl_hard <= scl_0;
	   sda_en_hard <= i2c_run && (!sda_0 || (!i2c_is_ackn && !sda_hard));	

      if (wen) busy_cntr[2:0] <= 3'h7;
		else if (|busy_cntr[2:0]) busy_cntr[2:0] <= busy_cntr[2:0]-1;
		
      busy <= (i2c_enrun && ((rpointer[5:0]!= fifo_wr_pointers_outr_r[5:0]) || (page_r[2:0]!=wpage0[2:0]))) ||
		        |busy_cntr[2:0] ||
				  i2c_run ||
				  reset_on;
    end
	 
   RAMB16_S9_S18 i_fifo (
                          .DOA(i2c_data[7:0]),     // Port A 8-bit Data Output
                          .DOPA(),                 // Port A 1-bit Parity Output
                          .ADDRA({page_r[2:0],
								          rpointer[5:0],
											 byte_number[1:0]}),       // Port A 11-bit Address Input
                          .CLKA(!sclk),            // Port A Clock
                          .DIA(8'h0),              // Port A 8-bit Data Input
                          .DIPA(1'b0),             // Port A 1-bit parity Input
                          .ENA(i2c_byte_start[0]), // Port A RAM Enable Input
                          .SSRA(1'b0),             // Port A Synchronous Set/Reset Input
                          .WEA(1'b0),              // Port A Write Enable Input

                          .DOB(),                  // Port B 16-bit Data Output
                          .DOPB(),                 // Port B 2-bit Parity Output
                          .ADDRB(i2c_cmd_wa[9:0]), // Port B 10-bit Address Input
                          .CLKB(!sclk),            // Port B Clock
                          .DIB(di_3[15:0]),        // Port B 16-bit Data Input
                          .DIPB(2'b0),             // Port-B 2-bit parity Input
                          .ENB(i2c_cmd_we),        // PortB RAM Enable Input
                          .SSRB(1'b0),             // Port B Synchronous Set/Reset Input
                          .WEB(1'b1)               // Port B Write Enable Input
                          );
	 
endmodule
