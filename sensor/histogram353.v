/*
** -----------------------------------------------------------------------------**
** histogram.v
**
** Calculation of image histograms
**
** Copyright (C) 2005-2010 Elphel, Inc.
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
module	histogram  (pclk,             // pixel clock (posedge, only some input signals
                     pclk2x,           // pclk multiplied by 2
						 	sclk,					// global (commands @negedge)
							wen,              // @negedge sclk, 1 ahead of write enable/read enable
					      rnext,            // read histogram, increment address
							wa,               // [2:0] register address:
							                    // old - 00 - {top, left}
													  // old - 01 - {height-1, width-1}
													  // old - 02 - hist. data start address (will also read pointed word to output word

							                    // 00 - left
							                    // 01 - top
													  // 02 - width-1
													  // 03 - height-1
													  // 04 - hist. data start address (will also read pointed word to output word
							hist_do,			    // [31:0] histogram data (actually now just [17:0])
							                   // 256*R, 256*Gr, 256*Gb, 256*B
							wd,					// [31:0] PIO data  to write
							frame_run,        // frame active - @posedge pclk
							line_run_a,         // hact_out @posedge pclk
							di_a,               // [15:0] @posedge pclk
							di_vld_a,            // di[15:0] valid @posedge pclk
							bayer_phase      // [1:0]
						 );

  input			 pclk;
  input			 pclk2x;
  input			 sclk;
  input			 wen;
  input			 rnext;
  input  [ 2:0] wa;
  output [31:0] hist_do;
//  input  [15:0] wd;
  input  [15:0] wd;
  input			 frame_run;
  input			 line_run_a;
  input  [15:0] di_a;
  input			 di_vld_a;
  input  [ 1:0] bayer_phase;
  
  parameter correct_bayer=2'b11; //AF2015: Correct Bayer to have histogram [2'b00] matcsh even row, even column data

  wire [17:0]   hist_do0;
  
// extra layer of registers
  reg			  line_run;
  reg  [15:0] di;
  reg			  pre_di_vld, di_vld;
  
   always @(posedge pclk) begin
    line_run <= line_run_a;
    di[15:0] <= di_a[15:0];
//    di_vld   <= di_vld_a;
   end
   always @ (posedge pclk2x) begin
    pre_di_vld   <= di_vld_a;
    di_vld   <= pre_di_vld; // so di_vld can be duplicated
   end
// In Model 353 switching to 2x multipled pixel clock without it did not work when sclk < 2 * pclk
// pixels always go in pairs (low/high byte of di)
   reg  [15:0]  di2x;
   reg  [ 3:0]  dvld2x; // dvld2x[0] is valid second half di_vld, dvld2x[1] - 1 pclk2x cycle later, dvld2x[2] - 2 cycles
   reg  [ 1:0]  bayer;
   reg  [ 1:0]  bayer_phase_latched;
   reg  [ 9:0]  hist_waddr;       // histogram memory address (write side) - {color,pixel_value}
   reg  [ 9:0]  hist_waddr_hold1; // hist_waddr - first  hold register (latency read-increment-write)
   reg  [ 9:0]  hist_waddr_hold2; // hist_waddr - second hold register (latency read-increment-write)
   reg          same_waddr;       // same pixel value (use previous result instead of memory as it is not written yet)
	reg          pre_same_addr;
   reg  [ 2:0]  frame_run_s;
   reg  [ 2:0]  line_run_s;
   reg          line_start,line_end;
   reg          frame_start;
//   reg          frame_end;

   reg  [ 5:0]  hist_seq; // delay line for the histogram calculation sequencer (each other bit can be active)
   reg  [ 9:0]  hist_init_cntr;
   reg          end_hist_init;
   reg  [17:0]  hist_pre;  // previous value of the histogram - before increment
   reg  [17:0]  hist_post; // new value of the histogram - after increment

   wire         hist_bank; // use FD so it will be reset at power up for simulation
   reg          odd_line;
   reg  [13:1]  pix_cntr;
   reg  [13:1] line_cntr;
  
   reg          line_started, // left margin over
                line_ended;   // right margin
   reg          frame_started;// top margin over
   reg          frame_ended;  // bottom margin
//	reg			 frame_ended_d; 
   reg          init_hist;    // write zeros to all hist elements.
   reg          init_hist_d;
                
   reg          window_on;    // window active
                
// increased dimesions to 8K  
   reg   [13:1] pos_left; // all dimensions will be even (to simplify processing pixel data packed in pairs)
   reg   [13:1] pos_top;
   reg   [13:1] size_width;
   reg   [13:1] size_height;

   reg   [1:0]  we_pos;
   reg   [1:0]  we_size;
   reg          we_addr;
   reg          we_addr_d;
//   reg          rd_hist;
   wire          rd_hist;
   reg   [ 9:0] hist_raddr; // histogram memory address (read side) - {color,pixel_value}
   reg          hist_wea;
   reg          hist_ena;
   reg          bayer_en;   // enable latching bayer
   reg          last_line;
   wire [17:0]  hist_doa; // RAM output
   wire [17:0]  hist_dia= hist_post[17:0]; // RAM input
	
	reg  [15:0]  wdd; // wd[15:0]  delayed by 1 clock
   reg  [13:1]  minus_pos_left;   // TIG
   reg          pos_left_is_zero; // TIG
	reg          line_start_posl_zero;  // start with pos_left_is_zero
	reg          line_start_posl_nzero; // start with !pos_left_is_zero
   assign rd_hist = rnext || we_addr_d;
   assign        hist_do[31:0]={14'h0,hist_do0[17:0]};

/*
//debug
 reg [9:0] test_cntr;
 always @ (posedge pclk2x) if (frame_run_s[1] && !frame_run_s[2]) begin
   test_cntr[9:0] <=  test_cntr[9:0]+1;
 end
 assign        hist_do[31:0]={1'h0,frame_run_s[1],hist_bank,test_cntr[9:0],frame_run,hist_do0[17:0]}; /// Temporary testing !!!
//  assign        hist_do[31:0]={1'h0,hist_bank,hist_raddr[9:0],2'h0,hist_do0[17:0]}; /// Temporary testing !!!
*/


// FDE  i_hist_bank (.C(pclk2x), .CE(frame_end),  .D(~hist_bank), .Q(hist_bank)); 
// switch bank at the start of frame
 FDE  i_hist_bank (.C(pclk2x), .CE(init_hist & ~init_hist_d),  .D(~hist_bank), .Q(hist_bank)); 


 always @ (posedge pclk2x) begin
   frame_run_s[2:0] <= {frame_run_s[1:0],frame_run};
   line_run_s[2:0]  <= {line_run_s[1:0], line_run};
   line_start       <= line_run_s[1]  && !line_run_s[2];
   line_start_posl_zero  <= line_run_s[1]  && !line_run_s[2] && pos_left_is_zero;  
   line_start_posl_nzero <= line_run_s[1]  && !line_run_s[2] && !pos_left_is_zero;  
   line_end         <= line_run_s[2]  && !line_run_s[1];
   frame_start      <= frame_run_s[1] && !frame_run_s[2];
//   frame_end        <= (frame_ended && !frame_ended_d);
	bayer_en         <= frame_start || (bayer_en && !line_start);
	if (bayer_en) bayer_phase_latched[1:0] <=bayer_phase[1:0];
   if      (!frame_run_s[2]) hist_init_cntr[9:0] <= 10'b0;
   else if (init_hist)       hist_init_cntr[9:0] <= hist_init_cntr[9:0] + 1;
   end_hist_init <= (hist_init_cntr[9:1]==9'h1ff);

   init_hist <= frame_run_s[1] && (init_hist?(~end_hist_init):~frame_run_s[2]);
	init_hist_d <= init_hist;
   
   if (!init_hist) hist_init_cntr[9:0] <= 10'h0;
   else            hist_init_cntr[9:0] <=  hist_init_cntr[9:0] + 1;


   dvld2x[3:0] <= {dvld2x[2:0], ~(|dvld2x[2:0]) & di_vld };
// include window_on (don't start if not)   
//   hist_seq[5:0] <= {hist_seq[4:0], dvld2x[1] | dvld2x[3]};
   hist_seq[5:0] <= {hist_seq[4:0], window_on && (dvld2x[1] | dvld2x[3])};
   hist_ena <= hist_seq[0] || hist_seq[5] || init_hist; // read, write, clear
   hist_wea <= hist_seq[5] || init_hist; // write, clear
  
// get 8-bit pixels from 16-bit packed words
   if      (dvld2x[0]) di2x[15:8] <= di[15:8];
   if      (dvld2x[0]) di2x[ 7:0] <= di[ 7:0];
   else if (dvld2x[2]) di2x[ 7:0] <= di2x[15:8];
// calculate bayer phase of the pixel  
//   reg  [ 1:0]  bayer_phase_latched;

//   if      (dvld2x[0])   bayer[0] <= bayer_phase[0];
   if      (dvld2x[0])   bayer[0] <= 1'b0;
   else if (dvld2x[2])   bayer[0] <= ~bayer[0];
//   if      (frame_start) bayer[1] <=  bayer_phase[1];
   if      (frame_start) bayer[1] <=  1'b0;
   else if (line_start)  bayer[1] <= ~bayer[1];
   if (hist_seq[1]) hist_waddr_hold1[9:0] <= hist_waddr[9:0];
   if (hist_seq[3]) hist_waddr_hold2[9:0] <= hist_waddr_hold1[9:0];
// we need to clear all histogram at the begining of a frame (will not work if the window is too small)   
   if      (init_hist)   hist_waddr[9:0] <= hist_init_cntr[9:0]; // to clear histogram memory
   else if (hist_seq[0]) hist_waddr[9:0] <= {bayer[1:0]^bayer_phase_latched[1:0] ^ correct_bayer,di2x[7:0]};
   else if (hist_seq[5]) hist_waddr[9:0] <= {hist_waddr_hold2[9:0]};
   
/*   same_waddr <= hist_seq[0] && // next cycle - read from memory
                 hist_seq[4] && // second next cycle - write modified
                 (di2x[7:0] == hist_waddr_hold2[7:0]); // same address (colors should be the same - same line, over one pixel)
*/					  
   pre_same_addr <= (di2x[7:0] == hist_waddr_hold2[7:0]);
   same_waddr <= hist_seq[1] && // next cycle - read from memory
                 hist_seq[5] && // second next cycle - write modified
                 pre_same_addr;
   if (hist_seq[2]) hist_pre[17:0] <= same_waddr? hist_post[17:0] : hist_doa[17:0]; // bypass memory for the same histogram count/color
// hist_pre[17:0] -> hist_post[17:0]    - dual cycle
// data to write to histogram memory   
   if      (init_hist)                                  hist_post[17:0] <= 18'h0; // to write 0 to all histogram page memory locations
   else if (hist_seq[4] && (hist_pre[17:0]!=18'h3ffff)) hist_post[17:0] <= hist_pre[17:0] + 1;	// saturate

// odd/even scan lines
   if      (frame_start) odd_line <= 1'b1;
   else if (line_end)    odd_line <= ~odd_line;
   
   minus_pos_left[13:1] <= -pos_left[13:1];   // TIG
   pos_left_is_zero     <= (pos_left[13:1]==13'h0); // TIG

// count pixels (in pairs) to determine active window 
//   if      (line_start) pix_cntr[13:1] <= ~pos_left[13:1];
//   else if (dvld2x[1])  pix_cntr[13:1] <=  pix_cntr[13:1]+1; // window_on will switch right after dvld2x[3]
   if      (line_start) pix_cntr[13:1] <= minus_pos_left[13:1]; // @dvld2x[0]
   else if (dvld2x[0])  pix_cntr[13:1] <= pix_cntr[13:1]+1; // (may change to dvld2x[2])window_on will switch right after dvld2x[3]
   
   if      (line_start_posl_nzero || !frame_run_s[2])                           line_started <= 1'h0;
   else if (line_start_posl_zero  || (dvld2x[2] &&((~pix_cntr[13:1])== 13'h0))) line_started <= 1'h1; // Doublecycle
   
   if      (line_start || !frame_run_s[2])                                    line_ended <= 1'h0;
   else if (dvld2x[2] && line_started && (pix_cntr[13:1] == size_width[13:1])) line_ended <= 1'h1;  // Doublecycle
      
   if      (frame_start)            line_cntr[13:1] <= ~pos_top[13:1];
   else if (line_end && !odd_line)  line_cntr[13:1] <= line_cntr[13:1]+1;
   
   if      (!frame_run_s[2])                     frame_started <= 1'h0;
   else if ((~line_cntr[13:1])== 13'h0)          frame_started <= 1'h1;
   last_line <= (line_cntr[13:1] == size_height[13:1]);
   if      (!frame_run_s[2])                                                                    frame_ended <= 1'h0;
   else if ((line_start && frame_started && last_line) || (frame_run_s[2] && ! frame_run_s[1])) frame_ended <= 1'h1;
//	frame_ended_d <= frame_ended;
//   window_on <= (line_start_posl_zero || line_started) && !line_ended && frame_started && !frame_ended;
//AF2015   window_on <= (line_start_posl_zero || (line_started && !line_ended)) && frame_started && !frame_ended;
   window_on <= (line_start_posl_zero || (line_started && !line_ended)) && frame_started && !frame_ended && !(line_start && last_line) ;
   
 end

 always @ (negedge sclk) begin
   wdd[15:0] <= wd[15:0];

   we_pos[1:0]  <= {wen && (wa[2:0]==3'h1), wen && (wa[2:0]==3'h0)};
   we_size[1:0] <= {wen && (wa[2:0]==3'h3), wen && (wa[2:0]==3'h2)};
   we_addr      <=  wen && (wa[2:0]==3'h4);
   we_addr_d    <=  we_addr;

   if (we_pos[0])  pos_left[13:1]    <= wdd[13: 1];
   if (we_pos[1])  pos_top[13:1]     <= wdd[13: 1];
   if (we_size[0]) size_width[13:1]  <= wdd[13: 1];
   if (we_size[1]) size_height[13:1] <= wdd[13: 1];
   
   if      (we_addr)   hist_raddr[9:0] <= wdd[9:0];
   else if (rd_hist)   hist_raddr[9:0] <= hist_raddr[9:0] + 1;
   
   
 end

 // port A - histogram calculation
 // port B - histogram readout   
   RAMB16_S9_S9 i_hist_low (
      .DOA(hist_doa[7:0]),                     // Port A 8-bit Data Output
      .DOPA(hist_doa[8]),                      // Port A 1-bit Parity Output
      .ADDRA({hist_bank,hist_waddr[9:0]}),     // Port A 11-bit Address Input
      .CLKA( pclk2x),                            // Port A Clock
      .DIA(hist_dia[7:0]),                     // Port A 8-bit Data Input
      .DIPA(hist_dia[8]),                      // Port A 1-bit parity Input
      .ENA(hist_ena),                          // Port A RAM Enable Input
      .SSRA(1'b0),                             // Port A Synchronous Set/Reset Input
      .WEA(hist_wea),                          // Port A Write Enable Input

      .DOB(hist_do0[7:0]),                      // Port B 8-bit Data Output
      .DOPB(hist_do0[8]),                       // Port B 1-bit Parity Output
      .ADDRB({~hist_bank,hist_raddr[9:0]}),    // Port B 11-bit Address Input
      .CLKB(!sclk),                            // Port B Clock
      .DIB(8'h0),                              // Port B 8-bit Data Input
      .DIPB(1'h0),                             // Port-B 1-bit parity Input
      .ENB(rd_hist),                           // PortB RAM Enable Input
      .SSRB(1'b0),                             // Port B Synchronous Set/Reset Input
      .WEB(1'b0)                               // Port B Write Enable Input
   );
  
   RAMB16_S9_S9 i_hist_high (
      .DOA(hist_doa[16:9]),                     // Port A 8-bit Data Output
      .DOPA(hist_doa[17]),                     // Port A 1-bit Parity Output
      .ADDRA({hist_bank,hist_waddr[9:0]}),     // Port A 11-bit Address Input
      .CLKA( pclk2x),                            // Port A Clock
      .DIA(hist_dia[16:9]),                    // Port A 8-bit Data Input
      .DIPA(hist_dia[17]),                     // Port A 1-bit parity Input
      .ENA(hist_ena),                          // Port A RAM Enable Input
      .SSRA(1'b0),                             // Port A Synchronous Set/Reset Input
      .WEA(hist_wea),                          // Port A Write Enable Input

      .DOB(hist_do0[16:9]),                     // Port B 8-bit Data Output
      .DOPB(hist_do0[17]),                      // Port B 1-bit Parity Output
      .ADDRB({~hist_bank,hist_raddr[9:0]}),    // Port B 11-bit Address Input
      .CLKB(!sclk),                            // Port B Clock
      .DIB(8'h0),                              // Port B 8-bit Data Input
      .DIPB(1'h0),                             // Port-B 1-bit parity Input
      .ENB(rd_hist),                           // PortB RAM Enable Input
      .SSRB(1'b0),                             // Port B Synchronous Set/Reset Input
      .WEB(1'b0)                               // Port B Write Enable Input
   );
  
  
  
  
  
endmodule

