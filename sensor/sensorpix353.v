/*
** -----------------------------------------------------------------------------**
** sensorpix353.v
**
** Input sensor data processing
**
** Copyright (C) 2002-2008 Elphel, Inc.
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

module sensorpix(  pclk,                  // clock (==pclk)
// control interface
                   sclk,               // global memory clock, @negedge
                   we_lensff,         // write parameters to lens flat field correction module

                   pre_wfpn,           // decoded addresses - write fpn [10:0] mode
                                       // [10] - testmode - if 1 generates gradient data (as Zoran chips) where pixel value = horizontal position
                                       // [9:7] - submode - subtract background (8 bits) mode (use di[7:0]):
                                       // 000 - no subtraction;
                                       // 001 - (finest) subtract 8 bit bkgnd from 12 bits pixels
                                       // 010 - shift 8bit bkgnd 1 bit  left before applying
                                       // 011 - shift 8bit bkgnd 2 bits left before applying
                                       // 100 - shift 8bit bkgnd 2 bits left before applying
                                       // 101 - shift 8bit bkgnd 2 bits left before applying
                                       // fpn data to subtract should be a little less to add a "fat zero" 
                                       // [6:4] - mpymode sensitivity correction mode (use di[15:8]):
                                       // 000 - no correction
                                       // 001 - fine correction (+/-3.125%)
                                       // 010 - fine correction (+/-6.25%)
                                       // 011 - fine correction (+/-12.5%)
                                       // 100 - +/- 25%
                                       // 101 - +/- 50%
                                       // [3] - wdth - word width: 0 - 8 bit, 1 - 16 bit (5 MSB == 0)
                                       // bits [2:0] - REMOVED 
                                       // [2:0] scaling of 11 bit FPN result to fit in 8bit output:
                                       // 00 - default - use [9:1]
                                       // 01 - use [10:2] - to protect from saturation after applying mpymode
                                       //      nominal range - 0..127
                                       // 10 - use [7:0] before saturation, "digital gain" == 4 (maximal)
                                       // 11 - use [8:1] before saturation, "digital gain" == 2
                   pre_wthrsh,         // write thresh. register [21:0]
                                       // [21:0] - trigger threshold - sum of pixels in a line,  [7:0] - for fpn mode
                   ta,                 // "curves" table write address
                   twce,               // "curves" table write enable
                   wd,                 // used for curves (instead of wd)- delayed one cwr pulse
                   en,                 //  Enable. Should go active before or with the first hact going active.
                                       // when low will also reset MSB of addresses - buffer page for ping-pong access.
                                       // SDRAM ch1 should be enabled earler to have data ready in the buffer
                                       // When going low will mask input hact, finish pending data->SDRAM and quit
                                       // So normal sequence is:
                                       //   1 - program (end enable) SDRAM channels 0 and 1, channel 1 will start reading
                                       //   2 - wait for frame sync and enable "en"
                                       //   3 (optional) - after frame is over (before the first hact of the next one)
                                       //      turn "en" off. If needed to restart - go to step 1 to keep buffer pages in sync.
                   trig,               // trigger out - fires sum of pixels in a line > threshold
                   trig_sel,           // threshold !=0, use this trigger and disable external
                   bayer,              // bayer phase, used for "gamma" tables
                                       // 1-st 256 16-bit words R, then 256*Gr, 256*Gb, 256*B

                   hact_out,

// sensor interface
                   hact,               // line active
//                 pxd,                // [11:0] - 12 bit pixel data
                   pxd,                // [15:0] - 16 bit pixel data
// channel 0 (data->SDRAM) interface
                   dwe,                // WE to SDRAM buffer
                   wa,                 // 10 bit address (2 MSB - page # for ping-pong access to 256x16 pages)
                   do,                 // 16 bit data to SDRAM (1/pixel for raw, 0.5/pixel for FPN processed
                   wpage,              // write page to SDRAM
//                  ch0rdy,            // SDRAM buffer is ready (will not be used as sensor does not wait)
// channel 1 (FPN data from SDRAM) interface
                   ra,
                   di,
                   rpage,
                   vacts_sclk,         // frame sync, @negedge single cycle
                   table_page);        // number of gamma table page currently in use (writes go to the other one)

  input           pclk;
  input           sclk;
  input           we_lensff;
  input           pre_wfpn;
  input           pre_wthrsh;
  input  [15:0]   wd;
  input           en;
  output          trig;
  output          trig_sel;
  input  [ 1:0]   bayer;              // bayer phase
  output          hact_out;

  input           hact;
  input  [15:0]   pxd;
  output          dwe;
  output [ 9:0]   wa;
  output [15:0]   do;
  output          wpage;
  output [ 9:0]   ra;
  input  [15:0]   di;
  output          rpage;
  input  [9:0]    ta;
  input           twce;
  input           vacts_sclk;
  output          table_page;
  wire            table_page;
  wire            next_table_page;
  reg    [12:0]   fsc;     // sensitivity correction (di[15:0] and mpymode - for now - leave 13 bits
  reg    [15:0]   pd_a;    // just delayed pixel data
//  wire [15:0]   fbg;     // scaled di[7:0] according to submode[1:0]
  reg    [15:0]   fbg;     // scaled di[7:0] according to submode[1:0]
  wire   [16:0]   pd_subp; // background subtracted
  reg    [15:0]   pd_sub;  // registered after bkgnd subtracted, applied zero saturation
  wire   [28:0]   pd_corrp;// full 29-bit result of application of sensitivity correction
  reg    [16:0]   pd_corr; // registered 17 MSBs from the result of sensitivity correction
  wire   [16:0]   pd_corr_r;  // registered pd_corr
//  reg    [16:0]   pd_corr_r2; // registered pd_corr_r (extra delay to match increased gamma latency)

  reg    [15:0]   do;      // 16-bit data to SDRAM
  wire            hact_dly3;// hact delayed 3 clocks to be combined with hact_en
//  wire          hact_outp;// hact delayed 5 clocks combined with hact_en to control writing to SDRAM
//  reg              hact_out; // hact delayed 6 clocks
//  wire          en_out;  // en delayed  5 clocks
// increased latency
  wire            hact_outp;// hact delayed 6 clocks combined with hact_en to control writing to SDRAM
  reg             hact_out; // hact delayed 7 clocks
  wire            en_out;  // en delayed  6 clocks
  
  reg    [ 9:0]   ra;
  reg    [ 9:0]   wa;
  reg             dwe;
  wire            incbra;  // increment read bank (ra[7])
  wire            incbwa;  // increment read bank (wa[7])
  reg             rpage;
  reg             wpage;
  reg    [ 9:0]   testdata;
  reg    [21:0]   thresh;
  reg    [22:0]   sumpix;
  reg             trig_sel;

  reg          testmode;
  reg [ 2:0]   submode;
  reg [ 2:0]   mpymode;
  reg          wdth;
//  reg  [ 2:0]   shft;
  reg          trig;
//  wire       hact_m= hact && en;
  reg          hact_m;
  reg   [3:0]  hact_d; /// combine sevaral dealys?


//  reg          hact_d0;

  wire [9:0]   ta;
  wire         twce;

  reg   [15:0] dsat; //saturated 16-bit corrected data - input to "curves"
  reg   [15:0] dsat_d; //
//  reg   [ 7:0] dsat_r; // delayed LSBs of dsat
  reg   [15:0] pd_lenscorr_in; //data input to lens flat field correction (after dsat
  wire  [15:0] pd_lenscorr_out; //data after lens flat fiel correction
  reg   [ 7:0] pd_lenscorr_out_d; // delayed LSBs of pd_lenscorr_out
  reg          en_d;
/*
                      .fstart(),    // frame start - single clock (will have frame latency as coefficients are written after the fstart)
                      .newline(),   // start of scan line  - ahead of linerun
                      .linerun(),   // active pixel output - latency will be = *** TBD

*/

  wire   [7:0] cdata;   //8-bit pixel data after "curves"
// modified table data to increase precision. table_base[9:0] is now 10 bits (2 extra).
// The 10-bit interpolation will be rounded to 8 bits at the very last stage
// 8 bit table_diff will be "floating point" with the following format
// now "signed" is 2's complement, was sign, abs() before

  wire   [7:0] table_diff_w; // 8 msbs it table word - msb - sign (0 plus, 1 - minus), other 7 bits - +/-127 difference to the next value
  wire   [9:0] table_base_w; // 8 lsbs in the table - base value, will be corrected using table_diff and input data lsbs (2 for now)
  wire  [35:0] table_mult;
// register decoded memory output
  reg    [9:0] table_base;
  reg   [10:0] table_diff;
  wire   [9:0] interp_data;
//
  reg          wfpn;
  reg          wthrsh;
  reg          twce_d;
  reg    [15:0] wdd;
//  wire         wfpn = pre_wfpn;
//  wire         wthrsh = pre_wthrsh;
  
//  reg          hact_d; //delayed by 1 pclk
  reg          bayer_nset; // set color to bayer (start of frame up to first hact)when zero
  wire         sync_bayer; // at the beginning of the line - sync color to bayer
  reg   [1:0]  color; // for selecting page in a gamma table
  reg          bayer0_latched; // latch bayer[0] at the beginning of first line

// adding latency (to increase sensor clock frequency) after multiplier.
// delaying table_mult[17:7], table_base [9:0] and pd_corr_r2[16:1]
// also SDRAM control outputs 
  reg   [17:7] table_mult_r;
  reg   [ 9:0] table_base_r;
//  reg   [16:1] pd_corr_r3;


//  input           vacts_sclk;
//twce_d
  FDE_1    i_table_page      (.C(sclk), .CE(vacts_sclk),         .D(next_table_page),          .Q(table_page));
  FDE_1    i_next_table_page (.C(sclk), .CE(twce_d && &ta[9:0]), .D(!table_page),         .Q(next_table_page));

//  assign sync_bayer=hact_d[0] && ~hact_d[1];
  assign sync_bayer=hact_d[1] && ~hact_d[2];
  assign       interp_data[9:0] = table_base_r[9:0]+table_mult_r[17:8]+table_mult_r[7]; //round
  assign       cdata[7:0] = interp_data[9:2]; //truncate

  reg [7:0] pd_lenscorr_out_d2; // AF2015 
  always @ (posedge pclk) begin
    table_base[9:0]  <= table_base_w[9:0];
    table_diff[10:0] <= table_diff_w[7]?
                          {table_diff_w[6:0],4'b0}:
                          {{4{table_diff_w[6]}},table_diff_w[6:0]}; 
///    dsat_r[7:0]      <= dsat[7:0];
    pd_lenscorr_out_d[7:0] <= pd_lenscorr_out[7:0];
    pd_lenscorr_out_d2 <= pd_lenscorr_out_d; // AF2015 - one more cycle delay
    table_mult_r[17:7] <= table_mult[17:7];
    table_base_r[ 9:0] <= table_base[ 9:0];
  end


   MULT18X18 i_table_mult (
      .P(table_mult),    // 36-bit multiplier output
      .A({{7{table_diff[10]}},table_diff[10:0]}),    // 18-bit multiplier input
//      .B({10'b0,pd_lenscorr_out_d[7:0]})     // 18-bit multiplier input
      .B({10'b0,pd_lenscorr_out_d2[7:0]})     // 18-bit multiplier input // AF2015 - one more cycle delay
   );


  always @ (negedge sclk) begin
    wfpn     <= pre_wfpn;
    wthrsh   <= pre_wthrsh;
    twce_d   <= twce;
    if (pre_wfpn || pre_wthrsh || twce) wdd[15:0]<= wd[15:0];
  end

  always @ (negedge sclk) if (wfpn) begin
   testmode       <= wdd[10];
   submode[2:0]   <= wdd[9:7];
   mpymode[2:0]   <= wdd[6:4];
   wdth           <= wdd[3];
  end
  always @ (negedge sclk) if (wthrsh) thresh[21:0] <= {wd[5:0],wdd[15:0]};
  always @ (posedge pclk) begin
    hact_m <= hact && en;
    en_d   <= en;
    hact_d[3:0] <= {hact_d[2:0],hact};
//    hact_d0<=hact;
//    hact_d<=hact_d0;
    bayer_nset <= en && (bayer_nset || hact);
    bayer0_latched <= bayer_nset? bayer0_latched:bayer[0];
//    color[1:0] <= { en? ((hact_d0 && ~hact_d)^color[1]):bayer[1] ,
//                   (en &&(hact_d || ~hact_d0))?~color[0]:bayer[0] }; // adjust here correct phase
    color[1:0] <= { bayer_nset? (sync_bayer ^ color[1]):bayer[1] ,
                   (bayer_nset &&(~sync_bayer))?~color[0]:bayer0_latched };
  end

  always @ (posedge pclk)
   if (!hact_d[0])  testdata[9:0] <= 10'b0;
   else           testdata[9:0] <= testdata[9:0]+1;

  always @ (posedge pclk) //fsc is left 13 bits
   case (mpymode[2:0])
    3'b000 : fsc[12:0] <= 13'h1000;                            // 0x1000
    3'b001 : fsc[12:0] <= {~di[15], di[15], di[15], di[15], di[15] ,di[15:8]};   // 0xf80..0x107f - +/- 3.125%
    3'b010 : fsc[12:0] <= {~di[15], di[15], di[15], di[15], di[15:8], 1'b0};     // 0xf00..0x10fe - +/- 6.25%
    3'b011 : fsc[12:0] <= {~di[15], di[15], di[15], di[15:8], 2'b00};
    3'b100 : fsc[12:0] <= {~di[15], di[15], di[15:8], 3'b000};
    3'b101 : fsc[12:0] <= {~di[15], di[15:8], 4'b0000};                          // 0x800..0x17f0 - +/- 50%
    3'b110 : fsc[12:0] <= {~di[15], di[15:8], 4'b0000};                          // 0x800..0x17f0 - +/- 50%
    3'b111 : fsc[12:0] <= {~di[15], di[15:8], 4'b0000};                          // 0x800..0x17f0 - +/- 50%
   endcase
  always @ (posedge pclk) pd_a[15:0] <= testmode? {testdata[9:0],6'b0}: pxd[15:0];


  always @ (posedge pclk) fbg[15:0]= {submode[2]? 
                     ((submode[0] ||submode[1])?
                       {di[7:0],4'b0}:       // >4
                       {1'b0,di[7:0],3'b0}): // 4
                     (submode[1]?
                      (submode[0]?
                       {2'b0,di[7:0],2'b0}:  // 3
                       {3'b0,di[7:0],1'b0}): // 2
                      (submode[0]?
                       {4'b0,di[7:0]}:       // 1
                       12'b0)),              // 0
                       4'b0};


  assign pd_subp[16:0]={1'b0,pd_a[15:0]}-{1'b0,fbg[15:0]};
  always @ (posedge pclk) pd_sub[15:0] <= pd_subp[16]? 16'b0: pd_subp[15:0];
  assign pd_corrp[28:0]=   pd_sub[15:0] * fsc[12:0];
  always @ (posedge pclk) pd_corr[16:0] <= pd_corrp[28:12]; // 12 LSBs discarded

  SRL16 i_pd_corr_r1  (.Q(pd_corr_r[1]), .A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b0), .CLK(pclk), .D(pd_corr[1]));   ///7+1=8 (was 3 before lens correction)
  SRL16 i_pd_corr_r2  (.Q(pd_corr_r[2]), .A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b0), .CLK(pclk), .D(pd_corr[2]));   ///7+1=8 (was 3 before lens correction)
  SRL16 i_pd_corr_r3  (.Q(pd_corr_r[3]), .A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b0), .CLK(pclk), .D(pd_corr[3]));   ///7+1=8 (was 3 before lens correction)
  SRL16 i_pd_corr_r4  (.Q(pd_corr_r[4]), .A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b0), .CLK(pclk), .D(pd_corr[4]));   ///7+1=8 (was 3 before lens correction)
  SRL16 i_pd_corr_r5  (.Q(pd_corr_r[5]), .A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b0), .CLK(pclk), .D(pd_corr[5]));   ///7+1=8 (was 3 before lens correction)
  SRL16 i_pd_corr_r6  (.Q(pd_corr_r[6]), .A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b0), .CLK(pclk), .D(pd_corr[6]));   ///7+1=8 (was 3 before lens correction)
  SRL16 i_pd_corr_r7  (.Q(pd_corr_r[7]), .A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b0), .CLK(pclk), .D(pd_corr[7]));   ///7+1=8 (was 3 before lens correction)
  SRL16 i_pd_corr_r8  (.Q(pd_corr_r[8]), .A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b0), .CLK(pclk), .D(pd_corr[8]));   ///7+1=8 (was 3 before lens correction)
  SRL16 i_pd_corr_r9  (.Q(pd_corr_r[9]), .A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b0), .CLK(pclk), .D(pd_corr[9]));   ///7+1=8 (was 3 before lens correction)
  SRL16 i_pd_corr_r10 (.Q(pd_corr_r[10]),.A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b0), .CLK(pclk), .D(pd_corr[10]));  ///7+1=8 (was 3 before lens correction)
  SRL16 i_pd_corr_r11 (.Q(pd_corr_r[11]),.A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b0), .CLK(pclk), .D(pd_corr[11]));  ///7+1=8 (was 3 before lens correction)
  SRL16 i_pd_corr_r12 (.Q(pd_corr_r[12]),.A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b0), .CLK(pclk), .D(pd_corr[12]));  ///7+1=8 (was 3 before lens correction)
  SRL16 i_pd_corr_r13 (.Q(pd_corr_r[13]),.A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b0), .CLK(pclk), .D(pd_corr[13]));  ///7+1=8 (was 3 before lens correction)
  SRL16 i_pd_corr_r14 (.Q(pd_corr_r[14]),.A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b0), .CLK(pclk), .D(pd_corr[14]));  ///7+1=8 (was 3 before lens correction)
  SRL16 i_pd_corr_r15 (.Q(pd_corr_r[15]),.A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b0), .CLK(pclk), .D(pd_corr[15]));  ///7+1=8 (was 3 before lens correction)
  SRL16 i_pd_corr_r16 (.Q(pd_corr_r[16]),.A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b0), .CLK(pclk), .D(pd_corr[16]));  ///7+1=8 (was 3 before lens correction)

  always @ (posedge pclk) begin
     dsat[15:0]           <= pd_corrp[28]?16'hffff:pd_corrp[27:12];
     dsat_d[15:0]         <= dsat[15:0];
     pd_lenscorr_in[15:0] <= dsat_d[15:0]; /// just delay by 1 cycle
///TODO: done. Need to delay pd_corr_r3 by 4 more cycles for lens flat field correction

 // Raw data will be 1/2 of the full scale to allow correction to fit in 16-bit range
    do[15:0]         <= (wdth)? {pd_corr_r[16:1]} : {cdata[7:0],do[15:8]};
  end

//  assign  incbra= (ra[7:0] == 8'hff) || (|ra[7:0] && (~hact_m));
  assign incbra= (ra[7:0] == 8'hff) || (|ra[7:0] && (~(hact & en)));
  always @ (posedge pclk) begin
    rpage <= incbra;
//    if (!hact_m)   ra[7:0]    <= 8'h0;
    if (!(hact && en)) ra[7:0]    <= 8'h0;
    else               ra[7:0]    <= ra[7:0]+1;
    if (!en)           ra[9:8] <= 2'h0;
    else if (incbra)   ra[9:8] <= ra[9:8]+1;
    hact_out <= hact_outp;
    dwe       <= wdth? hact_outp : (!dwe && hact_out);
  end
/*
  SRL16 i_hact_dly3  (.Q(hact_dly3), .A0(1'b0), .A1(1'b1), .A2(1'b0), .A3(1'b0), .CLK(pclk), .D(hact_m));   // dly=2+1
  SRL16 i_en_out     (.Q(en_out),    .A0(1'b1), .A1(1'b0), .A2(1'b1), .A3(1'b0), .CLK(pclk), .D(en));    // dly=5+1
  SRL16 i_hact_outp  (.Q(hact_outp), .A0(1'b1), .A1(1'b0), .A2(1'b1), .A3(1'b0), .CLK(pclk), .D(hact_m));   // dly=5+1
*/
/// NOTE: adding 5 cycles here
  SRL16 i_hact_dly3  (.Q(hact_dly3), .A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b0), .CLK(pclk), .D(hact_m));   // dly=2+1+5
  SRL16 i_en_out     (.Q(en_out),    .A0(1'b0), .A1(1'b1), .A2(1'b0), .A3(1'b1), .CLK(pclk), .D(en));       // dly=5+1+5
//AF2015  SRL16 i_hact_outp  (.Q(hact_outp), .A0(1'b0), .A1(1'b1), .A2(1'b0), .A3(1'b1), .CLK(pclk), .D(hact_m));   // dly=5+1+5
  SRL16 i_hact_outp  (.Q(hact_outp), .A0(1'b1), .A1(1'b0), .A2(1'b0), .A3(1'b1), .CLK(pclk), .D(hact_m));   // dly=5+1+5
  assign incbwa= (dwe && (wa[7:0]==8'hff)) || (|wa[7:0] && !hact_out);
  always @ (posedge pclk) begin
    wpage <= incbwa;
    if (!hact_out)   wa[7:0] <= 8'h0;
    else if (dwe)    wa[7:0] <= wa[7:0]+1;
    if (!en_out)     wa[9:8] <= 2'h0;
    else if (incbwa) wa[9:8] <= wa[9:8]+1;
  end  
    // disable to save resources - registers will be optimized out
/**/
  always @ (posedge pclk) if (!hact_dly3) sumpix[22:0] <={1'b0,~thresh[21:0]};
    else                                  sumpix[22:0] <=sumpix[22:0]+pd_corr[16:6];  // preserve for now
  always @ (posedge pclk) trig_sel <= (|thresh[21:0]);
  always @ (posedge pclk) trig <= trig_sel && sumpix[22];
/**/
//next_table_page
   RAMB16_S9_S9 i_cstableh (
      .DOA({table_diff_w[6:0],table_base_w[9]}),// Port A 8-bit Data Output
      .DOPA(table_diff_w[7]),                   // Port A 8-bit Parity Output
      .ADDRA({table_page,color[1:0],pd_lenscorr_out[15:8]}),          // Port A 11-bit Address Input
      .CLKA(pclk),                              // Port A Clock
      .DIA(8'b0),                               // Port A 8-bit Data Input
      .DIPA(1'b0),                              // Port A 1-bit parity Input
      .ENA(1'b1), //(change to en???)           // Port A RAM Enable Input
      .SSRA(1'b0),                              // Port A Synchronous Set/Reset Input
      .WEA(1'b0),                               // Port A Write Enable Input

      .DOB(),                                   // Port B 8bit Data Output
      .DOPB(),                                  // Port B 1-bit Parity Output
      .ADDRB({~table_page,ta[9:0]}),            // Port B 11-bit Address Input
      .CLKB(!sclk),                             // Port B Clock
      .DIB({wd[0],wdd[15:9]}),                  // Port B 8-bit Data Input
      .DIPB(wd[1]),                             // Port-B 1-bit parity Input
      .ENB(twce_d),                             // PortB RAM Enable Input
      .SSRB(1'b0),                              // Port B Synchronous Set/Reset Input
      .WEB(1'b1)                                // Port B Write Enable Input
   );
   RAMB16_S9_S9 i_cstablel (
      .DOA(table_base_w[7:0]),                  // Port A 8-bit Data Output
      .DOPA(table_base_w[8]),                   // Port A 8-bit Parity Output
      .ADDRA({table_page,color[1:0],pd_lenscorr_out[15:8]}),          // Port A 11-bit Address Input
      .CLKA(pclk),                              // Port A Clock
      .DIA(8'b0),                               // Port A 8-bit Data Input
      .DIPA(1'b0),                              // Port A 1-bit parity Input
      .ENA(1'b1), //(change to en???)           // Port A RAM Enable Input
      .SSRA(1'b0),                              // Port A Synchronous Set/Reset Input
      .WEA(1'b0),                               // Port A Write Enable Input

      .DOB(),                                   // Port B 8-bit Data Output
      .DOPB(),                                  // Port B 1-bit Parity Output
      .ADDRB({~table_page,ta[9:0]}),            // Port B 11-bit Address Input
      .CLKB(!sclk),                             // Port B Clock
      .DIB(wdd[7:0]),                           // Port B 8-bit Data Input
      .DIPB(wdd[8]),                            // Port-B 1-bit parity Input
      .ENB(twce_d),                             // PortB RAM Enable Input
      .SSRB(1'b0),                              // Port B Synchronous Set/Reset Input
      .WEB(1'b1)                                // Port B Write Enable Input
   );
   
   /// AF2015 *************** Fixing old bug - moved outside ******************
/*   
   reg [1:0] newline_d;
   reg [1:0] linerun_d;
   always @ (posedge pclk) begin
       newline_d <= {newline_d[0],hact & ~hact_d[0]};
       linerun_d <= {linerun_d[0],hact_d[1]};
   end
*/   
lens_flat i_lens_flat(.sclk(sclk),                   /// system clock @negedge
                      .wen(we_lensff),               /// write LSW from di
                      .di(wd[15:0]),                 /// [15:0] data in
                      .pclk(pclk),                   /// pixel clock (@pclk)
                      .fstart(en && !en_d),          /// frame start - single clock (will have frame latency as coefficients are written after the fstart)
                      .newline(hact & ~hact_d[0]),    /// start of scan line  - ahead of linerun
//                      .newline(newline_d[1]),    /// start of scan line  - ahead of linerun
                      .linerun(hact_d[1]),           /// active pixel output - latency will be = 3 clocks
//                      .linerun(linerun_d[1]),           /// active pixel output - latency will be = 3 clocks
                      .bayer(bayer[1:0]),
                      .pixdi(pd_lenscorr_in[15:0]),  /// pixel data in,16 bit (normal data is positive, 15 bits)
                      .pixdo(pd_lenscorr_out[15:0])  /// pixel data out, same format as input
                       );

endmodule

