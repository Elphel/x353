/*
** -----------------------------------------------------------------------------**
** rtc353.v
**
** Real time clock, counts seconds (32 bit), microseconds (20 bits)
** allows fine adjustment (0.1 ppm to =/- 0.3%), used in timestamping images.
**
** Copyright (C) 2005-2007 Elphel, Inc
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

module rtc353    (mclk,    // system clock (negedge)
                  pre_we,  // 1 cycle ahead of writing data
                  wa,      // [1:0] write address
                           // 0 - write microseconds (actual write will happen after writing seconds)
                           // 1 - write seconds
                           // 2 - write correction (16 bit, signed
                           // 3 - nop, just latch the output 32+20 bits to read to CPU). Maybe nop (or 2 writes) are needed between read.
                  wd,      // [31:0] data to write, valid 1 cycle after pre_we, wa
                  musec,   // [19:0] usec output (register is latched by writing), 0..999999 (0..0xf423f)
                  msec,    // [31:0] seconds latched counter output
                  clk12,   // 12MHz clock (no PLL, just xtall). will be re-synchronized to mclk to avoid extra GCLK
                  pclk,    // sensor clock (posedge)
                  platch,  // latch counters sync to pclk (up to ~0.5usec delay. Start of first HACT after VACT?
                  pusec,   // [19:0] usec output - sync to sensor clock
                  psec,    // [31:0] seconds counter output
                  usec,    // running usec data out
                  sec);    // running sec data out
// counter is updated each 0.5 (non-compensated) microsecond. Usually counters are incremented each other upd pulse, but an interval
// may be 1 or 3 upd periods (0.5/1.5 usec)
    input         mclk;
    input         pre_we;
    input  [ 1:0] wa;
    input  [15:0] wd;
    output [19:0] musec;
    output [31:0] msec;
    input         clk12;
    input         pclk;
    input         platch;
    output [19:0] pusec;
    output [31:0] psec;
    output [19:0] usec;
    output [31:0] sec;
    

    reg           we_usec;
//    reg    [ 1:0] we_sec;
    reg           we_sec;
    reg           set_cntr;
    reg           we_corr;
    
    reg    [19:0] wusec;
    reg    [31:0] wsec;
    reg    [15:0] corr;
    
    reg    [19:0] usec;
    reg    [31:0] sec;

    reg    [19:0] usec_plus1;
    reg    [31:0] sec_plus1;

    
    reg    [19:0] musec;
    reg    [31:0] msec;

    reg    [19:0] pusec;
    reg    [31:0] psec;
    
    reg    [ 2:0] clk12_d; // make a clean 12MHz clock sync to system clock
    reg    [ 2:0] clk12_s; // make a clean 12MHz clock sync to system clock 
    reg    [ 2:0] cntr6; // divide by 6 (12MHz->2MHz)

    reg           pend_set_cntr; //ending set_cntr (will be synchronized)
    reg    [3:0]  halfusec;
    reg    [1:0]  inc_usec; // 2 bits - to reduce FO problems
    reg    [1:0]  inc_sec;
    reg   [23:0]  acc;
    wire  [24:0]  next_acc;
    reg     [2:0] usec_lsb; // usec LSB sampled @posedge pclk, delayed
    reg           platch_sample; // sample TS until usec LSB does not change (before and after
    reg           platch_test;   // see if need to re_sample;
    
    
    
    wire          enable_rtc; // for simulation
	 
	 reg  [15:0]   wd_r; // delayed data in (low bits)
	 
	 wire [31:0]   wdd={wd[15:0],wd_r[15:0]};
    assign next_acc[24:0]= {1'b0,acc[23:0]} + {1'b0,~corr [15], {7{corr [15]}}, corr[15:0]};

    always @ (posedge pclk) begin
      usec_lsb[2:0] <= {usec_lsb[1:0],usec[0]};
      platch_sample <= platch || (platch_test && ((usec_lsb[1]!=usec_lsb[0]) ||(usec_lsb[2]!=usec_lsb[0])));
      platch_test<=platch_sample;
      if (platch_sample) begin
        pusec[19:0] <= usec[19:0];
        psec [31:0] <= sec [31:0];
      end
    end

//    FDE_1  i_enable_rtc (.C(mclk),.CE(we_sec[0]),.D(1'b1),.Q(enable_rtc));
    FDE_1  i_enable_rtc (.C(mclk),.CE(we_sec),.D(1'b1),.Q(enable_rtc));
    
    always @ (negedge mclk) begin
	   wd_r[15:0] <= wd[15:0];
      clk12_d[2:0] <= {clk12_d[1:0], clk12};
      clk12_s[2:0] <= {clk12_s[1] && !clk12_s[0], clk12_s[0], (clk12_d[2:1]==2'h3) || (clk12_s[0] && !(clk12_d[2:1]==2'h0))};
    
      we_usec <= (wa[1:0]==2'h0) && pre_we;
//      we_sec[1:0]  <= {we_sec[0],(wa[1:0]==2'h1) && pre_we};
      we_sec  <= (wa[1:0]==2'h1) && pre_we;
      
//      if      (we_sec[0])     pend_set_cntr <= 1'b1;
      if      (we_sec)     pend_set_cntr <= 1'b1;
      else if (halfusec[3])pend_set_cntr <= 1'b0;
      
		if      (!enable_rtc) set_cntr <= 1'b0;
      else if (halfusec[3]) set_cntr <= pend_set_cntr;
      
      we_corr <= (wa[1:0]==2'h2) && pre_we;
      if (pre_we) begin
        musec[19:0] <= usec[19:0];
        msec [31:0] <= sec [31:0];
      end
            
      if (we_usec)   wusec[19:0] <= wdd[19:0];
//      if (we_sec[0]) wsec [31:0] <= wdd[31:0];
      if (we_sec)    wsec [31:0] <= wdd[31:0];
      if (we_corr)   corr [15:0] <= wdd[15:0];

      if (!enable_rtc)       cntr6[2:0] <= 3'h5;
      else if (clk12_s[2])   cntr6[2:0] <= (cntr6[2:0]==3'h0)?3'h5:(cntr6[2:0] - 1);
      
      halfusec[3:0] <= {halfusec[2:0],clk12_s[2] && (cntr6[2:0]==3'h0)};
      
      if (halfusec[1]) acc[23:0] <= set_cntr?24'h0:next_acc[23:0];
      
      inc_usec[1:0] <= {inc_usec[0],halfusec[1] & next_acc[24]};
      inc_sec [1:0] <= {inc_sec[0], halfusec[1] && next_acc[24] && (usec[19:0]==20'hf423f)};
      

// reducing delay from set_cntr to sec[]. usec[] - don't know how to constraint it really, so adding layer of registers
      sec_plus1[31:0]  <= sec[31:0] + 1;
      usec_plus1[19:0] <= usec[19:0] + 1;
      
      if      (set_cntr)    usec[19:0] <= wusec[19:0];
      else if (inc_sec[1])  usec[19:0] <= 20'h0;
      else if (inc_usec[1]) usec[19:0] <= usec_plus1[19:0];
      
      if      (set_cntr)    sec[31:0] <= wsec[31:0];
      else if (inc_sec[1])  sec[31:0] <= sec_plus1[31:0];
            
    end
    
endmodule


module timestamp353( mclk,    // system clock (negedge)
                     pre_we,  // 1 cycle ahead of writing data
                     wd,      // [1:0] data to write, valid 1 cycle after pre_we, wa
                     pclk,  // pixel clock
                     pxdi,  // [15:0] pixel data from sensor
                     pxdo,  // [15:0] data to replace pxdi (next cycle)
                     vacts, // vertical sync
                     hacti, // hact input
                     hacto, // hact output (next cycle)
                     sec,   // [31:0] number of seconds
                     usec,   // [19:0] number of microseconds
                     tlatch); // latch time - 1-st hact after vact
    
    input         mclk;
    input         pre_we;
    input   [1:0] wd;
    input         pclk;
    input  [15:0] pxdi;
    output [15:0] pxdo;
    input         vacts;
    input         hacti;
    output        hacto;
    input  [31:0] sec;
    input  [19:0] usec;
    output        tlatch;
    wire          ts_active;
    
    reg    [15:0] pxdo;
    reg           hacto;
    wire          ts_bit;
    
    reg          hact_d;
    reg          vact_pend;
    reg          odd_line;
//    reg  [ 3:0]  line;
    reg  [ 2:0]  line;
    reg          ts_line;  // line when timestamp is active    
    reg          tlatch;
    
    wire         start_ts;
    reg          pre_start_ts, start_ts_r; // only in normal mode
    reg          ts_continue;
    reg  [ 4:0]  ts_count;
    reg  [25:0]  ts_data;
    reg          wr_tsmode;
    wire   [1:0]  tsmode;    //[1:0] 0 - off, (pixels/lines below start from 0)
                            //      1 - 2rd and 3-th lines, from pixel 2 (normal mode, frame not increased)
                            //      2 - 0 and 1 lines, after pixel data (adds 28 pixels to each line:
                            //                                       {seconds[31:6],2'b0}               MSB first for line 0
                            //                                       {seconds[5:0],useconds[19:0],2'b0} MSB first for line 1
    

//delaying ts_data, pxdo by 1 clock
    reg   [15:0] pxdi_r;    // delayed pxdi
    reg          use_ts;    // use timestamp data
    reg          ts_active_r;
    reg          ts_line_r;
	 reg    [1:0] wdd;

    
    assign         ts_bit=ts_data[25];
    assign  start_ts  = (tsmode[1] && !hacti && hact_d) || start_ts_r;
    assign  ts_active = ts_continue || start_ts;

    FDE_1  i_tsmode_0 (.C(mclk),.CE(wr_tsmode),.D(wdd[0]),.Q(tsmode[0]));
    FDE_1  i_tsmode_1 (.C(mclk),.CE(wr_tsmode),.D(wdd[1]),.Q(tsmode[1]));


    always @ (negedge mclk) begin
	   wdd[1:0] <= wd[1:0];
      wr_tsmode <= pre_we;
    end
    
    always @ (posedge pclk) begin
      use_ts <= ts_active || (hact_d && !hacti);
      pxdi_r[15:0]                <= pxdi[15:0];
      if      (use_ts) pxdo[15:0] <= {16{ts_bit}};
      else if (hact_d) pxdo[15:0] <= pxdi_r[15:0];


      hact_d <= hacti;
      hacto  <= hacti || ts_active; 
      if (vacts)                           vact_pend <= 1'b1;
      else if (hacti && !hact_d)           vact_pend <= 1'b0;
      if (hacti && !hact_d) odd_line <= !vact_pend && !odd_line;
//      if (hacti && !hact_d) line[3:0] <= vact_pend?4'h1:{line[2:0],1'b0};
      if (hacti && !hact_d) line[2:0] <= vact_pend ? 3'h1 : {line[1:0],1'b0};
      
      if (hacti && !hact_d) ts_line <= tsmode[1]? (vact_pend || line[0] ) : (!vact_pend && (line[1] || line[2]));
      
      tlatch <= vact_pend && hacti && !hact_d;
      pre_start_ts <= (tsmode[1:0] == 2'h1) && 
		                hacti && !hact_d && !vact_pend  && (line[1] || line[2]);// before line changes
      
      start_ts_r   <= pre_start_ts;
      
      if      (start_ts)            ts_count[4:0] <= tsmode[1]?5'h1a:5'h18;
      else if (ts_continue)         ts_count[4:0] <= ts_count[4:0] - 1;
      
      if (tsmode[1:0]==2'h0)  	   ts_continue <= 1'b0;
      else if (start_ts)            ts_continue <= 1'b1;
      else if (ts_count[4:0]==5'h0) ts_continue <= 1'b0;
      
//      if      (ts_active)           ts_data[25:0]<= {ts_data[24:0],1'b0};
//      else if (ts_line)             ts_data[25:0]<= odd_line?sec[31:6]:{sec[5:0],usec[19:0]};
//      else	                       ts_data[25:0]<= 16'h0;
      ts_active_r <= ts_active;
      ts_line_r   <= ts_line;
      if      (ts_active_r)         ts_data[25:0] <= {ts_data[24:0],1'b0};
      else if (ts_line_r)           ts_data[25:0] <= odd_line?sec[31:6]:{sec[5:0],usec[19:0]};
	  else	                        ts_data[25:0] <= 26'h0;
    end   
                     
endmodule
                             
