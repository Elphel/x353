/*
** -----------------------------------------------------------------------------**
** sdseq333.v
**
** formely part of mcontr333.v
**
** Copyright (C) 2002-2008 Elphel, Inc
**
** -----------------------------------------------------------------------------**
**  This file is part of X333
**  X333 is free software - hardware description language (HDL) code.
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
// 05/06/2008 - modified to delay precharge after write (tWR) - there was violation with
// clock period less than 7.5ns
// external commands sync to clk180
module sdseq (clk0,	// global clock 75-100MHz (hope to get to 120MHz with Spartan3)
				  rst,	// when active will use mancmd as a source for ras,cas,we,ba,a
				  xfer,	// start block transfer (=stepsEn[2]) - when address is ready
				  mode,	// 0 - 128x1, 1 - 18x9 (will work only in read mode, channnel 2)
				  wnr,
				  refr,	// start auto refresh (should be delayed same time as read/write)
				  sa,		// block start address
				  chsel,	// channel number (0..3) - used to generate delayed dsel;
              param, // dual-purpose parameter. In 256x1 mode specifies (5 LSBs) number of 8-word long groups to r/w (0- all 32),
				         //    actually for all but 0 (when exactly 256 words are xfered) will transfer one word more
 						   // In 18x9 mode specifies bits 13:8 of address increment between lines
				  mancmd,//	always high except 1 cycle long when it sources {ras,cas,we,ba[1:0],a[11:0]}. Works only whe rst is active
			   			// Will use 18 bits from unused descriptor cell , a[11] will be skipped and set to 1'b0????
				  sfa,   // [23:8]), start address of a frame - ugly hack to roll over the last (extra) 4 lines of a frame
						   // to the beginning of a frame - used in photofinish mode.
				  rovr,	   // roll over mode (photofinish, cntrl0[12]==1, last line of tiles)
              drun_rd,  // enable read data (from SDRAM) to blockRAM buffer
              drun_wr,  // enable write data (to SDRAM) from blockRAM buffer
              prefirstdrun, // 1 cycle ahead of the first drun_rd/drun_wr to be used by bufCntr256 to sample channel enable
				  dlast,    //last cycle of drun active for read, 1 cycle later for write **** for the last cycle in multi-drun accesses !
				  precmd,	// 1 cycle ahead {ras,cas,we} command bits to be registered at the output pads
				  prea,		// 13 bits (including ba) 1 cycle ahead of output A[12:0]
				  preb,		// 2 bits of bank address 1 cycle ahead of output BA[1:0]
				  next,		// get next request from que.
				  pre2trist,	// tristate data outputs (two ahead).
              predqt,   // tristate DQ outputs (one extra for FF in output buffer)
              dmask,     // [1:0] - now both the same (even number of words written)
              dqs_re,   // enable read from DQS i/o-s for phase adjustments  (latency 2 from the SDRAM RD command)
				  dsel);		// data buffer select (number of buffer to source SDRAM data in), so it is needed only for WRITE to SDRAM

	input					clk0;
   input             rst,xfer,mode,wnr,refr;
	input		[24:3]	sa;
	input		[ 1:0]	chsel;
   input    [ 5:0]   param;
	input		[17:0]	mancmd;
	input    [24:8]	sfa; // start frame address - photofinish hack
	input		         rovr;   // roll over mode (photofinish, cntrl0[12]==1, last line of tiles)

   output            drun_rd;
   output            drun_wr;
   output            prefirstdrun;
   output            dlast;
	output	[ 2:0]	precmd;
	output	[12:0]	prea;
	output	[ 1:0]	preb;
	output				next;
	output				pre2trist;
   output            predqt;
   output   [ 1:0]   dmask;
   output            dqs_re;
	output	[ 1:0]	dsel;

   reg     [4:0]     left; // full page will be 31, not 32 copunts - first delay will just be longer
   reg               mode_r;//, wnr_r; //after xfer
   reg               prefirst,first;

   reg   [1:0]    dsel;
   reg   [12:0]   prea;
   reg   [ 1:0]   preb;
   wire  [2:0]    precmd;
   reg   [1:0]    dmask; // for now both bits are the same
   reg            next;
   reg            pre2trist;// made 1 extra early - add FF to the i/op module
   reg            predqt;  // will go through additional FF at falling clk0 in I/O module
   reg            drun_rd;
   reg            drun_wr; //3 cycles ahead of data out available inside FPGA (before DDR resyncing)
   reg            dlast;
   reg            dqs_re;
// internals
 	reg	[24:3] fullAddr;	// full SDRAM address;
	reg	[24:3] nextAddr;		// full SDRAM address for the next 10-word block (in next line)
//	reg	[24:9] nextPageAddr; // used only for the the second access in mode1
	reg  [24:10] nextPageAddr; // used only for the the second access in mode1
   reg          start_m1;     // start in mode1 - both first and next lines
   reg          startf_m1;    // start in mode1 - only first line !
   reg          start_m0r;    // start in mode0 - read
   reg          start_m0w;    // start in mode0 - write
//   wire         pre6prech_m;   // 6 cycles ahead of precmd "precharge" mode1, single access or second in dual
   reg          pre6prech_m;   // 6 cycles ahead of precmd "precharge" mode1, single access or second in dual
   reg   [1:0]  pre7prech_m;   // 7 cycles ahead of precmd "precharge", [0] - read, [1] - write

   wire         pre2prech_m;   // 2 cycles ahead of precmd "precharge" mode1, single access or second in dual
   wire         pre2prech_m1d1; // 2 cycles ahead of precmd "precharge" mode1, dual access, first cycle
   wire         pre2act_m1d2;   // 2 cycles ahead of precmd "activate" mode1, dual access, second cycle
   wire         prenext_m1s;    // 1 cycle  ahead of possible "next" - mode1, single access - starts from start_m1
   wire         prenext_m1d;    // 1 cycle  ahead of possible "next" - mode1, dual access
   wire         precontinue_m1;  // 3 cycles ahead of "activate" of continuation cycle
   reg          continue_m1;    // 2 cycles ahead of "activate" of continuation cycle
   wire         continue_m0;    // 2 cycles ahead of "activate" of continuation cycle
   reg          possible_dual; // lower bits in full address are all ones - possible two acccesses will be needed in the current line
   reg          setNextAddr;    // update nextAddr
   reg          decLeft;
   wire         prenext_m0r;
   wire         prenext_m0w1;
   wire         prenext_m0;
   wire         pre2refr;
   wire         prenext_refr;
   reg          prerefr;
   reg          preact;
   reg          preprech;
   reg          preread;
   reg          prewrite;
   reg          prerw; // for address multiplexor
   reg          pre1act_m1d2;
   wire         pre2read;
   wire         pre2write;
   reg          predrun_wr;
   reg          pre4drun_rd;
   wire         predrun_rd;
   wire         pre4drun_rd_abort; // in second access of dual-access, where precharge command is later (to meet activate-to-precharge time)
   wire         predrun_wr_abort; // write nneeds extra NOP befrore precharge
   wire         predmask;
   wire         predlast_rd;
   wire         predlast_wr;
   wire         predqs_re;
   reg          repeat_r;
   wire         repeat_r_end;
   reg          repeat_w;
   wire         pre2read_next8;
   wire         pre2write_next8;
   reg          prefirstdrun;     // to sample cs in bufCntr256
   wire         pre2firstdrun_rd;
	reg          rollover;  //photofinish hack
   reg          prenext_wr; // same timing as it was "next" earlier
   wire         pre_next_old=(left==5'h1) && (prenext_m1s || prenext_m1d || prenext_m0);
	always @ (negedge clk0)
	  if (rst) begin
	    mode_r <= 1'b0;
	  end else if (xfer) begin
	    mode_r <= mode;
	  end
    always @ (negedge clk0) begin
	  rollover <=  (left[4:0]==5'h4) && rovr;
     if      (first)               fullAddr[ 9:3] <= sa[ 9:3];
     else if (pre2act_m1d2)        fullAddr[ 9:3] <= 7'h0;
     else if (prerw)               fullAddr[ 9:3] <= fullAddr[ 9:3]+1;
     else if (continue_m1)         fullAddr[ 9:3] <= {rollover?sfa[9:8]:nextAddr[9:8],nextAddr[ 7:3]}; // stored from sa[7:3]

     if      (first)          fullAddr[24:10] <= sa[24:10];
     else if (continue_m1)    fullAddr[24:10] <= rollover?sfa[24:10]:nextAddr[24:10];
     else if (pre2act_m1d2)   fullAddr[24:10] <= nextPageAddr[24:10];

     if      (pre2prech_m1d1) nextPageAddr[24:10] <= fullAddr[24:10]+1;

     if      (rst)            possible_dual <= 0;
     else if (first)          possible_dual <= (sa[ 7:3] == 5'h1f);
     start_m1    <= !rst && ((first && mode_r) || continue_m1);
     startf_m1   <= !rst && (first && mode_r);
     start_m0r   <= first && !mode_r && !wnr;
     start_m0w   <= first && !mode_r &&  wnr;
     setNextAddr   <= start_m1;
     if (setNextAddr) nextAddr[24:8] <= fullAddr[24:8]+param[5:0]; // 2 cycles after fullAddr was set
     if (first)       nextAddr[ 7:3] <= sa [7:3];
     prenext_wr <= drun_wr && pre_next_old;
     next <= prenext_refr || prenext_wr || (!drun_wr && pre_next_old); // add m0 and refr here too
     decLeft <= (prenext_m1s || prenext_m1d || prenext_m0); // add m0 and refr here too
     if (first) left[4:0] <= (mode)? 5'h14:((param[4:0]==5'b0)?5'h1f:param[4:0]);
///AF2015 - revert:    	 if (first || rst) left[4:0] <= (mode)? 5'h14:((param[4:0]==5'b0)?5'h1f:param[4:0]);
     else if (decLeft)   left[4:0] <= left[4:0] -1;
   end


// aHa! Here was the bug!!
// MSRL16_1 i_prenext_m1s      (.Q(prenext_m1s),   .A(4'h0), .CLK(clk0), .D(start_m1 &&  ~(possible_dual && fullAddr[8])));
 MSRL16_1 i_prenext_m1s      (.Q(prenext_m1s),   .A(4'h0), .CLK(clk0), .D(start_m1 &&  ~(possible_dual && &fullAddr[9:8])));
// modified to match tWR (delay precharge after write)
// MSRL16_1 i_pre6prech_m     (.Q(pre6prech_m),     .A(4'h1), .CLK(clk0), .D(prenext_m1s ||
//                                                                             prenext_m1d ||
//                                                                             ((left==5'h1) &&  prenext_m0)));
 MSRL16_1 i_pre2prech_m     (.Q(pre2prech_m),     .A(4'h3), .CLK(clk0), .D(pre6prech_m));

// MSRL16_1 i_pre2prech_m1d1   (.Q(pre2prech_m1d1),   .A(4'h5), .CLK(clk0), .D(start_m1 &&  (possible_dual && fullAddr[8])));
 MSRL16_1 i_pre2prech_m1d1   (.Q(pre2prech_m1d1),   .A(4'h5), .CLK(clk0), .D(start_m1 &&  (possible_dual && &fullAddr[9:8])));
 MSRL16_1 i_prenext_m1d      (.Q(prenext_m1d),      .A(4'h1), .CLK(clk0), .D(pre2prech_m1d1));
 MSRL16_1 i_pre2act_m1d2     (.Q(pre2act_m1d2),     .A(4'h0), .CLK(clk0), .D(prenext_m1d));

 MSRL16_1 i_precontinue_m1   (.Q(precontinue_m1),   .A(4'h1), .CLK(clk0), .D((left!=5'h0) && pre2prech_m));
  
 MSRL16_1 i_prenext_m0r       (.Q(prenext_m0r),     .A((param[4:0]==5'b0)?4'h3:4'h0), .CLK(clk0), .D(start_m0r));

 MSRL16_1 i_prenext_m0w1       (.Q(prenext_m0w1),     .A((param[4:0]==5'b0)?4'h6:4'h3), .CLK(clk0), .D(start_m0w));
 assign  prenext_m0=prenext_m0r || prenext_m0w1 || continue_m0;

 MSRL16_1 i_continue_m0      (.Q(continue_m0),      .A(4'h3), .CLK(clk0), .D(!rst && (left!=5'h1) && prenext_m0));
 MSRL16_1 i_pre2refr      (.Q(pre2refr),      .A(4'h6), .CLK(clk0), .D(refr));
 MSRL16_1 i_prenext_refr  (.Q(prenext_refr),  .A(4'h2), .CLK(clk0), .D(pre2refr));

// secondary signals (just delays from the primary ones)

 
 MSRL16_1 i_pre2read          (.Q(pre2read),          .A(4'h1), .CLK(clk0), .D(start_m0r || start_m1 || pre1act_m1d2));
 MSRL16_1 i_pre2write         (.Q(pre2write),         .A(4'h1), .CLK(clk0), .D(start_m0w));
 MSRL16_1 i_predrun_rd        (.Q(predrun_rd),        .A(4'h2), .CLK(clk0), .D(pre4drun_rd));
 MSRL16_1 i_pre4drun_rd_abort (.Q(pre4drun_rd_abort), .A(4'h4), .CLK(clk0), .D(pre2act_m1d2));
 MSRL16_1 i_predrun_wr_abort  (.Q(predrun_wr_abort),  .A(4'h0), .CLK(clk0), .D((prenext_m0w1 || continue_m0) && (left==5'h1)));
 MSRL16_1 i_predlast_rd       (.Q(predlast_rd),       .A(4'h2), .CLK(clk0), .D(pre4drun_rd && (preprech || pre4drun_rd_abort)));
 MSRL16_1 i_predlast_wr       (.Q(predlast_wr),       .A(4'h0), .CLK(clk0), .D(predrun_wr && predrun_wr_abort));
 MSRL16_1 i_predmask          (.Q(predmask),          .A(4'h2), .CLK(clk0), .D(!predrun_wr));
 MSRL16_1 i_predqs_re         (.Q(predqs_re),         .A(4'h1), .CLK(clk0), .D(pre4drun_rd));

 MSRL16_1 i_pre2read_next8    (.Q(pre2read_next8),    .A(4'h2), .CLK(clk0), .D(preread));
 MSRL16_1 i_pre2write_next8   (.Q(pre2write_next8),   .A(4'h2), .CLK(clk0), .D(prewrite));
 assign repeat_r_end= pre2prech_m || pre2prech_m1d1; // will work for read - add for write
 MSRL16_1 i_pre2firstdrun_rd  (.Q(pre2firstdrun_rd),  .A(4'h5), .CLK(clk0), .D(start_m0r || startf_m1));
 

   always @ (negedge clk0) begin
    pre7prech_m[1:0] <= {pre7prech_m[0],prenext_m1s ||
                                        prenext_m1d ||
                                       (prenext_m0 && (left==5'h1))};
    pre6prech_m   <= drun_wr?pre7prech_m[1]:pre7prech_m[0];

    prefirstdrun <= start_m0w || pre2firstdrun_rd;
    continue_m1  <= precontinue_m1;
    repeat_r     <= !rst && !repeat_r_end && (repeat_r || first || continue_m1);
    repeat_w     <= !rst && !pre6prech_m && (repeat_w || first);
    prefirst     <= xfer;
    first        <= prefirst;
    pre1act_m1d2 <= !rst && pre2act_m1d2;
    prerefr      <= !rst && pre2refr;
    preact       <= !rst && (first || continue_m1 || pre2act_m1d2);
    preprech     <= !rst && (pre2prech_m || pre2prech_m1d1);

    preread      <= !rst && ~(pre2prech_m || pre2prech_m1d1) &&  (pre2read  || (pre2read_next8 && repeat_r));
    prewrite     <= !rst && (pre2write || (pre2write_next8 && repeat_w));
    prerw        <= (~(pre2prech_m || pre2prech_m1d1) &&  (pre2read  || (pre2read_next8 && repeat_r))) ||
                       (pre2write || (pre2write_next8 && repeat_w));
                       
//    prea[12:0]   <=  rst? mancmd[12:0]  : (prerw?  {4'b0,fullAddr[8:3],3'b0} :(first?sa[21:9] :fullAddr[21:9]));
//    preb[ 1:0]   <=  rst? mancmd[14:13] :                                     (first?sa[23:22]:fullAddr[23:22]);
    
    prea[12:0]   <=  rst?
                       mancmd[12:0]  : 
                         (prerw?
                           {3'b0,fullAddr[9:3],3'b0} :
                             (first?
                               sa[22:10] :
                               fullAddr[22:10]));
    preb[ 1:0]   <=  rst?
                       mancmd[14:13] :
                         (first?
                           sa[24:23]:
                           fullAddr[24:23]);
    
    if (start_m0w) dsel[1:0] <= chsel[1:0];
    predrun_wr   <= start_m0w || (!rst && predrun_wr && ~predrun_wr_abort);
    pre4drun_rd  <= preread  || (!rst && pre4drun_rd && ~(preprech || pre4drun_rd_abort));

    drun_rd <= predrun_rd;
    drun_wr <= predrun_wr;
    dlast   <= (left==5'h0 ) && (predlast_rd || predlast_wr) && !pre1act_m1d2; // !pre1act_m1d2 removes first pulse in dual access lines
    dmask <={predmask,predmask};
    pre2trist<=rst || preprech || (pre2trist && ~prewrite);
    predqt  <=rst || (pre2trist && ~prewrite);
    dqs_re   <=predqs_re;
   end


// Use FF for cas, ras, we for correct simulation

  FD_1 #(.INIT(1'b1)) i_precmd_0	(.D(mancmd[15] && ~(prewrite | preprech)),         .C(clk0),.Q(precmd[0]));	//WE
  FD_1 #(.INIT(1'b1)) i_precmd_1	(.D(mancmd[16] && ~(prerefr | preread | prewrite)),.C(clk0),.Q(precmd[1]));	//CAS
  FD_1 #(.INIT(1'b1)) i_precmd_2	(.D(mancmd[17] && ~(prerefr | preact | preprech)), .C(clk0),.Q(precmd[2]));	//RAS

endmodule
