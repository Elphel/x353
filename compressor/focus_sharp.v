/*
** -----------------------------------------------------------------------------**
** focus_sharp.v
**
** Module to determine focus sharpness on  by integrating
** DCT coefficient, multiplied my 8x8 array and squared
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
`timescale 1ns/1ps
//TODO: Modify to work with other modes (now only on color)
module focus_sharp(clk,	// pixel clock
						 en,	// enable (0 resets counter)
                   sclk, // system clock, twe, ta,tdi - valid @negedge (ra, tdi - 2 cycles ahead
						 twe, // enable write to a table
						 ta,  // [9:0]  table address
						 tdi,  // [15:0] table data in (8 LSBs - quantization data)
						 mode, // focus mode (combine image with focus info) - 0 - none, 1 - replace, 2 - combine all,  3 - combine woi
						 firsti,   // first macroblock
						 lasti,    // last macroblock
						 tni,	   // block number in a macronblock - 0..3 - Y, >=4 - color (sync to stb)
						 stb,		// strobe that writes ctypei, dci
						 start,// marks first input pixel (needs 1 cycle delay from previous DCT stage)
						 di,	 // [11:0] pixel data in (signed)
						 quant_ds, //quantizator ds
						 quant_d, //[11:0]quantizator data output
                   quant_dc_tdo, //[15:0], MSB aligned coefficient for the DC component (used in focus module)
//						 quant_dc_tdo_stb,						 
						 do,	 // [11:0] pixel data out, make timing ignore (valid 1.5 clk earlier that Quantizer output)
						 ds,  // data out strobe (one ahead of the start of dv)
						 hifreq  //[31:0])  //  accumulated high frequency components in a frame sub-window
                   );



    input			clk;
    input			en;
    input			sclk;
    input			twe;
    input [ 9:0]	ta;
    input [15:0]	tdi;
	 input [ 1:0]  mode;
    input			firsti;	//  first macroblock (sync to stb)
    input			lasti;	// last macroblock (sync to stb)
    input [ 2:0]  tni;   	// block number in a macronblock - 0..3 - Y, >=4 - color (sync to stb)
    input			stb;		// strobe that writes ctypei, dci
    input			start;
    input [12:0]  di;
	 input         quant_ds;
    input [12:0]  quant_d;
	 input [15:0]  quant_dc_tdo; // MSB aligned coefficient for the DC component (used in focus module)
//	 input         quant_dc_tdo_stb;						 
    output[12:0]  do;
    output			ds;
    output [31:0] hifreq;
	 

    wire  [15:0]  tdo;
	 reg   [ 5:0]  tba;
	 reg   [11:0]  wnd_reg; // intermediate register
	 reg           wnd_wr;  // writing window
	 reg   [ 2:0]  wnd_a;   // window register address
	 
 // next measured in 8x8 blocks, totalwidth - write one less than needed (i.e. 511 fro the 512-wide window)
 // blocks on the border are included
	 reg   [ 8:0]  wnd_left;
	 reg   [ 8:0]  wnd_right;
	 reg   [ 8:0]  wnd_top;
	 reg   [ 8:0]  wnd_bottom;
	 reg   [ 8:1]  wnd_totalwidth;
	 reg   [ 3:0]  filt_sel0; // select filter number, 0..14 (15 used for window parameters)
	 reg   [ 3:0]  filt_sel;  // select filter number, 0..14 (15 used for window parameters)
    reg           stren; // strength (visualization)
// if (dcc_stb) dcc_first <= color_first && dcc_run && dcc_stb && ctype && !ctype_prev[0];
    reg   [ 2:0]  ic;
    reg   [ 2:0]  oc;
	 wire          first,last; //valid at start (with first di word), switches immediately after
	 wire  [ 2:0]  tn;
	 reg   [31:0]  hifreq;
	 reg   [39:0]  acc_frame;
//	 reg   [11:0]  do;  // combined quantizator/focus output
    reg   [12:0]  do;  // combined quantizator/focus output
	 reg   [12:0]  pre_do;
	 reg           pre_ds;
	 reg           need_corr_max; // limit output by quant_dc_tdo
	 reg   [11:0]  fdo; // focus data output
	 reg           ds;
	 reg           start_d; //start delayed by 1
	 reg   [ 2:0]  tn_d; //tn delayed by 1

    wire          out_mono;
    wire          out_window;
    wire  [12:0]  combined_qf; 
	 wire  [12:0]  next_do;
    wire  [12:0]  fdo_minus_max;
    reg [11:0] di_d;
    reg [11:0] d1; 
//    reg [9:0] start2;
    reg [8:0] start2;
//    reg [8:0] finish2;
    reg [7:0] finish2;
    reg [5:0] use_k_dly;
    reg [23:0] acc_blk; // accumulator for the sum ((a[i]*d[i])^2)
    reg [22:0] sum_blk; // accumulator for the sum ((a[i]*d[i])^2), copied at block end
    reg        acc_ldval; // value to load to acc_blk: 0 - 24'h0, 1 - 24'h7fffff
    wire acc_clear=start2[8];
    wire acc_add=use_k_dly[4];
    wire acc_corr=use_k_dly[5];
//    wire use_prod=use_k_dly[2] ;// feed multiplier input regs with the delayed product
    wire acc_to_out=finish2[6];      
    wire [17:0] mult_a;
    wire  [17:0] mult_b;
    wire [35:0] mult_p;
    reg  [17:0] mult_s; //truncated and saturated (always positive) multiplier result (before calculating squared)
    reg         next_ac; // next will be AC component
    reg         use_coef; // use multiplier for the first operation - DCT coeff. by table elements
    reg         started_luma;// started Luma block 
    reg         luma_dc_out; // 1 cycle ahead of the luma DC component out (optionally combined with the WOI (mode=3))
    reg         luma_dc_acc; // 1 cycle ahead of the luma DC component out (always combined with the WOI)
    reg         was_last_luma;
    reg         copy_acc_frame;
    assign 	      fdo_minus_max[12:0]= {1'b0,fdo[11:0]}-{1'b0,quant_dc_tdo[15:5]};
	 
/*
	 assign        combined_qf[12:0]=stren?({quant_d[11],quant_d[11:0]}+{1'b0,fdo[11:0]}): //original image plus positive
	                                       ({quant_d[11],quant_d[11],quant_d[11:1]}+ // half original 
														 {fdo_minus_max[12],fdo_minus_max[12:1]}); // plus half signed
    assign        next_do[12:0] =  (mode[1:0]==2'h1)?(luma_dc_out?fdo_minus_max[12:0]:13'h0):
                                   ((mode[1] && luma_dc_out )? combined_qf[12:0]: {1'b0,quant_d[11:0]} );
*/
    assign        combined_qf[12:0]=stren?({quant_d[12:0]}+{1'b0,fdo[11:0]}): //original image plus positive
                                          ({quant_d[12],quant_d[12:1]}+ // half original 
                                           {fdo_minus_max[12],fdo_minus_max[12:1]}); // plus half signed
	 assign        next_do[12:0] =  (mode[1:0]==2'h1)?(luma_dc_out?fdo_minus_max[12:0]:13'h0):
	                                ((mode[1] && luma_dc_out )? combined_qf[12:0]: {quant_d[12:0]} );

	 always @ (posedge clk) begin
	   if (!en) ic[2:0] <= 3'b0;
		else if (stb) ic[2:0] <= ic[2:0]+1;
	   if (!en) oc[2:0] <= 3'b0;
		else if (start) oc[2:0] <= oc[2:0]+1;
	 end

// writing window parameters in the last bank of a table	 
    always @ (negedge sclk) begin
      if (twe) begin
		  wnd_reg[11:0] <= tdi[11:0] ;
		  wnd_a  <= ta[2:0];
		end
		wnd_wr <= twe && (ta[9:3]==7'h78) ; // first 8 location in the last 64-word bank
		if (wnd_wr) begin
        case (wnd_a[2:0])
          3'h0: wnd_left[8:0]       <= wnd_reg[11:3] ;
          3'h1: wnd_right[8:0]      <= wnd_reg[11:3] ;
          3'h2: wnd_top[8:0]        <= wnd_reg[11:3] ;
          3'h3: wnd_bottom[8:0]     <= wnd_reg[11:3] ;
          3'h4: wnd_totalwidth[8:1] <= wnd_reg[11:4] ;
          3'h5: filt_sel0[3:0]      <= wnd_reg[3:0] ;
          3'h6: stren               <= wnd_reg[0] ;
        endcase
		end
	 end
// determine if this block needs to be processed (Y, inside WOI)
	 reg  [ 7:0]  mblk_hor; //horizontal macroblock (2x2 blocks) counter
	 reg  [ 7:0]  mblk_vert; //vertical macroblock (2x2 blocks) counter
	 wire         start_of_line= (first || (mblk_hor[7:0] == wnd_totalwidth[8:1]));
	 wire         first_in_macro= (tn[2:0]==3'h0);
	 reg          in_woi; // maybe specified as slow

	 always @(posedge clk) begin
	   if (first_in_macro && start) mblk_hor[7:0] <= start_of_line? 8'h0:(mblk_hor[7:0]+1);
	   if (first_in_macro && start && start_of_line) mblk_vert[7:0] <= first? 8'h0:(mblk_vert[7:0]+1);
		start_d <= start;
		tn_d[2:0] <= tn[2:0];
		if (start_d) in_woi <= !tn_d[2] && 
		                                       ({mblk_hor [7:0],tn_d[0]} >= wnd_left[8:0]) &&
		                                       ({mblk_hor [7:0],tn_d[0]} <= wnd_right[8:0]) &&
		                                       ({mblk_vert[7:0],tn_d[1]} >= wnd_top[8:0]) &&
		                                       ({mblk_vert[7:0],tn_d[1]} <= wnd_bottom[8:0]);
	 end
 
//Will use posedge sclk to balance huffman and system

 wire clkdiv2;
	FD i_clkdiv2(.C(clk), .D(!clkdiv2), .Q(clkdiv2));
 reg [2:0] clksync;
 wire      csync=clksync[2];
	always @ (posedge sclk) begin
	   clksync[2:0] <= {(clksync[1]==clksync[0]),clksync[0],clkdiv2};
	end


   always @ (posedge clk) begin
       if (di[11]==di[12]) di_d[11:0] <=di[11:0];
       else di_d[11:0] <= {~di[11],{11{di[11]}}}; //saturate
   end
 
assign 	  mult_a[17:0] = use_coef ? {1'b0,tdo[15:0],1'b0}: mult_s[17:0];
assign	  mult_b[17:0] = use_coef ? {d1[10:0],{7{d1[0]}}}: mult_s[17:0];

   always @ (posedge sclk) begin
	  filt_sel[3:0] <= filt_sel0[3:0];
	  if (clksync[2]) d1[11:0]<=di_d[11:0];
//	  start2[9:0] <= {start2[8:0], start && csync};
	  start2[8:0] <= {start2[7:0], start && csync};
//     finish2[8:0]<= {finish2[7:0],use_coef && !next_ac};
     finish2[7:0]<= {finish2[6:0],use_coef && !next_ac};

	  
	  if      (!en || start2[0]) tba[5:0] <= 6'h0;
	  else if (!csync && (tba[5:0] != 6'h3f))   tba[5:0] <= tba[5:0] + 1;
	  mult_s[17:0] <= (&mult_p[35:31] || !(&mult_p[35:31]))?mult_p[31:14]:18'h1ffff;
	  next_ac <= en && (start2[3] || (next_ac && ((tba[5:0] != 6'h3f) || csync )));
	  use_coef <= next_ac && !csync;
	  use_k_dly[5:0]<={use_k_dly[4:0],use_coef};
     acc_ldval <= !(|start2[7:6]);
     if      (acc_clear || (acc_corr && acc_blk[23])) acc_blk[23:0] <= {1'b0,{23{acc_ldval}}};
     else if (acc_add)   acc_blk[23:0] <= acc_blk[23:0]+mult_p[35:8];
     if (acc_to_out) fdo[11:0] <= (|acc_blk[23:20])?12'hfff:acc_blk[19:8]; // positive, 0..0xfff
	  if (acc_to_out) sum_blk[22:0] <= acc_blk[22:0]; // accumulator for the sum ((a[i]*d[i])^2), copied at block end

   end

//	acc_blk will (after corr) be always with MSB=0 - max 24'h7fffff
// for image output - max 24'h0fffff->12 bit signed, shifted
// combining output
//assign        combined_qf[12:0]={quant_d[11],quant_d[11:0]}+{fdo[11],fdo[11:0]};


    SRL16 i_out_mono   (.Q(out_mono),   .A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b1), .CLK(clk), .D(started_luma)); // timing not critical
    SRL16 i_out_window (.Q(out_window), .A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b1), .CLK(clk), .D(in_woi)); // timing not critical
	 always @ (posedge clk) begin
	   if (start) started_luma <= !tn[2];
	   luma_dc_out <= quant_ds && out_mono && ((mode[1:0]!=3) || out_window);
		luma_dc_acc <= quant_ds && out_mono && out_window;
      was_last_luma <= en && last && out_mono;
		copy_acc_frame <= was_last_luma && !out_mono;
		if (first && first_in_macro) acc_frame[39:0] <= 40'h0;
		else if (luma_dc_acc)        acc_frame[39:0] <= acc_frame[39:0] + sum_blk[22:0];
		if (copy_acc_frame) hifreq[31:0] <= acc_frame[39:8];
	   pre_ds <= quant_ds;
	   ds <= pre_ds;
		pre_do[12:0] <= next_do[12:0];
		need_corr_max <=luma_dc_out && (mode[1:0]!=2'h0);
/*		do[11:0] <= (need_corr_max && !pre_do[12] && (pre_do[11] || (pre_do[10:0]>quant_dc_tdo[15:5])) )?
		           {1'b0,quant_dc_tdo[15:5]} :
					  pre_do[11:0];
*/
      do[12:0] <= (need_corr_max && !pre_do[12] && (pre_do[11] || (pre_do[10:0]>quant_dc_tdo[15:5])) )?
                 {2'b0,quant_dc_tdo[15:5]} :
                 pre_do[12:0];
	 end
   
   MULT18X18SIO #(
      .AREG(1), // Enable the input registers on the A port (1=on, 0=off)
      .BREG(1), // Enable the input registers on the B port (1=on, 0=off)
      .B_INPUT("DIRECT"), // B cascade input "DIRECT" or "CASCADE" 
      .PREG(1)  // Enable the input registers on the P port (1=on, 0=off)
   ) i_focus_mult (
      .BCOUT(), // 18-bit cascade output
      .P(mult_p),    // 36-bit multiplier output
      .A(mult_a),    // 18-bit multiplier input
      .B(mult_b),    // 18-bit multiplier input
      .BCIN(18'h0), // 18-bit cascade input
      .CEA(en), // Clock enable input for the A port
      .CEB(en), // Clock enable input for the B port
      .CEP(en), // Clock enable input for the P port
      .CLK(sclk), // Clock input
      .RSTA(1'b0), // Synchronous reset input for the A port
      .RSTB(1'b0), // Synchronous reset input for the B port
      .RSTP(1'b0)  // Synchronous reset input for the P port
   );





	 
	 RAM16X1D i_tn0    (.D(tni[0]),.DPO(tn[0]),.A0(ic[0]),.A1(ic[1]),.A2(1'b0),.A3(1'b0),.DPRA0(oc[0]),.DPRA1(oc[1]),.DPRA2(1'b0),.DPRA3(1'b0),.WCLK(clk),.WE(stb));
	 RAM16X1D i_tn1    (.D(tni[1]),.DPO(tn[1]),.A0(ic[0]),.A1(ic[1]),.A2(1'b0),.A3(1'b0),.DPRA0(oc[0]),.DPRA1(oc[1]),.DPRA2(1'b0),.DPRA3(1'b0),.WCLK(clk),.WE(stb));
	 RAM16X1D i_tn2    (.D(tni[2]),.DPO(tn[2]),.A0(ic[0]),.A1(ic[1]),.A2(1'b0),.A3(1'b0),.DPRA0(oc[0]),.DPRA1(oc[1]),.DPRA2(1'b0),.DPRA3(1'b0),.WCLK(clk),.WE(stb));
	 RAM16X1D i_first  (.D(firsti),.DPO(first),.A0(ic[0]),.A1(ic[1]),.A2(1'b0),.A3(1'b0),.DPRA0(oc[0]),.DPRA1(oc[1]),.DPRA2(1'b0),.DPRA3(1'b0),.WCLK(clk),.WE(stb));
	 RAM16X1D i_last   (.D(lasti), .DPO(last), .A0(ic[0]),.A1(ic[1]),.A2(1'b0),.A3(1'b0),.DPRA0(oc[0]),.DPRA1(oc[1]),.DPRA2(1'b0),.DPRA3(1'b0),.WCLK(clk),.WE(stb));

	 

   RAMB16_S18_S18 i_focus_dct_tab (
      .DOA(tdo[15:0]),       // Port A 16-bit Data Output
      .DOPA(),     // Port A 2-bit Parity Output
      .ADDRA({filt_sel[3:0],tba[2:0],tba[5:3]}),   // Port A 10-bit Address Input
      .CLKA(sclk),     // Port A Clock
      .DIA(16'b0),       // Port A 16-bit Data Input
      .DIPA(2'b0),     // Port A 2-bit parity Input
      .ENA(1'b1),       // Port A RAM Enable Input
      .SSRA(1'b0),     // Port A Synchronous Set/Reset Input
      .WEA(1'b0),       // Port A Write Enable Input

      .DOB(), // Port B 16-bit Data Output
      .DOPB(),     // Port B 4-bit Parity Output
      .ADDRB({ta[9:0]}),   // Port B 2-bit Address Input
      .CLKB(!sclk),     // Port B Clock
      .DIB(tdi[15:0]),       // Port B 16-bit Data Input
      .DIPB(2'b0),     // Port-B 2-bit parity Input
      .ENB(1'b1),       // PortB RAM Enable Input
      .SSRB(1'b0),     // Port B Synchronous Set/Reset Input
      .WEB(twe)        // Port B Write Enable Input
   );

endmodule

