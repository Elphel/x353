/*
** -----------------------------------------------------------------------------**
** encoderDCAC353.v
**
** RLL encoder for JPEG compressor
**
** Copyright (C) 2002-2008 Elphel, Inc
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


// Accepts  13-bits signed data (only 12-bit can be ecoded), so DC difference (to be encoded) is limited (saturated) to 12 bits, not the value itself
// AC - always limited to 800 .. 7ff
module encoderDCAC(clk,
						en,
//						firsti,	// was "first MCU in a frame" (@ stb)
						lasti,	// was "last MCU in a frame" (@ stb)
                  first_blocki,   // first block in frame - save fifo write address (@ stb) 
                  comp_numberi,   // [2:0] component number 0..2 in color, 0..3 - in jp4diff, >= 4 - don't use (@ stb) 
                  comp_firsti,    // fitst this component in a frame (reset DC) (@ stb) 
                  comp_colori,    // use color - huffman? (@ stb) 
                  comp_lastinmbi, // last component in a macroblock (@ stb) is it needed?
						stb,		// strobe that writes firsti, lasti, tni,average
						zdi,		// [11:0] zigzag-reordered data input
                  first_blockz,  // first block input (@zds)
						zds,		// strobe - one ahead of the DC component output
						last,		//
						do,
						dv
						);
    input		 clk;
    input		 en;
//    input		 firsti;
    input		 lasti;
    input       first_blocki;   // first block in frame - save fifo write address (@ stb) 
    input [2:0] comp_numberi;   // [2:0] component number 0..2 in color, 0..3 - in jp4diff, >= 4 - don't use (@ stb) 
    input       comp_firsti;    // fitst this component in a frame (reset DC) (@ stb) 
    input       comp_colori;    // use color - huffman? (@ stb) 
    input       comp_lastinmbi; // last component in a macroblock (@ stb) 
    input		 stb;
    input [12:0] zdi;
    input       first_blockz;  // first block input (@zds)
    input 		 zds;

    output		  last;

	 output	[15:0]	do;
	 output				dv;

//	 reg	[2:0]	ic;
//	 reg	[2:0]	oc;

    reg			last;

// 8x13  DC storage memory
    reg  [12:0]   dc_mem[7:0];
    reg  [12:0]   dc_diff0, dc_diff;
    wire [11:0]   dc_diff_limited=  (dc_diff[12]==dc_diff[11])? dc_diff[11:0] : {~dc_diff[11],{11{dc_diff[11]}}}; // difference (to be encoded) limited to fit 12 bits
    reg  [12:0]   dc_restored; // corrected DC value of the current block, compensated to fit difference to 12 bits
	 reg  [5:0]		rll_cntr;
	 reg [15:0]		do;
//	 reg				dv0;
	 reg				dv;

//
	 reg	[5:0]	cntr;
//	 reg			ac_on;
	 reg	[11:0] ac_in;

	 wire			izero=(ac_in[11:0]==12'b0);

	 reg	[14:0] val_r;	// DC diff/AC values to be sent out, registered

	 reg			 DCACen;	// enable DC/AC (2 cycles ahead of do
    wire			 rll_out;
//	 wire			 eob_out;
	 wire			 pre_dv;
	 reg			 was_nonzero_AC;
    reg  [12:0] zdi_d;
    reg   [3:0] zds_d;
//    wire        dc_mem_we=  zds_d[3]; // not needed
    wire        DC_tosend=  zds_d[2];
    wire        pre_DCACen= zds_d[1];

    wire  [2:0] comp_numbero;   // [2:0] component number 0..2 in color, 0..3 - in jp4diff, >= 4 - don't use
    wire        comp_firsto;    // first this component in a frame (reset DC)
    wire        comp_coloro;    // use color - huffman?
//    wire        first_blocko;   // first block in frame - save fifo write address
    wire        comp_lastinmbo; // last component in a macroblock
    wire        lasto;          // last macroblock in a frame
    reg   [2:0] block_mem_ra;
    reg   [2:0] block_mem_wa;
    reg   [2:0] block_mem_wa_save;
    reg   [6:0] block_mem[0:7];
    wire  [6:0] block_mem_o=block_mem[block_mem_ra[2:0]];
    assign      comp_numbero[2:0]= block_mem_o[2:0];
    assign      comp_firsto=      block_mem_o[3];
    assign      comp_coloro=       block_mem_o[4];
//    assign      first_blocko=      block_mem_o[5];
    assign      comp_lastinmbo=    block_mem_o[5];
    assign      lasto=             block_mem_o[6];
    always @ (posedge clk) begin
      if (stb) block_mem[block_mem_wa[2:0]] <= {lasti, comp_lastinmbi, /*first_blocki,*/ comp_colori,comp_firsti,comp_numberi[2:0]};
      if      (!en) block_mem_wa[2:0] <= 3'h0;
      else if (stb) block_mem_wa[2:0] <= block_mem_wa[2:0] +1;

      if (stb && first_blocki) block_mem_wa_save[2:0] <= block_mem_wa[2:0];

      if      (!en) block_mem_ra[2:0] <= 3'h0;
      else if (zds) block_mem_ra[2:0] <= first_blockz?block_mem_wa_save[2:0]:(block_mem_ra[2:0] +1);
    end

    assign	 rll_out= ((val_r[12] && !val_r[14]) || (ac_in[11:0]!=12'b0)) && (rll_cntr[5:0]!=6'b0);

	 assign	 pre_dv=rll_out || val_r[14] || was_nonzero_AC;

	 always @ (posedge clk) begin
	    val_r[14:0] <={ DC_tosend?
						    {en,
                       comp_coloro,
                       comp_lastinmbo && lasto, // last component's  in a frame DC coefficient
						     dc_diff_limited[11:0]}:
						    {2'b0,
							  (cntr[5:0]==6'h3f),
							  ac_in[11:0]}}; 
		 was_nonzero_AC <= en && (ac_in[11:0]!=12'b0) && DCACen;
		 if (pre_dv) do <= rll_out? {3'b0,val_r[12],6'b0,rll_cntr[5:0]}:{1'b1,val_r[14:0]};
 		 dv	<= pre_dv;
	 	 DCACen	<= en && (pre_DCACen || (DCACen && (cntr[5:0]!=6'h3f)));	// adjust
	 	 if (!DCACen) cntr[5:0] <=6'b0;
		 else			  cntr[5:0] <=cntr[5:0]+1;
	 end

	 always @ (posedge clk) begin
      zdi_d[12:0] <= zdi[12:0];
      ac_in[11:0] <= (zdi_d[12]==zdi_d[11])? zdi_d[11:0]:{~zdi_d[11],{11{zdi_d[11]}}};  // always // delay + saturation
		
		if (DC_tosend || !izero || !DCACen) rll_cntr[5:0]	<= 6'h0;
		else if (DCACen) rll_cntr[5:0]	<= rll_cntr[5:0] +1 ;
      if (DC_tosend) last <= lasto;
	 end

// DC components
	 always @ (posedge clk) begin
      zds_d[3:0]           <= {zds_d[2:0], zds};
	 	if (zds_d[0])   dc_diff0[12:0] <= comp_firsto?13'b0:dc_mem[comp_numbero[2:0]];
		if (zds_d[1])   dc_diff [12:0] <= zdi_d[12:0]-dc_diff0[12:0];
      if (zds_d[2])   dc_restored[12:0] <=  dc_diff0[12:0] + {dc_diff_limited[11],dc_diff_limited[11:0]};
      if (zds_d[3])   dc_mem[comp_numbero[2:0]]   <= dc_restored[12:0];
	 end


// Generate output stream to facilitate huffman encoding. The data will go to FIFO (16x) to compensate for possible long Huffman codes
// and/or zero-byte insertions
// format:
// {2'b11, color,last block,      dc[11:0]} - DC data
// {2'b10, 1'b0, last coeff,      ac[11:0]} - AC data (last coeff is set if it is last- 63-rd AC coefficient)
// {2'h00, 2'b00,      6'b0,rll[ 5:0]} - RLL zeroes.
// {2'h00, 2'b01,      6'b0,rll[ 5:0]} - end of block. lower 6 bits will have length that should be ignored

endmodule
