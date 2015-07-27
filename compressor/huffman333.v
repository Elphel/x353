/*
** -----------------------------------------------------------------------------**
** huffman333.v
**
** Huffman encoder for JPEG compressorrdy
**
** Copyright (C) 2002-2004 Elphelk, Inc
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
// 01/22/2004 - extended flush until ready (modified stuffer.v too)
module huffman    (pclk,	// half frequency, sync to incoming data
						 clk,		// pixel clock
						 en,		// enable (0 resets) sync to pclk
//						 cwr,		// CPU WR global clock
						 twe,		// enable write to a table - now the following will be valid ant negedge clk
						 ta,		// [8:0]  table address
						 tdi,		// [15:0] table data in
//						 readback,	 // [23:0] readback data
						 di,		// [15:0]	specially RLL prepared 16-bit data (to FIFO) (sync to pclk)
						 ds,		// di valid strobe  (sync to pclk)
						 rdy,		// receiver (bit stuffer) is ready to accept data
						 do,		// [15:0]	output data
						 dl,		// [3:0] data length (4'h0 is 'h16)
						 dv,		// output data valid
						 flush,	// last block done - flush the rest bits
						 last_block,
						 test_lbw,
						 gotLastBlock
						 );

    input 			pclk;
    input 			clk;
    input			en;
//    input			cwr;
    input			twe;
    input [ 8:0]	ta;
    input [15:0]	tdi;
//    output [23:0]	readback;	 // readback data

    input [15:0]	di;
    input			ds;
    input			rdy;
    output [15:0]	do;
    output [ 3:0]	dl;
    output			dv;
	 output			flush;
	 
	 output        last_block;
	 output        test_lbw;
	 output        gotLastBlock;
	 reg           test_lbw;
      wire [19:0] tables_out;
	 wire [15:0]	hcode;     // table output huffman code (1..16 bits)
	 wire [ 3:0]	hlen;		// table - code length only 4 LSBs are used
	 wire	 [11:0]	unused;     // SuppressThisWarning Veditor UNUSED 
	 reg	 [ 7:0]	haddr_r;	// index in huffman table	
	 wire [ 7:0]   haddr_next;
	 wire [ 8:0]	haddr;	// index in huffman table	 (after latches)
	 wire	[15:0]	fifo_o;
	 reg				stuffer_was_rdy;
	 wire			tables_re;
	 wire			read_next;	// assigned depending on steps (each other cycle for normal codes, each for special 00/F0

reg	[5:0]	steps;
// first stage registers 
reg	[5:0]	rll;	// 2 MSBs - counter to send "f0" codes

// replacing SRL16 with FD as SRL has longer output delay from clock 
  reg [3:0]    rll1;
  reg [3:0]    rll2;
reg			typeDC;
reg			typeAC;
reg  [11:0] sval;	// signed input value

wire	[1:0]	code_typ0;	// valid at steps[0]

reg			tbsel_YC0;	// valid at steps[0] - 0 -Y table, 1 - CbCr
  reg  [1:0]     code_typ1;
  reg  [1:0]     code_typ2;
  reg            code_typ3;
  reg            code_typ4;
reg			tbsel_YC1;
reg			tbsel_YC2;
reg			tbsel_YC3;
reg			last_block;	// 


reg	[15:0]	out_bits;	// bits to send
reg    [3:0]	out_len;		// length of bits to send (4'h0 means 16)
wire			fifo_or_full;	// fifo output register full read_next
wire			will_read;
wire [10:0]	var_do;
wire	[3:0]	var_dl;
wire	[3:0]	var_dl_late;

//wire	dv;
reg	dv;
reg	dv0;

//reg	dv_d;						// dv0 delayed
//reg	[15:0]	out_bits_d;	// bits to send (delayed)
//reg    [3:0]	out_len_d;		// length of bits to send (4'h0 means 16) (delayed)

reg			flush;

//wire [23:0]	readback;	 // readback data
reg			eob;
wire			gotDC;
wire			gotAC;
wire			gotRLL;
wire			gotEOB;
wire			gotLastBlock;
wire			gotLastWord;
wire			gotColor;

wire           want_read; // as will_read, but w/o fifo status
reg	ready_to_flush;	// read the last data from fifo
 reg             en2x; // en sync to clk;

reg [15:0]	do;
reg [ 3:0]	dl;

wire        pre_dv;
wire [15:0] pre_bits;
wire [ 3:0] pre_len;

reg         twe_d; // table write enable (twe) delayued by 1 clock

 always @ (negedge clk) en2x <= en;
assign	gotDC=	     fifo_o[15] &&  fifo_o[14];
assign	gotAC=	     fifo_o[15] && !fifo_o[14];
assign	gotRLL=	    !fifo_o[15] && !fifo_o[12];
assign	gotEOB=	    !fifo_o[15] &&  fifo_o[12];
assign	gotLastBlock=  fifo_o[15] &&  fifo_o[14] && fifo_o[12];
assign	gotLastWord=  !fifo_o[14] &&  fifo_o[12];	// (AC or RLL) and last bit set
assign	gotColor= fifo_o[13];

    always @(negedge clk) stuffer_was_rdy <= !en2x || rdy; // stuffer ready shoud be on if !en (move to register?)for now]
wire stuffer_was_rdy_early;
wire want_read_early;
  LD  i_stuffer_was_rdy_early (.Q(stuffer_was_rdy_early),.G(clk),.D(!en2x || rdy));


  LD  i_tables_re (.Q(tables_re),.G(clk),.D(en2x && rdy));


    assign read_next= en2x && ((!steps[0] && !rll[5]) || eob ) && fifo_or_full; // fifo will never have data after the last block...
    assign will_read= stuffer_was_rdy && fifo_or_full && en2x && ((!steps[0] && !rll[5]) || eob ); // fifo will never have data after the last block...
    assign want_read= stuffer_was_rdy && ((!steps[0] && !rll[5]) || eob ); // for FIFO
    assign want_read_early= stuffer_was_rdy_early && ((!steps[0] && !rll[5]) || eob ); // for FIFO

  always @ (negedge clk) if (stuffer_was_rdy) begin
      eob <= read_next && gotEOB;// will be 1 only during step[0]
  		if (!en2x) steps[5:0]	<= 'b0;
		else     steps[5:0]	<= {	steps[4] && code_typ4, // will be skipped for codes 00/F0
											steps[3:0],
											(read_next && !(gotRLL && (fifo_o[5:4]==2'b00))) || rll[5] }; // will not start if it was <16, waiting for AC

  end
  always @ (negedge clk)	begin
	   last_block <= en2x && (last_block?(!flush):(stuffer_was_rdy && will_read && gotLastBlock));
	   ready_to_flush <= en2x && (ready_to_flush?(!flush):(stuffer_was_rdy && last_block &&  will_read && gotLastWord));
		test_lbw <= en2x && last_block &&  gotLastWord;
// did not work if flush was just after not ready?
  	   flush	<= en2x &&( flush?(!rdy):(rdy && stuffer_was_rdy && ready_to_flush && !(|steps)) );
  end


  always @ (negedge clk) if (will_read) begin
  	 typeDC						<= gotDC;
  	 typeAC						<= gotAC;
	 sval[11:0]					<= fifo_o[11:0];
	 if (gotDC)  tbsel_YC0	<= gotColor;
  end
  

  always @ (negedge clk) if (stuffer_was_rdy) begin
    if (!en2x || (read_next && gotAC) || (steps[0] && typeAC)) rll[5:4] <= 2'b0;
    else if (read_next && gotRLL)									 rll[5:4] <= fifo_o[5:4];
	 else if (rll[5:4]!=2'b00)										    rll[5:4] <= rll[5:4]-1;

 	 if (!en2x || (read_next && !gotAC && !gotRLL) || (steps[0] && typeAC)) rll[3:0] <= 4'b0;
    else if (read_next && gotRLL)									 rll[3:0] <= fifo_o[3:0];

  end



  assign code_typ0={typeDC || (!eob && (rll[5:4]==2'b0)),
                    typeDC || (!eob && (rll[5:4]!=2'b0))};

  assign haddr_next[7:0] = code_typ2[1]?
                                        (code_typ2[0]?{var_dl[3:0],4'hf}:       // DC (reusing the spare cells of the AC table)
								              {rll2[3:0],var_dl[3:0]}): // AC normal code
								(code_typ2[0]?8'hf0:				//skip 16 zeros code
								              8'h00);				//skip to end of block code

  always @ (negedge clk) if (stuffer_was_rdy && steps[2]) begin	// may be just if (stuffer_was_rdy)
	haddr_r[7:0]	<= haddr_next[7:0];
  end
  LD  i_haddr_7 (.Q(haddr[7]),.G(clk),.D((stuffer_was_rdy && steps[2])?haddr_next[7]:haddr_r[7]));
  LD  i_haddr_6 (.Q(haddr[6]),.G(clk),.D((stuffer_was_rdy && steps[2])?haddr_next[6]:haddr_r[6]));
  LD  i_haddr_5 (.Q(haddr[5]),.G(clk),.D((stuffer_was_rdy && steps[2])?haddr_next[5]:haddr_r[5]));
  LD  i_haddr_4 (.Q(haddr[4]),.G(clk),.D((stuffer_was_rdy && steps[2])?haddr_next[4]:haddr_r[4]));
  LD  i_haddr_3 (.Q(haddr[3]),.G(clk),.D((stuffer_was_rdy && steps[2])?haddr_next[3]:haddr_r[3]));
  LD  i_haddr_2 (.Q(haddr[2]),.G(clk),.D((stuffer_was_rdy && steps[2])?haddr_next[2]:haddr_r[2]));
  LD  i_haddr_1 (.Q(haddr[1]),.G(clk),.D((stuffer_was_rdy && steps[2])?haddr_next[1]:haddr_r[1]));
  LD  i_haddr_0 (.Q(haddr[0]),.G(clk),.D((stuffer_was_rdy && steps[2])?haddr_next[0]:haddr_r[0]));

// 
  assign pre_dv =         steps[4] || (steps[5] && (var_dl_late[3:0]!=4'b0));
  assign pre_bits[15:0]	= steps[5]?{5'b0,var_do[10:0]}:     hcode[15:0];
  assign pre_len [ 3:0]	= steps[5]?      var_dl_late[ 3:0]: hlen  [3:0];

  always @ (negedge clk) if (stuffer_was_rdy) begin
     dv0             <= pre_dv;
     out_bits[15:0]	<= pre_bits[15:0];
     out_len [ 3:0]	<= pre_len [ 3:0];
  end
  always @ (negedge clk) if (!en2x || rdy) begin
    dv       <= stuffer_was_rdy? pre_dv:dv0;
    do[15:0] <= stuffer_was_rdy? pre_bits[15:0]:out_bits[15:0];
    dl[ 3:0] <= stuffer_was_rdy? pre_len [ 3:0]:out_len [ 3:0];

  end



// "Extract shift registers" in synthesis should be off! FD has lower output delay than SRL16
  always @ (negedge clk) if (stuffer_was_rdy) begin
    code_typ1[1:0] <= code_typ0[1:0];
    code_typ2[1:0] <= code_typ1[1:0];
    code_typ3      <= code_typ2[1];
    code_typ4      <= code_typ3;
    rll1[3:0]      <= rll[3:0];
    rll2[3:0]      <= rll1[3:0];
    tbsel_YC1      <= tbsel_YC0;
    tbsel_YC2      <= tbsel_YC1;
    tbsel_YC3      <= tbsel_YC2;
  end
  LD  i_haddr_8 (.Q(haddr[8]),.G(clk),.D(stuffer_was_rdy?tbsel_YC2:tbsel_YC3));
  LD_1 i_hlen3  (.Q( hlen[ 3]),.G(clk),.D(tables_out[19]));  
  LD_1 i_hlen2  (.Q( hlen[ 2]),.G(clk),.D(tables_out[18]));  
  LD_1 i_hlen1  (.Q( hlen[ 1]),.G(clk),.D(tables_out[17]));  
  LD_1 i_hlen0  (.Q( hlen[ 0]),.G(clk),.D(tables_out[16]));  
  LD_1 i_hcode15(.Q(hcode[15]),.G(clk),.D(tables_out[15]));  
  LD_1 i_hcode14(.Q(hcode[14]),.G(clk),.D(tables_out[14]));  
  LD_1 i_hcode13(.Q(hcode[13]),.G(clk),.D(tables_out[13]));  
  LD_1 i_hcode12(.Q(hcode[12]),.G(clk),.D(tables_out[12]));  
  LD_1 i_hcode11(.Q(hcode[11]),.G(clk),.D(tables_out[11]));  
  LD_1 i_hcode10(.Q(hcode[10]),.G(clk),.D(tables_out[10]));  
  LD_1 i_hcode9 (.Q(hcode[ 9]),.G(clk),.D(tables_out[ 9]));  
  LD_1 i_hcode8 (.Q(hcode[ 8]),.G(clk),.D(tables_out[ 8]));  
  LD_1 i_hcode7 (.Q(hcode[ 7]),.G(clk),.D(tables_out[ 7]));  
  LD_1 i_hcode6 (.Q(hcode[ 6]),.G(clk),.D(tables_out[ 6]));  
  LD_1 i_hcode5 (.Q(hcode[ 5]),.G(clk),.D(tables_out[ 5]));  
  LD_1 i_hcode4 (.Q(hcode[ 4]),.G(clk),.D(tables_out[ 4]));  
  LD_1 i_hcode3 (.Q(hcode[ 3]),.G(clk),.D(tables_out[ 3]));  
  LD_1 i_hcode2 (.Q(hcode[ 2]),.G(clk),.D(tables_out[ 2]));  
  LD_1 i_hcode1 (.Q(hcode[ 1]),.G(clk),.D(tables_out[ 1]));  
  LD_1 i_hcode0 (.Q(hcode[ 0]),.G(clk),.D(tables_out[ 0]));  
  huff_fifo i_huff_fifo(.pclk(pclk),
		              .clk(clk),
				    .en(en),					// will reset if ==0 (sync to pclk)
				    .di(di[15:0]),			// data in (sync to pclk)
				    .ds(ds),				// din valid (sync to pclk)
				    .want_read(want_read),
                .want_read_early(want_read_early),
				    .dav(fifo_or_full),	// FIFO output register has data 
				    .q(fifo_o[15:0]));	// output data (will add extra buffering if needed)

    varlen_encode i_varlen_encode(.clk(clk),
										.en(stuffer_was_rdy), //will enable registers. 0 - freese
										.start(steps[0]),
										.d(sval[11:0]),		// 12-bit signed
										.l(var_dl[ 3:0]),		// [3:0] code length
										.l_late(var_dl_late[3:0]),
										.q(var_do[10:0]));	// [10:0]code
   always @ (negedge clk) twe_d <= twe;
   RAMB16_S18_S36 i_htab (
                          .DOA(),           // Port A 16-bit Data Output
                          .DOPA(),          // Port A 2-bit Parity Output
                          .ADDRA({ta[8:0],twe_d}),  // Port A 10-bit Address Input
                          .CLKA(!clk),      // Port A Clock
                          .DIA(tdi[15:0]),  // Port A 16-bit Data Input
                          .DIPA(2'b0),      // Port A 2-bit parity Input
                          .ENA(1'b1),       // Port A RAM Enable Input
                          .SSRA(1'b0),      // Port A Synchronous Set/Reset Input
                          .WEA(twe | twe_d),// Port A Write Enable Input

                          .DOB({unused[11:0],tables_out[19:0]}),      // Port B 32-bit Data Output
                          .DOPB(),          // Port B 4-bit Parity Output
                          .ADDRB(haddr[8:0]),  // Port B 9-bit Address Input
                          .CLKB(clk),       // Port B Clock
                          .DIB(32'b0),      // Port B 32-bit Data Input
                          .DIPB(4'b0),      // Port-B 4-bit parity Input
                          .ENB(tables_re),      // PortB RAM Enable Input
                          .SSRB(1'b0),      // Port B Synchronous Set/Reset Input
                          .WEB(1'b0)        // Port B Write Enable Input
   );
endmodule


//used the other edge of the clk2x
module huff_fifo (pclk,
                  clk,
						en,	// will reset if ==0 (sync to pclk)
						di,   // data in (sync to pclk)
						ds,   // din valid (sync to pclk)
						want_read,
                  want_read_early, 
						dav,	// FIFO output register has data (fifo_or_full)
						q);   // output data

	input			 pclk,clk,en,ds, want_read, want_read_early; //,got;		// will_read;
	input	[15:0] di;
	output		 dav;
	output[15:0] q;
	
	reg      [9:0]	 wa;
	reg	     [9:0]	 sync_wa;	// delayed wa, re-calculated at output clock
	reg 	 [9:0]	 ra_r;
	wire	 [9:0]	 ra;
    wire [15:0]	 q;
    reg          load_q;    // SuppressThisWarning Veditor VDT_BUG
	wire [15:0]	 fifo_o;
	reg			 ds1;	// ds delayed by one pclk to give time to block ram to write data. Not needed likely.
	reg			 synci;
	reg	[1:0]	 synco;
    reg			 sync_we; // single clk period pulse for each ds@pclk
    reg             en2x; // en sync to clk;

	reg			 re_r;
	wire			 re;
	reg             dav; // output latch has data
	reg			 fifo_dav; // RAM output reg has data
	reg             dav_and_fifo_dav;
	wire            ram_dav;  // RAM has data inside
   reg   [9:0]     diff_a;
	wire            next_re;


  always @ (posedge pclk) begin // input stage, no overrun detection
    if (!en)	   wa[9:0] <= 10'b0;
    else if (ds)  wa[9:0] <= wa[9:0]+1;
    ds1	                <= ds && en;
    if (!en)      synci   <= 1'b0;
    else if (ds1) synci   <= ~synci;
   end
  always @ (negedge clk) begin
    en2x <= en;
    synco[1:0]   <= {synco[0],synci};
    sync_we      <= en2x && (synco[0] != synco[1]);
  end

  assign ram_dav= sync_we || (diff_a[9:0] != 10'b0);
//  assign next_re= ram_dav && (!dav || !fifo_dav || want_read);
  assign next_re= ram_dav && (!dav_and_fifo_dav || want_read);
  
  always @ (negedge clk) begin
    dav              <= en2x && (fifo_dav || (dav && !want_read));
    fifo_dav         <= en2x && (ram_dav ||(dav && fifo_dav && !want_read));
	 dav_and_fifo_dav <= en2x && (fifo_dav || (dav && !want_read)) && (ram_dav ||(dav && fifo_dav && !want_read)); // will optimize auto
    re_r    <= en2x &&  next_re;
    if (!en2x)			 sync_wa[9:0] <= 10'b0;
	 else if (sync_we) sync_wa[9:0] <= sync_wa[9:0]+1;
    if        (!en2x)               ra_r  [9:0] <= 10'b0;
	 else if (next_re)             ra_r  [9:0] <= ra_r[9:0]+1;
    if (!en2x)                      diff_a[9:0] <= 10'b0;
      else if (sync_we && !next_re) diff_a[9:0] <= diff_a[9:0]+1;
	   else if (!sync_we && next_re) diff_a[9:0] <= diff_a[9:0]-1; 



  end
  LD i_re  (.Q(re),.G(clk),.D(next_re));  

  LD i_ra9 (.Q(ra[9]),.G(clk),.D(ra_r[9]));  
  LD i_ra8 (.Q(ra[8]),.G(clk),.D(ra_r[8]));  
  LD i_ra7 (.Q(ra[7]),.G(clk),.D(ra_r[7]));  
  LD i_ra6 (.Q(ra[6]),.G(clk),.D(ra_r[6]));  
  LD i_ra5 (.Q(ra[5]),.G(clk),.D(ra_r[5]));  
  LD i_ra4 (.Q(ra[4]),.G(clk),.D(ra_r[4]));  
  LD i_ra3 (.Q(ra[3]),.G(clk),.D(ra_r[3]));  
  LD i_ra2 (.Q(ra[2]),.G(clk),.D(ra_r[2]));  
  LD i_ra1 (.Q(ra[1]),.G(clk),.D(ra_r[1]));  
  LD i_ra0 (.Q(ra[0]),.G(clk),.D(ra_r[0]));  
  always @ (posedge clk) begin
    load_q <= dav?want_read_early:re_r;
  end
  LD_1 i_q15 (.Q( q[15]),.G(clk),.D(load_q?fifo_o[15]:q[15]));  
  LD_1 i_q14 (.Q( q[14]),.G(clk),.D(load_q?fifo_o[14]:q[14]));  
  LD_1 i_q13 (.Q( q[13]),.G(clk),.D(load_q?fifo_o[13]:q[13]));  
  LD_1 i_q12 (.Q( q[12]),.G(clk),.D(load_q?fifo_o[12]:q[12]));  
  LD_1 i_q11 (.Q( q[11]),.G(clk),.D(load_q?fifo_o[11]:q[11]));  
  LD_1 i_q10 (.Q( q[10]),.G(clk),.D(load_q?fifo_o[10]:q[10]));  
  LD_1 i_q9  (.Q( q[ 9]),.G(clk),.D(load_q?fifo_o[ 9]:q[ 9]));  
  LD_1 i_q8  (.Q( q[ 8]),.G(clk),.D(load_q?fifo_o[ 8]:q[ 8]));  
  LD_1 i_q7  (.Q( q[ 7]),.G(clk),.D(load_q?fifo_o[ 7]:q[ 7]));  
  LD_1 i_q6  (.Q( q[ 6]),.G(clk),.D(load_q?fifo_o[ 6]:q[ 6]));  
  LD_1 i_q5  (.Q( q[ 5]),.G(clk),.D(load_q?fifo_o[ 5]:q[ 5]));  
  LD_1 i_q4  (.Q( q[ 4]),.G(clk),.D(load_q?fifo_o[ 4]:q[ 4]));  
  LD_1 i_q3  (.Q( q[ 3]),.G(clk),.D(load_q?fifo_o[ 3]:q[ 3]));  
  LD_1 i_q2  (.Q( q[ 2]),.G(clk),.D(load_q?fifo_o[ 2]:q[ 2]));  
  LD_1 i_q1  (.Q( q[ 1]),.G(clk),.D(load_q?fifo_o[ 1]:q[ 1]));  
  LD_1 i_q0  (.Q( q[ 0]),.G(clk),.D(load_q?fifo_o[ 0]:q[ 0]));  

/*
  RAMB4_S16_S16 i_fifo   (.DOB(fifo_o[15:0]),
 				 	 .ADDRA(wa[7:0]),
				 	 .CLKA(pclk),
				 	 .DIA(di[15:0]),
				 	 .ENA(ds),.RSTA(1'b0),.WEA(1'b1),
	   			 	 .ADDRB(ra[7:0]),
				 	 .CLKB(clk),
				 	 .DIB(16'b0),.ENB(re),.RSTB(1'b0),.WEB(1'b0));
*/
   RAMB16_S18_S18 i_fifo (
                          .DOA(),            // Port A 16-bit Data Output
                          .DOPA(),           // Port A 2-bit Parity Output
                          .ADDRA(wa[9:0]),   // Port A 10-bit Address Input
                          .CLKA(pclk),       // Port A Clock
                          .DIA(di[15:0]),    // Port A 16-bit Data Input
                          .DIPA(2'b0),       // Port A 2-bit parity Input
                          .ENA(ds),          // Port A RAM Enable Input
                          .SSRA(1'b0),       // Port A Synchronous Set/Reset Input
                          .WEA(1'b1),        // Port A Write Enable Input

                          .DOB(fifo_o[15:0]),// Port B 16-bit Data Output
                          .DOPB(),           // Port B 2-bit Parity Output
                          .ADDRB(ra[9:0]),   // Port B 10-bit Address Input
                          .CLKB(clk),        // Port B Clock
                          .DIB(16'b0),       // Port B 16-bit Data Input
                          .DIPB(2'b0),       // Port-B 2-bit parity Input
                          .ENB(re),          // PortB RAM Enable Input
                          .SSRB(1'b0),       // Port B Synchronous Set/Reset Input
                          .WEB(1'b0)         // Port B Write Enable Input
                          );

 
endmodule


// Encoder will work 2 cycles/"normal" word, 1 cycle for codes "00" and "f0",
// only magnitude output is needed ASAP (2 cycles, the value out should be
// valid on the 5-th cycle - it will latency 4 cycles run each other cycle
// I'll make a shortcut - all codes processed in 2 cycles.

module	varlen_encode (clk,
								en,	// will enable registers. 0 - "freese" at once
								start, // (not faster than each other cycle)
								d,		// 12-bit signed
								l,		// [3:0] code length
								l_late,// delayed l (sync to q)
								q);	// [10:0]code
  input				clk, en,start;
  input	[11:0]	d;
  output	[ 3:0]	l;
  output	[ 3:0]	l_late;
  output	[10:0]	q;
  reg		[11:0]	d1;
  reg		[10:0]	q,q0;
  reg		[ 3:0]	l,l_late;
  reg		[2:0]		cycles;

  wire			this0=  |d1[ 3:0];
  wire			this1=  |d1[ 7:4];
  wire			this2=  |d1[10:8];
  wire	[1:0]	codel0={|d1[ 3: 2],d1[ 3] || (d1[ 1] & ~d1[ 2])};
  wire	[1:0]	codel1={|d1[ 7: 6],d1[ 7] || (d1[ 5] & ~d1[ 6])};
  wire	[1:0]	codel2={|d1[   10],          (d1[ 9] & ~d1[10])};
  wire	[3:0] codel= this2? {2'b10,codel2[1:0]} :
                     (this1? {2'b01,codel1[1:0]} :
							(this0? {2'b00,codel0[1:0]} : 4'b1111));	// after +1 will be 0;

  always @ (negedge clk)  if (en) begin
  		cycles[2:0]	<= {cycles[1:0],start};
  end

  always @ (negedge clk) if (en && start) begin
 		d1[  11]	<=  d[11];
 		d1[10:0]	<=  d[11]?-d[10:0]:d[10:0];
  end

  always @ (negedge clk) if (en & cycles[0]) begin
		q0[10:0]	<= d1[11]?~d1[10:0]:d1[10:0];
		l	<= codel[3:0]+1;	// needed only ASAP, valid only 2 cycles after start
  end
  always @ (negedge clk) if (en & cycles[2]) begin
     q[10:0]	<= q0[10:0];
	  l_late[3:0]	<= l[3:0];
  end

endmodule
