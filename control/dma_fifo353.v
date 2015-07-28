/*
** -----------------------------------------------------------------------------**
** dma_fifosync353.v
**
** DMA controller/fifo
**
** Copyright (C) 2002-2007 Elphel, Inc.
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
// with long 8-cycle bursts it makes sense to move everything to internal clock domain (no need fro Gray counters)
// uses GCLK buffer (so 2 for 2 DMA channels)
module dma_fifo_sync (	clk,		// system clock, 120MHz? (currentle negedge used)
						pre_wen,	// decoded addresses (valid @ negedge clk)
						wd,		      // {enable update, pio_mode, enable channel}

						dreq,		// DREQ output to CPU
						dack, 	// DACK input from CPU
//                  re,      // read enable (CE | OE)
                  oe,      // just OE
                  pioinc,  // increment buffer address in PIO mode
						d,			// [31:0]	- data to cpu
                  we,      // write enable (sync to posedge clk)
                  di,      // 16-bit data to write (sync to posedge clk)
						enabled, // channel enabled
						real_empty
                  ,test1,
                  test2
                  );	// pessimistic, with several clk delay

    input			clk;
    input			pre_wen;
    input 	[2:0] wd; // bit2 - enable update of bits [1:0], bit 0 - enable channel, bit 1 - PIO mode
    output			dreq;
    input			dack;
//    input         re;
    input         oe;
    input         pioinc;
    output [31:0]	d;
    input         we;
    input  [15:0] di;
	 output			enabled;
	 output			real_empty;

    output [7:0] test1;
    output [7:0] test2;

	 reg           wen;
    wire          en;
    reg    [9:0]  wab;      // binary write counter
    reg    [8:0]  rab;      // binary read counter

    wire          rst;
    reg    [3:0]  empties;
    wire          en0;
    wire          pio, pio0; //enable pio read mode
    
    reg    [2:0]  di_d;
    assign rst=!en;
	 assign enabled=en;
    
	 assign real_empty=empties[3];
    assign test1=rab[7:0];
    assign test2=wab[7:0];

    wire   swclk; // global clock, switched between OE (when DACK active) and extra pulse generated to read first word from FIFO.
    reg    firstclk; // generated 1-clk_long pulse as a source for swclk
    reg    written_burst; // 16 of 16-bit words are just written to FIFO
    wire   mem_re;        // memory RE signal (alo increments counters)
//AF2015: Could not find any iob=true for this register, but it is still duplicated
// OK, it has only sync inputs, so no risk to have different values here and in the IOB
//FlipFlop i_dma_fifo0/dreq has been replicated 1 time(s) to handle iob=true attribute.
//FlipFlop i_dma_fifo1/dreq has been replicated 1 time(s) to handle iob=true attribute.
    reg    dreq;
    reg    [1:0] burst_start_sync;  // 1 clk long pulse (clk-sync) after start of the DMA burst
    reg    [1:0] dack_r; // synchronize to clk, dual to prevent metastability
    reg    [9:0] infifo;    // number of words in FIFO (10 bits, to implement overflow)
    reg          first_four; // sync to OE
    reg    [2:0] first_four_r; // sync to clk
    reg          nempty; // registered fifo_not_empty 
    reg          lessthan2; // registered fifo has less than 2 words
    assign mem_re= dreq || !dack_r[1] || (rab[2:0]!=3'b0);


    always @ (negedge clk) begin
      wen <= pre_wen;
      di_d[2:0] <= wd[2:0];
    end
 	 FDE_1 i_en0       (.C(clk), .D(di_d[0]), .CE(wen & di_d[2]), .Q(en0));
 	 FD    i_en_tig_   (.C(clk), .D(en0),          .Q(en));
 	 FDE_1 i_pio0      (.C(clk), .D(di_d[1]), .CE(wen & di_d[2]), .Q(pio0));
 	 FD    i_pio_tig_  (.C(clk), .D(pio0),            .Q(pio));
    BUFG i_swclk (.O(swclk),.I(dack?oe:(!firstclk && !(pio && pioinc))));


    always @ (posedge clk) begin
      if      (rst) wab[9:0] <= 10'b0;
      else if ( we) wab[9:0] <= wab[9:0] + 1;
      written_burst <= we && (wab[3:0]==4'hf);
      nempty    <= (infifo[9:0]!=10'h0);
      lessthan2 <= (infifo[9:1]== 9'h0);
      if (!en) infifo[9:0] <= 10'h0;
      else if ( written_burst && !burst_start_sync[0]) infifo[9:0] <= infifo[9:0]+1;
      else if (!written_burst &&  burst_start_sync[0]) infifo[9:0] <= infifo[9:0]-1;
      dack_r[1:0] <= {dack_r[0], dack};

      if (rst || firstclk) firstclk <= 1'b0;
//      else if (!pio && written_burst && !dreq && !dack_r[1]) firstclk <= 1'b1; //dreq & dack_r1[1] overlap
// don't need to add &&!burst_start_sync[1] as burst_start_sync[1] overlaps with dack_r[1]
      else if (!pio && nempty && !dreq && !dack_r[1]) firstclk <= 1'b1; //dreq & dack_r1[1] overlap

//      if (rst || ((infifo[9:0]==10'h0) && burst_start_sync[1])) dreq <= 1'b0;
// changed to faster (registered) version. burst_start_sync[1] happens just first cycle after infifo[9:0] was decremented
// so we need that a cycle it was >1, not !=0. We can miss increment count (written_burst), but don't bother
// adding condition - not a great loss.
      if (rst || (lessthan2 && burst_start_sync[1])) dreq <= 1'b0;
      else if (firstclk)                             dreq <= 1'b1;
      
      burst_start_sync[1:0] <= {burst_start_sync[0],~first_four_r[2] & first_four_r[1]};

      first_four_r[2:0]     <= {first_four_r[1:0],first_four};
      
      
	   empties[3:0]	<=	{4{!rst && (infifo[9:0]==10'h0) && !dreq && !dack_r[1]}} & {empties[2:0],1'b1};
    end

    always @ (posedge rst or posedge swclk) begin
      if (rst) rab[8:0] <= 9'h0;
      else if (mem_re) rab[8:0] <= rab[8:0] + 1;
      if (rst) first_four <= 1'b0;
      else if (rab[1:0]==2'h1) first_four <= ~rab[2];
    end



   RAMB16_S18_S36 i_dmafifobuff  (
      .DOA(),              // Port A 16-bit Data Output - FPN (sensor) side
      .DOPA(),             // Port A 2-bit Parity Output
      .ADDRA(wab[9:0]),    // Port A 10-bit Address Input
//      .CLKA(!clk),         // Port A Clock
      .CLKA(clk),          // Port A Clock
      .DIA(di[15:0]),      // Port A 16-bit Data Input
      .DIPA(2'b0),         // Port A 2-bit parity Input
      .ENA(we),            // Port A RAM Enable Input
      .SSRA(1'b0),         // Port A Synchronous Set/Reset Input
      .WEA(1'b1),          // Port A Write Enable Input

      .DOB(d),             // Port B 32-bit Data Output - SDRAM side
      .DOPB(),             // Port B 4-bit Parity Output
      .ADDRB(rab[8:0]),    // Port B 9-bit Address Input
      .CLKB(swclk),        // Port B Clock
      .DIB(32'b0),         // Port B 32-bit Data Input
      .DIPB(4'b0),         // Port-B 4-bit parity Input
      .ENB(mem_re),        // PortB RAM Enable Input
      .SSRB(1'b0),         // Port B Synchronous Set/Reset Input
      .WEB(1'b0)           // Port B Write Enable Input
   );
endmodule


