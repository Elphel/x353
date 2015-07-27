/*
** -----------------------------------------------------------------------------**
** mcontr353.v
**
** 4 channel block access SDRAM controller
**
** Copyright (C) 2002-2010 Elphel, Inc.
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

/*
SDRAM controller supports 4 channels to access memory in parallel. Data is transferred in pages 256x16 (or 512x8)- for sequentional access
(i.e. written by sensor) or in square (for 8bit mode) tiles of 18x18x8bits. This mode is needed by JPEG
encoder - each 16x16 MCU will be bayer-decoded to 4 8x8 Y (intensity) blocks, one 8x8 Cr and one 8x8 Cb (4:2:0).
Each scan line is aligned to 128x16-bit page boundary.
SDRAM chip itself is 16-bit wide (16Mx16), data channels have different widths. system is little endian, so LSB has lower address
Three of data channels have predefined transfer direction and the control registers should be written accordingly,
the last one is bidirectional for PIO acces to SDRAM from CPU.
Channel 0 provides data from the sensor (it can be combined with FPN correction data) to the SDRAM in either 8-bit mode or 16-bit.
If the data is to be JPEG compressed it should be in 8-bit mode.
Channel 1 reads FPN data from SDRAM (8bit - subtract, 8 bit - scale) and feeds it to FPGA for FPN correction of the data on the way
from sensor to SDRAM (through channel 0)
Channel 2 provides data to JPEG encoder in 16x16x8bit MCUs. It can also provide data dirsectly to CPU through DMA bypassing any compression
Channel 3 provides bidirectional access from CPU through 8x32 bit window. To read/write more there is a special command to proceed to next
page and a status bit to test if data is ready. After writing data it is needed to go through the whole page (or just issue "next page") command
as the data transfer to CPU takes place only when the whole page is filled (total of 8 8x32 subpages). There is also a special status bit
to determine if the data is actually written to sdram and the channel may be reprogrammed (i.e. for reading) 

*/
module mcontr(
               clk0,     // SDRAM clocks (hope to get 120MHz)
               restart_en,  // enable restarting selected channels
               restart,     // reinitialize channels (posedge-sensitive, masked by enXfer0

               bonded,       //[3:0] - channel bonded with the other (just for channel 2), make it TIG
               nextBlocksEn, // enable read blocks to FIFO. disabled when init or roll over, enable by confirmRead

//               mclk,    // modified CPU global clock - write and read with a7==1 (for block memory R/W)
//descriptor memory and channel 3
               ia,       // internal 4-bit address bus (fast - directly from I/O pads)
               as,       // 4 bit address to select descriptor address and data word (3 bits)
               am,       // switching between ia (async read) and as (sync write)
               dscs,      // WE for descriptor memory
               mwnr,      // CPU write, not read (valid for 2 cycles at negedge mclk)
               init_ch3,   // write to init cnhannel 3 (will reset address r/w)
               next_ch3,   // advance to the next channel3 page, reset address
               readNextFrame3, //enable reading to the SDRAM buffer (after channel init or frame over), set to 1'b1 if not needed
               menrw,   // enable CPU read/write  32-bit word from buffer(valid @ mweoe)
               di,      // 32-bit data from CPU (descriptor and channel 3)
               dsdo,      // [31:0] data out from descriptor memory
               do,      // 32-bit data to CPU (channel 3)
               rdy,      // ready to r/w next 128x32 page (channel 3)
               wrempty, // write buffer empty (valid only in write mode).

                 ch0clk,// clock for channel0
               ch0we,   // channel 0 (sensor->SDRAM) WE
               ch0a,      // [7:0] channel 0 address (MSB - block #)
               ch0di,   // [15:0] channel 0 data in
               stch0,   // start channel 0 transfer (up to 2 w/o confirmation)
               ch0rdy,   // channel 0 ready
               readNextFrame0, //enable reading to the SDRAM buffer (after channel init or frame over), set to 1'b1 if not needed

                 ch1clk,// clock for channel1
               ch1a,      // [7:0] channel 1 (SDRAM -> sensor calibration data) address (MSB - block #)
               ch1do,   // [15:0] channel 1 data out
               stch1,   // start channel 1 transfer (up to 2 w/o confirmation)
               ch1rdy,   // channel 1 ready
               readNextFrame1, //enable reading to the SDRAM buffer (after channel init or frame over), set to 1'b1 if not needed

                 ch2clk,// clock for channel2 
               ch2a,      // [10:0] channel 2 (SDRAM -> compressor MCU) address (MSB - block #)
               ch2do,   // [7:0] channel 2 data out
               stch2,   // start channel 2 transfer (up to 2 w/o confirmation)
               ench2,   // enable read from channel 2 buffer to compressor
               ch2rdy,   // channel 2 ready
               readNextFrame2, //enable reading to the SDRAM buffer (after channel init or frame over), set to 1'b1 if not needed
               nextFrame,// (level) generated before actual block is filled - needed to be combined with the pulse from buffer control
               chInitOnehot, //[3:0] decoded channel init pulses, 2 cycles behind chInit
               
//SDRAM intefrace
               sddi,      // [15:0] data from SDRAM  (1 cycle delayed)
               sddo,      // [15:0] data to SDRAM      (1 cycle ahead)
               sda,      // [12:0] address to SDRAM (1 cycle ahead)
               sdba,      // [ 1:0] bank address to SDRAM (1 cycle ahead)
               sdwe,      // WE  command bit to SDRAM (1 cycle ahead)
               sdras,   // RAS command bit to SDRAM (1 cycle ahead)
               sdcas,   // CAS command bit to SDRAM (1 cycle ahead)
               trist,   // tristate data to SDRAM   (2 cycles ahead)
               predqt,   // tristate DQ outputs (one extra for FF in output buffer)
               dmask,     // [1:0] - now both the same (even number of words written)
               dqs_re   // enable read from DQS i/o-s for phase adjustments  (latency 2 from the SDRAM RD command)
               );

    input            clk0;
    input            restart_en;  // enable restarting selected channels
    input   [ 3:0]   restart;     // reinitialize channels (posedge-sensitive, masked by enXfer0

    output  [ 3:0]   bonded;      //[3:0] - channel bonded with the other (just for channel 2), make it TIG
    output  [ 3:0]   nextBlocksEn; // enable read blocks to FIFO. disabled when init or roll over, enable by confirmRead


    input   [ 3:0]   ia;
    input   [ 3:0]   as;
    input   [ 3:0]   am;
    input            dscs;
    input            mwnr;
    input            init_ch3;
    input            next_ch3;
    input            readNextFrame3; //enable reading to the SDRAM buffer (after channel init or frame over), set to 1'b1 if not needed

    input            menrw;
    input   [31:0]   di;
    output   [31:0]   dsdo;
    output   [31:0]   do;
    output            rdy;
    output            wrempty;


      input            ch0clk;
    input            ch0we;
    input   [ 9:0]    ch0a;
    input   [15:0]   ch0di;
    input            stch0;
    output            ch0rdy;
    input            readNextFrame0; //enable reading to the SDRAM buffer (after channel init or frame over), set to 1'b1 if not needed
    input            ch1clk;
    input   [ 9:0]   ch1a;
    output   [15:0]   ch1do;
    input            stch1;
    output            ch1rdy;
    input            readNextFrame1; //enable reading to the SDRAM buffer (after channel init or frame over), set to 1'b1 if not needed

    input            ch2clk;
    input   [10:0]   ch2a;
    output   [ 7:0]   ch2do;
    input            stch2;
    input            ench2;
    output            ch2rdy;
    input            readNextFrame2; //enable reading to the SDRAM buffer (after channel init or frame over), set to 1'b1 if not needed
    output [ 3:0]   nextFrame;
    output [ 3:0]   chInitOnehot;// decoded channel init pulses, 2 cycles behind chInit
    input   [31:0]   sddi;
    output   [31:0]   sddo;
    output   [12:0]   sda;
    output   [ 1:0]   sdba;
    output            sdwe;
    output            sdras;
    output            sdcas;
    output            trist;
    output            predqt;   // enable DQ outputs (one extra for FF in output buffer)
    output  [ 1:0]   dmask;     // [1:0] - now both the same (even number of words written)
    output            dqs_re;    // enable read from DQS i/o-s for phase adjustments  (latency 2 from the SDRAM RD command)

// photofinish hack
    wire    [24:8]   sfa;     // start address of a frame - ugly hack to roll over the last (extra) 4 lines of a frame
                                       // to the beginning of a frame - used in photofinish mode.
    wire               rovr;           // roll over mode (photofinish, cntrl0[12]==1
    wire    [ 3:0]   nextBlocksEn; // enable read blocks to FIFO. disabled when init or roll over, enable by confirmRead

//  block RAM modules
// channel 0: 256x16in, 256x16 out
    wire   [31:0]   ch0rd;   // read data from ch0 buffer
    wire   [31:0]   ch3rd;   // read data from ch3 buffer (CPU->SDRAM)
    wire   [ 8:0]   bmad0;   // buffer memory address channel 0 (SDRAM side)
    wire   [ 8:0]   bmad1;   // buffer memory address channel 1 (SDRAM side)
    wire   [ 8:0]   bmad2;   // buffer memory address channel 2 (SDRAM side)
    wire   [ 8:0]   bmad3;   // buffer memory address channel 3 (SDRAM side)
    wire            ch0en;   // channel 0 buffer EN (buffer->SDRAM)
    wire            ch1we;   // channel 1 buffer WE (SDRAM->buffer)
    wire            ch2we;   // channel 2 buffer WE (SDRAM->buffer)
    wire            ch3owe;   // channel 3 buffer WE (SDRAM->buffer)
    wire            ch3en;   // channel 3 buffer EN (SDRAM<->buffer)


    wire            wrempty;   // channel3 ch3Access->output
    wire            rdy;

    wire [4:0] curChan;      //  1-hot servicing channel
    wire        ch_drun_rd;    // enable read data  (SDRAM->block RAM buffer)
    wire        ch_drun_wr;    // enable write data (SDRAM<-block RAM buffer)
    wire        ch_dlast;   //last cycle of drun active for read, 1 cycle later for write
    wire        ch_prefirstdrun; // 1 cycle ahead of the first drun_rd/drun_wr to be used by bufCntr256 to sample channel enable
    reg  [3:0]  curChanLate; // will be valid at the ch_prefirstdrun

// SDRAM side of data buffers control


    wire        chSt1;      // chArbit      -> descript.
    wire        chSt2;      // descript      -> sdseq
    wire        chInit;      // chArbit       -> descript.
    wire [ 1:0] chNum;      // chArbit       -> descript.
//    wire [23:3] startAddr;   // descript.    -> sdseq
    wire [24:3] startAddr;   // descript.    -> sdseq
    wire        mode;      // descript.    -> sdseq
    wire        WnR;       // descript.    -> sdseq and arbit.? and main?
    wire [ 5:0] param;     // descript.    -> sdseq
    wire [ 1:0] nBuf;      // descript.    -> bufCntr
    wire [17:0] mancmd;    // descript.    -> sdseq
    wire        enSDRAM;   // descript.    -> sdseq +??
    wire        disSDRAM=!enSDRAM;   // descript.    -> sdseq +??
    wire        enRefresh; // descript.    -> refreshRequest)
    wire [ 3:0] chReqInit; // descript.    -> channelRequest(i)
    wire [ 3:0] nextFrame; // descript.    -> out



    wire [ 3:0] enXfer;    // descript.    -> channelRequest<i>
    wire [ 3:0] dnch;      // bufCntr<i>   ->  channelRequest<i>

    wire [ 4:0] chnReq;    // request SDRAM access from channel ([4] - refresh) +++ channelRequest<i>, refreshRequest -> chArbit
    wire [ 3:0] chnReqInit;// request Initialization of tile number for channel +++ channelRequest<i> -> chArbit
    wire [ 3:0] chnAckn;   // chArbit   -> channelRequest<i>

    wire        refrStart;   // chArbit   -> sdseq

// Now the I/O buffers. Will use just 2-to-1 MUXes for data out to SDRAM (channel 0 and channel3)
    reg  [31:0] sddo_reg;
    wire        nextReq;      // sdseq      -> chArbit
    wire [ 1:0] dsel;
    reg  [ 1:0] ch3page;
    reg  [ 6:0] ch3maddr;

   always @ (negedge clk0)sddo_reg[31:0] <= dsel[1]? ch3rd[31:0]:ch0rd[31:0];
   assign sddo[31:0]=   sddo_reg[31:0];

// generate single-cycle pulses after leading edges of dcc-related strobes
//   posoneshot i_dccstb(dccvld,clk0,dccstb);
//   posoneshot i_finish_dcc_pulse(finish_dcc,clk0,finish_dcc_pulse);
   RAMB16_S18_S36 i_chan0buf  (
      .DOA(),              // Port A 16-bit Data Output    - sensor side
      .DOB(ch0rd[31:0]),   // Port B 32-bit Data Output - SDRAM side
      .DOPA(),             // Port A 2-bit Parity Output
      .DOPB(),             // Port B 4-bit Parity Output
      .ADDRA(ch0a[9:0]),   // Port A 10-bit Address Input
      .ADDRB(bmad0[8:0]),  // Port B 9-bit Address Input
      .CLKA(ch0clk),       // Port A Clock
      .CLKB(!clk0),        // Port B Clock
      .DIA(ch0di[15:0]),   // Port A 16-bit Data Input
      .DIB(32'b0),         // Port B 32-bit Data Input
      .DIPA(2'b0),         // Port A 2-bit parity Input
      .DIPB(4'b0),         // Port-B 4-bit parity Input
      .ENA(ch0we),         // Port A RAM Enable Input
      .ENB(ch0en),         // PortB RAM Enable Input
      .SSRA(1'b0),         // Port A Synchronous Set/Reset Input
      .SSRB(1'b0),         // Port B Synchronous Set/Reset Input
      .WEA(1'b1),          // Port A Write Enable Input
      .WEB(1'b0)           // Port B Write Enable Input
   );
/*
   defparam i_chan0buf.INIT_A = 18'h0; // Value of output RAM registers on Port A at startup
   defparam i_chan0buf.INIT_B = 18'h0; // Value of output RAM registers on Port B at startup
   defparam i_chan0buf.SRVAL_A = 18'h0; // Port A ouput value upon SSR assertion
   defparam i_chan0buf.SRVAL_B = 18'h0; // Port B ouput value upon SSR assertion
   defparam i_chan0buf.WRITE_MODE_A = "WRITE_FIRST"; // WRITE_FIRST, READ_FIRST or NO_CHANGE
   defparam i_chan0buf.WRITE_MODE_B = "WRITE_FIRST"; // WRITE_FIRST, READ_FIRST or NO_CHANGE
*/
   RAMB16_S18_S36 i_chan1buf  (
      .DOA(ch1do[15:0]),   // Port A 16-bit Data Output - FPN (sensor) side
      .DOB(),              // Port B 32-bit Data Output - SDRAM side
      .DOPA(),             // Port A 2-bit Parity Output
      .DOPB(),             // Port B 4-bit Parity Output
      .ADDRA(ch1a[9:0]),   // Port A 10-bit Address Input
      .ADDRB(bmad1[8:0]),  // Port B 9-bit Address Input
      .CLKA(ch1clk),       // Port A Clock
      .CLKB(!clk0),        // Port B Clock
      .DIA(16'b0),         // Port A 16-bit Data Input
      .DIB(sddi[31:0]),    // Port B 32-bit Data Input
      .DIPA(2'b0),         // Port A 2-bit parity Input
      .DIPB(4'b0),         // Port-B 4-bit parity Input
      .ENA(1'b1),          // Port A RAM Enable Input
      .ENB(ch1we),         // PortB RAM Enable Input
      .SSRA(1'b0),         // Port A Synchronous Set/Reset Input
      .SSRB(1'b0),         // Port B Synchronous Set/Reset Input
      .WEA(1'b0),          // Port A Write Enable Input
      .WEB(1'b1)           // Port B Write Enable Input
   );
/*
   defparam i_chan1buf.INIT_A = 18'h0; // Value of output RAM registers on Port A at startup
   defparam i_chan1buf.INIT_B = 18'h0; // Value of output RAM registers on Port B at startup
   defparam i_chan1buf.SRVAL_A = 18'h0; // Port A ouput value upon SSR assertion
   defparam i_chan1buf.SRVAL_B = 18'h0; // Port B ouput value upon SSR assertion
   defparam i_chan1buf.WRITE_MODE_A = "WRITE_FIRST"; // WRITE_FIRST, READ_FIRST or NO_CHANGE
   defparam i_chan1buf.WRITE_MODE_B = "WRITE_FIRST"; // WRITE_FIRST, READ_FIRST or NO_CHANGE
*/
   RAMB16_S9_S36 i_chan2buf (
      .DOA(ch2do[ 7:0]),   // Port A 8-bit Data Output   - compressor side
      .DOB(),              // Port B 32-bit Data Output  - SDRAM side
      .DOPA(),             // Port A 1-bit Parity Output
      .DOPB(),             // Port B 4-bit Parity Output
      .ADDRA(ch2a[10:0]),  // Port A 11-bit Address Input
      .ADDRB(bmad2[8:0]),  // Port B 9-bit Address Input
      .CLKA(ch2clk),       // Port A Clock
      .CLKB(!clk0),        // Port B Clock
      .DIA(8'b0),          // Port A 8-bit Data Input
      .DIB(sddi[31:0]),    // Port B 32-bit Data Input
      .DIPA(1'b0),         // Port A 1-bit parity Input
      .DIPB(4'b0),         // Port-B 4-bit parity Input
      .ENA(ench2),         // Port A RAM Enable Input
      .ENB(ch2we),         // PortB RAM Enable Input
      .SSRA(1'b0),         // Port A Synchronous Set/Reset Input
      .SSRB(1'b0),         // Port B Synchronous Set/Reset Input
      .WEA(1'b0),          // Port A Write Enable Input
      .WEB(1'b1)           // Port B Write Enable Input
   );
/*
   defparam i_chan2buf.INIT_A = 18'h0; // Value of output RAM registers on Port A at startup
   defparam i_chan2buf.INIT_B = 18'h0; // Value of output RAM registers on Port B at startup
   defparam i_chan2buf.SRVAL_A = 18'h0; // Port A ouput value upon SSR assertion
   defparam i_chan2buf.SRVAL_B = 18'h0; // Port B ouput value upon SSR assertion
   defparam i_chan2buf.WRITE_MODE_A = "WRITE_FIRST"; // WRITE_FIRST, READ_FIRST or NO_CHANGE
   defparam i_chan2buf.WRITE_MODE_B = "WRITE_FIRST"; // WRITE_FIRST, READ_FIRST or NO_CHANGE
*/
// as it is 32-bit wide it will go directly to CPU interface, not intemediate 32/16 buffer
// read will have latency of 1 
   RAMB16_S36_S36 i_chan3buf (
      .DOA(do[31:0]),      // Port A 32-bit Data Output    - CPU side
      .DOB(ch3rd[31:0]),   // Port B 32-bit Data Output    - SDRAM side
      .DOPA(),             // Port A 4-bit Parity Output
      .DOPB(),             // Port B 4-bit Parity Output
//      .ADDRA({ch3page[1:0],ma[6:0]}),  // Port A 9-bit Address Input
      .ADDRA({ch3page[1:0],ch3maddr[6:0]}),  // Port A 9-bit Address Input
      .ADDRB(bmad3[8:0]),  // Port B 9-bit Address Input
//      .CLKA(mclk),         // Port A Clock
      .CLKA(!clk0),         // Port A Clock
      .CLKB(!clk0),        // Port B Clock
      .DIA(di[31:0]),      // Port A 32-bit Data Input
//      .DIA(d_late[31:0]),      // Port A 32-bit Data Input
      .DIB(sddi[31:0]),    // Port B 32-bit Data Input
      .DIPA(4'b0),         // Port A 4-bit parity Input
      .DIPB(4'b0),         // Port-B 4-bit parity Input
      .ENA(menrw),         // Port A RAM Enable Input
      .ENB(ch3en),         // PortB RAM Enable Input
      .SSRA(1'b0),         // Port A Synchronous Set/Reset Input
      .SSRB(1'b0),         // Port B Synchronous Set/Reset Input
      .WEA(mwnr),          // Port A Write Enable Input
      .WEB(ch3owe)         // Port B Write Enable Input
   );
  // now bmad addresses 32-bit words

  always @ (negedge clk0)  if (chSt2) curChanLate[3:0] <= curChan[3:0];

  bufCntr256   i_bufCntr0 (.clk(clk0), .rst(!enSDRAM), .cs(curChanLate[0]), .init(ch_prefirstdrun), .bank(nBuf[1:0]),
                           .drun_rd(1'b0), .drun_wr(ch_drun_wr), .dlast(ch_dlast),
                             .a(bmad0[8:0]),  .en(ch0en),              .done(dnch[0]),.we());
  bufCntr256   i_bufCntr1 (.clk(clk0), .rst(!enSDRAM), .cs(curChanLate[1]), .init(ch_prefirstdrun), .bank(nBuf[1:0]),
                           .drun_rd(ch_drun_rd), .drun_wr(1'b0), .dlast(ch_dlast),
                             .a(bmad1[8:0]), .we(ch1we),  .done(dnch[1]), .en());
  bufCntr256   i_bufCntr2 (.clk(clk0), .rst(!enSDRAM), .cs(curChanLate[2]), .init(ch_prefirstdrun), .bank(nBuf[1:0]),
                           .drun_rd(ch_drun_rd), .drun_wr(1'b0), .dlast(ch_dlast),
                             .a(bmad2[8:0]), .we(ch2we),  .done(dnch[2]), .en());
  bufCntr256   i_bufCntr3 (.clk(clk0), .rst(!enSDRAM), .cs(curChanLate[3]), .init(ch_prefirstdrun), .bank(nBuf[1:0]),
                           .drun_rd(ch_drun_rd), .drun_wr(ch_drun_wr), .dlast(ch_dlast),
                           .a(bmad3[8:0]), .en (ch3en), .we(ch3owe), .done(dnch[3])
                      );
// synchronizing readNextFrame[3:0] (sync to channel clock)
// with the system clock, making sure the pulse will be at least one clk0 long, level -> level
wire [3:0] readNextFrame_rst; // async reset for readNextFrame0[3:0]
reg  [3:0] readNextFrameS; // sync to ext clock
reg  [3:0] confirmRead0;   // 1-st stage of sync
reg  [3:0] confirmRead;    // second stage of sync
assign readNextFrame_rst[3:0] = confirmRead[3:0] & ~{readNextFrame3,readNextFrame2,readNextFrame1,readNextFrame0};
always @ (posedge ch0clk or posedge readNextFrame_rst[0])
  if (readNextFrame_rst[0]) readNextFrameS[0] <= 1'b0;
  else                      readNextFrameS[0] <= readNextFrame0;
always @ (posedge ch1clk or posedge readNextFrame_rst[1])
  if (readNextFrame_rst[1]) readNextFrameS[1] <= 1'b0;
  else                      readNextFrameS[1] <= readNextFrame1;
always @ (posedge ch2clk or posedge readNextFrame_rst[2])
  if (readNextFrame_rst[2]) readNextFrameS[2] <= 1'b0;
  else                      readNextFrameS[2] <= readNextFrame2;
always @ (posedge clk0 or   posedge readNextFrame_rst[3]) // posedge here is OK
  if (readNextFrame_rst[3]) readNextFrameS[3] <= 1'b0;
  else                      readNextFrameS[3] <= readNextFrame3;
always @ (negedge clk0) begin
  if (!enSDRAM) begin // just for simulation
    confirmRead0[3:0] <= 4'h0;
  end else begin
    confirmRead0[3:0] <= readNextFrameS[3:0];
  end
  confirmRead[3:0] <= confirmRead0[3:0];
end

 descrproc i_descrproc(.clk(clk0),   // change later to clk0 (negedge)?
                  .ia(ia[3:0]),       // internal 4-bit address bus (fast - directly from I/O pads)
                  .as(as[3:0]),       // 4 bit address to select descriptor address and data word (3 bits)
                  .am(am[3:0]),       // switching between ia (async read) and as (sync write)
                  .mcs(dscs),            // write descriptor memory (from CPU)
                  .mdi(di[17:0]),      // 16-bit (of 32) data from CPU to descriptor memory
                  .mdo(dsdo[31:0]),      // 31-bit data from descriptor memory to CPU (lower 16 - same as written, high 15 - readonly current state
                  .chStIn(chSt1),      // data channel start (generated by arbiter)
                  .chInit(chInit),      // will be generated by arbiter, arbiter will not wait for confiramtion
                  .chNum(chNum[1:0]),   // 2-bit channel number. Is set by arbiter at chStIn or chInit (is zero when SDRAM is disabled)
                  .chStOut(chSt2),      // channel start output (to sequencer). Delayed by 6 tacts from chStIn
                  .sa(startAddr[24:3]),// start address of the block
                  .sfa(sfa[24:8]),     // start address of a frame - ugly hack to roll over the last (extra) 4 lines of a frame
                                       // to the beginning of a frame - used in photofinish mode.
                  .rovr(rovr),         // roll over mode (photofinish, cntrl0[12]==1, last line of tiles)

                  .mode(mode),         // mode (0 - 128x1, 1 - 16x8)
                  .WnR(WnR),            // write /not read
//                  .blkSz(blkSz[6:3]),   // block size (4 bits), 0 - all 128
                  .nBuf(nBuf[1:0]),      // buffer page to use
//                    .nam(nam[6:0]),      // address modifier after the group of 8 words is processed. 6 MSBs should go to bits [12..7] of adder input. LSB - to bit [3]
                  .seq_par(param[5:0]), // [5:0] sequencer parameters
                                       // dual-purpose parameter. In 128x1 mode specifies (4 LSBs) number of 8-word long groups to r/w (0- all 16),
                                       // actually for all but 0 (when exactly 128 words a xfered) will transfer one word more
                                        // In 18x9 mode specifies bits 12:7 of address increment between lines
                  .mancmd(mancmd[17:0]),// 17-bit manual command for SDRAM (when writig data to RO location 4'h3
                  .enSDRAM(enSDRAM),   // output that enables SDRAM auto access. When 0 - only manual commands are allowed.  Written at address 0'h7, bit 0
                  .enRefresh(enRefresh),// written at address 0'h7, bit 1
                  .enXfer(enXfer[3:0]), // enable trasfer through channel (will add dependency later)
                  .chReqInit(chReqInit[3:0]),// request (to arbiter) init channel
                  .nextFrame(nextFrame[3:0]),// (level) generated before actual block is filled - needed to be combined with the pulse from buffer control
                  .confirmRead(confirmRead[3:0]), // confirm channel read (level OK), needed after start or roll over, FIFO will not be filled otherwise - should be sync to clk
                  .bonded (bonded[3:0]),    //[3:0] - channel bonded with the other (just for channel 2), make it TIG
                  .restart_en(restart_en),  // enable restarting selected channels
                  .restart(restart[3:0]),    // reinitialize channels (posedge-sensitive, masked by enXfer0
                  .nextBlocksEn(nextBlocksEn[3:0]) // enable read blocks to FIFO. disabled when init or roll over, enable by confirmRead

//                  .ch3next(ch3next),   // software generated request for the channel 3 xfer (will be OR-ed with hw input) - when writig data to RO location 4'hf);
//                  .initch3(initch3),   // decoded write to channel 3 mode (valid @ MWCLK) 
//                  .wnrbit(wnrbit)
                  );      // data to descriptor memory bit matching wnr (write-not-read)

    channelRequest i_channelRequest0 (.rst(!enSDRAM),            // probably for simulation only...
                                      .init(chReqInit[0]),      // 1 cycle long, sync to iclk
                                      .eclk(ch0clk),            // external clock (posedge)
                                      .start(stch0),            // sync to eclk start command
                                      .iclk(clk0),               // internal clk  (negedge)
                                      .enXfer(enXfer[0]),      // enable xfer request
                                      .ackn(chnAckn[0]),         // acknowledge pulse (sync to iclk)
                                      .wnr(WnR),               // will be valid next cycle after ackn (if it was ackn to rqInit)
                                      .done(dnch[0]),            // data transfer over
                                      .rq(chnReq[0]),            // request (level, sync to iclk)
                                      .rqInit(chnReqInit[0]),   // request to Init channel (level, sync to iclk)
                                      .rdy(ch0rdy),             // external ready output
                                      .rdy_async(),             // output 
                                      .wrempty());               // output 

    channelRequest i_channelRequest1 (.rst(!enSDRAM),            // probably for simulation only...
                                      .init(chReqInit[1]),      // 1 cycle long, sync to iclk
                                      .eclk(ch1clk),            // external clock (posedge)
                                      .start(stch1),            // sync to eclk start command
                                      .iclk(clk0),                // internal clk (negedge)
                                      .enXfer(enXfer[1]),      // enable xfer request
                                      .ackn(chnAckn[1]),         // acknowledge pulse (sync to iclk)
                                      .wnr(WnR),               // will be valid next cycle after ackn (if it was ackn to rqInit)
                                      .done(dnch[1]),            // data transfer over
                                      .rq(chnReq[1]),            // request (level, sync to iclk)
                                      .rqInit(chnReqInit[1]),   // request to Init channel (level, sync to iclk)
                                      .rdy(ch1rdy),             // external ready output
                                      .rdy_async(),             // output 
                                      .wrempty());               // output 
                                      

    channelRequest   i_channelRequest2 (.rst(!enSDRAM),            // probably for simulation only...
                                      .init(chReqInit[2]),      // 1 cycle long, sync to iclk
                                      .eclk(ch2clk),            // external clock (negedge)
                                      .start(stch2),            // sync to eclk start command
                                      .iclk(clk0),               // internal clk (negedge)
                                      .enXfer(enXfer[2]),      // enable xfer request
                                      .ackn(chnAckn[2]),         // acknowledge pulse (sync to iclk)
                                      .wnr(WnR),               // will be valid next cycle after ackn (if it was ackn to rqInit)
                                      .done(dnch[2]),            // data transfer over
                                      .rq(chnReq[2]),            // request (level, sync to iclk)
                                      .rqInit(chnReqInit[2]),   // request to Init channel (level, sync to iclk)
                                      .rdy(ch2rdy),             // external ready output
                                      .rdy_async(),             // output 
                                      .wrempty());               // output 

    channelRequest_1 i_channelRequest3 (.rst(!enSDRAM),            // probably for simulation only...
                                      .init(chReqInit[3]),      // 1 cycle long, sync to iclk. For now - do nothing in dcc mode
                                      .eclk(clk0),               // external clock, here the same clk0.
//                                      .start(ch3next),         // sync to eclk start command (maybe OR hw & SW ?)
                                      .start(next_ch3),        // sync to eclk start command (maybe OR hw & SW ?)
                                      .iclk(clk0),               // internal clk (negedge)
                                      .enXfer(enXfer[3]),      // enable xfer request
                                      .ackn(chnAckn[3]),         // acknowledge pulse (sync to iclk)
                                      .wnr(WnR),               // will be valid next cycle after ackn (if it was ackn to rqInit)
                                      .done(dnch[3]),            // data transfer over
                                      .rq(chnReq[3]),            // request (level, sync to iclk)
                                      .rqInit(chnReqInit[3]),   // request to Init channel (level, sync to iclk)
                                      .rdy(),            // external ready output
                                      .rdy_async(rdy),            // external ready output
                                      .wrempty(wrempty));      // write buffer empty (may reprogram channel)

   always @ (posedge disSDRAM or negedge clk0)
      if      (disSDRAM)  ch3page <= 2'b0;
      else if (init_ch3)  ch3page <= 2'b0;
      else if (next_ch3)  ch3page <= ch3page+1;
   always @ (posedge disSDRAM or negedge clk0)
      if      (disSDRAM)             ch3maddr <=7'b0;
      else if (init_ch3 || next_ch3) ch3maddr <=7'b0;
      else if (menrw)                ch3maddr <= ch3maddr + 1;

//    wire        ch3wnr;
//    reg  [ 6:0] ch3maddr;



//   FDE   i_ch3wnr (.C(mclk),.CE(initch3),.D(wnrbit),.Q(ch3wnr));


   refreshRequest i_refreshRequest   (.enable(enRefresh),         // enable autorefresh
                                       .clk(clk0),               //
                                     .ackn(refrStart),         // acknowledge pulse (sync to clk)
                                     .rq(chnReq[4]));            // request (level, sync to clk)

 chArbit i_chArbit(.clk(clk0),                // clk0
                    .rst(!enSDRAM),            // disable SDRAM automatic access ("manual" for initialization is still enabled)
                    .rq(chnReq[4:0]),         // request for channels 0..3 and auto refresh
                    .rqInit(chnReqInit[3:0]),   // request init for channels 0..3
                    .ch(curChan[4:0]),          // 1-hot active channel (incl. auto refresh)
                    .chNum(chNum[1:0]),         // [1:0]   channel number (to access descriptor memory). SDRAM write mux should register.
                    .chStart(chSt1),            // start data channel (read descriptor,...). Also acknowledge selected channel
                    .chInit(chInit),             // Init channel descriptor (chStart will be also active, but ignored)
                   .chInitOnehot(chInitOnehot[3:0]), // decoded channel init pulses, 2 cycles behind chInit
                    .ackn(chnAckn[3:0]),      // request acknowledge for channels 0..3
                    .refrStart(refrStart),      // start auto refresh cycle
                    .next(nextReq));            // from SDRAM sequencer - enable next channel in que.

   sdseq i_sdseq (.clk0(clk0),   // global clock 75-100MHz (hope to get to 120MHz with Spartan3)
                  .rst(!enSDRAM),   // when active will use mancmd as a source for ras,cas,we,ba,a
                   .xfer(chSt2),   // start block transfer (=stepsEn[2]) - when address is ready
                   .mode(mode),   // 0 - 128x1, 1 - 18x9 (will work only in read mode, channnel 2)
                   .wnr(WnR),
                   .refr(refrStart),   // start auto refresh (should be delayed same time as read/write)
                  .sa(startAddr[24:3]),      // block start address
                  .chsel(chNum[1:0]),   // channel number (0..3) - used to generate delayed dsel;
                  .param(param[5:0]),  // dual-purpose parameter. In 256x1 mode specifies (5 LSBs) number of 8-word long groups to r/w (0- all 32),
                                        //    actually for all but 0 (when exactly 256 words are xfered) will transfer one word more
                                         // In 18x9 mode specifies bits 13:8 of address increment between lines
                   .mancmd(mancmd[17:0]),//   always high except 1 cycle long when it sources {ras,cas,we,ba[1:0],a[11:0]}. Works only whe rst is active
                         // Will use 18 bits from unused descriptor cell , a[11] will be skipped and set to 1'b0????
                  .sfa(sfa[24:8]),     // start address of a frame - ugly hack to roll over the last (extra) 4 lines of a frame
                                       // to the beginning of a frame - used in photofinish mode.
                  .rovr(rovr),         // roll over mode (photofinish, cntrl0[12]==1, last line of tiles)

                  .drun_rd(ch_drun_rd),  // enable read data (from SDRAM) to blockRAM buffer
                  .drun_wr(ch_drun_wr),  // enable write data (to SDRAM) from blockRAM buffer
                  .prefirstdrun(ch_prefirstdrun), // 1 cycle ahead of the first drun_rd/drun_wr to be used by bufCntr256 to sample channel enable
                  .dlast(ch_dlast),    //last cycle of drun active for read, 1 cycle later for write
                   .precmd({sdras,
                            sdcas,
                          sdwe}),   // 1 cycle ahead {ras,cas,we} command bits to be registered at the output pads
                   .prea(sda[12:0]),      // 13 bits (including ba) 1 cycle ahead of output A[12:0]
                   .preb(sdba[1:0]),      // 2 bits of bank address 1 cycle ahead of output BA[1:0]
                   .next(nextReq),      // get next request from que.
                  .pre2trist(trist),   // tristate data outputs (two ahead).
                  .predqt(predqt),   // tristate DQ outputs (one extra for FF in output buffer)
                  .dmask(dmask[1:0]),     // [1:0] - now both the same (even number of words written)
                  .dqs_re(dqs_re),   // enable read from DQS i/o-s for phase adjustments  (latency 2 from the SDRAM RD command)
                  .dsel(dsel[1:0]));      // data buffer select (number of buffer to source SDRAM data in), so it is needed only for WRITE to SDRAM



endmodule

// To add extra register layer between blockRAM and SDRAM modify sdseq to generate run8, last 1 cycle earlier in write mode,
// modify bufCntr, bufCntr3 to delay done output in write mode.
module bufCntr256 (clk,   // SDRAM clock (negedge)
                   rst,   // needed for simulation only...
                   cs,   // enables init and xfer commands
                   init, // 1-cycle cmd: reset address to 9'h00/9'h80/9'h100/9'h180 depending on bank, store wnr.
                   bank, // quater of buffer to use
                   drun_rd, // enable write data to buffer (read from SDRAM)
                   drun_wr, // enable read  data from buffer (write to SDRAM)
                   dlast,// accompanies the last cycle of drun - will generate done and skip to next bank. Delayed by 1 for buff->SDRAM
                          // Not needed for full block transfers
                   a,    // buffer address
                   en,   // buffer r/w enable
                   we,    // buffer write enable
                   done);
  input        clk;
  input        rst;
  input        cs;
  input        init;
  input  [1:0] bank;
  input        drun_rd;
  input        drun_wr;
  input        dlast;
  output   [8:0] a;
  output       en;
  output       we;
  output       done;

   reg         we;
   reg         en;
   reg   [8:0]   a;
   reg         done;
   reg         cs_r;
   wire        sinit= cs && init;

   always @ (negedge clk) begin
     if (rst)       cs_r  <= 1'b0;
     else if (init) cs_r  <= cs;
     en <= cs_r && (drun_rd || drun_wr);
     we <= cs_r && drun_rd;
     done <= cs_r && dlast; // dlast should be already delayed by 1 cycle for buff->SDRAM
     if (rst || sinit || done) a[6:0] <=7'b0;
     else if (en) a[6:0] <= a[6:0]+1;
     a[8:7]      <=  {~rst,~rst} & (sinit ? bank[1:0]: (a[8:7]+ done));
   end
endmodule

module channelRequest (rst,   // only for simulation?
                       init,   // 1 cycle long, sync to iclk
                       eclk,   // external clock  (posedge)
                       start,   // sync to eclk start command
                       iclk,   // internal clk    (negedge)
                       enXfer,// enable xfer request
                       ackn,   // acknowledge pulse (sync to iclk)
                       wnr,   // will be valid skipping 2 cycles (better - 3) ackn (if it was ackn to rqInit)
                       done,   // data transfer over
                       rq,    // request (level, sync to iclk)
                       rqInit,// request to Init channel (level, sync to iclk)
                       rdy,   // external ready output - valid with continuous eclk only
                       rdy_async, // w/o eclk
                       wrempty);// nothing left to write (may reprogram channel). Now in read mode indicates that there are no pending reads
   input         rst;
   input         init;
   input         eclk;
   input         start;
   input         iclk;
   input         ackn;
   input         wnr;
   input         done;
   input         enXfer;
   output      rq;
   output      rqInit;
   output      rdy;
   output      rdy_async;
   output      wrempty;

   reg [2:0] ecnt;   // gray, 0->1->3->2->6->7->5->4
   reg [2:0] icnt;   // gray, 0->1->3->2->6->7->5->4
   reg [2:0] rcnt;   // gray, 0->1->3->2->6->7->5->4
   wire[2:0] next_ecnt;
   wire[2:0] next_icnt;
   wire[2:0] next_rcnt;
   wire       cntrsInit;
   reg       cntrsValid;
   reg       rq;
   reg       rqInit;
   wire       ready_off;   // counter comparison, goes active with eclk, inactive - with iclk
   reg        rdy;
   reg       en_done;   // to disable done initiated by previous activity
   reg       current_wnr;
   wire      rdy_async;
   assign    rdy_async=!ready_off;
   // async clock metastability (ecnt[2:0]!=rcnt[2:0]) and  (ecnt[2:0]!=icnt[2:0]) should not be a problem
   // as "other" clock may only change from rq inactive to rq active (reverse is done by "this" clock), so
   // there will be no false rq, only may be missed - and it is OK, it will be served later.
   // rq may have a one-cycle glitch from "active" to "inactive" it should not cause problems in chArbit
   // same is valid for "rdy" output - it may only be pessimistic, no false rdy.

   assign next_ecnt={ecnt[2]^ ((ecnt==3'b010)||(ecnt==3'b100)),
                     ecnt[1]^ ((ecnt==3'b001)||(ecnt==3'b111)),
                     ~ecnt[2] ^ ecnt[1]};

   assign next_icnt={icnt[2]^ ((icnt==3'b010)||(icnt==3'b100)),
                     icnt[1]^ ((icnt==3'b001)||(icnt==3'b111)),
                     ~icnt[2] ^ icnt[1]};
   assign next_rcnt={rcnt[2]^ ((rcnt==3'b010)||(rcnt==3'b100)),
                     rcnt[1]^ ((rcnt==3'b001)||(rcnt==3'b111)),
                     ~rcnt[2] ^ rcnt[1]};
   always @ (negedge iclk) en_done <= !rqInit && (en_done || ackn);


   assign   ready_off= !cntrsValid || (ecnt[2:0] == rcnt[2:0]);
   always @ (posedge eclk or posedge rst)
     if (rst)  ecnt <= 3'b0;
     else if (start) ecnt[2:0] <= next_ecnt;

// reduce latency of rdy - valid next cycle after start
   always @ (posedge eclk or posedge ready_off)
     if (ready_off)   rdy <= 1'b0;
     else            rdy <= 1'b1;

   assign   wrempty= cntrsValid && (current_wnr?(ecnt[2:0]=={!rcnt[2],!rcnt[1],rcnt[0]}):(icnt[2:0] != ecnt[2:0]));
     always @ (negedge iclk) begin
     cntrsValid <= cntrsInit || (cntrsValid && ! init);
     rqInit <= !rst && (init || (rqInit && !ackn));
     if (cntrsInit) current_wnr <=wnr;
     icnt[2:0] <= cntrsInit? {ecnt[2]^(!wnr),ecnt[1]^(!wnr),ecnt[0]}: ((ackn && cntrsValid)?            next_icnt[2:0]: icnt[2:0]);
     rcnt[2:0] <= cntrsInit? {ecnt[2]^( wnr),ecnt[1]^( wnr),ecnt[0]}: ((en_done && done && cntrsValid)? next_rcnt[2:0]: rcnt[2:0]);
     rq <= rqInit || (enXfer && cntrsValid && (icnt[2:0] != ecnt[2:0]));
   end
//   SRL16_1 i_cntrsInit  (.Q(cntrsInit), .A0(1'b1), .A1(1'b1), .A2(1'b0), .A3(1'b0), .CLK(iclk), .D(ackn && rqInit));   // dly=3+1
   MSRL16_1 i_cntrsInit (.Q(cntrsInit), .A(4'h3), .CLK(iclk), .D(ackn && rqInit));// dly=3+1
endmodule

module channelRequest_1 (rst,   // only for simulation?
                         init,   // 1 cycle long, sync to iclk
                         eclk,   // external clock  (negedge)
                         start,   // sync to eclk start command
                         iclk,   // internal clk    (negedge)
                         enXfer,// enable xfer request
                         ackn,   // acknowledge pulse (sync to iclk)
                         wnr,   // will be valid skipping 2 cycles (better - 3) ackn (if it was ackn to rqInit)
                         done,   // data transfer over
                         rq,    // request (level, sync to iclk)
                         rqInit,// request to Init channel (level, sync to iclk)
                         rdy,   // external ready output - valid with continuous eclk only
                         rdy_async, // w/o eclk
                       wrempty);// nothing left to write (may reprogram channel). Now in read mode indicates that there are no pending reads
   input         rst;
   input         init;
   input         eclk;
   input         start;
   input         iclk;
   input         ackn;
   input         wnr;
   input         done;
   input         enXfer;
   output      rq;
   output      rqInit;
   output      rdy;
   output      rdy_async;
   output      wrempty;

   reg [2:0] ecnt;   // gray, 0->1->3->2->6->7->5->4
   reg [2:0] icnt;   // gray, 0->1->3->2->6->7->5->4
   reg [2:0] rcnt;   // gray, 0->1->3->2->6->7->5->4
   wire[2:0] next_ecnt;
   wire[2:0] next_icnt;
   wire[2:0] next_rcnt;
   wire       cntrsInit;
   reg       cntrsValid;
   reg       rq;
   reg       rqInit;
   wire       ready_off;   // counter comparison, goes active with eclk, inactive - with iclk
   reg        rdy;
   reg       en_done;   // to disable done initiated by previous activity
   reg       current_wnr;
   wire      rdy_async;
   assign    rdy_async=!ready_off;

   // async clock metastability (ecnt[2:0]!=rcnt[2:0]) and  (ecnt[2:0]!=icnt[2:0]) should not be a problem
   // as "other" clock may only change from rq inactive to rq active (reverse is done by "this" clock), so
   // there will be no false rq, only may be missed - and it is OK, it will be served later.
   // rq may have a one-cycle glitch from "active" to "inactive" it should not cause problems in chArbit
   // same is valid for "rdy" output - it may only be pessimistic, no false rdy.

   assign next_ecnt={ecnt[2]^ ((ecnt==3'b010)||(ecnt==3'b100)),
                     ecnt[1]^ ((ecnt==3'b001)||(ecnt==3'b111)),
                     ~ecnt[2] ^ ecnt[1]};

   assign next_icnt={icnt[2]^ ((icnt==3'b010)||(icnt==3'b100)),
                     icnt[1]^ ((icnt==3'b001)||(icnt==3'b111)),
                     ~icnt[2] ^ icnt[1]};
   assign next_rcnt={rcnt[2]^ ((rcnt==3'b010)||(rcnt==3'b100)),
                     rcnt[1]^ ((rcnt==3'b001)||(rcnt==3'b111)),
                     ~rcnt[2] ^ rcnt[1]};
   always @ (negedge iclk) en_done <= !rqInit && (en_done || ackn);


   assign   ready_off= !cntrsValid || (ecnt[2:0] == rcnt[2:0]);
   always @ (negedge eclk or posedge rst)
     if (rst)  ecnt <= 3'b0;
     else if (start) ecnt[2:0] <= next_ecnt;

// reduce latency of rdy - valid next cycle after start
   always @ (negedge eclk or posedge ready_off)
     if (ready_off)   rdy <= 1'b0;
     else            rdy <= 1'b1;

   assign   wrempty= cntrsValid && (current_wnr?(ecnt[2:0]=={!rcnt[2],!rcnt[1],rcnt[0]}):(icnt[2:0] != ecnt[2:0]));
     always @ (negedge iclk) begin
     cntrsValid <= cntrsInit || (cntrsValid && ! init);
     rqInit <= !rst && (init || (rqInit && !ackn));
     if (cntrsInit) current_wnr <=wnr;
     icnt[2:0] <= cntrsInit? {ecnt[2]^(!wnr),ecnt[1]^(!wnr),ecnt[0]}: ((ackn && cntrsValid)?            next_icnt[2:0]: icnt[2:0]);
     rcnt[2:0] <= cntrsInit? {ecnt[2]^( wnr),ecnt[1]^( wnr),ecnt[0]}: ((en_done && done && cntrsValid)? next_rcnt[2:0]: rcnt[2:0]);
     rq <= rqInit || (enXfer && cntrsValid && (icnt[2:0] != ecnt[2:0]));
   end
//   SRL16_1 i_cntrsInit  (.Q(cntrsInit), .A0(1'b1), .A1(1'b1), .A2(1'b0), .A3(1'b0), .CLK(iclk), .D(ackn && rqInit));   // dly=3+1
   MSRL16_1 i_cntrsInit (.Q(cntrsInit), .A(4'h3), .CLK(iclk), .D(ackn && rqInit));// dly=3+1
endmodule

module refreshRequest (enable,// enable autorefresh
                       clk,   //
                       ackn,   // acknowledge pulse (sync to clk)
                       rq);   // request (level, sync to clk)
   parameter   REFRESHPERIOD=11'h400;
   input         enable;
   input         clk;
   input         ackn;
   output      rq;

   reg   [12:0]   nRefrDue;
   reg   [10:0]   rcntr;
   reg            rtim;
   reg            rq;
   always @ (negedge clk)
     if (!enable) nRefrDue <= {1'b1,12'b0};
     else if (ackn) nRefrDue <= nRefrDue - 1;
     else if (rtim) nRefrDue <= nRefrDue + 1;
   always @ (negedge clk) begin
     if (!enable | rtim) rcntr[10:0] <= REFRESHPERIOD;
     else  rcntr[10:0] <= rcntr-1;
     rtim <= !nRefrDue[12] && !(|rcntr[10:0]);   // nRefrDue[12] to "saturate" number of refr. cycles due to 4096
   end
   always @ (negedge clk)      rq <= enable  && |nRefrDue[12:0];
endmodule


///Adding chInitOnehot output

module chArbit (clk,         // SDCLK
                rst,         // disable SDRAM automatic access ("manual" for initialization is still enabled)
                rq,         // request for channels 0..3 and auto refresh
                rqInit,      // request init for channels 0..3
                ch,         // 1-hot active channel (incl. auto refresh)
                chNum,      // [1:0]   channel number (to access descriptor memory). SDRAM write mux should register.
                chStart,   // start data channel (read descriptor,...). Also acknowledge selected channel
                chInit,      // Init channel descriptor (chStart will be also active, but ignored)
                chInitOnehot, // decoded channel init pulses, 2 cycles behind chInit
                ackn,      // request acknowledge for channels 0..3
                refrStart,   // start auto refresh cycle
                next);      // from SDRAM sequencer - enable next channel in que.

   input            clk,rst;
   input      [4:0]   rq;
   input      [3:0]   rqInit;
   output   [4:0]   ch;
   output   [1:0]   chNum;
   output         chStart;
   output         chInit;
   output   [3:0] chInitOnehot;
   output   [3:0]   ackn;
   output         refrStart;
   input            next;
   
   reg   [4:0]   ch;
   reg   [3:0] chInitOnehot;
   reg   [3:0] ch_d; /// ch[3:0] delayed by 1 clock;
   reg         chInit_d; /// chInit  delayed by 1 clock;
   reg   [4:0]   prq;   /// priority applied
   reg   [1:0]   chNum;
   reg         chStart;
   reg         chInit;
   wire         wasChInit;
   reg         refrStart;
   wire      eqZero=(ch[4:0]==5'b0);
   reg   [3:0] ackn;
   always @ (negedge clk )
     prq[4:0]<={rq[4:0]==5'b10000,
                   rq[3:0]==4'b1000,
                   rq[2:0]==3'b100,
                   rq[1:0]==2'b10,
                   rq[0]  ==1'b1
                   };
   always @ (negedge clk or posedge rst)
     if (rst) begin
       ch[4:0]      <= 5'b0;
       chStart      <= 1'b0;
       chInit      <= 1'b0;
       ackn[3:0]   <= 4'b0; 
       refrStart   <= 1'b0;
        chNum[1:0] <= 2'b0;
   end else begin
       if (next || eqZero)  ch[4:0] <= prq[4:0];
       else  if (wasChInit) ch[4:0] <= 5'b0;
       chStart   <= (next || eqZero) && (prq[3:0]!=0);
       ackn[3:0]   <= {(next || eqZero) && prq[3],
                      (next || eqZero) && prq[2],
                      (next || eqZero) && prq[1],
                      (next || eqZero) && prq[0]};
       chInit      <= (next || eqZero) && ((prq[3:0] & rqInit[3:0])!=0);
       refrStart   <= (next || eqZero) &&  prq[4];
       if (chStart) chNum[1:0] <= {ch[2] || ch[3], ch[1] || ch[3]};
   end
   MSRL16_1 i_wasChInit (.Q(wasChInit), .A(4'h1), .CLK(clk), .D(chInit));// dly=1+1

   always @ (negedge clk) begin
     ch_d[3:0] <= ch[3:0];
     chInit_d  <= chInit;
     chInitOnehot <= {4{chInit_d}} & ch_d[3:0];
   end
endmodule
