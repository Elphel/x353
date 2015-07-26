/*
** -----------------------------------------------------------------------------**
** compressor333.v
**
** Top level module for JPEG compressor part of FPGA
**
** Copyright (C) 2002-2010 Elphel, Inc
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
`timescale 1ns/1ps
`define debug_compressor
module compressor(//  cwr,   // CPU write - global clock
                    clk,   // compressor pixel clock.(80MHz)
                    clk2x, // twice clk, used for huffman/stuffer stages. leading front should be before clk leading front (160MHz)
                    cwe,   // we to compressor from CPU (now @ negedge clk) - only command register and number of MCUs
                    wr_saturation, // write color saturation vaues
                                // (currently 10 bits - cb from idi[9:0],
                                //  cr - from  idi[21:12]
                    wr_quantizer_mode, // Quantizer tuning - Coring table pair number (0..7)

                    rs,      // 0 - bit modes,
                           // 1 - number of MCUs
                    twqe,  // quantization table write enable
                    twce,   // coring functions tables (@negedge clk - addr and data valid this cycle and one before)
                    twhe,  // huffamn table write enable
                    twfe,  // table to be multiplied by DCT coefficients (then squared and accumulated)
                           //   to determine focus sharpness
                    
                    ta,    // [8:0] addresses words in tables (quantization and huffman)
                    di,    // [31:0] data from CPU
                    vacts_long, // long vacts pulse (15 scan lines) to delay comprssor start (@pixel clock, need re-sync)
                    eot,   // ( to interrupts?) pulse while processing the last MCU in a frame - predictable time to end of DMA transfer
                             // SDRAM interface
                    done_input,      // will go high after EOT and persist until DCT is enabled
                    done_compress,   // will go high after all data is sent out
                    done_compress_pulse,   // Does not need to be reset
                    compressor_started,   // single sclk compressor started
                    is_compressing,      // @posedge clk, from go_single to eot (actual or simulated)

                    chInitOnehot2, // decoded channel init pulses, 2 cycles behind chInit (to reset page address)
                    nextBlocksEn, // When combined with SDRAM data ready, may be used to end frame compression input (instead of block counter)
                    pxd,   // [7:0] data from SDRAM organized as 18x18x8bit MCUs+overlapping margins
                    pxa,   // [10:0] - address to SDRAM buffer (2 MSBs - page. Starts with 0)
                    pxrdy, // SDRAM buffer (channel 2) has a page ready
                    nxtpage,// read next page to SDRAM buffer (from SDRAM)
                    inc_sdrama, //enable read from SDRAM buffer
                    confirmFrame2Compressor, // (re) enable reading SDRAM to channel 2 FIFO (it stops after each frame over and after channel init)
                    restart_memory, // restart memory channel (to align after broken frames) - masked by enable bit in memory controller
                    bonded,    // channel bonded with the other (just for channel 2), make it TIG
                    bayer_phase,// [1:0] bayer color filter phase 0:(GR/BG), 1:(RG/GB), 2: (BG/GR), 3: (GB/RG)
// Reusing channel 3 for DC components output (****** currently disconnected ******)
                     dccout,         //enable output of DC and HF components for brightness/color/focus adjustments
                     hfc_sel,        // [2:0] (for autofocus) only components with both spacial frequencies higher than specified will be added
                    statistics_dv,     //sclk
                    statistics_do,//[15:0] sclk
// start of frame time (set at first HACT after first VACT)
                    sec,    // [31:0] number of seconds
                    usec,    // [19:0] number of microseconds
                     q,
                     qv,
                     imgptr, // [23:0]image pointer in 32-byte chunks 
                     hifreq  //[31:0])  //  accumulated high frequency components in a frame sub-window
                     ,dma_is_reset     // DMA module is reset, enable resetting data counters
                     ,test_state // {is_compressing, cmprs_repeat,cmprs_en}
`ifdef debug_compressor
                     ,test_cntr0  // [31:0] just for testing
`endif

`ifdef debug_stuffer
                     ,tst_stuf_negedge, // [3:0] just for testing
                     tst_stuf_posedge, // [3:0] just for testing
                     tst_stuf_etrax, // [3:0] just for testing
                     test_cntr,  // [3:0] just for testing
                     test_cntr1  // [3:0] just for testing
`endif
                    );

    input         clk;
    input         clk2x;
    input          cwe;
    input         wr_saturation;
    input         wr_quantizer_mode;
    input         rs;
    input         twqe;
    input         twce;
    input         twhe;
    input         twfe;
//  input  [11:0]  ta;
    input  [ 9:0]  ta;

    input  [15:0]   di;
    input         vacts_long; // long vacts pulse (15 scan lines) to delay comprssor start (@pixel clock, need re-sync)
    output         eot;
    output         done_input;      // will go high after EOT and persist until DCT is enabled
    output         done_compress;   // will go high after all data is sent out
    output        done_compress_pulse;   // Does not need to be reset
    output        compressor_started;
    output        is_compressing;

    input         chInitOnehot2; // @negedge clk2x
    input         nextBlocksEn;  // (@negedge clk2x) When combined with SDRAM data ready, may be used to end frame compression input (instead of block counter)

    input  [ 7:0]   pxd;
    output [10:0] pxa;
    input         pxrdy;
    output         nxtpage;
    output        inc_sdrama;
    output        confirmFrame2Compressor;
    output        restart_memory; // restart memory channel (to align after broken frames) - masked by enable bit in memory controller
    input         bonded;    // channel bonded with the other (just for channel 2), make it TIG
    
    input  [ 1:0] bayer_phase;
    input         dccout;
    input  [2:0]  hfc_sel;
    output        statistics_dv;
    output [15:0] statistics_do;
    input [31:0]  sec;
    input [19:0]  usec;
    output    [15:0]   q;
    output             qv;
     output [23:0]   imgptr;
    output [31:0] hifreq;
    input         dma_is_reset;     // DMA module is reset, enable resetting data counters
    output [ 2:0] test_state; // {is_compressing, cmprs_repeat,cmprs_en}
//   wire           is_compressing; // from go_single to eot (@posedge clk)
`ifdef debug_compressor
    output [31:0] test_cntr0;
`endif
`ifdef debug_stuffer
    output [3:0]  tst_stuf_negedge;
    output [3:0]  tst_stuf_posedge;
    output [3:0]  tst_stuf_etrax;
    output [3:0]  test_cntr;
    output [7:0]  test_cntr1;
`endif

 wire       [ 9:0] m_cb; // variable Cb coefficient (was 'h90 constant)
 wire       [ 9:0] m_cr; // variable Cr coefficient (was 'hb6 constant)

 reg       [15:0] stuffer_do_p;
 reg              stuffer_dv_p;

 wire      [15:0] q;
 wire             qv;


    wire   [ 1:0] bayer_phase;
    wire [7:0]    n000; // number of zero pixels (255 if 256) in a 16x16 tile (overlap ignored,before color conversion)
    wire [7:0]    n255; // number of 0xff pixels (255 if 256) in a 16x16 tile (overlap ignored,before color conversion)

    wire   [15:0] dccdata;
    reg           finish_dcc;
    wire          dccvld;
    wire          dccout;
    reg           dcc_en;
    wire            eot;
    wire          eot_real;
    wire            nxtpage;
//TODO: move focus control outside,
// add second bit to scaling differences (not just 0.5 and 1.0, but also 0.75 - probably most reasonable)
//================NOTE: After memory controller is reprogrammed, compressor should be reset (reset+restarted) - else buffer pages would mismatch (seen that) =============================
//    wire [ 8:0]   ta; // now inputs
//    wire             twqe;
//    wire             twhe;
// new control bits:
// [23] ==1 - set focus mode
// [22:21] 0 - none
//         1 - replace
//         2 - combine for all image
//         3 - combine in window only
// [20] ==1 - set bayer shift (from the gammas)
// [19:18]  - bayer shift value (from the gammas)
// [17] == 1 - set processed tile position in the 20x20 tile got from memory
// [16:14] 0 - top left alignment
//         1 - 1 pixel  right and 1 - down from the top left alignment
//         2 - 2 pixels right and 2 - down from the top left alignment
//         3 - 3 pixels right and 3 - down from the top left alignment
//         4 - 4 pixels right and 4 - down from the top left alignment
// [13]==1 - enable color modes
// [12:9]== 0 - monochrome, (4:2:0),
//          1 - color, 4:2:0, 18x18(old)
//          2 - jp4, original (4:2:0),
//          3 - jp4, dc -improved (4:2:0),
//          4 - color, 4:2:0, 20x20, middle of the tile (not yet implemented)
//          5 - jp4, 4 blocks, (legacy)
//          6 - jp4, 4 blocks, dc -improved
//          7 - jp4, 4 blocks, differential red := (R-G1), blue:=(B-G1), green=G1, green2 (G2-G1). G1 is defined by Bayer shift, any pixel can be used
//          8 - jp4, 4 blocks, differential HDR: red := (R-G1), blue:=(B-G1), green=G1, green2 (high gain)=G2) (G1 and G2 - diagonally opposite)
//          9 - jp4, 4 blocks, differential, divide differences by 2: red := (R-G1)/2, blue:=(B-G1)/2, green=G1, green2 (G2-G1)/2
//         10 - jp4, 4 blocks, differential HDR: red := (R-G1)/2, blue:=(B-G1)/2, green=G1, green2 (high gain)=G2), 
//         11-13 - reserved
//         14 - mono, 4 blocks
//         15 - reserved
// [8:7] == 0,1 - NOP, 2 -   disable, 3 - enable subtracting of average value (DC component), bypassing DCT
// [6] == 1 - enable quantization bank select, 0 - disregard bits [5:3]
// [5:3] = quantization page number (0..7)
// [2]== 1 - enable on/off control:
// [1:0]== 0 - reset compressor, stop immediately
//         1 - enable compressor, disable repetitive mode
//         2 - enable compressor, compress single frame
//         3 - enable compressor, enable repetitive mode
//
// control registetr bits
   wire            cr_w;                        // data written to cr (1 cycle) - now just to reset legacy IRQ
   wire            raw_dv;                     // input pixel data valid (pxd[7:0]may be sent to DMA buffer through multiplexor)
   wire            color_dv; // unused                  // color data valid       (color_d[7:0] may be sent to DMA buffer through multiplexor)
   wire   [ 9:0]   color_d;                     // data out stream from color space converter (6X64 blocks for each MCU - 4Y, Cb,Cr)
   wire   [ 2:0]   color_tn;                  // tile number in an MCU from color space converter (valid @ color_dv)
   wire   [ 8:0]   color_avr;                  // [8:0]   DC (average value) 9 bit signed for Y: 9'h000..9'h0ff, for C - 9h100..9'h000..9'h0ff
   wire            color_first;               // sending first MCU (valid @ ds)
   wire            color_last;                  // sending last MCU (valid @ ds)

//   wire         dct_en;
   wire         dct_start;
   wire         dct_dv;
   wire [12:0]   dct_out;
   wire         dct_last_in;   // output high during input of the last of 64 pixels in a 8x8 block
   wire         dct_pre_first_out; // 1 cycle ahead of the first output in a 64 block

   wire [17:0]   ntiles; //number of 16x16 MCU tiles in a frame to process
   reg         quant_start;
   wire   [12:0] quant_do;
//   wire         quant_dv;
   wire         quant_ds;
//usefocus
   wire   [12:0] focus_do;
   wire         focus_ds;
 
// wire   enc_first;
 wire enc_last;
 wire   [15:0] enc_do;
 wire          enc_dv;

 wire   [15:0]   huff_do;
 wire   [ 3:0]   huff_dl;
 wire         huff_dv;
 wire         flush;   // flush stuffer
 wire   stuffer_rdy;


 //assign stuffer_rdy=1'b1;
 wire   [15:0]   stuffer_do;
 wire            stuffer_dv;
 wire            stuffer_done;
 reg            stuffer_done_persist;
 reg            done_input;      // will go high after EOT and persist until DCT is enabled
 reg            done_compress;   // will go high after all data is sent out
 reg   [1:0]   stuffer_done_sync;
 reg           done_compress_pulse;
 
 reg   [1:0]   compressor_starting;
 reg           compressor_started;  // sync @(negedge clk2x)

 // interclock sync
 reg   cmprs_en_2x_n;   // enable DCT (0- resets buffer pages) - negedge, earlier than 
// reg   cmprs_en_2x_p;      // enable DCT (0- resets buffer pages) - posedge
 reg   cr_w_2x_n;        // 2 cycles long
// reg   cr_w_2x_p;        // 2 cycles long

 reg  wr_saturation_d; //wr_saturation next
// reg  wr_quantizer_mode_d;

   reg pre_finish_dcc;

//   reg [1:0] vlong;
   reg       vlong;
   reg       vlong1_or_not_bonded;
   reg       not_vlong1_or_not_bonded;
//   reg       go_dly;
   reg            go_rq; // request for "go" pulse (will wait for the end of vlong[1] if started during that time
   reg            go_single; // single pulse to start/restart compressor (not in the vacts_long interval)
   wire           confirmFrame2Compressor;

   wire           is_compressing; // from go_single to eot (@posedge clk)
   wire           is_compressing_d; // delayed by several cycles (8) to overlap possible abort_compress_clk;
   wire            force_flush;    // force flush - abort compressing (when writing tiles during is_compressing)
                                  // single cycle @ negedge clk2x   
   reg            abort_compress;     //from force_flush till stuffer_done @negedge clk2x
   reg    [1:0]   abort_compress_clk; // sync to posedge clk - at the end will be simulated
   reg            force_eot;
   
   wire   [2:0]   memWasInit; // syncronizing chInitOnehot2 to clk so buffer page may be reset

    wire           cmprs_en;
   wire           cmprs_start;
   wire           cmprs_repeat;
   wire   [ 2:0]  cmprs_qpage;
   wire           cmprs_dcsub;
   wire   [ 3:0]  cmprs_mode;
   wire   [ 1:0]  cmprs_fmode;
   wire   [ 2:0]  tile_shift; 
// signals synchronized to start of frame compress
   reg    [ 2:0]  cmprs_qpage_this;
   reg            cmprs_dcsub_this;
//   reg    [ 3:0]  cmprs_mode_this;
   reg    [ 1:0]  cmprs_fmode_this;
   reg    [ 2:0]  tile_shift_this;
   reg    [ 1:0]  cmprs_bayer_this; 
   wire           pre_go_single;
   reg            ignore_color;      // ignore color information
   reg            four_blocks;
   reg            jp4_dc_improved;
   reg    [ 1:0]  tile_margin;
   reg    [ 2:0]  converter_type; // 0 - color18, 1 - color20, 2 - mono, 3 - jp4, 4 - jp4-diff , 5 jp4-diff, divide diffs by 2
   wire    [1:0]  bayer_shift;


   wire    [2:0]  component_num;    //[1:0] - component number (YCbCr: 0 - Y, 1 - Cb, 2 - Cr, JP4: 0-1-2-3 in sequence (depends on shift)
   wire           component_color;  // use color quantization table (YCbCR, jp4diff)
   wire           component_first;  // first this component in a frame (DC absolute, otherwise - difference to previous)
   wire           component_lastinmb;  // last component in a macroblock;
   reg            scale_diff;
   reg            hdr;
   reg            confirm_en;
   reg            confirm;
   wire           restart_memory;
   reg            noMoreData_en=0; // enable noMoreData (to delay noMoreData, preventing it from the previous frame)
   reg            noMoreData; // used as alternative to end frame input (possibly broken frame)
   reg   [ 1:0]   nextBlocksEnS; // re-sync to clock

   assign     test_state={is_compressing,cmprs_repeat,cmprs_en};

//     input         dma_is_reset;     // DMA module is reset, enable resetting data counters
   reg    [1:0]  reset_data_counters;
   always @ (negedge clk2x) begin
     reset_data_counters[1:0] <={reset_data_counters[0], dma_is_reset && !cmprs_en_2x_n};
   end



  assign eot=eot_real || force_eot; // force_eot during abort compress
//     input         dma_is_reset;     // DMA module is reset, enable resetting data counters
// assign finish_dcc=stuffer_done;
 assign q[15:0]=                 stuffer_do_p[15:0];
 assign qv=                      stuffer_dv_p;
// assign pre_go_single=           cmprs_en && go_rq && !vlong[1] && !is_compressing && !go_single;
 assign pre_go_single=           cmprs_en &&
                                 go_rq &&
//                                  !vlong[1] &&
                                 not_vlong1_or_not_bonded &&
                                 !is_compressing &&
                                 !is_compressing_d &&
                                 !abort_compress_clk[1] &&
                                 !go_single;
   always @ (posedge clk) begin
     confirm_en <= cmprs_en;
//     confirm <= confirm_en && (go_single || (confirm && !inc_sdrama));
     
     confirm <= confirm_en && (go_single || // confirm memory so it can start reading new frame
//                              memWasInit[2] || // re-confirm memory if it was reset
                              
//                              (confirm && !inc_sdrama));
                              (confirm && !nxtpage)); // until the next block is requestedfrom SDRAM
     
     nextBlocksEnS[1:0] <= {nextBlocksEnS[0], nextBlocksEn};
     noMoreData_en <= cmprs_en && (nxtpage || (noMoreData_en &&  !go_single));
///     noMoreData <= noMoreData_en && confirm_en && !confirm && !nextBlocksEnS[1] && !nextBlocksEnS[0] && !pxrdy;
//     noMoreData <= confirm_en && !confirm && !nextBlocksEnS[1] && !nextBlocksEnS[0] && !pxrdy;
//     noMoreData <= confirm_en && !confirm && !nextBlocksEnS[1] && !nextBlocksEnS[0] && !pxrdy && !go_rq;
     noMoreData <= confirm_en &&
                   !confirm &&
                   !nextBlocksEnS[1] && !nextBlocksEnS[0] &&
                   !pxrdy &&
                   (!go_rq || is_compressing); // prevent noMoreData when just starting, but not when could not finish (too high fps)
   end

 
 reg    bonded_sync;
 always @ (posedge clk) bonded_sync <= bonded; // TIG
 
/// assign confirmFrame2Compressor= go_rq;
 assign confirmFrame2Compressor= confirm;
 assign restart_memory=go_single;
 
 //restart_memory

/// synchronizing chInitOnehot2 from negedge clk2x to posedge clk
   FDCE_1 i_memWasInit0 (.C(clk2x),.CE(chInitOnehot2),.CLR(memWasInit[2]), .D(1'b1),  .Q(memWasInit[0]));
   FD       i_memWasInit1 (.C(clk),                 .D(memWasInit[0]),                 .Q(memWasInit[1]));
   FD       i_memWasInit2 (.C(clk),                 .D(memWasInit[1] & !memWasInit[2]),.Q(memWasInit[2]));

//TODO: add longer delay between frames to make sure they will not get corruped because of the trailer?
FD i_is_compressing (.Q(is_compressing),.C(clk), .D(cmprs_en && (go_single || (is_compressing && !eot)) ));   

// SRL16 i_is_compressing_d(.Q(is_compressing_d),.D(is_compressing_d), .CLK(clk),  .A0(1'b1),  .A1(1'b1), .A2(1'b1), .A3(1'b0));   // dly=7+1
 SRL16 i_is_compressing_d(.Q(is_compressing_d),.D(is_compressing), .CLK(clk),  .A0(1'b1),  .A1(1'b1), .A2(1'b1), .A3(1'b0));   // dly=7+1

   always @ (negedge clk2x) begin
     abort_compress <= cmprs_en_2x_n && (force_flush || (abort_compress && !stuffer_done));
   end
   always @ (posedge clk) begin
     abort_compress_clk[1:0] <= {abort_compress_clk[0],abort_compress};
     force_eot <= abort_compress_clk[0] && !abort_compress_clk[1];
   end

   always @ (posedge clk) begin   // any write to cr will reset done
//     vlong[1:0] <= {vlong[0],vacts_long}; // dual sync
     vlong <= vacts_long; // dual sync
     vlong1_or_not_bonded <= vlong || !bonded_sync;
     not_vlong1_or_not_bonded <= !vlong || !bonded_sync;
    
//     go_rq  <= cmprs_en && (cmprs_start || (cmprs_repeat && vlong[1]) || (go_rq && !go_single));
     go_rq  <= cmprs_en && (cmprs_start || (cmprs_repeat && vlong1_or_not_bonded) || (go_rq && !go_single));
     go_single <= pre_go_single;
     if (pre_go_single) begin
       cmprs_qpage_this[2:0]<=cmprs_qpage[2:0];
       cmprs_dcsub_this     <=cmprs_dcsub;
//       cmprs_mode_this[3:0] <=cmprs_mode[3:0];
       cmprs_fmode_this[1:0]<=cmprs_fmode[1:0];
       tile_shift_this[2:0]<= tile_shift[2:0];
// will inctanciate ROM
       case (cmprs_mode[3:0])
        4'h0: begin //monochrome, (4:2:0),
                ignore_color     <= 1;
                four_blocks      <= 0;
                jp4_dc_improved  <= 0;
                tile_margin[1:0] <= 0;
                converter_type[2:0] <= 2; // 0 - color18, 1 - color20, 2 - mono, 3 - jp4, 4 - jp4-diff
                scale_diff       <=0;
                hdr              <=0;
                cmprs_bayer_this[1:0] <= (bayer_phase[1:0]+bayer_shift[1:0]) ^ {2{tile_shift[0]}};
              end
        4'h1: begin //color, 4:2:0, 18x18(old)
                ignore_color     <= 0;
                four_blocks      <= 0;
                jp4_dc_improved  <= 0;
                tile_margin[1:0] <= 1;
                converter_type[2:0] <= 0; // 0 - color18, 1 - color20, 2 - mono, 3 - jp4, 4 - jp4-diff
                scale_diff       <=0;
                hdr              <=0;
                cmprs_bayer_this[1:0] <= (bayer_phase[1:0]+bayer_shift[1:0]) ^ {2{tile_shift[0]}};
              end
        4'h2: begin // jp4, original (4:2:0),
                ignore_color     <= 1;
                four_blocks      <= 0;
                jp4_dc_improved  <= 0;
                tile_margin[1:0] <= 0;
                converter_type[2:0] <= 3; // 0 - color18, 1 - color20, 2 - mono, 3 - jp4, 4 - jp4-diff
                scale_diff       <=0;
                hdr              <=0;
                cmprs_bayer_this[1:0] <= (bayer_phase[1:0]+bayer_shift[1:0]) ^ {2{tile_shift[0]}};
              end
        4'h3: begin // jp4, dc -improved (4:2:0),
                ignore_color     <= 1;
                four_blocks      <= 0;
                jp4_dc_improved  <= 1;
                tile_margin[1:0] <= 0;
                converter_type[2:0] <= 3; // 0 - color18, 1 - color20, 2 - mono, 3 - jp4, 4 - jp4-diff
                scale_diff       <=0;
                hdr              <=0;
                cmprs_bayer_this[1:0] <= (bayer_phase[1:0]+bayer_shift[1:0]) ^ {2{tile_shift[0]}};
              end
        4'h4: begin // color, 4:2:0, 20x20, middle of the tile (not yet implemented)
                ignore_color     <= 0;
                four_blocks      <= 0;
                jp4_dc_improved  <= 0;
                tile_margin[1:0] <= 2;
                converter_type[2:0] <= 1; // 0 - color18, 1 - color20, 2 - mono, 3 - jp4, 4 - jp4-diff
                scale_diff       <=0;
                hdr              <=0;
                cmprs_bayer_this[1:0] <= (bayer_phase[1:0]+bayer_shift[1:0]) ^ {2{~tile_shift[0]}};
              end
        4'h5: begin // jp4, 4 blocks, (legacy)
                ignore_color     <= 1;
                four_blocks      <= 1;
                jp4_dc_improved  <= 0;
                tile_margin[1:0] <= 0;
                converter_type[2:0] <= 3; // 0 - color18, 1 - color20, 2 - mono, 3 - jp4, 4 - jp4-diff
                scale_diff       <=0;
                hdr              <=0;
                cmprs_bayer_this[1:0] <= (bayer_phase[1:0]+bayer_shift[1:0]) ^ {2{~tile_shift[0]}};
              end
        4'h6: begin // jp4, 4 blocks, dc -improved
                ignore_color     <= 1;
                four_blocks      <= 1;
                jp4_dc_improved  <= 1;
                tile_margin[1:0] <= 0;
                converter_type[2:0] <= 3; // 0 - color18, 1 - color20, 2 - mono, 3 - jp4, 4 - jp4-diff
                scale_diff       <=0;
                hdr              <=0;
                cmprs_bayer_this[1:0] <= (bayer_phase[1:0]+bayer_shift[1:0]) ^ {2{~tile_shift[0]}};
              end
        4'h7: begin // jp4, 4 blocks, differential
                ignore_color     <= 1;
                four_blocks      <= 1;
                jp4_dc_improved  <= 0;
                tile_margin[1:0] <= 0;
                converter_type[2:0] <= 4; // 0 - color18, 1 - color20, 2 - mono, 3 - jp4, 4 - jp4-diff
                scale_diff       <=0;
                hdr              <=0;
                cmprs_bayer_this[1:0] <= (bayer_phase[1:0]+bayer_shift[1:0]) ^ {2{~tile_shift[0]}};
              end
        4'h8: begin // jp4, 4 blocks, differential, hdr
                ignore_color     <= 1;
                four_blocks      <= 1;
                jp4_dc_improved  <= 0;
                tile_margin[1:0] <= 0;
                converter_type[2:0] <= 4; // 0 - color18, 1 - color20, 2 - mono, 3 - jp4, 4 - jp4-diff
                scale_diff       <=0;
                hdr              <=1;
                cmprs_bayer_this[1:0] <= (bayer_phase[1:0]+bayer_shift[1:0]) ^ {2{~tile_shift[0]}};
              end
        4'h9: begin // jp4, 4 blocks, differential, divide diff by 2
                ignore_color     <= 1;
                four_blocks      <= 1;
                jp4_dc_improved  <= 0;
                tile_margin[1:0] <= 0;
                converter_type[2:0] <= 4; // 0 - color18, 1 - color20, 2 - mono, 3 - jp4, 4 - jp4-diff
                scale_diff       <=1;
                hdr              <=0;
                cmprs_bayer_this[1:0] <= (bayer_phase[1:0]+bayer_shift[1:0]) ^ {2{~tile_shift[0]}};
              end
        4'ha: begin // jp4, 4 blocks, differential, hdr, divide diff by 2
                ignore_color     <= 1;
                four_blocks      <= 1;
                jp4_dc_improved  <= 0;
                tile_margin[1:0] <= 0;
                converter_type[2:0] <= 4; // 0 - color18, 1 - color20, 2 - mono, 3 - jp4, 4 - jp4-diff
                scale_diff       <=1;
                hdr              <=1;
                cmprs_bayer_this[1:0] <= (bayer_phase[1:0]+bayer_shift[1:0]) ^ {2{~tile_shift[0]}};
              end
        4'he: begin // mono, 4 blocks
                ignore_color     <= 1;
                four_blocks      <= 1;
                jp4_dc_improved  <= 0;
                tile_margin[1:0] <= 0;
                converter_type[2:0] <= 2; // 0 - color18, 1 - color20, 2 - mono, 3 - jp4, 4 - jp4-diff
                cmprs_bayer_this[1:0] <= (bayer_phase[1:0]+bayer_shift[1:0]) ^ {2{~tile_shift[0]}};
              end
       endcase
     end
   end

// [12:9]== 0 - color, 4:2:0
//          1 - monochrome, (4:2:0),
//          2 - jp4, original (4:2:0),
//          3 - jp4, dc -improved (4:2:0),
//          4 - color, 4:2:0, 20x20, middle of the tile (not yet implemented)
//          5 - jp4, 4 blocks, (legacy)
//          6 - jp4, 4 blocks, dc -improved
//          7 - jp4, 4 blocks, differential red := (R-G1), blue:=(B-G1), green=G1, green2 (G2-G1). G1 is defined by Bayer shift, any pixel can be used
//          8 - jp4, 4 blocks, differential HDR: red := (R-G1), blue:=(B-G1), green=G1, green2 (high gain)=G2) (G1 and G2 - diagonally opposite)
//          9 - jp4, 4 blocks, differential, divide differences by 2: red := (R-G1)/2, blue:=(B-G1)/2, green=G1, green2 (G2-G1)/2
//         10 - jp4, 4 blocks, differential HDR: red := (R-G1)/2, blue:=(B-G1)/2, green=G1, green2 (high gain)=G2), 
//         11-13 - reserved
//         14 - mono, 4 blocks
//         15 - reserved
 
 

 always @ (negedge clk2x) begin
   cmprs_en_2x_n <= cmprs_en;
   cr_w_2x_n <= cr_w;

 end

 always @ (posedge clk2x) begin
//   cmprs_en_2x_p <= cmprs_en_2x_n;
//   cr_w_2x_p <= cr_w_2x_n;

 end



   always @ (negedge clk2x) begin   // any write to cr will reset done
     stuffer_done_persist <= !cr_w_2x_n && cmprs_en_2x_n && (stuffer_done_persist || stuffer_done );
     done_compress   <= !cr_w_2x_n && cmprs_en_2x_n && (done_compress || stuffer_done_persist);
     stuffer_done_sync[1:0] <= {stuffer_done_sync[0],stuffer_done};
     done_compress_pulse <= stuffer_done_sync[0] && !stuffer_done_sync[1];
     compressor_starting[1:0] <= {compressor_starting[0],color_first & dct_start};
     compressor_started <= (color_tn[2:0]==3'h0) &&compressor_starting[0] && !compressor_starting[1];  // sync @(negedge clk2x)
   end
   
   always @ (posedge clk) begin   // any write to cr will reset done
     done_input      <= !cr_w && cmprs_en && (done_input    || eot);
   end

   always @ (posedge clk2x) begin
        stuffer_do_p[15:0] <= stuffer_do[15:0];
        stuffer_dv_p      <= stuffer_dv;
   end
 wire stuffer_flushing;
`ifdef debug_stuffer
//testing
   reg [3:0] tst_stuf_negedge, tst_stuf_posedge;
   reg tst_enc_dv_r;

//   always @ (negedge clk2x) begin   // any write to cr will reset done
      // cmprs_en_2x_n
     /*if (!cmprs_en_2x_n)  tst_stuf_negedge <=0;
     else*/ //if (stuffer_dv) tst_stuf_negedge <= tst_stuf_negedge+1;
//   end
//   always @ (posedge clk2x) begin   // any write to cr will reset done
     /*if      (!cmprs_en)  tst_stuf_posedge <=0;
     else*/ //if (stuffer_dv_p) tst_stuf_posedge <= tst_stuf_posedge+1;
//   end
   always @ (negedge clk2x) begin
     tst_enc_dv_r <= enc_dv;
     if (stuffer_done) tst_stuf_negedge <=0;
     else if (tst_enc_dv_r)  tst_stuf_negedge <= tst_stuf_negedge+1; // twice per
   end
   always @ (posedge clk) begin  // any write to cr will reset done
//     if (stuffer_done) tst_stuf_negedge <=0;
//     else if (enc_dv)  tst_stuf_negedge <= tst_stuf_negedge+1;
  
     if (go_single)    tst_stuf_posedge <= 0;
     else if (enc_dv)  tst_stuf_posedge <= tst_stuf_posedge+1;
   end
`endif


 always @ (negedge clk2x) pre_finish_dcc <= stuffer_done;
 always @ (posedge clk2x) finish_dcc     <= pre_finish_dcc; //stuffer+done - @negedge clk2x
 //propagation of first block through compressor pipeline
  wire          first_block_color=(color_tn[2:0]==3'h0) && color_first;        // while color conversion,
  reg           first_block_color_after;  // after color conversion,
  reg           first_block_dct;     // after DCT
  wire          first_block_quant;   // after quantizer
  always @ (posedge clk) begin
    if (dct_start)   first_block_color_after <= first_block_color;
    if (dct_last_in) first_block_dct   <= first_block_color_after;
  end

 compr_ifc i_compr_ifc (.clk(clk),         // pixel clock (80MHz)
                          .sclk(clk2x),      // system clock (160MHz)
                          .cwe(cwe),         // we to compressor from CPU (sync to negedge sclk)
                          .rs(rs),         // register select:
                                          // 0 - bit modes,
                                          // 1 - write ntiles;
                          .di(di[15:0]),     // [15:0] data from CPU
                        .cr_w(cr_w),      // data written to cr (1 cycle long)
                        .ntiles(ntiles[17:0]),// - number of 16x16 MCU tiles in a frame to process
                        .cmprs_en(cmprs_en),    // enable compressor
                        .cmprs_start(cmprs_start), // single cycle when single or constant compression is turned on
                        .cmprs_repeat(cmprs_repeat),// high in repetitive mode
                        .cmprs_qpage(cmprs_qpage), // [2:0] - quantizator page number (0..7)
                        .cmprs_dcsub(cmprs_dcsub), // subtract dc level before DCT, restore later
                        .cmprs_mode(cmprs_mode),  // [3:0] - compressor mode
                        .cmprs_shift(tile_shift[2:0]), // tile shift from top left corner
                        .cmprs_fmode(cmprs_fmode), //[1:0] - focus mode
                        .bayer_shift(bayer_shift), // additional shift to bayer mosaic
                        .is_compressing(is_compressing), // high from start of compressing till EOT (sync to posedge clk)
                          .abort_compress(abort_compress),
                        .force_flush(force_flush)); // abort compress - generate flush pulse, force end of image over DMA, update counter
                                                   // single-cycle @ negedge sclk
                        
`ifdef debug_compressor
wire        debug_bcntrIsZero;
wire [17:0] debug_bcntr;
`endif
 color_proc i_color_proc(.clk(clk),            // pixel clock 37.5MHz
                         .en(cmprs_en),      // Enable (0 will reset states)
                         .en_sdc(cmprs_dcsub_this), // enable subtracting of DC component
                         .go(go_single),      // now - always single-cycle pulse (TODO: simplify color_proc)
                         .nblocks(ntiles[17:0]),// [15:0] number of 16x16 blocks to read (valid @ "go" pulse only)
                         .eot(eot_real),         // single-cycle end of transfer pulse
                         .m_cb(m_cb[9:0]),       // [9:0] scale for CB - default 0.564 (10'h90)
                         .m_cr(m_cr[9:0]),       // [9:0] scale for CB - default 0.713 (10'hb6)
                          .ignore_color(ignore_color),   //zero Cb/Cr components
                         .four_blocks(four_blocks), // use only 6 blocks for the output, not 6
                         .jp4_dc_improved(jp4_dc_improved), // in JP4 mode, compare DC coefficients to the same color ones
                         .tile_margin(tile_margin[1:0]), // margins around 16x16 tiles (0/1/2)
                         .tile_shift(tile_shift_this[2:0]), // tile shift from top left corner
                         .converter_type(converter_type[2:0]), // 0 - color18, 1 - color20, 2 - mono, 3 - jp4, 4 - jp4-diff
                         .scale_diff(scale_diff),     // divide differences by 2 (to fit in 8-bit range)
                         .hdr(hdr),            // second green absolute, not difference
                         .memWasInit(memWasInit[2]), // memory channel2 was just initialized - reset page address
                         .di(pxd[7:0]),           // [7:0]
                         .sdram_a(pxa[10:0]),     // [10:0]    (2 MSBs - SDRAM buffer page number)
                         .sdram_rdy(pxrdy),        // SDRAM buffer ready
                         .sdram_next(nxtpage),    // request to read a page to SDRAM buffer
                         .inc_sdrama(inc_sdrama), // enable read sdram buffer
                         .noMoreData(noMoreData), // used as alternative to end frame input (possibly broken frame)
                         .dv_raw(raw_dv),           // data valid for di (for testing to bypass color conversion - use di[7:0])
   
//                         .do(color_d[8:0]),        // [9:0] data out (4:2:0) (signed, average=0)
                         .do(color_d[9:0]),     // [9:0] data out (4:2:0)
                          .avr(color_avr[8:0]),     // [8:0] DC (average value) - RAM output, no register. For Y components 9'h000..9'h0ff, for C - 9'h100..9'h0ff
                         .dv(color_dv),         // out data valid (will go high for at least 64 cycles)
                         .ds(dct_start),         // single-cycle mark of the first pixel in a 64 (8x8) - pixel block
                         .tn(color_tn[2:0]),      // [2:0] tile number 0..3 - Y, 4 - Cb, 5 - Cr (valid with start)
                         .first(color_first),   // sending first MCU (valid @ ds)
                         .last(color_last),      // sending last MCU (valid @ ds)
                         .n000(n000[7:0]),      // [7:0] number of zero pixels (255 if 256)
                         .n255(n255[7:0]),      // [7:0] number of 0xff pixels (255 if 256)
                         .bayer_phase(cmprs_bayer_this[1:0]), // bayer color filter phase 0:(GR/BG), 1:(RG/GB), 2: (BG/GR), 3: (GB/RG)
// below signals valid at ds ( 1 later than tn, first, last)
                         .component_num(component_num[2:0]),    //[1:0] - component number (YCbCr: 0 - Y, 1 - Cb, 2 - Cr, JP4: 0-1-2-3 in sequence (depends on shift) >=4 - don't use
                         .component_color(component_color),     // use color quantization table (YCbCR, jp4diff)
                         .component_first(component_first),      // first this component in a frame (DC absolute, otherwise - difference to previous)
                         .component_lastinmb(component_lastinmb) // last component in a macroblock;
`ifdef debug_compressor
                         ,.bcntrIsZero(debug_bcntrIsZero)                         
                         ,.bcntr(debug_bcntr[17:0])
`endif
                         );

//assign color_d[9]=color_d[8]; // temporary

 xdct       i_xdct ( .clk(clk),             // top level module
                     .en(cmprs_en),       // if zero will reset transpose memory page numbers
                     .start(dct_start),    // single-cycle start pulse that goes with the first pixel data. Other 63 should follow
                     .xin(color_d[9:0]),    // [7:0] - input data
                     .last_in(dct_last_in),   // output high during input of the last of 64 pixels in a 8x8 block //
                     .pre_first_out(dct_pre_first_out),// 1 cycle ahead of the first output in a 64 block

                     .dv(dct_dv),          // data output valid. Will go high on the 94-th cycle after the start
                     .d_out(dct_out[12:0]));// [12:0]output data

// probably dcc things are not needed anymore

 always @ (posedge clk) quant_start <= dct_pre_first_out;

 always @ (posedge clk) begin
  if (!dccout) dcc_en <=1'b0;
  else if (dct_start && color_first && (color_tn[2:0]==3'b001)) dcc_en <=1'b1; // 3'b001 - closer to the first "start" in quantizator
 end
 wire [15:0] quant_dc_tdo;// MSB aligned coefficient for the DC component (used in focus module)

 //wire [3:0]  coring_num;
 wire [2:0]  coring_num;
 
 quantizator i_quantizator(.clk(clk),   // pixel clock
                     .en(cmprs_en),   // enable (0 resets counter)
                     .sclk(clk2x), // system clock, twe, ta,tdi - valid @negedge (ra, tdi - 2 cycles ahead
                     .twqe(twqe), // enable write to a table
                     .twce(twce),   // coring functions tables (@negedge clk - addr and data valid this cycle and one before)
                     .ta(ta[8:0]),  // [6:0]  table address
                     .tdi(di[15:0]),  // 
                     .ctypei(component_color),   // component type input (Y/C)
                     .dci(color_avr[8:0]),   // [8:0] - average value in a block - subtracted before DCT (now 9 bits signed)
                     .first_stb(first_block_color), //this is first stb pulse in a frame

                     .stb(dct_start),      // strobe that writes ctypei, dci
                     .tsi(cmprs_qpage_this[2:0]),  // table (quality) select
                     .pre_start(dct_pre_first_out),
                     .first_in(first_block_dct), // first block in (valid @ start)
                     .first_out(first_block_quant), // valid @ ds

                     .di(dct_out[12:0]),    // [8:0] pixel data in (signed)
                     .do(quant_do[12:0]),    // [11:0] pixel data out signed
//                     .dv(quant_dv),    // data out valid
                     .dv(),    // data out valid
                     .ds(quant_ds),  // data out strobe (one ahead of the start of dv)
                     .dc_tdo(quant_dc_tdo[15:0]), //[15:0], MSB aligned coefficient for the DC component (used in focus module)
//                     .dc_tdo_stb(quant_dc_tdo_stb),
                     .dcc_en(dcc_en),  // enable dcc (sync to beginning of a new frame)
                     .hfc_sel(hfc_sel), // hight frequency components select [2:0] (includes components with both numbers >=hfc_sel
                     .color_first(color_first), // first MCU in a frame NOTE: ONLY NEEDED for DCC
                     .coring_num(coring_num), ///coring table par number
                     .dcc_vld(dccvld),
                     .dcc_data(dccdata[15:0]),
                     .n000(n000[7:0]),      // input [7:0] number of zero pixels (255 if 256) - to be multiplexed with dcc
                     .n255(n255[7:0]));     // input [7:0] number of 0xff pixels (255 if 256) - to be multiplexed with dcc
                     
//TODO: compact table                     
focus_sharp i_focus_sharp(.clk(clk),   // pixel clock
                   .en(cmprs_en),   // enable (0 resets counter)
                   .sclk(clk2x), // system clock, twe, ta,tdi - valid @negedge (ra, tdi - 2 cycles ahead
                   .twe(twfe), // enable write to a table
                   .ta(ta[9:0]),  // [9:0]  table address
                   .tdi(di[15:0]),  // [15:0] table data in (8 LSBs - quantization data)
                   .mode(cmprs_fmode_this[1:0]), // focus mode (combine image with focus info) - 0 - none, 1 - replace, 2 - combine all,  3 - combine woi
//                   .stren(focus_strength),
                   .firsti(color_first),  // first macroblock
                   .lasti(color_last),    // last macroblock
                   .tni(color_tn[2:0]),   // block number in a macronblock - 0..3 - Y, >=4 - color (sync to stb)
                   .stb(dct_start),      // strobe that writes ctypei, dci
                   .start(quant_start),// marks first input pixel (needs 1 cycle delay from previous DCT stage)
                   .di(dct_out[12:0]),    // [11:0] pixel data in (signed)
                   .quant_ds(quant_ds), // quantizator data strobe (1 before DC)
                   .quant_d(quant_do[12:0]), // quantizator data output
                   .quant_dc_tdo(quant_dc_tdo[15:0]), //[15:0], MSB aligned coefficient for the DC component (used in focus module)
//                   .quant_dc_tdo_stb(quant_dc_tdo_stb),
                   .do(focus_do[12:0]),    // [11:0] pixel data out (AC is only 9 bits long?) - changed to 10
                   .ds(focus_ds),  // data out strobe (one ahead of the start of dv)
                   .hifreq(hifreq[31:0])  //[31:0])  //  accumulated high frequency components in a frame sub-window
                   );

dcc_sync i_dcc_sync(//.clk(clk),
                    .sclk(clk2x),
                    .dcc_en(dcc_en),                   // clk rising, sync with start of the frame
                    .finish_dcc(finish_dcc),           // sclk rising
                    .dcc_vld(dccvld),                 // clk rising
                    .dcc_data(dccdata[15:0]),         //[15:0] clk risimg
                    .statistics_dv(statistics_dv),     //sclk
                    .statistics_do(statistics_do[15:0])//[15:0] sclk
                 );

// generate DC data/strobe for the direct output (re) using sdram channel3 buffering
// encoderDCAC is updated to handle 13-bit signed data instead of the 12-bit. It will limit the values on ot's own
 encoderDCAC i_encoderDCAC(.clk(clk),
                     .en(cmprs_en),
//                     .firsti(color_first),   // was "first MCU in a frame" (@ stb)
                     .lasti(color_last),   // was "last MCU in a frame" (@ stb)
//                     .tni(color_tn[2:0]),      // [2:0] tile number in MCU (0..5) - was valid @ stb
                     .first_blocki(first_block_color),   // first block in frame - save fifo write address (@ stb) 
                     .comp_numberi(component_num[2:0]),   // [2:0] component number 0..2 in color, 0..3 - in jp4diff, >= 4 - don't use (@ stb) 
                     .comp_firsti(component_first),    // fitst this component in a frame (reset DC) (@ stb) 
                     .comp_colori(component_color),    // use color - huffman? (@ stb) 
                     .comp_lastinmbi(component_lastinmb), // last component in a macroblock (@ stb) 
                     .stb(dct_start),      // strobe that writes firsti, lasti, tni,average
                     .zdi(focus_do[12:0]),      // [12:0] 
                     .first_blockz(first_block_quant),  // first block input (@zds)
                     .zds(focus_ds),      // strobe - one ahead of the DC component output
                     .last(enc_last),      // - not used
                     .do(enc_do[15:0]),
                     .dv(enc_dv)
                     );


 wire last_block, test_lbw;
 huffman i_huffman  (.pclk(clk),      // pixel clock
                      .clk(clk2x),   // twice frequency - uses negedge inside
                      .en(cmprs_en),      // enable (0 resets counter) sync to .pclk(clk)
//                      .cwr(cwr),      // CPU WR global clock
                      .twe(twhe),      // enable write to a table
                      .ta(ta[8:0]),      // [8:0]  table address
                      .tdi(di[15:0]),      // [23:0] table data in (8 LSBs - quantization data, [13:9] zigzag address
                      .di(enc_do[15:0]),      // [15:0]   specially RLL prepared 16-bit data (to FIFO)
                      .ds(enc_dv),      // di valid strobe
                      .rdy(stuffer_rdy),      // receiver (bit stuffer) is ready to accept data
                      .do(huff_do),      // [15:0]   output data
                      .dl(huff_dl),      // [3:0]   output width (0==16)
                      .dv(huff_dv),      // output data bvalid
                     .flush(flush),
                     .last_block(last_block),
                     .test_lbw(),
                     .gotLastBlock(test_lbw));   // last block done - flush the rest bits

`ifdef debug_compressor
 reg  eot_2x_n,stuffer_flush_enable;
// wire stuffer_flushing;
 always @ (negedge clk2x) begin
   eot_2x_n <=eot;
   stuffer_flush_enable <= cmprs_en_2x_n && (stuffer_flush_enable?(!stuffer_flushing):eot_2x_n);
 end   
 reg  [31:0] test_cntr0;
 reg  [1:0] was_flush;
 reg  [1:0] was_eot;
 reg  [1:0] was_last_block;
 reg  [1:0] was_test_lbw;
 reg  [1:0] was_go_single;
 reg  [1:0] was_is_compressing;
// reg  [1:0] was_go_single;
 reg  [1:0] was_bcntrIsZero;
 reg  [1:0] was_go_rq;
 
 
 always @ ( negedge clk2x) begin
//   was_flush[1:0] <= {was_flush[0],flush};
   was_flush     [1:0] <= {was_flush[0],     flush && stuffer_flush_enable};
//   was_eot       [1:0] <= {was_eot[0],       eot_2x_n};
   was_last_block[1:0] <= {was_last_block[0],last_block};
   was_test_lbw  [1:0] <= {was_test_lbw[0],  test_lbw};
   was_go_single[1:0] <= {was_go_single[0],go_single};
   was_is_compressing  [1:0] <= {was_is_compressing[0],  is_compressing};
//   if      (!cmprs_en_2x_n)              test_cntr0[7:0] <= 0;
//   else if (was_flush[1:0]==2'h1)        test_cntr0[7:0] <=test_cntr0[7:0] + 1; // 1->7a (was 7b without mask?)
////   if      (!cmprs_en_2x_n)              test_cntr0[15:8] <= 0;
////   else if (was_eot[1:0]==2'h1)          test_cntr0[15:8] <=test_cntr0[15:8] + 1;  // 1->7a !!!
//////   if      (!cmprs_en_2x_n)              test_cntr0[23:16] <= 0;
//////   else if (was_last_block[1:0]==2'h1)   test_cntr0[23:16] <=test_cntr0[23:16] + 1;  // 1->7b
////// if      (!cmprs_en_2x_n)              test_cntr0[31:24] <= 0;
//////   else if (was_test_lbw[1:0]==2'h1)     test_cntr0[31:24] <=test_cntr0[31:24] + 1; // 1->7b

////   if      (!cmprs_en_2x_n)              test_cntr0[23:16] <= 0;
////   else if (was_go_single[1:0]==2'h1)    test_cntr0[23:16] <=test_cntr0[23:16] + 1;  // 1->2
////   if      (!cmprs_en_2x_n)                test_cntr0[31:24] <= 0;
////   else if (was_is_compressing[1:0]==2'h1) test_cntr0[31:24] <=test_cntr0[31:24] + 1; // 1->2
 end   
 
// debug_bcntrIsZero
// reg  [1:0] was_go_single;
// reg  [1:0] was_bcntrIsZero;

// debug_bcntrIsZero
 reg debug1;
 always @ ( posedge clk) begin
   was_eot              [1:0] <= {was_eot[0],       eot};
   was_go_rq            [1:0] <= {was_go_rq[0],     go_rq};
   was_bcntrIsZero      [1:0] <= {was_bcntrIsZero[0],  debug_bcntrIsZero};
////                                        test_cntr0[7:0]   <= debug_bcntr[7:0];      // 0 -> 0 -> 78 -> 78 -> 78 
///                                        test_cntr0[15:0]   <= debug_bcntr[15:0];      // 0 -> 0 -> 78 -> 78 -> 78 
//   if      (was_go_rq[1:0]==2'h1)       test_cntr0[15:8]  <= 0;
//   else if (was_eot[1:0]==2'h1)         test_cntr0[15:8]  <=test_cntr0[15:8] + 1;

//   if      (!cmprs_en)                  test_cntr0[23:16] <= 0;
//   else if (go_single)                  test_cntr0[23:16] <=test_cntr0[23:16] + 1;    // 0 -> 1 ->  2 ->  3 ->  4
//   if      (!cmprs_en)                  test_cntr0[31:24] <= 0;
//   else if (was_bcntrIsZero[1:0]==2'h1) test_cntr0[31:24] <=test_cntr0[31:24] + 1;    // 0 -> 1 ->  1 ->  2 ->  3

// should end up 1 more than number of blocks?
//   if      (go_single)                  test_cntr0[31:16] <= 0;
//   else if (nxtpage)                    test_cntr0[31:16] <=test_cntr0[31:16] + 1;    //

   if      (go_single)                  test_cntr0[15:0]  <= 0;
   else if (nxtpage)                    test_cntr0[14:0]  <= test_cntr0[14:0] + 1;    //
   if      (go_single)                  test_cntr0[30:16] <= test_cntr0[14:0];
   if      (was_go_rq[1:0]==2'h1)       debug1 <= 1'b0;
   else if (pxrdy)                      debug1 <= 1'b1;
   if      (go_single)                  test_cntr0[31] <= debug1;
   test_cntr0[15] <= pxrdy;

 end  
`endif

 



 stuffer   i_stuffer  (.clk(clk2x),         //clock - uses negedge inside
                     .en(cmprs_en_2x_n),         // enable, 0- reset
                     .reset_data_counters(reset_data_counters[1]), // reset data transfer counters (only when DMA and compressor are disabled)
//                     .flush(flush),      // flush output data (fill byte with 0, long word with FFs
                     .flush(flush || force_flush),      // flush output data (fill byte with 0, long word with FFs
                     .stb(huff_dv),      // input data strobe
                     .dl(huff_dl),         // [3:0] number of bits to send (0 - 16)
                     .d(huff_do),         // [15:0] input data to shift (only lower bits are valid)
// time stamping - will copy time at the end of color_first (later than the first hact after vact in the current froma, but before the next one
// and before the data is needed for output 
                     .color_first(color_first), //
                     .sec(sec[31:0]),
                     .usec(usec[19:0]),
                     .rdy(stuffer_rdy),      // enable huffman encoder to proceed. Used as CE for many huffman encoder registers
                     .q(stuffer_do),         // [15:0] output data
                     .qv(stuffer_dv),      // output data valid
                     .done(stuffer_done),
                     .imgptr (imgptr[23:0]), // [23:0]image pointer in 32-byte chunks
                     .flushing(stuffer_flushing)
`ifdef debug_stuffer
                     ,.etrax_dma_r(tst_stuf_etrax[3:0]) // [3:0] just for testing
                     ,.test_cntr(test_cntr[3:0])
                     ,.test_cntr1(test_cntr1[7:0])
`endif
                     );

always @ (negedge clk2x) wr_saturation_d <= wr_saturation;
   FDE_1 #(.INIT(1'b0))  i_m_cb0 (.C(clk2x),.CE(wr_saturation),.D(di[ 0]),.Q(m_cb[0]));
   FDE_1 #(.INIT(1'b0))  i_m_cb1 (.C(clk2x),.CE(wr_saturation),.D(di[ 1]),.Q(m_cb[1]));
   FDE_1 #(.INIT(1'b0))  i_m_cb2 (.C(clk2x),.CE(wr_saturation),.D(di[ 2]),.Q(m_cb[2]));
   FDE_1 #(.INIT(1'b0))  i_m_cb3 (.C(clk2x),.CE(wr_saturation),.D(di[ 3]),.Q(m_cb[3]));
   FDE_1 #(.INIT(1'b1))  i_m_cb4 (.C(clk2x),.CE(wr_saturation),.D(di[ 4]),.Q(m_cb[4]));
   FDE_1 #(.INIT(1'b0))  i_m_cb5 (.C(clk2x),.CE(wr_saturation),.D(di[ 5]),.Q(m_cb[5]));
   FDE_1 #(.INIT(1'b0))  i_m_cb6 (.C(clk2x),.CE(wr_saturation),.D(di[ 6]),.Q(m_cb[6]));
   FDE_1 #(.INIT(1'b1))  i_m_cb7 (.C(clk2x),.CE(wr_saturation),.D(di[ 7]),.Q(m_cb[7]));
   FDE_1 #(.INIT(1'b0))  i_m_cb8 (.C(clk2x),.CE(wr_saturation),.D(di[ 8]),.Q(m_cb[8]));
   FDE_1 #(.INIT(1'b0))  i_m_cb9 (.C(clk2x),.CE(wr_saturation),.D(di[ 9]),.Q(m_cb[9]));

   FDE_1 #(.INIT(1'b0))  i_m_cr0 (.C(clk2x),.CE(wr_saturation),  .D(di[12]),.Q(m_cr[0]));
   FDE_1 #(.INIT(1'b1))  i_m_cr1 (.C(clk2x),.CE(wr_saturation),  .D(di[13]),.Q(m_cr[1]));
   FDE_1 #(.INIT(1'b1))  i_m_cr2 (.C(clk2x),.CE(wr_saturation),  .D(di[14]),.Q(m_cr[2]));
   FDE_1 #(.INIT(1'b0))  i_m_cr3 (.C(clk2x),.CE(wr_saturation),  .D(di[15]),.Q(m_cr[3]));
   FDE_1 #(.INIT(1'b1))  i_m_cr4 (.C(clk2x),.CE(wr_saturation_d),.D(di[ 0]),.Q(m_cr[4]));
   FDE_1 #(.INIT(1'b1))  i_m_cr5 (.C(clk2x),.CE(wr_saturation_d),.D(di[ 1]),.Q(m_cr[5]));
   FDE_1 #(.INIT(1'b0))  i_m_cr6 (.C(clk2x),.CE(wr_saturation_d),.D(di[ 2]),.Q(m_cr[6]));
   FDE_1 #(.INIT(1'b1))  i_m_cr7 (.C(clk2x),.CE(wr_saturation_d),.D(di[ 3]),.Q(m_cr[7]));
   FDE_1 #(.INIT(1'b0))  i_m_cr8 (.C(clk2x),.CE(wr_saturation_d),.D(di[ 4]),.Q(m_cr[8]));
   FDE_1 #(.INIT(1'b0))  i_m_cr9 (.C(clk2x),.CE(wr_saturation_d),.D(di[ 5]),.Q(m_cr[9]));

//always @ (negedge clk2x) wr_quantizer_mode_d <= wr_quantizer_mode;

   FDE_1   i_coring_num0   (.C(clk2x),.CE(wr_quantizer_mode),.D(di[ 0]),.Q(coring_num[0]));
   FDE_1   i_coring_num1   (.C(clk2x),.CE(wr_quantizer_mode),.D(di[ 1]),.Q(coring_num[1]));
   FDE_1   i_coring_num2   (.C(clk2x),.CE(wr_quantizer_mode),.D(di[ 2]),.Q(coring_num[2]));

endmodule

// ----------------- submodules ---------------------

// [16] ==1 - set focus mode
// [15:14] 0 - none
//         1 - replace
//         2 - combine for all image
//         3 - combine in window only
// [13]==1 - enable color modes
// [12:9]== 0 - color, 4:2:0
//          1 - monochrome, 6/4 blocks (as 4:2:0)
//          2 - jp4, 6 blocks, original
//          3 - jp4, 6 blocks, dc -improved
//          4 - mono, 4 blocks (but still not actual monochrome JPEG as the blocks are scanned in 2x2 macroblocks)
//          5 - jp4,  4 blocks, dc-improved
//          6 - jp4,  differential
//          7 - 15 - reserved
// [8:7] == 0,1 - NOP, 2 -   disable, 3 - enable subtracting of average value (DC component), bypassing DCT
// [6] == 1 - enable quantization bank select, 0 - disregard bits [5:3]
// [5:3] = quantization page number (0..7)
// [2]== 1 - enable on/off control:
// [1:0]== 0 - reset compressor, stop immediately
//         1 - enable compressor, disable repetitive mode
//         2 - enable compressor, compress single frame
//         3 - enable compressor, enable repetitive mode

module compr_ifc   (clk,   // compressor input clock (1/2 of sclk)
//                    cwr,   // CPU write - global clock
                    sclk,   // system clock (120MHz)
                    cwe,   // we to compressor from CPU (sync to negedge sclk)
                    rs,      // 0 - bit modes,
                           // 1 - write ntiles;
                    di,     // [15:0] data from CPU (sync to negedge sclk)
                    cr_w,   // data written to cr (1 cycle long) - just to reset legacy IRQ
                    ntiles,//[17:0] - number of tiles in a frame to process
                    cmprs_en,    // enable compressor
                    cmprs_start, // single cycle when single or constant compression is turned on
                    cmprs_repeat,// high in repetitive mode
                    cmprs_qpage, // [2:0] - quantizator page number (0..7)
                    cmprs_dcsub, // subtract dc level before DCT, restore later
                    cmprs_mode,  // [3:0] - compressor mode
                    cmprs_shift, // tile shift from top left corner
                    cmprs_fmode, //[1:0] - focus mode
                    bayer_shift, // additional shift to bayer mosaic
                    is_compressing, // high from start of compressing till EOT (sync to posedge clk)
                    abort_compress,
                    force_flush); // abort compress - generate flush pulse, force end of image over DMA, update counter
                                  // single-cycle @ negedge sclk
//                    force_eot);  // sync to posedge clk - simulates eot to abort compression 

   input            clk,
                  sclk,
                  cwe;
   input          rs;
   input  [15:0]   di;
   output         cr_w;
   output [17:0]   ntiles;
   output         cmprs_en;    // enable compressor
   output         cmprs_start; // single cycle when single or constant compression is turned on
   output         cmprs_repeat;// high in repetitive mode
   output [ 2:0]  cmprs_qpage; // [2:0] - quantizator page number (0..7)
   output         cmprs_dcsub; // subtract dc level before DCT, restore later
   output [ 3:0]   cmprs_mode;  // [3:0] - compressor mode
   output [ 2:0]  cmprs_shift; // tile shift from top left corner
   output [ 1:0]   cmprs_fmode;  // [1:0] - focus mode
   output [ 1:0]  bayer_shift; // additional shift to bayer mosaic
   input          is_compressing; // high from start of compressing till EOT (sync to posedge clk)
   input          abort_compress;
   output         force_flush; // abort compress - generate flush pulse, force end of image over DMA, update counter
//   output          force_eot;  // sync to posedge clk - simulates eot to abort compression 


   reg            cmprs_en;    // enable compressor
   wire            cmprs_start; // single cycle when single or constant compression is turned on
   reg            cmprs_repeat;// high in repetitive mode
   reg    [ 2:0]  cmprs_qpage; // [2:0] - quantizator page number (0..7)
   reg            cmprs_dcsub; // subtract dc level before DCT, restore later
   reg    [ 3:0]  cmprs_mode;  // [3:0] - compressor mode
   reg    [ 2:0]  cmprs_shift; // tile shift from top left corner
   reg    [ 1:0]  cmprs_fmode;  // [1:0] - focus mode
   reg    [ 1:0]  bayer_shift; // additional shift to bayer mosaic
   wire   [ 1:0]  cmprs_start_c;

   wire            cmprs_en_s;    // enable compressor
   wire            cmprs_start_s; // single cycle when single or constant compression is turned on
   wire            cmprs_repeat_s;// high in repetitive mode
   wire    [ 2:0]  cmprs_qpage_s; // [2:0] - quantizator page number (0..7)
   wire            cmprs_dcsub_s; // subtract dc level before DCT, restore later
   wire    [ 3:0]  cmprs_mode_s;  // [3:0] - compressor mode
   wire    [ 2:0]  cmprs_shift_s; // tile shift from top left corner
   wire    [ 1:0]  cmprs_fmode_s;  // [1:0] - focus mode
   wire    [ 1:0]  bayer_shift_s; // additional shift to bayer mosaic




   reg  [23:0]   cr;
   reg  [17:0]   ntiles;
   reg  [15:0] ntiles0;
   reg  [17:0] ntiles1;
   reg  [17:0] ntiles1_prev;
   

   wire [23:0]   cri;
   wire  [1:0]   rcs;

   wire         cr_w;
   wire [1:0]   cr_wi;


   assign      rcs[1:0]={cwe && rs, cwe && ~rs};
   reg         rcs0_d, rcs0_dd, rcs1_d;
   reg [2:0]   rcs1d;

   reg    [1:0]   is_compressing_sclk; // sync to negedge sclk
   reg    [1:0]   is_compressing_or_flushing_sclk=2'h0; // includes flushing
   reg            force_flush; // abort compress - generate flush pulse, force end of image over DMA, update counter

 //  wire [1:0]  f_eot;
 //  wire        force_eot;
   reg       cmprs_en_long;   /// waits for compressor to finish gracefully with timeout enforced
   reg       cmprs_en_finish; /// finishing compressor flushing
   reg [6:0] cmprs_en_timeout;
   reg       cmprs_en_s_d;
   reg       force_flush1; ///force flush caused by turning compressor off
   always @ (negedge sclk) begin
     cmprs_en_s_d <= cmprs_en_s;
     cmprs_en_finish <= is_compressing_or_flushing_sclk[1] &&                  /// when compressor is still running
                        ((!cmprs_en_s && cmprs_en_s_d) ||                      /// and compressor is disabled by command 
                         (cmprs_en_finish && (cmprs_en_timeout[6:0]!= 7'h0))); /// timeout (maybe not needed?)
     if (!cmprs_en_finish) cmprs_en_timeout[6:0] <= 7'h50; //80
     else                  cmprs_en_timeout[6:0] <=cmprs_en_timeout[6:0]-1;
     cmprs_en_long <= cmprs_en_s || cmprs_en_s_d || cmprs_en_finish;                   
     force_flush1<= is_compressing_sclk[1] && /// is still compressing
                    !cmprs_en_s &&            /// first cycle when compressor enable is turned off
                     cmprs_en_s_d;
   end

   always @ (negedge sclk) begin
     rcs0_d  <= rcs[0];
//     rcs1_d  <= rcs[1];
     rcs1d[2:0]<={rcs1d[1:0],rcs[1]};
     rcs0_dd <= rcs0_d;
     is_compressing_or_flushing_sclk[1:0] <= {is_compressing_or_flushing_sclk[0],
                                              is_compressing | (abort_compress & is_compressing_or_flushing_sclk[1])};
     is_compressing_sclk[1:0]             <= {is_compressing_sclk[0], is_compressing};
//     force_flush <= is_compressing_sclk[1] && rcs[1];
///add double cycle if needed
     force_flush <= force_flush1 ||(rcs1d[2] && is_compressing_sclk[1] && (ntiles1_prev[17:0] != ntiles1[17:0]));
   end  

   FDE_1   i_cri00 (.C(sclk),.CE(rcs[0]),.D(di[ 0]),.Q(cri[ 0]));
   FDE_1   i_cri01 (.C(sclk),.CE(rcs[0]),.D(di[ 1]),.Q(cri[ 1]));
   FDE_1   i_cri02 (.C(sclk),.CE(rcs[0]),.D(di[ 2]),.Q(cri[ 2]));
   FDE_1   i_cri03 (.C(sclk),.CE(rcs[0]),.D(di[ 3]),.Q(cri[ 3]));
   FDE_1   i_cri04 (.C(sclk),.CE(rcs[0]),.D(di[ 4]),.Q(cri[ 4]));
   FDE_1   i_cri05 (.C(sclk),.CE(rcs[0]),.D(di[ 5]),.Q(cri[ 5]));
   FDE_1   i_cri06 (.C(sclk),.CE(rcs[0]),.D(di[ 6]),.Q(cri[ 6]));
   FDE_1   i_cri07 (.C(sclk),.CE(rcs[0]),.D(di[ 7]),.Q(cri[ 7]));
   FDE_1   i_cri08 (.C(sclk),.CE(rcs[0]),.D(di[ 8]),.Q(cri[ 8]));
   FDE_1   i_cri09 (.C(sclk),.CE(rcs[0]),.D(di[ 9]),.Q(cri[ 9]));
   FDE_1   i_cri10 (.C(sclk),.CE(rcs[0]),.D(di[10]),.Q(cri[10]));
   FDE_1   i_cri11 (.C(sclk),.CE(rcs[0]),.D(di[11]),.Q(cri[11]));
   FDE_1   i_cri12 (.C(sclk),.CE(rcs[0]),.D(di[12]),.Q(cri[12]));
   FDE_1   i_cri13 (.C(sclk),.CE(rcs[0]),.D(di[13]),.Q(cri[13]));
   FDE_1   i_cri14 (.C(sclk),.CE(rcs[0]),.D(di[14]),.Q(cri[14]));
   FDE_1   i_cri15 (.C(sclk),.CE(rcs[0]),.D(di[15]),.Q(cri[15]));

   FDE_1   i_cri16 (.C(sclk),.CE(rcs0_d),.D(di[ 0]),.Q(cri[16]));
   FDE_1 i_cri17 (.C(sclk),.CE(rcs0_d),.D(di[ 1]),.Q(cri[17]));
   FDE_1 i_cri18 (.C(sclk),.CE(rcs0_d),.D(di[ 2]),.Q(cri[18]));
   FDE_1 i_cri19 (.C(sclk),.CE(rcs0_d),.D(di[ 3]),.Q(cri[19]));
   FDE_1 i_cri20 (.C(sclk),.CE(rcs0_d),.D(di[ 4]),.Q(cri[20]));
   FDE_1 i_cri21 (.C(sclk),.CE(rcs0_d),.D(di[ 5]),.Q(cri[21]));
   FDE_1 i_cri22 (.C(sclk),.CE(rcs0_d),.D(di[ 6]),.Q(cri[22]));
   FDE_1 i_cri23 (.C(sclk),.CE(rcs0_d),.D(di[ 7]),.Q(cri[23]));
// just for legacy cr_w
   FDCE_1 i_cr_wi0 (.C(sclk),.CE(rcs0_d),.CLR(cr_w),.D(1'b1),           .Q(cr_wi[0]));
   FD       i_cr_wi1 (.C(clk),                       .D(cr_wi[0]        ),.Q(cr_wi[1]));
   FD       i_cr_w   (.C(clk),                       .D(cr_wi[1] & !cr_w),.Q(cr_w    ));

//   always @ (posedge clk) cr[23:0] <=cri[23:0];   // make it sync
   always @ (negedge sclk) if (rcs[1]) ntiles0[15:0] <= di[15:0];
//   always @ (negedge sclk) if (rcs1_d) ntiles1[17:0] <= {di[15:0],ntiles0[15:0]};
   always @ (negedge sclk) if (rcs1d[0]) ntiles1[17:0] <= {di[15:0],ntiles0[15:0]};
   always @ (negedge sclk) if (rcs1d[2]) ntiles1_prev[17:0] <= ntiles1[17:0];

//     rcs1d[2:0]<={rcs1d[1:0],rcs[1]};

// command decode

   FDE_1   i_cmprs_en_s       (.C(sclk),.CE(rcs0_dd && cri[2]), .D(cri[1:0]!=2'h0),.Q(cmprs_en_s));
   FDE_1   i_cmprs_repeat_s   (.C(sclk),.CE(rcs0_dd && cri[2]), .D(cri[1:0]==2'h3),.Q(cmprs_repeat_s));
   FD_1    i_cmprs_start_s    (.C(sclk),.D (rcs0_dd && cri[2] && ((cri[1:0]==2'h2) || ((cri[1:0]==2'h3) && ! cmprs_repeat_s ))),.Q(cmprs_start_s));
   FDE_1   i_cmprs_qpage_s0   (.C(sclk),.CE(rcs0_dd && cri[6]), .D(cri[ 3]),        .Q(cmprs_qpage_s[0]));
   FDE_1   i_cmprs_qpage_s1   (.C(sclk),.CE(rcs0_dd && cri[6]), .D(cri[ 4]),        .Q(cmprs_qpage_s[1]));
   FDE_1   i_cmprs_qpage_s2   (.C(sclk),.CE(rcs0_dd && cri[6]), .D(cri[ 5]),        .Q(cmprs_qpage_s[2]));
   FDE_1   i_cmprs_dcsub_s    (.C(sclk),.CE(rcs0_dd && cri[8]), .D(cri[ 7]),        .Q(cmprs_dcsub_s));
   FDE_1   i_cmprs_mode_s0    (.C(sclk),.CE(rcs0_dd && cri[13]),.D(cri[ 9]),        .Q(cmprs_mode_s[0]));
   FDE_1   i_cmprs_mode_s1    (.C(sclk),.CE(rcs0_dd && cri[13]),.D(cri[10]),        .Q(cmprs_mode_s[1]));
   FDE_1   i_cmprs_mode_s2    (.C(sclk),.CE(rcs0_dd && cri[13]),.D(cri[11]),        .Q(cmprs_mode_s[2]));
   FDE_1   i_cmprs_mode_s3    (.C(sclk),.CE(rcs0_dd && cri[13]),.D(cri[12]),        .Q(cmprs_mode_s[3]));
   FDE_1   i_cmprs_shift_s0   (.C(sclk),.CE(rcs0_dd && cri[17]),.D(cri[14]),        .Q(cmprs_shift_s[0]));
   FDE_1   i_cmprs_shift_s1   (.C(sclk),.CE(rcs0_dd && cri[17]),.D(cri[15]),        .Q(cmprs_shift_s[1]));
   FDE_1   i_cmprs_shift_s2   (.C(sclk),.CE(rcs0_dd && cri[17]),.D(cri[16]),        .Q(cmprs_shift_s[2]));
   FDE_1   i_bayer_shift_s0   (.C(sclk),.CE(rcs0_dd && cri[20]),.D(cri[18]),        .Q(bayer_shift_s[0]));
   FDE_1   i_bayer_shift_s1   (.C(sclk),.CE(rcs0_dd && cri[20]),.D(cri[19]),        .Q(bayer_shift_s[1]));
   FDE_1   i_cmprs_fmode_s0   (.C(sclk),.CE(rcs0_dd && cri[23]),.D(cri[21]),        .Q(cmprs_fmode_s[0]));
   FDE_1   i_cmprs_fmode_s1   (.C(sclk),.CE(rcs0_dd && cri[23]),.D(cri[22]),        .Q(cmprs_fmode_s[1]));

   
   always @ (posedge clk) begin
     ntiles[17:0] <=ntiles1[17:0];   // make it sync
///  cmprs_en         <= cmprs_en_s;
     cmprs_en         <= cmprs_en_long; // extended enable, waiting for the compressor to output data length
     cmprs_repeat     <= cmprs_repeat_s;
     cmprs_qpage[2:0] <= cmprs_qpage_s[2:0];
     cmprs_dcsub      <=cmprs_dcsub_s;
     cmprs_mode[3:0]  <=cmprs_mode_s[3:0];
     cmprs_shift[2:0] <=cmprs_shift_s[2:0];
     bayer_shift[1:0] <=bayer_shift_s[1:0];
     cmprs_fmode[1:0] <=cmprs_fmode_s[1:0];
   end  

   FDCE_1 i_cmprs_start_c0 (.C(sclk),.CE(cmprs_start_s),   .CLR(cmprs_start),.D(1'b1),           .Q(cmprs_start_c[0]));
   FD       i_cmprs_start_c1 (.C(clk),                       .D(cmprs_start_c[0]        ),.Q(cmprs_start_c[1]));
   FD       i_cmprs_start    (.C(clk),                       .D(cmprs_start_c[1] & !cmprs_start),.Q(cmprs_start   ));


endmodule

// syncronizes dcc data with dma1 output, adds 16..31 16-bit zero words for Axis DMA

module dcc_sync (//clk,
                 sclk,
                 dcc_en, // clk rising, sync with start of the frame
                 finish_dcc, // sclk rising
                 dcc_vld,    // clk rising
                 dcc_data, //[15:0] clk risimg
                 statistics_dv, //sclk
                 statistics_do //[15:0] sclk
                 );
//input         clk;
input         sclk, dcc_en, finish_dcc,dcc_vld;
input  [15:0] dcc_data;
output        statistics_dv;
output [15:0] statistics_do;
reg    [15:0] statistics_do;
reg           statistics_we;
reg           statistics_dv;
reg           dcc_run;
reg           dcc_finishing;
reg           skip16; // output just 16 zero words (data was multiple of 16 words)

reg    [ 4:0] dcc_cntr;

always @ (posedge sclk) begin
  dcc_run <= dcc_en;
  statistics_we <= dcc_run && dcc_vld && !statistics_we;
  statistics_do[15:0] <= statistics_we?dcc_data[15:0]:16'h0;
  statistics_dv <= statistics_we || dcc_finishing;
  skip16 <= finish_dcc && (statistics_dv?(dcc_cntr[3:0]==4'hf):(dcc_cntr[3:0]==4'h0) ); 
  if (!dcc_run)           dcc_cntr[3:0] <= 4'h0;
  else if (statistics_dv) dcc_cntr[3:0] <= dcc_cntr[3:0]+1; 
  dcc_cntr[4]   <= dcc_run && ((dcc_finishing && ((dcc_cntr[3:0]==4'hf)^dcc_cntr[4]) || skip16));
  dcc_finishing <= dcc_run && (finish_dcc   || (dcc_finishing && (dcc_cntr[4:1]!=4'hf)));

end

endmodule

