/*
** -----------------------------------------------------------------------------**
** sensortrig.v
**
** Synchronization/triggering controller for sensor readout
**dsat
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
module   sensortrig (pclk,
                     sclk,              // global
                     wlin,              // decoded address, write last line # in frame (since trigger in external trigger mode)
                     wcmd,              // decoded address, write command from d[3:0]:
// now wcmd, d[28:16] - number of lines to delay frame syn interrupt
                                        // d[3] - 0 skip lines, 1 - skip frames
                                        // d[2] - enable
                                        // d[1] - external (0 - internal)
                                        // d[0] - continuous
                     framesync_dly,     // write frame sync (vacts_out) delay in lines
                     d,                // [31:0][11:0] data in
                     compressed_frames, // [7:0] - bitmask of frames to compress (generate vacts15)
                     frame_num,         // [2:0] current frame number (switches after vacts_sclk)
                     trig,             // external trigger, 0->1 transition
                     hact,             // hact (line active)
                     vacts_in,         // single-cycle frame sync
                     vacts_out,        // single-cycle frame sync (divided if needed in photofinish mode)
                     vacts15,          // pulse from vacts active for 15 scan lines
                     sensor_en,        // enable sensor output
                     trig_v,           // [13:0] line number when trigger occured
                     trig_h,           // [13:0] pixel number when trigger occured
                     status,           // [2:0]: 00 - off, 01 - waiting for vacts to start, 10 - frame active, 11 - frame over
                     frame_run,        // active while frame is being acquired (include all line_run)
                     xfer_over_irq,    // pulse after transfer of specified number of lines (after trigger)
                     trig_irq,         // single-cycle (pclk) pulse after external trigger

                     fillfactory,      // fillfactory sensor mode
                     ystart,           // start of frame (fillfactory sensor mode) - 3 cycles long;
                     yclock            // start of line  (fillfactory sensor mode) - 1 cycle long;
                     
                   );

  input         pclk;
  input         sclk;
  input         wlin;
  input         wcmd;
  input [15:0]  d;
  
  input [ 7:0]  compressed_frames; //[7:0] - bitmask of frames to compress (generate vacts15)
  input [ 2:0]  frame_num;

  input         framesync_dly; // write frame sync (vacts_out) delay in lines
  input         trig;
  input         hact;
  input         vacts_in;
  output        vacts_out;
  output        vacts15;
  output        sensor_en;
  output [13:0] trig_v;
  output [13:0] trig_h; // can be 12'h800

  output [2:0] status;  // modify, MSB - "done"
  output       frame_run;
  output       xfer_over_irq;
  output       trig_irq;

  input         fillfactory;     // fillfactory sensor mode
  output        ystart;          // start of frame (fillfactory sensor mode) - 3 cycles long;
  output        yclock;          // start of line  (fillfactory sensor mode) - 1 cycle long;

  wire          fillfactory;     // fillfactory sensor mode
  reg           ystart;          // start of frame (fillfactory sensor mode) - 3 cycles long;
  wire          yclock;          // start of line  (fillfactory sensor mode) - 1 cycle long;


  wire         sensor_en;
  wire   [2:0] status;

  reg    [1:0] hact_dly;
  reg          nxt_line;
  reg          nxt_line1; // delayed

  wire         nxt_lf;  // next line/frame (depending on cmd[3]


  wire   state1,  // waiting vacts
         state2,  // got vacts, (==hact_en)
         state3;  // got external trigger (does not turn off state2)
  wire   trig0,   // sync to trig
         trig1,   // first registered to pclk (not used by itself)
         trig2;   // registered by pclk, latency 2
  wire   start0,  // sync to wclk
         start1,  // 1-st registered by pclk
         start;   // 2-nd registered by pclk

  wire   [3:0]    cmd;  // command register, written @wclk (wcmd=1'b1)
  reg    [13:0]   nlines;  // lines to acquire
  reg    [13:0]   vact_dly; //delay vact irq by this number of lines
  reg    [13:0]   vact_dly_cntr;


  reg    [13:0]   trig_v;
  reg    [13:0]   trig_h; // can be 12'h800
  reg    [13:0]   lines_left;
  wire            xfer_over;
  reg             xfer_over_irq;
  reg             done;
  wire            sync_wr,sync_wr0,sync_wr1; // writing nlines in ff mode;


  reg             frame_run;
  reg             pre_vacts_out; //at the beginning of the frame
  reg             pre_vacts_out_d;     // delayed by some 1 pre_vacts_out
  reg             vacts_out;     // delayed by some lines (0 - next cycle after  pre_vacts_out_d)
  reg             vacts_dly_on; // from pre_vacts till vacts;
  wire            wlin_pclks0;
  reg    [ 1:0]   wlin_pclks;
  reg    [13:0]   nff;     // photofinish mode - number of sensor frames in one "frame"
  reg    [13:0]   ff_count;  // frame in frame counter
  wire            ff_count_eq0;

  reg     [3:0]   vacts15_cntr;
  reg             vacts15;

  reg             set_lines_left;

  assign       yclock=nxt_line;
  assign       nxt_lf=(cmd[3])? pre_vacts_out: nxt_line; // next line/frame (depending on cmd[3]

  reg            en_vacts15_sclk,en_vacts15;
  reg    [7:0]   frame_num_1shot;
  always @ (negedge sclk) begin
    frame_num_1shot <={frame_num[2:0]==3'h7,
                       frame_num[2:0]==3'h6,
                       frame_num[2:0]==3'h5,
                       frame_num[2:0]==3'h4,
                       frame_num[2:0]==3'h3,
                       frame_num[2:0]==3'h2,
                       frame_num[2:0]==3'h1,
                       frame_num[2:0]==3'h0};
    en_vacts15_sclk <= | (frame_num_1shot[7:0] & compressed_frames[7:0]);                       
  end


  assign   ff_count_eq0= (ff_count[13:0]==14'h0);
  FDCE_1 i_wlin_pclks0  (.C(sclk),.CE(wlin),.CLR(wlin_pclks[1]), .D(1'b1), .Q(wlin_pclks0));
  always @ (posedge pclk) begin
    en_vacts15 <= en_vacts15_sclk;
    wlin_pclks[1:0] <= {wlin_pclks[0] && !wlin_pclks[1], wlin_pclks0};
    if (wlin_pclks[1] || (nff[13:0] == 14'h0) || (ff_count_eq0 && vacts_in)) ff_count[13:0] <= nff[13:0];
    else if (vacts_in) ff_count[13:0] <= ff_count[13:0]-1;
    pre_vacts_out <= vacts_in && ff_count_eq0;
    pre_vacts_out_d <= pre_vacts_out;
    if      (pre_vacts_out)            vact_dly_cntr[13:0] <= vact_dly[13:0];
    else if (nxt_line && vacts_dly_on) vact_dly_cntr[13:0] <= vact_dly_cntr[13:0] -1 ;
    if  (pre_vacts_out) vacts_dly_on <= 1'b1;
    else if (vacts_out) vacts_dly_on <= 1'b0;
    vacts_out <= !vacts_out && vacts_dly_on &&
       ((pre_vacts_out_d && (vact_dly[13:0]==14'h0)) ||
        (nxt_line &&   (vact_dly_cntr[13:0]==14'h0)));
// generate vacts15 - vacts_out delayed by 15 scan lines (before compressor could start)
    if      (vacts_out)            vacts15_cntr[3:0] <= 4'hf;
    else if (nxt_line && vacts15)  vacts15_cntr[3:0] <= vacts15_cntr[3:0] -1 ;
    if      (vacts_out)                              vacts15 <= en_vacts15;
    else if (nxt_line && (vacts15_cntr[3:0] ==4'h0)) vacts15 <= 1'b0 ;
  end

  FDE_1  i_cmd_0  (.C(sclk), .CE(wcmd), .D(d[0]), .Q(cmd[0]));
  FDE_1  i_cmd_1  (.C(sclk), .CE(wcmd), .D(d[1]), .Q(cmd[1]));
  FDE_1  i_cmd_2  (.C(sclk), .CE(wcmd), .D(d[2]), .Q(cmd[2]));
  FDE_1  i_cmd_3  (.C(sclk), .CE(wcmd), .D(d[3]), .Q(cmd[3]));

  always @ (negedge sclk) if (framesync_dly) vact_dly[13:0] <= d[13:0];


// write command synchronization
  FDC_1  i_start0 (.C(sclk),.CLR(start), .D(start0 || wcmd), .Q(start0));
  FDC    i_start1 (.C(pclk), .CLR(start), .D(start0),         .Q(start1));
  FD     i_start  (.C(pclk),              .D(start1),         .Q(start ));

// write wlin synchronization
  FDC_1  i_sync_wr0  (.C(sclk), .CLR(sync_wr), .D(sync_wr0 || (wlin && fillfactory)), .Q(sync_wr0));
  FDC    i_sync_wr1  (.C(pclk), .CLR(sync_wr), .D(sync_wr0),         .Q(sync_wr1));
  FD     i_sync_wr   (.C(pclk),                .D(sync_wr1),         .Q(sync_wr ));
  
// external trigger synchronization
// warnings for combinatorial input for trig ++++++++++++++++++++
  FDC i_trig0  (.C(trig),.CLR(trig2 || !state2 || state3), .D(state2 && !state3 && cmd[1]),.Q(trig0));
  FDC i_trig1  (.C(pclk), .CLR(trig2),                      .D(trig0),            .Q(trig1));
  FD  i_trig2  (.C(pclk),                                   .D(trig1),            .Q(trig2));

// state transitions (state3 and state2 are NOT mutually exclusive)
  FD  i_state1 (.C(pclk), .D(cmd[2] && (state1? (!pre_vacts_out) : ((xfer_over && cmd[0]) || (start && !state2)))), .Q(state1));
  FD  i_state2 (.C(pclk), .D(cmd[2] && (state2? (!xfer_over) : (state1 && pre_vacts_out))), .Q(state2));
  FD  i_state3 (.C(pclk), .D(cmd[2] && (state3? (!xfer_over) : ((state1 && pre_vacts_out && !cmd[1]) || trig2))), .Q(state3));

  always @ (negedge sclk) if (wlin && !d[14]) begin ///(d[14]==1) - write number of pixels in a line
    if   (d[15])  nff[13:0]    <= d[13:0];
    else          nlines[13:0] <= d[13:0];
  end 


  assign sensor_en=state2;
  always @ (posedge pclk) begin
    frame_run <= state2 && !state3;
    hact_dly[1:0] <= {hact_dly[0],hact};
    nxt_line      <= (hact_dly[1] && !hact_dly[0]) || (fillfactory && ystart && !nxt_line && !nxt_line1);
//  nxt_line1     <= nxt_line;
    nxt_line1     <= yclock;
//  sync_wr_d1    <= sync_wr;
//  sync_wr_d2    <= sync_wr_d1;
    ystart        <= (((hact_dly[0] && !hact && (lines_left[13:0]==14'b1)) || sync_wr) || ystart) && !nxt_line1 && fillfactory;  // 3 cycles long
//  count_h[11:0] <= hact_dly[1]? (count_h[11:0]+(!state3)):12'b0; // will stop counting
  end

  always @ (posedge pclk)
    if (pre_vacts_out && state1) trig_v[13:0] <= 14'b0;
    else if (nxt_line && state2 && !state3 && !trig2) trig_v[13:0] <= trig_v[13:0]+1;

  always @ (posedge pclk) if (state2 && !state3 && !trig2) begin
    if (!hact_dly[1]) trig_h[13:0] <= 14'b0;
    else              trig_h[13:0] <= trig_h[13:0]+1;
  end
    

  always @ (posedge pclk)
    set_lines_left <= state1 || (set_lines_left && !hact);
  always @ (posedge pclk)
//    if (fillfactory?(sync_wr || ((lines_left[13:0]==14'b0) && hact_dly[1] && !hact_dly[0])):state1) lines_left[13:0] <= nlines[13:0];
    if (fillfactory?(sync_wr || ((lines_left[13:0]==14'b0) && hact_dly[1] && !hact_dly[0])):set_lines_left) lines_left[13:0] <= nlines[13:0];
    else if (fillfactory ? nxt_line : (nxt_lf && (state3 || trig2))) lines_left[13:0] <= lines_left[13:0]-1;
//   assign xfer_over=state3 && (lines_left[13:0]==14'h001) && nxt_lf;  
   assign xfer_over=state3 && (((lines_left[13:0]==14'h001) && nxt_lf) ||
                    (vacts_in && ff_count_eq0));  // abort input frame at the (next) frame sync
//vacts_in && ff_count_eq0
  always @ (posedge pclk) xfer_over_irq <= xfer_over;

  always @ (posedge pclk) done <= !start && (done || xfer_over);

  assign status[2:0]= {done,state2,state1 || state3};
  assign trig_irq=trig2;
//  assign hact_en=state2;

endmodule

