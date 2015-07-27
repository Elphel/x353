/*
** -----------------------------------------------------------------------------**
** camsync.v
**
** Synchronization between cameras using GPIO lines on th 10353 board:
**  - triggering from selected line(s) with filter;
**  - programmable delay to actual trigger (in pixel clock periods)
**  - Generating trigger output to selected GPIO line (and polarity)
**    or directly to the input delay generator (see bove)
**  - single/repetitive output with specified period in pixel clocks
**
** Copyright (C) 2007 Elphel, Inc
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

module camsync       (sclk, // @negedge
                         pre_wen, // 1 cycle ahead of write data
//                         di,      // [31:0] data in
                         di,      // [15:0] data in
                         wa,      // [1:0] write address
                           // 0 - source of trigger (12 bit pairs, LSB - level to trigger, MSB - use this bit). All 0 - internal trigger
                           //     in internal mode output has variable delay from the internal trigger (relative to sensor trigger)
                           // 1 - input trigger delay (pixel clocks) (NOTE: 0 - trigger disabled - WRONG)
                           // 2 - 24 bit pairs: MSB - enable selected line, LSB - level to send when trigger active
                           //     bit 25==1 some of the bits use test mode signals:
                           // 3 - output trigger period (duration constant of 256 pixel clocks). 
                           //     d==0 - disable (stop periodic mode)
                           //     d==1 - single trigger
                           //     d==2..255 - set output pulse / input-output serial bit duration (no start generated)
                           //     256>=d - repetitive trigger
                         pclk,    // pixel clock (global)
                         triggered_mode, // use triggered mode (0 - sensor is free-running)
                         trigrst,   // single-clock start of frame input (resets trigger output) posedge
                         gpio_in, // 12-bit input from GPIO pins
                         gpio_out,// 12-bit output to GPIO pins
                         gpio_out_en,// 12-bit output enable to GPIO pins
                         trigger1, // 1 cycle-long trigger output
                         trigger, // active high trigger to the sensor (reset by vacts)
                         overdue,     // prevents lock-up when no vact was detected during one period and trigger was toggled
                         ts_snd_en,   // enable sending timestamp over sync line
                         ts_external, // 1 - use external timestamp, if available. 0 - always use local ts
                         ts_snap,     // make a timestamp pulse  single @(posedge pclk)
                                      // timestamp should be valid in <16 pclk cycles
                         ts_snd_sec,  // [31:0] timestamp seconds to be sent over the sync line
                         ts_snd_usec, // [19:0] timestamp microseconds to be sent over the sync line
                         ts_rcv_sec,  // [31:0] timestamp seconds received over the sync line
                         ts_rcv_usec,// [19:0] timestamp microseconds received over the sync line
                         ts_stb);    // strobe when received timestamp is valid
    parameter PRE_MAGIC= 6'b110100;
    parameter POST_MAGIC=6'b001101;
    input         sclk;
    input           pre_wen;
//    input  [31:0] di;
    input  [15:0] di;
    input  [ 1:0] wa;
    input         pclk;
    input         triggered_mode; // use triggered mode (0 - sensor is free-running)
    input         trigrst; //vacts;
    input  [11:0] gpio_in;
    output [11:0] gpio_out;
    output [11:0] gpio_out_en;
    output        trigger1;
    output        trigger;
    output        overdue;

    input         ts_snd_en;   // enable sending timestamp over sync line
    input         ts_external; // 1 - use external timestamp, if available. 0 - always use local ts
    output        ts_snap;     // make a timestamp pulse (from the internal generator
                                      // timestamp should be valid in <16 pclk cycles
    
    input  [31:0] ts_snd_sec;  // [31:0] timestamp seconds to be sent over the sync line
    input  [19:0] ts_snd_usec; // [19:0] timestamp microseconds to be sent over the sync line
    output [31:0] ts_rcv_sec;  // [31:0] timestamp seconds received over the sync line
    output [19:0] ts_rcv_usec;// [19:0] timestamp microseconds received over the sync line
    output             ts_stb;    // strobe when received timestamp is valid



// delaying everything by 1 clock to reduce data fan in
    reg    [1:0]  wad;
    reg   [31:0]  did;
    reg           pre_wend;
    reg    [3:0]  wen;
    reg           high_zero;       // 24 MSBs are zero 
    reg    [11:0] input_use;       // 1 - use this bit
    reg    [11:0] input_pattern;   // data to be compared for trigger event to take place
    reg           pre_input_use_intern;// @(negedge sclk) Use internal trigger generator, 0 - use external trigger (also switches delay from input to output)
    reg           input_use_intern;//  @(posedge clk) 
    reg    [31:0] input_dly;       // delay value for the trigger
    reg    [11:0] gpio_out_en;     // which GPIO lines to drive
    reg    [11:0] gpio_active;     // output levels on the selected GPIO lines during output pulse (will be negated when inactive)
    reg           testmode;        // drive some internal signals to GPIO bits
    reg           outsync;         // during output active
    reg           out_data;        // output data (modulated with timestamp if enabled)
    reg    [31:0] repeat_period;    // restart period in repetitive mode
    reg           start,start_d;   // start single/repetitive output pulse(s)
    reg           rep_en;          // enable repetitive mode
    reg           start_en;
    wire          start_to_pclk;
    reg    [2:0]  start_pclk; // start and restart
    reg   [31:0]  restart_cntr; // restart period counter
    reg    [1:0]  restart_cntr_run; // restart counter running
    wire          restart;          // restart out sync
///AF:      reg    [8:0]  out_pulse_cntr;
    reg           trigger_condition; // GPIO input trigger condition met
    reg           trigger_condition_d; // GPIO input trigger condition met, delayed (for edge detection)
    reg           trigger_condition_filtered; // trigger condition filtered
//    reg           trigger_condition_filtered_d; // trigger condition filtered, delayed (for edge detection)
    reg    [6:0]  trigger_filter_cntr;
    reg           trigger1;
    wire          trigger1_dly16; // trigger1 delayed by 16 clk cycles to get local timestamp
//    reg           trigger;
    wire          trigger;       // for happy simulator
    reg           overdue;

    reg           start_dly;      // start delay (external input filtered or from internal single/rep)
    reg   [31:0]  dly_cntr;       // trigger delay counter
//    reg           dly_cntr_run;   // trigger delay counter running
    wire          dly_cntr_run;   // trigger delay counter running (to use FD for simulation)
    reg           dly_cntr_run_d; // trigger delay counter running - delayed by 1

    wire          pre_start_out_pulse;
    reg           start_out_pulse; /// start generation of output pulse. In internal trigger mode uses delay counter, in external - no delay
    
///AF:     reg           pre_start;
    reg   [31:0]  pre_period;
    reg   [ 7:0]  bit_length='hff; /// Output pulse duration or bit duration in timestamp mode
                                   /// input will be filtered with (bit_length>>2) duration
    wire  [ 7:0]  bit_length_plus1; // bit_length+1
    reg   [ 7:0]  bit_length_short; /// 3/4 bit duration, delay for input strobe from the leading edge.
                                   
    wire          pre_start0;
    reg           start0;
    wire          pre_set_bit;
    reg           set_bit;
    wire          pre_set_period;
    reg           set_period;
    wire          start_pclk16 ;// delayed start to wait for time stamp to be available

    reg   [31:0]  sr_snd_first;
    reg   [31:0]  sr_snd_second;

    reg   [31:0]  sr_rcv_first;
    reg   [31:0]  sr_rcv_second;
    reg   [ 7:0]  bit_snd_duration;
    reg   [ 5:0]  bit_snd_counter;
    reg   [ 7:0]  bit_rcv_duration;
    reg           bit_rcv_duration_zero; // to make it faster, duration always >=2
    reg   [ 6:0]  bit_rcv_counter; // includes "deaf" period ater receving
    reg           bit_snd_duration_zero; //    
//   input         ts_snd_en;   // enable sending timestamp over sync line
    reg           ts_snd_en_pclk;
    
    reg   [31:0]  ts_rcv_sec;  // timestamp seconds received over the sync line
    reg   [19:0]  ts_rcv_usec; // timestamp microseconds received over the sync line
    reg           rcv_run_or_deaf; // counters active
    wire          rcv_run;     // receive in progress, will always last for 64 bit_length+1 intervals before ready for the new input pulse
    reg           rcv_run_d;
    reg           rcv_done_rq; // request to copy time stamp (if it is not ready yet)
    reg           rcv_done_rq_d;
    reg           rcv_done; /// rcv_run ended, copy timestamp if requested
//    reg           rcv_deaf; // would not accept new trigger until this is over
//    reg           rcv_run_or_deaf; // counters active
    wire          pre_rcv_error;  // pre/post magic does not match, set ts to all ff-s
    reg           rcv_error;

    reg           ts_external_pclk; // 1 - use external timestamp (combines ts_external and input_use_intern)
    reg           triggered_mode_pclk;
    
    reg           ts_snap;
    reg           ts_stb;         // strobe when received timestamp is valid (single sclk cycle)
    wire          ts_stb_pclk;
    reg    [2:0]  ts_pre_stb;

//! in testmode GPIO[11] and GPIO[10] use internal signals instead of the outsync:
//! bit 11 - same as TRIGGER output to the sensor (signal to the sensor may be disabled externally)
//!          then that bit will be still from internall trigger to frame valid
//! bit 10 - dly_cntr_run (delay counter run) - active during trigger delay
    assign rcv_run=rcv_run_or_deaf && bit_rcv_counter[6];
    assign bit_length_plus1 [ 7:0] =bit_length[7:0]+1;

    assign        pre_start_out_pulse=input_use_intern?(dly_cntr_run_d && !dly_cntr_run):start_pclk16;



    assign  gpio_out[9: 0] = out_data? gpio_active[9: 0]: ~gpio_active[9: 0];
    assign  gpio_out[10] = (testmode? dly_cntr_run: out_data)? gpio_active[10]: ~gpio_active[10];
    assign  gpio_out[11] = (testmode? trigger:      out_data)? gpio_active[11]: ~gpio_active[11];
    assign  restart= restart_cntr_run[1] && !restart_cntr_run[0];
    
    assign  pre_set_bit=     (|did[31:8]==0) && |did[7:1]; // 2..255
    assign  pre_start0=       |did[31:0] && !pre_set_bit;
    assign  pre_set_period = !pre_set_bit;
    
    
    always @ (negedge sclk) begin
      pre_wend <= pre_wen;
      if (pre_wen)  wad[1:0] <= wa[1:0];
      if (pre_wen)  did[15: 0] <= di[15:0];
      if (pre_wend) did[31:16] <= di[15:0];

      wen[3:0] <= pre_wend?{(wad[1:0]==2'h3),(wad[1:0]==2'h2),(wad[1:0]==2'h1),(wad[1:0]==2'h0)}:4'b0;
      if (wen[0]) input_use[11:0] <= {did[23],did[21],did[19],did[17],did[15],did[13],did[11],did[9],did[7],did[5],did[3],did[1]};
      pre_input_use_intern <= (input_use[11:0]==12'h0); // use internal source for triggering
      if (wen[0]) input_pattern[11:0] <= {did[22],did[20],did[18],did[16],did[14],did[12],did[10],did[8],did[6],did[4],did[2],did[0]};
      if (wen[1]) input_dly[31:0] <= did[31:0];
      if (wen[2]) gpio_out_en[11:0] <= {did[23],did[21],did[19],did[17],did[15],did[13],did[11],did[9],did[7],did[5],did[3],did[1]};
      if (wen[2]) gpio_active[11:0] <= {did[22],did[20],did[18],did[16],did[14],did[12],did[10],did[8],did[6],did[4],did[2],did[0]};
      if (wen[2]) testmode <= did[24];
      
//      if (wen[3]) repeat_period[31:0] <= did[31:0];
      if (wen[3]) pre_period[31:0] <= did[31:0];
      if (wen[3]) high_zero <=        did[31:8]==24'b0;
      

//      start <= wen[3] && (did[31:0]!=32'h0);
      start0     <= wen[3] && pre_start0;
      set_bit    <= wen[3] && pre_set_bit;
      set_period <= wen[3] && pre_set_period;

      if (set_period) repeat_period[31:0] <= pre_period[31:0];
      if (set_bit)        bit_length[7:0] <= pre_period[ 7:0];
     
      start  <= start0;
      start_d <= start;

      start_en <= (repeat_period[31:0]!=0);
      if (set_period) rep_en <= !high_zero;
    end

    MSRL16 i_start_pclk16   (.Q(start_pclk16),   .A(4'hf), .CLK(pclk), .D(start_pclk[2]));
    MSRL16 i_strigger1_dly16(.Q(trigger1_dly16), .A(4'hf), .CLK(pclk), .D(trigger1));

//! synchronize start from sclk to pclk    
//! Generating repetition (period should be exactly N, not N+/-1) and output pulse
    FDCE_1 i_start_to_pclk (.C(sclk), .CE(start_d),.CLR(start_pclk[1] || !start_en),.D(1'b1),.Q(start_to_pclk));
    always @ (posedge pclk) begin
     ts_snap <=  (start_pclk[2] && ts_snd_en_pclk) || //strobe by internal generator if output timestamp is enabled
                 (trigger1 && !ts_external_pclk); // get local timestamp of trigger1 if it is used


      ts_snd_en_pclk<=ts_snd_en;
      input_use_intern <= pre_input_use_intern;
      ts_external_pclk<= ts_external && !input_use_intern;
     
//      start_pclk[1:0] <= {start_pclk[0] || (restart && rep_en), start_to_pclk && !start_pclk[0]};
      start_pclk[2:0] <= {(restart && rep_en) || (start_pclk[1] && !restart_cntr_run[1] && !restart_cntr_run[0] && !start_pclk[2]),
                          start_pclk[0],
                          start_to_pclk && !start_pclk[0]};
      restart_cntr_run[1:0] <= {restart_cntr_run[0],start_en && (start_pclk[2] || (restart_cntr_run[0] && (restart_cntr[31:2] !=0)))};
      if (restart_cntr_run[0]) restart_cntr[31:0] <= restart_cntr[31:0] - 1;
      else restart_cntr[31:0] <= repeat_period[31:0];
      
      start_out_pulse <= pre_start_out_pulse;
/// Generating output pulse - 64* bit_length if timestamp is disabled or
/// 64 bits with encoded timestamp, including pre/post magic for error detectrion
      outsync <= start_en && (start_out_pulse || (outsync && !((bit_snd_duration[7:0]==0) &&(bit_snd_counter[5:0]==0))));
      if (!outsync || (bit_snd_duration[7:0]==0)) bit_snd_duration[7:0] <= bit_length[7:0];
      else  bit_snd_duration[7:0] <= bit_snd_duration[7:0] - 1;
      bit_snd_duration_zero <= bit_snd_duration[7:0]==8'h1;

      if (!outsync) bit_snd_counter[5:0] <=ts_snd_en_pclk?63:3; /// when no ts serial, send pulse 4 periods long (max 1024 pclk)
      /// Same bit length (1/4) is used in input filter/de-glitcher
      else if (bit_snd_duration[7:0]==0)  bit_snd_counter[5:0] <=  bit_snd_counter[5:0] -1;

      if (!outsync)                       sr_snd_first[31:0]  <= {PRE_MAGIC,ts_snd_sec[31:6]};
//      else if (bit_snd_duration[7:0]==0)  sr_snd_first[31:0]  <={sr_snd_first[30:0],sr_snd_second[31]};
      else if (bit_snd_duration_zero)     sr_snd_first[31:0]  <={sr_snd_first[30:0],sr_snd_second[31]};
      if (!outsync)                       sr_snd_second[31:0] <= {ts_snd_sec[5:0], ts_snd_usec[19:0],POST_MAGIC};
//      else if (bit_snd_duration[7:0]==0)  sr_snd_second[31:0] <={sr_snd_second[30:0],1'b0};
      else if (bit_snd_duration_zero)     sr_snd_second[31:0] <={sr_snd_second[30:0],1'b0};
      out_data <=outsync && (ts_snd_en_pclk?sr_snd_first[31]:1'b1);
    end
 
 
 
// Detecting input sync pulse (filter - 64 pclk, pulse is 256 pclk)

//    FD i_dly_cntr_run (.C(pclk),.D(start_dly || (dly_cntr_run && (dly_cntr[31:0]!=0))),.Q(dly_cntr_run)); // for simulator to be happy
// even more for simulator
      FD i_dly_cntr_run (.C(pclk),.D(triggered_mode && (start_dly || (dly_cntr_run && (dly_cntr[31:0]!=0)))),.Q(dly_cntr_run)); // for simulator to be happy

//    FD i_trigger      (.C(pclk),.D(trigger1  || (trigger && !trigrst)),  .Q(trigger)); // for simulator to be happy
/// Now trigger1 toggles trigger output to prevent lock-up if no vacts
/// Lock-up could take place if:
/// 1 - Sesnoris in snapshot mode
/// 2 - trigger was applied before end of previous frame.
/// With implemented toggling 1 extra pulse can be missed (2 with the original missed one), but the system will not lock-up 
/// if the trigger pulses continue to come.

    assign pre_rcv_error= (sr_rcv_first[31:26]!=PRE_MAGIC) || (sr_rcv_second[5:0]!=POST_MAGIC);

    FD i_trigger      (.C(pclk),.D(trigrst?1'b0:(trigger1 ^ trigger)),  .Q(trigger)); // for simulator to be happy
    always @ (posedge pclk) begin
      if (trigrst)       overdue <= 1'b0;
      else if (trigger1) overdue <= trigger;

      triggered_mode_pclk<= triggered_mode;
      bit_length_short[7:0] <= bit_length[7:0]-bit_length_plus1[7:2]-1; // 3/4 of the duration

      trigger_condition <= (((gpio_in[11:0] ^ input_pattern[11:0]) & input_use[11:0]) == 12'b0);
      trigger_condition_d <= trigger_condition;
     
      if (!triggered_mode || (trigger_condition !=trigger_condition_d)) trigger_filter_cntr <= {1'b0,bit_length[7:2]};
      else if (!trigger_filter_cntr[6]) trigger_filter_cntr<=trigger_filter_cntr-1;
     
      if (input_use_intern) trigger_condition_filtered <= 1'b0;
      else if (trigger_filter_cntr[6]) trigger_condition_filtered <= trigger_condition_d;
      
                                     
      rcv_run_or_deaf <= start_en && (trigger_condition_filtered ||
//                                     (rcv_run_or_deaf && !((bit_rcv_duration[7:0]==0) &&(bit_rcv_counter[6:0]==0))));
                                     (rcv_run_or_deaf && !(bit_rcv_duration_zero  && (bit_rcv_counter[6:0]==0))));

      rcv_run_d <= rcv_run; 
      start_dly <= input_use_intern ? (start_pclk16 && start_en) : (rcv_run && !rcv_run_d);
// simulation problems w/o "start_en &&" ? 

      dly_cntr_run_d <= dly_cntr_run;
      if (dly_cntr_run) dly_cntr[31:0] <= dly_cntr[31:0] -1;
      else              dly_cntr[31:0] <= input_dly[31:0];
      trigger1 <= input_use_intern ? (start_pclk16 && start_en):(dly_cntr_run_d && !dly_cntr_run);/// bypass delay to trigger1 in internal trigger mode
/// 64-bit serial receiver (52 bit payload, 6 pre magic and 6 bits post magic for error checking
      if      (!rcv_run_or_deaf)         bit_rcv_duration[7:0] <= bit_length_short[7:0]; // 3/4 bit length-1
      else if (bit_rcv_duration[7:0]==0) bit_rcv_duration[7:0] <= bit_length[7:0];       // bit length-1
      else                               bit_rcv_duration[7:0] <= bit_rcv_duration[7:0]-1;
      bit_rcv_duration_zero <= bit_rcv_duration[7:0]==8'h1;
      if      (!rcv_run_or_deaf)         bit_rcv_counter[6:0]  <= 127;
//      else if (bit_rcv_duration[7:0]==0) bit_rcv_counter[6:0]  <= bit_rcv_counter[6:0] -1;
      else if (bit_rcv_duration_zero)    bit_rcv_counter[6:0]  <= bit_rcv_counter[6:0] -1;

//      if (rcv_run && (bit_rcv_duration[7:0]==0)) begin
      if (rcv_run && bit_rcv_duration_zero) begin
        sr_rcv_first[31:0]  <={sr_rcv_first[30:0],sr_rcv_second[31]}; 
        sr_rcv_second[31:0] <={sr_rcv_second[30:0],trigger_condition_filtered};
      end

      rcv_done_rq <= start_en && ((ts_external_pclk && trigger1_dly16) || (rcv_done_rq && rcv_run));
      rcv_done_rq_d <= rcv_done_rq;
      rcv_done <= rcv_done_rq_d && !rcv_done_rq;
//      triggered_mode_pclk<= triggered_mode;
      
      rcv_error <= pre_rcv_error;

      if (rcv_done) begin
        ts_rcv_sec  [31:0] <= {sr_rcv_first[25:0],sr_rcv_second[31:26]};
        ts_rcv_usec [19:0] <= rcv_error?20'hfffff:   sr_rcv_second[25:6];
      end else if (!triggered_mode_pclk || (!ts_external_pclk && trigger1_dly16 )) begin
        ts_rcv_sec  [31:0] <=  ts_snd_sec [31:0];
        ts_rcv_usec [19:0] <=  ts_snd_usec[19:0];
      end
//      ts_stb<= rcv_done || (!ts_external_pclk && trigger1_dly16 );    // strobe when received timestamp is valid
    end    

    FDCE i_ts_stb_pclk (.C(pclk), .CE(rcv_done || (!ts_external_pclk && trigger1_dly16 )),.CLR(ts_stb),.D(1'b1),.Q(ts_stb_pclk));
    always @ (negedge sclk) begin
      ts_pre_stb[2:0] <= {ts_pre_stb[1:0],ts_stb_pclk};
      ts_stb <= ts_pre_stb[1] && !ts_pre_stb[2];
    end
    
    
    
endmodule

