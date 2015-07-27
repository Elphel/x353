module  imu_logger   ( clk,   // system clock, negedge
                       xclk,  // half frequency (80 MHz nominal)
                       we,    // write enable (lower 16 bits, high - next cycle)
                       wa,    // write address(1)/data(0)
                       di,    // 16-bit data in (32 multiplexed)
                       usec,  // un-latched timestamp microseconds
                       sec,   // un-latched timestamp seconds
                       ext_di,
                       ext_do,
                       ext_en,
                       ts_rcv_sec,  // [31:0] timestamp seconds received over the sync line
                       ts_rcv_usec, // [19:0] timestamp microseconds received over the sync line
                       ts_stb,      // strobe when received timestamp is valid - single negedge sclk cycle
                       data_out,    // 16-bit data out to DMA1 (@negedge clk)
                       data_out_stb,// data out valid (@negedge clk)
                       sample_counter, // could be DMA latency, safe to use sample_counter-1
                       debug_state
                       );
                       
  input         clk;   // system clock, negedge
  input         xclk;  // half frequency (80 MHz nominal)
  input         we;    // write enable (lower 16 bits, high - next cycle)
  input         wa;    // write address(1)/data(0)
  input  [15:0] di;    // 16-bit data in (32 multiplexed)
  input  [19:0] usec;  // latched timestamp microseconds
  input  [31:0] sec;   // latched timestamp seconds
  input  [11:0] ext_di; // external GPIO in
  output [11:0] ext_do; // external GPIO out
  output [11:0] ext_en; // external GPIO enable out
  input  [31:0] ts_rcv_sec;  // [31:0] timestamp seconds received over the sync line
  input  [19:0] ts_rcv_usec; // [19:0] timestamp microseconds received over the sync line
  input         ts_stb;      // strobe when received timestamp is valid - single negedge sclk cycle
  output [15:0] data_out;    // 16-bit data out to DMA1 (@negedge clk)
  output        data_out_stb;// data out valid (@negedge clk)
  output [23:0] sample_counter; // could be DMA latency, safe to use sample_counter-1
  output [31:0] debug_state;

  wire         ser_di;      // gps serial data in
  wire         gps_pulse1sec;
  wire         mosi;  // to IMU, bit 2 in J9
  wire         miso;  // from IMU, bit 3 on J9 
  wire         sda, sda_en, scl, scl_en;

  reg    [6:0]  ctrl_addr=7'h0; // 0 - period, 1 - reserved, 2..31 - registers to log, >32 - gps parameters, >64 - odometer message
  reg           we_d; // only if wa was 0
  reg           we_imu;
  reg           we_gps;
  reg           we_period;
  reg           we_bit_duration;
  reg           we_message;
  reg           we_config;
  reg           we_config_imu; // bits 1:0, 2 - enable slot[1:0]
  reg           we_config_gps; // bits 6:3, 7 - enable - {ext,invert, slot[1:0]} slot==0 - disable
  reg           we_config_msg; // bits 12:8,13 - enable - {invert,extinp[3:0]} extinp[3:0]=='hf' - disable
  reg           we_config_syn; // bit  14,  15 - enable  - enable logging external timestamps

//  reg           we_config_rst; // bit  16,  17 - enable - reset modules 
//  reg           we_config_debug; // bits  21:18, 22 - enable
  reg    [15:0] di_d;
//  reg           di_d2; 

  reg    [1:0]  config_imu;
  reg    [3:0]  config_gps;
  reg    [4:0]  config_msg;
  reg           config_syn;
  reg           config_rst;
  reg    [3:0]  config_debug;

  reg    [1:0]  config_imu_mclk;
  reg    [3:0]  config_gps_mclk;
  reg    [4:0]  config_msg_mclk;
  reg           config_syn_mclk;
  reg           config_rst_mclk;
  reg    [3:0]  config_debug_mclk;

  reg    [1:0]  config_imu_pre;
  reg    [3:0]  config_gps_pre;
  reg    [4:0]  config_msg_pre;
  reg           config_syn_pre;
  reg           config_rst_pre;
  reg    [3:0]  config_debug_pre;
 
  reg   [15:0]  bitHalfPeriod;//  serial gps speed - number of xclk pulses in half bit period
  reg           we_bitHalfPeriod;
  reg   [15:0]  bitHalfPeriod_mclk;

  reg           enable_gps;
  reg           enable_msg;
  reg           enable_syn;
  reg           enable_timestamps;
  wire          message_trig;

  reg          ts_stb_rq;
  reg    [1:0] ext_ts_stb;

  wire        gps_ts_stb, ser_do,ser_do_stb;
  wire [15:0] imu_data;
  wire [15:0] nmea_data;
  wire [15:0] extts_data;
  wire [15:0] msg_data;

  wire [15:0] timestamps_rdata; // multiplexed timestamp data

  reg   [2:0] gps_pulse1sec_d;
  reg   [1:0] gps_pulse1sec_denoise;
  reg   [7:0] gps_pulse1sec_denoise_count;
  reg         gps_pulse1sec_single;
//  wire        gps_ts; // single cycle @posedge xclk

  wire  [3:0] timestamp_request; // 0 - imu, 1 - gps, 2 - ext, 3 - msg
  wire  [3:0] timestamp_ackn;

  wire [23:0] sample_counter;// could be DMA latency, safe to use sample_counter-1
  wire  [3:0] timestamp_request_long; //from sub-module ts request until reset by arbiter, to allow  timestamp_ackn
  wire  [3:0] channel_ready;  // 0 - imu, 1 - gps, 2 - ext, 3 - msg
  wire  [3:0] channel_next;   // 0 - imu, 1 - gps, 2 - ext, 3 - msg
  wire  [1:0] channel;        // currently logged channel number
  wire  [1:0] timestamp_sel;  // selected word in timestamp (0..3)
  wire        ts_en;          // log timestamp (when false - data)
  wire        mux_data_valid; // data valid from multiplexer (to xclk->clk converter fifo)
  reg  [15:0] mux_data_source;// data multiplexed from 1 of the 4 channels
  reg         mux_rdy_source; // data ready multiplexed from 1of the 4 channels (to fill rest with zeros)
  reg  [15:0] mux_data_final; // data multiplexed between timestamps and channel data (or 0 if ~ready)
  wire [15:0] data_out;    // 16-bit data out to DMA1 (@negedge clk)
  wire        data_out_stb;// data out valid (@posegedge clk)

  wire        rs232_wait_pause;// may be used as reset for decoder
  wire        rs232_start;          // serial character start (single pulse)
  wire        nmea_sent_start;          // serial character start (single pulse)
  reg         pre_message_trig;

 // reg  [1:0]  debug_reg;
  reg [7:0] dbg_cntr;


  assign ext_en[11:0]= {5'b0,(config_imu[1:0]==2'h3)?1'b1:1'b0,1'b0,(config_imu[1:0]==2'h2)?1'b1:1'b0,1'b0,(config_imu[1:0]==2'h1)?1'b1:1'b0,(config_imu[1:0]!=2'h0)?{sda_en,scl_en}:2'h0};
  assign ext_do[11:0]= {5'b0,(config_imu[1:0]==2'h3)?mosi:1'b0,1'b0,(config_imu[1:0]==2'h2)?mosi:1'b0,1'b0,(config_imu[1:0]==2'h1)?mosi:1'b0,(config_imu[1:0]!=2'h0)?{sda,scl}:2'h0};
  assign miso=         config_imu[1]?(config_imu[0]?ext_di[7] :ext_di[5]):(config_imu[0]?ext_di[3]:1'b0);
  assign ser_di=       config_gps[1]?(config_gps[0]?ext_di[6] :ext_di[4]):(config_gps[0]?ext_di[2]:1'b0);
  assign gps_pulse1sec=config_gps[2]^(config_gps[1]?(config_gps[0]?ext_di[7] :ext_di[5]):(config_gps[0]?ext_di[3]:1'b0));

//sngl_wire  
  
  always @(config_msg[3:0] or ext_di[11:0])  begin
    case (config_msg[3:0])
      4'h0:   pre_message_trig = ext_di[0];
      4'h1:   pre_message_trig = ext_di[1];
      4'h2:   pre_message_trig = ext_di[2];
      4'h3:   pre_message_trig = ext_di[3];
      4'h4:   pre_message_trig = ext_di[4];
      4'h5:   pre_message_trig = ext_di[5];
      4'h6:   pre_message_trig = ext_di[6];
      4'h7:   pre_message_trig = ext_di[7];
      4'h8:   pre_message_trig = ext_di[8]; // internal optocoupler, use invert 5'h18
      4'h9:   pre_message_trig = ext_di[9];
      4'ha:   pre_message_trig = ext_di[10];// external optocoupler, use invert 5'h1a
      4'hb:   pre_message_trig = ext_di[10];
      default:pre_message_trig = 1'b0;
    endcase
  end 
  assign message_trig= config_msg[4]^pre_message_trig;

  assign timestamp_request[1]=config_gps[3]? (config_gps[2]?nmea_sent_start:gps_ts_stb):gps_pulse1sec_single;
 

// filter gps_pulse1sec
  always @ (posedge xclk) begin
    if  (config_rst) gps_pulse1sec_d[2:0] <= 3'h0;
    else      gps_pulse1sec_d[2:0] <= {gps_pulse1sec_d[1:0], gps_pulse1sec};
    
    if      (config_rst)                      gps_pulse1sec_denoise[0] <= 1'b0;
    else if (gps_pulse1sec_denoise_count[7:0]==8'h0) gps_pulse1sec_denoise[0] <= gps_pulse1sec_d[2];
    
    if (gps_pulse1sec_d[2]==gps_pulse1sec_denoise[0]) gps_pulse1sec_denoise_count[7:0] <= 8'hff;
    else                            gps_pulse1sec_denoise_count[7:0] <= gps_pulse1sec_denoise_count[7:0] - 1;
    
    gps_pulse1sec_denoise[1] <= gps_pulse1sec_denoise[0];
    gps_pulse1sec_single <= !gps_pulse1sec_denoise[1] && gps_pulse1sec_denoise[0];
  end


// re-sync single pulse @ negedge sclk - ts_stb to @posedge xclk
  always @ (posedge ext_ts_stb[1] or negedge clk) begin
    if (ext_ts_stb[1])        ts_stb_rq <= 1'b0;
    else if (config_rst_mclk) ts_stb_rq <= 1'b0;
    else if (ts_stb)          ts_stb_rq <= 1'b1;
  end
  always @ (posedge xclk) begin
     ext_ts_stb[1:0] <= {ext_ts_stb[0] & ~ext_ts_stb[1],ts_stb_rq};
  end

  always @ (negedge clk) begin
    if (we)      di_d[15:0] <= di[15:0];
//    di_d2 <=di_d[0];
//    we_d       <= we && !wa;
    we_d       <= we && !wa;
    we_imu     <= we && !wa && (ctrl_addr[6:5] == 2'h0);
    we_gps     <= we && !wa && (ctrl_addr[6:5] == 2'h1);
    we_message <= we && !wa && (ctrl_addr[6:5] == 2'h2);
//    we_timer[4:0] <=  {we_timer[3:0], we && !wa && (ctrl_addr[5:0]==6'h0)} ;
    we_period  <=      we && !wa && (ctrl_addr[6:0]==7'h0);
    we_bit_duration <= we && !wa && (ctrl_addr[6:0]==7'h1);
    we_bitHalfPeriod<= we && !wa && (ctrl_addr[6:0]==7'h2);
    we_config      <=  we && !wa && (ctrl_addr[6:0]==7'h3);
    we_config_imu  <=  we && !wa && (ctrl_addr[6:0]==7'h3) && di[ 2];
    we_config_gps  <=  we && !wa && (ctrl_addr[6:0]==7'h3) && di[ 7];
    we_config_msg  <=  we && !wa && (ctrl_addr[6:0]==7'h3) && di[13];
    we_config_syn  <=  we && !wa && (ctrl_addr[6:0]==7'h3) && di[15];
//    we_config_rst  <=  we_config && di[1];

    if (we_config_imu) config_imu_mclk[1:0] <= di_d[ 1:0]; // bits 1:0, 2 - enable slot[1:0]
    if (we_config_gps) config_gps_mclk[3:0] <= di_d[ 6:3]; // bits 6:3, 7 - enable - {ext,inver, slot[1:0]} slot==0 - disable
    if (we_config_msg) config_msg_mclk[4:0] <= di_d[12:8]; // bits 12:8,13 - enable - {invert,extinp[3:0]} extinp[3:0]=='hf' - disable
    if (we_config_syn) config_syn_mclk      <= di_d[  14]; // bit  14,  15 - enable
    if (we_config && di[1]) config_rst_mclk <= di[0]; // bit  16,  17 - enable

    if (we_config && di[6]) config_debug_mclk[3:0] <= di[5:2]; // bit  21:18,  22 - enable
    
    if (we_bitHalfPeriod) bitHalfPeriod_mclk[15:0]<=di_d[15:0];

    if      (we && wa) ctrl_addr[6:5] <= di[6:5];
    if      (we && wa) ctrl_addr[4:0] <= di[4:0];
    else if (we_d && (ctrl_addr[4:0]!=5'h1f)) ctrl_addr[4:0] <=ctrl_addr[4:0]+1; // no roll over, 
    
  end

  always @ (posedge xclk) begin
    bitHalfPeriod[15:0] <= bitHalfPeriod_mclk[15:0];
    config_imu_pre[1:0] <= config_imu_mclk[1:0];
    config_gps_pre[3:0] <= config_gps_mclk[3:0];
    config_msg_pre[4:0] <= config_msg_mclk[4:0];
    config_syn_pre      <= config_syn_mclk;
    config_rst_pre      <= config_rst_mclk;
    config_debug_pre[3:0] <= config_debug_mclk[3:0];

    config_imu[1:0] <= config_imu_pre[1:0];
    config_gps[3:0] <= config_gps_pre[3:0];
    config_msg[4:0] <= config_msg_pre[4:0];
    config_syn      <= config_syn_pre;
    config_rst      <= config_rst_pre;
    config_debug[3:0] <= config_debug_pre[3:0];

    enable_gps         <= (config_gps[1:0] != 2'h0) && !config_rst;
    enable_msg         <= (config_gps[3:0] != 4'hf) && !config_rst;
    enable_syn         <= config_syn && !config_rst;
    enable_timestamps  <= !config_rst;
  end

  always @ (posedge xclk) begin
    mux_data_source[15:0] <= channel[1]?(channel[0]?msg_data[15:0]:extts_data[15:0]):(channel[0]?nmea_data[15:0]:imu_data[15:0]);
    mux_rdy_source        <= channel[1]?(channel[0]?channel_ready[3]:channel_ready[2]):(channel[0]?channel_ready[1]:channel_ready[0]);
    mux_data_final[15:0]  <= ts_en? timestamps_rdata[15:0]:(mux_rdy_source?mux_data_source[15:0]:16'h0); // replace 16'h0 with some pattern to debug output
  end

imu_spi i_imu_spi ( .sclk(clk),  // system clock, negedge
                    .xclk(xclk),  // half frequency (80 MHz nominal)
                    .we_ra(we_imu), // write enable for registers to log (@negedge clk)
                    .we_div(we_bit_duration),// write enable for clock dividing(@negedge clk)
                    .we_period(we_period),// write enable for IMU cycle period(@negedge clk) 0 - disable, 1 - single, >1 - half bit periods
                    .wa(ctrl_addr[4:0]),    // write address for register (5 bits, @negedge clk)
                    .di(di[15:0]),    // 16?-bit data in  (di, not di_d)
                    .mosi(mosi),  // to IMU, bit 2 in J9
                    .miso(miso),  // from IMU, bit 3 on J9 
                    .config_debug(config_debug[3:0]),
                    .sda(sda),   // sda, shared with i2c, bit 1
                    .sda_en(sda_en), // enable sda output (when sda==0 and 1 cycle after sda 0->1)
                    .scl(scl),   // scl, shared with i2c, bit 0
                    .scl_en(scl_en), // enable scl output (when scl==0 and 1 cycle after sda 0->1)
//                    .sngl_wire(sngl_wire), // single wire clock/data for the 103695 rev A

                    .ts(timestamp_request[0]),    // timestamop request
                    .rdy(channel_ready[0]),    // data ready
                    .rd_stb(channel_next[0]), // data read strobe (increment address)
                    .rdata(imu_data[15:0])); // data out (16 bits)
/*
logs events from odometer (can be software triggered), includes 56-byte message written to the buffer
So it is possible to assert trig input (will request timestamp), write message by software, then
de-assert the trig input - message with the timestamp will be logged
fixed-length de-noise circuitry with latency 256*T(xclk) (~3usec)
*/
imu_message i_imu_message(.sclk(clk),   // system clock, negedge
                       .xclk(xclk),  // half frequency (80 MHz nominal)
                       .we(we_message),    // write enable for registers to log (@negedge sclk), with lower data half
                       .wa(ctrl_addr[3:0]),    // write address for register (4 bits, @negedge sclk)
                       .di(di[15:0]),    // 16-bit data in  multiplexed 
                       .en(enable_msg),    // enable module operation, if 0 - reset
                       .trig(message_trig),  // leading edge - sample time, trailing set rdy
                       .ts(timestamp_request[3]),    // timestamop request
                       .rdy(channel_ready[3]),    // data ready
                       .rd_stb(channel_next[3]), // data read strobe (increment address)
                       .rdata(msg_data[15:0])); // data out (16 bits)
/* logs frame synchronization data from other camera (same as frame sync) */
// ts_stb (mclk) -> trig)
imu_exttime i_imu_exttime(.xclk(xclk),  // half frequency (80 MHz nominal)
                       .en(enable_syn),    // enable module operation, if 0 - reset
                       .trig(ext_ts_stb[1]),  // external time stamp updated, single pulse @posedge xclk
                       .usec(ts_rcv_usec[19:0]),  // microseconds from external timestamp (should not chnage after trig for 10 xclk)
                       .sec(ts_rcv_sec[31:0]),   // seconds from external timestamp
                       .ts(timestamp_request[2]),    // timestamop request
                       .rdy(channel_ready[2]),    // data ready
                       .rd_stb(channel_next[2]), // data read strobe (increment address)
                       .rdata(extts_data[15:0])); // data out (16 bits)

imu_timestamps i_imu_timestamps (
                        .sclk(clk), // 160MHz, negedge            
                        .xclk(xclk), // 80 MHz, posedge
                        .rst(!enable_timestamps),  // reset (@posedge xclk)
                        .sec(sec[31:0]),  // running seconds (@negedge sclk)
                        .usec(usec[19:0]), // running microseconds (@negedge sclk)
                        .ts_rq(timestamp_request_long[3:0]),// requests to create timestamps (4 channels), @posedge xclk
                        .ts_ackn(timestamp_ackn[3:0]), // timestamp for this channel is stored
                        .ra({channel[1:0],timestamp_sel[1:0]}),   // read address (2 MSBs - channel number, 2 LSBs - usec_low, (usec_high ORed with channel <<24), sec_low, sec_high
                        .dout(timestamps_rdata[15:0]));// output data
wire [0:0] debug_state_unused; // SuppressThisWarning Veditor UNUSED
rs232_rcv i_rs232_rcv (.xclk(xclk),           // half frequency (80 MHz nominal)
                       .bitHalfPeriod(bitHalfPeriod[15:0]),  // half of the serial bit duration, in xclk cycles
                       .ser_di(ser_di),              // rs232 (ttl) serial data in
                       .ser_rst(!enable_gps),        // reset (force re-sync)
                       .ts_stb(gps_ts_stb),          // strobe timestamp (start of message) (reset bit counters in nmea decoder)
                       .wait_just_pause(rs232_wait_pause),// may be used as reset for decoder
                       .start(rs232_start),          // serial character start (single pulse)
                       
                       .ser_do(ser_do),         // serial data out(@posedge xclk) LSB first!
                       .ser_do_stb(ser_do_stb),    // output data strobe (@posedge xclk), first cycle after ser_do becomes valid
//                       .debug(debug_state[4:0]),
                       .debug({debug_state_unused,debug_state[15:12]}),
                       .bit_dur_cntr(debug_state[31:16]),
                       .bit_cntr(debug_state[11:7])
                       );
//  output [15:0] debug_state;
// reg [7:0] dbg_cntr;
//    assign debug_state[15:12]=3'b0;
    assign debug_state[6:0] = dbg_cntr [6:0];

 always @ (posedge xclk) begin
   if (!enable_gps) dbg_cntr[7:0] <= 8'h0;
//   else if (ser_do_stb) dbg_cntr[7:0] <= dbg_cntr[7:0]+1;
   else if (rs232_start) dbg_cntr[7:0] <= dbg_cntr[7:0]+1;
 end
nmea_decoder i_nmea_decoder (.sclk(clk),   // system clock, @negedge
                             .we(we_gps),     // registers write enable (@negedge sclk)
                             .wa(ctrl_addr[4:0]),     // registers write adderss
                             .wd(di_d[7:0]),     // write data
                             .xclk(xclk),   // 80MHz, posedge
                             .start(gps_ts_stb),  // start of the serial message
                             .rs232_wait_pause(rs232_wait_pause),// may be used as reset for decoder
                             .start_char(rs232_start),           // serial character start (single pulse)
                             .nmea_sent_start(nmea_sent_start),  // serial character start (single pulse)
                             .ser_di(ser_do), // serial data in (LSB first)
                             .ser_stb(ser_do_stb),// serial data strobe, single-cycle, first cycle after ser_di valid
                             .rdy(channel_ready[1]),    // encoded nmea data ready
                             .rd_stb(channel_next[1]), // encoded nmea data read strobe (increment address)
                             .rdata(nmea_data[15:0]), // encoded data (16 bits)
                             .ser_rst(!enable_gps),        // reset (now only debug register)
//                             .debug(debug_state[31:8])
//                             .debug(debug_state[15:8])
                             .debug()
                             );
                             

logger_arbiter i_logger_arbiter(.xclk(xclk), // 80 MHz, posedge
                                .rst(config_rst), // module reset
                                .ts_rq_in(timestamp_request[3:0]),      // in requests for timestamp (single-cycle - just leading edge )
                                .ts_rq(timestamp_request_long[3:0]),         // out request for timestamp, to timestmp module
                                .ts_grant(timestamp_ackn[3:0]),      // granted ts requests from timestamping module
                                .rdy(channel_ready[3:0]),           // channels ready (leading edge - became ready, trailing - no more data, use zero)
                                .nxt(channel_next[3:0]),           // pulses to modules to output next word
                                .channel(channel[1:0]),       // decoded channel number (2 bits)
                                .ts_sel(timestamp_sel[1:0]),        // select timestamp word to be output (0..3)
                                .ts_en(ts_en),         // 1 - use timestamp, 0 - channel data (or 16'h0 if !ready)
                                .dv(mux_data_valid),            // output data valid (from registered mux - 2 stage - first selects data and ready, second ts/data/zero)
                                .sample_counter(sample_counter));// number of 64-byte samples logged

buf_xclk_mclk16 i_buf_xclk_mclk16(.xclk(xclk), // posedge
                                  .mclk(clk), // posedge!
                                  .rst(config_rst),  // @posedge xclk
                                  .din(mux_data_final[15:0]),
                                  .din_stb(mux_data_valid),
                                  .dout(data_out[15:0]),
                                  .dout_stb(data_out_stb));

endmodule

module logger_arbiter(xclk, // 80 MHz, posedge
                      rst,          // module reset
                      ts_rq_in,    // in requests for timestamp (single-cycle - just leading edge )
                      ts_rq,        // out request for timestamp, to timestmp module
                      ts_grant,     // granted ts requests from timestamping module
                      rdy,          // channels ready (leading edge - became ready, trailing - no more data, use zero)
                      nxt,          // pulses to modules to output next word
                      channel,      // decoded channel number (2 bits)
                      ts_sel,       // select timestamp word to be output (0..3)
                      ts_en,        // 1 - use timestamp, 0 - channel data (or 16'h0 if !ready)
                      dv,           // output data valid (from registered mux - 2 stage - first selects data and ready, second ts/data/zero)
                      sample_counter);// number of 64-byte samples logged

  input         xclk;  // half frequency (80 MHz nominal)
  input         rst;   // reset module
  input  [ 3:0] ts_rq_in; // in requests for timestamp (sinlgle-cycle)
  output [ 3:0] ts_rq;        // out request for timestamp, to timestmp module
  input  [ 3:0] ts_grant;     // granted ts requests from timestamping module
  input  [ 3:0] rdy;          // channels ready (leading edge - became ready, trailing - no more data, use zero)
  output [ 3:0] nxt;          // pulses to modules to output next word
  output [ 1:0] channel;      // decoded channel number (2 bits)
  output [ 1:0] ts_sel;       // select timestamp word to be output (0..3)
  output        ts_en;        // 1 - use timestamp, 0 - channel data (or 16'h0 if !ready)
  output        dv;           // output data valid (from registered mux - 2 stage - first selects data and ready, second ts/data/zero)
  output [23:0] sample_counter;// number of 64-byte samples logged

  reg    [3:0] ts_rq_in_d;
  reg    [3:0] ts_rq;
  reg    [3:0] ts_valid;
//  reg    [3:0] ts_rq_reset;
  reg    [3:0] channels_ready;// channels granted and ready
  reg    [3:1] chn1hot;       // channels 1-hot - granted and ready, priority applied
  reg          rq_not_zero;   // at least one channel is ready for processing (same time as chn1hot[3:0])
  reg    [1:0] channel;
///AF:  reg          start;
  reg          busy;
  wire         wstart;
  reg          ts_en;
  reg    [4:0] seq_cntr;
  reg          seq_cntr_last;
  reg    [1:0] ts_sel;
  reg          dv;
  reg          inc_sample_counter;
  reg   [23:0] sample_counter;// number of 64-byte samples logged
  reg   [ 3:0] nxt;
  reg          pre_nxt;
  reg   [ 3:0] chn_servicing; //1-hot channel being service
//  reg   [ 3:0] rdy_d;
  wire   [3:0] wts_rq;
  assign wstart=   !busy && rq_not_zero;
  assign wts_rq[3:0]= ts_rq_in[3:0] & ~ts_rq_in_d[3:0] & (~rdy[3:0] | chn_servicing[3:0]);
  always @ (posedge xclk) begin
    ts_rq_in_d[3:0] <= ts_rq_in[3:0];
//    rdy_d[3:0] <=rdy[3:0];
    if (wstart) channel[1:0] <= {chn1hot[3] | chn1hot[2],chn1hot[3] | chn1hot[1]};
    
    if     (wstart) chn_servicing[3:0]  <= {chn1hot[3:1], ~|chn1hot[3:1]};
    else if (!busy) chn_servicing[3:0]  <= 4'h0;

//    if (rst) ts_rq[3:0] <= 4'h0;
//    else ts_rq[3:0] <= ~ts_rq_reset[3:0] & ((ts_rq_in[3:0] & ~ts_rq_in_d[3:0]) | ts_rq[3:0]);

    if (rst) ts_rq[3:0] <= 4'h0;
//    else ts_rq[3:0] <=  ~ts_grant & ( (ts_rq_in[3:0] & ~ts_rq_in_d[3:0] & (~rdy[3:0] | ~ts_valid[3:0])) | ts_rq[3:0]);
    else ts_rq[3:0] <=  ~ts_grant & ( wts_rq[3:0] | ts_rq[3:0]);

    if (rst) ts_valid[3:0] <= 4'h0;
//    else ts_valid[3:0] <= ~ts_rq_reset[3:0] &( ts_grant[3:0] | (ts_valid & ~(ts_rq_in[3:0] & ~ts_rq_in_d[3:0] & ~rdy[3:0])));
    else ts_valid[3:0] <= (ts_grant[3:0] | (ts_valid & ~wts_rq[3:0]));

//    if (rst) request[3:0] <= 4'h0;
//    else request[3:0] <= ~ts_rq_reset[3:0] &( request[3:0] | (rdy[3:0] & ~rdy_d[3:0])));
//    channels_ready[3:0] <= ts_grant[3:0] & rdy[3:0];
    channels_ready[3:0] <= ts_valid[3:0] & rdy[3:0] & ~chn_servicing[3:0]; // ready should go down during servicing

    rq_not_zero <= channels_ready[3:0] != 4'h0;

    chn1hot[3:1] <= {channels_ready[3] & ~|channels_ready[2:0],
                     channels_ready[2] & ~|channels_ready[1:0],
                     channels_ready[1] &  ~channels_ready[0]};

///AF:    start <= wstart;

    if  ((seq_cntr[4:0]=='h1e) || rst) busy <= 1'b0;
    else if (rq_not_zero)              busy <= 1'b1;

//    if (!busy) seq_cntr[4:0] <= 5'h1f;
    if (!busy) seq_cntr[4:0] <= 5'h0;
    else       seq_cntr[4:0] <= seq_cntr[4:0] + 1;

    seq_cntr_last <= (seq_cntr[4:0]=='h1e);


    if      (wstart)              ts_en <=1'b1;
    else if (seq_cntr[1:0]==2'h3) ts_en <=1'b0;
    
    if (!ts_en) ts_sel[1:0] <= 2'h0;
    else        ts_sel[1:0] <=  ts_sel[1:0] + 1;

    if (!busy || (seq_cntr[4:0]=='h1d)) pre_nxt <= 1'b0;
    else if (seq_cntr[4:0]=='h01)       pre_nxt <= 1'b1;
/*    
    nxt [3:0]  <= pre_nxt? { channel[1] &  channel[0],
                             channel[1] & ~channel[0],
                            ~channel[1] &  channel[0],
                            ~channel[1] & ~channel[0]}:4'h0;
*/
    nxt [3:0]  <= pre_nxt? chn_servicing[3:0]:4'h0;
/*
    ts_rq_reset[3:0] <= start? { channel[1] &  channel[0],
                                 channel[1] & ~channel[0],
                                ~channel[1] &  channel[0],
                                ~channel[1] & ~channel[0]}:4'h0;
*/
    dv <= busy || seq_cntr_last;

    inc_sample_counter <= seq_cntr_last;

    if (rst)                     sample_counter[23:0] <= 24'h0;
    else if (inc_sample_counter) sample_counter[23:0] <= sample_counter[23:0] +1;

   
  end
endmodule

  module buf_xclk_mclk16 (xclk, // posedge
                          mclk, // posedge
                          rst,  // @posedge xclk
                          din,
                          din_stb,
                          dout,
                          dout_stb);

  input         xclk;  // half frequency (80 MHz nominal)
  input         mclk;  // system clock - frequency (160 MHz nominal)
  input         rst;   // reset module
  input  [15:0] din;
  input         din_stb;
  output [15:0] dout;
  output        dout_stb;

  reg   [1:0] wa;
  reg   [1:0] wa_mclk;
  reg   [1:0] wa_mclk_d;
  reg         rst_mclk;
  reg   [1:0] ra;
  reg   [1:0] ra_next;
  reg         inc_ra;
  wire [15:0] pre_dout;
  reg  [15:0] dout;
  reg         dout_stb;
  always @ (posedge xclk) begin
    if      (rst)     wa[1:0] <= 2'h0;
    else if (din_stb) wa[1:0] <={wa[0],~wa[1]};
  end
  always @ (posedge mclk) begin
    wa_mclk[1:0]   <= wa[1:0];
    wa_mclk_d[1:0] <= wa_mclk[1:0];
    rst_mclk<= rst;
    if (rst_mclk) ra[1:0] <= 2'h0;
    else          ra[1:0] <= inc_ra?{ra[0],~ra[1]}:{ra[1],ra[0]};

    if (rst_mclk) ra_next[1:0] <= 2'h1;
    else          ra_next[1:0] <= inc_ra?{~ra[1],~ra[0]}:{ra[0],~ra[1]};

    inc_ra <= !rst && (ra[1:0]!=wa_mclk_d[1:0]) && (!inc_ra || (ra_next[1:0]!=wa_mclk_d[1:0]));
    dout_stb <= inc_ra;
    if (inc_ra) dout[15:0] <= pre_dout[15:0];
  end


  myRAM_WxD_D #( .DATA_WIDTH(16),.DATA_DEPTH(2))
            i_fifo_4x16   (.D(din[15:0]),
                             .WE(din_stb),
                             .clk(xclk),
                             .AW(wa[1:0]),
                             .AR(ra[1:0]),
                             .QW(),
                             .QR(pre_dout[15:0]));

endmodule

module  imu_spi   ( sclk,  // system clock, negedge
                    xclk,  // half frequency (80 MHz nominal)
                    we_ra, // write enable for registers to log (@negedge clk)
                    we_div,// write enable for clock dividing(@negedge clk)
                    we_period,// write enable for IMU cycle period(@negedge clk) 0 - disable, 1 - single, >1 - half bit periods
                    wa,    // write address for register (5 bits, @negedge clk)
                    di,    // 16?-bit data in  (di, not di_d)
                    mosi,  // to IMU, bit 2 in J9
                    miso,  // from IMU, bit 3 on J9
                    config_debug, // bit 0 - long sda_en
                    sda,   // sda, shared with i2c, bit 1
                    sda_en, // enable sda output (when sda==0 and 1 cycle after sda 0->1)
                    scl,   // scl, shared with i2c, bit 0
                    scl_en, // enable scl output (when scl==0 and 1 cycle after sda 0->1)
//                    sngl_wire, // single wire clock/data for the 103695 rev A
                    ts,    // timestamop request
                    rdy,    // data ready
                    rd_stb, // data read strobe (increment address)
                    rdata); // data out (16 bits)

  input         sclk;   // system clock, negedge
  input         xclk;  // half frequency (80 MHz nominal)
  input         we_ra; // write enable for registers to log (@negedge sclk)
  input         we_div;// write enable for clock dividing(@negedge sclk)
  input         we_period;// write enable for IMU cycle period(@negedge clk)
  input  [4:0]  wa;    // write address for register (5 bits, @negedge sclk)
  input  [15:0] di;    // 16-bit data in
  output        mosi;  // to IMU, bit 2 in J9
  input         miso;  // from IMU, bit 3 on J9 
  input [3:0]   config_debug;
  output        sda;   // sda, shared with i2c, bit 1
  output        sda_en; // enable sda output (when sda==0 and 1 cycle after sda 0->1)
  output        scl;   // scl, shared with i2c, bit 0
  output        scl_en; // enable scl output (when scl==0 and 1 cycle after sda 0->1)
  output        ts;    // timestamp request

  output        rdy;    // encoded nmea data ready
  input         rd_stb; // encoded nmea data read strobe (increment address)
  output [15:0] rdata;  // encoded data (16 bits)
//  output        sngl_wire; // combined clock/data
  
  reg    [ 7:0] bit_duration_mclk=8'h0; 
  reg    [ 7:0] bit_duration; 
  reg    [ 7:0] bit_duration_cntr=8'h0; 
  reg           bit_duration_zero; // just for simulation

  reg    [ 3:0] clk_en=4'h0;
  reg    [ 1:0] clk_div;
  reg    [ 4:0] imu_in_word=   5'b0; // number of IMU output word in a sample (0..31), 0..3 - timestamp
  reg           pre_imu_wr_buf,imu_wr_buf;
  wire   [15:0]  imu_in_buf;

  reg    [4:0]  reg_seq_number; // number of register in a sequence
  wire   [6:1]  imu_reg_number; // register numer to read 
  
  reg    [1:0]  seq_state; // 0 - idle, 1 - prepare spi(4?), 2 - spi-comm(32*29), 3 - finish (2)
  reg    [9:0]  seq_counter;
  reg           end_spi, end_prepare;
  reg           set_mosi_prepare, set_mosi_spi;
  reg           seq_counter_zero, pre_seq_counter_zero;
  reg   [15:0]  mosi_reg;
  wire          mosi;
  reg           sda, sda_d;
  wire          sda_en;
  reg           scl, scl_d;
  wire          scl_en;
  reg           shift_miso;
  reg   [15:0]  miso_reg;
  reg           last_bit; // last clk _/~ in spi word (but first one)
  reg           last_bit_ext=1'b0; // from last bit till buffer write
  reg           last_buf_wr;
  reg  [ 4:0]   raddr;
  reg           rdy=1'b0;
  reg           imu_start;
  reg           ts; // delay imu_start by one cycle, so it will be aftre rdy is reset

  reg    [31:0] period; // 0 - disable, 1 - single, >1 - period in 50 ns steps
  reg    [15:0] di_d;

  reg           imu_enabled_mclk;
  reg    [1:0]  imu_enabled=2'h0;
  reg           imu_run_mclk;
  reg    [1:0]  imu_run;
  reg           imu_when_ready_mclk;
  reg    [1:0]  imu_when_ready;
  
  reg           imu_run_confirmed;
  reg           imu_start_mclk;
  reg    [1:0]  imu_start_grant;
  reg           imu_start_first;
  reg           imu_start_first_was;
  reg    [31:0] period_counter;
  wire          en;
  reg    [4:01] we_timer;
  reg           first_prepare;
  reg    [1:0]  first_prepare_d;
  wire          config_long_sda_en;
  wire          config_late_clk;
  reg    [7:0]  stall_dur_mclk;  
  reg    [7:0]  stall_dur;
  reg           stall;       // stall between words to satisfy SPI stall time
  reg    [7:0]  stall_cntr;  // stall counter (in half sclk periods)
  reg           set_stall;
  reg           skip_stall; // first word after CS -\_
  wire          shift_mosi;
  
  reg       imu_ready_reset; 
  reg [6:0] imu_ready_denoise_count;
  reg [2:0] imu_data_ready_d; 
  reg [5:0] imu_data_ready;
  reg [1:0] seq_state_zero;  

  reg       pre_scl;
  reg [2:0] sngl_wire_stb;
  reg [1:0] sngl_wire_r;
  wire      sngl_wire;
  wire      config_single_wire; // used in 103695 rev A
  
  assign    sngl_wire=~|sngl_wire_r[1:0];
  
  assign        shift_mosi=(clk_en[3] && seq_counter[0] && !stall);
  assign mosi=config_single_wire?sngl_wire:mosi_reg[15];
  
  assign config_long_sda_en=config_debug[0];
  assign config_late_clk=   config_debug[1];
  assign config_single_wire=config_debug[2];
  
  assign en=imu_enabled[1];
  assign sda_en= !config_single_wire && (!sda || !sda_d || (config_long_sda_en && (seq_state[1:0]!=2'b0)));
  assign scl_en= !config_single_wire && (!scl || !scl_d);
  
  always @ (negedge sclk) begin
    di_d[15:0] <= di[15:0];
    if (we_div) bit_duration_mclk[7:0]<=di_d[7:0];
    if (we_div) stall_dur_mclk[7:0]<=di_d[15:8];
    we_timer[4:1] <= {we_timer[3:1], we_period};

    if (we_period)    period[31:0]<={di[15:0],di_d[15:0]};
    if (we_timer[2]) imu_run_mclk <= (period[31:1]!=31'b0); // double-cycle
    if (we_timer[3]) imu_enabled_mclk <= imu_run_mclk | period[0];
    
    if (we_timer[2]) imu_when_ready_mclk <= &period[31:16]; // double-cycle
    
    if (!imu_enabled_mclk || imu_start_grant[1]) imu_start_mclk<=1'b0;
    else if (we_timer[4])imu_start_mclk<=imu_enabled_mclk;

  end

// debounce imu_data_ready
  always @ (posedge xclk) begin
    seq_state_zero[1:0] <= {seq_state_zero[0], ~|seq_state[1:0]};
    imu_ready_reset <= !imu_enabled[1] || (seq_state[1:0]!=2'b0) || !imu_when_ready[1];
    if (imu_ready_reset) imu_data_ready_d[2:0] <=3'b0;
    else                 imu_data_ready_d[2:0] <= {imu_data_ready_d[1:0], miso};
    
    if (imu_ready_reset)                         imu_data_ready[0] <= 1'b0;
    else if (imu_ready_denoise_count[6:0]==7'h0) imu_data_ready[0] <= imu_data_ready_d[2];
    
    if (imu_data_ready_d[2]==imu_data_ready[0]) imu_ready_denoise_count[6:0] <= 7'h7f; // use period LSBs?
    else                                        imu_ready_denoise_count[6:0] <= imu_ready_denoise_count[6:0] - 1;
    
    if (imu_ready_reset)                              imu_data_ready[1] <= 1'b0;
    else if (imu_data_ready[0])                       imu_data_ready[1] <= 1'b1;
    
    if (imu_ready_reset)                              imu_data_ready[2] <= 1'b0;
    else if (imu_data_ready[1] && !imu_data_ready[0]) imu_data_ready[2] <= 1'b1;
    
    if (imu_ready_reset)                              imu_data_ready[3] <= 1'b0;
    else if (imu_data_ready[2] &&  imu_data_ready[0]) imu_data_ready[3] <= 1'b1;
    
    if (clk_en[1]) imu_data_ready[4] <= imu_data_ready[3] ;
    
    imu_data_ready[5] <=clk_en[1] && imu_data_ready[3] && !imu_data_ready[4]; // single pulse @clk_en[2]
  end

  always @ (posedge xclk) begin
    imu_enabled[1:0]     <= {imu_enabled[0],imu_enabled_mclk}; 
    imu_run[1:0]         <= {imu_run[0],imu_run_mclk};

    imu_when_ready[1:0]         <= {imu_when_ready[0],imu_when_ready_mclk};

    if        (~imu_run[1:0]) imu_run_confirmed <= 1'b0;
    else if (imu_start_first) imu_run_confirmed <= imu_run[1];
    imu_start_grant[1:0] <= {imu_enabled_mclk && (imu_start_grant[0] || (imu_start_grant[1] && !imu_start)),imu_start_mclk};
    imu_start_first_was <= imu_start_grant[1] && (imu_start_first || imu_start_first_was);
    
    imu_start_first<=clk_en[1] && imu_start_grant[1] && !imu_start_first_was; // single xclk at clk_en[2] time slot
    imu_start            <=(!imu_when_ready[1] && imu_start_first)||
                           (!imu_when_ready[1] && imu_run_confirmed && (period_counter[31:0]==32'h1) && clk_en[2]) ||
                           imu_data_ready[5]; // single pulses at clk_en[3]

    if (imu_start || imu_when_ready[1]) period_counter[31:0] <= period[31:0];
    else if (clk_en[3]) period_counter[31:0] <= period_counter[31:0] - 1;

  end


  always @ (posedge xclk) begin
    bit_duration[7:0] <= bit_duration_mclk[7:0];
    stall_dur[7:0]       <= stall_dur_mclk[7:0];

    bit_duration_zero <= (bit_duration[7:0]==8'h0);
    clk_div[1:0]      <= en?(clk_div[1:0]+1):2'b0;
    clk_en[3:0]       <= {clk_en[2:0],clk_div[1:0]==2'h3};
    
    if (bit_duration_zero || (bit_duration_cntr[7:0]==8'h0)) bit_duration_cntr[7:0]<=bit_duration[7:0];
    else bit_duration_cntr[7:0] <= bit_duration_cntr[7:0]-1;
    
    clk_en[3:0]  <= {clk_en[2:0],bit_duration_cntr[7:0]==8'h3};  // change 9'h3 to enforce frequency limit
  end  
  
  always @ (posedge xclk) begin
    pre_seq_counter_zero  <= clk_en[1] && (seq_counter[9:0]==10'h0) && (seq_state[1:0]!=2'h0); // active at clk_en[2]
    seq_counter_zero      <= pre_seq_counter_zero; // active at clk_en[3]
    if (!en)       seq_state[1:0] <= 2'h0;
    else if (imu_start)                                  seq_state[1:0] <= 2'h1;
    else if (seq_counter_zero ) seq_state[1:0] <= seq_state[1:0] + 1; // will not count from 0 as seq_counter_zero will be disabled
    
    if            (!en) first_prepare <=1'b0;
    else if (imu_start) first_prepare <=1'b1;
    else if (clk_en[3]) first_prepare <=1'b0;
    
    if            (!en) first_prepare_d[1:0] <= 2'b0;
    else if (clk_en[3]) first_prepare_d[1:0] <= {first_prepare_d[0],first_prepare};
    
    end_prepare <= pre_seq_counter_zero && (seq_state[1:0]==2'h1);
    end_spi       <= pre_seq_counter_zero && (seq_state[1:0]==2'h2);
    
    if      (!en)                                           seq_counter[9:0] <= 10'h000;
    else if (imu_start)                                     seq_counter[9:0] <= config_late_clk?10'h005:10'h003; // should be odd
    else if (end_prepare)                                   seq_counter[9:0] <= 10'h39f;
    else if (end_spi)                                       seq_counter[9:0] <= 10'h001;
    else if (clk_en[3] && (seq_state[1:0]!=2'h0) && !stall) seq_counter[9:0] <= seq_counter[9:0] - 1;
    set_mosi_prepare <= clk_en[2] && first_prepare;
    set_mosi_spi       <= clk_en[2] && (seq_state[1:0]==2'h2) && (seq_counter[4:0]==5'h1f) && (seq_counter[9:5] != 5'h0) && !stall; // last word use zero
    
// no stall before the first word
    if      (!en)                          skip_stall <= 1'b0;
    else if (end_prepare)                  skip_stall <= 1'b1;
    else if (clk_en[3])                    skip_stall <= 1'b0;
     
//    set_stall          <= clk_en[2] && (seq_state[1:0]==2'h2) && (seq_counter[4:0]==5'h1f) && !skip_stall; // same as  set_mosi_spi, but including last
//    set_stall          <= clk_en[1] && (seq_state[1:0]==2'h2) && (seq_counter[4:0]==5'h1f) && !skip_stall && !stall; // @ clk_en[2]
    set_stall          <= clk_en[0] && (seq_state[1:0]==2'h2) && (seq_counter[4:0]==5'h1f) && !skip_stall && !stall; // @ clk_en[1]

    if      (!en)               mosi_reg[15:0] <= 16'h0;
    else if (set_mosi_prepare)  mosi_reg[15:0] <= 16'h7fff;
    else if (set_mosi_spi)      mosi_reg[15:0] <= {1'b0,imu_reg_number[6:1],9'b0};
    else if (shift_mosi)        mosi_reg[15:0] <= {mosi_reg[14:0],1'b0};
//  assign        shift_mosi=(clk_en[3] && seq_counter[0] && !stall);


// stall switches at clk_en[2]
// stall switches at clk_en[1]
    if      (!en)                         stall_cntr[7:0] <= 8'h0;
    else if (set_stall)                   stall_cntr[7:0] <= stall_dur[7:0];
    else if (clk_en[1])                   stall_cntr[7:0] <= stall?(stall_cntr[7:0]-1):8'h0;




    if      (!en)                               stall <= 1'b0;
    else if (set_stall)                         stall <= (stall_dur[7:0]!=0);
    else if (clk_en[1] && (stall_cntr[7:1]==0)) stall <= 1'b0;
    


    if      (!en) sda <=1'b1;
    else if (clk_en[3])       sda <= !(first_prepare_d[1] || (seq_counter[0] && (seq_state[1:0]==2'h3))) ;

    if      (!en) sda_d <=1'b1;
    else if (clk_en[3])       sda_d <= sda;

//    if      (!en) scl <=1'b1;
//    else if (clk_en[3])       scl <= (seq_state[1:0]!=2'h2) || !seq_counter[0] || stall;

    if      (!en) pre_scl <=1'b1;
    else if (clk_en[2])       pre_scl <= (seq_state[1:0]!=2'h2) || !seq_counter[0] || stall;
    
    scl <= pre_scl;
    
    sngl_wire_stb[2:0] <={sngl_wire_stb[1:0], en & ((scl ^ pre_scl) | end_prepare)};

    if      (!en)                              sngl_wire_r[0]<=1'b0;
//    else if (!pre_scl && scl)           sngl_wire_r[0]<=1'b1;
//    else if (!mosi || sngl_wire_stb[2]) sngl_wire_r[0]<=1'b0;
    else if ((pre_scl ^scl) | end_prepare)     sngl_wire_r[0]<=1'b1;
    else if (!mosi_reg[15] || sngl_wire_stb[2] || scl) sngl_wire_r[0]<=1'b0;
    
   
    if      (!en) scl_d <=1'b1;
    else if (clk_en[3])       scl_d <= scl;
    
    if      (imu_start)     reg_seq_number[4:0] <= 5'h04;
    else if (set_mosi_spi)  reg_seq_number[4:0] <= reg_seq_number[4:0] + 1;

    shift_miso <= !scl_d && clk_en[2]; // active at clk_en[3]
//    shift_miso <= !scl_d && clk_en[2] && !stall; // active at clk_en[3]

    if (shift_miso)     miso_reg[15:0] <= {miso_reg[14:0], miso};

    last_bit <= clk_en[2] && (seq_state[1:0]==2'h2) && (seq_counter[4:0]==5'h0) && (seq_counter[9:5]!=5'h1c);
    last_bit_ext <= en && (last_bit || (last_bit_ext && !(clk_en[2] && !seq_counter[0])));

    pre_imu_wr_buf <=clk_en[1] && last_bit_ext && !seq_counter[0]; 
    imu_wr_buf <= pre_imu_wr_buf;
    if    (imu_start) imu_in_word[4:0] <= 5'h0;
    else if (imu_wr_buf) imu_in_word[4:0] <= imu_in_word[4:0] + 1;

    last_buf_wr <= (pre_imu_wr_buf && (seq_state[1:0]==2'h3));
    
  end  

  always @ (negedge xclk) begin
     sngl_wire_r[1] <= sngl_wire_stb[0];
  end

  always @ (posedge xclk) begin

    if (!en || imu_start) raddr[4:0] <= 5'h0;
    else if (rd_stb)      raddr[4:0] <= raddr[4:0] + 1;

    if      (imu_start || (rd_stb && (raddr[4:0]==5'h1b)) || !en) rdy <= 1'b0; // only 28 words, not 32
    else if (last_buf_wr)                                         rdy <= 1'b1;

    ts <=imu_start;

  end  
  
   assign imu_in_buf[15:0]= miso_reg[15:0];
  myRAM_WxD_D #( .DATA_WIDTH(6),.DATA_DEPTH(5))
            i_registers2log   (.D(di_d[6:1]),
                             .WE(we_ra),
                             .clk(!sclk),
                             .AW(wa[4:0]),
                             .AR(reg_seq_number[4:0]),
                             .QW(),
                             .QR(imu_reg_number[6:1]));

  myRAM_WxD_D #( .DATA_WIDTH(16),.DATA_DEPTH(5))
            i_odbuf0    (.D(imu_in_buf[15:0]),
                        .WE(imu_wr_buf),
                        .clk(xclk),
                        .AW(imu_in_word[4:0]),
                        .AR(raddr[4:0]),
                        .QW(),
                        .QR(rdata[15:0]));



endmodule
/*
logs events from odometer (can be software triggered), includes 56-byte message written to the buffer
So it is possible to assert trig input (will request timestamp), write message by software, then
de-assert the trig input - message with the timestamp will be logged
fixed-length de-noise circuitry with latency 256*T(xclk) (~3usec)
*/

module  imu_message ( sclk,   // system clock, negedge
                       xclk,  // half frequency (80 MHz nominal)
                       we,    // write enable for registers to log (@negedge sclk), with lower data half
                       wa,    // write address for register (4 bits, @negedge sclk)
                       di,    // 16-bit data in  multiplexed 
                       en,    // enable module operation, if 0 - reset
                       trig,  // leading edge - sample time, trailing set rdy
                       ts,    // timestamop request
                       rdy,    // data ready
                       rd_stb, // data read strobe (increment address)
                       rdata); // data out (16 bits)

  input         sclk;   // system clock, negedge
  input         xclk;  // half frequency (80 MHz nominal)
  input         we;    // write enable for registers to log (@negedge sclk)
  input  [3:0]  wa;    // write address for register (4 bits, @negedge sclk)
  input  [15:0] di;    // 16-bit data in (32 multiplexed)
  input         en;    // enable
  input         trig;  // leading edge - sample time, trailing set rdy
  output        ts;    // timestamp request
  output        rdy;    // encoded nmea data ready
  input         rd_stb; // encoded nmea data read strobe (increment address)
  output [15:0] rdata;  // encoded data (16 bits)

  reg  [ 4:0]   raddr;
  reg           rdy=1'b0;
  reg           we_d;
  reg  [ 4:1]   waddr;
  reg  [ 2:0]   trig_d;
  reg  [ 7:0]   denoise_count;
  reg  [ 1:0]   trig_denoise;
  reg           ts;
  reg    [15:0] di_d;

  always @ (negedge sclk) begin
    di_d[15:0] <= di[15:0];
    waddr[4:1] <= wa[3:0];
    we_d <=we;
  end
  always @ (posedge xclk) begin
    if  (!en) trig_d[2:0] <= 3'h0;
    else      trig_d[2:0] <= {trig_d[1:0], trig};
    if      (!en)                      trig_denoise[0] <= 1'b0;
    else if (denoise_count[7:0]==8'h0) trig_denoise[0] <= trig_d[2];
    if (trig_d[2]==trig_denoise[0]) denoise_count[7:0] <= 8'hff;
    else                            denoise_count[7:0] <= denoise_count[7:0] - 1;
    trig_denoise[1] <= trig_denoise[0];
    ts <= !trig_denoise[1] && trig_denoise[0];

    if (!en || ts)   raddr[4:0] <= 5'h0;
    else if (rd_stb)    raddr[4:0] <= raddr[4:0] + 1;

    if  (ts || (rd_stb && (raddr[4:0]==5'h1b)) || !en) rdy <= 1'b0;
    else if (trig_denoise[1] && !trig_denoise[0])     rdy <= 1'b1;
  end

  myRAM_WxD_D #( .DATA_WIDTH(16),.DATA_DEPTH(5))
            i_odbuf (.D(di_d[15:0]),
                     .WE(we | we_d),
                     .clk(~sclk),
                     .AW({waddr[4:1],we_d}),
                     .AR(raddr[4:0]),
                     .QW(),
                     .QR(rdata[15:0]));

endmodule

/*
logs frame synchronization data from other camera (same as frame sync)
*/

module  imu_exttime (  xclk,  // half frequency (80 MHz nominal)
                       en,    // enable module operation, if 0 - reset
                       trig,  // external time stamp updated
                       usec,  // microseconds from external timestamp (should not chnage after trig for 10 xclk)
                       sec,   // seconds from external timestamp
                       ts,    // timestamop request
                       rdy,    // data ready
                       rd_stb, // data read strobe (increment address)
                       rdata); // data out (16 bits)

  input         xclk;  // half frequency (80 MHz nominal)
  input         en;    // enable
  input         trig;  // external time stamp updated
  input  [19:0] usec;  // microseconds from external timestamp
  input  [31:0] sec;   // seconds from external timestamp
  output        ts;    // timestamp request
  output        rdy;   // encoded nmea data ready
  input         rd_stb;// encoded nmea data read strobe (increment address)
  output [15:0] rdata; // encoded data (16 bits)

  reg  [ 4:0]   raddr;
  reg           rdy=1'b0;
  

  reg           we, pre_we;
  reg  [ 3:0]   pre_waddr;
  reg  [ 1:0]   waddr;
  reg  [ 2:0]   trig_d;
  reg           pre_ts,ts;
  reg  [15:0]   time_mux;
  always @ (posedge xclk) begin
    if  (!en) trig_d[2:0] <= 3'h0;
    else      trig_d[2:0] <= {trig_d[1:0], trig};

    pre_ts <= !trig_d[2] && trig_d[1];
    ts <= pre_ts; // delayed so arbiter will enable ts to go through
    if      (!en || pre_ts)     pre_waddr[3:0] <= 4'b0;
    else if (!pre_waddr[3]) pre_waddr[3:0] <= pre_waddr[3:0] + 1;
    if (pre_waddr[0]) waddr[1:0] <=pre_waddr[2:1];
    if (pre_waddr[0] && !pre_waddr[3]) case (pre_waddr[2:1])
      2'b00: time_mux[15:0] <= usec[15:0];
      2'b01: time_mux[15:0] <= {12'h0,usec[19:16]};
      2'b10: time_mux[15:0] <= sec[15:0];
      2'b11: time_mux[15:0] <= sec[31:16];
    endcase
    pre_we<=pre_waddr[0] && !pre_waddr[3];
    we <= pre_we;

    if (!en || pre_ts)   raddr[4:0] <= 5'h0;
    else if (rd_stb)    raddr[4:0] <= raddr[4:0] + 1;

    if  (pre_ts || (rd_stb && (raddr[1:0]==2'h3)) || !en) rdy <= 1'b0;
    else if (we && (waddr[1:0]==2'h3))                rdy <= 1'b1;
  end

  myRAM_WxD_D #( .DATA_WIDTH(16),.DATA_DEPTH(2))
            i_odbuf (.D(time_mux[15:0]),
                     .WE(we),
                     .clk(xclk),
                     .AW(waddr[1:0]),
                     .AR(raddr[1:0]),
                     .QW(),
                     .QR(rdata[15:0]));

endmodule



module  rs232_rcv (xclk,           // half frequency (80 MHz nominal)
                   bitHalfPeriod,  // half of the serial bit duration, in xclk cycles
                   ser_di,         // rs232 (ttl) serial data in
                   ser_rst,        // reset (force re-sync)
                   ts_stb,         // strobe timestamp (start of message) (reset bit counters in nmea decoder)
                   wait_just_pause,// may be used as reset for decoder
                   start,          // serial character start (single pulse)
//                   char,           // byte out
//                   char_stb);      // char strobe (@posedge xclk)
                   ser_do,         // serial data out(@posedge xclk) LSB first!
                   ser_do_stb,    // output data strobe (@posedge xclk), first cycle after ser_do becomes valid
                   debug,        // {was_ts_stb, was_start, was_error, was_ser_di_1, was_ser_di_0} - once after reset
                   bit_dur_cntr,
                   bit_cntr);
  input         xclk;           // half frequency (80 MHz nominal)
  input  [15:0] bitHalfPeriod;  // half of the serial bit duration, in xclk cycles
  input         ser_di;         // rs232 (ttl) serial data in
  input         ser_rst;        // reset (force re-sync)
  output        ts_stb;         // strobe timestamp (start of message)
  output        wait_just_pause;// may be used as reset for decoder
  output        start;          // serial character start (single pulse)
  output [4:0]  debug;          // {was_ts_stb, was_start, was_error, was_ser_di_1, was_ser_di_0} - once after reset
  output        ser_do;         // serial data out(@posedge xclk)
  output        ser_do_stb;     // output data strobe (@posedge xclk), 2 cycles after ser_do becomes valid
  output [15:0] bit_dur_cntr;   // debug
  output  [4:0] bit_cntr;       // debug
  
  reg  [4:0] ser_di_d;
  reg        ser_filt_di;
  reg        ser_filt_di_d;
  reg        bit_half_end; // last cycle in half-bit
  reg        last_half_bit;
  reg        wait_pause;   // waiting input to stay at 1 for 10 cycles
  reg        wait_start;   // (or use in_sync - set it after wait_pause is over?
  reg        receiving_byte;
  reg        start;
  reg [15:0] bit_dur_cntr; // bit duration counter (half bit duration)
  reg [4:0]  bit_cntr;     // counts half-bit intervals
  wire       error;        // low level during stop slot
  reg  [1:0] restart;
  wire       reset_wait_pause;
  reg        ts_stb;
  reg        shift_en;
  
  reg        ser_do;
  reg        ser_do_stb;
  wire       sample_bit;
  wire       reset_bit_duration;
  reg        wait_just_pause;
  wire       wstart;
  wire [4:0] debug;
  reg  [4:0] debug0;          // {was_ts_stb, was_start, was_error, was_ser_di_1, was_ser_di_0} - once after reset // SuppressThisWarning Veditor UNUSED
  assign     reset_wait_pause= (restart[1] && !restart[0]) || (wait_pause && !wait_start && !ser_di);
  assign     error=!ser_filt_di && last_half_bit && bit_half_end && receiving_byte;
  assign     sample_bit=shift_en && bit_half_end && !bit_cntr[0];
  assign     reset_bit_duration= reset_wait_pause || start || bit_half_end || ser_rst;
  
  assign wstart=wait_start && ser_filt_di_d && !ser_filt_di;
  
  assign debug[4:0] = {1'b0,wait_start,wait_pause,receiving_byte,shift_en};
  always @ (posedge xclk) begin
//    reg  [4:0] ser_di_d;
//  reg        ser_filt_di;
//  reg        ser_filt_di_d;
    ser_di_d[4:0] <= {ser_di_d[3:0],ser_di};
    if (ser_rst || &ser_di_d[4:0]) ser_filt_di <= 1'b1;
    else if      (~|ser_di_d[4:0]) ser_filt_di <= 1'b0;

    ser_filt_di_d <= ser_filt_di;
    
    restart[1:0] <= {restart[0],(ser_rst || (last_half_bit && bit_half_end && receiving_byte))};
    wait_pause <= !ser_rst && (reset_wait_pause ||
                               (receiving_byte && last_half_bit && bit_half_end ) ||
                               (wait_pause && !(last_half_bit && bit_half_end) && !(wait_start && !ser_filt_di)));
//    start                 <= wait_start && ser_di_d && !ser_di;
    start                 <= wstart;
//    ts_stb <= !wait_pause && wait_start && ser_di_d && !ser_di;
    ts_stb <= !wait_pause && wstart; // only first start after pause
    bit_half_end <=(bit_dur_cntr[15:0]==16'h1) && !reset_bit_duration;
    
//    wait_start <= ser_di && !ser_rst && ((wait_pause || receiving_byte) && last_half_bit && bit_half_end  || wait_start);
    wait_start <= !ser_rst && ((wait_pause || receiving_byte) && last_half_bit && bit_half_end  || (wait_start && !wstart));
//    receiving_byte <= !ser_rst && !error && (start || (receiving_byte && !(last_half_bit && bit_half_end)));
    receiving_byte <= !ser_rst && (start || (receiving_byte && !(last_half_bit && bit_half_end)));
    wait_just_pause <=wait_pause && !wait_start;
    
    
    if (reset_bit_duration) bit_dur_cntr[15:0] <= bitHalfPeriod[15:0];
    else                    bit_dur_cntr[15:0] <= bit_dur_cntr[15:0] - 1;
    
    if (reset_wait_pause || ser_rst)  bit_cntr[4:0] <= 5'h13;
    else if (start)                    bit_cntr[4:0] <= 5'h12;
    else if (bit_half_end)             bit_cntr[4:0] <= bit_cntr[4:0] - 1;

    last_half_bit <= ((bit_cntr[4:0] == 5'h0) && !bit_half_end); 
    shift_en <= receiving_byte &&  ((bit_half_end && ( bit_cntr[3:0]==4'h2))? bit_cntr[4]:shift_en);

   
    if (sample_bit) ser_do <= ser_filt_di;
    ser_do_stb <= sample_bit; 
    
    
    if (ser_rst) debug0[4:0] <=5'b0;
    else debug0[4:0] <= debug | {ts_stb,start,error,ser_di_d[0],~ser_di_d[0]};
  end
endmodule


module nmea_decoder (sclk,   // system clock, @negedge
                     we,     // registers write enable (@negedge sclk)
                     wa,     // registers write adderss
                     wd,     // write data
                     xclk,   // 80MHz, posedge
                     start,  // start of the serail message
                     rs232_wait_pause,// may be used as reset for decoder
                     start_char,           // serial character start (single pulse)
                     nmea_sent_start,  // serial character start (single pulse)
                     ser_di, // serial data in (LSB first)
                     ser_stb,// serial data strobe, single-cycle, first cycle after ser_di valid
                     rdy,    // encoded nmea data ready
                     rd_stb, // encoded nmea data read strobe (increment address)
                     rdata,   // encoded data (16 bits)
                     ser_rst,
                     debug);
                     
  input         sclk;   // system clock, @negedge
  input         we;     // registers write enable (@negedge sclk)
  input  [4:0]  wa;     // registers write adderss
  input  [7:0]  wd;     // write data
  input         xclk;   // 80MHz, posedge
  input         start;  // start of the serail message (after pause only)
  input         rs232_wait_pause;// may be used as reset for decoder
  input         start_char;           // serial character start (single pulse)
  output        nmea_sent_start;  // serial character start (single pulse), will repeat until got "$" and the sentence recognized
  input         ser_di; // serial data in (LSB first)
  input         ser_stb;// serial data strobe, single-cycle, ends 2 cycles after ser_di valid
  output        rdy;    // encoded nmea data ready
  input         rd_stb; // encoded nmea data read strobe (increment address)
  output [15:0] rdata;  // encoded data (16 bits)

  input         ser_rst;
  output[23:0]  debug;


  reg    [ 9:0] bitnum;
  reg           gp_exp_bit;                   
  reg           valid; // so far valid sentence
  reg    [3:0]  sentence1hot; // one-hot sentence, matching first 6 bytes ($GPxxx)
  reg           restart; // reset byte number if the first byte was not "$"
  reg           start_d;
  reg    [3:0]  stb; // ser_stb delayed
  reg           msb,bits37,bit3;
  reg           vfy_dollar;
  reg           vfy_gp;
  reg           vfy_sel_sent;
  reg           vfy_first_comma; // first comma after $GPxxx
  
  reg           proc_fields;
  reg           last_vfy_gp;   // delayed by 1 cycle from bit counters
  reg           last_vfy_sent; // delayed by 1 cycle from bit counters
//  reg   [3:0]   sent_sel_cntr; // counts 3 times to 5, as $GPxxx - each 'x' is an upper case latter (0x40..0x5f)
  reg           lsbs5;         // 5 LSBs during reading 3 last letters in $GPxxx
  reg   [3:0]   gpxxx_addr;
  wire  [3:1]   sentence1hot_pri; // sentence1hot made really one-hot
  reg   [1:0]   sentence; // decoded sentence number (0..3)
  reg   [4:0]   format_length; // number of fields in the sentence
  reg   [4:0]   format_length_plus_7;
  reg   [4:0]   format_field;  // current number of the field in the sentence
  wire          start_format;
  reg           read_format_length; //, read_format_length_d;
  reg           read_format_byte;
  reg           shift_format_byte;
  reg           format_over;
  reg           sentence_over;
  reg   [7:0]   format_byte;
  reg   [7:1]   last_byte;
  wire          wcomma; // comma
  wire          weof;   //asterisk, or cr/lf (<0x10)
  wire          wsep;  //any separator 
  reg   [3:0]   nibble;
  reg   [3:0]   nibble_pre;
  wire  [7:0]   wbyte;
  reg           nibble_stb;
  reg           first_byte_in_field;
  reg   [1:0]   extra_nibble; // empty byte field - send two 4'hf nibbles
  reg   [6:0]   nibble_count;
  wire [15:0]   rdata; // encoded data (16 bits)
  reg  [ 4:0]   raddr;
  wire  [3:0]   gpxxx_w_one;
  wire  [7:0]   format_data;
  wire          w_sentence_over;
  reg   [4:0]   last_word_written; // number of the last word (4 nibbles) written - used ro deassert rdy (garbage after)
  reg           rdy=1'b0;
  reg           nmea_sent_start;
  reg           save_sent_number;
//    input         ser_rst;
  reg  [ 7:0]   debug0;
  reg  [15:0]   debug1;
  reg  [15:0]   debug1_or;
  wire [23:0]   debug;

//  assign debug[23:0] = {debug1[15:0],debug0[7:0]};

  assign debug[23:0] =  {1'b0,
                         proc_fields,
                         vfy_first_comma,
                         vfy_sel_sent,
                         vfy_gp,
                         vfy_dollar,
                         bitnum[9:0],
                         debug0[7:0]};

  assign sentence1hot_pri[3:1]={sentence1hot[3]& ~|sentence1hot[2:0],
                                sentence1hot[2]& ~|sentence1hot[1:0],
                                sentence1hot[1]&  ~sentence1hot[0]};
//  assign start_format=(last_vfy_sent && (sentence1hot[3:0]!=4'h0) && (stb[3] && msb));
  assign start_format=(vfy_first_comma && (sentence1hot[3:0]!=4'h0) && (stb[3] && msb));
  
  assign wbyte[7:0]={ser_di,last_byte[7:1]}; // valid up to stb[3];
  assign wcomma= proc_fields && msb && (wbyte[7:0]==8'h2c);
  assign weof=   proc_fields && msb && ((wbyte[7:0]==8'h2a) || (wbyte[7:4]==4'h0)); // 0x2a or 0x0? (<0x10)
  assign wsep= wcomma || weof;
//  assign w_sentence_over=wsep && (format_field[2:0]==format_length[2:0]) && (format_field[4:3]==(format_length[4:3]+1));
  assign w_sentence_over=wsep && (format_field[4:0]==format_length_plus_7[4:0]);
//format_length_plus_7
  always @ (posedge xclk) begin
    if (ser_rst) debug0 [7:0] <= 8'b0;
    else debug0 [7:0] <=debug0 [7:0] | {rdy,
                                      proc_fields,
                                      shift_format_byte,
                                      start_format,
                                      vfy_first_comma,
                                      vfy_sel_sent,
                                      vfy_gp,
                                      vfy_dollar};

    if (ser_rst) debug1 [15:0] <= 16'b0;
    else if (stb[1] && vfy_sel_sent && lsbs5) debug1 [15:0] <= debug1 [15:0] | debug1_or [15:0];

    case (gpxxx_addr[3:0])
      4'h0:  debug1_or[15:0] <= 16'h0001;
      4'h1:  debug1_or[15:0] <= 16'h0002;
      4'h2:  debug1_or[15:0] <= 16'h0004;
      4'h3:  debug1_or[15:0] <= 16'h0008;
      4'h4:  debug1_or[15:0] <= 16'h0010;
      4'h5:  debug1_or[15:0] <= 16'h0020;
      4'h6:  debug1_or[15:0] <= 16'h0040;
      4'h7:  debug1_or[15:0] <= 16'h0080;
      4'h8:  debug1_or[15:0] <= 16'h0100;
      4'h9:  debug1_or[15:0] <= 16'h0200;
      4'ha:  debug1_or[15:0] <= 16'h0400;
      4'hb:  debug1_or[15:0] <= 16'h0800;
      4'hc:  debug1_or[15:0] <= 16'h1000;
      4'hd:  debug1_or[15:0] <= 16'h2000;
      4'he:  debug1_or[15:0] <= 16'h4000;
      4'hf:  debug1_or[15:0] <= 16'h8000;
    endcase
                                      
    stb[3:0] <= {stb[2:0], ser_stb};
    start_d <= start;
    restart <= start || sentence_over || stb[2] && msb && ((!valid && (vfy_dollar || last_vfy_gp || vfy_first_comma)) || // may abort earlier (use vfy_gp)
                                          ((sentence1hot==4'h0) &&  last_vfy_sent)); // may abort earlier (use vfy_sel_sent)
 
    if      (start_d)  bitnum[2:0] <= 3'h0;
    else if (stb[3]) bitnum[2:0] <= bitnum[2:0] + 1;

    if      (start_d)  msb <= 1'b0;
    else if (stb[3])   msb <= (bitnum[2:0] ==3'h6);

    if      (start_d)  bit3  <= 1'b0;
    else if (stb[3])   bit3 <= (bitnum[2:0] ==3'h2);

    if      (start_d)  bits37 <= 1'b0;
    else if (stb[3])   bits37 <= (bitnum[1:0] ==2'h2);

    if      (start_d)  lsbs5 <= 1'b1;
    else if (stb[3])   lsbs5 <= !bitnum[2] || (bitnum[2:0] ==3'h7);
    
    if      (restart)       bitnum[9:3] <= 'h0;
    else if (stb[3] && msb) bitnum[9:3] <=  bitnum[9:3] + 1;
    
    if      (restart || rs232_wait_pause)  vfy_dollar <= 1'b1;  // byte 0
    else if (stb[3] && msb)                vfy_dollar <= 1'b0;

    last_vfy_gp <= vfy_gp && !bitnum[3];
    if      (restart)       vfy_gp <= 1'b0;
    else if (stb[3] && msb) vfy_gp <= (valid && vfy_dollar) || (vfy_gp && !last_vfy_gp); // bytes 1-2

    last_vfy_sent <= vfy_sel_sent && (bitnum[3] && bitnum[5]);
    if      (restart)       vfy_sel_sent <= 1'b0;
    else if (stb[3] && msb) vfy_sel_sent <= (valid && last_vfy_gp) || (vfy_sel_sent && !last_vfy_sent); // bytes 3,4,5

    if      (restart)       vfy_first_comma <= 1'b0;
    else if (stb[3] && msb) vfy_first_comma <= last_vfy_sent;
    
    if (restart)                                                      valid <= 1'b1; // ready @ stb[2]
    else if (stb[1] && (ser_di!=gp_exp_bit) &&
                       (vfy_dollar || vfy_gp || vfy_first_comma || (vfy_sel_sent && !lsbs5))) valid <= 1'b0;

 
    if       (!vfy_sel_sent) gpxxx_addr[3:0] <= 4'h0;
    else if (lsbs5 &&stb[3]) gpxxx_addr[3:0] <= gpxxx_addr[3:0] + 1;
    
    if (vfy_gp)                                sentence1hot[3:0] <= 4'hf;
    else if (stb[1] && vfy_sel_sent && lsbs5)  sentence1hot[3:0] <= sentence1hot & (ser_di?(gpxxx_w_one[3:0]): (~gpxxx_w_one[3:0]));

    if (last_vfy_sent && stb[3] && msb) sentence[1:0] <= {sentence1hot_pri[3] | sentence1hot_pri[2], sentence1hot_pri[3] | sentence1hot_pri[1]};
    
    if (restart || sentence_over) proc_fields <=1'b0;
    else if (start_format)        proc_fields <=1'b1;
    
    if (!proc_fields)            format_field[4:0] <= 5'h0;
    else if (read_format_length) format_field[4:0] <= 5'h8;
    else if (format_over)        format_field[4:0] <= format_field[4:0] + 1;
    format_length_plus_7[4:0] <= format_length[4:0]+7;
    if      (start_format)  first_byte_in_field <=1'b1;
    else if (stb[3] && msb) first_byte_in_field <=  format_over;
    
    read_format_length <= start_format;
    
    if (read_format_length) format_length[4:0] <= format_data[4:0];
    
    read_format_byte <= read_format_length || (format_over && format_field[2:0]==3'h7); // @stb[4]
    shift_format_byte <= format_over; // @stb[4]
    if       (read_format_byte) format_byte[7:0] <= format_data[7:0];
    else if (shift_format_byte) format_byte[7:0] <= {1'b0,format_byte[7:1]};
//     format_byte[0] - current format
    if (stb[3]) last_byte[7:1] <= {ser_di,last_byte[7:2]};
    format_over   <=  stb[2] && wsep;
//    sentence_over <=  stb[2] && (weof || (wsep && w_sentence_over));
    sentence_over <=  stb[2] && (weof || w_sentence_over);

    if (bits37 && stb[3]) nibble_pre[3:0] <= last_byte[4:1]; // always OK

    if      (stb[3] && bit3)                             nibble[3:0] <= nibble_pre[3:0];
    else if (stb[3] && msb &&  wsep && (first_byte_in_field || !format_byte[0]))  nibble[3:0] <= 4'hf;
    else if (stb[3] && msb &&           format_byte[0])   nibble[3:0] <= {wsep,nibble_pre[2:0]};
    else if (save_sent_number) nibble[3:0] <= {2'b0,sentence[1:0]};
    
//first_byte_in_field   

    extra_nibble[1:0] <= {extra_nibble[0],
                          msb &&  wsep && first_byte_in_field & proc_fields & stb[3] & format_byte[0]};// active at stb[4], stb[5]
    save_sent_number <= start_format; // valid at stb[4]
    nibble_stb <= save_sent_number ||
                    (proc_fields && ((stb[3] && bit3 && !first_byte_in_field) ||
                    (stb[3] && msb  && !first_byte_in_field && format_byte[0]) ||
                    (stb[3] && msb  && wsep))) || extra_nibble[1]; // extra_nibble[1] will repeat 4'hf

    if    (start_format) nibble_count[6:0] <= 7'h0;
    else if (nibble_stb) nibble_count[6:0] <= nibble_count[6:0] + 1;
    
//    if (weof && stb[3]) raddr[4:0] <= 5'h0;
    if (sentence_over) raddr[4:0] <= 5'h0;
    else if (rd_stb)    raddr[4:0] <= raddr[4:0] + 1;
    if (nibble_stb) last_word_written[4:0]<=nibble_count[6:2];
    if      (start || vfy_first_comma || (rd_stb && ((raddr[4:0]==5'h1b) ||(raddr[4:0]==last_word_written[4:0])))) rdy <= 1'b0;
    else if (sentence_over)   rdy <= 1'b1;
    nmea_sent_start <= start_char && vfy_dollar;
  end
// output buffer to hold up to 32 16-bit words. Written 1 nibble at a time
  myRAM_WxD_D #( .DATA_WIDTH(4),.DATA_DEPTH(5))
            i_odbuf0   (.D(nibble[3:0]),
                        .WE(nibble_stb && (nibble_count[1:0]==2'h0)),
                        .clk(xclk),
                        .AW(nibble_count[6:2]),
                        .AR(raddr[4:0]),
                        .QW(),
                        .QR(rdata[3:0]));

  myRAM_WxD_D #( .DATA_WIDTH(4),.DATA_DEPTH(5))
            i_odbuf1   (.D(nibble[3:0]),
                        .WE(nibble_stb && (nibble_count[1:0]==2'h1)),
                        .clk(xclk),
                        .AW(nibble_count[6:2]),
                        .AR(raddr[4:0]),
                        .QW(),
                        .QR(rdata[7:4]));

  myRAM_WxD_D #( .DATA_WIDTH(4),.DATA_DEPTH(5))
            i_odbuf2   (.D(nibble[3:0]),
                        .WE(nibble_stb && (nibble_count[1:0]==2'h2)),
                        .clk(xclk),
                        .AW(nibble_count[6:2]),
                        .AR(raddr[4:0]),
                        .QW(),
                        .QR(rdata[11:8]));

  myRAM_WxD_D #( .DATA_WIDTH(4),.DATA_DEPTH(5))
            i_odbuf3   (.D(nibble[3:0]),
                        .WE(nibble_stb && (nibble_count[1:0]==2'h3)),
                        .clk(xclk),
                        .AW(nibble_count[6:2]),
                        .AR(raddr[4:0]),
                        .QW(),
                        .QR(rdata[15:12]));

  myRAM_WxD_D #( .DATA_WIDTH(4),.DATA_DEPTH(4))
            i_gpxxx   (.D(wd[3:0]),
                       .WE(we &  ~wa[4]), // we_d, decoded sub_address
                       .clk(!sclk),
                       .AW(wa[3:0]),
                       .AR(gpxxx_addr[3:0]),
                       .QW(),
                       .QR(gpxxx_w_one[3:0]));
// for each of the four sentences first byte - number of field (<=24), next 3 bytes - formats for each nmea filed (LSB first):
// 0 - nibble ("-" -> 0xd, "." -> 0xe), terminated with 0xf
// 1 - byte (2 nibbles), all bytes but last have MSB clear, last - set.
// No padding of nibbles to byte borders, bytes are encoded as 2 nibbles
  myRAM_WxD_D #( .DATA_WIDTH(8),.DATA_DEPTH(4))
            i_format   (.D(wd[7:0]),
                       .WE(we & wa[4]), // we_d, decoded sub_address
                       .clk(!sclk),
                       .AW(wa[3:0]),
                       .AR({sentence[1:0],format_field[4:3]}),
                       .QW(),
                       .QR(format_data[7:0]));
// ROM to decode "$GP"                     
  always @ (posedge xclk) begin
    if (ser_stb) case ({(bitnum[4] & ~ vfy_sel_sent) | vfy_first_comma, bitnum[3] | vfy_sel_sent | vfy_first_comma, bitnum[2:0]}) // during vfy_sel_sent will point to 1 ('G')
      5'h00:  gp_exp_bit <= 1'b0; //$
      5'h01:  gp_exp_bit <= 1'b0;
      5'h02:  gp_exp_bit <= 1'b1;
      5'h03:  gp_exp_bit <= 1'b0;
      5'h04:  gp_exp_bit <= 1'b0;
      5'h05:  gp_exp_bit <= 1'b1;
      5'h06:  gp_exp_bit <= 1'b0;
      5'h07:  gp_exp_bit <= 1'b0;
      5'h08:  gp_exp_bit <= 1'b1; //G
      5'h09:  gp_exp_bit <= 1'b1;
      5'h0a:  gp_exp_bit <= 1'b1;
      5'h0b:  gp_exp_bit <= 1'b0;
      5'h0c:  gp_exp_bit <= 1'b0;
      5'h0d:  gp_exp_bit <= 1'b0;
      5'h0e:  gp_exp_bit <= 1'b1;
      5'h0f:  gp_exp_bit <= 1'b0;
      5'h10:  gp_exp_bit <= 1'b0; //P
      5'h11:  gp_exp_bit <= 1'b0;
      5'h12:  gp_exp_bit <= 1'b0;
      5'h13:  gp_exp_bit <= 1'b0;
      5'h14:  gp_exp_bit <= 1'b1;
      5'h15:  gp_exp_bit <= 1'b0;
      5'h16:  gp_exp_bit <= 1'b1;
      5'h17:  gp_exp_bit <= 1'b0;
      5'h18:  gp_exp_bit <= 1'b0; //'h2c: "," - will use later - attach first comma to $GPxxx,
      5'h19:  gp_exp_bit <= 1'b0;
      5'h1a:  gp_exp_bit <= 1'b1;
      5'h1b:  gp_exp_bit <= 1'b1;
      5'h1c:  gp_exp_bit <= 1'b0;
      5'h1d:  gp_exp_bit <= 1'b1;
      5'h1e:  gp_exp_bit <= 1'b0;
      5'h1f:  gp_exp_bit <= 1'b0;
      default:gp_exp_bit <= 1'bX;
    endcase
  end
endmodule

module imu_timestamps (
                        sclk, // 160MHz, negedge            
                        xclk, // 80 MHz, posedge
                        rst,  // reset (@posedge xclk)
                        sec,  // running seconds (@negedge sclk)
                        usec, // running microseconds (@negedge sclk)
                        ts_rq,// requests to create timestamps (4 channels), @posedge xclk
                        ts_ackn, // timestamp for this channel is stored
                        ra,   // read address (2 MSBs - channel number, 2 LSBs - usec_low, (usec_high ORed with channel <<24), sec_low, sec_high
                        dout);// output data
   input         sclk;
   input         xclk;
   input         rst;
   input  [31:0] sec;
   input  [19:0] usec;
   input  [ 3:0] ts_rq;
   output [ 3:0] ts_ackn;
   input  [ 3:0] ra;
   output [15:0] dout;
   
   reg    [31:0] sec_latched;
   reg    [19:0] usec_latched;
   reg    [15:0] ts_mux;
   
   reg    [ 3:0] wa;
   reg           srst;
   reg    [3:0]  rq_d;
   reg    [3:0]  rq_d2;
   reg    [3:0]  rq_r;
   reg    [3:0]  rq_sclk;
   reg    [3:0]  rq_sclk2;
   reg    [3:0]  pri_sclk;
   reg    [3:0]  pri_sclk_d;
   reg    [3:0]  rst_rq;
   reg    [9:0]  proc;
   wire          wstart;
   reg           we;
   wire   [3:0]  wrst_rq;
   reg    [3:0]  ts_preackn;
   reg    [3:0]  ts_ackn;
   assign        wstart=|pri_sclk[3:0] && (pri_sclk[3:0] != pri_sclk_d[3:0]);
   assign        wrst_rq[3:0]={wa[3]&wa[2],wa[3]&~wa[2],~wa[3]&wa[2],~wa[3]&~wa[2]} & {4{proc[5]}};
   always @ (posedge xclk) begin
     rq_d[3:0] <= ts_rq[3:0];
     rq_d2[3:0] <= rq_d[3:0];
   end
   
   always @ (negedge sclk) begin
     srst <= rst;
     rq_sclk[3:0]  <= srst?4'h0:(~rst_rq[3:0] & (rq_r[3:0] | rq_sclk[3:0])) ;
     rq_sclk2[3:0] <= srst?4'h0:(~rst_rq[3:0] & rq_sclk[3:0]) ;
     pri_sclk[3:0] <= {rq_sclk2[3] & ~|rq_sclk2[2:0],
                       rq_sclk2[2] & ~|rq_sclk2[1:0],
                       rq_sclk2[1] &  ~rq_sclk2[0],
                       rq_sclk2[0]};
     pri_sclk_d[3:0] <= pri_sclk[3:0];
     proc[9:0] <= {proc[8:0], wstart};
     if (proc[0]) wa[3:2] <= {|pri_sclk_d[3:2], pri_sclk_d[3] | pri_sclk_d[1]};
     if (proc[0]) sec_latched[31:0] <= sec[31:0];
     if (proc[0]) usec_latched[19:0] <= usec[19:0];
//     if (proc[2]) ts_mux[15:0] <= {6'h0,wa[3:2],4'h0,usec[19:16]};
     casex({proc[8],proc[6],proc[4],proc[2]})
//       4'bXXX1: ts_mux[15:0] <= {6'h0,wa[3:2],4'h0,usec_latched[19:16]};
//       4'bXX1X: ts_mux[15:0] <= usec_latched[15: 0];
//       4'bX1XX: ts_mux[15:0] <=  sec_latched[31:16];
//       4'b1XXX: ts_mux[15:0] <=  sec_latched[15: 0];
       4'bXXX1: ts_mux[15:0] <= usec_latched[15: 0];
       4'bXX1X: ts_mux[15:0] <= {6'h0,wa[3:2],4'h0,usec_latched[19:16]};
       4'bX1XX: ts_mux[15:0] <= sec_latched[15: 0];
       4'b1XXX: ts_mux[15:0] <= sec_latched[31:16];

     endcase
     we <= proc[3] || proc[5] || proc[7] || proc[9];
     if (proc[2]) wa[1:0] <= 2'b0;
     else if (we) wa[1:0] <= wa[1:0] + 1;
     rst_rq[3:0] <= wrst_rq[3:0] | {4{srst}};
   end
   always @ (posedge xclk or posedge rq_sclk2[0]) begin
     if       (rq_sclk2[0])          rq_r[0] <= 1'b0;
     else  if (srst)                 rq_r[0] <= 1'b0;
     else  if (rq_d[0] && !rq_d2[0]) rq_r[0] <= 1'b1;
   end
   always @ (posedge xclk or posedge rq_sclk2[1]) begin
     if       (rq_sclk2[1])          rq_r[1] <= 1'b0;
     else  if (srst)                 rq_r[1] <= 1'b0;
     else  if (rq_d[1] && !rq_d2[1]) rq_r[1] <= 1'b1;
   end
   always @ (posedge xclk or posedge rq_sclk2[2]) begin
     if       (rq_sclk2[2])          rq_r[2] <= 1'b0;
     else  if (srst)                 rq_r[2] <= 1'b0;
     else  if (rq_d[2] && !rq_d2[2]) rq_r[2] <= 1'b1;
   end
   always @ (posedge xclk or posedge rq_sclk2[3]) begin
     if       (rq_sclk2[3])          rq_r[3] <= 1'b0;
     else  if (srst)                 rq_r[3] <= 1'b0;
     else  if (rq_d[3] && !rq_d2[3]) rq_r[3] <= 1'b1;
   end

   always @ (posedge xclk or posedge rst_rq[0]) begin
     if       (rst_rq[0])  ts_preackn[0] <= 1'b1;
     else  if (!ts_rq[0])  ts_preackn[0] <= 1'b0;
   end
   always @ (posedge xclk or posedge rst_rq[1]) begin
     if       (rst_rq[1])  ts_preackn[1] <= 1'b1;
     else  if (!ts_rq[1])  ts_preackn[1] <= 1'b0;
   end
   always @ (posedge xclk or posedge rst_rq[2]) begin
     if       (rst_rq[2])  ts_preackn[2] <= 1'b1;
     else  if (!ts_rq[2])  ts_preackn[2] <= 1'b0;
   end
   always @ (posedge xclk or posedge rst_rq[3]) begin
     if       (rst_rq[3])  ts_preackn[3] <= 1'b1;
     else  if (!ts_rq[3])  ts_preackn[3] <= 1'b0;
   end

   always @ (posedge xclk) begin
     ts_ackn[3:0] <= ts_preackn[3:0] & ts_rq[3:0];
   end

  myRAM_WxD_D #( .DATA_WIDTH(16),.DATA_DEPTH(4))
            i_ts   (.D(ts_mux[15:0]),
                       .WE(we), // we_d, decoded sub_address
                       .clk(!sclk),
                       .AW(wa[3:0]),
                       .AR(ra[3:0]),
                       .QW(),
                       .QR(dout[15:0]));
                       
endmodule 
