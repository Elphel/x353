/** -----------------------------------------------------------------------------**
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
module three_motor_driver ( clk,   // system clock, negedge
                            xclk,  // half frequency (80 MHz nominal)
                            we,    // write enable (lower 16 bits, high - next cycle)
                            wa,    // write address(1)/data(0)
                            di,    // 16-bit data in (32 multiplexed)
                            do,    // 16-bit data output
                            encod1, // 2-bit encoder data input, motor1
                            encod2, // 2-bit encoder data input, motor2
                            encod3, // 2-bit encoder data input, motor3
                            mot1,   // 2 bits motor1 control output (11 - shorted, 00 - stop)
                            mot2,   // 2 bits motor1 control output (11 - shorted, 00 - stop)
                            mot3   // 2 bits motor1 control output (11 - shorted, 00 - stop)
                           );
  parameter DATA_WIDTH=24;                       
  input         clk;   // system clock, negedge
  input         xclk;  // half frequency (80 MHz nominal)
  input         we;    // write enable (lower 16 bits, high - next cycle)
  input         wa;    // write address(1)/data(0)
  input  [15:0] di;    // 16-bit data in (32 multiplexed)
  output [31:0] do;    // 16-bit data output
  input   [1:0] encod1; // 2-bit encoder data input, motor1
  input   [1:0] encod2; // 2-bit encoder data input, motor2
  input   [1:0] encod3; // 2-bit encoder data input, motor3
  output  [1:0] mot1;   // 2 bits motor1 control output (11 - shorted, 00 - stop)
  output  [1:0] mot2;   // 2 bits motor1 control output (11 - shorted, 00 - stop)
  output  [1:0] mot3;   // 2 bits motor1 control output (11 - shorted, 00 - stop)

  reg    [10:0] addr; // 11-th bit only for table readback
  reg           we_d; // only if wa was 0
  reg           we_pos;
  reg           we_ctl;
  reg           we_timing;
  reg           we_timing1;
  reg           we_decrement_on_pulse;
  reg     [7:0] pwm_cycle; // nominal 67 (0x43)
  reg     [2:0] pwm_delay; // nominal 2 - 0.3 usec
  reg     [3:0] deglitch_div; // nominal 7 ~=1 MHz

// for metastability reduction on the clock domain boarder
  reg     [7:0] pre_pwm_cycle; // nominal 67 (0x43)
  reg     [2:0] pre_pwm_delay; // nominal 2 - 0.3 usec
  reg     [3:0] pre_deglitch_div; // nominal 7 ~=1 MHz
  reg    [11:0] enc_period_cycle; // 1111 (0x457 ) so period measurement counter increments 6 times per millisecond (full speed encoder period)
                              // adjust to actual speed - period is floating point, maximal precision for 0..8 counts
  reg    [11:0] pre_enc_period_cycle;
  reg    [ 4:0] dec_on_pulse;
  reg    [ 4:0] pre_dec_on_pulse;

  reg    [15:0] di_d;
  reg           en_lut; // just to save energy
  wire    [3:0] pwm_code;  //code to PWM module.0x20 - stop, else pwm_code[3] - direction, pwm_code[2:0] - PWM duty factor (0 - 0%, 1 - 25%, 2 - 27.5%, ..,7 - 100%)
  wire    [3:0] reg_addr; //={addr[1], addr[0] ^ (~addr[1]), addr[2],addr[3]}; // 0..3 - current positions, next - debug data (deglitch, period, pwm. 0 - control, 1..3 - motors

  wire   [DATA_WIDTH-1:0] state_next;
  wire   [DATA_WIDTH-1:0] state_current;
  wire    [3:0] sequence;
  wire    [3:0] sequence_next;
  wire   [DATA_WIDTH-1:0] target_pos;
  wire          reset_positions_clk;
  wire          enable_mot_clk;
  reg           pre_enable_mot,  enable_mot;
  reg           pre_reset_positions,  reset_positions;
  reg           pre_first;
  wire    [1:0] encoder;
  assign        encoder[1:0]=sequence[3]?(sequence[2]? encod3[1:0]:encod2[1:0]):encod1[1:0];

//  wire    [3:0] reg_addr={addr[1], addr[0] ^ (~addr[1]), addr[2],addr[3]}; // 0..3 - current positions, next - debug data (deglitch, period, pwm. 0 - control, 1..3 - motors
//  assign read_addr[3:0]={reg_addr[1:0],reg_addr[3], ~reg_addr[2]}; //reg_addr[3:0]
  assign reg_addr[3:0]={addr[1:0],addr[3], ~addr[2]};



  always @ (negedge clk) begin
    we_d       <= ~wa & we;
    we_pos     <= we && (!wa) && (addr[9:2]==0);
    we_ctl     <= we && (!wa) && (addr[9:0]==4);
    we_timing  <= we && (!wa) && (addr[9:0]==5);
    we_timing1 <= we && (!wa) && (addr[9:0]==6);
    we_decrement_on_pulse <= we && (!wa) && (addr[9:0]==7);
    if (we_timing) begin
      pre_pwm_cycle[7:0]    <= di_d[ 7: 0];
      pre_pwm_delay[2:0]    <= di_d[10: 8];
      pre_deglitch_div[3:0] <= di_d[15:12];
    end
    if (we_timing1) begin
      pre_enc_period_cycle[11:0] <= di_d[11:0];
    end
    if (we_decrement_on_pulse) begin
      pre_dec_on_pulse[4:0] <= di_d[4:0];
    end

  end
  always @ (negedge clk) begin
   if (we)      di_d[15:0] <= di[15:0];
   if (we & wa) addr[10:9]   <= di[10:9] ;
   if (we & wa) addr[8:0] <= di[8:0] ;
   else if (we_d )  addr[8:0] <= addr[8:0]+1 ; // autoincrement, but in the same device only
  end



FDCE_1 i_reset_positions_clk(.C(clk),.CE(we_ctl && di_d[2]),.CLR(reset_positions),.D(1'b1),   .Q(reset_positions_clk));
FDCE_1 i_enable_mot_clk     (.C(clk),.CE(we_ctl && di_d[1]),.CLR(1'b0),           .D(di_d[0]), .Q(enable_mot_clk));
  always @ (posedge xclk) begin
      pwm_cycle[7:0]    <= pre_pwm_cycle[7:0];
      pwm_delay[2:0]    <= pre_pwm_delay[2:0];
      deglitch_div[3:0] <= pre_deglitch_div[3:0];
      enc_period_cycle[11:0] <= pre_enc_period_cycle[11:0];
      dec_on_pulse[4:0] <= pre_dec_on_pulse[4:0];
      pre_reset_positions <= reset_positions_clk;
      if (pre_first) reset_positions <= pre_reset_positions; // single sequence period;
      pre_enable_mot            <= enable_mot_clk;
      if (pre_first) enable_mot <= pre_enable_mot;
  end

// reset at simulation
  assign sequence_next[1:0]=sequence[1:0]+1;
  assign sequence_next[3:2]=(sequence[1:0]==3'b11)?((sequence[3:2]==3'b11)?2'b01:(sequence[3:2]+1)):sequence[3:2];
  FD i_sequence_0(.C(xclk),.D(sequence_next[0]), .Q(sequence[0]));
  FD i_sequence_1(.C(xclk),.D(sequence_next[1]), .Q(sequence[1]));
  FD i_sequence_2(.C(xclk),.D(sequence_next[2]), .Q(sequence[2]));
  FD i_sequence_3(.C(xclk),.D(sequence_next[3]), .Q(sequence[3]));
  always @ (posedge xclk) begin
       pre_first <=(sequence[3:0]==4'b1110);
  end

/// First cycle: deglitching
//synthesis translate_off
  defparam i_deglitch_encoder.IGNORE_EN = 1'b0;
//synthesis translate_on
  wire  [8:0]  period_cur=state_current[15:7] ;
  wire  [8:0]  period_new;  // multiplex to state_next[15:7]
  wire  [4:0]  deglitch_cur=state_current[6:2] ;
  wire  [4:0]  deglitch_new;  // multiplex to state_next[6:2]
  wire  [1:0]  encoder_used=state_current[1:0] ;
  wire  [1:0]  encoder_new;   //  multiplex to state_next[1:0]
  wire         pre_incdec; // not registered
  wire         inc, dec, inc_dec; // registered inside deglitch_encoder
  wire  [4:0]  encoded_period;
  wire [DATA_WIDTH-1:0]  rdo; // position/register file read data
  wire [15:0]  tdo; // table readback output
  wire [31:0]  do;  // multiplexed readback
  assign do[31:0]=addr[9]?{16'h0,tdo[15:0]}:{{(32-DATA_WIDTH){rdo[DATA_WIDTH-1]}},rdo[DATA_WIDTH-1:0]};

  deglitch_encoder i_deglitch_encoder
             (.clk(xclk),                       // posedge, 80 MHz
              .en_in(!reset_positions),         // enable (just for simulation), for real - let it on even when motors are off
              .process(sequence[1:0]==2'h0),                    // process data this cycle (store inc, dec)
              .pre_first(pre_first),            // next whill be the first motor in a cycle (may use just spread bits)
              .cycle_div(deglitch_div[3:0]),    // [3:0] clock divisor - nominally 1/7 (80/4/3/7~=1MHz)
              .deglitch_cur(deglitch_cur[4:0]), // [4:0] current value of deglitch counter for this channel , read from RAM 
              .deglitch_new(deglitch_new[4:0]), // [4:0] next value of deglitch counter for this channel, to write to RAM 
              .encoder(encoder[1:0]),           // [1:0] current encoder data
              .encoder_used(encoder_used[1:0]), // [1:0] encoder data from RAM (deglitched)
              .encoder_new(encoder_new[1:0]),   // encoder data to RAM (deglitched)
              .pre_incdec(pre_incdec),          // combinatorial output, valid same cucle
              .inc(inc),                        // increment position counter, registered. On top level apply inc/dec to the next cycle, not to the current
              .dec(dec),                        // decrement position counter, registered
              .inc_dec(inc_dec)                 // increment or decrement position counter, registered
               );
//synthesis translate_off
  defparam i_period_encoder.IGNORE_EN = 1'b0;
//synthesis translate_on
 period_encoder i_period_encoder(
               .clk(xclk),                          // posedge, 80 MHz
               .en_in(!reset_positions),            // enable (just for simulation), for real - let it on even when motors are off
               .pre_first(pre_first),               // next will be the first motor in a cycle (may use just spread bits)
               .process(sequence[1:0]==2'h0),       // increment (with limit) period
               .inc_dec(pre_incdec),                // encoder pulse detected
               .cycle_div(enc_period_cycle[11:0]),  // [11:0] clock divisor - nominally 1111 (0x457 ), 6KHz (6 cycles per encoder phase, full speed)
               .decr_on_pulse(dec_on_pulse[4:0]),   // [4:0] decrease calculated period code by this amount after the encoder pulse. Normally - just 1?
               .period_cur(period_cur[8:0]),    // [8:0] current period from register file,
               .period_new(period_new[8:0]),    // [8:0] updated period to be stored back to register file
               .encoded_period(encoded_period[4:0]) //4:0] encoded period (registered, valid 2 cycles after process
               );

/// Second cycle: updating position, calculating position error
  wire [DATA_WIDTH-1:0] position_next; // multiplex to state_next[15:0]
  wire [DATA_WIDTH:0] position_diff;
  wire [ 5:0] minus_diff;
  reg  [ 5:0] position_err; // saturated and registered position_diff
//  assign position_next[15:0]= (reset_positions || (sequence[1:0]!=2'h1))? 16'h0:(inc?(state_current[15:0]+1):(dec?(state_current[15:0]-1):(state_current[15:0]) ) );
  assign position_next[DATA_WIDTH-1:0]= reset_positions? target_pos[DATA_WIDTH-1:0]:
                                       (inc?(state_current[DATA_WIDTH-1:0]+1):
                                       (dec?(state_current[DATA_WIDTH-1:0]-1):(state_current[DATA_WIDTH-1:0]) ) );
//  assign position_diff={target_pos[15],target_pos[15:0]}-{state_current[15],state_current[15:0]};
//  assign minus_diff[4:0]=  state_current[4:0] - target_pos[4:0];
  assign position_diff={state_current[DATA_WIDTH-1],state_current[DATA_WIDTH-1:0]}-{target_pos[DATA_WIDTH-1],target_pos[DATA_WIDTH-1:0]};
  assign minus_diff[5:0]=  target_pos[5:0]-state_current[5:0];
  always @ (posedge xclk) if (sequence[1:0]==2'h1) begin
       position_err[5:0] <= (position_diff[DATA_WIDTH:5]=={(DATA_WIDTH-4){1'b0}})  ? position_diff[5:0]:
                           ((position_diff[DATA_WIDTH:5]=={(DATA_WIDTH-4){1'b1}})? {1'b1,minus_diff[4:0]|{5{minus_diff[5]}}}:{position_diff[DATA_WIDTH],5'h1f});
  end
/// third cycle: calculating speed, using RAM table to calculate pwm code

/// Combine old (freom register file) /new encoded periods, inc/dec pulses to prepare a 6-bit (partial) index for a RAM table.
/// Together with the position error (another 6 bits) the full 12-bit undex provides 1 4-bit PWM code from the RAM table
/// Updates direction immediately, period - if inc_dec or when new period > saved period (so it will be updated even if the motor is stopped)
  wire         dir_last=       state_current[5];   // last stored direction (from register file)
  wire [4:0]   enc_period_last=state_current[4:0]; // [4:0] last encoded period (from register file)
  wire         dir_this;                           // new value of direction to be stored in the register file and used in the index, multiplex to state_next[5]
  wire [4:0]   enc_period_this;                    // [4:0] encoded period for the register file/ table index, multiplex to state_next[4:0]

//synthesis translate_off
  defparam i_calc_speed.IGNORE_EN = 1'b0;
//synthesis translate_on
 calc_speed i_calc_speed(
               .clk(xclk),             // posedge, 80 MHz
               .en_in(!reset_positions),           // enable (just for simulation), for real - let it on even when motors are off
               .process(sequence[1:0]==2'h2),      // increment (with limit) period
               .inc_dec(inc_dec),         // encoder pulse detected
               .inc(inc),             // encoder position increment
               .dec(dec),             // encoder position decrement
               .enc_period_curr(encoded_period[4:0]), // encoded current period (still running if !inc_dec)
               .dir_last(dir_last),        // last stored direction (from register file)
               .enc_period_last(enc_period_last[4:0]), // [4:0] last encoded period (from register file)
               .dir_this(dir_this),        // new value of direction to be stored in the register file and used in the index
               .enc_period_this(enc_period_this[4:0])  // [4:0] encoded period for the register file/ table index
               );

/// fourth cycle: processing PWM
  wire [2:0] cur_pwm=state_current[2:0];
  wire [2:0] new_pwm; // multiplex to state_next[2:0]
  wire [1:0] next_mot;
  reg        set_mot;
  reg  [1:0] mot1;   // 2 bits motor1 control output (11 - shorted, 00 - stop)
  reg  [1:0] mot2;   // 2 bits motor1 control output (11 - shorted, 00 - stop)
  reg  [1:0] mot3;   // 2 bits motor1 control output (11 - shorted, 00 - stop)

 motor_pwm i_motor_pwm(
               .clk(xclk),                    // posedge, 80 MHz
               .en(enable_mot),               // enable, 0 turns off motors and resets counterts
               .pre_first(pre_first),         // next whill be the first motor in a cycle (may use just spread bits)
               .pwm_delay(pwm_delay[2:0]),    // [2:0] - turns off bridge during transition (i.e. 10->00->11->00->10 ..). Nominal 0.3usec - 2 (0x18) cycles (80/4/3=6.66MHz)
               .pwm_cycle(pwm_cycle[7:0]),    // [7:0] - pwm cycle duration Nominal 10usec or 67 (0x43) 6.66MHz cycles. Total PWM period will be 8 of these cycles
               .spread({sequence[3:2],1'b0}), // [2:0] shift on phase between motors, adds this number to the current phase (use just 2 MSBs for 3 motors)
               .cur_pwm(cur_pwm[2:0]),        // [2:0] - PWM data stored in per-motor memory. [1]  - 'on', [0] direction, [2] - enable. That bit is used to turn off motor during PWM transitions
               .pwm_code(pwm_code[3:0]),      // [3:0] - code from the RAM. 0x8 - stop,  , otherwise [3] - direction, [2:0]/8 - duty cycle (8 - 100% on), 0 - 0%, 1 - 25%, 3 - 37.5%...
               .new_pwm(new_pwm[2:0]),        // [2:0]
               .mot(next_mot[1:0])            // [1:0] - data to be copied to the motor outputs
                );

  always @ (posedge xclk)  begin
     set_mot <= (sequence[1:0]==2'h2); // active during last (fourth) cycle
     if (set_mot && !sequence[3])                 mot1[1:0] <=next_mot[1:0];
     if (set_mot &&  sequence[3] && !sequence[2]) mot2[1:0] <=next_mot[1:0];
     if (set_mot &&  sequence[3] &&  sequence[2]) mot3[1:0] <=next_mot[1:0];
  end

/// Multiplex register file input from multiple bit fields, depending on the sequence state

/*
0:  wire  [8:0]  period_new;        // multiplex to state_next[15:7]
    wire  [4:0]  deglitch_new;      // multiplex to state_next[6:2]
    wire  [1:0]  encoder_new;       //  multiplex to state_next[1:0]

1:   wire [15:0] position_next;     // multiplex to state_next[15:0]

2:   wire         dir_this;         // new value of direction to be stored in the register file and used in the index, multiplex to state_next[5]
     wire [4:0]   enc_period_this;  // [4:0] encoded period for the register file/ table index, multiplex to state_next[4:0]

3:  wire [2:0] new_pwm;             // multiplex to state_next[2:0]

*/
   always @ (posedge xclk)  begin
     en_lut <= (sequence[1:0]==2'h1); // valid at sequence[1:0]==2'h2
   end

    assign state_next[DATA_WIDTH-1:0]=sequence[1]?
                             (sequence[0]?{{(DATA_WIDTH-3){1'b0}},new_pwm[2:0]}:
                                          {{(DATA_WIDTH-6){1'b0}},dir_this,enc_period_this[4:0]}):
                             (sequence[0]?position_next[DATA_WIDTH-1:0]:
                             {{(DATA_WIDTH-16){1'b0}},period_new[8:0],deglitch_new[4:0],encoder_new[1:0]}); // {0{1'b0}} OK
//Instance of motor table RAM, defines behaviour of the motor depending on current position and speed
// May add data initialization to theis memory
   RAMB16_S4_S18  i_motor_ram (
      .CLKA(xclk),               // Port A Clock
      .ENA(en_lut),              // Port A RAM Enable Input
      .WEA(1'b0),                // Port A Write Enable Input
      .SSRA(1'b0),               // Port A Synchronous Set/Reset Input
      .ADDRA({position_err[5:0],dir_this,enc_period_this[4:0]}),   // Port A 12-bit Address Input
      .DIA(4'b0),                // Port A 4-bit Data Input
      .DOA(pwm_code[3:0]),       // Port A 4-bit Data Output registered?

      .CLKB(!clk),               // Port B Clock
//      .ENB(addr[9] && ((~wa && we) || we_d)),           // Port B RAM Enable Input
//      .WEB(1'b1),                // Port B Write Enable Input
      .ENB(1'b1),           // Port B RAM Enable Input
      .WEB(addr[9] && ((~wa && we) || we_d)),                // Port B Write Enable Input
//      .ADDRB({addr[8:0],we_d}),  // Port B 10-bit Address Input
      .ADDRB({addr[8:0],(we | we_d)?we_d:addr[10]}),  // Port B 10-bit Address Input
      .DIB(di[15:0]),            // Port B 16-bit Data Input
      .DIPB(2'b0),               // Port-B 2-bit parity Input
      .DOB(tdo[15:0]),           // Port B 16-bit Data Output
      .DOPB(),                   // Port B 2-bit Parity Output
      .SSRB(1'b0)                // Port B Synchronous Set/Reset Input
   );
   myRAM_WxD_D_1 #( .DATA_WIDTH(DATA_WIDTH),.DATA_DEPTH(2))
             i_target_pos   (.D((DATA_WIDTH>16)?{di[DATA_WIDTH-17:0],di_d[15:0]}:di_d[15:0]),
                             .WE(we_pos),
                             .clk(clk),
                             .AW(addr[1:0]),
                             .AR(sequence[3:2]),
                             .QW(),
                             .QR(target_pos[DATA_WIDTH-1:0]));
//   wire [3:0] read_addr; // reorder addresses, so loc 1..3 will read current positions of the motors
//   assign read_addr[3:0]={reg_addr[1:0],reg_addr[3], ~reg_addr[2]}; //reg_addr[3:0]
   myRAM_WxD_D #( .DATA_WIDTH(DATA_WIDTH),.DATA_DEPTH(4))
            i_current_state   (.D(state_next[DATA_WIDTH-1:0]),
                             .WE(1'b1),
                             .clk(xclk),
                             .AW(sequence[3:0]),
//                             .AR(read_addr[3:0]),
                             .AR(reg_addr[3:0]),
                             .QW(state_current[DATA_WIDTH-1:0]),
                             .QR(rdo[DATA_WIDTH-1:0]));


endmodule


/// Combine old (freom register file) /new encoded periods, inc/dec pulses to prepare a 6-bit (partial) index for a RAM table.
/// Together with the position error (another 6 bits) the full 12-bit undex provides 1 4-bit PWM code from the RAM table
/// Updates direction immediately, period - if inc_dec or when new period > saved period (so it will be updated even if the motor is stopped)
module calc_speed (clk,             // posedge, 80 MHz
                   en_in,           // enable (just for simulation), for real - let it on even when motors are off
                   process,         // increment (with limit) period
                   inc_dec,         // encoder pulse detected
                   inc,             // encoder position increment
                   dec,             // encoder position decrement
                   enc_period_curr, // encoded current period (still running if !inc_dec)
                   dir_last,        // last stored direction (from register file)
                   enc_period_last, // [4:0] last encoded period (from register file)
                   dir_this,        // new value of direction to be stored in the register file and used in the index
                   enc_period_this  // [4:0] encoded period for the register file/ table index
                   );
 parameter IGNORE_EN=1;
 input         clk;             // posedge, 80MHz
 input         en_in;           // enable, 0 resets period counter - simulation only
 input         process;         // increment (with limit) period
 input         inc_dec;         // encoder pulse detected
 input         inc;             // encoder position increment
 input         dec;             // encoder position decrement
 input   [4:0] enc_period_curr; // encoded current period (still running if !inc_dec)
 input         dir_last;        // last stored direction (from register file)
 input   [4:0] enc_period_last; // [4:0] last encoded period (from register file)
 output        dir_this;        // new value of direction to be stored in the register file and used in the index
 output  [4:0] enc_period_this; // [4:0] encoded period for the register file/ table index

 wire          en=(IGNORE_EN)?1'b1:en_in; // will be overwritten, so en will be used in simulation only, ignored in synthesis

 wire          dir_this;
 wire    [4:0] enc_period_this;
 wire    [5:0] diff;
 assign dir_this=              en & (inc_dec?dec:dir_last);
 assign diff[5:0]=            {1'b0,enc_period_curr}-{1'b0,enc_period_last};
 assign enc_period_this[4:0]= en ? ((diff[5] && !inc_dec)?enc_period_last[4:0]:enc_period_curr[4:0]):5'b0;
endmodule


module period_encoder (clk,           // posedge, 80 MHz
                       en_in,         // enable (just for simulation), for real - let it on even when motors are off
                       pre_first,     // next whill be the first motor in a cycle (may use just spread bits)
                       process,       // increment (with limit) period
                       inc_dec,       // encoder pulse detected
                       cycle_div,     // [11:0] clock divisor - nominally 1111 (0x457 ), 6KHz (6 cycles per encoder phase, full speed)
                       decr_on_pulse, // [4:0] decrease calculated period code by this amount after the encoder pulse. Normally - just 1?
                       period_cur,    // [8:0] current period from register file,
                       period_new,    // [8:0] updated period to be stored back to register file
                       encoded_period //4:0] encoded period (registered, valid 2 cycles after process
                      );
 parameter IGNORE_EN=1;
 input         clk;            // posedge, 80MHz
 input         en_in;          // enable, 0 resets period counter - simulation only
 input         process;        // increment (with limit) period
 input         pre_first;      // next whill be the first motor in a cycle (may use just spread bits)
 input         inc_dec;        // encoder pulse detected
 input  [11:0] cycle_div;      // [11:0] clock divisor - nominally 1111 (0x457 ), 6KHz (6 cycles per encoder phase, full speed)
 input   [4:0] decr_on_pulse;  // decrease calculated period code by this amount after the encoder pulse. Normally - just 1?
 input   [8:0] period_cur;     // [8:0] current period from register file,
 output  [8:0] period_new;     // [8:0] updated period to be stored back to register file
 output  [4:0] encoded_period; //4:0] encoded period (registered, valid 2 cycles after process

 wire          en=(IGNORE_EN)?1'b1:en_in; // will be overwritten, so en will be used in simulation only, ignored in synthesis
 
 reg    [11:0] prescaler;
 reg           inc_period;
 wire   [8:0]  period_new;
 wire   [6:0]  pri_enc;
 reg    [4:0]  pre_enc_per; // valid next cycle, not decremented after the pulse
 reg           inc_dec_d;   // next cycle after inc_dec
 reg    [4:0]  encoded_period;
 wire   [5:0]  decreased_pre_enc_per;
 reg           process_d;
// prescaler counter, inc_period should have frequency ~6 times (<8) higher than maximal encoder phase chnge frequency
 always @ (posedge clk) if (pre_first) begin // change these registers once per cycle of all motors
   prescaler=    (!en || (prescaler==0))?cycle_div:(prescaler-1);
   inc_period=   en && (prescaler==0);
 end
// assign        period_new[8:0]=(inc_dec || !en)?9'h0:
 assign        period_new[8:0]=(inc_dec || !en)?{9{~en}}: /// reset to longest period (lowest speed)
                               ((!inc_period || (period_cur[8:6]==3'h7))? period_cur[8:0]:(period_cur[8:0]+1));

 assign pri_enc={period_cur[8],
                 (period_cur[8:7]==2'h1),
                 (period_cur[8:6]==3'h1),
                 (period_cur[8:5]==4'h1),
                 (period_cur[8:4]==5'h1),
                 (period_cur[8:3]==6'h1),
                 (period_cur[8:3]==6'h0)};

 always @ (posedge clk) if (process) begin
   pre_enc_per[4:0] <= {|pri_enc[6:3],
                        |pri_enc[6:5] | |pri_enc[2:1],
                         pri_enc[6] | pri_enc[4] |  pri_enc[2] |  (pri_enc[0] & period_cur[2]),
                         (pri_enc[6] & period_cur[7]) |
                         (pri_enc[5] & period_cur[6]) |
                         (pri_enc[4] & period_cur[5]) |
                         (pri_enc[3] & period_cur[4]) |
                         (pri_enc[2] & period_cur[3]) |
                         (pri_enc[1] & period_cur[2]) |
                         (pri_enc[0] & period_cur[1]),
                         (pri_enc[6] & period_cur[6]) |
                         (pri_enc[5] & period_cur[5]) |
                         (pri_enc[4] & period_cur[4]) |
                         (pri_enc[3] & period_cur[3]) |
                         (pri_enc[2] & period_cur[2]) |
                         (pri_enc[1] & period_cur[1]) |
                         (pri_enc[0] & period_cur[0]) };
   inc_dec_d <= inc_dec;
 end
 assign  decreased_pre_enc_per[5:0] = {1'b0,pre_enc_per[4:0]}-{1'b0,decr_on_pulse};

 always @ (posedge clk)  process_d <= process;

 always @ (posedge clk) if (process_d) begin
   encoded_period[4:0] <= inc_dec_d? (decreased_pre_enc_per[5]? 5'h0:decreased_pre_enc_per[4:0]):(pre_enc_per[4:0]);
 end


endmodule


// 1 cycle - deglitch,
// 2-cycle - upgdate current position counter (register it)
// 3 cycle - read target position, subtract current
//1111 (0x457 )

module deglitch_encoder (clk,          // posedge, 80 MHz
                         en_in,        // enable (just for simulation), for real - let it on even when motors are off
                         process,      // process data this cycle (store inc, dec)
                         pre_first,    // next whill be the first motor in a cycle (may use just spread bits)
                         cycle_div,    // [3:0] clock divisor - nominally 1/7 (80/4/3/7~=1MHz)
                         deglitch_cur, // [4:0] current value of deglitch counter for this channel , read from RAM 
                         deglitch_new, // [4:0] next value of deglitch counter for this channel, to write to RAM 
                         encoder,      // [1:0] current encoder data
                         encoder_used, // [1:0] encoder data from RAM (deglitched)
                         encoder_new,  // encoder data to RAM (deglitched)
                         pre_incdec,   // combinatorial output, valid same cucle
                         inc,          // increment position counter, registered. On top level apply inc/dec to the next cycle, not to the current
                         dec,          // decrement position counter, registered
                         inc_dec       // increment or decrement position counter, registered
                        );
 parameter IGNORE_EN=1;
 input         clk;          // posedge, 80MHz
 input         en_in;        // enable, 0 turns off motors and resets counterts
 input         process;      // process data this cycle (store inc, dec)
 input         pre_first;    // next whill be the first motor in a cycle (may use just spread bits)
 input   [3:0] cycle_div;    // [3:0] clock divisor - nominally 1/7 (80/4/3/7~=1MHz)
 input   [4:0] deglitch_cur; // [4:0] current value of deglitch counter for this channel , read from RAM 
 output  [4:0] deglitch_new; // [4:0] next value of deglitch counter for this channel, to write to RAM 
 input   [1:0] encoder;      // current encoder data
 input   [1:0] encoder_used; // encoder data from RAM (deglitched)
 output  [1:0] encoder_new;  // encoder data to RAM (deglitched)
 output        pre_incdec;   // will increment/decrement position counter
 output        inc;          // increment position counter
 output        dec;          // decrement position counter
 output        inc_dec;      // increment or decrement position counter, registered

 wire          en=(IGNORE_EN)?1'b1:en_in; // will be overwritten, so en will be used in simulation only, ignored in synthesis

 wire    [4:0] deglitch_new;
 wire    [1:0] encoder_new;
 reg           inc;          // increment position counter
 reg           dec;          // decrement position counter
 reg           inc_dec;       // increment or decrement position counter, registered

 reg    [3:0] cycle_cntr;
 reg          deglitch_next;
 wire         pre_incdec;

// assign   deglitch_new=(!en || (encoder[1:0]==encoder_used[1:0]))?0:((deglitch_cur==5'h1f)?5'h1f:(deglitch_cur+1));
 assign   deglitch_new=(!en || (encoder[1:0]==encoder_used[1:0]))?0:((deglitch_cur==5'h1f)?5'h1f:(deglitch_cur+deglitch_next));
 assign   encoder_new= (!en || (deglitch_cur==5'h1f))?encoder:encoder_used;
 assign   pre_incdec=en && (encoder_new[1:0]!=encoder_used[1:0]);

 always @ (posedge clk) if (pre_first) begin // change these registers once per cycle of all motors
   cycle_cntr=    (!en || (cycle_cntr==0))?cycle_div:(cycle_cntr-1);
   deglitch_next=   en && (cycle_cntr==0);
 end

 always @ (posedge clk) if (process) begin
   inc_dec <=  pre_incdec;
   dec<=en && ((encoder_new==2'b00) && (encoder_used==2'b01) ||
               (encoder_new==2'b01) && (encoder_used==2'b11) ||
               (encoder_new==2'b11) && (encoder_used==2'b10) ||
               (encoder_new==2'b10) && (encoder_used==2'b00));
   inc<=en && ((encoder_new==2'b00) && (encoder_used==2'b10) ||
               (encoder_new==2'b10) && (encoder_used==2'b11) ||
               (encoder_new==2'b11) && (encoder_used==2'b01) ||
               (encoder_new==2'b01) && (encoder_used==2'b00));
//   cycle_cntr<=    (!en || (cycle_cntr==0))?cycle_div:(cycle_cntr-1);
//   deglitch_next<=   en && (cycle_cntr==0);
 end



endmodule


module motor_pwm( clk,      // posedge, 80MHz
                  en,       // enable, 0 turns off motors and resets counterts
                  pre_first,// next whill be the first motor in a cycle (may use just spread bits)
                  pwm_delay,// [2:0] - turns off bridge during transition (i.e. 10->00->11->00->10 ..). Nominal 0.3usec - 2 (0x18) cycles (80/4/3=6.66MHz)
                  pwm_cycle,// [7:0] - pwm cycle duration Nominal 10usec or 67 (0x43) 6.66MHz cycles. Total PWM period will be 8 of these cycles
                  spread,   // [2:0] shift on phase between motors, adds this number to the current phase (use just 2 MSBs for 3 motors)
                  cur_pwm,  // [2:0] - PWM data stored in per-motor memory. [1]  - 'on', [0] direction, [2] - enable. That bit is used to turn off motor during PWM transitions
                  pwm_code, // [3:0] - code from the RAM. 0x8 - stop,  , otherwise [3] - direction, [2:0]/8 - duty cycle (8 - 100% on), 0 - 0%, 1 - 25%, 3 - 37.5%...
                  new_pwm,  // [2:0]
                  mot      // [1:0] - data to be copied to the motor outputs
                );
 input         clk;      // posedge, 80MHz
 input         en;       // enable, 0 turns off motors and resets counterts
 input         pre_first;// next whill be the first motor in a cycle (may use just spread bits)
 input   [2:0] pwm_delay;// [2:0] - turns off bridge during transition (i.e. 10->00->11->00->10 ..). Nominal 0.3usec - 24 (0x18) cycles
 input   [7:0] pwm_cycle;// [7:0] - pwm cycle duration Nominal 10usec oror 67 (0x43) 6.66MHz cycles. Total PWM period will be 8 of these cycles
 input   [2:0] spread;   // [2:0] shift on phase between motors, adds this number to the current phase (use just 2 MSBs for 3 motors)
 input   [2:0] cur_pwm;  // [2:0] - PWM data stored in per-motor memory. [1:0] - motors, [2] - enable. That bit is used to turn off motor during PWM transitions
 input   [3:0] pwm_code; // [3:0] - code from the RAM. 0x8 - stop  , otherwise [3] - direction, [2:0]/7 - duty cycle (7 - 100% on)
 output  [2:0] new_pwm;  // [2:0]
 output  [1:0] mot;      // [1:0] - data to be copied to the motor outputs

 reg     [9:0] pwm_cycle_count;
 reg           pwm_cycle_next;
 reg     [3:0] pwm_delay_count;
 reg           pwm_delay_end;
 reg     [2:0] pwm_phase;
 wire    [2:0] spreaded_phase;
 wire          pwn_on;
 wire          pwn_off;
 wire    [2:0] new_pwm;
 wire          stop_cmd;
 wire    [3:0] pwm_diff;
 wire [1:0] mot;

 assign spreaded_phase[2:0]= pwm_phase[2:0]+spread[2:0];
 assign pwn_on= pwm_cycle_next && (spreaded_phase == 0) && (pwm_code != 0);
 assign pwn_off=pwm_cycle_next && (spreaded_phase == pwm_code);
 assign stop_cmd= !en || (pwm_code==8) ;
 assign pwm_diff[3:0]={1'b0,pwm_code[2:0]}-{1'b0,spreaded_phase[2:0]}; 

 assign new_pwm[0]=!stop_cmd && (pwm_cycle_next? pwm_code[3] : cur_pwm[0]); // direction
 assign new_pwm[1]=!stop_cmd && (pwm_cycle_next?((pwm_code[2:0]!=0) && !pwm_diff[3]):cur_pwm[1]); // turn off (by en) immediately, others - only at pwm_cycle_next;
 assign new_pwm[2]=!en || (pwm_cycle_next? (new_pwm[1:0]!=cur_pwm[1:0]):(cur_pwm[2]&& !pwm_delay_end));
 assign mot[1:0]=(stop_cmd || new_pwm[2])? 2'b0:(new_pwm[1]?{new_pwm[0],!new_pwm[0]}:2'b11);
 always @ (posedge clk) if (pre_first) begin // change these registers once per cycle of all motors
   pwm_cycle_count<= (!en || (pwm_cycle_count==0))?pwm_cycle:(pwm_cycle_count-1);
   pwm_cycle_next<=    en && (pwm_cycle_count==0);
//   pwm_delay_count<= (!en || (pwm_delay_count==0))?0:(pwm_cycle_next?pwm_delay: (pwm_delay_count-1));
   pwm_delay_count<= (!en || pwm_cycle_next)?pwm_delay:((pwm_delay_count==0)?0: (pwm_delay_count-1));
   pwm_delay_end<=     en && (pwm_delay_count==1);
   pwm_phase<=       (!en)? 0: (pwm_phase + pwm_cycle_next); // divide by 8
 end

endmodule

