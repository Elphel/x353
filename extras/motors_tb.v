`timescale 1ns/1ps
/*
 *  It is just a placeholder for a real testbench. Made from 333, not everything updated
 */
module testbench();
   parameter CLK_PER = 6.25;//	160MHz
   parameter CLK2WA= 1;
   parameter WA2WD= 1;

///AF:     parameter aaaa=0;
///AF:     parameter aaaa=1;

    reg         sclk,xclk;
    reg  [15:0] wd;
    reg         wa;
    reg         we;
    wire [31:0] rd; // SuppressThisWarning Veditor UNUSED

    wire [1:0] encod1;
    wire [1:0] encod2;
    wire [1:0] encod3;

    wire [1:0] mot1;
    wire [1:0] mot2;
    wire [1:0] mot3;
    reg        sim_mot_en;


three_motor_driver i_three_motor_driver(
                            .clk(sclk),   // system clock, negedge

                            .xclk(xclk),  // half frequency (80 MHz nominal)

                            .we(we),    // write enable (lower 16 bits, high - next cycle)

                            .wa(wa),    // write address(1)/data(0)

                            .di(wd),    // 16-bit data in (32 multiplexed)

                            .do(rd),    // 16-bit data output

                            .encod1(encod1[1:0]), // 2-bit encoder data input, motor1

                            .encod2(encod2[1:0]), // 2-bit encoder data input, motor2

                            .encod3(encod3[1:0]), // 2-bit encoder data input, motor3

                            .mot1(mot1[1:0]),   // 2 bits motor1 control output (11 - shorted, 00 - stop)

                            .mot2(mot2[1:0]),   // 2 bits motor1 control output (11 - shorted, 00 - stop)

                            .mot3(mot3[1:0])   // 2 bits motor1 control output (11 - shorted, 00 - stop)

                           );

motor i_motor1 (.clk(xclk),
                .en(sim_mot_en),
                .pwr(mot1[1:0]),
                .enc(encod1[1:0]));
motor i_motor2 (.clk(xclk),
                .en(sim_mot_en),
                .pwr(mot2[1:0]),
                .enc(encod2[1:0]));
motor i_motor3 (.clk(xclk),
                .en(sim_mot_en),
                .pwr(mot3[1:0]),
                .enc(encod3[1:0]));

    initial begin

      $dumpfile("motors.lxt");
      $dumpvars(0,testbench.i_three_motor_driver);  // SuppressThisWarning Veditor VDT_BUG
/*
      $dumpvars(0,testbench.i_three_motor_driver.addr);
      $dumpvars(0,testbench.i_three_motor_driver.reg_addr);
      $dumpvars(0,testbench.i_three_motor_driver.inc);
      $dumpvars(0,testbench.i_three_motor_driver.dec);
      $dumpvars(0,testbench.i_three_motor_driver.inc_dec);


      $dumpvars(0,testbench.i_three_motor_driver.minus_diff);

      $dumpvars(0,testbench.i_three_motor_driver.position_err);
      $dumpvars(0,testbench.i_three_motor_driver.dir_this);
      $dumpvars(0,testbench.i_three_motor_driver.enc_period_this);

      $dumpvars(0,testbench.i_three_motor_driver.pwm_code);



      $dumpvars(0,testbench.i_three_motor_driver.sequence);
      $dumpvars(0,testbench.i_three_motor_driver.position_diff);
      $dumpvars(0,testbench.i_three_motor_driver.i_motor_pwm);
*/
      $dumpvars(0,testbench.rd);
      $dumpvars(0,testbench.i_motor1); // SuppressThisWarning Veditor VDT_BUG - it is resolved
      $dumpvars(0,testbench.i_motor2.position);
      $dumpvars(0,testbench.i_motor2.speed);
      $dumpvars(0,testbench.i_motor3.position);
      $dumpvars(0,testbench.i_motor3.speed);

      $dumpvars(0,testbench.encod1);
      $dumpvars(0,testbench.encod2);
      $dumpvars(0,testbench.encod3);




      sclk<=0;
      xclk<=0;
//      encod1 <= 0;
//      encod2 <= 0;
//      encod3 <= 0;
      sim_mot_en <=0;

      wait (~glbl.GSR);

//      sclk<=1;
      $display ("reset done at %t",$time);

      #10;

      write_ad (1,4);
      write_ad (0,6);
      write_ad (0,'h7243);  ///pwm_cycle=0x43, pwm_delay=2, deglitch_div=7
      write_ad (0,'h457);  ///enc_period_cycle=0x457
      write_ad (0,'h2);  ///decrement_on_pulse

      write_ad (1,1); // target position
      write_ad (0,0); // target position1
      write_ad (0,0); // target position2
      write_ad (0,0); // target position3

      program_table;

      write_ad (1,4);
      write_ad (0,7); // reset, then enable motors
      #1000; // wait 1 usec
      write_ad (1,1);
      write_ad (0,32'hffffffce); // write motor1 target - -50
      write_ad (0,10); // write motor2 target - +10
      write_ad (0,1); // write motor3 target - +1
      write_ad (1,1); // point data out to motor1 position
      sim_mot_en <= 1'b1;
      #10000000;
$finish;      
      #60000000;
      write_ad (1,1); // target position
      write_ad (0,0); // target position1
      write_ad (0,0); // target position2
      write_ad (0,0); // target position3
      write_ad (1,4);
      write_ad (0,2); // disable motors
      #1000000;

//    #21000000;

    $finish;
    end

//  always #(CLK_PER/2) if (~glbl.GSR) sclk <=   ~sclk;

  always #(CLK_PER/2) sclk <=   ~sclk;


  always @ (posedge sclk) xclk <=   ~xclk;
  


  task write_ad;
    input        a;
    input [31:0] d;
    begin
      wait(sclk);wait(~sclk);
      #CLK2WA;
      wa=a;
      we=1'b1;
      #WA2WD;
      wd=d[15:0];
      wait(sclk);wait(~sclk);
      #CLK2WA;
      wa=1'bx;
      we=1'b0;
      #WA2WD;
      wd=d[31:16];
      wait(sclk);wait(~sclk);
      #CLK2WA;
      #WA2WD;
      wd=16'bx;
    end
  endtask

  task program_table;

    reg [31:0] data[0:511]; // SuppressThisWarning Veditor VDT_BUG - assigned in system task
    integer i;
    begin
      $readmemh("motor.dat",data);
      write_ad (1,512);
      for (i=0;i<512;i=i+1) begin
        write_ad (0,data[i]);
      end
    end
  endtask




endmodule

module motor (clk,
              en,
              pwr,
              enc);
  parameter SAMPLE_PERIOD=100; // ns
  parameter VMAX=1000.0; /// pulses/sec
  parameter EMF= 0.5;    /// part of the voltage that is caused by rotation (remaining goes to current -> force->acceleration). Not yet used // SuppressThisWarning Veditor UNUSED
//  parameter ACCEL=10.0;  /// number of VMAX/sec if full power is applied, speed==0
  parameter ACCEL=300.0;  /// number of VMAX/sec if full power is applied, speed==0
// no simulation of friction yet
  input       clk; // SuppressThisWarning Veditor UNUSED
  input       en;  // SuppressThisWarning Veditor UNUSED
  input  [1:0] pwr;
  output [1:0] enc;

  reg     mclk;
  real    position, position0;
  real    speed, speed0;
//  time    t,t0;
///AF:    integer itime;
///AF:    real    rtime;
  reg [1:0] enc;
  reg [1:0] enc_bin;
  real    t,t0, dt, e,f;
  
  initial begin
    position0=0;
    speed0=0;
    t0=$time;
    mclk=0;
  end
  always #(SAMPLE_PERIOD/2) mclk <=   ~mclk;


//  always @ (posedge clk) begin
  always @ (posedge mclk) begin
   t=$time;
   dt=(t-t0)/1000000000; // in seconds
   case (pwr)
   0: f = (speed>0)?-1.0:1.0;
   1: f = 1.0;
   2: f = -1.0;
   3: f = 0.0;
   endcase
   e=f-speed/VMAX;
   speed=speed0+ACCEL*VMAX*e*dt;
   position=position0+(speed+speed0)*dt/2;
   enc_bin[1:0]=position;
   #1;
//   itime=sim_time;
//   rtime=sim_time;
   enc[1:0] = {enc_bin[1],enc_bin[1] ^ enc_bin[0]};
   t0=t;
   speed0=speed;
   position0=position;
  end
 
endmodule
