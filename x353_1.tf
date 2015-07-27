`timescale 1ns/1ps


//`define PF
//compressor waits for sensor
`define TEST_ABORT
`define SYNC_COMPRESS
//TODO: when TEST_INSUFFICIENT_DATA, i_color_proc does not generate last0, because all_ready==0 when bcntr==0,
// i.e. override ignore/ready from noMoreData until last0

//`define TEST_INSUFFICIENT_DATA
`define TRIGGERED_MODE
`define CONTINUOUS_COMPRESSION
//`define TOO_HIGH_FPS
`define ALL_SLOW_FPS
`define TEST_NO_WAIT_FRAME_SYNC
//`define TRY_SLOW_FPS
/// Enable flushing unfinished frame if there is no data in the memory (end of frame)
`define ENDFRAMES
/// Enable reset memory controllers (channles 0,1,2) at each frame
`define RESET_MCONTR
//`define LATE_DMA
`define TEST_EXTERNAL_SYNC
`define TEST_OUTPUT_TIMESTAMP // send timestamop out
//`define FORCE_INTERNAL_TIMESTAMP // always use internal TS, ignore incoming


`define TEST_BAD_FRAME //abbreviate one frame

///AF2015  `define TEST_IMU

module testbench353();
  reg [639:0] TEST_TITLE; //SuppressThisWarning Veditor Simulator wave

  parameter SYNC_BIT_LENGTH=8-1; /// 7 pixel clock pulses
  parameter FPGA_XTRA_CYCLES= 1500; // 1072+;
  parameter HISTOGRAM_LEFT=  0; //2;   // left   
  parameter HISTOGRAM_TOP =  2;   // top
  parameter HISTOGRAM_WIDTH= 6;  // width
  parameter HISTOGRAM_HEIGHT=6;  // height
  parameter CLK0_PER = 6.25;   //160MHz
  parameter CLK1_PER = 10.4;     //96MHz
  parameter CLK3_PER = 83.33;   //12MHz
  parameter CPU_PER=10.4;
`ifdef IVERILOG
    `define SIMULATION 1              
     initial   $display("IVERILOG is defined");
    `include "IVERILOG_INCLUDE.v"
`else
    initial $display("IVERILOG is not defined");
    parameter lxtname = "x353_1.lxt";
`endif

`ifdef SYNC_COMPRESS
  parameter DEPEND=1'b1;
`else  
  parameter DEPEND=1'b0;
`endif

`ifdef TEST_ABORT
`endif
  
 parameter HBLANK=           12; /// 52;
 parameter WOI_HEIGHT=       32;
 parameter WOI_WIDTH=        64;
 parameter FULL_WIDTH=       WOI_WIDTH+4;
 parameter BLANK_ROWS_BEFORE=1; //8; ///2+2 - a little faster than compressor
 parameter BLANK_ROWS_AFTER= 1; //8;
 parameter TRIG_LINES=       8;
 parameter VBLANK=           2; /// 2 lines //SuppressThisWarning Veditor UNUSED
 parameter CYCLES_PER_PIXEL= 3; /// 2 for JP4, 3 for JPEG

`ifdef PF
  parameter PF_HEIGHT=8;
  parameter FULL_HEIGHT=WOI_HEIGHT;
  parameter PF_STRIPES=WOI_HEIGHT/PF_HEIGHT;
`else  
  parameter PF_HEIGHT=0;
  parameter FULL_HEIGHT=WOI_HEIGHT+4;
  parameter PF_STRIPES=0;
`endif

 parameter VIRTUAL_WIDTH=    FULL_WIDTH+HBLANK;
 parameter VIRTUAL_HEIGHT=   FULL_HEIGHT+BLANK_ROWS_BEFORE+BLANK_ROWS_AFTER;  //SuppressThisWarning Veditor UNUSED
 
 parameter TRIG_INTERFRAME=100; /// extra 100 clock cycles between frames  //SuppressThisWarning Veditor UNUSED
// parameter TRIG_OUT_DATA=     'h800000; // external cable
 parameter TRIG_OUT_DATA=        'h80000; // internal cable
 parameter TRIG_EXTERNAL_INPUT=  'h20000; // internal cable, low level on EXT[8]

 parameter TRIG_DELAY=      200; /// delay in sensor clock cycles
 wire [23:0] FRAME_COMPRESS_CYCLES;
 wire [23:0] FRAME_COMPRESS_CYCLES_INPUT;
 wire [23:0] TRIG_PERIOD;
 assign FRAME_COMPRESS_CYCLES=(WOI_WIDTH &'h3fff0) * (WOI_HEIGHT &'h3fff0) * CYCLES_PER_PIXEL + FPGA_XTRA_CYCLES;
 assign FRAME_COMPRESS_CYCLES_INPUT=(FRAME_COMPRESS_CYCLES*CLK0_PER)/CLK1_PER;
`ifdef ALL_SLOW_FPS
     assign TRIG_PERIOD =  2* FRAME_COMPRESS_CYCLES_INPUT; /// twice slower than maximal compressor can do
`else
  `ifdef TOO_HIGH_FPS
     assign TRIG_PERIOD =   VIRTUAL_WIDTH * (VIRTUAL_HEIGHT + TRIG_LINES + VBLANK); /// maximal sensor can do
  `else
     assign TRIG_PERIOD =   FRAME_COMPRESS_CYCLES_INPUT; /// maximal compressor can do
//   parameter TRIG_PERIOD=     1.5*(VIRTUAL_WIDTH * (VIRTUAL_HEIGHT + TRIG_LINES + VBLANK)); ///TODO: Improve (calculate)
  `endif
`endif
 
 parameter X313_WA_CAMSYNCTRIG=   'h78; // trigger condition, 0 - internal, else dibits ((use<<1) | level) for each GPIO[11:0] pin
 parameter X313_WA_CAMSYNCDLY=    'h79; // trigger delay, 32 bits in pixel clocks
 parameter X313_WA_CAMSYNCOUT=    'h7a; // trigger output to GPIO, dibits ((use << 1) | level_when_active). Bit 24 - test mode, when GPIO[11:10] are controlled by other internal signals
 parameter X313_WA_CAMSYNCPER=    'h7b; // output sync period (32 bits, in pixel clocks). 0- stop. 1..256 - single, >=256 repetitive with specified period.
 parameter X313_WA_DCR0=          'h4e;
 parameter X313_WA_DCR1=          'h4f;

 parameter X313_WA_DCR0_TRIGEN=   'h180000;
 parameter X313_WA_DCR0_TRIGDIS=  'h100000;

 parameter X313_WA_DCR0_ENDFRAMESEN=  'h600000;
 parameter X313_WA_DCR0_ENDFRAMESDIS= 'h400000;  //SuppressThisWarning Veditor UNUSED

 parameter X353_WA_DCR0_RESET_MCONTREN=  'h60; //  1 - enable reset memory controllers (channles 0,1,2) at each frame
 parameter X353_WA_DCR0_RESET_MCONTRDIS=  'h40; // 0 - disable reset memory controllers (channles 0,1,2) at each frame  //SuppressThisWarning Veditor UNUSED

 parameter X313_WA_IOPINS=        'h70;    // bits [31:24] - enable channels (channel 0 -software, enabled at FPGA init)

 parameter X313_WA_IOPINS_EN_TRIG_OUT= 'h0c000000;
 parameter X313_WA_IOPINS_DIS_TRIG_OUT='h08000000;  //SuppressThisWarning Veditor UNUSED



`ifdef TEST_IMU
     parameter X313_WA_IOPINS_EN_IMU_OUT= 'hc0000000;
     parameter X313_WA_IOPINS_DIS_IMU_OUT='h80000000;  //SuppressThisWarning Veditor UNUSED
     parameter X313_WA_IMU_CTRL= 'h7f;
    
     parameter X313_WA_IMU_DATA= 'h7e;
     parameter X313_RA_IMU_DATA= 'h7e; // read fifo word, advance pointer (32 reads w/o ready check) 
     parameter X313_RA_IMU_STATUS= 'h7f; // LSB==ready
     parameter IMU_PERIOD= 'h800; // normal period
     parameter IMU_AUTO_PERIOD= 'hffff0000; // period defined by IMU ready
 
     parameter IMU_BIT_DURATION= 'h3; // actual F(scl) will be F(xclk)/2/(IMU_BIT_DURATION+1)

     parameter IMU_READY_PERIOD=100000; //100usec
     parameter IMU_NREADY_DURATION=10000; //10usec
     parameter IMU_GPS_BIT_PERIOD='h20; // serial communication duration of a bit (in system clocks)
// use start of trigger as a timestamp (in async mode to prevent timestamp jitter)
// parameter X313_WA_DCR1_EARLYTRIGEN='hc; //OBSOLETE!
// parameter X313_WA_DCR1_EARLYTRIGDIS='h8;
`endif
 parameter X313_WA_DCR1_EXTERNALTSEN='hc;
 parameter X313_WA_DCR1_EXTERNALTSDIS='h8;  //SuppressThisWarning Veditor UNUSED

 parameter X313_WA_DCR1_OUTPUTTSEN= 'h300000;
 parameter X313_WA_DCR1_OUTPUTTSDIS='h200000;  //SuppressThisWarning Veditor UNUSED



//#define  X353DCR0__ENDFRAMES__BITNM  21
//#define  X353DCR0__ENDFRAMES__WIDTH   1

/// channel 3 has highest priority, 0 - lowest. If channel wants this pin as output, the (enabled) channel with highest number will get it
/// bits 31,30 control channel 3 (0x00000000, 0x40000000 - no change, 0x80000000 - disable, 0xc0000000 - enable)
/// bits 29,28 control channel 2 (0x00000000, 0x10000000 - no change, 0x20000000 - disable, 0x30000000 - enable) - 10364 board
/// bits 27,26 control channel 1 (0x00000000, 0x04000000 - no change, 0x08000000 - disable, 0x0c000000 - enable) - 10369 board

`ifdef TRIGGERED_MODE
  parameter TRIG_MODE_CTL=X313_WA_DCR0_TRIGEN;
`else
  parameter TRIG_MODE_CTL=X313_WA_DCR0_TRIGDIS;
`endif

`ifdef TEST_EXTERNAL_SYNC
  parameter TRIG_CODITION=TRIG_EXTERNAL_INPUT;
`else
  parameter TRIG_CODITION=0;
`endif

`ifdef TEST_OUTPUT_TIMESTAMP // send timestamop out
  parameter OUTPUTTS=   X313_WA_DCR1_OUTPUTTSEN;
`else
  parameter OUTPUTTS=    X313_WA_DCR1_OUTPUTTSEN;
`endif

`ifdef FORCE_INTERNAL_TIMESTAMP // send timestamop out
  parameter EXTERNALTS=    X313_WA_DCR1_EXTERNALTSDIS;
`else
  parameter EXTERNALTS=    X313_WA_DCR1_EXTERNALTSEN;
`endif



/*
  parameter SYNC_BIT_LENGTH=8-1; /// 7 pixel clock pulses

`define TEST_EXTERNAL_SYNC
`define TEST_EXTERNAL_TIMESTAMP // send timestamop out
`define FORCE_INTERNAL_TIMESTAMP // always use internal TS, ignore incoming

 parameter TRIG_OUT_DATA=         'h80000; // internal cable
 parameter TRIG_EXTERNAL_INPUT=   'h40000; // internal cable, low level on EXT[8]


 parameter X313_WA_DCR1_EXTERNALTSEN='hc;
 parameter X313_WA_DCR1_EXTERNALTSDIS='h8;

 parameter X313_WA_DCR1_OUTPUTTSEN= 'h300000;
 parameter X313_WA_DCR1_OUTPUTTSDIS='h200000;


`define TEST_EXTERNAL_SYNC
`define TEST_EXTERNAL_TIMESTAMP

`define TEST_BAD_FRAME //abbreviate one frame
module testbench353();
  parameter SYNC_BIT_LENGTH=8-1; /// 7 pixel clock pulses
#define  X353DCR1__OUTPUTTS__BITNM 20 // output timestamp, not just pulse
#define  X353DCR1__OUTPUTTS__WIDTH  1
#define  X353DCR1__EXTERNALTS__BITNM   2 // use external timestamp if available
#define  X353DCR1__EXTERNALTS__WIDTH   1

*/


//   parameter CLK0_PER = 6.25;   //160MHz
//   parameter CLK1_PER = 25;   //40MHz
//   parameter CLK1_PER = 10.4;     //96MHz
//   parameter CLK3_PER = 83.33;   //12MHz
//   parameter CPU_PER=10.4;
   parameter DMA_BURST=8;  //SuppressThisWarning Veditor UNUSED

///AF:  reg TEST_CPU_WR_OK;
///AF:  reg TEST_CPU_RD_OK;
reg SERIAL_BIT = 1'b1;   // SuppressThisWarning Veditor UNUSED - simulator test WAVE
reg GPS1SEC    = 1'b0;   // SuppressThisWarning Veditor UNUSED - simulator test WAVE
reg ODOMETER_PULSE= 1'b0;// SuppressThisWarning Veditor UNUSED - simulator test WAVE
integer SERIAL_DATA_FD;  // SuppressThisWarning Veditor UNUSED - simulator test WAVE
reg IMU_DATA_READY;      // SuppressThisWarning Veditor UNUSED - simulator test WAVE
/*
 parameter IMU_READY_PERIOD=100000; //100usec
 parameter IMU_NREADY_DURATION=10000; //10usec

*/
wire [11:0] EXT; // bidirectional

`ifdef TEST_IMU
    `include "imu_sim_init_include.vh"
`endif
// Inputs
    wire [11:0] PXD;
    wire BPF;
    wire HACT;
    wire VACT;
    wire VACT1CYCLE; //SuppressThisWarning Veditor UNUSED
    reg TTRIG;
    reg CLK3;
    reg CLK2; //SuppressThisWarning Veditor UNUSED
    reg CLK1;
    reg CLK0;
  //  reg SDCLK_DLL;
    reg  [12:0] A;
    wire [12:0] SYSTEM_A;
    reg WE;
    reg OE;
    reg CE;
    reg CE1;
    reg DACK;
    reg DACK1;
//
    reg   [31:0] CPU_DO;
    reg          CPU_OE;   // enable data from CPU to D[31:0]
    reg   [31:0] CPU_DI;
    reg   [31:0] DMA_DI;   //SuppressThisWarning Veditor UNUSED
    reg   [31:0] DMA_DI_1; //SuppressThisWarning Veditor UNUSED
///AF:      reg  [31:0] DMA_CNTR;
///AF:      reg  [31:0] DMA_CNTR_1;
    reg   [11:0]   SDRAM_MODE;   // shadow register for SDRAM controller modes
 // Outputs
    wire DCLK;
    wire MRST;
    wire ARO;
    wire ARST;
///AF:      wire CNVSYNC;
///AF:      wire CNVCLK;
///AF:      wire XRST;
///AF:      wire AUXCLK;
    wire [14:0] SDA;
    wire SDCLK, SDNCLK;
//    wire SDCKE;
    wire SDWE;
    wire SDCAS;
    wire SDRAS;
//    wire SDCS;
    wire SDLDM;
    wire SDUDM;
    wire DREQ;
    wire DREQ1;
    wire IRQ;

//   wire SDCLK_DLL;
    wire SDCLK_FB;
    wire SDNCLK_FB;

// Bidirs
    wire SCL0;
///AF:      wire SCL1;
    wire SDA0;
///AF:      wire SDA1;
///AF:      wire EXPS;
    wire TRIG; //SuppressThisWarning Veditor UNUSED
    wire [15:0] SDD;
    wire [31:0] D;
    wire LDQS;
    wire UDQS;

// CPU data:
   reg CPU_CLK;
// changing to vector CPU bus cycles (1 level of interrupts, I/O cycles just duplicated for normal/from ISR:
parameter BUSOP_DMA_0=  0;
parameter BUSOP_DMA_1=  1;
parameter BUSOP_ISR_WR= 2;
parameter BUSOP_ISR_RD= 3;
parameter BUSOP_ISR_RD1=4;
parameter BUSOP_IO_WR=  5;
parameter BUSOP_IO_RD=  6;
parameter BUSOP_IO_RD1= 7;
   reg [7:0] BUS_EN; // Enable state of for the different bus activity types
   reg [7:0] BUS_RQ; // Vector of bus operations requests
   reg [7:0] BUS;    // Vector of currently active bus operations


   reg CPU_IO;   // CPU IO in progress, may not start IO/DMA //SuppressThisWarning Veditor UNUSED
//   integer LOCK;
///AF:     reg  [10:0] LOCK_ADDR; // LSB - address, 3 MSBs: 0 - read, 1 - write, 2 - DMA0, 3 - DMA1, 4 - INTA
///AF:     reg [31:0] LOCK_DATA; // Data wants to write
//   reg DMA_EN;
//   reg DMA_EN_1;
   assign D[31:0]=CPU_OE? CPU_DO[31:0]: 32'bz;

//   assign #(1) SDCLK_DLL=SDCLK;   // function simulation - hold time violation for sdram. Make it's clock faster?
   assign #(1) SDCLK_FB=SDCLK;
   assign #(1) SDNCLK_FB=SDNCLK;

   integer histogram_total;
   reg [9:0] histogram_count;

   reg [23:0] IMG_POINTER; //SuppressThisWarning Veditor UNUSED
   reg [1:0]  FOCUS_MODE;
   reg        BLOCK_HACT=0;

assign TRIG=TTRIG; 

   reg [2:0] I2C_FRAME; // frame number modulo 8 as seen in i2c_writeonly   //SuppressThisWarning Veditor UNUSED
// ************************* Instantiate the X353 ****************************
assign SYSTEM_A[12:0]={5'b0,A[7:0]}; // will make it tri-state when testing bus acquisition
wire SENSPGM,DUMMYVFEF,ALWAYS0;
wire SDCLKE;
wire [1:0] BA; // don't need now //SuppressThisWarning Veditor UNUSED
wire       SYS_SDWE, SYS_SDCAS, SYS_SDRAS, SYS_SDCLK, SYS_BUSEN, BG, BROUT; //SuppressThisWarning Veditor UNUSED
wire       SYS_SDCLKI = 1'bx; // not tested
wire       BRIN  =      1'bx; // not tested

/// connect external sync
assign EXT[8]=EXT[9];

wire  external_sync_line=~EXT[9]; //SuppressThisWarning Veditor UNUSED
x353 i_x353 (
             .PXD(PXD[11:2]), 
             .DCLK(DCLK),
             .BPF(BPF), 
             .VACT(VACT), 
             .HACT(HACT && ! BLOCK_HACT),
             .MRST(MRST),
             .ARO(ARO),
             .ARST(ARST),
             .SCL0(SCL0), 
             .SDA0(SDA0), 
             .CNVSYNC(PXD[0]), 
             .CNVCLK(PXD[1]), 
             .SENSPGM(SENSPGM),

             .DUMMYVFEF(DUMMYVFEF),
             .ALWAYS0(ALWAYS0),

             .EXT(EXT[11:0]),

             .CLK3(CLK3), 
             .CLK2(),
             .CLK4(),
             .CLK1(CLK1), 
             .CLK0(CLK0), 

             .UDQS(UDQS),
             .LDQS(LDQS),
             .SDD(SDD), 
             .SDA(SDA),
             .SDWE(SDWE), 
             .SDCAS(SDCAS), 
             .SDRAS(SDRAS), 
             .SDUDM(SDUDM),
             .SDLDM(SDLDM),
             .SDCLK_FB(SDCLK_FB),
             .SDCLK(SDCLK),
             .SDNCLK(SDNCLK),
             .SDNCLK_FB(SDNCLK_FB),
             .SDCLKE(SDCLKE),

             .D(D),
             .A(SYSTEM_A[12:0]),
             .BA(BA[1:0]),
             .SYS_SDWE(SYS_SDWE),
             .SYS_SDCAS(SYS_SDCAS),
             .SYS_SDRAS(SYS_SDRAS),
             .SYS_SDCLKI(SYS_SDCLKI),
             .SYS_SDCLK(SYS_SDCLK),
             .SYS_BUSEN(SYS_BUSEN),

             .WE(WE),
             .OE(OE),
             .CE(CE),
             .CE1(CE1),
             .DREQ0(DREQ),
             .DACK0(DACK),
             .DREQ1(DREQ1),
             .DACK1(DACK1),
             .IRQ(IRQ),
             .BG(BG),
             .BRIN(BRIN),
             .BROUT(BROUT)
             );
    

// Instance of Micron MT48LC8M16LFFF8
// cheating - no such actual signal :-(
reg SDCKE; //SuppressThisWarning Veditor UNUSED
initial begin
  SDCKE=0;
  #1000;
  SDCKE=1;
end
wire SDCLK_D;
wire SDNCLK_D;
assign #(2) SDCLK_D=SDCLK;
assign #(2) SDNCLK_D=SDNCLK;

ddr          i_mt46v16m16fg (.Dq(SDD[15:0]),
                             .Dqs({UDQS,LDQS}),
                             .Addr(SDA[12:0]),
                             .Ba(SDA[14:13]),
                             .Clk(SDCLK_D),
                             .Clk_n(SDNCLK_D),
//                             .Cke(1'b1),
//                             .Cke(SDCKE),
                             .Cke(SDCLKE),
                             .Cs_n(1'b0),
                             .Ras_n(SDRAS),
                             .Cas_n(SDCAS),
                             .We_n(SDWE),
                             .Dm({SDUDM,SDLDM}));

// sensor 12 bits chip
sensor12bits i_sensor12bits(.MCLK(DCLK),   // Master clock
                  .MRST(MRST),   // Master Reset - active low
                  .ARO(ARO),   // Array read Out.
                  .ARST(ARST),   // Array Reset. Active low
                  .OE(1'b0),   // output enable active low
                  .SCL(SCL0),   // I2C clock
                  .SDA(SDA0),   // I2C data
                  .OFST(1'b1),   // I2C address ofset by 2: for simulation 0 - still mode, 1 - video mode.
                  .D(PXD[11:0]),      // [9:0] data output
                  .DCLK(BPF),   // Data output clock
                  .BPF(),   // Black Pixel Flag
                  .HACT(HACT),   // Horizontal Active
                  .VACT(VACT),
                  .VACT1(VACT1CYCLE) // output 
                  
               );// Vertical Active

// testing end of SDRAM page - process 17 tiles starting at the 3-rd 128-words ina 512 words page
defparam i_sensor12bits.lline   =   VIRTUAL_WIDTH;

// +++++++++++ Normal mode (not photofinish+timestamping) ++++++++++
defparam i_sensor12bits.ncols   =   FULL_WIDTH; //132; //130; //258;//50; //40; //22; //58; //56; // 129; //128;   //1288;
// +++++++++++ photofinish mode ++++++++++
//defparam i_sensor12bits.ncols   =   104; //132-28; 132; //130; //258;//50; //40; //22; //58; //56; // 129; //128;   //1288;



//defparam i_sensor12bits.nrows   =   36; //34; //18; //34; //32;   //   1032;
defparam i_sensor12bits.nbpf   =      0; //4;//20;   //16; // bpf length (now if 0 will skip both BPF and pause between BPF and HACT)
defparam i_sensor12bits.ramp   =      0; //1;   // 1 - ramp, 0 - random
//defparam i_sensor12bits.ramp   =      1;   // 1 - ramp, 0 - random
defparam i_sensor12bits.new_bayer= 1; // 0 - old (16x16), 1 - new (18x18)
defparam i_sensor12bits.nrowb = BLANK_ROWS_BEFORE;   // number of "blank rows" from vact to 1-st hact
defparam i_sensor12bits.nrowa = BLANK_ROWS_AFTER;   // number of "blank rows" from last hact to end of vact
defparam i_sensor12bits.trigdly = TRIG_LINES;   // delay between trigger input and start of output (VACT) in lines

// +++++++++++ Normal mode ++++++++++
`ifdef PF
  defparam i_sensor12bits.nrows   = PF_HEIGHT;
`else
  defparam i_sensor12bits.nrows   =   FULL_HEIGHT;
`endif
/*
 parameter BLANK_ROWS_BEFORE=8;
 parameter BLANK_ROWS_AFTER= 8;
 parameter TRIG_LINES=       8;

*/

// Initialize Inputs


        initial begin
//    $dumpfile("x353.lxt");
    $dumpfile(lxtname);
    $dumpvars(0,testbench353); //testbench353 cannot be resolved to a signal or parameter //SuppressThisWarning Veditor
            TTRIG = 1;
            CLK3 = 0;
            CLK2 = 0;
            CLK1 = 0;
            CLK0 = 0;
            A = 13'bx;
            WE = 1'b1;
            OE = 1'b1;
            CE = 1'b1;
            CE1 = 1'b1;
            DACK = 0;
            DACK1 = 0;
             CPU_DO = 32'b0;
             CPU_OE = 1'b0;
            CPU_DI = 32'bx;
            SDRAM_MODE= 0;
            CPU_CLK=1'b0;
            CPU_IO=1'b0;
            dma_en(0,0);
            dma_en(1,0);
            BUS[7:0]    =8'h0;
            BUS_EN[7:0] =8'h0;
            BUS_RQ[7:0] =8'h0;
            FOCUS_MODE = 2'h0;
`ifdef TEST_IMU
            IMU_103695REVA = 1'b0;
`endif
            
`ifdef LATE_DMA
`else
     dma_en(0,1);
`endif
// temporary for IMU testing
//   #200000;
//   $finish;            
`ifdef TEST_IMU
            `include "imu_sim_include.vh"
`endif
    #200000;
    TEST_TITLE = "FIRST_INIT_DONE";
    $display("===================== TEST_%s ========================= @%t",TEST_TITLE,$time);
    $finish;
end


// Second async test for IMU
`ifdef TEST_IMU
  initial begin
    #10000;
    while (!$feof (SERIAL_DATA_FD)) begin
      repeat (20*IMU_BIT_DURATION) begin  wait (CLK0); wait (~CLK0);  end
      send_serial_line;
      send_serial_bit('h0a);
      GPS1SEC=1'b1;
      send_serial_line;
      send_serial_bit('h0a);
      GPS1SEC=1'b0;
      send_serial_line;
      send_serial_bit('h0a);
      send_serial_pause;
      send_serial_pause;
      ODOMETER_PULSE=1'b1;
      send_serial_pause;
      ODOMETER_PULSE=1'b0;
//  repeat (20) send_serial_pause;
    end
  end 
`endif


initial begin
   #250000;
   wait (~VACT);
   wait (VACT);
   cpu_wr('h4e,    'h600);   // Switch to sensor clock (0x400 - switch to internal)
   cpu_wr('h08,    'h7f0);   //  DCM: reset sensor_phase and pclk2x
end
initial begin
  #31500;
  TTRIG=0;
  #1000;
  TTRIG=1;

  #5000;
  TTRIG=0;
  #1000;
  TTRIG=1;
    TEST_TITLE = "EXT_TRIG_DONE";
    $display("===================== TEST_%s ========================= @%t",TEST_TITLE,$time);
  
end

  always #(CLK0_PER/2) CLK0 =   ~CLK0;
  always #(CLK1_PER/2) CLK1 =   ~CLK1;
  always #(CLK3_PER/2) CLK3 =   ~CLK3;
  always #(CPU_PER/2)  CPU_CLK = ~CPU_CLK;

// CPU bus operations arbiter
// BUS[] bits will be released by individual tasks
  always @ (posedge CPU_CLK) if (BUS[7:0]==8'h0) begin
   if      (BUS_EN[0] & BUS_RQ[0]) BUS[0]=1'b1;
   else if (BUS_EN[1] & BUS_RQ[1]) BUS[1]=1'b1;
   else if (BUS_EN[2] & BUS_RQ[2]) BUS[2]=1'b1;
   else if (BUS_EN[3] & BUS_RQ[3]) BUS[3]=1'b1;
   else if (BUS_EN[4] & BUS_RQ[4]) BUS[4]=1'b1;
   else if (BUS_EN[5] & BUS_RQ[5]) BUS[5]=1'b1;
   else if (BUS_EN[6] & BUS_RQ[6]) BUS[6]=1'b1;
   else if (BUS_EN[7] & BUS_RQ[7]) BUS[7]=1'b1;
  end

   initial forever begin
    dma_rd(0,8);
   end

   initial forever begin
//    dma_rd_1(1,1); does not work - may fix later, but it is not used in the actual CPU
    dma_rd_1(1,8);
   end

  always @ (negedge IRQ) begin
// disable "normal" wr/rd
      BUS_EN[BUSOP_IO_WR]   = 0;
      BUS_EN[BUSOP_IO_RD]   = 0;
      BUS_EN[BUSOP_IO_RD1]  = 0;
//     dma_en(0,0); // not needed anymore?
// we need to make sure that previous instance of cpu_wr_int (if any) exited (non reenterable tasks).
// For the next requests it is not needed as those ops are disabled and will not start again until enabled
      wait (~(BUS[BUSOP_IO_WR] | BUS[BUSOP_IO_RD]  | BUS[BUSOP_IO_RD1]) );
//      BUS_RQ[BUSOP_ISR_WR] = 1;
//      wait (BUS[BUSOP_ISR_WR]);
      cpu_wr_isr ('h1c,'hffff); // reset all interrupts

      cpu_rd_isr ('h14); // image pointer
      IMG_POINTER[23:0] = CPU_DI[23:0];
      cpu_rd_isr ('h16)   ;
      I2C_FRAME[2:0]=      CPU_DI[2:0];
      
      # 500; // latency
//      cpu_wr_isr ('hc,{FOCUS_MODE[1:0],13'hff}); // just continue - nothing but interrupt is reset
//      cpu_wr_isr ('hc,{FOCUS_MODE[1:0],13'h3ff}); // just continue - nothing but interrupt is reset (0x100 - second quant. table)
      FOCUS_MODE[1:0] =  FOCUS_MODE[1:0]+1;
//      dma_en(0,1);

//     # 1000; // latency
//     read256_ch3;
// re-enable normal wr/rd
      BUS_EN[BUSOP_IO_WR]   = 1;
      BUS_EN[BUSOP_IO_RD]   = 1;
      BUS_EN[BUSOP_IO_RD1]  = 1;
  end



`ifdef TEST_ABORT

  initial begin
    # (842000);
    cpu_wr(8'h0c, /// compressor cmd 
           4);    /// stop
  end
`endif

  initial begin
   # (398500);
   cpu_wr('h4e,'h18); // switch to internal HACT, see if it does not break line
   # (11500);
   cpu_wr('h4e,'h10); // switch to internal HACT, see if it does not break line
  end

//Simulation main sequence:
 `define TEST_CH3_AND_PHASE

`ifdef TEST_BAD_FRAME //abbreviate one frame
 initial begin
   repeat (9) begin  wait (VACT); wait(~VACT);  end
   wait (VACT);
   repeat (FULL_HEIGHT>>1) begin  wait (HACT); wait(~HACT);  end


 $display ("****** HACT is blocked to simulate BAD FRAME at %t",$time);

   BLOCK_HACT=1;
   wait (VACT);
 $display ("****** HACT is released (simulated BAD FRAME) at %t",$time);
   BLOCK_HACT=0;
 end
`endif

  initial begin
      wait (~glbl.GSR);
 $display ("                TRIG_PERIOD= %d",TRIG_PERIOD);
 $display ("      FRAME_COMPRESS_CYCLES= %d",FRAME_COMPRESS_CYCLES);
 $display ("FRAME_COMPRESS_CYCLES_INPUT= %d",FRAME_COMPRESS_CYCLES_INPUT);
 $display ("reset done at %t",$time);
    TEST_TITLE = "RESET_DONE";
    $display("===================== TEST_%s ========================= @%t",TEST_TITLE,$time);

#10;

      BUS_EN[BUSOP_ISR_WR]  = 1;
      BUS_EN[BUSOP_ISR_RD]  = 1;
      BUS_EN[BUSOP_ISR_RD1] = 1;
      BUS_EN[BUSOP_IO_WR]   = 1; // disable inside ISR
      BUS_EN[BUSOP_IO_RD]   = 1; // disable inside ISR
      BUS_EN[BUSOP_IO_RD1]  = 1; // disable inside ISR

// reset DCMs
//   cpu_wr('h1b,    'h00);   //  async reset, does not depend on clocks
//   cpu_wr('h08,    'hff);   //  DCM: both sensor and SDRAM

/// Switch to sensor clock
//   cpu_wr('h4e,    'h600);   // Switch to sensor clock (0x400 - switch to internal)


   cpu_wr('h08,    'h7ff);   //  DCM: both sensor and SDRAM - UPDATE: added reset for sensor_phase

// try I2C here
/*
   cpu_wr('ha,'h0);   // no frame sync delay
   cpu_wr('h5,'h8000); // no photofinish yet
   cpu_wr('h5,'h4000 | FULL_WIDTH); //number of pixelas in a line - 68 (for internal HACT)
*/

   cpu_wr(X313_WA_DCR0,        X313_WA_DCR0_TRIGDIS); // turn off to reset output

   cpu_wr(X313_WA_CAMSYNCPER, 'h0); ///reset circuitry
   cpu_wr(X313_WA_CAMSYNCTRIG,TRIG_CODITION);  /// use internal/external trigger
   cpu_wr(X313_WA_CAMSYNCDLY,  TRIG_DELAY);
   cpu_wr(X313_WA_CAMSYNCOUT,  TRIG_OUT_DATA);
//   cpu_wr(X313_WA_CAMSYNCPER,  TRIG_PERIOD); /// starts generatoe
   cpu_wr(X313_WA_DCR0,        TRIG_MODE_CTL); // turns on /remains off triggered mode (TRIG=4)
   cpu_wr(X313_WA_DCR1,        OUTPUTTS); // enables/disbles sending out timestamps with trigger pulse
   cpu_wr(X313_WA_DCR1,        EXTERNALTS); // use external (if available) or internal timestamps in frme data
   cpu_wr(X313_WA_CAMSYNCPER, SYNC_BIT_LENGTH); ///set (bit_length -1) (should be 2..255)

   cpu_wr(X313_WA_IOPINS,      X313_WA_IOPINS_EN_TRIG_OUT); // Enable GPIO output from camsync module
   
`ifdef TEST_IMU
    `include "imu_sim2_include.vh"
`endif

      program_quantization;
      program_huffman;
      program_curves;
      program_focus_filt;
      program_coring;
// lower 3 bits of left/right/top/bottom will be ignored. Window includes borders
// left[11:0], right[11:0], top[11:0], bottom[11:0], full_width[11:0], power, filter_sel[3:0];
//      set_focus_filt(0,100,8,24,127,0,0);
      set_focus_filt(0,100,8,24,127,0,1);
      set_zero_bin  (8'hc0,8'h80); // zero_bin 0.75 (half), bias (0.5 - true rounding)

   cpu_wr(X313_WA_CAMSYNCPER,  {8'b0,TRIG_PERIOD}); /// starts generatoe *******************new 



   cpu_wr('ha,'h0);   // no frame sync delay
   cpu_wr('h5,'h8000); // no photofinish yet
   cpu_wr('h5,'h4000 | FULL_WIDTH); //number of pixelas in a line - 68 (for internal HACT)

/// set flat field

/*
 parameter X313_WA_IOPINS=        'h70;    // bits [31:24] - enable channels (channel 0 -software, enabled at FPGA init)
 parameter X313_WA_IOPINS_EN_TRIG_OUT='h0c000000;

`ifdef TRIGGERED_MODE
  parameter TRIG_MODE_CTL=X313_WA_DCR0_TRIGEN;
`else
  parameter TRIG_MODE_CTL=X313_WA_DCR0_TRIDIS;
`endif

   cpu_wr('h31,'h00000000); // [BB] => 0x0
   cpu_wr('h31,'h00400000); // [AC] => 0
   cpu_wr('h31,'h00448000); // [CC] => 0x8000
   cpu_wr('h31,'h00480000); // [AA] => 0
   cpu_wr('h31,'h004c0000); // [CA] => 0
   cpu_wr('h31,'h00800000); // [BC] => 0
   cpu_wr('h31,'h00a00000); // [BA] => 0
   cpu_wr('h31,'h00c00000); // [AB] => 0
   cpu_wr('h31,'h00df0000); // [AB] => 0
   cpu_wr('h31,'h00e00000); // [CB] => 0
   cpu_wr('h31,'h00502000); // [scales0] => 32768
   cpu_wr('h31,'h00524000); // [scales1] => 32768
   cpu_wr('h31,'h00548000); // [scales2] => 32768
   cpu_wr('h31,'h00570000); // [scales3] => 32768
   cpu_wr('h31,'h00600000); // [fatzero_in] => 0
   cpu_wr('h31,'h00610000); // [fatzero_out] => 0
   cpu_wr('h31,'h00620001); // [post_scale] => 3 - X
*/

// test i2c_writeonly
   cpu_wr('h5f,'h4f0a); // reset, 4 bytes long, 10 clocks/quarter bit
// twice - to set known state in the simulator

   cpu_wr('h6f,'h4000); // cmd_sequencer: reset
   cpu_wr('h6f,'h4000); // cmd_sequencer: reset // twice - to set known state in the simulator

   cpu_wr('h5f,'h4f0a); // reset, 4 bytes long, 10 clocks/quater bit
   cpu_rd('h16)   ; I2C_FRAME[2:0]=CPU_DI[2:0];//status
   cpu_rd('h16)   ; I2C_FRAME[2:0]=CPU_DI[2:0];//status
   cpu_rd('h16)   ; I2C_FRAME[2:0]=CPU_DI[2:0];//status
   cpu_rd('h16)   ; I2C_FRAME[2:0]=CPU_DI[2:0];//status
   cpu_rd('h16)   ; I2C_FRAME[2:0]=CPU_DI[2:0];//status
   cpu_rd('h16)   ; I2C_FRAME[2:0]=CPU_DI[2:0];//status
   cpu_rd('h16)   ; I2C_FRAME[2:0]=CPU_DI[2:0];//status
// testing software i2c
   cpu_wr('h5f,'h3000); // run i2c - reset software bits
   cpu_wr('h5f,'h2000); // stop i2c (hardware - enable software)
   cpu_wr('h5f,'h80000); // SDA = 1
   cpu_wr('h5f,'h40000); // SDA = 0
   cpu_wr('h5f,'h20000); // SCL = 1
   cpu_wr('h5f,'h10000); // SCL = 0
   cpu_wr('h5f,'h80000); // SDA = 1
   cpu_wr('h5f,'h20000); // SCL = 1
   cpu_wr('h5f,'hc0000); // SDA = z
   cpu_wr('h5f,'h30000); // SCL = z

   cpu_wr('h5f,'h3000); // run i2c
   
   cpu_wr('h51,'h90040793);
   cpu_wr('h52,'h90050a23);
   
   cpu_wr('h52,'h90080001);
   cpu_wr('h53,'h90090123);
   
   cpu_wr('h5a,'h90091234);
   
   cpu_wr('h54,'h9004001f);
   cpu_wr('h54,'h9005002f);

   cpu_wr('h5b,'h90020013);
   cpu_wr('h5b,'h90030017);
// now - sequencer   
   cpu_wr('h6f,'h3000); // cmd_sequencer: run

// Modify to use new control register control
//   cpu_wr(   0,'h80000000);
//   cpu_wr('h68,'h0603a646);
   cpu_wr('h68, 'h4e000000 |
                       'h7 | // bayer=3
 //                     'h60 | // enable resetting memory channels for each frame
                     'h180 | // zoran=1
                    'h6000 | // arst= 1
                   'h18000 ); // aro= 1
   cpu_wr('h68, 'h4f000000 |
                       'h3); // mrst=1

//   cpu_wr(5,'h24); // write number of lines - prevent "x" in frame counter 
//   cpu_wr('h61,'h05000024);
//   cpu_wr('h61,'h05000030); // intentionally wrong
   cpu_wr('h68,'h05000030); // intentionally wrong
//   cpu_wr('h64,'h05000024); //correct
   cpu_wr('h63,'h05000000 | FULL_HEIGHT); //correct
   cpu_wr('h63,'h05008000 | ((PF_STRIPES!=0)?(PF_STRIPES-1):0)); // number of frames in one (photofinish mode, |0x8000)
   
   

   cpu_rd('h10)   ; //status

// program rtc
   cpu_wr('h4a,      'h8000);   //  maximal correction to the rtc
   cpu_wr('h48,     'h00000);   //  microseconds
   cpu_wr('h49,32'h12345678);   //  seconds
//   #1000;
//   cpu_wr('h45,32'h12345678);   //  seconds     - repeat for simulation

/* ********************************** INIT was here *************************
      program_quantization;
      program_huffman;
      program_curves;
      program_focus_filt;
// lower 3 bits of left/right/top/bottom will be ignored. Window includes borders
// left[11:0], right[11:0], top[11:0], bottom[11:0], full_width[11:0], power, filter_sel[3:0];
//      set_focus_filt(0,100,8,24,127,0,0);
      set_focus_filt(0,100,8,24,127,0,1);
      set_zero_bin  (8'hc0,8'h80); // zero_bin 0.75 (half), bias (0.5 - true rounding)
*/
//  input [11:0] left;
//  input [11:0] right;
//  input [11:0] top;
//  input [11:0] bottom;
//  input [11:0] full_width; // 4 LSBs ignored
//  input [ 3:0] filter_sel;
//  input        filter_strength;

//histogram
//   cpu_wr('h40,'h002);   // left - 2
//   cpu_wr('h41,'h002);  // top - 2
//   cpu_wr('h42,'h07ff);   // bigger that the readout window
//   cpu_wr('h43,'h07ff); // bigger that the readout window

//   cpu_wr('h61,'h40000002);   // left   
   cpu_wr('h61,'h40000000 | HISTOGRAM_LEFT);   // left   
   cpu_wr('h61,'h41000000 | HISTOGRAM_TOP );   // top
//   cpu_wr('h61,'h420007ff);
//   cpu_wr('h61,'h430007ff);
   cpu_wr('h61,'h42000000 | (HISTOGRAM_WIDTH-2)); // width
   cpu_wr('h61,'h43000000 | (HISTOGRAM_HEIGHT-2)); // height


    TEST_TITLE = "INIT_DRAM";
    $display("===================== TEST_%s ========================= @%t",TEST_TITLE,$time);

      init_sdram;

    TEST_TITLE = "INIT_DRAM_DONE";
    $display("===================== TEST_%s ========================= @%t",TEST_TITLE,$time);
`ifdef ENDFRAMES
   cpu_wr(X313_WA_DCR0, X313_WA_DCR0_ENDFRAMESEN);   // enable ending frames if insufficient data
`else
   cpu_wr(X313_WA_DCR0, X313_WA_DCR0_ENDFRAMESDIS);   // disable ending frames if insufficient data
`endif
`ifdef RESET_MCONTR
   cpu_wr(X313_WA_DCR0, X353_WA_DCR0_RESET_MCONTREN);   // enable ending frames if insufficient data
`else
   cpu_wr(X313_WA_DCR0, X353_WA_DCR0_RESET_MCONTRDIS);   // disable ending frames if insufficient data
`endif

   cpu_wr(7,32'h30);   // divisor
   
   

// +++++++++++ Normal mode ++++++++++

/*
`ifdef PF
  parameter PF_HEIGHT=8;
  parameter FULL_HEIGHT=WOI_HEIGHT;
  parameter PF_STRIPES=WOI_HEIGHT/PF_HEIGHT;
//   init_chan_seq ('h62,2,1,0,1,'h200000,'h07,'h10);  // ch2,mode1,wnr0,depend1,sa000000,nTileX10, nTileY10

   init_chan_seq ('h62,2,1,(PF_HEIGHT>0)?1:0,1,'h200000,(WOI_WIDTH>>3)-1,(WOI_HEIGHT & 'h3ff0)-'h10);  // ch2,mode1,wnr0,depend1,sa000000,nTileX10, nTileY10
//   cpu_wr('h62,'h0d00000f); //15 +1 MCU to process - wrong, each frame has just 8 here

   cpu_wr('h62,'h0d000000 | (((WOI_WIDTH>>4)*(WOI_HEIGHT>>4))-1); //7 +1 MCU to process

*/




//   init_chan (0,0,1,1,'h200000,'h07,'h23);  // 

   init_chan (0,0,1,1,'h200000,(WOI_WIDTH>>4)-1,FULL_HEIGHT -1); // channel 0 - let it have depend?



   cpu_wr('h4c,'h1);   // time stamp mode 1 (
// +++++++++++ photofinish mode ++++++++++
//   init_chan (0,0,1,1,'h200000,'h07,'h1f);  // 
//   cpu_wr('h48,'h2);   //  time stamp mode 2

   
//   cpu_wr(2,32'h4);   // 16 bit mode, no FPN correction

/*
   cpu_wr(2,32'h0);   // 8 bit mode, no FPN correction
   cpu_wr(3,32'h0);   // real, not virtual trigger
     cpu_wr('ha,32'h0);   // no frame sync delay
   cpu_wr(4,32'h5);   // continuous, internal, enable
*/
   cpu_wr('h61,'h02000000);
   cpu_wr('h61,'h03000000);
//   cpu_wr('h62,'h0a000000);
//   cpu_wr('h61,'h04000005);
   cpu_wr('h61,DEPEND?'h04000005:'h04000004); // if no depend, do not run sensor channel continuously, just once (to get some data)

//   cpu_wr('h63,'h04000000); // testing - just stop
//   cpu_wr('h62,'h04000000); // testing - just stop
//   cpu_wr('h63,'h04000004); // testing - single


//   cpu_wr(4,32'he);   // single, external, enable, frames
//   cpu_wr(4,32'h6);   // single, external, enable, lines

//     dma_en(0,1);
     dma_en(1,1);
    TEST_TITLE = "DMA_EN_1_1";
    $display("===================== TEST_%s ========================= @%t",TEST_TITLE,$time);

//***************   cpu_wr(1,32'h00000);   // disable and reset dma *** immediate ***
//   cpu_wr('h60,'h01000000);

      // testing end of SDRAM page - process 17 tiles starting at the 3-rd 128-words ina 512 words page
      //   init_chan (2,1,0,1,'h200000,'h02,'h20);  // ch2,mode1,wnr0,depend1,sa000000,nTileX2, nTileY20
      //   init_chan (2,1,0,1,'h200180,'h10,'h12);  // ch2,mode1,wnr0,depend1,sa000000,nTileX10, nTileY10

      // init_chan (2,1,0,1,'h200180,'h10,'h12);  // ch2,mode1,wnr0,depend1,sa000000,nTileX10, nTileY10
      //   init_chan (2,1,0,1,'h200180,'h08,'h22);  // ch2,mode1,wnr0,depend1,sa000000,nTileX10, nTileY10
      //   init_chan (2,1,0,1,'h200180,'h07,'h21);  // ch2,mode1,wnr0,depend1,sa000000,nTileX10, nTileY10
 // +++++++++++ Normal mode ++++++++++
//     init_chan (2,1,0,1,'h200000,'h07,'h10);  // ch2,mode1,wnr0,depend1,sa000000,nTileX10, nTileY10
// +++++++++++ photofinish mode ++++++++++
//     init_chan (2,3,0,1,'h200000,'h07,'h10);  // ch2,mode3,wnr0,depend1,sa000000,nTileX10, nTileY10


    //   cpu_wr('hf,32'h00003);   // 3 +1 MCU to process
    //   cpu_wr('hf,32'h00016);   //22 +1 MCU to process
    // testing end of SDRAM page - process 17 tiles starting at the 3-rd 128-words ina 512 words page
//   cpu_wr('hd,32'h0000f);   //15 +1 MCU to process
//   cpu_wr(1,  32'h70000);   // jpeg, dma enable - both channels !!

//   cpu_wr('h62,'h0d00000f);
//   cpu_wr('h62,'h01070000); // dma?
   cpu_wr('h62,'h0100002d); // dma enable, both channels

$display ("saturation=2");
//   cpu_wr(9,  32'h16c0120);   // saturation=2
   cpu_wr('h62,'h0916c120);   // saturation=2

   cpu_wr('h1c,'hffff);  // reset interrupts ++ now
   cpu_wr('h1c,'hffff);  // second time - for simulator
//    cpu_wr('h1a,'h800f);   // enable waiting for frame sync, enable waiting for dma fifo empty, reset circuitry
//    cpu_wr('h1e,'h0100);   // enable "smart" interrupt
//   cpu_wr('h62,'h1a00800f); // enable waiting for frame sync, enable waiting for dma fifo empty, reset circuitry
//   cpu_wr('h62,'h1e000100); // enable "smart" interrupt
   cpu_wr('h68,'h1a00800f); // ASAP enable waiting for frame sync, enable waiting for dma fifo empty, reset circuitry
   cpu_wr('h68,'h1e000100); // ASAP enable "smart" interrupt

// Program compressor
//      cpu_wr ('hc,'h2ff);
//      cpu_wr ('hc,'h62ff); // focus mode 3
//      cpu_wr ('hc,'h63ff); // focus mode 3 - low quality
//      cpu_wr_isr ('hc,{FOCUS_MODE[1:0],13'h3ff}); // just continue - nothing but interrupt is reset (0x100 - second quant. table)
//      FOCUS_MODE[1:0] =  FOCUS_MODE[1:0]+1;
//color
/*
   cpu_wr('h62,'h0c0063ff);  // focus mode 3 - low quality
   cpu_wr('h63,'h0c0003ff);  // focus mode 0
   cpu_wr('h63,'h0c0023ff);  // focus mode 1
   cpu_wr('h63,'h0c0043ff);  // focus mode 2
   cpu_wr('h64,'h0c0063ff);  // focus mode 3
*/
//jp4
/*
   cpu_wr('h62,'h0c0077ff);  // focus mode 3 - low quality
   cpu_wr('h63,'h0c0017ff);  // focus mode 0
   cpu_wr('h64,'h0c0037ff);  // focus mode 1
   cpu_wr('h65,'h0c0057ff);  // focus mode 2
   cpu_wr('h66,'h0c0077ff);  // focus mode 3
*/
//  wire [ 8:0]   ta; // now inputs
//  wire          twqe;
//  wire          twhe;
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
// [8:7] == 0,1 - NOP, 2 - disable, 3 - enable subtracting of average value (DC component), bypassing DCT
// [6] == 1 - enable quantization bank select, 0 - disregard bits [5:3]
// [5:3] = quantization page number (0..7)
// [2]== 1 - enable on/off control:
// [1:0]== 0 - reset compressor, stop immediately
//         1 - enable compressor, disable repetitive mode
//         2 - enable compressor, compress single frame
//         3 - enable compressor, enable repetitive mode


   init_chan_seq ('h62,2,1,(PF_HEIGHT>0)?1:0,DEPEND,'h200000,(WOI_WIDTH>>4)-1,(WOI_HEIGHT & 'h3ff0)-'h10);  // ch2,mode1,wnr0,depend1,sa000000,nTileX10, nTileY10
   cpu_wr('h68,'h0d017fff); // just testing that more than 16 bits go through
`ifdef TEST_INSUFFICIENT_DATA
  // Don't set correct number of blocks, leave the number larger
`else
   cpu_wr('h62,'h0d000000 | (((WOI_WIDTH>>4)*(WOI_HEIGHT>>4))-1)); //7 +1 MCU to process
`endif
`ifdef TOO_HIGH_FPS

//   cpu_wr('h63,(X313_WA_CAMSYNCPER<<24) | (( TRIG_PERIOD + (TRIG_PERIOD>>1)) & 'hffffff) ); /// slower frame rate
   cpu_wr('h66,(X313_WA_CAMSYNCPER<<24) | FRAME_COMPRESS_CYCLES_INPUT ); /// slower frame rate
`endif

/// instead of the combined command above, trying separate ones
  cpu_wr('h62,'h0c000040); // quality page 0
  cpu_wr('h62,'h0c002200); // color - mode 1
//  cpu_wr('h62,'h0c002400); // JP46 - mode 2
  cpu_wr('h62,'h0c000006); // mode - single
  cpu_wr('h62, 'h4e000000 | 'h4 );// bayer=0
  cpu_wr('h64, 'h4e000000 | 'h5 );// bayer=1

/*
          AX(0x000000): writing 0x000000 to 0x31
          AY(0x000000): writing 0x080000 to 0x31
           C(0x008000): writing 0x108000 to 0x31
          BX(0x000000): writing 0x200000 to 0x31
          BY(0x000000): writing 0x400000 to 0x31
     scales0(0x008000): writing 0x608000 to 0x31
     scales1(0x008000): writing 0x628000 to 0x31
     scales2(0x008000): writing 0x648000 to 0x31
     scales3(0x008000): writing 0x668000 to 0x31
  fatzero_in(0x000000): writing 0x680000 to 0x31
 fatzero_out(0x000000): writing 0x690000 to 0x31
  post_scale(0x000001): writing 0x6a0001 to 0x31
*/



   cpu_wr('h62,'h31000000); // [AX] => 0x0
   cpu_wr('h62,'h31080000); // [AY] => 0
   cpu_wr('h62,'h31108000); // [C] => 0x8000
   cpu_wr('h62,'h31200000); // [BX] => 0
   cpu_wr('h62,'h31400000); // [BY] => 0

   cpu_wr('h62,'h31608000); // [scales0] => 32768
   cpu_wr('h62,'h31628000); // [scales1] => 32768
   cpu_wr('h62,'h31648000); // [scales2] => 32768
   cpu_wr('h62,'h31668000); // [scales3] => 32768
   cpu_wr('h62,'h31680000); // [fatzero_in] => 0
   cpu_wr('h62,'h31690000); // [fatzero_out] => 0
   cpu_wr('h62,'h316a0001); // [post_scale] => 3 - X

   cpu_wr('h63,'h31020000); // [AX] => 0x20000
   cpu_wr('h63,'h310a0000); // [AY] => 0x20000

   cpu_wr('h64,'h31200000); // [BX] => 0x180000
   cpu_wr('h64,'h31400000); // [BY] => 0x180000

`ifdef CONTINUOUS_COMPRESSION
    TEST_TITLE = "START_CONTINUOUS_COMPRESSION";
    $display("===================== TEST_%s ========================= @%t",TEST_TITLE,$time);

`else
    TEST_TITLE = "INIT_CHAN_SEQ";
   init_chan_seq ('h64,2,1,(PF_HEIGHT>0)?1:0,DEPEND,'h200000,(WOI_WIDTH>>4)-1,(WOI_HEIGHT & 'h3ff0)-'h10);  // ch2,mode1,wnr0,depend1,sa000000,nTileX10, nTileY10
  `ifdef TEST_INSUFFICIENT_DATA
  // Don't set correct number of blocks, leave the number larger
  `else
     cpu_wr('h62,'h0d000000 | (((WOI_WIDTH>>4)*(WOI_HEIGHT>>4))-1)); //7 +1 MCU to process
  `endif
`endif

/*
task program_compressor;
  input [ 7:0] address;
  input [ 1:0] focus_mode;
  input [ 1:0] bayer_shift;
  input [ 2:0] tile_shift;
  input [ 3:0] mode;
  input        dcsub;
  input [ 2:0] qpage;
  input [ 1:0] cmd;

*/
`ifdef CONTINUOUS_COMPRESSION
//  program_compressor ('h65,0,0,0, 2, 1,0,3); //focus mode 0 - sub dc, repetitive, mode  5 (jp46),    shift 0 quality=100?
  program_compressor ('h65,0,0,0, 2, 1,1,3); //focus mode 0 - sub dc, repetitive, mode  5 (jp46),    shift 0 quality=70?
`else
  program_compressor ('h64,0,0,0, 2, 1,0,2); //focus mode 0 - sub dc, single,     mode  7 (jp46),    shift 0 quality=100?
  program_compressor ('h65,0,0,0, 2, 1,0,3); //focus mode 0 - sub dc, repetitive, mode  5 (jp46),    shift 0 quality=100?
  program_compressor ('h66,0,0,0, 2, 1,1,3); //focus mode 0 - sub dc, repetitive, mode  2 (jp46),   shift 0 quality=70?
`endif

`ifdef TEST_NO_WAIT_FRAME_SYNC
   cpu_wr('h66,'h1a000002); // disable waiting for frame sync, leave waiting for dma fifo empty
`endif




   init_chan (3,0,1,0,0,'h20,'hf);  // ch3,mode1,wnr1,depend0,sa0,nTileX1f, nTileY2 writes 1 full and 1 small
      #200;
      read_ch3_descript;
        read_status;
`ifdef TEST_CH3_AND_PHASE
    TEST_TITLE = "write256_ch3-1";
    $display("===================== TEST_%s ========================= @%t",TEST_TITLE,$time);
      write256_ch3(16'h1100);
    TEST_TITLE = "write256_ch3-2";
    $display("===================== TEST_%s ========================= @%t",TEST_TITLE,$time);
      write256_ch3(16'h2200);
    TEST_TITLE = "write256_ch3-3";
    $display("===================== TEST_%s ========================= @%t",TEST_TITLE,$time);
      write256_ch3(16'h3300);
    TEST_TITLE = "write256_ch3-4";
    $display("===================== TEST_%s ========================= @%t",TEST_TITLE,$time);
      write256_ch3(16'h4400);
    TEST_TITLE = "write256_ch3-5";
    $display("===================== TEST_%s ========================= @%t",TEST_TITLE,$time);
      write256_ch3(16'h5500);
    TEST_TITLE = "write256_ch3-6";
    $display("===================== TEST_%s ========================= @%t",TEST_TITLE,$time);
      write256_ch3(16'h6600);
    TEST_TITLE = "write256_ch3-7";
    $display("===================== TEST_%s ========================= @%t",TEST_TITLE,$time);
      close_ch3;   // wait write buffer empty
    TEST_TITLE = "write256_ch3-8";
    $display("===================== TEST_%s ========================= @%t",TEST_TITLE,$time);
      read_ch3_descript;
        read_status;
   init_chan (3,0,0,0,0,'h20,'hf);  // ch3,mode1,wnr1,depend0,sa0,nTileX1f, nTileY2 writes 1 full and 1 small
      cpu_wr(8,'h0); // reset errors phase
      read256_ch3;
      repeat (1) cpu_wr(8,'h1); // increase phase
      read256_ch3;
      repeat (1) cpu_wr(8,'h1); // increase phase
      read256_ch3;
//      cpu_wr(8,'h3); // reset phase
      repeat (1) cpu_wr(8,'h1); // increase phase
      read256_ch3;
      repeat (1) cpu_wr(8,'h2); // decrease phase
      read256_ch3;
      repeat (1) cpu_wr(8,'h2); // decrease phase
      read256_ch3;
      repeat (1) cpu_wr(8,'h2); // decrease phase
      read256_ch3;

`endif
///TODO: There is a mixture of several test below, clean them up
`ifdef TRY_SLOW_FPS
    wait (i_x353.i2c_frame_no[2:0]==7);
    cpu_wr('h63,(X313_WA_CAMSYNCPER<<24) | 2*FRAME_COMPRESS_CYCLES_INPUT ); /// twice slower than frame compression
    cpu_wr('h62,(X313_WA_CAMSYNCDLY<<24) | ((TRIG_DELAY<<3) & 'hffffff)); // test longer delay (longer than serial communication)
    cpu_wr('h64,(X313_WA_DCR1<<24) | X313_WA_DCR1_EXTERNALTSDIS); // Shitch to internal timestamps
    cpu_wr('h65,(X313_WA_DCR1<<24) | X313_WA_DCR1_OUTPUTTSDIS); /// turn off sending out timestamps

`endif

/*
//   cpu_wr('h68,'h1a00800f); // ASAP enable waiting for frame sync, enable waiting for dma fifo empty, reset circuitry

   cpu_wr(X313_WA_DCR1,        OUTPUTTS); // enables/disbles sending out timestamps with trigger pulse

 parameter X313_WA_DCR1_OUTPUTTSEN= 'h300000;
 parameter X313_WA_DCR1_OUTPUTTSDIS='h200000;

  parameter EXTERNALTS=    X313_WA_DCR1_EXTERNALTSDIS;
   cpu_wr(X313_WA_DCR1,        EXTERNALTS); // use external (if available) or internal timestamps in frme data

*/


/**/
/*      forever begin
         #100    read_status;
         cpu_rd('h16)   ; I2C_FRAME[2:0]=CPU_DI[2:0];//status
         cpu_wr('h47,0);   //  latch rtc out
         cpu_rd('h44);
         cpu_rd('h45);
         cpu_rd('h14); // image pointer

         end
*/         
   end
//TODO: forever loop that reads status (above) mess up with reading histograms (below)
 initial begin
   #530000;     // sync to frame
//   cpu_wr('h42,'h000100);   // start histogram read from second color
   cpu_wr('h44,'h000000);   // start histogram read from first color
   histogram_total =0;
   histogram_count =0;
   repeat (1024) begin
     cpu_rd_ce1('h45);
     histogram_total = histogram_total+CPU_DI;
     histogram_count = histogram_count+1;
   end

 end


// CPU tasks
parameter CPU_C_A   =    3;
parameter CPU_A_WL   = 10;
parameter CPU_A_RL   = 10; //10;
parameter CPU_RL_RH   = 10; //10;
parameter CPU_WL_D    = 4;
parameter CPU_D_WH   = 16;
parameter CPU_WH_A    = 2;
parameter CPU_WH_D    = 4;
parameter CPU_RH_A    = 2;



  task   dma_en;
    input   chn; // DMA channel (0/1)
    input   d;   // 0 - disable, 1 - enable;
    begin
     if (chn) BUS_EN[BUSOP_DMA_1] = d;
     else     BUS_EN[BUSOP_DMA_0] = d;
    end
  endtask

  task   dma_rd;
    input  [ 7:0]  ia;
    input  [ 7:0]  burst;
    integer        i;
    begin
            wait (DREQ); // in this model CPU will not abanon DMA even if DREQ is reset before granted
            wait (~BUS[BUSOP_DMA_0]);
            BUS_RQ[BUSOP_DMA_0] = 1;
            wait (BUS[BUSOP_DMA_0]);
            BUS_RQ[BUSOP_DMA_0] = 0;
            wait (CPU_CLK); wait (~CPU_CLK);  wait (CPU_CLK);
            for (i = 0; i < burst; i = i+1) begin
              wait (~CPU_CLK); wait (CPU_CLK);
// once per burst
#(CPU_C_A)    A[7:0] = ia[7:0];
              DACK   = 1'b1;
              CE     = 1'b0;
#(CPU_A_RL)   OE     = 1'b0;
#(CPU_RL_RH)  DMA_DI = D[31:0];
              OE     = 1'b1;
            end
#(CPU_RH_A) A[7:0]   = 8'bx;
            CE       = 1'b1;
            DACK     = 1'b0;
            BUS[BUSOP_DMA_0] =0;
// delay to let data from the CPU be written to the SDRAM
            for (i = 0; i <  burst; i = i+1) begin
              wait (~CPU_CLK); wait (CPU_CLK);
              wait (~CPU_CLK); wait (CPU_CLK);
            end
      end
  endtask

  task   dma_rd_1;
    input  [ 7:0]  ia;
    input  [ 7:0]  burst;
    integer        i;
    begin
            wait (DREQ1); // in this model CPU will not abanon DMA even if DREQ is reset before granted
            wait (~BUS[BUSOP_DMA_1]);
            BUS_RQ[BUSOP_DMA_1] = 1;
            wait (BUS[BUSOP_DMA_1]);
            BUS_RQ[BUSOP_DMA_1] = 0;
            wait (CPU_CLK); wait (~CPU_CLK);  wait (CPU_CLK);
            for (i = 0; i < burst; i = i+1) begin
              wait (~CPU_CLK); wait (CPU_CLK);
// once per burst
#(CPU_C_A)    A[7:0]   = ia[7:0];
              DACK1    = 1;
              CE       = 0;
#(CPU_A_RL)   OE       = 0;
#(CPU_RL_RH)  DMA_DI_1 = D[31:0];
              OE       = 1;
            end
#(CPU_RH_A) A[7:0]   = 8'bx;
            CE       = 1;
            DACK1    = 0;
            BUS[BUSOP_DMA_1] =0;
// delay to let data from the CPU be written to the SDRAM
            for (i = 0; i <  burst; i = i+1) begin
              wait (~CPU_CLK); wait (CPU_CLK);
              wait (~CPU_CLK); wait (CPU_CLK);
            end
      end
  endtask






  task   cpu_wr;
    input   [ 7:0]   ia;
    input   [31:0]   id;
     begin
             wait (~BUS[BUSOP_IO_WR]);
             BUS_RQ[BUSOP_IO_WR] = 1;
             wait (BUS[BUSOP_IO_WR]);
             BUS_RQ[BUSOP_IO_WR] = 0;
#(CPU_C_A)   A[7:0]          = ia[7:0];
             CE              = 0;
#(CPU_A_WL)  WE              = 0;
             CPU_DO[31:0]    = id[31:0];
#(CPU_WL_D)  CPU_OE          = 1;
#(CPU_D_WH)  WE              = 1;
#(CPU_WH_A)  A[7:0]          = 8'bx;
             CE              = 1;
#(CPU_WH_D-CPU_WH_A) CPU_OE  = 0;
             BUS[BUSOP_IO_WR]    = 0;
     end
  endtask

  task   cpu_rd;
    input   [ 7:0]   ia;
     begin
             wait (~BUS[BUSOP_IO_RD]);
             BUS_RQ[BUSOP_IO_RD] = 1;
             wait (BUS[BUSOP_IO_RD]);
             BUS_RQ[BUSOP_IO_RD] = 0;
#(CPU_C_A)   A[7:0]          = ia[7:0];
             CE              = 0;
#(CPU_A_RL)  OE              = 0;
#(CPU_RL_RH) CPU_DI[31:0]    = D[31:0];
             CPU_DI[31:0]    = D[31:0];
             OE              = 1;
#(CPU_RH_A)  A[7:0]          = 8'bx;
             CE              = 1;
             BUS[BUSOP_IO_RD]    = 0;
     end
  endtask

  task   cpu_rd_ce1;
    input   [ 7:0]   ia;
     begin
             wait (~BUS[BUSOP_IO_RD1]);
             BUS_RQ[BUSOP_IO_RD1] = 1;
             wait (BUS[BUSOP_IO_RD1]);
             BUS_RQ[BUSOP_IO_RD1] = 0;
#(CPU_C_A)   A[7:0]          = ia[7:0];
             CE1             = 0;
#(CPU_A_RL)  OE              = 0;
#(CPU_RL_RH) CPU_DI[31:0]    = D[31:0];
             OE              = 1;
#(CPU_RH_A)  A[7:0]          = 8'bx;
             CE1             = 1;
             BUS[BUSOP_IO_RD1]    = 0;
     end
  endtask

  task   cpu_wr_isr;
    input   [ 7:0]   ia;
    input   [31:0]   id;
     begin
             wait (~BUS[BUSOP_ISR_WR]);
             BUS_RQ[BUSOP_ISR_WR] = 1;
             wait (BUS[BUSOP_ISR_WR]);
             BUS_RQ[BUSOP_ISR_WR] = 0;
#(CPU_C_A)   A[7:0]          = ia[7:0];
             CE              = 0;
#(CPU_A_WL)  WE              = 0;
             CPU_DO[31:0]    = id[31:0];
#(CPU_WL_D)  CPU_OE          = 1;
#(CPU_D_WH)  WE              = 1;
#(CPU_WH_A)  A[7:0]          = 8'bx;
             CE              = 1;
#(CPU_WH_D-CPU_WH_A) CPU_OE  = 0;
             BUS[BUSOP_ISR_WR]    = 0;
     end
  endtask

  task   cpu_rd_isr;
    input   [ 7:0]   ia;
     begin
             wait (~BUS[BUSOP_ISR_RD]);
             BUS_RQ[BUSOP_ISR_RD] = 1;
             wait (BUS[BUSOP_ISR_RD]);
             BUS_RQ[BUSOP_ISR_RD] = 0;
#(CPU_C_A)   A[7:0]          = ia[7:0];
             CE              = 0;
#(CPU_A_RL)  OE              = 0;
#(CPU_RL_RH) CPU_DI[31:0]    = D[31:0];
             CPU_DI[31:0]    = D[31:0];
             OE              = 1;
#(CPU_RH_A)  A[7:0]          = 8'bx;
             CE              = 1;
             BUS[BUSOP_ISR_RD]    = 0;
     end
  endtask

  task   cpu_rd_ce1_isr;    //SuppressThisWarning Veditor UNUSED TASK
    input   [ 7:0]   ia;
     begin
             wait (~BUS[BUSOP_ISR_RD1]);
             BUS_RQ[BUSOP_ISR_RD1] = 1;
             wait (BUS[BUSOP_ISR_RD1]);
             BUS_RQ[BUSOP_ISR_RD1] = 0;
#(CPU_C_A)   A[7:0]          = ia[7:0];
             CE1             = 0;
#(CPU_A_RL)  OE              = 0;
#(CPU_RL_RH) CPU_DI[31:0]    = D[31:0];
             OE              = 1;
#(CPU_RH_A)  A[7:0]          = 8'bx;
             CE1             = 1;
             BUS[BUSOP_ISR_RD1]    = 0;
     end
  endtask




parameter   SDRAM_WAITINIT= 1000;   // actually - 100usec - will it check?
parameter   SDRAM_MANCMD=   8'h23;
parameter   SDRAM_ENABLE=   8'h27; //!NOTE: Changed format - now each bit is replaced by a dibit - 0x - don't change, 10 - reset, 11 - set
parameter   CHN_BASEA=      8'h20;
parameter   CHN3_BASEA=      8'h2c;
parameter   STATUS_ADDR=   8'h10;
parameter   CH3_DATA_WND=   8'h30;
parameter   CH3_RDY_BITNUM=7;
parameter   CH3_WEMPTY_BITNUM=8;   // was 0??
   task read_status;
     begin
      cpu_rd(STATUS_ADDR);
     end
   endtask


   task init_sdram;
    begin
      #(SDRAM_WAITINIT);
      cpu_wr(SDRAM_MANCMD,32'h17fff);   // precharge, a[10]=1 - all banks
      #(100);
      cpu_wr(SDRAM_MANCMD,32'h02000);   // load extended mode register - enable DLL
      #(100);
      cpu_wr(SDRAM_MANCMD,32'h00163);   // load mode register (CL=2.5, burst length=8 - no full page)and reset DLL
      #(100);
      cpu_wr(SDRAM_MANCMD,32'h17fff);   // precharge, a[10]=1 - all banks
      #(100);
      cpu_wr(SDRAM_MANCMD,32'h8000);   // refresh
      #(100);
      cpu_wr(SDRAM_MANCMD,32'h8000);   // refresh
      #(100);
      cpu_wr(SDRAM_MANCMD,32'h00063);   // load mode register (CL=2.5, burst length=8 - no full page)and reset DLL - not need for Micron
      #(100);


// enable SDRAM controller and refresh (all channels disabled)
      SDRAM_MODE = 'h00;   cpu_wr(SDRAM_ENABLE,{20'b0,SDRAM_MODE});   // to init to 0 (for simulation only)
      #(100);
//      SDRAM_MODE=6'h03;   cpu_wr(SDRAM_ENABLE,SDRAM_MODE);   // All channels disabled, only refresh and sdram itself
      SDRAM_MODE= 'haaf; cpu_wr(SDRAM_ENABLE,{20'b0,SDRAM_MODE}); // All channels disabled, only refresh and sdram itself
    end
   endtask


   task init_chan;
    input   [1:0]   ichnum;   // channel number 0..3
    input   [1:0]   imode;   // mode - 3 - 1+ "rollover"
    input         iwnr;      // write
    input         idep;
    input  [26:0]   isa;      // start address - lower 8 bits will be ignored
    input   [9:0]   inTileX;   // mode0: 5 MSBs - number of full (256*16) pages in line, 4 LSBs - additional partial page
    input  [11:0] inTileY; // in mode1 only 7MSBs are used
    reg     [24:0]   sa;
    reg     [15:0]   w0;
    reg     [15:0]   w1;
    reg     [15:0]   w2;
    begin
    sa[24:0] = {isa[24:8],8'b0};
    $display ("init_chan: num=%x, mode=%x, rollover=%x, WnR=%x, depend=%x, startAddr=%x, nTileX=%x, ntileY=%x",ichnum,imode[0], imode[1],iwnr,idep,sa,inTileX,inTileY);
//    w0[15:0]=   {imode,iwnr,idep,1'b0,isa[19:8]};
    w0[15:0]=   {imode[0],(iwnr | &imode[1:0]) ,idep,isa[20:8]};
    w1[15:0]=   {2'b0,inTileX[9:0],isa[24:21]};
    w2[15:0]=   {4'b0,inTileY[11:0]};
    $display ("    writing %x to %x",w1,CHN_BASEA+4*ichnum+1);
      cpu_wr(CHN_BASEA+4*ichnum+1,{16'b0,w1});
    $display ("    writing %x to %x",w2,CHN_BASEA+4*ichnum+2);
      cpu_wr(CHN_BASEA+4*ichnum+2,{16'b0,w2});
    $display ("    writing %x to %x",w0,CHN_BASEA+4*ichnum+0);
      cpu_wr(CHN_BASEA+4*ichnum+0,{16'b0,w0});
// enable channel:
//      SDRAM_MODE=SDRAM_MODE | (6'h4 << ichnum);
      SDRAM_MODE= (12'h030 << (ichnum <<1));
      case (ichnum )
       4'h0: SDRAM_MODE= 12'h030;
       4'h1: SDRAM_MODE= 12'h0c0;
       4'h2: SDRAM_MODE= 12'h300;
       4'h3: SDRAM_MODE= 12'hc00;
      endcase
      cpu_wr(SDRAM_ENABLE,{20'b0,SDRAM_MODE});
    end
   endtask
//     init_chan (2,1,0,1,'h200000,'h07,'h10);  // ch2,mode1,wnr0,depend1,sa000000,nTileX10, nTileY10

   task init_chan_seq;
    input   [7:0] seq_addr; // sequencer address to write to
    input   [1:0]   ichnum;   // channel number 0..3
    input   [1:0]   imode;   // mode - 3 - 1+ "rollover"
    input         iwnr;      // write
    input         idep;
    input  [26:0]   isa;      // start address - lower 8 bits will be ignored
    input   [9:0]   inTileX;   // mode0: 5 MSBs - number of full (256*16) pages in line, 4 LSBs - additional partial page
    input  [13:0] inTileY; // in mode1 only 7MSBs are used
    reg     [24:0]   sa;
    reg     [15:0]   w0;
    reg     [15:0]   w1;
    reg     [15:0]   w2;
    reg     [ 7:0]   a0;
    reg     [ 7:0]   a1;
    reg     [ 7:0]   a2;
    begin
    sa[24:0] = {isa[24:8],8'b0};
    $display ("init_chan_seq: num=%x, mode=%x, rollover=%x, WnR=%x, depend=%x, startAddr=%x, nTileX=%x, ntileY=%x",ichnum,imode[0], imode[1],iwnr,idep,sa,inTileX,inTileY);
//    w0[15:0]=   {imode,iwnr,idep,1'b0,isa[19:8]};
    w0[15:0]=   {imode[0],(iwnr | &imode[1:0]) ,idep,isa[20:8]};
    w1[15:0]=   {2'b0,inTileX[9:0],isa[24:21]};
    w2[15:0]=   {2'b0,inTileY[13:0]};
    a0[7:0]=CHN_BASEA+4*ichnum+0;
    a1[7:0]=CHN_BASEA+4*ichnum+1;
    a2[7:0]=CHN_BASEA+4*ichnum+2;
    $display ("    writing %x to %x",{a1[7:0],8'h0,w1[15:0]},seq_addr);
      cpu_wr(seq_addr, {a1[7:0],8'h0,w1[15:0]});
    $display ("    writing %x to %x",{a2[7:0],8'h0,w2[15:0]},seq_addr);
      cpu_wr(seq_addr, {a2[7:0],8'h0,w2[15:0]});
    $display ("    writing %x to %x",{a0[7:0],8'h0,w0[15:0]},seq_addr);
      cpu_wr(seq_addr, {a0[7:0],8'h0,w0[15:0]});

// enable channel:
//      SDRAM_MODE=SDRAM_MODE | (6'h4 << ichnum);
//      SDRAM_MODE= (12'h030 << (ichnum <<1));
      case (ichnum )
       4'h0: SDRAM_MODE= 12'h030;
       4'h1: SDRAM_MODE= 12'h0c0;
       4'h2: SDRAM_MODE= 12'h300;
       4'h3: SDRAM_MODE= 12'hc00;
      endcase
//      cpu_wr(SDRAM_ENABLE,SDRAM_MODE);
    $display ("    writing %x to %x",{SDRAM_ENABLE[7:0],12'h0,SDRAM_MODE[11:0]},seq_addr);
      cpu_wr(seq_addr, {SDRAM_ENABLE[7:0],12'h0,SDRAM_MODE[11:0]});
    end
   endtask


   task close_ch3;
    begin
        read_status;
      while (!CPU_DI[CH3_WEMPTY_BITNUM]) read_status;
    end
   endtask

   task read_ch3_descript;
     begin
      cpu_rd(CHN3_BASEA+0);
      cpu_rd(CHN3_BASEA+1);
      cpu_rd(CHN3_BASEA+2);
     end
   endtask

   task write256_ch3;   // write 16 16-bit words (as 8 x 32) , each next is greater by 1, starting with first;
     input   [15:0] first;
     reg    [15:0] i;
     begin
     $display ("write_256 (%d)",first);
        read_status;
      while (!CPU_DI[CH3_RDY_BITNUM]) read_status;
//      for (i = 0; i < 128; i = i+1) cpu_wr(CH3_DATA_WND+i,first+2*i+(first+2*i+1)*65536);
      for (i = 0; i < 128; i = i+1) cpu_wr(CH3_DATA_WND,first+2*i+(first+2*i+1)*65536);
      cpu_wr(CHN3_BASEA+3,32'b0);
     end
   endtask

/*
   task writeGrad_ch3;   // write 256 8-bit words (as 64 x 32) , same all lines, with value=first+x
     input   [7:0] first;
     integer i;
     reg      [7:0] j;
     begin
      $display ("writeGrad_ch3 (%d)",first);
        read_status;
      while (!CPU_DI[CH3_RDY_BITNUM]) read_status;
      for (i =0; i< 128; i = i+8) begin
        for (j = 0; j < 32; j = j+4)
//            cpu_wr(CH3_DATA_WND+i+(j>>2),{(first+(j & 4'hf)+2'h3),(first+(j & 4'hf)+2'h2),(first+(j & 4'hf)+2'h1),(first+(j & 4'hf)+2'h0)});
            cpu_wr(CH3_DATA_WND,{(first+(j & 4'hf)+2'h2),(first+(j & 4'hf)+2'h1),(first+(j & 4'hf)+2'h0)});
      end
      cpu_wr(CHN3_BASEA+3,32'b0);
    end
   endtask

 task writeGrad_ch3_A;   // write 256 8-bit words (as 64 x 32) , same all lines, with value=first+x
     input   [7:0] first;
     integer i;
     reg      [7:0] j;
     begin
      $display ("writeGrad_ch3_A (%d)",first);
      read_status;
      while (!CPU_DI[CH3_RDY_BITNUM]) read_status;
      for (i =0; i< 128; i = i+8) begin
        for (j = 0; j < 32; j = j+4)
//            cpu_wr(CH3_DATA_WND+i+(j>>2),{(first+(((j & 4'hf)+2'h3)<<4)),(first+(((j & 4'hf)+2'h2)<<4)),(first+(((j & 4'hf)+2'h1)<<4)),(first+(((j & 4'hf)+2'h00)<<4))});
            cpu_wr(CH3_DATA_WND,{(first+(((j & 4'hf)+2'h2)<<4)),(first+(((j & 4'hf)+2'h1)<<4)),(first+(((j & 4'hf)+2'h0)<<4))});
      end
      cpu_wr(CHN3_BASEA+3,32'b0);
    end
   endtask

*/
// 1-cycle latency!
   task read256_ch3;   // read 16 16-bit words (as 8 x 32)
     reg    [15:0] i;
     begin
        read_status;
      while (!CPU_DI[CH3_RDY_BITNUM]) read_status;
//      cpu_rd(CH3_DATA_WND);
      cpu_rd_ce1(CH3_DATA_WND);
      for (i = 0; i < 127; i = i+1) begin
//        cpu_rd(CH3_DATA_WND+i+1);
        cpu_rd_ce1(CH3_DATA_WND);
      $display ("ch3 (%x) = %x",i,CPU_DI);
      end
//      cpu_rd(CH3_DATA_WND+127);
      cpu_rd_ce1(CH3_DATA_WND);
      $display ("ch3 (%x) = %x",127,CPU_DI);
      cpu_wr(CHN3_BASEA+3,32'b0);
     end
   endtask
//parameter   CH3_RDY_BITNUM=4;
//parameter   CH3_WEMPTY_BITNUM=5;   // was 0??
/*
   task read128_ch3;   
     integer i;
     begin
     $display ("read_128");
      for (i = 0; i < 8; i = i+1) read_ch3;
     end
   endtask
*/
task program_huffman;
// huffman tables data
  reg   [23:0]   huff_data[0:511]; // SuppressThisWarning VEditor : assigned in $readmem() system task
  integer i;
  begin
    $readmemh("huffman.dat",huff_data);
    cpu_wr ('he,'h200);   // start address of huffman tables
    for (i=0;i<512;i=i+1) begin
      cpu_wr('hf,{8'b0,huff_data[i]});
    end
  end
endtask

task program_quantization;
// quantization tables data
//  reg   [11:0]   quant_data[0:255];
  reg   [15:0]   quant_data[0:255];  // SuppressThisWarning VEditor : assigned in $readmem() system task
  integer i;
  begin
//    $readmemh("quantization.dat",quant_data);
    $readmemh("quantization_100.dat",quant_data);
    cpu_wr ('he,'h0);   // start address of quantization tables
    for (i=0;i<256;i=i+2) begin
      cpu_wr('hf,{quant_data[i+1],quant_data[i]});
    end
  end
endtask

task program_coring;
// coring tables data
  reg   [15:0]   coring_data[0:1023];  // SuppressThisWarning VEditor : assigned in $readmem() system task
  integer i;
  begin
//    $readmemh("quantization.dat",quant_data);
    $readmemh("coring.dat",coring_data);
    cpu_wr ('he,'hc00);   // start address of coring tables
    for (i=0;i<1024;i=i+2) begin
      cpu_wr('hf,{coring_data[i+1],coring_data[i]});
    end
  end
endtask



task program_focus_filt;
// focus quality filter data 
  reg   [15:0]   filt_data[0:127];  // SuppressThisWarning VEditor : assigned in $readmem() system task
  integer i;
  begin
    $readmemh("focus_filt.dat",filt_data);
    cpu_wr ('he,'h800);   // start address of focus filter tables
    for (i=0;i<128;i=i+1) begin
      cpu_wr('hf,{16'b0,filt_data[i]});
    end
  end
endtask

/*

task set_focus_filt;
// lower 3 bits of left/right/top/bottom will be ignored. Window includes borders
  input [11:0] left;
  input [11:0] right;
  input [11:0] top;
  input [11:0] bottom;
  input [11:0] full_width; // 4 LSBs ignored
  input [ 3:0] filter_sel;
  input        filter_strength;
  begin
    cpu_wr ('he,'hbc0); // start address of focus parameters (page 15)
    cpu_wr ('hf,left[11:0]);
    cpu_wr ('hf,right[11:0]);
    cpu_wr ('hf,top[11:0]);
    cpu_wr ('hf,bottom[11:0]);
    cpu_wr ('hf,full_width[11:0]);
    cpu_wr ('hf,filter_sel[3:0]);
    cpu_wr ('hf,filter_strength);
  end
endtask

   cpu_wr('h68,'h0c0063ff);  // focus mode 3

*/
task set_focus_filt;
// lower 3 bits of left/right/top/bottom will be ignored. Window includes borders
  input [11:0] left;
  input [11:0] right;
  input [11:0] top;
  input [11:0] bottom;
  input [11:0] full_width; // 4 LSBs ignored
  input [ 3:0] filter_sel;
  input        filter_strength;
  begin
//    cpu_wr ('he,'hbc0);   // start address of focus parameters (page 15)
    cpu_wr('h68,'h0e000bc0);  // ASAP, start address of focus parameters (page 15)
//    cpu_wr ('hf,left[11:0]);
    cpu_wr('h68,{8'h0f,12'h0,left[11:0]});
//    cpu_wr ('hf,right[11:0]);
    cpu_wr('h68,{8'h0f,12'h0,right[11:0]});
//    cpu_wr ('hf,top[11:0]);
    cpu_wr('h68,{8'h0f,12'h0,top[11:0]});
//    cpu_wr ('hf,bottom[11:0]);
    cpu_wr('h68,{8'h0f,12'h0,bottom[11:0]});
//    cpu_wr ('hf,full_width[11:0]);
    cpu_wr('h68,{8'h0f,12'h0,full_width[11:0]});
//    cpu_wr ('hf,filter_sel[3:0]);
    cpu_wr('h68,{8'h0f,20'h0,filter_sel[3:0]});
//    cpu_wr ('hf,filter_strength);
    cpu_wr('h68,{8'h0f,23'h0,filter_strength});
  end
endtask

task set_zero_bin;
// lower 3 bits of left/right/top/bottom will be ignored. Window includes borders
  input [7:0] zero_bin;
  input [7:0] quant_bias;
  begin
//    cpu_wr('h68,'h0e000be0);  // ASAP, start address of focus parameters (page 15)
//    cpu_wr('h68,{8'h0f,8'h0,quant_bias[7:0],zero_bin[7:0]});
    cpu_wr('h68,{8'h0b,8'h0,quant_bias[7:0],zero_bin[7:0]});
  end
endtask


task program_curves;
  reg   [9:0]   curves_data[0:1027]; // SuppressThisWarning VEditor : assigned in $readmem() system task
  integer n,i,base,diff,diff1;
///AF:    reg [10:0] curv_diff;
  begin
    $readmemh("linear1028rgb.dat",curves_data);
//    $readmemh("zero1028rgb.dat",curves_data);
    cpu_wr ('he,'h400);   // start address of quantization tables

    for (n=0;n<4;n=n+1) begin
      for (i=0;i<256;i=i+1) begin
        base =curves_data[257*n+i];
        diff =curves_data[257*n+i+1]-curves_data[257*n+i];
        diff1=curves_data[257*n+i+1]-curves_data[257*n+i]+8;
//        $display ("%x %x %x %x %x %x",n,i,curves_data[257*n+i], base, diff, diff1);
        #1;
        if ((diff>63) || (diff < -64)) cpu_wr('hf,{14'b0,1'b1,diff1[10:4],base[9:0]});
        else                           cpu_wr('hf,{14'b0,1'b0,diff [ 6:0],base[9:0]});
      end
    end  
  end
endtask
/// NOTE: Can not use sequencer to program tables - may collide with software table writes !!!
task pre_program_curves; // all but last word, last word schedule through sequencer to provided address //SuppressThisWarning Veditor UNUSED TASK
  input [7:0]  seq_addr;
  reg     [9:0]   curves_data[0:1027]; // SuppressThisWarning VEditor : assigned in $readmem() system task
  integer n,i,base,diff,diff1;
///AF:    reg [10:0] curv_diff;
  reg [23:0] data;
  begin
    $readmemh("linear1028rgb.dat",curves_data);
//    $readmemh("zero1028rgb.dat",curves_data);
    cpu_wr ('he,'h400);   // start address of quantization tables

    for (n=0;n<4;n=n+1) begin
      for (i=0;i<256;i=i+1) begin
        base =curves_data[257*n+i];
        diff =curves_data[257*n+i+1]-curves_data[257*n+i];
        diff1=curves_data[257*n+i+1]-curves_data[257*n+i]+8;
//        $display ("%x %x %x %x %x %x",n,i,curves_data[257*n+i], base, diff, diff1);
        #1;
        if ((diff>63) || (diff < -64)) data={6'b0,1'b1,diff1[10:4],base[9:0]};
        else                           data={6'b0,1'b0,diff [ 6:0],base[9:0]};
        if ((n<3) || (i<255)) cpu_wr('hf, {8'b0,data});
        else begin
           cpu_wr(seq_addr,'h0e0007ff);
          cpu_wr(seq_addr,{8'h0f,data[23:0]});
        end
        
      end
    end
//   cpu_wr('h61,'h04000005);
    
  end
endtask


task program_compressor;
  input [ 7:0] address;
  input [ 1:0] focus_mode;
  input [ 1:0] bayer_shift;
  input [ 2:0] tile_shift;
  input [ 3:0] mode;
  input        dcsub;
  input [ 2:0] qpage;
  input [ 1:0] cmd;
  begin
   cpu_wr(address[7:0],
         {8'h0c,
          1'b1,focus_mode[1:0],
          1'b1,bayer_shift[1:0],
          1'b1,tile_shift[2:0],
          1'b1,mode[3:0],
          1'b1,dcsub,
          1'b1,qpage[2:0],
          1'b1,cmd[1:0]});
  end
endtask
`ifdef TEST_IMU
    `include "imu_sim_tasks_include.vh"
`endif


endmodule

module oneshot(trigger,
               out);
  input  trigger;
  output out;
  reg    out;
  event  start; //SuppressThisWarning Veditor "event" is not supported in VDT?
  parameter duration=4000;
  initial out= 0;
  always @ (posedge trigger) begin
    disable timeout;
    #0 -> start;
  end
  always @start
    begin : timeout
      out = 1;
      # duration out = 0;
    end
endmodule

module dly5taps (dly_in,
                dly_out);
  input        dly_in;
  output [5:1] dly_out;
  reg    [5:1] dly_out;
  parameter dly=6; // delay per tap, ns

  always @ (dly_in)     # dly dly_out[1] <= dly_in;
  always @ (dly_out[1]) # dly dly_out[2] <= dly_out[1];
  always @ (dly_out[2]) # dly dly_out[3] <= dly_out[2];
  always @ (dly_out[3]) # dly dly_out[4] <= dly_out[3];
  always @ (dly_out[4]) # dly dly_out[5] <= dly_out[4];
endmodule
