/*
** -----------------------------------------------------------------------------**
** x353.v
**
** Top level module
**
** Copyright (C) 2004-2010 Elphel, Inc
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

// `define debug_stuffer
 `define debug_dma_count
 `define debug_compressor
 `define debug_mcontr_reset
// `define DEBUG_IMU
module x353 #(
    parameter IOSTANDARD_CLK =        "LVCMOS33",

    parameter IOSTANDARD_SYS =        "LVCMOS33",
    parameter SLEW_SYS =              "SLOW",
    parameter DRIVE_SYS =             8,

    parameter IBUF_DELAY_SYS_A =      "0",
    parameter IFD_DELAY_SYS_A =       "0",

    parameter IBUF_DELAY_SYS_D =      "0",
    parameter IFD_DELAY_SYS_D =       "0",

    parameter IBUF_DELAY_SYS_WOE =    "0",
    parameter IFD_DELAY_SYS_WOE =     "0",

    parameter IBUF_DELAY_SYS_CE =     "0",
    parameter IFD_DELAY_SYS_CE =      "0",

    parameter IBUF_DELAY_SYS_DACK =   "0",
    parameter IFD_DELAY_SYS_DACK =    "0",

    parameter IBUF_DELAY_SYS_SDCLK =  "0",
    parameter IFD_DELAY_SYS_SDCLK =   "0",

    parameter SLEW_SYS_DREQ =         "SLOW",
    parameter DRIVE_SYS_DREQ =         4,

    parameter IOSTANDARD_EXT =        "LVCMOS33",
    parameter SLEW_EXT =              "SLOW",
    parameter DRIVE_EXT =             12,
    
    parameter IOSTANDARD_SENSOR =     "LVCMOS25",
    parameter SLEW_SENSOR =           "SLOW",
    parameter DRIVE_SENSOR =          4,
    
    parameter IOSTANDARD_SENSOR_CLK = "LVCMOS25",
    parameter SLEW_SENSOR_CLK =       "SLOW",
    parameter DRIVE_SENSOR_CLK =      4,

    parameter IFD_DELAY_SENSOR_PXD =  "0",
    parameter IFD_DELAY_SENSOR_VHACT ="0",
    parameter IBUF_DELAY_SENSOR_PXD =  "0",
    parameter IBUF_DELAY_SENSOR_VHACT ="0",
    
    parameter IOSTANDARD_SDRAM =      "SSTL2_I",
    parameter DRIVE_SDRAM_DATA =      12,
    parameter SLEW_SDRAM_DATA =       "SLOW",
    parameter DRIVE_SDRAM_ABC =       12,
    parameter SLEW_SDRAM_ABC =        "SLOW",
    parameter IOSTANDARD_SDRAM_DIFF = "DIFF_SSTL2_I",
    parameter SLEW_SDRAM_DIFF =       "SLOW"
    
    ) (
    inout   [9:0] PXD,
    inout         DCLK,
    inout         BPF,
    input         VACT,
    input         HACT,
    inout         MRST,
    output        ARO,
    output        ARST,
    inout         SCL0,
    inout         SDA0,
    inout         CNVSYNC,
    inout         CNVCLK,
    inout         SENSPGM,
    inout         DUMMYVFEF, // output is not enough
    inout         ALWAYS0,   // will be pulled down to fool the software - it does not know it is always 0.
    
    inout  [11:0] EXT,
    input         CLK3,
    input         CLK2,
    input         CLK4,
    input         CLK1,
    input         CLK0,
    
    inout         UDQS,
    inout         LDQS,
    inout  [15:0] SDD,
    output [14:0] SDA,
    output        SDWE,
    output        SDCAS,
    output        SDRAS,
    output        SDUDM,
    output        SDLDM,
    input         SDCLK_FB,
    output        SDCLK,
    output        SDNCLK,
    input         SDNCLK_FB,
    output        SDCLKE,
    inout  [31:0] D,
    inout  [12:0] A,
    inout   [1:0] BA,
    output        SYS_SDWE,
    output        SYS_SDCAS,
    output        SYS_SDRAS,
    input         SYS_SDCLKI,
    output        SYS_SDCLK,
    output        SYS_BUSEN,
    input         WE,
    input         OE,
    input         CE,
    input         CE1,
    output        DREQ0,
    input         DACK0,
    output        DREQ1,
    input         DACK1,
    output        IRQ,
    output        BG,
    input         BRIN,
    output        BROUT);
      parameter MODELREV=32'h0353402b;    // adding more bits to motors positions
//      parameter MODELREV=32'h0353402a;    // IMU restart after ready (DIO2)
//      parameter MODELREV=32'h03534029;    // working on IMU
//      parameter MODELREV=32'h03534028;    // DMA for logging, changing DMA control (neds drivers update)
//      parameter MODELREV=32'h03534027;    // NMEA logging
//    parameter MODELREV=32'h03534026;    // IMU logging
//    parameter MODELREV=32'h03534025;    // decimating compressed frames (to increase autoexposure fps in eyesis)
//    parameter MODELREV=32'h03534024;    // implementing LUT-based coring
//    parameter MODELREV=32'h03534023;    // fixing zero bin
//    parameter MODELREV=32'h03534022;    // working on timestamps over inter-camera sync connection
//    parameter MODELREV=32'h03534021;    // fixed lock-ups 
//    parameter MODELREV=32'h03534020;    // more debug, not yet fixed
//    parameter MODELREV=32'h0353401f;    // making sensor trigger signal to toggle if no vacts to prevent lock-up
//    parameter MODELREV=32'h0353401e;    // modifying trigger circuitry (delay to output), filter VACTS
//    parameter MODELREV=32'h0353401d;    // added ending frame compression if no more data is available
//    parameter MODELREV=32'h0353401c;    // debugging stuck compressor, debug output at reg 60
//    parameter MODELREV=32'h0353401b;    // making graceful compressor reset
//    parameter MODELREV=32'h0353401a;    // debugging, modified irq_smart not to miss interrupt when compressing spans over two vacts
//    parameter MODELREV=32'h03534019;    // increasing maximal window size in histograms and focus
//    parameter MODELREV=32'h03534018;    // increasing maximal frame height to 0x4000 (16K)
//    parameter MODELREV=32'h03534017;    // increasing maximal frame height to 0x4000 (16K)
//    parameter MODELREV=32'h03534016;    // adding optional reset of the mcontr channels at each frame to prevent out-of-sync after some broken frames
//    parameter MODELREV=32'h03534015;    // troubleshooting new sync
//    parameter MODELREV=32'h03534014;    // hact duration defined internally, new phase sync
//    parameter MODELREV=32'h03534013;    // Fixing (removing) stray color in mono/jp46 modes
//    parameter MODELREV=32'h03534012;    // Special version with motor control - 10364
//    parameter MODELREV=32'h03534011;    // Migrated to WebPack 10.1.03 - back?? Pullup on SCL0
//    parameter MODELREV=32'h03534010;    // Migrated to WebPack 10.1.03 - back (no, it was typo) Pullup on SCL0
//    parameter MODELREV=32'h0353400f;    // Migrated to WebPack 10.2.03. Fixing lens_flat bugs
//    parameter MODELREV=32'h0353400e;    // changing something - sometimes it helps against Place:848 - Automatic clock placement failed
//    parameter MODELREV=32'h0353400d;    // starting lens flat field correction
//    parameter MODELREV=32'h0353400c;    // fixing histogram more
//    parameter MODELREV=32'h0353400b;    // fixing histogram
//    parameter MODELREV=32'h0353400a;    // aborting frame input at the frame sync even if line counter did not expire
//    parameter MODELREV=32'h03534009;    // number of lines latency
//    parameter MODELREV=32'h03534008;  // adjusting bayer latencies between sensorpix, histograms, compressor
//    parameter MODELREV=32'h03534007;  // bug fix in compressor command, 0x16 now returns both sequencer and i2c sequencer frame no.
//    parameter MODELREV=32'h03534006;  // debugging sequencers
//    parameter MODELREV=32'h03534005;  // bug fix in colorproc + adding comressor abort (with flush) when MCU number is written
//    parameter MODELREV=32'h03534004;  // some cleanup
//    parameter MODELREV=32'h03534003;  // fixing proble when the compression was started too early
//    parameter MODELREV=32'h03534002;  // just changing to see if weird behaviour is not related to hardware
//    parameter MODELREV=32'h03534001;  // improving DCT precision, adding quantizer tuning
//    parameter MODELREV=32'h03533022; // changing memory enable bits (reg 0x27). Now - dibits 0x - don't change, 10 reset, 11 - set
//    parameter MODELREV=32'h03533021; // Quantization tables - now 16 bits (were 12)
//    parameter MODELREV=32'h03533020; // improving jp4 and mono
//    parameter MODELREV=32'h0353301f; // addind pipelined register writing
//    parameter MODELREV=32'h0353301e; // timestamp selected in async mode (early trigger)
//  parameter MODELREV=32'h0353301c; // Two-page gamma tables, switches at firs VACT after writing to the last word
//  parameter MODELREV=32'h0353301b; // i2c master (write only) buffered controller
//  parameter MODELREV=32'h0353301a; // trying SLEW=SLOW DRIVE=4 on the system bus
//  parameter MODELREV=32'h03533019; // fixing another camsync bugs - no async restart of the timing generator
//  parameter MODELREV=32'h03533018; // fixing camsync bug
//  parameter MODELREV=32'h03533017; // limiting the range of focus visualization
//  parameter MODELREV=32'h03533016; // changing table writes from 32-bit to a pair of 16-bit ones, removed dummy_usb
//  parameter MODELREV=32'h03533015; // focus quality - improved
//  parameter MODELREV=32'h03533014; // focus quality
//  parameter MODELREV=32'h03533013; // histogram window bug fix
//  parameter MODELREV=32'h03533012; // external sync, using sensor TRIG pin
//  parameter MODELREV=32'h03533011; //  bug fix in JP4 mode
//  parameter MODELREV=32'h03533010; //  increased sync delay for 10347 to match extra latency in PXD path
//  parameter MODELREV=32'h0353300f; //  4 phase shifts for VACT, HACT, BPF
//  parameter MODELREV=32'h0353300e; //  changing input PXD, HACT, VACT to pclk2x with enable
//  parameter MODELREV=32'h0353300d; //  adding testing PXD at +/i T/4 from the posedge pclk
//  parameter MODELREV=32'h0353300a; //  fixing histogram at hogh pixel clock (double pixel clock)
//  parameter MODELREV=32'h03533009; //  force senspgm high to reduce noise on long cable
//  parameter MODELREV=32'h03533008; //  increasing maximal pixel clock to 96 MHz
//  parameter MODELREV=32'h03533007; //  modified control of 12 I/O to aux board
//  parameter MODELREV=32'h03533006; // 
//  parameter MODELREV=32'h03533005; // fixing getting stuck after setting wrong CLK0 frequency
//  parameter MODELREV=32'h03533004; // switching to 16-bit pixel data (aligned to the MSB)
//  parameter MODELREV=32'h03533003; // fixing dcm phase shift
//  parameter MODELREV=32'h03533002; // fixing dcm phase shift
//  parameter MODELREV=32'h03533001; // Modifying to work with 10359 board
//  parameter MODELREV=32'h03531036; // Moving to ISE 9.1
//  parameter MODELREV=32'h03531035; // Adding phase adjustment for sensor clock
//  parameter MODELREV=32'h03531034; // stopped working - fixed
//  parameter MODELREV=32'h03531033; // Troubleshooting DMA FIFO
//  parameter MODELREV=32'h03531032; // Changing DMA - continue
//  parameter MODELREV=32'h03531031; // Changing DMA - continue
//  parameter MODELREV=32'h03531030; // Changing DMA
//  parameter MODELREV=32'h03531029; // Fixing mcontr problems (pf-mode, increased page size). Mode1+WnR=pcf mode
//  parameter MODELREV=32'h03531028; // Fixing DCM control
//  parameter MODELREV=32'h03531027; // Removing sclk90
//  parameter MODELREV=32'h03531026; // Reverted do 03531023 - it is better to have data bus not simultaneous.
//                                      Will need to get rid of c90 to save global resources
//  parameter MODELREV=32'h03531025; // Could not fit extra cloick - weird BUFG OK, BUGMUX - not. Eliminating on of sclk90/sclk270
//  parameter MODELREV=32'h03531024; // System data bus to use global OE and DDR register for tristate control
           
//  parameter MODELREV=32'h03531023; // Trying monitoring/resetting DCMs. It seem that they stop working if clock changes
//  parameter MODELREV=32'h03531022; // Fixed RTC divider to use 12 MHz input clock
//  parameter MODELREV=32'h03531021; // Changing DMA to 8-word atomic bursts
//  parameter MODELREV=32'h03531021; // improved DQS, restored DUMMYVREF
//  parameter MODELREV=32'h03531020; // New board revision - different pinout (removed DUMMYVREF - wrong)
//  parameter MODELREV=32'h03531016; // added DUMMYVREF so VREF pins in bank3 will not be made output with high level
//  parameter MODELREV=32'h03531015; // starting 353
//  parameter MODELREV=32'h03331014; // moving to xilinx 8.2i
//  parameter MODELREV=32'h03331013; // extra latency for 16-bit data to match increased latency in gamma
//  parameter MODELREV=32'h03331012; // reading image pointer
//  parameter MODELREV=32'h03331011; // improving "gamma" tables
//  parameter MODELREV=32'h03331010; // adding support for 5MPix 12 bits Micron sensors
//  parameter MODELREV=32'h0333100f; // adding timestamps to DMA0 output
//  parameter MODELREV=32'h0333100e; // support for USB, unified I/O pins control, vector interrupts
//  parameter MODELREV=32'h0333100d; // adding real time clock for time stamping
//  parameter MODELREV=32'h0333100c; // adding real time clock for time stamping
//  parameter MODELREV=32'h0333100b; // applied all fixes from 313 branch (yellow/blue lines)
//  parameter MODELREV=32'h0333100a; // delay moved to register 0x0a
//  parameter MODELREV=32'h03331009; // making delay of the vact by specified number of lines
//  parameter MODELREV=32'h03331008; // bug fix for normal mode 
//  parameter MODELREV=32'h03331007; // adding photofinish (frame rollover for the last 4 lines) mode
//  parameter MODELREV=32'h03331006; // switched bayer_phase[1] in histogram and sensorpix,
//  parameter MODELREV=32'h03331005; // switched bayer_phase[1] in histogram and sensorpix,
                                   // now gamma and hist go in R, Gr, Gb, B order
//  parameter MODELREV=32'h03331004; // did not work, trying with old stuffer.v
//  parameter MODELREV=32'h03331003; // speeding stuffer, fixing histogram
//  parameter MODELREV=32'h03331002; // histogram
//  parameter MODELREV=32'h03331001; // restored statistics
//  parameter MODELREV=32'h0333000f; // temporarily reusing for rev B
//  parameter MODELREV=32'h0333000e; // restoring JPEG

  
    
 PULLDOWN i_PD_ALWAYS0(.O(ALWAYS0));     // Pulldown output (connect directly to top-level port)

`ifdef debug_mcontr_reset
wire [31:0] debug_mcontr_reset_data;
`endif

// internal wires
wire  iclk3;

   wire             dccout;       // enable output of DC and HF components - not used anymore //SuppressThisWarning Veditor UNUSED
   wire             dcc_enabled; //SuppressThisWarning Veditor UNUSED
   wire             dcc_rdy=0;      // Ready to read 128 DC coefficients in dcc mode. Actually they will be availabla a little later after being copied to
                                 // 8x32 output buffer


    
   wire  [31:0]   sddo_p;  // data out to SDRAM, input to pad FF
   wire  [31:0]   sddi_r;     // data from SDRAM  (1 cycle delayed)

   wire  [12:0]   sda_p;   // SDRAM address,  input to pad FF
   wire  [ 1:0]   sdba_p;  // same as above
   wire           pretrist;// tristate SDRAM data bus,  input to pad FF

   wire  [31:0]   iod;     // 32-bit data to be sent to CPU
   wire  [31:0]   idi;     // 32-bit data from CPU
///AF:     wire  [31:0]   dcr;     // 32-bit control register
   wire  [ 7:0]   as;      // internal address bus - sync to sclk0
   wire  [ 7:0]   am;      // internal address bus - multiplexed between input (for reads) and sycnc (for writes) - should not read too soon after write
   wire  [ 7:0]   ia;      // internal address bus - before latches

   wire           sclk0;    // 120MHz (same as CLK0) - SDRAM/compressor controller global clock
   wire           sclk180; // 120MHz (same as CLK0) - phase shifted 180 degrees (maybe not needed) //SuppressThisWarning Veditor UNUSED
   wire           sclk270; // 120MHz (same as CLK0) - phase shifted 270 degrees
   wire           pclk; //  sensor pixel clock
   wire           xclk; // 60MHz (=CLK0/2) - JPEG compressor clock (input stages)

   wire           sensor_en;
   wire           ihact;
   wire           iihact; //to make it possible to re-designate HACT and VACT pins
   wire           vacts;      // single-cycle frame sync @pclk
   wire           vacts_long;    // pulse from vacts active for 15 scan lines

   wire           vacts_every;          // noramlly - same as vacts, but in photo-finish mode vacts is one per N sensor vacts_every
                                        // vacts_every is filtered to prevent multiple vacts from malfunctioning sensor/sensor fpga,
                                        // allow one per trigger in triggered mode. Minimal period set to 129 pixel clocks
   wire  [15:0]   ipxd;
   wire           sens_we;
///AF:    wire  [15:0]   sens_wdl;
   wire           sens_wpage;
   wire  [ 9:0]   fpn_a;
   wire  [15:0]   fpn_d;
   wire           fpn_rpage;
   wire           blockvsync; // disable vacts_sclk
   wire           vacts_sclki;
   reg   [ 1:0]   vacts_sclko;
   wire           vacts_sclk=vacts_sclko[1]; //vacts resynchronized to a single cycle @negedge sclk
 
   wire           gamma_table_page; // 0/1 - table page used for gamma (switched at first vacts after last table write)
   wire     icnvsync;
   wire     icnvclk;


   wire pclki;
   wire sens_clk; // clock from sensor

   wire cb_pxd14;  // 1 - use 14-bit data from sensor
   wire cb_reset_mcontr; // 1-reset mcontr channels at each frame
   wire cb_break_frames; /// Enable ending frame if no more data is available (before specified number of blocks)
   wire [1:0] cb_debug;//  register hact, vact N/4 Tpclk later than data (needed for MT9P001 @ 96MHz)
   wire  cb_encnvclk; // 0 - 12 bit (5MPix), 1 - 10-bit (older sensors) with converter clock output
   wire  cb_sensor_trigger; // use trigger signal on ARO output (inverted, active low)
//   wire  cb_early_trigger; // when (cb_sensor_trigger==1) use trigger start as timestamp (to prevent jitter caused by variable exposure)
   wire  cb_external_timestamp; // use external timestamp if available
   wire  cb_output_timestamp;   // output timestamp, not just pulse
   
   wire  [1:0] cb_bayer_phase;
   wire  [2:0] cb_hfc_sel;
   wire  [1:0] cb_pclksrc; // pclk source  0 - CLK1, 1,2,3 - from sensor
   wire        cb_xt_pol; // exetrnal trigger polarity (0: 1->0, 1: 0->1)
   wire        cb_zoran;   // sensor MRST and DCLK polarity (0 - Kodak, 1 - Zoran and Micron) NEW: ONLY MRST!
   wire        cb_use_sensor_clk;   /// 1 - use sensor clock output as input FIFO DCM input, 0 - use pclk
   wire        cb_dclkmode;
   wire        cb_hact_regen; // 0 - use hact from sensor, 1 - use length written to memory controller channel 0
   wire  [7:0] cb_compressed_frames; // bitmask of frames to compress (modulo 8), defaults to 0xff
   
   wire  imrst,iarst, iaro, itrig;
   wire        ts_snap;  // make snapshot of the local_time to timestamp (@posedge pclk)
   wire [31:0] ts_sync_sec;  // timestamp, sec from camsync (local or external)   
   wire [19:0] ts_sync_usec; // timestamp, usec from camsync (local or external)
   wire        ts_stb;    // strobe when received timestamp is valid - single negedge sclk cycle
   
   wire        vact_overdue; // generated at camsync to bypass filtering out second vact after trigger. Needed to prevent lock-up
                             // when exposure > triger period (and trigger is operated as divide-by-2)

     wire        idreq0;
     wire        idack0;
     wire        idreq1;
     wire        idack1;

   wire           da_ctl_24l, da_ctl_8h;     // WE to control 32-bit register (1 loc) *********** obsolete ************** //SuppressThisWarning Veditor UNUSED
   wire           da_dmamode;       // select writing to dma_cntr/dma_raw (1 loc)
   wire           da_sensormode;    // select writing to sensorpix (1 loc)
   wire           da_virttrig;      // write virtual trigger threshold
   wire           da_sensortrig;    // sensor control: bit0 continuous, bit1 - external, bit2 - enable
   wire           da_sensortrig_lines; // write number of lines to be transferred in a frame (or aftre trigger)
   wire           da_dswe;          // select reading/writing to mcontr (16 loc)
   wire           da_mem;           // read/write to SDRAM buffer through 128 32-bit-word window (==a[7])
   wire           da_lensff;        // lens flat field correction parameters (1 location, 8 bit address, 16bit - data)
   
   wire           da_sens_dc;       // write clock divisor for sensor DCDC converter
   wire           da_interrupts;    // new value (1c..1f)
   wire           da_compressor;
   wire           da_dcm;
   wire           da_saturation;    // write color saturation vaues (currently 10 bits
   wire           da_hist;          // write/read histogram related data/registers
   wire           da_hist_next;     // increment histogram address
   wire           da_framesync_dly; //write frame sync delay (in scan lines)
   wire           da_quantizer_mode;  // Quantizer tuning - 0..7 - zero bin, 15:8 - quantizer bias
   wire           da_rtc;
   wire           da_timestamp;
   wire           da_io_pins;       // write control data to i/o pins (multiplexer) 0x70
   wire           da_pio_dmafifo;          // increment address of the DMA FIFO(S) (PIO mode should be enabled in the desired channel)
   wire           da_xjtag;
   wire           da_extsync;         // control of external sync module 0x78 - 0x7b
   wire           da_extio;           // external I/O (motor control 0x7c-0x7d)
   wire           da_imu;             // IMU I/O (0x7e-0x7f)
//   wire           da_imu_read;        // read any of the IMU data/status (updates memory output reg)
//   wire           da_imu_next;        // read IMU dataread from pointer, increment address
   wire           da_i2c;             // i2c control  0x50 - 0x53
   wire           da_irq_smart;       // single IRQ control (0x1a)
   wire           da_sequencer;
   wire           da_dcr;             // write ro control registers (each group is enabled by data bit), instead of the da_ctl_24l, da_ctl_8h
   wire           smart_irq;          // irq that combines frame sync and compressor done
   wire   [19:0]  pio_usec;
   wire   [31:0]  pio_sec;
   wire   [19:0]  pusec;
   wire   [31:0]  psec;
   
   wire           ihact_ts;    //  combined with timestamp in photo-finish mode (delayed by 1 pclk)
   wire  [15:0]   ipxd_ts; // pixel data combined with time stamp data (delayed by 1 pclk)
   wire           platch; // sync to pclk start of first hact after vact - copy timer data


   wire           senspgmin;   // state of the SENSPGM I/O pin (read)
   wire           xfpgadone;   // state of the MRST pin ("DONE" pin on an external FPGA)
   wire           xfpgatdo;    // TDO read from an external FPGA
   wire           xfpgastat;   // Multiplexed read data from xjtag (senspgmin/xfpgadone/xfpgatdo)

   wire           xpgmen;      // enable programming mode for an external FPGA
   wire           xfpgaprog;   // PROG_B to be sent to an external FPGA
   wire           xfpgatck;    // TCK to be sent to an external FPGA
   wire           xfpgatms;    // TMS to be sent to an external FPGA
   wire           xfpgatdi;    // TDI to be sent to an external FPGA


   wire  [31:0]   irqr;// [31:0] data out (bits {15:0} - masked interrupts, [31:0] - same before mask
   wire           iirq;


   wire  [31:0]   dma_d0;
   wire  [31:0]   dma_d1;

   wire  [ 9:0]   sens_a;     // [7:0] channel 0 address (MSB - block #)
   wire  [15:0]   sens_d;     // [15:0] channel 0 data in

   wire  trigger_single_pclk; // start of trigger //SuppressThisWarning Veditor UNUSED

   wire  [31:0]   bdo;     // 32-bit data from SDRAM channel3
   wire  [31:0]   dsdo;    // 32-bit data from SDRAM descriptor memory
   wire  [10:0]   ch2a;    // SDRAM channel 2 buffer address
   wire  [ 7:0]   ch2do;   // SDRAM channel 2 data out

   wire  [31:0]   hist_do; // histogram data out, actully only [17:0];

   wire  sr_sda0, sr_scl0;
///AF:     wire  sr_sda1, sr_scl1;
   wire  sr_memrdy,sr_ch0rdy,sr_ch1rdy,sr_ch2rdy,sr_wrempty;
   wire  [3:0] sr_nextFrame;  //[3:0]
   wire  [3:0] chInitOnehot; // decoded channel init pulses, 2 cycles behind chInit
   
   wire  [2:0] sr_sensortrig; //sr_sensortrig[1:0]: 00 - idle, 01 - waiting vacts to start, 10 - waiting for trigger, 11 - frame over
                              //sr_sensortrig[2] - frame done (reset by writing command to sensor_trig
   wire [13:0] trig_v;        // number of scanline when the trigger was got
   wire [13:0] trig_h;        // number of pixel in a line when the trigger was got
   wire        xfer_over_irq; // single pixel clock "end of frame transfer" interrupt
   wire        compressor_eot;// compressor read in the last MCU (predictable time to end of transfer
// wire        compressor_overrun; // will persist untill dct disabled
   wire        compressor_done_input;     // will go high after EOT and persist until DCT is enabled
   wire        compressor_done_compress;  // will go high after all data is sent out
   wire        compressor_started; // single @ negedge sclk0 pulse when compressor is started (each frame in continuous mode)
   wire        is_compressing;    // @posedge clk, from go_single to eot (actual or simulated)
                                  // does not inclusde flushing

   wire        compressor_done_pulse; // does not need to be reset, single cycle (sclk)
   wire        dma_empty;       // dma output buffer empty and dma is enabled (pessimistic - with delay)
///AF:    wire        dma_empty0,dma_empty1;       // dma output buffer empty and dma is enabled (pessimistic - with delay)

   wire           virt_trig;
   wire           virt_sel;  //SuppressThisWarning Veditor UNUSED

   wire        ch2_en_rd_buff;

   wire dcm_rst, dcm_incdec, dcm_en, dcm_clk, dcm_done; // TODO: make module to controll these signal by software (and measure read phase)
   wire [1:0] dcm_err;
   wire       udqsr90, ldqsr90, udqsr270, ldqsr270;
   wire [1:0] sddm_p; // write mask (sync with data)
   wire       sddqt;
   wire       wnr; //write/not read - sync to sclk

  wire [31:0] rd_regs; // read 0x10-0x14


   wire [11:0]  ta; //table address - will be valid at twr_xx and one cycle beforfe (@ negedge clk)
//   wire         twce;
   wire twr_quant;  // write enable to quantization table (@negedge clk - addr and data valid this cycle and one before)
   wire twr_coring; // coring functions tables (@negedge clk - addr and data valid this cycle and one before)

   
   wire twr_huff;   // write enable to huffman table (@negedge clk - addr and data valid this cycle and one before)
   wire twr_gamma;  // write enable to "gamma" table (@negedge clk - addr and data valid this cycle and one before)
   wire twr_focus;  // write enable to "focus" table (coefficients to multiply DCT results before accumulating squares)

   wire        statistics_dv;      // image statistics word valid (sync to clk) //SuppressThisWarning Veditor UNUSED
   wire [15:0] statistics_do;      // 16-bit image statistics data to write //SuppressThisWarning Veditor UNUSED
   wire        line_run;
   wire        frame_run;          // SuppressThisWarning Veditor UNUSED

// synthesis attribute keep of rd_regs is true;

 wire [15:0]   stuffer_do;
 wire          stuffer_dv;


///AF: wire  [3:0] test_dma_rcntr;
wire  [3:0] test_dma_wcntr; // SuppressThisWarning Veditor UNUSED
wire  [3:0] test_fifo;      // SuppressThisWarning Veditor UNUSED

///AF: wire  [3:0] test_dma1_rcntr;
wire  [7:0] test_dma1_wcntr; // SuppressThisWarning Veditor UNUSED
wire  [7:0] test_fifo1;


wire [11:0]  io_pins;  // status of 12 i/o pins (from J2 connector)
wire [11:0]  io_do;    // data to i/o pins
wire [11:0]  io_t;     // tristate i/o pins
wire [11:0]  io_da;    // data to i/o pins from channel A (camsync)
wire [11:0]  io_da_en; // i/o pins data out enable from channel A (camsync)
wire [11:0]  io_db;    // data to i/o pins from channel B (TBD)
wire [11:0]  io_db_en; // i/o pins data out enable from channel B (TBD)
wire [11:0]  io_dc;    // data to i/o pins from channel C (TBD)
wire [11:0]  io_dc_en; // i/o pins data out enable from channel C (TBD)


// outputs from i2c_writeonly controller, that works in parallel with the software one
wire         i2c_busy;   // busy (do not use software i2i)
wire         i2c_scl;    // i2c SCL
wire         i2c_sda;    // i2c SDA
wire         i2c_scl_en;     // switch i2c control to i2c_writeonly (from software direct bit control PIO)
wire         i2c_sda_en; // enable SDA output
wire  [2:0]  i2c_frame_no; // i2c frame number (modulo 8)

//wire  [7:0]  sequencer_condition;
wire         sequencer_rq;
wire         sequencer_ack;
wire  [7:0]  sequencer_a;
wire [23:0]  sequencer_d;
wire  [2:0]  sequencer_frame_no; // i2c frame number (modulo 8)


wire [15:0] irq_in;    // interrupt input (posedge sensitive, maybe async)
wire [23:0] imgptr;    // running accumulated image size in 32-byte chunks. Cleared when compressor is stopped,
                       // updated when compression of the frame is over.
wire [31:0] hifreq;    // accumulated high-freq components

//wire iclk0; // before BUFG


wire       adcmrst ;//async DCM reset
wire       clockios_locked;
wire [7:0] clockios_status; // bit 1
wire       dcm_locked;
wire [7:0] dcm_status;
wire       sens_dcm_locked;
wire       sens_dcm_done;
wire [7:0] sens_dcm_status;
wire [1:0] sens_ph_err = 0; // currently not assigned (removed module) 

wire       ioe;//  internal (not global) version of OE (for DMA)
wire [1:0] phsel; // additional phase select for SDRAM clock

wire       pclk2x; // twice frequency of the pclk, global
wire [1:0] sensordat_pherr; // sync to posedge pclk, PXD[2] differs with 1/4 early, 1/4 late //SuppressThisWarning Veditor UNUSED

wire       sensor_trigger; // signal to start CMOS sensor in sync mode

wire       confirmFrame2Compressor; // pulse to start reading a new frame to buffer for compressor (generated at start of each frame by the compressor)
                                    // mcontr will stop to read to channel FIFO at the end of frame, wait for confirmation
`ifdef debug_compressor
    wire [31:0] printk_compressor; //SuppressThisWarning Veditor UNUSED
`endif
//ia

`ifdef debug_stuffer
    wire [7:0]  testwire;
    wire [3:0]  tst_stuf_etrax;
    reg  [3:0]  tst_cmd_cntr;
    reg         tst_rdy_after_eot;
///AF:    wire dma0_enabled; // just for debug
///AF:    wire dma1_enabled; // just for debug
`endif
`ifdef debug_dma_count
    wire [31:0] printk;
`endif
// Needed anyway:
    wire dma0_enabled; // just for debug
    wire dma1_enabled; // just for debug

/*
//xfer_over_irq
reg [3:0] test_11;
always @ (posedge pclk) if (xfer_over_irq) begin
  test_11[3:0]<=test_11[3:0]+1;
end
*/
wire    [3:0] nextBlocksEn; // When combined with SDRAM data ready, may be used to end frame compression input (instead of block counter)

wire    [3:0] restart; // reinitialize mcontr channels (normally after frame sync, if enabled)

// Wires missing in the original 353 design
    wire    sdwe_p;          // WE  command bit to SDRAM (1 cycle ahead)
    wire    sdras_p;         // CAS command bit to SDRAM (1 cycle ahead)
    wire    sdcas_p;         // CAS command bit to SDRAM
    wire    sd_dqsrd;        // enable read from DQS i/o-s for phase adjustments  (latency 2 from the SDRAM RD command)
    wire    trig_irq;        // single-cycle (pclk) pulse at external interrupt
    wire    da_init_ch3;     // write to init cnhannel 3 (will reset address r/w)
    wire    da_next_ch3;     // advance to the next channel3 page, reset address
    wire    stch2;           // start channel 2 transfer


`ifdef DEBUG_IMU// debug IMU
    wire imu_enabled_mclk;
    wire imu_run_mclk;
    wire [31:0] imu_period;
`endif    





  assign rd_regs=
                    ia[2]? 
//                   (ia[1]?{14'h0,gamma_table_page,i2c_busy,13'h0, i2c_frame_no[2:0]}: // 0x16, 0x17
                     (ia[1]?{14'h0,gamma_table_page,i2c_busy,5'h0,sequencer_frame_no[2:0],5'h0,i2c_frame_no[2:0]}: // 0x16, 0x17
                        (ia[0]?hifreq[31:0]:{8'h0,imgptr[23:0]})):     // 0x15, 0x14
                     (ia[1]?
                       (ia[0] ?
                        (MODELREV   //                            // 13 - model/revision ?
                         ):
                        {2'b0,trig_v[13:0],2'b0,trig_h[13:0]}     // 12 - trigger phase (row/column of readout when the trigger occured)
                      ):
                       (ia[0] ?
                        irqr[31:0]:                        // 11 - interrupts
                         {                                  // 10 - status register
                          1'b0,
                          
                          clockios_status[1:0],
                          clockios_locked,
                          
                          sens_dcm_status[0],                    // Sensor DCM overflow
                          sens_dcm_locked,                       // Sensor DCM overflow
                          sens_dcm_done,                         // Sensor DCM cmd (inc/dec/reset) done
                          sens_ph_err[1:0],                     // Sensor DCM phase error 

                          dcm_status[0],                    // SDRAM DCM overflow
                          dcm_locked,                       // SDRAM DCM overflow
                          dcm_done,                         // SDRAM DCM cmd (inc/dec) done
                          dcm_err[1:0],                     // SDRAM DCM phase error 
                          dma_empty,                        // dma output buffer empty and dma is enabled (pessimistic - with delay)
                          compressor_done_compress,         // will go high after all data is sent out
                          compressor_done_input,            // will go high after EOT and persist until DCT is enabled
                          dcc_rdy,                          // 128 of DC components from JPEG encoder are available (same as interrupt)
                          sr_sensortrig[2:0],               // [15:13]
//                          test_11[3:0],
                          sr_nextFrame[3:0],                // [12:9]
                          sr_wrempty,                       // [8] CPU write buffer empty (valid only in write mode). May reprogram channel3
                          sr_memrdy,                        // [7] ready to r/w next 8x32 page (channel 3) CPU<->SDRAM
                          sr_ch2rdy,                        // [6] ready to read next page  (channel 2) SDRAM->DMA
                          sr_ch1rdy,                        // [5] ready to read next page  (channel 1) SDRAM->FPN(SENSOR)
                          sr_ch0rdy,                        // [4] ready to write next page (channel 0) SENSOR->SDRAM
                          sr_scl0,sr_sda0                   // I2C to sensor
                         }
                        ));
                   
//         ,.test1( test_fifo[3:0])
  //       ,.test2( test_dma_wcntr[3:0])
//   wire [31:0] imu_di; // External data in - currently from the motor control;
//   wire        imu_ready; // External data in - currently from the motor control;
   wire [15:0] imu_dma_data; // data to DMA1 fifo (@negedge clk) 
   wire        imu_dma_stb;  // data strobe to DMA1 fifo (@negedge clk)
   wire [23:0] imu_sample_counter; // number of 64-byte samples written
   wire [31:0] imu_debug_state;    // watching how far serial/nmea processing got
   
   wire [31:0] motor_di; //  motor control;
   assign iod[31:0] =(ia[6])?
                 ((ia[5])?
                   ((ia[4])?
                     ((ia[3] & ia[2])?
//                        (ia[1]?(ia[0]?{15'h0,imu_ready}:imu_di):motor_di[31:0]):
//                      (ia[1]?(ia[0]?{13'h0,imu_run_mclk,imu_enabled_mclk,imu_ready}:imu_di):(ia[0]?imu_period[31:0]:motor_di[31:0])):
//                        (ia[1]?(ia[0]?32'h0:{8'h0,imu_sample_counter[23:0]}):(ia[0]?32'h0:motor_di[31:0])):
                        (ia[1]?(ia[0]?imu_debug_state[31:0]:{8'h0,imu_sample_counter[23:0]}):(ia[0]?32'h0:motor_di[31:0])):
//imu_debug_state[15:0]                        
//wire imu_enabled_mclk;
//wire imu_run_mclk;
//wire [31:0] imu_period;
                        
                        {14'b0,xfpgatdo,xfpgastat,4'b0,io_pins[11:0]}):   // 0x70..0x7f 
`ifdef debug_stuffer
                       {1'b0,tst_rdy_after_eot,sr_ch2rdy,printk[8:4],tst_stuf_etrax[3:0],test_dma_wcntr[3:0],testwire[7:4],testwire[3:0],tst_cmd_cntr[3:0],printk[3:0]}
`else
  `ifdef debug_dma_count
//  assign printk={test_dma_wcntr[3:0],test_fifo[3:0],1'b0,compressor_test_state[2:0],debug_cntr_stuffer_dv[19:0]};
       `ifdef debug_mcontr_reset
                  (ia[3]?
                  {debug_mcontr_reset_data[31:0]}:
                  {printk[31:0]} /// {is_compressing,cmprs_repeat,cmprs_en}
                  )
       `else
                        {printk[31:0]}
       `endif
  `else
     `ifdef debug_compressor
//                           {printk_compressor[31:0]}
                           {printk_compressor[31:16],compressor_test_state[2:0],printk_compressor[12:0]} /// {is_compressing,cmprs_repeat,cmprs_en}
     `else
                           32'h0
     `endif
  `endif

`endif

                   ):
                   ((ia[4])?
                     32'h0:
                     (ia[3]?
                       (ia[0]?
                         pio_sec[31:0]:
                         {12'h0,pio_usec[19:0]}
                       ):
                       hist_do[31:0]
                     )
                   )
                 ):
                 (ia[5]?
                  (ia[4]?
                   (bdo[31:0]):     // 3x
                   ({dsdo[31:0]})): // 2x
                  (ia[4]? 
                   rd_regs[31:0]:
                   (ia[0]? 
                     dma_d1[31:0]:  //01/03/05/07/09/0b/0d/0f
                     dma_d0[31:0]))); //00/02/04/06/08/0a/0c/0e

// will change bits later (with software) to separate MRST and CLK sensor polarity
// IO pads and related FFs
     sddrio16 #(
                  .IOSTANDARD (IOSTANDARD_SDRAM),
                  .DRIVE      (DRIVE_SDRAM_DATA),
                  .SLEW       (SLEW_SDRAM_DATA))
            i_SDDd   (.c0(sclk0),.c270(sclk270),
                      .d(sddo_p[31:0]),.t(pretrist),
                      .q(sddi_r[31:0]),.dq(SDD[15:0]));
     sddrdm #(
                  .IOSTANDARD (IOSTANDARD_SDRAM),
                  .DRIVE      (DRIVE_SDRAM_DATA),
                  .SLEW       (SLEW_SDRAM_DATA))
                    i_SDUDM  (.c0(sclk0),.c270(sclk270),.d(sddm_p[1:0]),.dq(SDUDM));
     sddrdm  #(
                  .IOSTANDARD (IOSTANDARD_SDRAM),
                  .DRIVE      (DRIVE_SDRAM_DATA),
                  .SLEW       (SLEW_SDRAM_DATA))
                   i_SDLDM  (.c0(sclk0),.c270(sclk270),.d(sddm_p[1:0]),.dq(SDLDM));
     sdo15_2 #(
                  .IOSTANDARD (IOSTANDARD_SDRAM),
                  .DRIVE      (DRIVE_SDRAM_ABC),
                  .SLEW       (SLEW_SDRAM_ABC))
            i_SDA    (.c(sclk0),.d({sdba_p[1:0],sda_p[12:0]}),.q(SDA[14:0]));
     sdo1_2 #(
                  .IOSTANDARD (IOSTANDARD_SDRAM),
                  .DRIVE      (DRIVE_SDRAM_ABC),
                  .SLEW       (SLEW_SDRAM_ABC))
            i_SDRAS  (.c(sclk0),.d(sdras_p),.q(SDRAS));
     sdo1_2 #(
                  .IOSTANDARD (IOSTANDARD_SDRAM),
                  .DRIVE      (DRIVE_SDRAM_ABC),
                  .SLEW       (SLEW_SDRAM_ABC))
            i_SDCAS  (.c(sclk0),.d(sdcas_p),.q(SDCAS));
     sdo1_2 #(
                  .IOSTANDARD (IOSTANDARD_SDRAM),
                  .DRIVE      (DRIVE_SDRAM_ABC),
                  .SLEW       (SLEW_SDRAM_ABC))
            i_SDWE   (.c(sclk0),.d(sdwe_p),. q(SDWE ));

     sdo1_2 #(
                  .IOSTANDARD (IOSTANDARD_SDRAM),
                  .DRIVE      (DRIVE_SDRAM_ABC),
                  .SLEW       (SLEW_SDRAM_ABC))
            i_SDCLKE (.c(sclk0),.d(1'b1),. q(SDCLKE ));

// temporary change behaviour of dqs2 to fix pinout problem - will influence adjustment goal
     dqs2 #(.IOSTANDARD(IOSTANDARD_SDRAM),
                  .DRIVE      (DRIVE_SDRAM_DATA),
                  .SLEW       (SLEW_SDRAM_DATA))
              i_sddqs(.c0(sclk0), .c270(sclk270),
                  .t       (sddqt),       // 1/2 cycle before cmd "write" sent out to the SDRAM, sync to sclk180
                  .UDQS    (UDQS),         // UDQS I/O pin
                  .LDQS    (LDQS),         // LDQS I/O pin
                  .udqsr90 (udqsr90),      // data from SDRAM interface pin UDQS strobed at rising sclk90
                  .ldqsr90 (ldqsr90),      // data from SDRAM interface pin LDQS strobed at rising sclk90
                  .udqsr270(udqsr270),     // data from SDRAM interface pin UDQS strobed at rising sclk270
                  .ldqsr270(ldqsr270)      // data from SDRAM interface pin UDQS strobed at rising sclk270
                 );
                                  
                 
//synthesis translate_off
// These keepers are not really needed - just to make simulator happy

     KEEPER i_UDQS_KEEPER ( .O(UDQS) );
     KEEPER i_LDQS_KEEPER ( .O(LDQS) );
//synthesis translate_on


// syncronized wnr, as[7:0] and idi[31:0] will be valid for 2 cycles - during da_* and the next one. So it will be possible to delay actions
// if too time-limited.
wire drv_bus=1'b0;// drive system bus (to write to system memory)
wire nevr; // never true;
wire isys_sdclki, ibrin;
//    assign BROUT=1'b0;
    OBUF  #(
        .IOSTANDARD   (IOSTANDARD_SYS),
        .DRIVE        (DRIVE_SYS_DREQ),
        .SLEW         (SLEW_SYS_DREQ))
           i_BROUT (
             .I(1'b0),
             .O(BROUT));
//    assign SYS_BUSEN=1'b1;
    OBUF  #(
        .IOSTANDARD  (IOSTANDARD_SYS),
        .DRIVE       (DRIVE_SYS_DREQ),
        .SLEW        (SLEW_SYS_DREQ))
           i_SYS_BUSEN (
             .I(1'b1),
             .O(SYS_BUSEN));

   IBUF #(
        .IOSTANDARD       (IOSTANDARD_SYS),
        .IBUF_DELAY_VALUE (IBUF_DELAY_SYS_SDCLK),
        .IFD_DELAY_VALUE  (IFD_DELAY_SYS_SDCLK))
        i_SYS_SDCLKI(
            .I  (SYS_SDCLKI),
            .O  (isys_sdclki));
   IBUF #(
        .IOSTANDARD       (IOSTANDARD_SYS),
        .IBUF_DELAY_VALUE (IBUF_DELAY_SYS_DACK),
        .IFD_DELAY_VALUE  (IFD_DELAY_SYS_DACK))
             i_BRIN      (.I(BRIN),        .O(ibrin));

wire idummyvref;    
  IOBUF #(.IOSTANDARD(IOSTANDARD_SDRAM)) i_dummyvref (.I(1'b0), .T(nevr), .O(idummyvref), .IO(DUMMYVFEF));
  IBUF  #(.IOSTANDARD(IOSTANDARD_SENSOR)) i_always0   (.I(ALWAYS0),.O(nevr)); // same bank as sesnor, 2.5V

// That will keep isys_sdclki && ibrin && isenspgm && idummyvref && sdcl_fb from being optimized into oblivion
wire sdcl_fb;

wire never=nevr && isys_sdclki && ibrin && idummyvref && sdcl_fb;
  IOBUF #(.IOSTANDARD(IOSTANDARD_SYS)) i_BA0      (.I(1'b0), .T(!never),.O(), .IO(BA[0]));
  IOBUF #(.IOSTANDARD(IOSTANDARD_SYS)) i_BA1      (.I(1'b0), .T(!never),.O(), .IO(BA[1]));

  IOBUF #(.IOSTANDARD(IOSTANDARD_SYS)) i_SYS_SDWE  (.I(1'b1), .T(!never),.O(), .IO(SYS_SDWE));
  IOBUF #(.IOSTANDARD(IOSTANDARD_SYS)) i_SYS_SDCAS (.I(1'b1), .T(!never),.O(), .IO(SYS_SDCAS));
  IOBUF #(.IOSTANDARD(IOSTANDARD_SYS)) i_SYS_SDRAS (.I(1'b1), .T(!never),.O(), .IO(SYS_SDRAS));
  IOBUF #(.IOSTANDARD(IOSTANDARD_SYS)) i_SYS_SDCLK (.I(1'b0), .T(!never),.O(), .IO(SYS_SDCLK));
  IOBUF #(.IOSTANDARD(IOSTANDARD_SYS)) i_BG        (.I(1'b1), .T(!never),.O(), .IO(BG));

// synchronize vacts to negedge of the system clock (sclk) -> vacts_sclk
FDCE i_vacts_sclki(.C(pclk),.CE(vacts),.CLR(vacts_sclko[1]),.D(1'b1), .Q(vacts_sclki));
///blockvsync disables vacts_sclko[1] to prevent sequencers (i2c, cmd) from running during reset release
always @ (negedge sclk0) vacts_sclko[1:0] <= {vacts_sclko[0] & ~blockvsync, vacts_sclki && ! vacts_sclko[0] && !vacts_sclko[1]};


sysinterface#(
        .IOSTANDARD_SYS     (IOSTANDARD_SYS),
        .SLEW_SYS           (SLEW_SYS),
        .DRIVE_SYS          (DRIVE_SYS),
        .IBUF_DELAY_SYS_A   (IBUF_DELAY_SYS_A),
        .IFD_DELAY_SYS_A    (IFD_DELAY_SYS_A),
        .IBUF_DELAY_SYS_D   (IBUF_DELAY_SYS_D),
        .IFD_DELAY_SYS_D    (IFD_DELAY_SYS_D),
        .IBUF_DELAY_SYS_WOE (IBUF_DELAY_SYS_WOE),
        .IFD_DELAY_SYS_WOE  (IFD_DELAY_SYS_WOE),
        .IBUF_DELAY_SYS_CE  (IBUF_DELAY_SYS_CE),
        .IFD_DELAY_SYS_CE   (IFD_DELAY_SYS_CE)
    ) i_sysinterface(.clk(sclk0),
                            .drv_bus(drv_bus),                        // drive system bus (to write to system memory)
                            .d(D[31:0]),                              // 32 bit D[31:0] data pads
                            .oe(OE),                                  // OE pad
                            .ce(CE),                                  // CE pad (zero wait states)
                            .ce1(CE1),                                // CE1 pad (EW=1)
                            .we(WE),                                  // WE pad
                            .a(A[12:0]),                              // 13 bit A[12:0]
                            .iod(iod[31:0]),                          // internal 32-bit data out (FPGA->CPU) bus
                            .idi(idi[31:0]),                          // internal 32-bit data in, synchronous to clk
                            .ia(ia[7:0]),                             // internal 8-bit address bus (fast - directly from I/O pads)
                            .as(as[7:0]),                             // internal 8-bit address bus (synchronized to clk)
                            .am(am[7:0]),                             // multiplexed addr - switching between ia and as

                            .wnr(wnr),                                // write/not read, sync with clk
                            .da_ctl  (da_ctl_24l),                    // WE to control 32-bit register (1 loc)
                            .da_ctl_h(da_ctl_8h),                     // WE to control 32-bit register (1 loc)
                            .da_dmamode(da_dmamode),                  // select writing to dma_cntr/dma_raw (1 loc)
                            .da_sensormode(da_sensormode),            // select writing to sensorpix (1 loc)
                            .da_virttrig(da_virttrig),                // write virtual trigger threshold
                            .da_sensortrig(da_sensortrig),            // sensor control: bit0 continuous, bit1 - external, bit2 - enable
                            .da_sensortrig_lines(da_sensortrig_lines),// write number of lines to be transferred in a frame (or after trigger)
                            .da_dswe(da_dswe),                        // select reading/writing to mcontr (16 locations)
                            .da_init_ch3(da_init_ch3),                // write to init cnhannel 3 (will reset address r/w)
                            .da_next_ch3(da_next_ch3),                // advance to the next channel3 page, reset address
                            .da_mem(da_mem),                          // read/write to SDRAM buffer, autoincrement address
                                                                      // in read mode - needs CE1 to autoincrement!
                            .da_lensff(da_lensff),                    // lens flat field correction parameters (1 location, 8 bit address, 16bit - data)
                            .da_hist(da_hist),                        // 0x40..0x47  write/read histogram related data/registers
                                                                      // 40 - left
                                                                      // 41 - top
                                                                      // 42 - width-1
                                                                      // 43 - height-1
                                                                      // 44 - hist. data start address (will also read pointed word to output word
                            .da_hist_next(da_hist_next),              // 45 - read histogram (with CE1 to autoincrement)
                            .da_rtc(da_rtc),                          // 48 - write microseconds (actual write will happen after writing seconds)
                                                                      // 49 - write seconds
                                                                      // 4a - write correction (16 bit, signed
                                                                      // 4b - nop, just latch the output 32+20 bits to read to CPU). Maybe nop (or 2 writes) are needed between read.
                            .da_timestamp(da_timestamp),              // 4c - write timesatmp mode (0 - off, 1 - normal frames, 1 photo-finish)
                            .da_sens_dc(da_sens_dc),                  // write clock divisor for sensor DCDC converter
                            .da_interrupts(da_interrupts),            // interrupt control 0x1c..0x1f
                            .da_compressor(da_compressor),            // 0x0c - 0x0f - will be farther decoded in the compressor module
                            .da_dcm(da_dcm),                          // tune SDRAM clock
                            .da_saturation(da_saturation),            // write color saturation vaues (currently 10 bits
                            .da_framesync_dly(da_framesync_dly),      // delay frame sync interrupt by number of lines (0x0a)
                            .da_quantizer_mode(da_quantizer_mode),    // Quantizer tuning - 0..7 - zero bin, 15:8 - quantizer bias
                            .da_io_pins(da_io_pins),                  // write i/o pins control (for 6 pins to connector J2) - 0x70
                            .da_pio_dmafifo(da_pio_dmafifo),          // increment address of the DMA FIFO(S) (PIO mode should be enabled in the desired channel)
                            .da_xjtag(da_xjtag),                      // write to external (sensor board) JTAG
                            .da_extsync(da_extsync),                  // control of external sync module 0x78 - 0x7b
									 .da_extio(da_extio),                      // external I/O (motor control 0x7c-0x7d)
									 .da_imu(da_imu),                          // imu I/O (0x7e-0x7f)
//                            .da_imu_read(da_imu_read),        // read any of the IMU data/status (updates memory output reg)
//                            .da_imu_next(da_imu_next),        // read IMU dataread from pointer, increment address
                            .da_i2c(da_i2c),                          // i2c control 0x50 - 0x5f
                            .da_irq_smart(da_irq_smart),              // single IRQ control (0x1a)
                            .da_sequencer(da_sequencer),              // command sequencer (0x60..0x6f)
                            .da_dcr(da_dcr),                          // write to control registers (0x4e..0x4f), each bit/group with individual enable by data bit
                            .ta(ta[11:0]),                            // [11:0] table address - will be valid at twr_xx and one cycle beforfe (@ negedge clk)
                            .twr_quant(twr_quant),                    // write enable to quantization table (@negedge clk - addr and data valid this cycle and one before)
                            .twr_coring(twr_coring),                  // coring functions tables (@negedge clk - addr and data valid this cycle and one before)
                            .twr_huff(twr_huff),                      // write enable to huffman table (@negedge clk - addr and data valid this cycle and one before)
                            .twr_gamma(twr_gamma),                    // write enable to "gamma" table (@negedge clk - addr and data valid this cycle and one before)
                            .twr_focus(twr_focus),                    // write enable to "focus" table (@negedge clk - addr and data valid this cycle and one before)

                            .dcmrst(adcmrst),                         // (emergency)async DCM reset Seems SE hangs if the frequency changed on the fly
                            .ioe(ioe),
                            .seq_rq(sequencer_rq),                    // request from the sequencer
                            .seq_ack(sequencer_ack),                  // sequencer acknowledge
                            .seq_a(sequencer_a[7:0]),                 // address from the sequencer
                            .seq_d(sequencer_d[23:0])                 // data from the sequencer
                            );

extjtag i_extjtag       (.sclk(sclk0),           // @negedge
                         .pre_wen(da_xjtag),     // 1 cycle ahead of write data
                         .di(idi[31:0]),         // [31:0] data in (only some bits are used)
                         .xpgmen(xpgmen),        // enable programming mode for an external FPGA
                         .xfpgaprog(xfpgaprog),  // PROG_B to be sent to an external FPGA
                         .xfpgatck(xfpgatck),    // TCK to be sent to an external FPGA
                         .xfpgatms(xfpgatms),    // TMS to be sent to an external FPGA
                         .xfpgatdi(xfpgatdi),    // TDI to be sent to an external FPGA
                         
                         .senspgmin(senspgmin),  // state of the SENSPGM I/O pin (read)
                         .xfpgadone(xfpgadone),  // state of the MRST pin ("DONE" pin on an external FPGA)
                         .xfpgatdo(xfpgatdo),    // TDO read from an external FPGA

                         .state(xfpgastat));       // multiplexed state (one of 3 inputs)
wire [19:0] running_usec; // IMU microseconds
wire [31:0] running_sec; // IMU seconds

rtc353 i_rtc353            (.mclk(sclk0),           // system clock (negedge)
                            .pre_we(da_rtc),        // 1 cycle ahead of writing data
                            .wa(as[1:0]),           // [1:0] write address
                                                    // 0 - write microseconds (actual write will happen after writing seconds)
                                                    // 1 - write seconds
                                                    // 2 - write correction (16 bit, signed
                                                    // 3 - nop, just latch the output 32+20 bits to read to CPU). Maybe nop (or 2 writes) are needed between read.
                            .wd(idi[15:0]),         // [31:0] data to write, valid 1 cycle after pre_we, wa
                            .musec(pio_usec[19:0]), // [19:0] usec output (register is latched by writing), 0..999999 (0..0xf423f)
                            .msec (pio_sec[31:0]),  // [31:0] seconds latched counter output
                            .clk12(iclk3),          // 12MHz clock (no PLL, just xtall). will be re-synchronized to mclk to avoid extra GCLK
                            .pclk(pclk),            // sensor clock (posedge)
                            .platch(cb_sensor_trigger?ts_snap:platch),  // latch counters sync to pclk
                            .pusec(pusec[19:0]),    // [19:0] usec output - sync to sensor clock
                            .psec(psec[31:0]),      // [31:0] seconds counter output
                            .usec(running_usec[19:0]), // [19:0] running usec output
                            .sec(running_sec[31:0]));  //[31:0] running seconds counter output


timestamp353 i_timestamp353(.mclk(sclk0),    // system clock (negedge)
                            .pre_we(da_timestamp),  // 1 cycle ahead of writing data
                            .wd(idi[1:0]),      // [31:0] data to write, valid 1 cycle after pre_we, wa
                            .pclk(pclk),  // pixel clock
                            .pxdi(ipxd[15:0]),  // [9:0] pixel data from sensor
                            .pxdo(ipxd_ts[15:0]),  // [9:0] data to replace pxdi (next cycle)
                            .vacts(vacts_every), // vertical sync (actual sensor)
                            .hacti(ihact), // hact input
                            .hacto(ihact_ts), // hact output (next cycle)
                            .sec(ts_sync_sec[31:0]),    // [31:0] number of seconds
                            .usec(ts_sync_usec[19:0]),    // [19:0] number of microseconds
                            .tlatch(platch)); // latch time - 1-st hact after vact

// not used!
//SDCLK_FB  // feedback input from SDCLK pin
//SDNCLK_FB // feedback input from SDCLK pin
//     IBUFDS        i_sdcl_fb(.O(sdcl_fb),.I(SDCLK_FB),.IB(SDNCLK_FB)); // not used
     IBUFDS #(.IOSTANDARD(IOSTANDARD_SDRAM_DIFF))  i_sdcl_fb(.O(sdcl_fb),.I(SDNCLK_FB),.IB(SDCLK_FB)); // not used
// synthesis attribute KEEP of i_sdcl_fb  is "TRUE"

// assumed CL=2.5
 sdram_phase i_sdram_phase(
//***********************  .wclk(cwr),       // global CPU WE pulse
                           .pre_wcmd(da_dcm),       // decoded address - enables wclk
                           .wd(idi[3:0]),         // CPU write data [3:0]
                                          //       0 - nop, just reset status data
                                          //       1 - increase phase shift
                                          //       2 - decrease phase shift
                                          //       3 - reset phase shift to default (preprogrammed in FPGA configuration)
                           .ph_err(dcm_err[1:0]),     // [1:0] 0 - no data (SDRAM reads) since last change (wclk*wcmd)
                                       //       1 - clock is too late
                                       //       2 - clock is too early
                                       //       3 - OK (some measurements show too late, some - too early)
                           .sclk0(sclk0),      // global clock, phase 0
//                           .sclk90(sclk90),     // global clock, phase 0
                           .sclk270(sclk270),    // global clock, phase 0
                           .enrd180(sd_dqsrd),    // read enable, latency 2 from the command, sync with sclk0 falling edge
                           .udqsr90(udqsr90),    // data from SDRAM interface pin UDQS strobed at rising sclk90
                           .ldqsr90(ldqsr90),    // data from SDRAM interface pin LDQS strobed at rising sclk90
                           .udqsr270(udqsr270),   // data from SDRAM interface pin UDQS strobed at rising sclk270
                           .ldqsr270(ldqsr270),   // data from SDRAM interface pin UDQS strobed at rising sclk270
                           .dcm_rst(dcm_rst),    // set DCM phase to default
                           .dcm_clk(dcm_clk),    // clock for changing DCM phase (now == sclk0)
                           .dcm_en(dcm_en),     // enable inc/dec of the DCM phase
                           .dcm_incdec(dcm_incdec), // 0 - increment, 1 - decrement DCM phase
                           .phase90sel(phsel[1:0])  // add phase 0 - 0, 1 - 90, 2 - 180, 3 - 270
                     );


// separate processing of SDRAM-related clocks
//wire       adcmrst ;//async DCM reset
//wire       clockios_locked;
//wire [7:0] clockios_status; // bit 1
//wire       dcm_locked;
//wire [7:0] dcm_status;

 clockios353 #(.IOSTANDARD(IOSTANDARD_CLK)) i_iclockios(
                         .CLK0(CLK0),  // input clock pad - 120MHz
                         .sclk0(sclk0),   // global clock, 120MHz, phase=0  (addresses, commands should be strobed at neg edge)
                         .sclk180(sclk180), // global clock, 120MHz, phase=180 (maybe will not be needed)
                         .sclk270(sclk270),  // global clock, 120MHz, phase=270 (strobe data write to sdram)
                         .iclk0(),
                         .dcmrst(adcmrst),  //reset dcm
                         .locked(clockios_locked),  // dcm locked
                         .status(clockios_status[7:0])   // dcm status (bit 1 - dcm clkin stopped)
                        );
//BUFG i_pclk (.I(CLK1), .O(pclk));
//wire pclki;
//wire pclkig;   // global buffered pclki (maybe not needed at all - just to drive sensor_phase)?
//wire sens_clk; // clock from sensor

IBUF #(.IOSTANDARD(IOSTANDARD_CLK)) i_pclki  (.I(CLK1), .O(pclki));

//assign pclkig= pclki;
//assign pclkig= pclk;
//BUFG      i_pclk  (.I(CLK1), .O(pclk));
//BUFG      i_pclk  (.I(pclki), .O(pclk));
BUFGMUX i_pclk  (.O(pclk),  .I0(pclki), .I1(sens_clk), .S(|cb_pclksrc[1:0]));

                          
                              
 dcm333  #(
      .IOSTANDARD_SDRAM_DIFF (IOSTANDARD_SDRAM_DIFF),
      .SLEW_SDRAM_DIFF       (SLEW_SDRAM_DIFF)
      ) i_dcm333 ( .sclk(sclk0),       // input clock pad - 120MHz
                   .SDCLK(SDCLK),   // positive clock to SDRAM
                   .SDNCLK(SDNCLK),  // negative clock to SDRAM
                   .sdcl_fb(sdcl_fb),
                   .xclk(xclk),    // 60MHz for compressor
                   .phsel(phsel[1:0]),   // additional phase control for SDRAM CLK 

//                   .dcm_rst(dcm_rst), // reset DCM phase
                   .dcm_rst(dcm_rst || adcmrst), // reset DCM phase
                   .dcm_incdec(dcm_incdec),  // variable phase control to adjust SDCLK so read DQS is aligned with sclk90/sclk270
                   .dcm_en(dcm_en),
                   .dcm_clk(dcm_clk),
                   .dcm_done(dcm_done),
                   .locked(dcm_locked),  // dcm locked
                   .status(dcm_status[7:0])   // dcm status (bit 1 - dcm clkin stopped)
);


  dmapads#(
        .IOSTANDARD_SYS       (IOSTANDARD_SYS),
        .SLEW_SYS_DREQ        (SLEW_SYS_DREQ),
        .DRIVE_SYS_DREQ       (DRIVE_SYS_DREQ),
        .IBUF_DELAY_SYS_DACK  (IBUF_DELAY_SYS_DACK),
        .IFD_DELAY_SYS_DACK   (IFD_DELAY_SYS_DACK)
    ) i_dmapads   (
              .dreq0  (DREQ0),
              .dack0  (DACK0),
              .idreq0 (idreq0),
              .idack0 (idack0),
              .dreq1  (DREQ1),
              .dack1  (DACK1),
              .idreq1 (idreq1),
              .idack1 (idack1)
              );
//cb_pxd14
assign ihact=iihact;

//da_sensortrig_lines

///AF: reg  [9:0] ch0_blocks_in_line;
///AF: reg        mode16bits;
///AF: reg [13:3] pre_hact_length;
reg [13:0] hact_length;
always @(negedge sclk0) if (da_sensortrig_lines && (idi[15:14]==2'h1)) begin
  hact_length[13:0] <= idi[13:0];
end
reg        en_vacts_free=1'b1; // register to allow only one vacts after trigger in triggered mode. Allows first vacts after mode is set
always @ (posedge pclk) begin
   if (!cb_sensor_trigger) en_vacts_free<= 1'b1;
   else if (vacts_every)   en_vacts_free<= 1'b0;
end


  sensorpads   #(
        .IOSTANDARD_SENSOR       (IOSTANDARD_SENSOR),
        .SLEW_SENSOR             (SLEW_SENSOR),
        .DRIVE_SENSOR            (DRIVE_SENSOR),
        .IOSTANDARD_SENSOR_CLK   (IOSTANDARD_SENSOR_CLK),
        .SLEW_SENSOR_CLK         (SLEW_SENSOR_CLK),
        .DRIVE_SENSOR_CLK        (DRIVE_SENSOR_CLK),
        .IFD_DELAY_SENSOR_PXD    (IFD_DELAY_SENSOR_PXD),
        .IFD_DELAY_SENSOR_VHACT  (IFD_DELAY_SENSOR_VHACT),
        .IBUF_DELAY_SENSOR_PXD   (IBUF_DELAY_SENSOR_PXD),
        .IBUF_DELAY_SENSOR_VHACT (IBUF_DELAY_SENSOR_VHACT)
        
  )  i_sensorpads(
                     .sclk(sclk0),       // system clock, @negedge
                     .cmd(idi[10:4]),    // [6:0] command for phase adjustment @ negedge (slck) (MSB - reset x2 DCM)
							.wcmd(da_dcm),       // write command@ negedge (slck)
                     .dcm_done(sens_dcm_done),   // DCM command done
                     .dcm_status(sens_dcm_status[7:0]),// dcm status (bit 1 - dcm clkin stopped)
                     .dcm_locked(sens_dcm_locked), //   DCM locked
							.clk_sel(cb_use_sensor_clk),  // 0 - use clk, 1 - sensor dclk (bpf pad) for DCM input (if dclkmode - use clk if 0)
							.hact_length(hact_length[13:0]),// [13:0] WOI width-1 (to overwrite sensor HACT duration) - add later
                     .hact_regen(cb_hact_regen), // 0 - use hact from sensor, 1 - regenerate using hact_length

                              .clk(pclk), 
										.pclk2x(pclk2x),     // output - twice pixel clock
//                              .clk2(pclk2x),
                              .vact(VACT),
                              .hact(HACT),
                              .bpf(BPF),
                              .pxd({PXD[9:0],CNVCLK,CNVSYNC}),
                              .mrst(MRST),
                              .arst(ARST),
                              .aro(ARO),
                              .dclkmode(cb_dclkmode), // 0 - DCLK is clock to sensor, 1 - combined sync from sensor (like 10347)
                              .pxd14(cb_pxd14),       // use {vact,hact} as 2 LSB in 14-bit data
                              .debug(cb_debug[1:0]), // 2-bit debug mode input
                              .dclk(DCLK),
                              .en_vacts(en_vacts_free  || sensor_trigger || vact_overdue),  // disable processing second vact after the trigger in triggered mode
                              .vacts(vacts_every),
                              .ihact(iihact),
                              .sens_clk(sens_clk),
                              .ipxd(ipxd[15:0]),
                              .imrst(imrst == cb_zoran), // invert for Kodak and Fillfactory (cb_zoran ==0)
                              .iarst(iarst),
// for micron sensors                           
                              .iaro(cb_sensor_trigger? ~sensor_trigger : iaro), 
                              .cnvctl({icnvclk,icnvsync}),
                              .cnven(cb_encnvclk),
                              .senspgm(SENSPGM),            // SENSPGM I/O pin
                              .senspgmin(senspgmin),        // state of the SENSPGM I/O pin (read)
                              .xpgmen(xpgmen),              // enable programming mode for an external FPGA
                              .xfpgaprog(xfpgaprog),        // PROG_B to be sent to an external FPGA
                              .xfpgadone(xfpgadone),        // state of the MRST pin ("DONE" pin on an external FPGA)
                              .xfpgatck(xfpgatck),          // TCK to be sent to an external FPGA
                              .xfpgatms(xfpgatms),          // TMS to be sent to an external FPGA
                              .xfpgatdi(xfpgatdi),          // TDI to be sent to an external FPGA
                              .xfpgatdo(xfpgatdo)          // TDO read from an external FPGA
//                              ,.pherr(sensordat_pherr[1:0])  //[1:0] phase error (sync to posedge pclk) {too_early, too_late}

                              );
assign sensordat_pherr[1:0]=0; // restore later?
 PULLUP  i_PU_SCL0   (.O(SCL0));
 i2cpads i_i2cpads0  (.sda(SDA0),.scl(SCL0),
                      .sda_o (i2c_sda),
                      .sda_i (sr_sda0),
                      .sda_en(i2c_sda_en),
                      .scl_o (i2c_scl),
                      .scl_i (sr_scl0),
                      .scl_en(i2c_scl_en));
// Other modules
   assign restart[0]=vacts;
   assign restart[1]=vacts;
   assign restart[3]=0;
   wire [3:0]     bonded;    //[3:0] - channel bonded with the other (just for channel 2), make it TIG
//   only bonded[2] is used
   mcontr i_mcontr(
                   .clk0(sclk0),                     // SDRAM clocks (hope to get 120MHz)
                   .restart_en(cb_reset_mcontr),     // enable restarting selected channels
                   .restart(restart[3:0]),           // reinitialize channels (posedge-sensitive, masked by enXfer0
                   .bonded (bonded[3:0]),            //[3:0] - channel bonded with the other (just for channel 2), make it TIG
                   .nextBlocksEn(nextBlocksEn[3:0]), // enable read blocks to FIFO. disabled when init or roll over, enable by confirmRead

//                 .mclk(cwr),    // modified CPU global clock - write and read with a7==1 (for block memory R/W)
//descriptor memory and channel 3
                   .ia(ia[3:0]),       // internal 4-bit address bus (fast - directly from I/O pads)
                   .as(as[3:0]),       // 4 bit address to select descriptor address and data word (3 bits)
                   .am(am[3:0]),       // switching between ia (async read) and as (sync write)
                   .dscs(da_dswe),     // WE for descriptor memory
//
                   .mwnr(wnr),               // CPU write, not read (valid @ negedge clk0)
                   .init_ch3(da_init_ch3),   // write to init cnhannel 3 (will reset address r/w)
                   .next_ch3(da_next_ch3),   // advance to the next channel3 page, reset address
                   .readNextFrame3(1'b1), //enable reading to the SDRAM buffer (after channel init or frame over), set to 1'b1 if not needed

                   .menrw(da_mem),           // enable CPU read/write  32-bit word from buffer(valid @ mweoe)

                   .di(idi[31:0]),           // // 32-bit data from CPU (descriptor and channel 3)
//                 .d_late(d_late[31:0]),
                   .dsdo(dsdo[31:0]),        // [31:0] data out from descriptor memory
                   .do(bdo[31:0]),           // 32-bit data to CPU (channel 3)

                   .rdy(sr_memrdy),          // ready to r/w next 8x32 page (channel 3) CPU<->SDRAM
                   .wrempty(sr_wrempty),     // CPU write buffer empty (valid only in write mode). May reprogram channel3

                   .ch0clk(pclk),            // clock for channel0
                   .ch0we(sens_we),          // channel 0 (sensor->SDRAM) WE
                   .ch0a(sens_a[9:0]),       // [7:0] channel 0 address (MSB - block #)
                   .ch0di(sens_d[15:0]),     // [15:0] channel 0 data in
                   .stch0(sens_wpage),       // start channel 0 transfer (up to 2 w/o confirmation)
                   .ch0rdy(sr_ch0rdy),       // channel 0 ready -- not used
                   .readNextFrame0(1'b1),    //enable reading to the SDRAM buffer (after channel init or frame over), set to 1'b1 if not needed
                     .ch1clk(pclk),          // clock for channel1
                   .ch1a(fpn_a[9:0]),        // [7:0] channel 1 (SDRAM -> sensor calibration data) address (MSB - block #)
                   .ch1do(fpn_d[15:0]),      // [15:0] channel 1 data out
                   .stch1(fpn_rpage),        // start channel 1 transfer (up to 2 w/o confirmation)
                   .ch1rdy(sr_ch1rdy),       // channel 1 ready -- not used
                   .readNextFrame1(vacts),   // Will delay FPN to start of frame (need at least one line after vacts) enable reading to the SDRAM buffer (after channel init or frame over), set to 1'b1 if not needed
                     .ch2clk(xclk),          // clock for channel2 
                   .ch2a(ch2a[10:0]),        // [10:0] channel 2 (SDRAM -> compressor MCU) address (MSB - block #)
                   .ch2do(ch2do[7:0]),       // [7:0] channel 2 data out
                   .stch2(stch2),            // start channel 2 transfer
                   .ench2(ch2_en_rd_buff),   // enable read from channel 2 buffer to compressor
                   .ch2rdy(sr_ch2rdy),          // channel 2 ready
                   .readNextFrame2(confirmFrame2Compressor), //enable reading to the SDRAM buffer (after channel init or frame over), set to 1'b1 if not needed
                   .nextFrame(sr_nextFrame[3:0]),// (level) generated before actual block is filled - needed to be combined with the pulse from buffer control
                   .chInitOnehot(chInitOnehot[3:0]), // decoded channel init pulses, 2 cycles behind chInit
                   .sddi(sddi_r[31:0]),      // [15:0] data from SDRAM  (1 cycle delayed)
                   .sddo(sddo_p[31:0]),      // [15:0] data to SDRAM    (1 cycle ahead)
                   .sda(sda_p[12:0]),        // [11:0] address to SDRAM (1 cycle ahead)
                   .sdba(sdba_p[1:0]),       // [ 1:0] bank address to SDRAM (1 cycle ahead)
                   .sdwe(sdwe_p),            // WE  command bit to SDRAM (1 cycle ahead)
                   .sdras(sdras_p),          // CAS command bit to SDRAM (1 cycle ahead)
                   .sdcas(sdcas_p),          // CAS command bit to SDRAM
                   .trist(pretrist),         // tristate data to SDRAM   (1 cycle ahead)
                   .predqt(sddqt),           // fristate DQ outputs (one extra for FF in output buffer)
                   .dmask(sddm_p[1:0]),      // [1:0] - now both the same (even number of words written)
                   .dqs_re(sd_dqsrd));       // enable read from DQS i/o-s for phase adjustments  (latency 2 from the SDRAM RD command)


//TODO: apply bayer_phase to sensorpix (gamma tables)
sensorpix   i_sensorpix(
                   .pclk(pclk),  // clock (==pclk)
                   .sclk(sclk0),       // global memory clock, @negedge
                   .we_lensff(da_lensff), // write parameters to lens flat field correction module
                   .pre_wfpn(da_sensormode),//decoded addresses - write fpn [10:0] mode (delay by 1 clk internally)
                                       // [10] - testmode - if 1 generates gradient data (as Zoran chips) where pixel value = horizontal position
                                       // [9:7] - submode - subtract background (8 bits) mode (use di[7:0]):
                                       // 000 - no subtraction;
                                       // 001 - (finest) subtract 8 bit bkgnd from 12 bits pixels
                                       // 010 - shift 8bit bkgnd 1 bit  left before applying
                                       // 011 - shift 8bit bkgnd 2 bits left before applying
                                       // 100 - shift 8bit bkgnd 2 bits left before applying
                                       // 101 - shift 8bit bkgnd 2 bits left before applying
                                       // fpn data to subtract should be a little less to add a "fat zero" 
                                       // [6:4] - mpymode sensitivity correction mode (use di[15:8]):
                                       // 000 - no correction
                                       // 001 - fine correction (+/-3.125%)
                                       // 010 - fine correction (+/-6.25%)
                                       // 011 - fine correction (+/-12.5%)
                                       // 100 - +/- 25%
                                       // 101 - +/- 50%
                                       // [3] - wdth - word width: 0 - 8 bit, 1 - 16 bit (5 MSB == 0)
                                       // [2:0] scaling of 11 bit FPN result to fit in 8bit output:
                                       // 00 - default - use [9:1]
                                       // 01 - use [10:2] - to protect from saturation after applying mpymode
                                       //      nominal range - 0..127
                                       // 10 - use [7:0] before saturation, "digital gain" == 4 (maximal)
                                       // 11 - use [8:1] before saturation, "digital gain" == 2
                   .pre_wthrsh(da_virttrig),// write thresh. register [21:0]  (delay by 1 clk internally)
                                       // [21:0] - trigger threshold - sum of pixels in a line,  [7:0] - for fpn mode
//                 .ta(ta[8:0]),       // "curves" table write address
                   .ta(ta[9:0]),       // "curves" table write address
                   .twce(twr_gamma),   // "curves" table write enable
                   .wd(idi[15:0]),     // used for curves and data registers
                   .en(sensor_en),     // Enable. Should go active before or with the first hact going active.
                                       // when low will also reset MSB of addresses - buffer page for ping-pong access.
                                       // SDRAM ch1 should be enabled earler to have data ready in the buffer
                                       // When going low will mask input hact, finish pending data->SDRAM and quit
                                       // So normal sequence is:
                                       //   1 - program (end enable) SDRAM channels 0 and 1, channel 1 will start reading
                                       //   2 - wait for frame sync and enable "en"
                                       //   3 (optional) - after frame is over (before the first hact of the next one)
                                       //      turn "en" off. If needed to restart - go to step 1 to keep buffer pages in sync.
//                 .hact_en(hact_en),  // just mask hact
                   .trig(virt_trig),               // trigger out - fires sum of pixels in a line > threshold
                   .trig_sel(virt_sel),            // threshold !=0, use this trigger and disable external
                   .bayer(cb_bayer_phase[1:0]),              // bayer phase
                   .hact_out(line_run),

// sensor interface
                   .hact(ihact_ts),       // line active
                   .pxd(ipxd_ts[15:0]),      // [9:0] - 10 bit pixel data
// channel 0 (data->SDRAM) interface
                   .dwe(sens_we),         // WE to SDRAM buffer
                   .wa(sens_a[9:0]),      // 8 bit address sensorpix->SDRAM (MSB - page # for ping-pong access to 128x16 pages)
                   .do(sens_d[15:0]),     // 16 bit data to SDRAM (1/pixel for raw, 0.5/pixel for FPN processed
                   .wpage(sens_wpage),    // write page to SDRAM
// channel 1 (FPN data from SDRAM) interface
                   .ra(fpn_a[9:0]),       // 8 bit address SDRAM->sensorpix (MSB - page # for ping-pong access to 128x16 pages)
                   .di(fpn_d[15:0]),      // 16 bit FPN correction data from SDRAM
                   .rpage(fpn_rpage),     // request to read next page from SDRAM
                   .vacts_sclk(vacts_sclk),         // frame sync, @negedge single cycle
                   .table_page(gamma_table_page));        // number of gamma table page currently in use (writes go to the other one)


sensortrig i_sensortrig (  .pclk(pclk),
/****************/         .sclk(sclk0),                       // global (made of cpu WE)
                           .wlin(da_sensortrig_lines),      // decoded address, write last line # in frame (since trigger in external trigger mode)
                           .wcmd(da_sensortrig),            // decoded address, write command from d[2:0]:
                                                            // d[2] - enable
                                                            // d[1] - external (0 - internal)
                                                            // d[0] - continuous
                           .framesync_dly(da_framesync_dly),
                           .d(idi[15:0]),                   // [11:0] data in
                           .compressed_frames(cb_compressed_frames[7:0]), // [7:0] - bitmask of frames to compress (generate vacts15)
                           .frame_num(sequencer_frame_no[2:0]),         // [2:0] current frame number (switches after vacts_sclk)
                           .trig(virt_sel?virt_trig:((!cb_xt_pol) ^ itrig)),                          // external trigger, 0->1 transition
                           .hact(ihact),                    // hact (line active)
                           .vacts_in(vacts_every),          // single-cycle frame sync
                           .vacts_out(vacts),               // single-cycle frame sync (divided if needed in photofinish mode)
                           .vacts15(vacts_long),          // pulse from vacts active for 15 scan lines
                           .sensor_en(sensor_en),           // enable sensor output
                           .trig_v(trig_v[13:0]),           // [10:0] line number when trigger occured
                           .trig_h(trig_h[13:0]),           // [11:0] pixel number when trigger occured
                           .status(sr_sensortrig[2:0]),     // [2:0]: 00 - off, 01 - waiting for vacts to start, 10 - frame active, 11 - frame over
                           .frame_run(frame_run),           // active while frame is being acquired (include all line_run)
                           .xfer_over_irq(xfer_over_irq),   // pulse after transfer of specified number of lines (after trigger)
                           .trig_irq(trig_irq),             // single-cycle (pclk) pulse at external interrupt
                           .fillfactory(1'b0),    // fillfactory sensor mode
                           .ystart(),                 // start of frame (fillfactory sensor mode) - 3 cycles long;
                           .yclock()                  // start of line  (fillfactory sensor mode) - 1 cycle long;
                         );

histogram i_histogram (.pclk(pclk),              // pixel clock (posedge, only some input signals
                       .pclk2x(pclk2x),          // pclk multiplied by 2
                       .sclk(sclk0),             // global (commands @negedge)
                       .wen(da_hist),         // @negedge sclk, 1 ahead of write enable    - also reads histogram
                       .rnext(da_hist_next),  // read histogram, increment address
                       .wa(as[2:0]),         // [2:0] register address:
                                         // old - 00 - {top, left}
                                         // old - 01 - {height-1, width-1}
                                         // old - 02 - hist. data start address (will also read pointed word to output word

                                         // 00 - left
                                         // 01 - top
                                         // 02 - width-1
                                         // 03 - height-1
                                         // 04 - hist. data start address (will also read pointed word to output word
                       .hist_do(hist_do[31:0]),  // [31:0] histogram data (actually now just [17:0])
                       .wd(idi[15:0]),           // [31:0] PIO data  to write
                       .frame_run(sr_sensortrig[1]),    // frame active - @posedge pclk
                       .line_run_a(line_run),      // hact_out @posedge pclk
                       .di_a(sens_d[15:0]),        // [15:0] @posedge pclk
                       .di_vld_a(sens_we),         // di[15:0] valid @posedge pclk
                       .bayer_phase(cb_bayer_phase[1:0])      // [1:0]
                       );

irq_smart i_irq_smart   (.sclk(sclk0),               // @negedge
                         .wen(da_irq_smart),                // sync to address and d[0:15]
                         .di(idi[15:0]),                 // [15:0] data in, only [3:0] are currently used
                         .frame_sync(vacts_sclk),         // frame sync, single pulse @ negedge sclk
                         .is_compressing(is_compressing),  // @posedge clk, needs re-sync
//                         .compressor_started(compressor_started), // single pulse @ negedge sclk (this pulse comes before next frame sync if the current frame is being compressed)
                         .compressor_done(compressor_done_pulse),    // single pulse @ negedge sclk - compressor finished (some data is still in DMA FIFO)
                         .fifo_empty(dma_empty),         // DMA FIFO empty (no gaps between compressor_done and !fifo_empty)
                         .irq(smart_irq));               // single cycle $ negedge sclk output to be used as IRQ source

//wire [15:0] irq_in;
assign irq_in= {7'b0, // extra
                smart_irq,
                compressor_done_compress, // 7 - will go high after all data is sent out (reset by compressor)
                compressor_done_input,    // 6 - will go high after EOT and persist until DCT is enabled (reset by compressor)
                dcc_rdy,                  // 5 - obsolete in 333 - channel3 has 128 more dc coefficients (or some if the compression is finished)
                compressor_eot,           // 4 - compressor read in the last MCU (predictable time to end of transfer
                sr_sensortrig[2],         // 3 - (sync to pclk) - level, reset by writing to sensor triggering command register
                xfer_over_irq,            // 2 - (sync to pclk) - frame acquisition over
                trig_irq,                 // 1 - (sync to pclk) - external trigger
                vacts};                   // 0 - (sync to pclk) - frame sync
                
                

interrupts_vector i_interrupts_vector (.sclk(sclk0), // @negedge
                                      .pre_wen(da_interrupts), // 1 cycle ahead of write data
                                      .pre_wa(as[1:0]),        // [1:0] - write address:
                                                               //   0 - reset selected interrupt bits
                                                               //   1 - disable selected interrupt bits (mask)
                                                               //   2 - enable selected interrupt bits
                                                               //   3 - write vector number (bits [0:7], [11:8] - interrupt number (0..15)
                                      .di(idi[15:0]),          // [15:0] data in
                                      .do(irqr[31:0]),         // [31:0] data out (bits {15:0} - masked interrupts, [31:0] - same before mask
                                      .irq(iirq),              // interrupt request (active high)
                                      .inta(1'b1),             // interrupt acknowledge input (active low) 170ns long
                                      .irq_in(irq_in[15:0]),   // [15:0] input interrupt requests (posedge)
                                      .irqv());                // [7:0] interrupt vector (valid before end on inta
// currently not used
//assign io_db=   12'b0;
//assign io_db_en=12'b0;
                                      
twelve_ios i_twelve_ios  (.sclk(sclk0), // @negedge
                         .pre_wen(da_io_pins), // 1 cycle ahead of write data
//                         .di(idi[31:0]),       // [31:0] data in
                         .di(idi[15:0]),       // [31:0] data in
                         
                         .io_do(io_do[11:0]),   // [11:0] data to I/O pins
                         .io_t(io_t[11:0]),     // [11:0] tristate I/O pins
                         .da(io_da[11:0]),      // [11:0] data from port A
                         .da_en(io_da_en[11:0]),// [11:0] data enable from port A
                         .db(io_db[11:0]),      // [11:0] data from port B
                         .db_en(io_db_en[11:0]),// [11:0] data enable from port B
                         .dc(io_dc[11:0]),      // [11:0] data from port C
                         .dc_en(io_dc_en[11:0]));//[11:0] data enable from port C

// put motor contol here!!!								 
//	 .da_extio(da_extio),                      // external I/O (motor control 0x7c-0x7d)
//  wire [15:0] ext_di; // External data in - currently from the motor control;
// sclk0 - I/O, xclk - iternal (80MHz)
assign io_db[5:0]=   6'b0;
assign io_db_en=12'hfc0; // 6 MSBs - outputs, 6 LSBs - inputs
three_motor_driver i_three_motor_driver( .clk(sclk0),           // system clock, negedge
                                         .xclk(xclk),           // half frequency (80 MHz nominal)
                                         .we(da_extio),         // write enable (lower 16 bits, high - next cycle)
                                         .wa(as[0]),            // write address(1)/data(0)
                                         .di(idi[15:0]),        // 16-bit data in (32 multiplexed)
                                         .do(motor_di[31:0]),     // 16-bit data output
                                         .encod1(io_pins[1:0]), // 2-bit encoder data input, motor1
                                         .encod2(io_pins[3:2]), // 2-bit encoder data input, motor2
                                         .encod3(io_pins[5:4]), // 2-bit encoder data input, motor3
                                         .mot1(io_db[ 7: 6]),     // 2 bits motor1 control output (11 - shorted, 00 - stop)
                                         .mot2(io_db[ 9: 8]),     // 2 bits motor1 control output (11 - shorted, 00 - stop)
                                         .mot3(io_db[11:10])      // 2 bits motor1 control output (11 - shorted, 00 - stop)
                                        );
/*
wire imu_mosi,imu_miso,imu_sda,imu_sda_en,imu_scl,imu_scl_en;
wire gps_ser_di, gps_pulse1sec;
assign io_dc=      {9'b0,imu_mosi,imu_sda,imu_scl};
assign io_dc_en=   {9'b0,1'b1,    imu_sda_en,imu_scl_en};
assign imu_miso=   io_pins[3];
assign gps_ser_di=   io_pins[4];
assign gps_pulse1sec=io_pins[4];
*/
//wire [15:0] imu_dma_data; // data to DMA1 fifo (@negedge clk)
//wire        imu_dma_stb;  // data strobe to DMA1 fifo (@negedge clk)
//wire [23:0] imu_sample_counter; // number of 64-byte samples written
//running_sec
imu_logger i_imu_logger (.clk(sclk0),                // system clock, negedge
                         .xclk(xclk),                // half frequency (80 MHz nominal)
                         .we(da_imu),                // write enable (lower 16 bits, high - next cycle)
                         .wa(as[0]),                 // write address(1)/data(0)
                         .di(idi[15:0]),             // 16-bit data in (32 multiplexed)
                         .usec(running_usec[19:0]),  // un-latched timestamp microsecondstarget_pos[5:0]
                         .sec(running_sec[31:0]),    // un-latched timestamp seconds
                         .ext_di(io_pins[11:0]),
                         .ext_do(io_dc[11:0]),
                         .ext_en(io_dc_en[11:0]),
                         .ts_rcv_sec (ts_sync_sec[31:0]),  // [31:0] timestamp seconds received over the sync line
                         .ts_rcv_usec(ts_sync_usec[19:0]),// [19:0] timestamp microseconds received over the sync line
                         .ts_stb(ts_stb),       // strobe when received timestamp is valid - single negedge sclk cycle
                         .data_out(imu_dma_data[15:0]),    // 16-bit data out to DMA1 (@negedge clk)
                         .data_out_stb(imu_dma_stb),// data out valid (@negedge clk)
                         .sample_counter(imu_sample_counter[23:0]), // could be DMA latency, safe to use sample_counter-1
                         .debug_state(imu_debug_state[31:0])
                        );
 
//wire trigger_single_pclk;
camsync i_camsync       (.sclk(sclk0), // @negedge
                         .pre_wen(da_extsync), // 1 cycle ahead of write data
                         .di(idi[15:0]),      // [31:0] data in
                         .wa(as[1:0]),      // [1:0] write address
                           // 0 - source of trigger (12 bit pairs, LSB - level to trigger, MSB - use this bit). All 0 - internal trigger
                           // 1 - input trigger delay (pixel clocks), 0 - trigger disabled
                           // 2 - 24 bit pairs: MSB - enable selected line, LSB - level to send when trigger active
                           // 3 - output trigger period (duration constant of 256 pixel clocks). 
                           //     d==0 - disable (stop periodic mode)
                           //     0<d<256(?) single trigger
                           //     256>=d - repetitive trigger
                         .pclk(pclk),    // pixel clock (global)
                         .triggered_mode(cb_sensor_trigger),
                         .trigrst(vacts || !cb_sensor_trigger),   // single-clock start of frame input (resets trigger output) posedge
                         .gpio_in(io_pins[11:0]), // 12-bit input from GPIO pins
                         .gpio_out(io_da[11:0]),// 12-bit output to GPIO pins
                         .gpio_out_en(io_da_en[11:0]),// 12-bit output enable to GPIO pins
                         .trigger1(trigger_single_pclk), // 1 cycle-long trigger output
                         .trigger(sensor_trigger),
                         .overdue(vact_overdue),     // prevents lock-up when no vact was detected during one period and trigger was toggled
                         .ts_snd_en(cb_output_timestamp),   // enable sending timestamp over sync line
                         .ts_external(cb_external_timestamp), // 1 - use external timestamp, if available. 0 - always use local ts
                         .ts_snap(ts_snap),     // make a timestamp pulse  single @(posedge pclk)
                                         // timestamp should be valid in <16 pclk cycles
                         .ts_snd_sec (psec[31:0]),  // [31:0] timestamp seconds to be sent over the sync line
                         .ts_snd_usec(pusec[19:0]), // [19:0] timestamp microseconds to be sent over the sync line
                         .ts_rcv_sec (ts_sync_sec[31:0]),  // [31:0] timestamp seconds received over the sync line
                         .ts_rcv_usec(ts_sync_usec[19:0]),// [19:0] timestamp microseconds received over the sync line
                         .ts_stb(ts_stb)    // strobe when received timestamp is valid - single negedge sclk cycle
                         ); // active high trigger to the sensor (reset by .trigrst(vacts) )
i2c_writeonly i_i2c_writeonly
                        (.sclk(sclk0),      // @negedge
                         .wen(da_i2c),      // sync to address and d[0:15]
                         .wa(as[3:0]),      // [3:0] 0..7 - absolute data, 8..0x0e relative data, 0x0f - command
                         .di(idi[15:0]),    // [15:0] data in
                         .sync(vacts_sclk),// frame sync (used to synchronize data)
                         .busy(i2c_busy),// busy (do not use software i2i)
                         .scl(i2c_scl),// i2c SCL
                         .sda(i2c_sda),// i2c SDA
                         .scl_en(i2c_scl_en),  // switch i2c control to i2c_writeonly (from software direct bit control PIO)
                         .sda_en(i2c_sda_en),// enable SDA output
                         .frame_no(i2c_frame_no[2:0]));// i2c frame number (modulo 8)

`ifdef debug_stuffer
always @ (negedge sclk0) if (sequencer_ack) tst_cmd_cntr <= tst_cmd_cntr+1;
`endif

cmd_sequencer i_cmd_sequencer
                        (.sclk(sclk0),                        // @negedge
                         .wen(da_sequencer),                  // sync to address and d[0:15]
                         .wa(as[3:0]),                        // [3:0] 0..7 - absolute data, 8..0x0e relative data, 0x0f - command
                         .di(idi[15:0]),                      // [15:0] data in
                         .sync(vacts_sclk),                   // frame sync (used to synchronize data), vacts_sclk @negedge sclk
//                         .condition(sequencer_condition[7:0]),// condition to wait
                         .seq_rq(sequencer_rq),               // request from the sequencer
                         .seq_ack(sequencer_ack),             // sequencer acknowledge
                         .seq_a(sequencer_a[7:0]),            // address from the sequencer
                         .seq_d(sequencer_d[23:0]),           // data from the sequencer
                         .frame_no(sequencer_frame_no[2:0])); // [2:0] current frame modulo 8
wire [7:4] test_fifo_dummy; //SuppressThisWarning Veditor UNUSED        
wire [7:4] test_dma_wcntr_dummy; //SuppressThisWarning Veditor UNUSED        
 dma_fifo_sync i_dma_fifo0 (  .clk(sclk0),      // system clock, 120MHz? (currentle negedge used)
                        .pre_wen(da_dmamode),   // decoded addresses (valid @ negedge clk)
//                        .wd({idi[3],idi[1]}),      // only 2 bits are used -  {pio_mode,dma_enable} 1 cycle delay
                        .wd(idi[2:0]),      // {enable update, pio_mode, enable channel}
                        .dreq(idreq0),    // DREQ output to CPU
                        .dack(idack0),    // DACK input from CPU
                        .oe(ioe),
                        .pioinc(da_pio_dmafifo),  // increment buffer address in PIO mode

                        .d(dma_d0[31:0]),       // [31:0]   - data to cpu
                        .we(stuffer_dv),      // write enable (sync to clk)
                        .di({stuffer_do[7:0],stuffer_do[15:8]}),      // 16-bit data to write
                        .enabled(dma0_enabled),
                        .real_empty(dma_empty)
         ,.test1( {test_fifo_dummy[7:4], test_fifo[3:0]})
         ,.test2( {test_dma_wcntr_dummy[7:4], test_dma_wcntr[3:0]})

                        ); // pessimistic, with several clk delay
/*
 dma_fifo_sync i_dma_fifo1 (  .clk(sclk0),      // system clock, 120MHz? (currentle negedge used)
                        .pre_wen(da_dmamode),   // decoded addresses (valid @ negedge clk)
                        .wd({idi[4],idi[2]}),      //  only 2 bits are used -  {pio_mode,dma_enable}
                        .dreq(idreq1),    // DREQ output to CPU
                        .dack(idack1),    // DACK input from CPU
                        .oe(ioe),
                        .pioinc(da_pio_dmafifo),  // increment buffer address in PIO mode
                        .d(dma_d1[31:0]),       // [31:0]   - data to cpu
                        .we(statistics_dv),      // write enable (sync to clk)
                        .di(statistics_do[15:0]),// 16-bit data to write
                        .enabled(dcc_enabled),
                        .real_empty() //not use now?
                        ); // pessimistic, with several clk delay
*/                        
 dma_fifo_sync i_dma_fifo1 (  .clk(sclk0),      // system clock, 120MHz? (currentle negedge used)
                        .pre_wen(da_dmamode),   // decoded addresses (valid @ negedge clk)
//                        .wd({idi[4],idi[2]}),      //  only 2 bits are used -  {pio_mode,dma_enable}
                        .wd(idi[5:3]),      // {enable update, pio_mode, enable channel}
                        .dreq(idreq1),    // DREQ output to CPU
                        .dack(idack1),    // DACK input from CPU
                        .oe(ioe),
                        .pioinc(da_pio_dmafifo),  // increment buffer address in PIO mode
                        .d(dma_d1[31:0]),       // [31:0]   - data to cpu
                        .we(imu_dma_stb),      // write enable (sync to clk)
                        .di(imu_dma_data[15:0]),// 16-bit data to write
                        .enabled(dma1_enabled),
                        .real_empty() //not use now?
         ,.test1( test_fifo1[7:0])
         ,.test2( test_dma1_wcntr[7:0])
                        
                        ); // pessimistic, with several clk delay
                        
                       
/*
wire [15:0] imu_dma_data; // data to DMA1 fifo (@negedge clk)
wire        imu_dma_stb;  // data strobe to DMA1 fifo (@negedge clk)

*/                        

`ifdef debug_stuffer

always @ (posedge xclk) begin
 if (compressor_eot) tst_rdy_after_eot <= 0;
 else if (sr_ch2rdy) tst_rdy_after_eot <= 1;
end
`endif

`ifdef debug_mcontr_reset
  reg [12:0] debug_mcontr_count;
  reg [15:0] debug_mcontr_count_start;
  reg [15:0] debug_mcontr_count_end;
  reg [ 1:0] debug_mcontr_hactd;
  reg        debug_mcontr_hacts;
  reg        debug_mcontr_vacts;
  reg        debug_mcontr_restart2;
  reg        debug_mcontr_compressor_eot;
  
//  assign debug_mcontr_reset_data={3'h0, debug_mcontr_count_start[12:0],3'h0,debug_mcontr_count_end[12:0]};
  assign debug_mcontr_reset_data={debug_mcontr_count_start[15:0],debug_mcontr_count_end[15:0]};
  always @ (posedge pclk) begin
    debug_mcontr_hactd[1:0] <= {debug_mcontr_hactd[0],ihact_ts};
    debug_mcontr_hacts <= debug_mcontr_hactd[0] && !debug_mcontr_hactd[1];
    debug_mcontr_vacts <= vacts;
    if      (debug_mcontr_vacts) debug_mcontr_count <= 13'h0;
    else if (debug_mcontr_hacts) debug_mcontr_count <= debug_mcontr_count+1;
  end
  always @ (posedge xclk) begin
    debug_mcontr_restart2 <= restart[2];
    debug_mcontr_compressor_eot <= compressor_eot;
    if (debug_mcontr_restart2)       debug_mcontr_count_start <= {sr_ch2rdy, compressor_done_input, confirmFrame2Compressor, debug_mcontr_count};
    if (debug_mcontr_compressor_eot) debug_mcontr_count_end   <= {sr_ch2rdy, compressor_done_input, confirmFrame2Compressor, debug_mcontr_count};
    
  end
  reg    [11:0] debug_interrupt_counts;   //SuppressThisWarning Veditor UNUSED
  reg    [ 3:0] debug_interrupt_frames=0;
  reg    [ 3:0] debug_interrupt_starts=0;
  reg    [ 3:0] debug_interrupt_dones=0;
  
  always @ (negedge sclk0) begin
    if (vacts_sclk)            debug_interrupt_frames<=debug_interrupt_frames+1;
    if (compressor_started)    debug_interrupt_starts<=debug_interrupt_starts+1;
    if (compressor_done_pulse) debug_interrupt_dones <=debug_interrupt_dones+1;
    if (vacts_sclk) begin
      debug_interrupt_counts[11:0] <={debug_interrupt_dones[3:0],
                                      debug_interrupt_starts[3:0],
                                      debug_interrupt_frames[3:0]};
    end
  
  end
`endif
  wire   [2:0] compressor_test_state; // {is_compressing,cmprs_repeat,cmprs_en}    //SuppressThisWarning Veditor UNUSED
  compressor i_compressor( .clk(xclk),          // compressor pixel clock (80MHz?)
                           .clk2x(sclk0),       // twice compressor pixel clock (120MHz) - for huffman/stuffer), memory clock, input data clock (negedge)
                           .cwe(da_compressor), // we to compressor from CPU
                           .wr_saturation(da_saturation), // write color saturation vaues
                                                // (currently 10 bits - cb from idi[9:0],
                                                //  cr - from  idi[25:16]
                           .wr_quantizer_mode(da_quantizer_mode), // Quantizer tuning - 0..7 - zero bin, 15:8 - quantizer bias
                           .rs(as[0]),          // 0 - bit modes,
                                                // 1 - number of MCUs
                           .twqe(twr_quant),    // quantization table write enable
                           .twce(twr_coring),   // coring functions tables (@negedge clk - addr and data valid this cycle and one before)
                           .twhe(twr_huff),     // huffamn table write enable
                           .twfe(twr_focus),    // table to be multiplied by DCT coefficients (then squared and accumulated)
                                                // to determine focus sharpness
//                           .ta(ta[11:0]),        // [8:0] addresses words in tables (quantization and huffman)
                           .ta(ta[ 9:0]),        // [8:0] addresses words in tables (quantization and huffman)

                           .di(idi[15:0]),      //  data from CPU
                           .vacts_long(vacts_long), // long vacts pulse (15 scan lines) to delay comprssor start
                           .eot(compressor_eot),// ( to interrupts?) pulse while processing the last MCU in a frame - predictable time to end of DMA transfer
                                                // SDRAM interface
                           .done_input(compressor_done_input),    // will go high after EOT and persist until DCT is enabled
                           .done_compress(compressor_done_compress), // will go high after all data is sent out
                           .done_compress_pulse(compressor_done_pulse), // Does not need to be reset
                           .compressor_started(compressor_started),   // single sclk compressor started
                           .is_compressing(is_compressing),  // @posedge clk, from go_single to eot (actual or simulated)

                           .chInitOnehot2(chInitOnehot[2]), // decoded channel init pulses, 2 cycles behind chInit (to reset page address)
                           //if cb_break_frames==0 the nextBlocksEn feature will be disabled (old functionality)
                           .nextBlocksEn(!cb_break_frames || nextBlocksEn[2]), // When combined with SDRAM data ready, may be used to end frame compression input (instead of block counter)
                           .pxd(ch2do[7:0]),    // [7:0] data from SDRAM organized as 16x16x8bit MCUs
                           .pxa(ch2a[10:0]), // [8:0] - address to SDRAM buffer (2 MSBs - page. Starts with 0)
                           .pxrdy(sr_ch2rdy),   // SDRAM buffer (channel 2) has a page ready
                           .nxtpage(stch2),// read next page to SDRAM buffer (from SDRAM)
                           .inc_sdrama(ch2_en_rd_buff), //enable read from SDRAM buffer
                           .confirmFrame2Compressor(confirmFrame2Compressor), // (re) enable reading SDRAM to channel 2 FIFO (it stops after each frame over and after channel init)
                           .restart_memory(restart[2]), // restart memory channel (to align after broken frames) - masked by enable bit in memory controller
                           .bonded (bonded[2]),    // channel bonded with the other (just for channel 2), make it TIG
                           // DMA (output data) interface
                           .bayer_phase(cb_bayer_phase[1:0]), // bayer color filter phase 0:(GR/BG), 1:(RG/GB), 2: (BG/GR), 3: (GB/RG)
// Reusing channel 3 for DC components output (currently disconnected)
                             .dccout (dcc_enabled),       // enabled output of DC (and HF) components - not needed anymore
                             .hfc_sel(cb_hfc_sel),        // [2:0] (for autofocus) only components with both spacial frequencies higher than specified will be added
                           .statistics_dv(statistics_dv),     //sclk
                           .statistics_do(statistics_do[15:0]),//[15:0] sclk
                           .sec(ts_sync_sec[31:0]),    // [31:0] number of seconds
                           .usec(ts_sync_usec[19:0]),    // [19:0] number of microseconds
                           
                           .q(stuffer_do[15:0]),
                           .qv(stuffer_dv),
                           .imgptr (imgptr[23:0]), // [23:0]image pointer in 32-byte chunks 
                           .hifreq (hifreq[31:0])  //  accumulated high frequency components in a frame sub-window
                           ,.dma_is_reset(!dma0_enabled) // DMA module is reset, enable resetting data counters
                           ,.test_state(compressor_test_state[2:0]) // {is_compressing,cmprs_repeat,cmprs_en}
                           
`ifdef debug_compressor
                           ,.test_cntr0(printk_compressor[31:0])
`endif

`ifdef debug_stuffer
                           ,.tst_stuf_negedge(testwire[3:0]),
                           .tst_stuf_posedge(testwire[7:4]),
                           .tst_stuf_etrax(tst_stuf_etrax[3:0]), // [3:0] just for testing
                           .test_cntr(printk[3:0]), // [3:0] just for testing
                           .test_cntr1(printk[11:4]) // [3:0] just for testing
`endif
                           );


`ifdef debug_dma_count
//  wire [31:0] printk;
//  reg   [23:0] debug_cntr_stuffer_dv=24'h0;
  reg   [21:0] debug_cntr_stuffer_dv=22'h0;
//  reg   [7:0] debug_cntr_stuffer_dv1=8'h0;
  always @(negedge sclk0) begin
    if   (!dma0_enabled) debug_cntr_stuffer_dv <= 0;
    else if (stuffer_dv) debug_cntr_stuffer_dv <= debug_cntr_stuffer_dv+1;
  end
//  wire   [1:0] compressor_test_state; // {cmprs_repeat,cmprs_en}
  
//  assign printk={test_dma_wcntr[3:0],test_fifo[3:0],compressor_test_state[1:0],debug_cntr_stuffer_dv[21:0]};
/*
  assign printk={test_dma_wcntr[3:0],
                 test_fifo[3:0],
                 1'b0,compressor_test_state[2:0],
                 debug_interrupt_counts[11:0],
                 debug_cntr_stuffer_dv[7:0]};
*/
//imu_dma_stb
/*
  reg [9:0] test_imu_dma_stb_cntr;
  always @(posedge sclk0) begin
     if (imu_dma_stb) test_imu_dma_stb_cntr[9:0] <= test_imu_dma_stb_cntr[9:0]+1;
  end  
*/  
  assign printk={
                 8'b0,
                 //test_dma_wcntr[3:0],
                 //test_fifo[3:0],
                 dma1_enabled,
                 dma0_enabled,
//                 test_imu_dma_stb_cntr[9:0],
//                 1'b0,compressor_test_state[2:0],
//                 debug_interrupt_counts[3:0],
//                 2'b0,
//                 6'h0,
//                 test_imu_dma_stb_cntr[9:4],


//                 idack1,    // DACK input from CPU
//                 idreq1,    // DREQ output to CPU
//                 idack0,    // DACK input from CPU
//                 idreq0,    // DREQ output to CPU
                
//                 test_dma1_wcntr[7:0],
                 14'b0,
                 test_fifo1[7:0]};


`endif
                           

/*
`ifdef debug_dma_count
  wire [31:0] printk;
`endif
         ,.test1( test_fifo[3:0])
         ,.test2( test_dma_wcntr[3:0])

*/

                           
                           

  sensdcclk i_sensdcclk(.clk(pclk),    // sensor clock (may be variable)
                        .frame(vacts), // frame sync (1 cycle, sync to pclk)
                        .d(idi[6:0]),  // [6:0] divider. Nominal 5'h05 for 20MHz pixel rate @ MSBs -additional :16/:256/:4096 divider
                                       // data=(Fpix[MHz]/1.2)-1, if Fpix=20MHz, data=15.7->16=5'h10
                        .sclk(sclk0),
                        .pre_we(da_sens_dc),// write enable to sensdcclk
                        .cnvclk(icnvclk),  // 625KHz clock to drive DC-DC converter
                        .cnvext(icnvsync)); // 0 - DCDC use internal clock, 1 - external

 control_regs i_control_regs (
                         .sclk(sclk0),    // @negedge
                         .wen(da_dcr),     // sync to address and d[0:15], 0x4e/0x4f
                         .wa(as[1:0]),      // [1:0] register select
                         .di(idi[15:0]),      // [15:0] data in
/// outputs
                         .bayer_phase(cb_bayer_phase[1:0]), //[1:0]
                         .hact_regen(cb_hact_regen),
                         .reset_mcontr(cb_reset_mcontr),
                         .break_frames(cb_break_frames), /// Enable ending frame if no more data is available
                         .zoran(cb_zoran),
                         .use_sensor_clk(cb_use_sensor_clk), // removed - not needed, variable Bayer shift replaces it
                         .xt_pol(cb_xt_pol),
                         .arst(iarst),
                         .aro(iaro),
                         .encnvclk(cb_encnvclk),
                         .sensor_trigger(cb_sensor_trigger),
                         .mrst(imrst),
//                         .early_trigger(cb_early_trigger),
                         .external_timestamp(cb_external_timestamp), // use external timestamp if available
                         .output_timestamp(cb_output_timestamp),   // output timestamp, not just pulse
                         
                         .dclkmode(cb_dclkmode),
                         .pxd14(cb_pxd14),
                         .latehact(cb_debug[1:0]), //[1:0]=dcr[22:21];//  register hact, vact N/4 Tpclk later than data (needed for MT9P001 @ 96MHz)
                         .pclksrc(cb_pclksrc[1:0]),  //[1:0]=dcr[25:24]; // pclk source
                         .hfc_sel(cb_hfc_sel[2:0]),   //[2:0]=dcr[30:28];
                         .blockvsync(blockvsync), // block vsync from sensor to sequencers
                         .compressed_frames(cb_compressed_frames[7:0])
                      );   // [2:0] current frame modulo 8


     OBUF  #(
        .IOSTANDARD(IOSTANDARD_SYS),
        .DRIVE(DRIVE_SYS),
        .SLEW(SLEW_SYS))
      i_irq  (.I(!iirq),  .O(IRQ) );



// dummy module instances
(* keep *)
wire   iclk2; // SuppressThisWarning Veditor UNUSED
  IBUF #(.IOSTANDARD(IOSTANDARD_CLK)) i_iclk2 (.I(CLK2), .O(iclk2));
(* keep *)
wire   iclk4; // SuppressThisWarning Veditor UNUSED
  IBUF #(.IOSTANDARD(IOSTANDARD_CLK)) i_iclk4 (.I(CLK4), .O(iclk4));
  IBUF #(.IOSTANDARD(IOSTANDARD_CLK)) i_iclk3 (.I(CLK3), .O(iclk3));

// temporary assignments for outputs - connect later where it belongs

// 3 legacy wires  
  assign itrig=io_pins[5];
///AF:    assign sr_sda1=io_pins[1];
///AF:    assign sr_scl1=io_pins[0];

  IOBUF #(.IOSTANDARD(IOSTANDARD_EXT), .SLEW(SLEW_EXT), .DRIVE(DRIVE_EXT))
          i_iopins0 (.I(io_do[ 0]), .T(io_t[ 0]), .O(io_pins[ 0]), .IO(EXT[ 0]));
  IOBUF #(.IOSTANDARD(IOSTANDARD_EXT), .SLEW(SLEW_EXT), .DRIVE(DRIVE_EXT))
          i_iopins1 (.I(io_do[ 1]), .T(io_t[ 1]), .O(io_pins[ 1]), .IO(EXT[ 1]));
  IOBUF #(.IOSTANDARD(IOSTANDARD_EXT), .SLEW(SLEW_EXT), .DRIVE(DRIVE_EXT))
          i_iopins2 (.I(io_do[ 2]), .T(io_t[ 2]), .O(io_pins[ 2]), .IO(EXT[ 2]));
  IOBUF #(.IOSTANDARD(IOSTANDARD_EXT), .SLEW(SLEW_EXT), .DRIVE(DRIVE_EXT))
          i_iopins3 (.I(io_do[ 3]), .T(io_t[ 3]), .O(io_pins[ 3]), .IO(EXT[ 3]));
  IOBUF #(.IOSTANDARD(IOSTANDARD_EXT), .SLEW(SLEW_EXT), .DRIVE(DRIVE_EXT))
          i_iopins4 (.I(io_do[ 4]), .T(io_t[ 4]), .O(io_pins[ 4]), .IO(EXT[ 4]));
  IOBUF #(.IOSTANDARD(IOSTANDARD_EXT), .SLEW(SLEW_EXT), .DRIVE(DRIVE_EXT))
          i_iopins5 (.I(io_do[ 5]), .T(io_t[ 5]), .O(io_pins[ 5]), .IO(EXT[ 5]));
  IOBUF #(.IOSTANDARD(IOSTANDARD_EXT), .SLEW(SLEW_EXT), .DRIVE(DRIVE_EXT))
          i_iopins6 (.I(io_do[ 6]), .T(io_t[ 6]), .O(io_pins[ 6]), .IO(EXT[ 6]));
  IOBUF #(.IOSTANDARD(IOSTANDARD_EXT), .SLEW(SLEW_EXT), .DRIVE(DRIVE_EXT))
          i_iopins7 (.I(io_do[ 7]), .T(io_t[ 7]), .O(io_pins[ 7]), .IO(EXT[ 7]));
  IOBUF #(.IOSTANDARD(IOSTANDARD_EXT), .SLEW(SLEW_EXT), .DRIVE(DRIVE_EXT))
          i_iopins8 (.I(io_do[ 8]), .T(io_t[ 8]), .O(io_pins[ 8]), .IO(EXT[ 8]));
  IOBUF #(.IOSTANDARD(IOSTANDARD_EXT), .SLEW(SLEW_EXT), .DRIVE(DRIVE_EXT))
          i_iopins9 (.I(io_do[ 9]), .T(io_t[ 9]), .O(io_pins[ 9]), .IO(EXT[ 9]));
  IOBUF #(.IOSTANDARD(IOSTANDARD_EXT), .SLEW(SLEW_EXT), .DRIVE(DRIVE_EXT))
          i_iopins10(.I(io_do[10]), .T(io_t[10]), .O(io_pins[10]), .IO(EXT[10]));
  IOBUF #(.IOSTANDARD(IOSTANDARD_EXT), .SLEW(SLEW_EXT), .DRIVE(DRIVE_EXT))
          i_iopins11(.I(io_do[11]), .T(io_t[11]), .O(io_pins[11]), .IO(EXT[11]));
endmodule


