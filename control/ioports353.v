/*
** -----------------------------------------------------------------------------**
** ioports353.v
**
** I/O pads related circuitry
**
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


module dmapads (dreq0,dack0,idreq0,idack0,dreq1,dack1,idreq1,idack1);
	output	dreq0,dreq1;
	input	dack0,dack1;
	input	idreq0,idreq1;
	output	idack0,idack1;
	OBUF i_dreq0 (.I(idreq0), .O(dreq0));
   IBUF  i_dack0 (.I(dack0),  .O(idack0));
	OBUF i_dreq1 (.I(idreq1), .O(dreq1));
   IBUF      i_dack1 (.I(dack1),  .O(idack1));
endmodule




module i2cpads (sda,scl,sda_o,sda_i,sda_en,scl_o,scl_i,scl_en);
	inout		sda;
	inout		scl;
	input		sda_o;
	input		sda_en;
	output	sda_i;
	input		scl_o;
	input		scl_en;
	output	scl_i;
 IOBUF i_sda0 (.I(sda_o), .T(!sda_en), .O(sda_i), .IO(sda));
 IOBUF i_scl0 (.I(scl_o), .T(!scl_en), .O(scl_i), .IO(scl));
endmodule

module sysinterface(clk,
                    drv_bus,            // drive system bus (to write to system memory)
                    d,                  // 32 bit D[31:0] data pads
                    oe,                 // OE pad
                    ce,                 // CE pad (zero wait states)
                    ce1,                // CE1 pad (EW=1)
                    we,                 // WE pad
                    a,                  // 13 bit A[12:0] pads
                    iod,                // internal 32-bit data out (FPGA->CPU) bus
// as of v. 03533016 idi[31:16] is valid with da_*, twr_* and one cycle after,
// idi[15:0] is multiplexed: withda_*, twr_*it has di[15:0], next cycle = di[31:16]
                    idi,                // internal 32-bit data in, synchronous to clk
                    ia,                 // internal 8-bit address bus (fast - directly from I/O pads)
                    as,                 // internal 8-bit address bus (synchronized to clk)
                    am,                 // multiplexed addr - switching between ia and as
                    wnr,                // write/not read, sync with clk
                    da_ctl,             // WE to control 32-bit register, low 24 bits (1 loc)
						  da_ctl_h,           // enable writing to high 8 bits of the dcr
                    da_dmamode,         // select writing to dma_cntr/dma_raw (1 loc)
                    da_sensormode,      // select writing to sensorpix (1 loc)
                    da_virttrig,        // write virtual trigger threshold
                    da_sensortrig,      // sensor control: bit0 continuous, bit1 - external, bit2 - enable
                    da_sensortrig_lines,// write number of lines to be transferred in a frame (or aftre trigger)
                    da_dswe,            // select reading/writing to mcontr (16 locations)
                    da_init_ch3,        // write to init cnhannel 3 (will reset address r/w)
                    da_next_ch3,        // advance to the next channel3 page, reset address
                    da_mem,             // read/write to SDRAM buffer, autoincrement address
                                        // in read mode - needs CE1 to autoincrement!
						  da_lensff,          // lens flat field correction parameters (1 location, 8 bit address, 16bit - data)
                    da_hist,            // 0x40..0x47  write/read histogram related data/registers
                                        // 40 - left
                                        // 41 - top
                                        // 42 - width-1
                                        // 43 - height-1
                                        // 44 - hist. data start address (will also read pointed word to output word
                    da_hist_next,       // 45 - read histogram (with CE1 to autoincrement)
                    da_rtc,             // 48 - write microseconds (actual write will happen after writing seconds)
                                        // 49 - write seconds
                                        // 4a - write correction (16 bit, signed
                                        // 4b - nop, just latch the output 32+20 bits to read to CPU). Maybe nop (or 2 writes) are needed between read.
                    da_timestamp,       // 4c - write timesatmp mode (0 - off, 1 - normal frames, 1 photo-finish)
                    da_sens_dc,         // write clock divisor for sensor DCDC converter
                    da_interrupts,      // interrupt control 0x1c..0x1f
                    da_compressor,      // 0x0c - 0x0f - will be farther decoded in the compressor module
                    da_dcm,             // tune SDRAM clock
                    da_saturation,      // write color saturation vaues (currently 10 bits
                    da_framesync_dly,
						  da_quantizer_mode,  // Quantizer tuning - 0..7 - zero bin, 15:8 - quantizer bias
                    da_io_pins,         // write i/o pins control (for 6 pins to connector J2) - 0x70
                    da_pio_dmafifo,     // increment address of the DMA FIFO(S) (PIO mode should be enabled in the desired channel)
                    da_xjtag,           // write to external (sensor board) JTAG
                    da_extsync,         // control of external sync module 0x78 - 0x7b
						  da_extio,           // external I/O (motor control 0x7c-0x7d)
						  da_imu,             // IMU (imu r/w 0x7e-0x7f)
//                    da_imu_read,        // read any of the IMU data/status (updates memory output reg)
//                    da_imu_next,        // read IMU dataread from pointer, increment address
                    da_i2c,             // i2c_writeonly control (0x50 - 0x5f)
                    da_irq_smart,       // single IRQ control (0x1a)
                    da_sequencer,       // command sequencer (0x60..0x6f)
                    da_dcr,             // write to control registers (0x4e..0x4f), each bit/group with individual enable by data bit
                    ta,                 // [10:0] table address - will be valid at twr_xx and one cycle beforfe (@ negedge clk)
                    twr_quant,          // write enable to quantization table (@negedge clk - addr and data valid this cycle and one before)
                    twr_coring,         // coring functions tables (@negedge clk - addr and data valid this cycle and one before)
                    twr_huff,           // write enable to huffman table (@negedge clk - addr and data valid this cycle and one before)
                    twr_gamma,          // write enable to "gamma" table (@negedge clk - addr and data valid this cycle and one before)
                    twr_focus,          // write enable to "focus" table (@negedge clk - addr and data valid this cycle and one before)
                    dcmrst,             // (emergency)async DCM reset Seems SE hangs if the frequency changed on the fly
                    ioe,                // OE after IBUF
                    seq_rq,             // request from the sequencer
                    seq_ack,            // sequencer acknowledge
                    seq_a,              // address from the sequencer
                    seq_d               // data from the sequencer
                    );
    input            clk;
    input            drv_bus;
    inout	[31:0]	d;
    input				oe;
    input				ce;
    input				ce1;
    input				we;
    inout	[12:0]	a;
    input	[31:0]	iod;
    output	[31:0]	idi;
    output	 [7:0]	ia;
    output	 [7:0]	as; // output clock-synchronous address
    output   [7:0]   am;
    output           wnr;
    output  [11:0]   ta;
    output           twr_quant;
    output           twr_coring;
    output           twr_huff;
    output           twr_gamma;
	 output           twr_focus;

    output           da_ctl;				// WE to control 32-bit register (1 loc) (lower 25 bits)
    output           da_ctl_h;			// WE to control 32-bit register (1 loc) (top 8 bits)
    output           da_dmamode;			// select writing to dma_cntr/dma_raw (1 loc)
    output           da_sensormode;		// select writing to sensorpix (1 loc)
    output           da_virttrig;		// write virtual trigger threshold
    output           da_sensortrig;		// sensor control: bit0 continuous, bit1 - external, bit2 - enable
    output           da_sensortrig_lines;	// write number of lines to be transferred in a frame (or aftre trigger)
    output           da_dswe;				// select reading/writing to mcontr (16 locations)
    output           da_init_ch3;      // write to init cnhannel 3 (will reset address r/w)
    output           da_next_ch3;      // advance to the next channel3 page, reset address
    output           da_mem;				// read/write to SDRAM buffer, autoincrement address
                                       // in read mode - needs CE1 to autoincrement!
    output           da_lensff;        // lens flat field correction parameters (1 location, 8 bit address, 16bit - data)
    output           da_sens_dc;			// write clock divisor for sensor DCDC converter
    output           da_interrupts;    // interrupt mask
    output           da_compressor;    // 0x0c - 0x0f - will be farther decoded in the compressor module
    output           da_dcm;           // tune SDRAM clock
    output           da_saturation;    // write color saturation vaues (currently 10 bits
    output      		da_quantizer_mode;  // Quantizer tuning - 0..7 - zero bin, 15:8 - quantizer bias
    output           da_hist;			   // write/read histogram related data/registers
    output           da_hist_next;
    output           da_framesync_dly;
    output           da_io_pins;         // write i/o pins control (for 6 pins to connector J2) - 0x70
    
	 output           da_rtc;
    output           da_timestamp;       // 4c - write timesatmp mode (0 - off, 1 - normal frames, 1 photo-finish)
    output           dcmrst;             // async
    output           da_pio_dmafifo;     // increment address of the DMA FIFO(S) (PIO mode should be enabled in the desired channel)
    output           da_xjtag;           // write to external (sensor board) JTAG
    output           da_extsync;         // control of external sync module 0x78 - 0x7b
	 output           da_extio;           // external I/O (motor control 0x7c-0x7d)
	 output           da_imu;             // IMU    
//	 output           da_imu_read;        // read any of the IMU data/status (updates memory output reg)
//	 output           da_imu_next;        // read IMU data: read from pointer, increment address
    output           da_i2c;             // i2c_writeonly control (0x50 - 0x5f)
    output           da_irq_smart;       // single IRQ control (0x1a)
    output           da_sequencer;       // command sequencer (0x60..0x6f)
    output           da_dcr;             // write to control registers (0x4e..0x4f), each bit/group with individual enable by data bit
    output           ioe;

    input            seq_rq;             // request from the sequencer
    output           seq_ack;            // sequencer acknowledge
    input    [ 7:0]  seq_a;              // address from the sequencer
    input    [23:0]  seq_d;               // data from the sequencer
	 
    reg             dcmrst;
	 wire				cwr;
	 wire				iwe;
	 reg	[31:0]	idi;
	 wire				t;
	 wire				ioe;
	 wire				ice;
	 wire				ice1;
    wire          irnw; 
    wire [ 7:0]   ial; // enabled during oe/we high, held - during low
    reg           wnr;
    wire [31:0]   id0; //registered at the end of !cwr
    reg  [ 7:0]   ia0; //registered at the end of !cwr
    reg           irnw0;//registered at the end of !cwr
	 
    reg  [ 7:0]   as; // output clock-synchronous address
   reg            da_ctl;				// WE to control 32-bit register (1 loc)
	reg            da_ctl_h;         // enable writing to high 8 bits of the dcr
   reg            da_dmamode;			// select writing to dma_cntr/dma_raw (1 loc)
   reg            da_sensormode;		// select writing to sensorpix (1 loc)
   reg            da_virttrig;		// write virtual trigger threshold
   reg            da_sensortrig;		// sensor control: bit0 continuous, bit1 - external, bit2 - enable
   reg            da_sensortrig_lines;	// write number of lines to be transferred in a frame (or aftre trigger)
   reg            da_dswe;				// select reading/writing to mcontr (16 locations)
   reg            da_init_ch3;      // write to init cnhannel 3 (will reset address r/w)
   reg            da_next_ch3;		// advance to the next channel3 page, reset address
   reg            da_mem;				// read/write to SDRAM buffer, autoincrement address
   reg            da_lensff;        // lens flat field correction parameters (1 location, 8 bit address, 16bit - data)
	
   reg            da_sens_dc;			// write clock divisor for sensor DCDC converter
   reg            da_interrupts;    // interrupt mask
   reg            da_compressor;    // 0x0c - 0x0f - will be farther decoded in the compressor module
   reg            da_dcm;           // tune SDRAM clock phase
   reg            da_table_a;       // write table address (internal)
   reg            da_saturation;    // write color saturation values
	reg			   da_quantizer_mode;// Quantizer tuning - 0..7 - zero bin, 15:8 - quantizer bias
	
   reg            da_hist;          // write/read histogram related data/registers
   reg            da_framesync_dly;
   reg            da_io_pins;       // write i/o pins control (for 6 pins to connector J2) - 0x70
   reg            da_rtc;
   reg            da_timestamp;
   reg            da_pio_dmafifo0,     da_pio_dmafifo;   // increment address of the DMA FIFO(S) (PIO mode should be enabled in the desired channel)
   reg            da_xjtag;         // write to external (sensor board) JTAG
   reg            da_extsync;       // control of external sync module 0x78 - 0x7b
	reg            da_extio;         // external I/O (motor control 0x7c-0x7d)
	reg            da_imu;           // imu control
   reg            da_i2c;           // i2c_writeonly control (0x50 - 0x5f)
   reg            da_sequencer;     // command sequencer (0x60..0x6f)
   reg            da_dcr;           // write to control registers (0x4e..0x4f), each bit/group with individual enable by data bit
   reg            da_irq_smart;     // single IRQ control (0x1a)	
   reg            twr;              // write table data (address will be incremented 1 cycle after
   reg            da_hist_next0,       da_hist_next;     // reading histogram - ASAP, nosequencer - use sync2
//   reg            da_imu_read0,  da_imu_read; // read any of the IMU data/status (updates memory output reg)
//   reg            da_imu_next0,  da_imu_next; // read IMU dataread from pointer, increment address

   
   
   reg   [11:0]   pre_ta; // one cycle ahead of ta
   reg   [11:0]   ta;     // table address. valid with twr_* and 1 cycle after
   reg            twr_quant;
   reg            twr_coring;         // coring functions tables (@negedge clk - addr and data valid this cycle and one before)
   reg            twr_huff;
   reg            twr_gamma;
   reg            twr_focus;

   wire  [7:0]    am;
   reg            wra; // to select source of as - for 2 cycles during sync write will use as, else - ia;
// inter-clock synchronization
   wire           sync0;           // from end of the cwr low to sync2
   wire           sync1;           // sync0 registered @negedge sclk
   wire           sync2;           // sync1 registered @posedge sclk
   wire           sync_cwr_start0; // from start of the cwr low to sync_cwr_start2
	wire           sync_cwr_start1; // sync_cwr_start0 registered @negedge sclk
	wire           sync_cwr_start2; // sync_cwr_start1 registered @posedge sclk
	wire           sync_cwr_on;     // from sync_cwr_start1 to sync1, registered @posedge sclk
	reg     [7:0]  a_pio_seq_mux;   // address, multiplexed between direct/sequencer access (valid @wr_state[0])
	reg            wnr_seq_mux;     // write/not read, multiplexed between direct/sequencer access (valid @wr_state[0])
   reg    [31:0]  d_pio_seq_mux;   // data, multiplexed between direct/sequencer access (valid @wr_state[0])
	reg            seq_ack;         // using data from sequencer (first cycle)
//   reg     [2:0]  wr_state;        // 1-hot write sequence for both pio and seq.
   reg     [1:0]  wr_state;        // 1-hot write sequence for both pio and seq.
                                   // [0] - decoding address,
                                   // [1] - output low 16 (32)
                                   // [2] - output high 16 (8)

   assign   am[7:0]=    wra? as[7:0] : ia[7:0];
     IBUF  i_oe   (.I(oe),   .O(ioe ));
     IBUF  i_ce   (.I(ce),   .O(ice ));
     IBUF  i_ce1  (.I(ce1),  .O(ice1));
 	ipadql	i_we (.g(cwr),.q(iwe),.qr(irnw),.d(we));

// negative pulse - with CE (zero w.s.) - only with WE, with CE1 (EW=1) - both WE and OE  
   BUFG  i_cwr	(.I((ice | iwe)  & (ice1 | (iwe & ioe))), .O(cwr));
   wire [12:0] ao=13'b0;

   always @ (negedge clk) begin
//     wr_state[2:0] <= {wr_state[1:0], (~wr_state[0] &  ~ sync_cwr_on & seq_rq) | sync2};
     wr_state[1:0] <= {wr_state[0], (~wr_state[0] &  ~ sync_cwr_on & seq_rq) | sync2};
     seq_ack <= ~wr_state[0]  &  ~(sync_cwr_on | sync2) & seq_rq; 
	  if (sync2) a_pio_seq_mux[7:0]  <= ia0[7:0];
	  else       a_pio_seq_mux[7:0]  <= seq_a[7:0];
	  wnr_seq_mux <= !sync2 || !irnw0; // sequencer - write only
	  if (sync2) d_pio_seq_mux[31:0] <= id0[31:0];
	  else       d_pio_seq_mux[31:0] <= {8'b0,seq_d[23:0]};
   end	

   always @ (negedge clk) begin
     if (wr_state[0])      as[7:0]    <= a_pio_seq_mux[7:0];
     if (wr_state[0])      wnr        <= wnr_seq_mux;
     wra                              <= wr_state[0] && wnr_seq_mux;
	  if      (wr_state[0]) idi[31:16] <= d_pio_seq_mux[31:16];
	  if      (wr_state[0]) idi[15: 0] <= d_pio_seq_mux[15:0];
	  else if (wr_state[1]) idi[15: 0] <= idi[31:16];
   end

 // these signals will be valid after the end of the CPU r/w cycle 
   always @ (posedge cwr) begin
	  ia0[7:0]            <=  ial[7:0];
	  irnw0               <=  irnw;
     da_hist_next0       <=  irnw && (ial[7:0]==8'h45); // read from 0x45 (ASAP, no sequencer!)
//     da_imu_read0        <=  irnw && (ial[7:1]==7'h3f);  // 0x7e..0x7f, read 
//     da_imu_next0        <=  irnw && (ial[7:0]==8'h7e);  // 0x7e, read 
     dcmrst              <= !irnw && (ial[7:0]==8'h1b); // 0x1b  async signal to restart DCMs
     da_pio_dmafifo0     <=  irnw && (ial[7:1]==7'h0);  // 0x0..0x01, read 
   end

   always @ (negedge clk) begin
     da_hist_next       <= sync2 && da_hist_next0; // ASAP, no sequencer!

//     da_imu_read        <= sync2 && da_imu_read0;  // 0x7e..0x7f, read 
//     da_imu_next        <= sync2 && da_imu_next0;  // 0x7e, read 

     da_pio_dmafifo     <= sync2 && da_pio_dmafifo0;
     ta[11:0]           <= pre_ta[11:0];
   end

   always @ (negedge clk) begin
	  da_ctl_h           <= wr_state[0] && wnr_seq_mux && (a_pio_seq_mux[7:0]==8'h00);  // 0x00 WE to control 32-bit register (1 loc)
     da_dmamode         <= wr_state[0] && wnr_seq_mux && (a_pio_seq_mux[7:0]==8'h01);   // 0x01 select writing to dma_cntr/dma_raw (1 loc)
     da_sensormode      <= wr_state[0] && wnr_seq_mux && (a_pio_seq_mux[7:0]==8'h02);   // 0x02 select writing to sensorpix (1 loc)
     da_virttrig        <= wr_state[0] && wnr_seq_mux && (a_pio_seq_mux[7:0]==8'h03);  // 0x03 write virtual trigger threshold
     da_sensortrig      <= wr_state[0] && wnr_seq_mux && (a_pio_seq_mux[7:0]==8'h04);  // 0x04 select writing to sensorpix (1 loc)
     da_sensortrig_lines<= wr_state[0] && wnr_seq_mux && (a_pio_seq_mux[7:0]==8'h05);  // 0x05 write number of lines to be transferred in a frame (or aftre trigger)
     da_ctl             <= wr_state[0] && wnr_seq_mux  && ((a_pio_seq_mux[7:0]==8'h06) || (a_pio_seq_mux[7:0]==8'h00)); //0x00, 0x06 
     da_sens_dc         <= wr_state[0] && wnr_seq_mux  && (a_pio_seq_mux[7:0]==8'h07);  // 0x07 write to sensor DCDC converter frequency divider
     da_dcm             <= wr_state[0] && wnr_seq_mux  && (a_pio_seq_mux[7:0]==8'h08);  // 0x08  tune SDRAM clock phase
     da_saturation      <= wr_state[0] && wnr_seq_mux  && (a_pio_seq_mux[7:0]==8'h09);  // 0x09  write color saturation values
     da_framesync_dly   <= wr_state[0] && wnr_seq_mux  && (a_pio_seq_mux[7:0]==8'h0a);  // 0x0a  write frame sync interrupt delay (in scan lines)
     da_quantizer_mode  <= wr_state[0] && wnr_seq_mux  && (a_pio_seq_mux[7:0]==8'h0b);  // 0x0b  Quantizer tuning - 0..7 - zero bin, 15:8 - quantizer bias
     da_compressor      <= wr_state[0] && wnr_seq_mux  && (a_pio_seq_mux[7:1]==7'h06);  // 0x0c..0x0d - will be farther decoded in the compressor module
     da_table_a         <= wr_state[0] && wnr_seq_mux  && (a_pio_seq_mux[7:0]==8'h0e);  // 0x0e - write tables address
     twr                <= wr_state[0] && wnr_seq_mux  && (a_pio_seq_mux[7:0]==8'h0f);  // 0x0f - write tables data
     da_irq_smart       <= wr_state[0]  && (a_pio_seq_mux[7:0]==8'h1a);  // 0x1a  single IRQ control
     da_interrupts      <= wr_state[0]  && (a_pio_seq_mux[7:2]==6'h07);  // 0x1c - 0x1f interrup control
     da_dswe            <= wr_state[0]  && (a_pio_seq_mux[7:4]==4'h2 );  // 0x2x select reading/writing to mcontr (16 locations)
     da_init_ch3        <= wr_state[0]  && (a_pio_seq_mux[7:0]==8'h2c);  // 0x2c write to init cnhannel 3 (will reset address r/w)
     da_next_ch3        <= wr_state[0]  && (a_pio_seq_mux[7:0]==8'h2f);  // 0x2f advance to the next channel3 page, reset address
     da_mem             <= wr_state[0]  && (a_pio_seq_mux[7:0]==8'h30);  // 0x30 read/write to SDRAM buffer, autoincrement address
     da_lensff          <= wr_state[0]  && (a_pio_seq_mux[7:0]==8'h31);  // 0x31 lens flat field correction parameters (1 location, 8 bit address, 16bit - data)
     twr_quant          <= wr_state[0]  && (a_pio_seq_mux[7:0]==8'h0f) && (pre_ta[11: 9]  ==3'h0); // 3'h0
     twr_huff           <= wr_state[0]  && (a_pio_seq_mux[7:0]==8'h0f) && (pre_ta[11: 9]  ==3'h1); // 3'h1
     twr_gamma          <= wr_state[0]  && (a_pio_seq_mux[7:0]==8'h0f) && (pre_ta[11:10]  ==2'h1); // 3'h2..3'h3
     twr_focus          <= wr_state[0]  && (a_pio_seq_mux[7:0]==8'h0f) && (pre_ta[11:10]  ==2'h2); // 3'h4..3'h5
     twr_coring         <= wr_state[0]  && (a_pio_seq_mux[7:0]==8'h0f) && (pre_ta[11: 9]  ==3'h6); // 3'h6 (3'h7 spare)
     
     if      (da_table_a)       pre_ta[11:0]<= idi[11:0]; //d_pio_seq_mux[11:0];
     else if (twr)              pre_ta[11:0]<= pre_ta[11:0]+1;
     da_hist            <= wr_state[0]  && (a_pio_seq_mux[7:3]==5'h08);  // 0x40..0x47write/read histogram related data/registers
     da_rtc             <= wr_state[0]  && (a_pio_seq_mux[7:2]==6'h12);  // 0x48..0x4b
     da_timestamp       <= wr_state[0]  && (a_pio_seq_mux[7:0]==8'h4c);  // 0x4c  write timestamp mode
//     da_dcr             <= wr_state[0]  && (a_pio_seq_mux[7:1]==7'h27);  // 0x4e - 0x4f write to control registers, each bit/group with individual enable by data bit
// now da_dcr overlaps with 0x4c (timestamp mode), will be decoded in the control_regs module
     da_dcr             <= wr_state[0]  && (a_pio_seq_mux[7:2]==6'h13);  // 0x4c - 0x4f write to control registers, each bit/group with individual enable by data bit
     da_i2c             <= wr_state[0]  && (a_pio_seq_mux[7:4]==4'h5);   // 0x50 - 0x5f i2c_writeonly control
     da_sequencer       <= wr_state[0]  && (a_pio_seq_mux[7:4]==4'h6);   // 0x60 - 0x6f command sequencer
     da_io_pins         <= wr_state[0]  && (a_pio_seq_mux[7:0]==8'h70);  // 0x70  write i/o pins control (for 6 pins to connector J2) - 0x70
     da_xjtag           <= wr_state[0]  && (a_pio_seq_mux[7:0]==8'h74);  // 0x74 write to external (sensor board) JTAG
     da_extsync         <= wr_state[0]  && (a_pio_seq_mux[7:2]==6'h1e);  // 0x78 - 0x7b control of external sync module 
// TODO: add "wnr_seq_mux &&" where appropriate, otherwise pulse is generated on read with CE1 ! *****************      
     da_extio           <= wr_state[0]  && wnr_seq_mux && (a_pio_seq_mux[7:1]==7'h3e);    // external I/O (motor control 0x7c-0x7d)
     da_imu             <= wr_state[0]  && wnr_seq_mux && (a_pio_seq_mux[7:1]==7'h3f);    // IMU control (0x7e-0x7f)
   end

 	bpadql	i_a0 (.g(cwr),.q(ia[ 0]),.qr(ial[ 0]),.io(a[ 0]),.t(!drv_bus),.d(ao[ 0]));
 	bpadql	i_a1 (.g(cwr),.q(ia[ 1]),.qr(ial[ 1]),.io(a[ 1]),.t(!drv_bus),.d(ao[ 1]));
 	bpadql	i_a2 (.g(cwr),.q(ia[ 2]),.qr(ial[ 2]),.io(a[ 2]),.t(!drv_bus),.d(ao[ 2]));
 	bpadql	i_a3 (.g(cwr),.q(ia[ 3]),.qr(ial[ 3]),.io(a[ 3]),.t(!drv_bus),.d(ao[ 3]));
 	bpadql	i_a4 (.g(cwr),.q(ia[ 4]),.qr(ial[ 4]),.io(a[ 4]),.t(!drv_bus),.d(ao[ 4]));
 	bpadql	i_a5 (.g(cwr),.q(ia[ 5]),.qr(ial[ 5]),.io(a[ 5]),.t(!drv_bus),.d(ao[ 5]));
 	bpadql	i_a6 (.g(cwr),.q(ia[ 6]),.qr(ial[ 6]),.io(a[ 6]),.t(!drv_bus),.d(ao[ 6]));
 	bpadql	i_a7 (.g(cwr),.q(ia[ 7]),.qr(ial[ 7]),.io(a[ 7]),.t(!drv_bus),.d(ao[ 7]));
 	bpadql	i_a8 (.g(cwr),.q(),      .qr(),       .io(a[ 8]),.t(!drv_bus),.d(ao[ 8]));
 	bpadql	i_a9 (.g(cwr),.q(),      .qr(),       .io(a[ 9]),.t(!drv_bus),.d(ao[ 9]));
 	bpadql	i_a10(.g(cwr),.q(),      .qr(),       .io(a[10]),.t(!drv_bus),.d(ao[10]));
 	bpadql	i_a11(.g(cwr),.q(),      .qr(),       .io(a[11]),.t(!drv_bus),.d(ao[11]));
 	bpadql	i_a12(.g(cwr),.q(),      .qr(),       .io(a[12]),.t(!drv_bus),.d(ao[12]));

// inter-clock synchronization
   FDCE i_sync0             (.Q(sync0),          .C(cwr),.CE(1'b1),.CLR(sync2),.D(1'b1));
   FD_1 i_sync1             (.Q(sync1),          .C(clk),.D(sync0 && ! sync1));
   FD   i_sync2             (.Q(sync2),          .C(clk),.D(sync1));
   FDCE_1 i_sync_cwr_start0 (.Q(sync_cwr_start0),.C(cwr),.CE(1'b1),.CLR(sync_cwr_start2),.D(1'b1));
   FD_1 i_sync_cwr_start1   (.Q(sync_cwr_start1),.C(clk),.D(sync_cwr_start0 && ! sync_cwr_start1));
   FD   i_sync_cwr_start2   (.Q(sync_cwr_start2),.C(clk),.D(sync_cwr_start1));
   FD   i_sync_cwr_on       (.Q(sync_cwr_on),    .C(clk),.D(sync_cwr_start1 || (sync_cwr_on && !sync1)));

   LUT4 #(.INIT(16'hAA80)) i_dataouten ( .I0(1'b1), .I1(ice1), .I2(ice), .I3(ioe), .O(t));
 dpads32 i_dmapads32(.c(cwr),.t(t),.d(iod[31:0]),.q(id0[31:0]),.dq(d[31:0]));
endmodule



module dpads32(c,t,d,q,dq);
   input c,t;
   input  [31:0] d;
   output [31:0] q;
   inout  [31:0] dq;
   wire t0, t1;
// s---ynthesis attribute KEEP_HIERARCHY of i_t0 is true
// s---ynthesis attribute KEEP_HIERARCHY of i_t1 is true
 BUF i_t0   (.I(t), .O(t0));
 BUF i_t1   (.I(t), .O(t1));
 	dio1	i_d0  (.c(c),.t(t0),.d(d[ 0]),.q(q[ 0]),.dq(dq[ 0]));
	dio1	i_d1  (.c(c),.t(t0),.d(d[ 1]),.q(q[ 1]),.dq(dq[ 1]));
	dio1	i_d2  (.c(c),.t(t0),.d(d[ 2]),.q(q[ 2]),.dq(dq[ 2]));
	dio1	i_d3  (.c(c),.t(t0),.d(d[ 3]),.q(q[ 3]),.dq(dq[ 3]));
	dio1	i_d4  (.c(c),.t(t0),.d(d[ 4]),.q(q[ 4]),.dq(dq[ 4]));
	dio1	i_d5  (.c(c),.t(t0),.d(d[ 5]),.q(q[ 5]),.dq(dq[ 5]));
	dio1	i_d6  (.c(c),.t(t0),.d(d[ 6]),.q(q[ 6]),.dq(dq[ 6]));
	dio1	i_d7  (.c(c),.t(t0),.d(d[ 7]),.q(q[ 7]),.dq(dq[ 7]));
	dio1	i_d8  (.c(c),.t(t0),.d(d[ 8]),.q(q[ 8]),.dq(dq[ 8]));
	dio1	i_d9  (.c(c),.t(t0),.d(d[ 9]),.q(q[ 9]),.dq(dq[ 9]));
	dio1	i_d10 (.c(c),.t(t1),.d(d[10]),.q(q[10]),.dq(dq[10]));
	dio1	i_d11 (.c(c),.t(t1),.d(d[11]),.q(q[11]),.dq(dq[11]));
	dio1	i_d12 (.c(c),.t(t1),.d(d[12]),.q(q[12]),.dq(dq[12]));
	dio1	i_d13 (.c(c),.t(t1),.d(d[13]),.q(q[13]),.dq(dq[13]));
	dio1	i_d14 (.c(c),.t(t1),.d(d[14]),.q(q[14]),.dq(dq[14]));
	dio1	i_d15 (.c(c),.t(t1),.d(d[15]),.q(q[15]),.dq(dq[15]));
	dio1	i_d16 (.c(c),.t(t1),.d(d[16]),.q(q[16]),.dq(dq[16]));
	dio1	i_d17 (.c(c),.t(t0),.d(d[17]),.q(q[17]),.dq(dq[17]));
	dio1	i_d18 (.c(c),.t(t1),.d(d[18]),.q(q[18]),.dq(dq[18]));
	dio1	i_d19 (.c(c),.t(t1),.d(d[19]),.q(q[19]),.dq(dq[19]));
	dio1	i_d20 (.c(c),.t(t1),.d(d[20]),.q(q[20]),.dq(dq[20]));
	dio1	i_d21 (.c(c),.t(t1),.d(d[21]),.q(q[21]),.dq(dq[21]));
	dio1	i_d22 (.c(c),.t(t0),.d(d[22]),.q(q[22]),.dq(dq[22]));
	dio1	i_d23 (.c(c),.t(t1),.d(d[23]),.q(q[23]),.dq(dq[23]));
	dio1	i_d24 (.c(c),.t(t0),.d(d[24]),.q(q[24]),.dq(dq[24]));
	dio1	i_d25 (.c(c),.t(t1),.d(d[25]),.q(q[25]),.dq(dq[25]));
	dio1	i_d26 (.c(c),.t(t0),.d(d[26]),.q(q[26]),.dq(dq[26]));
	dio1	i_d27 (.c(c),.t(t0),.d(d[27]),.q(q[27]),.dq(dq[27]));
	dio1	i_d28 (.c(c),.t(t0),.d(d[28]),.q(q[28]),.dq(dq[28]));
	dio1	i_d29 (.c(c),.t(t0),.d(d[29]),.q(q[29]),.dq(dq[29]));
	dio1	i_d30 (.c(c),.t(t1),.d(d[30]),.q(q[30]),.dq(dq[30]));
	dio1	i_d31 (.c(c),.t(t1),.d(d[31]),.q(q[31]),.dq(dq[31]));

endmodule


module sddrio16(c0,/*c90,*/c270,d,t,q,dq);  //added an extra FF for the t signal
    input c0,/*c90,*/c270;
    input [31:0] d;
    input t;
    output [31:0] q;
    inout [15:0] dq;
    wire [31:0] q;
    sddrio0 i_dq0  (.c0(c0),/*.c90(c90),*/.c270(c270),.d({d[16],d[ 0]}),.t(t),.q({q[16],q[ 0]}),.dq(dq[ 0]));
    sddrio0 i_dq1  (.c0(c0),/*.c90(c90),*/.c270(c270),.d({d[17],d[ 1]}),.t(t),.q({q[17],q[ 1]}),.dq(dq[ 1]));
    sddrio0 i_dq2  (.c0(c0),/*.c90(c90),*/.c270(c270),.d({d[18],d[ 2]}),.t(t),.q({q[18],q[ 2]}),.dq(dq[ 2]));
    sddrio0 i_dq3  (.c0(c0),/*.c90(c90),*/.c270(c270),.d({d[19],d[ 3]}),.t(t),.q({q[19],q[ 3]}),.dq(dq[ 3]));
    sddrio0 i_dq4  (.c0(c0),/*.c90(c90),*/.c270(c270),.d({d[20],d[ 4]}),.t(t),.q({q[20],q[ 4]}),.dq(dq[ 4]));
    sddrio0 i_dq5  (.c0(c0),/*.c90(c90),*/.c270(c270),.d({d[21],d[ 5]}),.t(t),.q({q[21],q[ 5]}),.dq(dq[ 5]));
    sddrio0 i_dq6  (.c0(c0),/*.c90(c90),*/.c270(c270),.d({d[22],d[ 6]}),.t(t),.q({q[22],q[ 6]}),.dq(dq[ 6]));
    sddrio0 i_dq7  (.c0(c0),/*.c90(c90),*/.c270(c270),.d({d[23],d[ 7]}),.t(t),.q({q[23],q[ 7]}),.dq(dq[ 7]));
    sddrio0 i_dq8  (.c0(c0),/*.c90(c90),*/.c270(c270),.d({d[24],d[ 8]}),.t(t),.q({q[24],q[ 8]}),.dq(dq[ 8]));
    sddrio0 i_dq9  (.c0(c0),/*.c90(c90),*/.c270(c270),.d({d[25],d[ 9]}),.t(t),.q({q[25],q[ 9]}),.dq(dq[ 9]));
    sddrio0 i_dq10 (.c0(c0),/*.c90(c90),*/.c270(c270),.d({d[26],d[10]}),.t(t),.q({q[26],q[10]}),.dq(dq[10]));
    sddrio0 i_dq11 (.c0(c0),/*.c90(c90),*/.c270(c270),.d({d[27],d[11]}),.t(t),.q({q[27],q[11]}),.dq(dq[11]));
    sddrio0 i_dq12 (.c0(c0),/*.c90(c90),*/.c270(c270),.d({d[28],d[12]}),.t(t),.q({q[28],q[12]}),.dq(dq[12]));
    sddrio0 i_dq13 (.c0(c0),/*.c90(c90),*/.c270(c270),.d({d[29],d[13]}),.t(t),.q({q[29],q[13]}),.dq(dq[13]));
    sddrio0 i_dq14 (.c0(c0),/*.c90(c90),*/.c270(c270),.d({d[30],d[14]}),.t(t),.q({q[30],q[14]}),.dq(dq[14]));
    sddrio0 i_dq15 (.c0(c0),/*.c90(c90),*/.c270(c270),.d({d[31],d[15]}),.t(t),.q({q[31],q[15]}),.dq(dq[15]));
// s---ynthesis attribute KEEP_HIERARCHY of i_dq0  is "TRUE"
// s---ynthesis attribute KEEP_HIERARCHY of i_dq1  is "TRUE"
// s---ynthesis attribute KEEP_HIERARCHY of i_dq2  is "TRUE"
// s---ynthesis attribute KEEP_HIERARCHY of i_dq3  is "TRUE"
// s---ynthesis attribute KEEP_HIERARCHY of i_dq4  is "TRUE"
// s---ynthesis attribute KEEP_HIERARCHY of i_dq5  is "TRUE"
// s---ynthesis attribute KEEP_HIERARCHY of i_dq6  is "TRUE"
// s---ynthesis attribute KEEP_HIERARCHY of i_dq7  is "TRUE"
// s---ynthesis attribute KEEP_HIERARCHY of i_dq8  is "TRUE"
// s---ynthesis attribute KEEP_HIERARCHY of i_dq9  is "TRUE"
// s---ynthesis attribute KEEP_HIERARCHY of i_dq10 is "TRUE"
// s---ynthesis attribute KEEP_HIERARCHY of i_dq11 is "TRUE"
// s---ynthesis attribute KEEP_HIERARCHY of i_dq12 is "TRUE"
// s---ynthesis attribute KEEP_HIERARCHY of i_dq13 is "TRUE"
// s---ynthesis attribute KEEP_HIERARCHY of i_dq14 is "TRUE"
// s---ynthesis attribute KEEP_HIERARCHY of i_dq15 is "TRUE"
endmodule
// Made for CL=2.5
// all data to write is expected to be sync to posedge of c0 - phase=0,
// data to sdram is clocked at c270 (LSW) and c90 (MSW)
// MSB will be delayed by half-cycle internally
// tristate will be clocked at rising edge of c270
// All data read will be also sync to rising edge of c0 (LSB will be delayed internally)

module sddrio0(c0,/*c90,*/c270,d,t,q,dq); // made for CL=2.5, LSB first - c0 falling edge is before rising, gets LSB
    input       c0,/*c90,*/c270;
    input  [1:0] d;
    input        t;
    output [1:0] q;
    inout        dq;

  wire dr,t0,t1,tr,qp,q00,d1d;
  wire [1:0] d0;

  FD    i_d00 (.C(c0),  .D(d[0]), .Q(d0[0]));  //regular FF, not IOB
  FD    i_d01 (.C(c0),  .D(d[1]), .Q(d0[1]));  //regular FF, not IOB
  FD    i_d1d (.C(c270),.D(d0[1]),.Q(d1d));    //regular FF, not IOB

  FD_1 i_q0  (.C(c0),.D(q00),.Q(q[0]));  //regular FF, not IOB
  IOBUF i_dq (.I(dr), .T(tr),.O(qp), .IO(dq));
  FDDRCPE i_dr (.Q(dr),.C0(c270),.C1(!c270),.D0(d0[0]),.D1(d1d),.CE(1'b1),.CLR(1'b0),.PRE(1'b0));
  FD_1 #(.INIT(1'b1)) i_t0 (.C(c0), .D(t), .Q(t0));
  FD  #(.INIT(1'b1))  i_t1 (.C(c0), .D(t0), .Q(t1));
  FD  #(.INIT(1'b1))  i_tr (.C(c270), .D(t1), .Q(tr));
  IDDR2 i_qq(.Q0(q00),.Q1(q[1]),.C0(c0),.C1(!c0),.CE(1'b1), .D(qp), .R(1'b0), .S(1'b0) );	 

// synthesis attribute IOB of i_dr is "TRUE"
// synthesis attribute IOB of i_tr is "TRUE"
// synthesis attribute NODELAY of i_dq is "TRUE"
endmodule


module dqs2 (c0,/*c90,*/c270,
             t,          // 1.5 cycles before cmd "write" sent out to the SDRAM, sync to sclk180
             UDQS,        // UDQS I/O pin
             LDQS,        // LDQS I/O pin
             udqsr90,    // data from SDRAM interface pin UDQS strobed at rising sclk90
             ldqsr90,    // data from SDRAM interface pin LDQS strobed at rising sclk90
             udqsr270,   // data from SDRAM interface pin UDQS strobed at rising sclk270
             ldqsr270   // data from SDRAM interface pin UDQS strobed at rising sclk270
             );
    input c0,/*c90,*/c270,t;
    inout UDQS, LDQS;
    output udqsr90,ldqsr90,udqsr270,ldqsr270;
    wire  t0,t1,tr;
///AF:      wire  t2; 
    FD_1    #(.INIT(1'b1)) i_t0 (.C(c0),.D(t),.Q(t0));
    FD      #(.INIT(1'b1)) i_t1 (.C(c0),.D(t0),.Q(t1));
///AF:      FD      #(.INIT(1'b1)) i_t2 (.C(c270),.D(t0),.Q(t2));
    assign tr= t1;
    dqs2_0 i_dqsu(.c0(c0),/*.c90(c90),*/.c270(c270),.t(tr),.q({udqsr270,udqsr90}),.dq(UDQS));
    dqs2_0 i_dqsl(.c0(c0),/*.c90(c90),*/.c270(c270),.t(tr),.q({ldqsr270,ldqsr90}),.dq(LDQS));
    
endmodule

module dqs2_0(c0,/*c90,*/c270,t,q,dq);
    input       c0,/*c90,*/c270;
    input        t;
    output [1:0] q;
    inout        dq;

  wire qp;
  wire virtc0; // sync to c0

  IOBUF i_dq (.I(virtc0), .T(t),.O(qp), .IO(dq));

// reset DQS when tristated
  FDDRCPE i_dr (.Q(virtc0),.C0(c0),.C1(!c0),.D0(1'b1),.D1(1'b0),.CE(1'b1),.CLR(t),.PRE(1'b0));




// as in  IFDDRCPE.v
    FDCPE_1  #(.INIT(1'b0)) i_q0 (.C(c270), .CE(1'b1),.CLR(1'b0),.D(qp),.PRE(1'b0),.Q(q[0]));
    FDCPE                   i_q1 (.C(c270),.CE(1'b1),.CLR(1'b0),.D(qp),.PRE(1'b0),.Q(q[1]));
// synthesis attribute IOB of i_q0 is "TRUE"
// synthesis attribute IOB of i_q1 is "TRUE"
// synthesis attribute FAST of i_dq is "TRUE"
// synthesis attribute NODELAY of i_dq is "TRUE"


endmodule

//both bits are strobed at rising c270
module sddrdm(c0,/*c90,*/c270,d,dq);
    input       c0,/*c90,*/c270;
    input  [1:0] d;
    inout        dq; //SuppressThisWarning Veditor UNUSED
sddrdm0 i_dq (.c0(c0),/*.c90(c90),*/.c270(c270),.d(d),.dq(dq));
// s--ynthesis attribute KEEP_HIERARCHY of i_dq is "TRUE"
endmodule

module sddrdm0(c0,/*c90,*/c270,d,dq);
    input       c0,/*c90,*/c270;
    input  [1:0] d;
    output      dq;

  wire dr,d1d;
  wire [1:0] d0;
  OBUF i_dq (.I(dr), .O(dq));
//  FDDRCPE i_dr (.Q(dr),.C0(c270),.C1(c90),.D0(d0[0]),.D1(d1d),.CE(1'b1),.CLR(1'b0),.PRE(1'b0));
  FDDRCPE i_dr (.Q(dr),.C0(c270),.C1(!c270),.D0(d0[0]),.D1(d1d),.CE(1'b1),.CLR(1'b0),.PRE(1'b0));
  FD    i_d00(.C(c0),.D(d[0]),.Q(d0[0])); //regular FF, not IOB
  FD    i_d01(.C(c0),.D(d[1]),.Q(d0[1])); //regular FF, not IOB
  FD    i_d1d(.C(c270),.D(d0[1]),.Q(d1d)); //regular FF, not IOB
// synthesis attribute IOB of i_dr is "TRUE"
// synthesis attribute NODELAY of i_dq is "TRUE"
endmodule




// SDRAM address and ras/cas/we
module sdo15_2(c,d,q);		// inputs at rising edge, resyncs to falling edge, all go high at reset
    input c;
    input  [14:0] d;
    output [14:0] q;
 sdo1_2 i_q0  (.c(c),.d(d[ 0]),.q(q[ 0]));
 sdo1_2 i_q1  (.c(c),.d(d[ 1]),.q(q[ 1]));
 sdo1_2 i_q2  (.c(c),.d(d[ 2]),.q(q[ 2]));
 sdo1_2 i_q3  (.c(c),.d(d[ 3]),.q(q[ 3]));
 sdo1_2 i_q4  (.c(c),.d(d[ 4]),.q(q[ 4]));
 sdo1_2 i_q5  (.c(c),.d(d[ 5]),.q(q[ 5]));
 sdo1_2 i_q6  (.c(c),.d(d[ 6]),.q(q[ 6]));
 sdo1_2 i_q7  (.c(c),.d(d[ 7]),.q(q[ 7]));
 sdo1_2 i_q8  (.c(c),.d(d[ 8]),.q(q[ 8]));
 sdo1_2 i_q9  (.c(c),.d(d[ 9]),.q(q[ 9]));
 sdo1_2 i_q10 (.c(c),.d(d[10]),.q(q[10]));
 sdo1_2 i_q11 (.c(c),.d(d[11]),.q(q[11]));
 sdo1_2 i_q12 (.c(c),.d(d[12]),.q(q[12]));
 sdo1_2 i_q13 (.c(c),.d(d[13]),.q(q[13]));
 sdo1_2 i_q14 (.c(c),.d(d[14]),.q(q[14]));
endmodule

module sdo1_2(c,d,q); // input at rising edge, resyncs to falling
    input c;
    input  d;
    output q;
 sdo0_2	i_q  (.c(c),.d(d),.q(q));
// s--ynthesis attribute KEEP_HIERARCHY of i_q  is "TRUE"
endmodule

module sdo0_2(c,d,q); // input at rising edge, resyncs to falling, initializes to "1"
    input c;
    input d;
    output q;
wire d0, dr;
OBUF i_q  (.I(dr), .O(q));
FD   #(.INIT(1'b1)) i_d0   (.C(c), .D(d), .Q(d0));
//FD_1 i_dr (.C(c), .D(d), .Q(dr));
FD_1 #(.INIT(1'b1)) i_dr (.C(c), .D(d0), .Q(dr));
// synthesis attribute IOB of i_dr is "TRUE"

endmodule


module ipadql(g,q,qr,d);  //
    input g;
    output q;
    output qr;
    input d;
	ipadql0 i_q (.g(g),.q(q),.qr(qr),.d(d));
// s--ynthesis attribute KEEP_HIERARCHY of i_q is "TRUE"
endmodule


module ipadql0(g,q,qr,d);
    input g;
    output q;
    output qr;
    input d;
  IBUF  i_q (.I(d), .O(q));
  LD	  i_qr (.G(g), .D(q), .Q(qr));
// synthesis attribute IOB of i_qr is "TRUE"
// synthesis attribute NODELAY of i_q is "TRUE"

endmodule

module bpadql(g,q,qr,io,t,d);  //
    input g;
    output q;
    output qr;
    inout io;
    input t;
    input d;
	bpadql0 i_q (.g(g),.q(q),.qr(qr),.io(io),.t(t),.d(d));
// s--ynthesis attribute KEEP_HIERARCHY of i_q is "TRUE"
endmodule


module bpadql0(g,q,qr,io,t,d);
    input g;
    output q;
    output qr;
    inout io;
    input t;
    input d;
  IOBUF i_q (.I(d), .T(t),.O(q), .IO(io));
  LD	  i_qr (.G(g), .D(q), .Q(qr));
// synthesis attribute IOB of i_qr is "TRUE"
// synthesis attribute NODELAY of i_q is "TRUE"

endmodule



module dio1(c,t,d,q,dq);
    input c;
    input t;
    input d;
    output q;
    inout dq;
	dio0 i_dq (.c(c),.t(t),.d(d),.q(q),.dq(dq));
// s--ynthesis attribute KEEP_HIERARCHY of i_dq is "TRUE"
endmodule


module dio0(c,t,d,q,dq);
    input c;
    input t;
    input d;
    output q;
    inout dq;

	 wire q0;

  IOBUF i_dq (.I(d), .T(t),.O(q0), .IO(dq));
  FD    i_q  (.C(c), .D(q0), .Q(q));
// synthesis attribute IOB of i_q is "TRUE"
// synthesis attribute NODELAY of i_dq is "TRUE"

endmodule

