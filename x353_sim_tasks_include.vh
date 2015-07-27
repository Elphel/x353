/*******************************************************************************
 * Include file: x353_sim_tasks_include.vh
 * Date:2015-07-26  
 * Author: Andrey Filippov     
 * Description: Moved here all simulation tasks except the IMU logger 
 *
 * Copyright (c) 2015 Elphel, Inc .
 * x353_sim_tasks_include.vh is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 *  x353_sim_tasks_include.vh is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/> .
 *******************************************************************************/
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
 