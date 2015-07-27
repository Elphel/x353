/*
** -----------------------------------------------------------------------------**
** lens_flat.v
**
** Correction of lens+sensor vignetting. Initially it is just a quadratic function
** that can be improved later by a piece-linear table function T() of the calculated
** f(x,y)=p*(x-x0)^2 + q(y-yo)^2 + c.
** T(f(x,y)) can be used to approximate cos^4). or other vignetting functions
** 
** This function - f(x,y) or T(f(x,y)) here deal with full sensor data before 
** gamma-tables are applied and the data is compressed to 8 bits
**
** Copyright (C) 2008 Elphel, Inc
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
/*
F2(x,y)=p*(x-x0)^2 + q(y-yo)^2 + c=
       p*x^2 - (2*p*x0) * x + p* (x0*x0) + q*y^2 - (2*q*y0) * y + q* (y0*y0) + c=
       p* x^2 - (2*p*x0) * x + q* y^2 -(2*q)* y + (p* (x0*x0)+q* (y0*y0) + c)
Final:
F2(X,Y)=p* x^2 - (2*p*x0) * x + q* y^2 -(2*q)* y + (p* (x0*x0)+q* (y0*y0) + c):
Ax(Y)= p
Bx(Y)=-(2*p)
F(0,Y)= q*y^2 - (2*q*y0) * y + (q* (y0*y0) + c + p* (x0*x0))
C=  (q* (y0*y0) + c + p* (x0*x0));
BY= - (2*q*y0)
AY= q
AX= p
BX= -2*p*x0
*/

`timescale 1ns / 1ps
module lens_flat (sclk,      /// system clock @negedge
                  wen,       /// write LSW from di
                  di,        /// [15:0] data in
                  pclk,      /// pixel clock (@pclk)
                  fstart,    // frame start - single clock (will have frame latency as coefficients are written after the fstart)
                  newline,   // start of scan line  - ahead of linerun
                  linerun,   // active pixel output - latency will be = 4 clocks
                  bayer,
                  pixdi,     // pixel data in,  16 bit unsigned
                  pixdo      // pixel data out, 16 bit unsigned
                  );

    input         sclk;
    input         wen;
    input  [15:0] di;
    input         pclk;
    input         fstart;
    input         newline;
    input         linerun;
    input  [1:0]  bayer;
    input [15:0]  pixdi;
    output[15:0]  pixdo;
//    output [18:0] lens_corr;

    reg    [ 1:0]  wen_d;
    reg    [23:0] did;
    reg    [23:0] didd;
//    reg           we_AA,we_AB,we_AC,we_BA,we_BB,we_BC,we_CA,we_CB,we_CC;
    reg           we_AX,we_BX,we_AY,we_BY,we_C;
    reg           we_scales;/// write additional individual per-color scales (17 bits each)
    reg           we_fatzero_in,we_fatzero_out; ///
    reg           we_post_scale;
//F(x,y)=Ax*x^2+Bx*x+Ay*y^2+By*y+C
    reg    [18:0] AX; /// Ax
    reg    [18:0] AY; /// Ax
    reg    [20:0] BX; /// Bx
    reg    [20:0] BY; /// By
    reg    [18:0] C;  /// C
    reg    [16:0] scales[0:3]; // per-color coefficients
///AF:      reg    [16:0] scales_r;
    reg    [15:0] fatzero_in;     /// zero level to subtract before multiplication
    reg    [15:0] fatzero_out;    /// zero level to add after multiplication
    reg    [ 3:0] post_scale;     /// shift product after first multiplier - maybe needed when using decimation

    wire   [18:0] FY;    /// F(0,y)
    wire   [23:0] ERR_Y; /// running error for the first column
    wire   [18:0] FXY;   /// F(x,y)
///AF:      reg    [18:0] FXY_sat;
    reg   [ 4:0] lens_corr_out; /// lens correction out valid (first clock from column0 )
/// copied form sensorpix353.v
    reg           bayer_nset; 
    reg           bayer0_latched;
    reg    [1:0]  color;
    wire  [35:0]  mult_first_res;
    reg   [17:0]  mult_first_scaled; /// scaled multiplication result (to use with decimation to make parabola 'sharper')
    wire  [35:0]  mult_second_res;
    reg   [15:0]  pixdo;           /// output pixel data, 16 bits, saturated at positive

    wire  [20:0]  pre_pixdo_with_zero= mult_second_res[35:15] + {{5{fatzero_out[15]}},fatzero_out[15:0]};


    wire          sync_bayer=linerun && ~lens_corr_out[0];
    wire   [17:0] pix_zero = {2'b0,pixdi[15:0]}-{{2{fatzero_in [15]}},fatzero_in [15:0]};

    always @ (negedge sclk) begin
      wen_d[1:0]   <= {wen_d[0],wen};
      if (wen)      did[15: 0] <= di[15:0];
      if (wen_d[0]) did[23:16] <= di[ 7:0];
      didd[23:0] <= did[23:0];
      we_AX          <= wen_d[1] && (did[23:19]==5'h00); /// 00000
      we_AY          <= wen_d[1] && (did[23:19]==5'h01); /// 00001
      we_C           <= wen_d[1] && (did[23:19]==5'h02); /// 00010
      we_BX          <= wen_d[1] && (did[23:21]==3'h1 ); /// 001
      we_BY          <= wen_d[1] && (did[23:21]==3'h2 ); /// 010
      we_scales      <= wen_d[1] && (did[23:19]==5'h0c); /// 01100NN
      we_fatzero_in  <= wen_d[1] && (did[23:16]==8'h68); /// 01101000
      we_fatzero_out <= wen_d[1] && (did[23:16]==8'h69); /// 01101001
      we_post_scale  <= wen_d[1] && (did[23:16]==8'h6a); /// 01101010

      if (we_AX)  AX[18:0] <= didd[18:0];
      if (we_AY)  AY[18:0] <= didd[18:0];
      if (we_BX)  BX[20:0] <= didd[20:0];
      if (we_BY)  BY[20:0] <= didd[20:0];
      if (we_C)    C[18:0] <= didd[18:0];

      if (we_scales) scales[didd[18:17]] <= didd[16:0];
      if (we_fatzero_in)  fatzero_in [15:0] <= didd[15:0];
      if (we_fatzero_out) fatzero_out[15:0] <= didd[15:0];
      if (we_post_scale)  post_scale [ 3:0] <= didd[ 3:0];
    end
//reg color[1:0]
    always @ (posedge pclk) begin
      lens_corr_out[4:0]<={lens_corr_out[3:0],linerun};
      bayer_nset <= !fstart && (bayer_nset || linerun);
      bayer0_latched<= bayer_nset? bayer0_latched:bayer[0];
      color[1:0] <=  { bayer_nset? (sync_bayer ^ color[1]):bayer[1] ,
                   (bayer_nset &&(~sync_bayer))?~color[0]:bayer0_latched };

/// now scale the result (normally post_scale[2:0] ==1)
      case (post_scale [2:0])
        3'h0:mult_first_scaled[17:0]<=  (~mult_first_res[35] & |mult_first_res[34:33]) ? 18'h1ffff:mult_first_res[33:16]; /// only limit positive overflow
        3'h1:mult_first_scaled[17:0]<=  (~mult_first_res[35] & |mult_first_res[34:32]) ? 18'h1ffff:mult_first_res[32:15];
        3'h2:mult_first_scaled[17:0]<=  (~mult_first_res[35] & |mult_first_res[34:31]) ? 18'h1ffff:mult_first_res[31:14];
        3'h3:mult_first_scaled[17:0]<=  (~mult_first_res[35] & |mult_first_res[34:30]) ? 18'h1ffff:mult_first_res[30:13];
        3'h4:mult_first_scaled[17:0]<=  (~mult_first_res[35] & |mult_first_res[34:29]) ? 18'h1ffff:mult_first_res[29:12];
        3'h5:mult_first_scaled[17:0]<=  (~mult_first_res[35] & |mult_first_res[34:28]) ? 18'h1ffff:mult_first_res[28:11];
        3'h6:mult_first_scaled[17:0]<=  (~mult_first_res[35] & |mult_first_res[34:27]) ? 18'h1ffff:mult_first_res[27:10];
        3'h7:mult_first_scaled[17:0]<=  (~mult_first_res[35] & |mult_first_res[34:26]) ? 18'h1ffff:mult_first_res[26: 9];
      endcase

      if (lens_corr_out[4]) pixdo[15:0] <= pre_pixdo_with_zero[20]? 16'h0:   /// negative - use 0
                                        ((|pre_pixdo_with_zero[19:16])?16'hffff: ///>0xffff - limit by 0xffff
                                                                       pre_pixdo_with_zero[15:0]);
    end

  MULT18X18SIO #(
      .AREG(1), // Enable the input registers on the A port (1=on, 0=off)
      .BREG(1), // Enable the input registers on the B port (1=on, 0=off)
      .B_INPUT("DIRECT"), // B cascade input "DIRECT" or "CASCADE" 
      .PREG(1)  // Enable the input registers on the P port (1=on, 0=off)
   ) i_mult_first (
      .BCOUT(), // 18-bit cascade output
      .P(mult_first_res[35:0]),    // 36-bit multiplier output
//      .A(FXY[17]?18'h1ffff:FXY[17:0]),    // 18-bit multiplier input
      .A((FXY[18]==FXY[17])?FXY[17:0]:(FXY[18]?18'h20000:18'h1ffff)),    // 18-bit multiplier input
      .B({1'b0,scales[~color[1:0]]}),    // 18-bit multiplier input
      .BCIN(18'b0), // 18-bit cascade input
      .CEA(lens_corr_out[0]), // Clock enable input for the A port
      .CEB(lens_corr_out[0]), // Clock enable input for the B port
      .CEP(lens_corr_out[1]), // Clock enable input for the P port
      .CLK(pclk), // Clock input
      .RSTA(1'b0), // Synchronous reset input for the A port
      .RSTB(1'b0), // Synchronous reset input for the B port
      .RSTP(1'b0)  // Synchronous reset input for the P port
   );


  MULT18X18SIO #(
      .AREG(1), // Enable the input registers on the A port (1=on, 0=off)
      .BREG(0), // Enable the input registers on the B port (1=on, 0=off)
      .B_INPUT("DIRECT"), // B cascade input "DIRECT" or "CASCADE" 
      .PREG(1)  // Enable the input registers on the P port (1=on, 0=off)
   ) i_mult_second (
      .BCOUT(), // 18-bit cascade output
      .P(mult_second_res[35:0]),    // 36-bit multiplier output
      .A(pix_zero[17:0]),    // 18-bit multiplier input
      .B(mult_first_scaled[17:0]),    // 18-bit multiplier input - always positive
      .BCIN(18'b0), // 18-bit cascade input
      .CEA(lens_corr_out[2]), // Clock enable input for the A port
      .CEB(lens_corr_out[0]), // Clock enable input for the B port
      .CEP(lens_corr_out[3]), // Clock enable input for the P port
      .CLK(pclk), // Clock input
      .RSTA(1'b0), // Synchronous reset input for the A port
      .RSTB(1'b0), // Synchronous reset input for the B port
      .RSTP(1'b0)  // Synchronous reset input for the P port
   );



lens_flat_line #(.F_WIDTH(19), /// number of bits in the output result (signed)
                 .F_SHIFT(22), /// shift ~2*log2(width/2), for 4K width
                 .B_SHIFT(12), ///(<=F_SHIFT) shift b- coeff (12 is 2^12 - good for lines <4096, 1 output count per width)
                 .A_WIDTH(19), /// number of bits in a-coefficient  (signed). Just to match the caller - MSBs will be anyway discarded
                 .B_WIDTH(21))  /// number of bits in b-coefficient (signed).
     i_fy( .pclk(pclk),    /// pixel clock
           .first(fstart), /// initialize running parameters from the inputs (first column). Should be at least 1-cycle gap between "first" and first "next"
           .next(newline), /// calcualte next pixel
           .F0(C[18:0]),   /// value of the output in the first column (before saturation), 18 bit, unsigned
           .ERR0(24'b0),       /// initial value of the running error (-2.0<err<+2.0), scaled by 2^22, so 24 bits
           .A0(AY[18:0]),  /// Ay
           .B0(BY[20:0]),  /// By,  signed
           .F(FY[18:0]),
           .ERR(ERR_Y[23:0]));

lens_flat_line #(.F_WIDTH(19), /// number of bits in the output result
                 .F_SHIFT(22), /// shift ~2*log2(width/2), for 4K width
                 .B_SHIFT(12), ///(<=F_SHIFT) shift b- coeff (12 is 2^12 - good for lines <4096, 1 output count per width)
                 .A_WIDTH(19), /// number of bits in a-coefficient (unsigned). Just to match the caller - MSBs will be anyway discarded
                 .B_WIDTH(21))  /// number of bits in b-coefficient (signed).
     i_fxy( .pclk(pclk),    /// pixel clock
           .first(newline), /// initialize running parameters from the inputs (first column). Should be at least 1-cycle gap between "first" and first "next"
           .next(linerun), /// calcualte next pixel
           .F0(FY[18:0]),  /// value of the output in the first column (before saturation), 18 bit, unsigned
           .ERR0(ERR_Y[23:0]),       /// initial value of the running error (-2.0<err<+2.0), scaled by 2^22, so 24 bits
           .A0(AX[18:0]),  /// Ax(Y),  signed 
           .B0(BX[20:0]),  /// Bx(Y),  signed
           .F(FXY[18:0]),
           .ERR());

endmodule

module lens_flat_line(
          pclk,   /// pixel clock
          first,  /// initialize running parameters from the inputs (first column). Should be at least 1-cycle gap between "first" and first "next"
          next,   /// calcualte next pixel
          F0,     /// value of the output in the first column (before saturation), 18 bit, unsigned
          ERR0,   /// initial value of the running error (-2.0<err<+2.0), scaled by 2^22, so 24 bits
          A0,     /// a - fixed for negative values
          B0,
          F,
          ERR);     // output - 18 bits, unsigned (not saturated)
 parameter F_WIDTH= 18; /// number of bits in the output result
 parameter F_SHIFT=22; /// shift ~2*log2(width/2), for 4K width
 parameter B_SHIFT=12; ///(<=F_SHIFT) shift b- coeff (12 is 2^12 - good for lines <4096, 1 output count per width)
 parameter A_WIDTH=18; /// number of bits in a-coefficient (unsigned). Just to match the caller - MSBs will be anyway discarded
 parameter B_WIDTH=21; // number of bits in b-coefficient (signed).
 parameter DF_WIDTH=B_WIDTH-F_SHIFT+B_SHIFT; //21-22+12           11;  /// number of bits in step of F between (df/dx), signed

    input                pclk;
    input                first;
    input                next;
    input  [F_WIDTH-1:0] F0;
    input  [F_SHIFT+1:0] ERR0;

    input  [A_WIDTH-1:0] A0;
    input  [B_WIDTH-1:0] B0;
    output [F_WIDTH-1:0] F;
    output [F_SHIFT+1:0] ERR;

    reg    [F_SHIFT+1:0] ERR; /// running difference between ax^2+bx+c and y, scaled by 2^22, signed, should never overflow
    reg    [F_SHIFT+1:0] ApB; /// a+b, scaled by 2 ^22, high bits ignored (not really needed - can use ApB0
    reg    [F_SHIFT+1:1] A2X; /// running value for 2*a*x, scaled by 2^22, high bits ignored
    reg    [(DF_WIDTH)-1:0] dF;  /// or [9:0] - anyway only lower bits will be used in comparison operations
    reg    [F_WIDTH-1:0] F;   /// Running value of the output
    reg                  next_d, first_d; // delayed by 1 cycle
    reg    [F_WIDTH-1:0] F1;
    reg    [A_WIDTH-1:0] A;

    wire   [F_SHIFT+1:0] preERR={A2X[F_SHIFT+1:1],1'b0}+ApB[F_SHIFT+1:0]-{dF[1:0],{F_SHIFT{1'b0}}};
/// Increment can be 0 or +/-1, depending on the required correction
/// It relies on the facts that:
/// - the output F(x) is integer
/// - dF/dx does not chnage by more than +/-1 when x is incremented (abs (d2f/dx2)<1), so the algorithm to get
/// y=round(F(x)) is simple :
/// At each step x, try to chnage y by the same amount as was done at the previous step, adding/subtracting 1 if needed
/// and updating the new running error (difference between the current (integer) value of y and the precise value of F(x)
/// This error is calculated here with the 22 binary digits after the point.
///f=ax^2+bx+c
///
///1)  f <= f+ df +1
///    df <= df+1; 
///    err+= (2ax+a+b-df) -1
///2)  f <= f+ df
///    err+= (2ax+a+b-df)
///3)  f <= f+ df -1
///    df <= df-1; 
///    err+= (2ax+a+b-df) +1
///preERR->inc:
/// 100 -> 11
/// 101 -> 11
/// 110 -> 11
/// 111 -> 00
/// 000 -> 00
/// 001 -> 01
/// 010 -> 01
/// 011 -> 01
    wire           [1:0] inc=   {preERR[F_SHIFT+1] & (~preERR[F_SHIFT] |  ~preERR[F_SHIFT-1]),
                                (preERR[F_SHIFT+1:F_SHIFT-1] != 3'h0)  & 
                                (preERR[F_SHIFT+1:F_SHIFT-1] != 3'h7)};
    always @(posedge pclk) begin
     first_d <=first;
     next_d  <=next;
      if         (first) begin
        F1 [F_WIDTH-1:0] <=  F0[ F_WIDTH-1:0];
        dF[(DF_WIDTH)-1:0] <= B0[B_WIDTH-1: (F_SHIFT-B_SHIFT)];
        ERR[F_SHIFT+1:0] <= ERR0[F_SHIFT+1:0];
        ApB[F_SHIFT+1:0] <= {{F_SHIFT+2-A_WIDTH{A0[A_WIDTH-1]}},A0[A_WIDTH-1:0]}+{B0[B_WIDTH-1:0],{F_SHIFT-B_SHIFT{1'b0}}}; /// high bits from B will be discarded
        A  [A_WIDTH-1:0] <= A0[A_WIDTH-1:0];
      end else if (next) begin
        dF[(DF_WIDTH)-1:0] <= dF[(DF_WIDTH)-1:0]+{{((DF_WIDTH)-1){inc[1]}},inc[1:0]};
        ERR[F_SHIFT-1:0]<= preERR[F_SHIFT-1:0];
        ERR[F_SHIFT+1:F_SHIFT]<= preERR[F_SHIFT+1:F_SHIFT]-inc[1:0];
      end

      if     (first_d)   F[F_WIDTH-1:0] <=  F1[ F_WIDTH-1:0];
      else if (next_d)   F[F_WIDTH-1:0] <=  F[F_WIDTH-1:0]+{{(F_WIDTH-(DF_WIDTH)){dF[(DF_WIDTH)-1]}},dF[(DF_WIDTH)-1:0]};
      
      if     (first_d) A2X[F_SHIFT+1:1] <=                     {{F_SHIFT+2-A_WIDTH{A[A_WIDTH-1]}},A[A_WIDTH-1:0]};
      else if (next)   A2X[F_SHIFT+1:1] <=  A2X[F_SHIFT+1:1] + {{F_SHIFT+2-A_WIDTH{A[A_WIDTH-1]}},A[A_WIDTH-1:0]};
    end 
endmodule
