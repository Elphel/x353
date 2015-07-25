/*
** -----------------------------------------------------------------------------**
** csconvert_mono.v
**
** Monochrome and JP4 modules inplace of color converter ones
**
** Copyright (C) 2008 Elphel, Inc
**
** -----------------------------------------------------------------------------**
**  This file is part of X353
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
module csconvert_mono (en,
                       clk,
                       din,
                       pre_first_in,
                       y_out,
                       yaddr,
                       ywe,
                       pre_first_out);

    input        en;
    input        clk;			// clock
    input  [7:0] din; // input data in scanline sequence
    input        pre_first_in;      // marks the first input pixel
    output [7:0] y_out;  // output Y (16x16) in scanline sequence. Valid if ys active
    output [7:0] yaddr; // address for the external buffer memory to write 16x16x8bit Y data
    output       ywe;	 // wrire enable of Y data
    output       pre_first_out;

    wire         pre_first_out= pre_first_in;
//    wire   [7:0] y_out=         din[7:0];
    wire   [7:0] y_out=         {~din[7],din[6:0]};
    reg    [7:0] yaddr;
    reg          ywe;

    always @ (posedge clk) begin
      ywe <= en & (pre_first_in || (ywe && (yaddr[7:0] !=8'hff)));
      if (!en || pre_first_in) yaddr[7:0] <= 8'h0;
      else if (ywe)            yaddr[7:0] <= yaddr[7:0] + 1;
    end

endmodule

module csconvert_jp4  (en,
                       clk,
                       din,
                       pre_first_in,
                       y_out,
                       yaddr,
                       ywe,
                       pre_first_out);

    input        en;
    input        clk;         // clock
    input  [7:0] din; // input data in scanline sequence
    input        pre_first_in;      // marks the first input pixel
    output [7:0] y_out;  // output Y (16x16) in scanline sequence. Valid if ys active
    output [7:0] yaddr; // address for the external buffer memory to write 16x16x8bit Y data
    output       ywe;    // wrire enable of Y data
    output       pre_first_out;

    wire         pre_first_out= pre_first_in;
//    wire   [7:0] y_out=         din[7:0];
    wire   [7:0] y_out=         {~din[7],din[6:0]};
    reg    [7:0] yaddr_cntr;
    reg          ywe;
    wire   [7:0] yaddr= {yaddr_cntr[4],yaddr_cntr[7:5],yaddr_cntr[0],yaddr_cntr[3:1]};
    always @ (posedge clk) begin
      ywe <= en & (pre_first_in || (ywe && (yaddr[7:0] !=8'hff)));
      if (!en || pre_first_in) yaddr_cntr[7:0] <= 8'h0;
      else if (ywe)            yaddr_cntr[7:0] <= yaddr_cntr[7:0] + 1;
    end

endmodule

module csconvert_jp4diff  (en,
                           clk,
                           scale_diff,     // divide differences by 2 (to fit in 8-bit range)
                           hdr,            // second green absolute, not difference
                           din,
                           pre_first_in,
                           y_out,
                           yaddr,
                           ywe,
                           pre_first_out,
                           bayer_phase);
// synthesis attribute shreg_extract of csconvert_jp4diff is yes;

    input        en;
    input        clk;         // clock
    input        scale_diff;
    input        hdr;
    input  [7:0] din; // input data in scanline sequence
    input        pre_first_in;      // marks the first input pixel
    output [8:0] y_out;  // output Y (16x16) in scanline sequence. Valid if ys active
    output [7:0] yaddr; // address for the external buffer memory to write 16x16x8bit Y data
    output       ywe;    // wrire enable of Y data
    output       pre_first_out;
    input  [1:0] bayer_phase; // selected pixel will be absolute, others - difference

    reg          pre_first_out;
    reg    [2:0] pre2_first_out;
    reg    [8:0] y_out;
    reg    [8:0] pre_y_out;
    reg    [7:0] yaddr_cntr;
    reg    [7:0] pre_yaddr_cntr;
    reg    [7:0] pre2_yaddr_cntr;
//    reg    [7:0] icntr;
    reg          ywe;
    reg    [2:0] pre_ywe;
    reg    [7:0] yaddr;
//    reg    [4:0] out_dly;
    reg          dly_1;
    reg   [14:0] dly_16;
    reg          dly_17;
    wire         start_out=bayer_phase[1]?(bayer_phase[0]?dly_17:dly_16):(bayer_phase[0]?dly_1:pre_first_in);
    reg    [7:0] iadr;
    reg          iadr_run;
    reg    [1:0] mux_plus_sel;
    reg    [2:0] mux_minus_sel;
    reg          hdr_bit;
    reg    [1:0] scale_color;
    reg    [1:0] is_color;
 
    reg    [7:0] mux_plus;
    reg    [7:0] mux_minus;
    reg    [7:0] dd0;
    reg    [7:0] dd1;
    wire   [7:0] dd16;
    reg    [7:0] dd17;
    reg   [14:0] ddsr0,ddsr1,ddsr2,ddsr3,ddsr4,ddsr5,ddsr6,ddsr7; 
    wire   [8:0] scaled_pre_y_out= (scale_color[1])? +{pre_y_out[8],pre_y_out[8:1]}: pre_y_out[8:0];
    assign dd16[7:0]={ddsr7[14],ddsr6[14],ddsr5[14],ddsr4[14],ddsr3[14],ddsr2[14],ddsr1[14],ddsr0[14]};
    always @ (posedge clk) begin
      dly_1  <= pre_first_in;
      dly_17 <= dly_16[14];
      dly_16[14:0] <= {dly_16[13:0],dly_1};

      pre2_first_out[2:0]<= {pre2_first_out[1:0], start_out};
      pre_first_out<= pre2_first_out[2];

      iadr_run <= en & (start_out || (iadr_run && (iadr[7:0]!=8'hff)));
      pre_ywe[2:0] <= {pre_ywe[1:0],iadr_run};
      ywe <= pre_ywe[2];

      if      (!en || start_out)  iadr[7:0] <= 8'h0;
      else if (iadr_run)          iadr[7:0] <= iadr[7:0] + 1;
      pre2_yaddr_cntr[7:0] <= iadr[7:0];
      pre_yaddr_cntr [7:0] <= pre2_yaddr_cntr[7:0];
      yaddr_cntr[7:0]           <= pre_yaddr_cntr[7:0];
      yaddr[7:0] <= {yaddr_cntr[4],yaddr_cntr[7:5],yaddr_cntr[0],yaddr_cntr[3:1]};


      case ({bayer_phase[1:0],iadr[4],iadr[0]} )
       4'b0000: begin mux_plus_sel <= 2'h0; mux_minus_sel <= 3'h4; hdr_bit <=1'h0; end
       4'b0001: begin mux_plus_sel <= 2'h0; mux_minus_sel <= 3'h1; hdr_bit <=1'h0; end
       4'b0010: begin mux_plus_sel <= 2'h0; mux_minus_sel <= 3'h2; hdr_bit <=1'h0; end
       4'b0011: begin mux_plus_sel <= 2'h0; mux_minus_sel <= 3'h3; hdr_bit <=1'h1; end
       4'b0100: begin mux_plus_sel <= 2'h1; mux_minus_sel <= 3'h0; hdr_bit <=1'h0; end
       4'b0101: begin mux_plus_sel <= 2'h1; mux_minus_sel <= 3'h4; hdr_bit <=1'h0; end
       4'b0110: begin mux_plus_sel <= 2'h1; mux_minus_sel <= 3'h2; hdr_bit <=1'h1; end
       4'b0111: begin mux_plus_sel <= 2'h1; mux_minus_sel <= 3'h3; hdr_bit <=1'h0; end
       4'b1000: begin mux_plus_sel <= 2'h2; mux_minus_sel <= 3'h0; hdr_bit <=1'h0; end
       4'b1001: begin mux_plus_sel <= 2'h2; mux_minus_sel <= 3'h1; hdr_bit <=1'h1; end
       4'b1010: begin mux_plus_sel <= 2'h2; mux_minus_sel <= 3'h4; hdr_bit <=1'h0; end
       4'b1011: begin mux_plus_sel <= 2'h2; mux_minus_sel <= 3'h3; hdr_bit <=1'h0; end
       4'b1100: begin mux_plus_sel <= 2'h3; mux_minus_sel <= 3'h0; hdr_bit <=1'h1; end
       4'b1101: begin mux_plus_sel <= 2'h3; mux_minus_sel <= 3'h1; hdr_bit <=1'h0; end
       4'b1110: begin mux_plus_sel <= 2'h3; mux_minus_sel <= 3'h2; hdr_bit <=1'h0; end
       4'b1111: begin mux_plus_sel <= 2'h3; mux_minus_sel <= 3'h4; hdr_bit <=1'h0; end
      endcase
/*
      if (pre_ywe[0]) case (mux_plus_sel[1:0]) 
       2'h0:   mux_plus[7:0] <= dd17[7:0];
       2'h1:   mux_plus[7:0] <= dd16[7:0];
       2'h2:   mux_plus[7:0] <= dd1 [7:0];
       2'h3:   mux_plus[7:0] <= dd0 [7:0];
      endcase
      if (pre_ywe[0]) casex ({mux_minus_sel[2] | (hdr_bit & hdr), mux_minus_sel[1:0]}) 
       3'h0:   mux_minus[7:0] <= dd17[7:0];
       3'h1:   mux_minus[7:0] <= dd16[7:0];
       3'h2:   mux_minus[7:0] <= dd1 [7:0];
       3'h3:   mux_minus[7:0] <= dd0 [7:0];
       3'b1xx: mux_minus[7:0] <= 8'h0;
      endcase
*/
      if (pre_ywe[0]) case (mux_plus_sel[1:0]) 
       2'h0:   mux_plus[7:0] <= dd0 [7:0];
       2'h1:   mux_plus[7:0] <= dd1 [7:0];
       2'h2:   mux_plus[7:0] <= dd16[7:0];
       2'h3:   mux_plus[7:0] <= dd17[7:0];
      endcase
      if (pre_ywe[0]) casex ({mux_minus_sel[2] | (hdr_bit & hdr), mux_minus_sel[1:0]}) 
       3'h0:   mux_minus[7:0] <= dd0 [7:0];
       3'h1:   mux_minus[7:0] <= dd1 [7:0];
       3'h2:   mux_minus[7:0] <= dd16[7:0];
       3'h3:   mux_minus[7:0] <= dd17[7:0];
       3'b1xx: mux_minus[7:0] <= 8'h0;
      endcase

      is_color[1:0] <= {is_color[0], ~(mux_minus_sel[2] | (hdr_bit & hdr))}; // 1 for color components (diffs) ([0] valid at pre_ywe[1])
      scale_color[1:0] <= {scale_color[0], ~(mux_minus_sel[2] | (hdr_bit & hdr)) & scale_diff}; // 1 for color components (diffs) ([0] valid at pre_ywe[1])
      if (pre_ywe[1]) pre_y_out[8:0] <= {1'b0,mux_plus[7:0]} - {1'b0,mux_minus[7:0]};
//      if (scaled_pre_y_out[8]==scaled_pre_y_out[7]) y_out[7:0] <= (scaled_pre_y_out[7:0]^ 8'h80); // limit differences by 0/ff
//      else                                          y_out[7:0] <=  scaled_pre_y_out[8]?8'h0:8'hff;
      y_out[8:0] <=  scaled_pre_y_out[8:0] - {1'h0, ~is_color[1],7'h0}; // subtract 0x80 from Y components (make them -128+127)
      dd0[7:0] <= din [7:0];
      dd1[7:0] <= dd0 [7:0];
      ddsr0[14:0] <= {ddsr0[13:0],dd1[0]};
      ddsr1[14:0] <= {ddsr1[13:0],dd1[1]};
      ddsr2[14:0] <= {ddsr2[13:0],dd1[2]};
      ddsr3[14:0] <= {ddsr3[13:0],dd1[3]};
      ddsr4[14:0] <= {ddsr4[13:0],dd1[4]};
      ddsr5[14:0] <= {ddsr5[13:0],dd1[5]};
      ddsr6[14:0] <= {ddsr6[13:0],dd1[6]};
      ddsr7[14:0] <= {ddsr7[13:0],dd1[7]};
      dd17[7:0] <= dd16 [7:0];
    end
endmodule

