/*******************************************************************************
 * Include file: imu_sim_init_include.vh
 * Date:2015-07-26  
 * Author: Andrey Filippov     
 * Description: Moved here all simulation for IMU logger 
 *
 * Copyright (c) 2015 Elphel, Inc .
 * imu_sim_init_include.vh is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 *  imu_sim_init_include.vh is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/> .
 *******************************************************************************/
`ifdef TEST_IMU
//wire [11:0] EXT; // bidirectional
      wire IMU_SCL=EXT[0];
      wire IMU_SDA=EXT[1];
      wire IMU_MOSI=EXT[2];
      wire IMU_MISO=EXT[3]; //SuppressThisWarning Veditor UNUSED
      reg  IMU_EN;
      wire IMU_ACTIVE;
      wire IMU_NMOSI=!IMU_MOSI;
      wire [5:1] IMU_TAPS;
      reg        IMU_LATE_ACKN;
      reg        IMU_SCLK;
      reg        IMU_MOSI_REVA;
      reg        IMU_103695REVA;
      wire       IMU_MOSI_OUT;
      wire       IMU_SCLK_OUT;

      assign IMU_MOSI_OUT=IMU_103695REVA?IMU_MOSI_REVA:IMU_MOSI;
      assign IMU_SCLK_OUT=IMU_103695REVA?(IMU_SCLK):IMU_SCL;
      always @ (posedge IMU_SDA) begin
        IMU_EN<=IMU_MOSI;
      end
      wire IMU_CS=IMU_103695REVA?!IMU_ACTIVE:!(IMU_EN &&IMU_SDA);
      reg        IMU_MOSI_D;
      always @ (posedge IMU_SCLK_OUT) begin
//        IMU_MOSI_D<=IMU_MOSI;
        IMU_MOSI_D<=IMU_MOSI_OUT;
      end
      reg [15:0] IMU_LOOPBACK;
      always @ (negedge IMU_SCLK_OUT) begin
        if (!IMU_CS) IMU_LOOPBACK[15:0]<={IMU_LOOPBACK[14:0],IMU_MOSI_D};
      end
      assign EXT[3]=IMU_CS?IMU_DATA_READY:IMU_LOOPBACK[15];
      PULLUP  i_IMU_SDA   (.O(IMU_SDA));
      PULLUP  i_IMU_SCL   (.O(IMU_SCL));
      
      initial begin
        SERIAL_DATA_FD=$fopen("gps_data.dat","r"); 
      end
      
      always begin
       #(IMU_READY_PERIOD-IMU_NREADY_DURATION) IMU_DATA_READY=1'b0;
       #(IMU_NREADY_DURATION)                  IMU_DATA_READY=1'b1;
      end
      assign EXT[4]=SERIAL_BIT;
      assign EXT[5]=GPS1SEC;
      assign EXT[6]=ODOMETER_PULSE;
      oneshot i_oneshot  (.trigger(IMU_NMOSI),
                          .out(IMU_ACTIVE));
                          
      dly5taps i_dly5taps (.dly_in(IMU_NMOSI),
                           .dly_out(IMU_TAPS[5:1]));
      always @ (negedge IMU_ACTIVE or posedge IMU_TAPS[5])    if (!IMU_ACTIVE)    IMU_LATE_ACKN<= 1'b0; else IMU_LATE_ACKN<= 1'b1;
      always @ (negedge IMU_LATE_ACKN or posedge IMU_TAPS[4]) if (!IMU_LATE_ACKN) IMU_SCLK<= 1'b1;      else IMU_SCLK<= ~IMU_SCLK;
      always @ (negedge IMU_SCLK)  IMU_MOSI_REVA<= IMU_NMOSI;


`endif
 