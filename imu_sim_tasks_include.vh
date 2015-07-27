/*******************************************************************************
 * Include file: imu_sim_tasks_include.vh
 * Date:2015-07-26  
 * Author: Andrey Filippov     
 * Description: Moved here all simulation tasks for IMU logger 
 *
 * Copyright (c) 2015 Elphel, Inc .
 * imu_sim_tasks_include.vh is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 *  imu_sim_tasks_include.vh is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/> .
 *******************************************************************************/
`ifdef TEST_IMU

task send_serial_bit;
  input [7:0] data_byte;
  reg   [7:0] d;
  begin
    d <= data_byte;
    wait (CLK0); wait (~CLK0); 
// SERIAL_BIT should be 1 here
// Send start bit    
    SERIAL_BIT <= 1'b0;
    repeat (IMU_GPS_BIT_PERIOD) begin  wait (CLK0); wait (~CLK0);  end
// Send 8 data bits, LSB first    
    repeat (8) begin
      SERIAL_BIT <= d[0];
      #1 d[7:0] <= {1'b0,d[7:1]};
      repeat (IMU_GPS_BIT_PERIOD) begin  wait (CLK0); wait (~CLK0);  end
    end
// Send stop bit    
    SERIAL_BIT <= 1'b1;
    repeat (IMU_GPS_BIT_PERIOD) begin  wait (CLK0); wait (~CLK0);  end
  end
endtask  

task send_serial_pause;
  begin
    wait (CLK0); wait (~CLK0); 
    SERIAL_BIT <= 1'b1;
    repeat (16) begin
      repeat (IMU_GPS_BIT_PERIOD) begin  wait (CLK0); wait (~CLK0);  end
    end  
  end
endtask  

//        SERIAL_DATA_FD=$fopen("gps_data.dat","r"); 

task send_serial_line;
  integer char;
  begin
    char=0;
    while (!$feof (SERIAL_DATA_FD) && (char != 'h0a)) begin
      char=$fgetc(SERIAL_DATA_FD);
      send_serial_bit(char);
    end
  end  
endtask

`endif
 