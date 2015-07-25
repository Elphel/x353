#!/bin/sh
UNISIMS="../unisims"
rm -vf x353.lxt
echo "Using UNISIM library $UNISIMS"
time iverilog -Dlegacy_model -gno-specify -v -o x353mjpeg_test -sglbl -stestbench353 \
-y$UNISIMS \
x353_1.tf \
x353.v \
macros353.v \
sdram_phase.v \
ioports353.v \
sensorpads353.v \
clkios353.v \
sensor_phase353.v \
mcontr353.v \
descrproc353.v \
sdseq353.v \
sensdcclk333.v \
sensortrig.v \
sensorpix353.v \
lens_flat.v \
histogram353.v \
compressor333.v \
dma_fifo353.v \
color_proc353.v \
csconvert18.v \
csconvert_mono.v \
xdct353.v \
quantizator353.v \
encoderDCAC353.v \
huffman333.v \
stuffer333.v \
rtc353.v \
interrupts_vector333.v \
twelve_ios.v \
extjtag.v \
ddr.v \
sensor12bits.v \
camsync.v \
focus_sharp.v \
i2c_writeonly.v \
irq_smart.v \
cmd_sequencer.v \
control_regs.v \
10364.v \
imu_logger.v \
glbl.v  \
|| { echo "iverilog failed"; exit 1; } 

time vvp -v  x353mjpeg_test -lxt2  || { echo "vvp failed"; exit 1; } 

gtkwave x353.lxt x353_1.sav &



exit 0 

