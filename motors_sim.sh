#!/bin/sh
UNISIMS="../unisims"
rm -vf motors.lxt
echo "Using UNISIM library $UNISIMS"
time iverilog -Dlegacy_model -gno-specify -v -o motors_test -sglbl -stestbench \
-y$UNISIMS \
motors_tb.v \
10364.v \
macros353.v \
glbl.v  \
|| { echo "iverilog failed"; exit 1; } 

time vvp -v  motors_test -lxt2  || { echo "vvp failed"; exit 1; } 

gtkwave motors.lxt motors2.sav &


exit 0 

