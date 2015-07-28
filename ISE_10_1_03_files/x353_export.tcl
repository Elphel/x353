# version: 10.1.03
# 
# ProjectNavigator SourceControl export script
#
# This script is generated and executed by ProjectNavigator
# to copy files related to the current project to a
# destination staging area.
#
# The list of files being exported, as controled by
# GUI settings, is placed in the variable export_files.
# Each file will be passed to the procedure CopyOut.  This
# procedure will determine the exact destination, and will
# attempt the file copy if the source and destination are different.
#
# This script is not indended for direct customer editing.
#
# Copyright 2006, Xilinx, Inc.
#
#
# CopyOut -- copy a source file to the staging area, with
#            extra options for files remote (outside of) the
#            project directory.
proc CopyOut { srcfile staging_area copy_option } {
   set report_errors true
   if { $copy_option == "flatten" } {
       set dest [ file join $staging_area [ file tail $srcfile ] ]
   } elseif { $copy_option == "nop" } {
       set dest  $staging_area
       set report_errors false
   } elseif { [ file pathtype $srcfile ] != "relative" } {
       set srcfile2 [ string map {: _} $srcfile ]
       set dest [ file join $staging_area absolute $srcfile2 ]
   } elseif { [ expr { $copy_option == "absremote" } && { [string equal -length 2 $srcfile ".." ] } ] } {
       set dest [ file join $staging_area outside_relative [ string map {.. up} $srcfile ] ]
   } else {
       set srcfile2 [ string map {: _} $srcfile ]
       set dest [ file join $staging_area $srcfile2 ]
   }

   set dest [ file normalize $dest ]
   set src [ file normalize $srcfile ]

   if { [ expr { [ string equal $dest $src ] == 0 } && { [ file exists $src ] } ] } {
      file mkdir [ file dirname $dest ]
      if { [catch { file copy -force $src $dest } ] } {
         if { $report_errors } { puts "Could not copy $src to $dest." }
      }
   }
}

# change to the working directory
set old_working_dir [pwd]
cd [file normalize {/home/andrey/cvs_sync/elphel353-8.0.6.4/elphel353/fpga/x3x3} ]
set copy_option relative
set staging_area "/home/andrey/cvs_sync/elphel353-8.0.6.4/elphel353/fpga/x3x3"
set export_files { 
          "10364.v" 
          "camsync.v" 
          "clkios353.v" 
          "cmd_sequencer.v" 
          "color_proc353.v" 
          "compressor333.v" 
          "control_regs.v" 
          "coring.dat" 
          "csconvert18.v" 
          "csconvert_mono.v" 
          "ddr.v" 
          "ddr_parameters.v" 
          "descrproc353.v" 
          "dma_fifo353.v" 
          "encoderDCAC353.v" 
          "extjtag.v" 
          "focus_filt.dat" 
          "focus_sharp.v" 
          "glbl.v" 
          "gps_data.dat" 
          "histogram353.v" 
          "huffman.dat" 
          "huffman333.v" 
          "i2c_writeonly.v" 
          "imu_logger.v" 
          "interrupts_vector333.v" 
          "ioports353.v" 
          "irq_smart.v" 
          "lens_flat.v" 
          "linear1028rgb.dat" 
          "macros353.v" 
          "mcontr353.v" 
          "motor.dat" 
          "motors_sim.sh" 
          "motors_tb.v" 
          "quantization_100.dat" 
          "quantizator353.v" 
          "rtc353.v" 
          "sdram_phase.v" 
          "sdseq353.v" 
          "sensdcclk333.v" 
          "sensor.dat" 
          "sensor12bits.v" 
          "sensor_phase353.v" 
          "sensorpads353.v" 
          "sensorpix353.v" 
          "sensortrig.v" 
          "stuffer333.v" 
          "twelve_ios.v" 
          "x353.ucf" 
          "x353.v" 
          "x353_1.sav" 
          "x353_1.tf" 
          "x353_guide.ncd" 
          "x353_sim.sh" 
          "xdct353.v" 
                 }
foreach file $export_files {
   CopyOut $file $staging_area $copy_option
}
# copy export file to staging area
CopyOut "/home/andrey/cvs_sync/elphel353-8.0.6.4/elphel353/fpga/x3x3/x353_import.tcl" "$staging_area" nop
# return back
cd $old_working_dir

