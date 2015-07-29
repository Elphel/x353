x353
=====

FPGA code for the current Elphel NC353 camera (and other products such as Eyesis4pi based on the same
10353E system board ), updated to work with [VDT plugin](https://github.com/Elphel/vdt-plugin) for
Eclipse IDE and is compatible with Xilinx ISE 14.7 tools.

This repository is created as a reference for simulation of image acquisition, processing and compression
in the new NC393 camera that includes functionality of the previous one, so the same input image (on one of
the 4 channels) should generate the same intermediate and final compressed files on both cameras.

Project is modified to work with the current (and the last!) version of Xilinx ISE - 14.7, result bit file
is not tested enough, but it works in at least one camera. It is not yet safe to just replace the x353.bit
in camera unless you have experience in reflashing "bricked" camera as described in
[http://wiki.elphel.com/index.php?title=Netboot_firmware_upgrade] as it is easy to accidentally make a camera
non-bootable.

**NC353_TESTING** sub-directory contains description how to make such testing reasonably safe, together with
the 'fpga' init script (automatically executed at boot time) that should be replaced in the camera file system.

**ISE_10_1_03_files** directory contains files from the original design that relied on ancient ISE 10.1.03.

**ISE_14_7_results** includes log and results from the current tools
 
You may follow instructions in the README.md for the [VDT plugin](https://github.com/Elphel/vdt-plugin) , 
just use this (x353) project instead of the eddr3 mentioned in the documentation. And you will need to use
Xilinx ISE (not Xilinx Vivado) for this Spartan3e FPGA.

