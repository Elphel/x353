Testing new generated bitstream images in Elphel NC353 cameras
==============================================================

CAUTION: Replacing /etc/x353.bit bit file with the experimental one can make the camera
hang, at it it will repeat each time you reboot the camera as the /etc/init.d/fpga
script will attempt to use the same file again.

Here is the modified version of this script - you may use ftp client to replace the original 'fpga' script:

    $ ftp 192.168.0.9
    Connected to 192.168.0.9.
    220 Elphel (R) Model 353 Camera release 8.2.16 (May 16 2015) ready.
    Name (192.168.0.9:user): root
    331 User name okay, need password.
    Password: <pass>
    230 User logged in, proceed.
    Remote system type is UNIX.
    Using binary mode to transfer files.
    ftp> cd /etc/init.d
    250 Command successful.
    ftp> put fpga
    local: fpga remote: fpga
    200 Command okay.
    150 Opening data connection.
    226 Transfer complete.
    16251 bytes sent in 0.00 secs (453431.9 kB/s)
    ftp> exit
    221 Goodbye.

This is the modified part of the script: 
    
    FPGA_ONE_TIME_IMAGE="/etc/x353_experimental.bit"
    FPGA_TMP_IMAGE="/var/tmp/x353_experimental.bit"
    if [ -f $FPGA_ONE_TIME_IMAGE ] ; then
      echo "Moving $FPGA_ONE_TIME_IMAGE to /var/tmp"
      mv $FPGA_ONE_TIME_IMAGE /var/tmp
      sync
      FPGA_IMAGE=$FPGA_TMP_IMAGE
    fi
    echo "For testing unsafe experimental bitsteam images:"
    echo "Name this file as $FPGA_ONE_TIME_IMAGE - it will be 'deleted before used'"
    echo "so next boot will use the original (safe) bitstream file"

So regardless of success or failure of the testing of the experimental bitstream file
it will be used only until the next reboot/power cycle.

To test the new bitstream file you need to rename it to x353_experimental.bit and ftp to
camera /etc directory (supposing you started from the local directory with the
x353_experimental.bit file:

    $ ftp 192.168.0.9
    Connected to 192.168.0.9.
    220 Elphel (R) Model 353 Camera release 8.2.16 (May 16 2015) ready.
    Name (192.168.0.9:user): root
    331 User name okay, need password.
    Password: <pass>
    230 User logged in, proceed.
    Remote system type is UNIX.
    Using binary mode to transfer files.
    ftp> put x353_experimental.bit 
    local: x353_experimental.bit remote: x353_experimental.bit
    200 Command okay.
    150 Opening data connection.
    226 Transfer complete.
    480220 bytes sent in 1.45 secs (324.2 kB/s)
    ftp> exit
    221 Goodbye.

A sample bitstream file is in the ISE_14_7_results directory, with the default project settings
the new generated bitstream files will be in the ise_results subdirectory.

It is a good idea to telnet to the camera and issue a 'sync' command (equivalent to the
"safely remove" for the flash cards). Or just open [http://192.168.0.9/phpshell.php?command=sync]
in the browser (you'll need to modify IP if it was changed from the default one). 

Good hacking!

Elphel team