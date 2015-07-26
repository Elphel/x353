x353
=====

FPGA code for the current Elphel NC353 camera (and other products such as Eyesis4pi based on the same
10353E system board ), updated to work with [VDT plugin](https://github.com/Elphel/vdt-plugin)

This repository is created as a reference for simulation of image acquisition, processing and compression
in the new NC393 camera that includes functionality of the previous one, so the same input image (on one of
the 4 channels) should generate the same intermediate and final compressed files on both cameras.

We will also try to make it possible to generate functional bitstream files compatible with the existing
NC353 camera (so others will be able to modify their camera code with the current version of Xilinx tools),
but we are not there yet - ***this project is valid for simulation only!***

Here is what makes it difficult:

1. Xilinx abandoned support for the older devices in the current software called "Vivado".
2. Last verion of the ISE (it is ISE 14.7) can not use the older code "as is"
3. We were able to modify the Verilog code to be parsed by the current XST, but it does not
recognize some statements in the *.xcf constraints file (I had to rename original *.ucf to *.xcf).
4. Attempt to try old parser (Suggested by XST itself as the new parser is not the default for
the Spartan 3e): 
```
WARNING:Xst:3152 - You have chosen to run a version of XST which is not the default
   solution for the specified device family. You are free to use it in order to take
   advantage of its enhanced HDL parsing/elaboration capabilities. However,
   please be aware that you may be impacted by  language support differences.
   This version may also result in circuit performance and device utilization
   differences for your particular design. You can always revert back to the
   default XST solution by setting the "use_new_parser" option to value "no" 
   on the XST command line or in the XST process properties panel.
```
also failed. After I added recommended options:

```
run  -use_new_parser no -ifn  x353.prj  -ofn x353.ngc  -top x353  -p xc3s1200eft256  -uc x353.xcf  -opt_mode speed  -opt_level 1 
```

and ISE noticed that:

```
WARNING:Xst:1583 - You are using an internal switch '-use_new_parser'.
```

It still repeated the same  WARNING:Xst:3152 (see above) disregarding its own suggestion.

So we will need to find a way how to replace lines in the *xst file that cause errors in XST:

```
204 TIMEGRP "CPU_ADDR" =   pads("A<*>");
205 TIMEGRP "CPU_ADDRCE" = "CPU_ADDR"  pads("CE*");
206 TIMEGRP "CPU_DATA" =   pads("D<*>");
207 TIMEGRP "WE" =         pads("WE");
208 TIMEGRP "OE" =         pads("OE");
209 TIMEGRP "DACK_PAD"=    pads("DACK*");
209 TIMEGRP "DREQ_PAD"=    pads("DREQ*");
210 TIMEGRP "ALLPADS"=     pads("*");
```

```
ERROR:Xst:1888 - Processing TIMEGRP CPU_ADDR: User group 'pads("A<*>")' defined from
                 other user group pattern not supported.
```

Even Google does not know what to do about this Xilinx XST feature:

> No results found for "ERROR:Xst:1888".
>
> Results for ERROR:Xst:1888 (without quotes):
>
> ...

So we'try to find other ways to re-formulate old timing constraints preserving the same meaning and try
again to run tools.
