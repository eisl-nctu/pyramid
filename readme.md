# Introduction

In this project, we present an image pyramid accelerator for embedded processors that generates image pyramids of up to 24-levels of down-sampled resolutions for the input image. Since image pyramids are crucial for the coarse-to-fine analy-sis of many computer vision problems, more resolution levels in the image pyra-mid can improve the parameter-estimation accuracy. Furthermore, the down-sampling filters used in the proposed design is based on a long-tap Sine-windowed Sinc function filter. Therefore, it preserves more image details than other low-complexity filters such as the bilinear or the bicubic interpolation filter. The proposed circuit is verified on an FPGA development board with a Xilinx Kintex-7 device.

# Documents

A paper that presents the design details of the image pyramid generator is published at Samos 2020, the International Conference on Embedded Computer Systems: Architectures, Modeling and Simulation, July 2020. A draft pdf is available [here](https://github.com/eisl-nctu/pyramid/tree/master/docs/pyramid.pdf).

# Usage
The circuit is implemented as a AXI4 master IP. The RTL model is written in Verilog and Xilinx Vivado 2018.2 are used to synthesize the SoC. The Xilinx KC-705 development board was used as the verification platform.

A Vivado sample workspace can be downloaded from [here](https://github.com/eisl-nctu/pyramid/tree/master/archive/pyramid.zip). This workspace contains the test software (for Xilinx SDK) that uses a Microbalze processor to control the image down-scaling accelerator. The sample software reads an 640x480 input image through the UART port and prints the output image to the UART port. You can enable the logging function of your terminal emulator (Teraterm Pro in our case) to save the output image(s) to a file. An offline command-line tool, log2pgm, can be used to convert the logged text file to a PGM image for verification. 

We are working on a RISC-V SoC that has SD card access capabilities. We plan to migrate the image pyramid accelerator over to the RISC-V SoC when its ready.

# Contact Info
Embedded Intelligent Systems Lab (EISL)  
Department of Computer Science  
National Chiao Tung University  
Hsinchu, Taiwan  

eisl.nctu-at-gmail  
