// =============================================================================
//  Program : log2pgm.c
//  Author  : Chun-Jen Tsai
//  Date    : May/25/2020
// -----------------------------------------------------------------------------
//  Description:
//  This is a test program for the image down-sampler accelerator that can be
//  used for image-pyramid generation.
//
//  The program was tested using TeraTerm Pro as the terminal emulator. Once
//  the program is running one the Xilinx KC-705 development board, yoy should
//  enable the TeraTerm logging file by the menu item File->log. Then, you can
//  use the menu item "File->Send file" to send a 640x480 input image to the
//  KC-705 development board. The scaled image will be printed to the terminal
//  and the log file.
//  A command-line program "log2pgm" can then be used to convert the output
//  log file to a PGM image file. The program can be compiled using gcc.
//
// -----------------------------------------------------------------------------
//  Revision information:
//
//  None.
// -----------------------------------------------------------------------------
//  License information:
//
//  This software is released under the BSD-3-Clause Licence,
//  see https://opensource.org/licenses/BSD-3-Clause for details.
//  In the following license statements, "software" refers to the
//  "source code" of the complete hardware/software system.
//
//  Copyright 2020,
//                    Embedded Intelligent Systems Lab (EISL)
//                    Deparment of Computer Science
//                    National Chiao Tung Uniersity
//                    Hsinchu, Taiwan.
//
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its contributors
//     may be used to endorse or promote products derived from this software
//     without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
// =============================================================================

#include<stdio.h>
#include<stdlib.h>

#define MAX_TEXT_SIZE 80

int main(int argc, char *argv[])
{
    FILE *fp_in, *fp_out;
    char stemp[MAX_TEXT_SIZE];
    int tmp, width, height, pcount;
    char Y0, Y1, Y2, Y3;

    if (argc < 3)
    {
        fprintf(stderr, "Convert a pyramid output log file to a PGM image.\n");
        fprintf(stderr, "Usage: %s teraterm.log output.pgm\n", *argv);
        return 1;
    }
    fp_in = fopen(argv[1], "r");
    fp_out = fopen(argv[2], "wb");

    while (fgets(stemp, MAX_TEXT_SIZE, fp_in) == stemp)
    {
        if ((pcount = sscanf(stemp, "IMG %dx%d:", &width, &height)) == 2) break;
    }
    if (pcount != 2)
    {
        fprintf(stderr, "Cannot parse image width & height.\n");
        return 1;
    }

    fprintf(fp_out, "P5\n"); // image color type
    fprintf(fp_out, "%d %d\n", width, height); // image width & height
    fprintf(fp_out, "255\n");
    while (fscanf(fp_in, "%x\n", &tmp) != EOF)
    {
        Y3 = tmp & 255;
        tmp = tmp >> 8;
        Y2 = tmp & 255;
        tmp = tmp >> 8;
        Y1 = tmp & 255;
        tmp = tmp >> 8;
        Y0 = tmp & 255;
        fprintf(fp_out, "%c%c%c%c", (char) Y0, (char) Y1, (char) Y2, (char) Y3);
    }

    fclose(fp_in);
    fclose(fp_out);
    return 0;
}
