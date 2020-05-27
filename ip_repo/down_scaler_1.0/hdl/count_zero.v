`timescale 1 ns / 1 ps
// =============================================================================
//  Program : count_zero.v
//  Author  : Chiang-Yi Wang
//  Date    :
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Revised on Jul/21/2018.
// -----------------------------------------------------------------------------
//  Description:
//
//  Return the number of leading zeros in a 16-bit bit pattern.
//
//  Target Platform : Xilinx KC705
//  Tool Versions   : Tested using Vivado 2018.2.
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

module count_zero(input [15:0] in, output [4:0] out);

reg [4:0] out;

always@(*)
begin
    if (in[15])
        out = 0;
    else if (in[14])
        out = 1 ;
    else if (in[13])
        out = 2 ;
    else if (in[12])
        out = 3 ;
    else if (in[11])
        out = 4 ;
    else if (in[10])
        out = 5 ;
    else if (in[9])
        out  = 6 ;
    else if (in[8])
        out  = 7 ;
    else if (in[7])
        out  = 8 ;
    else if (in[6])
        out  = 9 ;
    else if (in[5])
        out  = 10;
    else if (in[4])
        out  = 11;
    else if (in[3])
        out  = 12;
    else if (in[2])
        out  = 13;
    else if (in[1])
        out  = 14;
    else if (in[0])
        out  = 15;
    else
        out = 16;
end
endmodule
