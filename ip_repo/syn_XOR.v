`timescale 1 ns / 1 ps
// =============================================================================
//  Program : syn_XOR.v
//  Author  : Chiang-Yi Wang
//  Date    :
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Revised on Jul/21/2018.
// -----------------------------------------------------------------------------
//  Description:
//
//  Clock synchronization scheme.
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

module syn_XOR
(
    input IN,
    output OUT,
    input TX_CLK,
    input RX_CLK,
    input RST_N
);

wire Y;
wire a;
reg P;
reg Q;

always@(posedge TX_CLK or negedge RST_N)
begin
  if(!RST_N)
    P <= 0;
  else
    P <= a;
end

always@(posedge RX_CLK or negedge RST_N)
begin
  if(!RST_N)
    Q <= 0;
  else
    Q <= Y;
end

xor g2 (a, IN, P);
synchronizer x1(.D(P), .Q(Y), .clk(RX_CLK), .rst_n(RST_N));
xor g1 (OUT, Q, Y);

endmodule
