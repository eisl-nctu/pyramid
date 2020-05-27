`timescale 1 ns / 1 ps
// =============================================================================
//  Program : divider.v
//  Author  : Chiang-Yi Wang
//  Date    :
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Revised on Jul/21/2018.
// -----------------------------------------------------------------------------
//  Description:
//
//  This is a 16-bit integer divider.
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

module divider(
    input clk,
    input rst,
    input en,
    input [15:0] dividend,
    input [15:0] divisor,
    output[15:0] quotient,
    output[15:0] remainder,
    output out_valid
);

wire [15:0] check_in;
wire [4:0] check_out;
reg [4:0] cnt;
reg [15:0] sum,reg_a,reg_b;
reg sign_A, sign_B;

parameter IDLE = 3'b000, INIT_A = 3'b001, INIT_B = 3'b010 ,SHIFTER = 3'b011, DONE = 3'b100, EXCEPTION = 3'b101, SIGN_ADJUST = 3'b110;
reg [2:0] st,nt;

assign check_in = (st == INIT_A) ? ( (sign_A) ? -reg_a : reg_a ) : reg_b;

count_zero cz(.in(check_in),.out(check_out));
assign out_valid = (st==DONE);
assign quotient = sum;
assign remainder = reg_a;

always@(posedge clk)
begin
    if(rst)
    begin
        reg_a <= 0;
        reg_b <= 0;
        sum   <= 0;
        cnt   <= 0;
    end
    else
    begin
        case(st)
            IDLE:
            begin
                if(en)
                begin
                    reg_a <= dividend;
                    reg_b <= divisor;
                    sum   <= 0;
                end
            end
            INIT_A:
            begin
                reg_a <= (sign_A) ? -reg_a : reg_a;
                reg_b <= (sign_B) ? -reg_b : reg_b;
                cnt   <= check_out;
            end
            INIT_B:
            begin
                reg_b <= reg_b << (check_out - cnt);
                cnt   <= (check_out - cnt);
            end
            SHIFTER:
            begin
                cnt<=cnt-1;
                reg_b <= reg_b>>1;
                if(reg_a>=reg_b)
                begin
                    reg_a <= reg_a - reg_b;
                    sum [cnt] <= 1'b1;
                end
            end
            SIGN_ADJUST:
            begin
                reg_a <= (sign_A) ? -reg_a : reg_a;
                sum <= (sign_A ^ sign_B) ? -sum : sum;
            end
        endcase
    end
end

always@(posedge clk)
begin
    if(rst)
    begin
        sign_A <= 0;
        sign_B <= 0;
    end
    else if (st == IDLE)
    begin
        sign_A <= 0;//dividend[15];
        sign_B <= 0;//divisor[15];
    end
end

always@(posedge clk)
begin
    if(rst)
        st<=IDLE;
    else
        st<=nt;
end

always@(*)
begin
    case(st)
        IDLE:
            if(en)
                nt = INIT_A;
            else
                nt = IDLE;
        INIT_A:
            nt = INIT_B;
        INIT_B:
            if (check_out == 16)
                nt = EXCEPTION;
            else if ((check_out < cnt) && (sign_A || sign_B))
                nt = SIGN_ADJUST;
            else if (check_out < cnt)
                nt = DONE;
            else
                nt = SHIFTER;
        SHIFTER:
            if (cnt==0 && (sign_A || sign_B))
                nt = SIGN_ADJUST;
            else if (cnt==0)
                nt = DONE;
            else
                nt = SHIFTER;
        SIGN_ADJUST:
            nt = DONE;
        DONE:
            nt = IDLE;
        EXCEPTION :
            nt = EXCEPTION;
        default:
            nt = IDLE;
    endcase
end

endmodule
