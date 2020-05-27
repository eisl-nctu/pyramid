`timescale 1 ns / 1 ps
// =============================================================================
//  Program : dscaler.v
//  Author  : Chiang-Yi Wang
//  Date    : Nov/22/2017
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Revised on Jul/21/2018.
// -----------------------------------------------------------------------------
//  Description:
//
//  This HDL code is the main module of the down-scalar for a HW-SW
//  codesigned 24-level image pyramid generator.
//
//  This down scaler supports 24-level decimation rate, and adopted sine-window
//  sinc-function filter.  Decimation rates supported are 31/32, 30/32, 29/32,
//  ..., 8/32.  The input image width and height are 640 x 480.
//  The burst length for AXI bus read/write transfer is 32 (words). If the
//  signal 'burst_len' is modified, the offsets M_AXI_ARADDR and M_AXI_AWADDR
//  must be changed accordingly.
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

/*
   The down-scaler supports decimation:
 
  640x480 (32 / 32)          460x345 (23 / 32) (0.72)   300x225 (15 / 32) (0.47)    
  620x465 (31 / 32) (0.97)   440x330 (22 / 32) (0.69)   280x210 (14 / 32) (0.44)    
  600x450 (30 / 32) (0.94)   420x315 (21 / 32) (0.66)   260x195 (13 / 32) (0.41)    
  580x435 (29 / 32) (0.91)   400x300 (20 / 32) (0.62)   240x180 (12 / 32) (0.38)    
  560x420 (28 / 32) (0.88)   380x285 (19 / 32) (0.59)   220x165 (11 / 32) (0.34)    
  540x405 (27 / 32) (0.84)   360x270 (18 / 32) (0.56)   200x150 (10 / 32) (0.31)    
  520x390 (26 / 32) (0.81)   340x255 (17 / 32) (0.53)   180x135 (9 / 32) (0.28)     
  500x375 (25 / 32) (0.78)   320x240 (16 / 32) (0.50)   160x120 (8 / 32) (0.25)     
  480x360 (24 / 32) (0.75)
 
  if (input_width*4 > 15*output_width)      filter = 7;
  else if(input_width*7 > 20*output_width)  filter = 6;
  else if(input_width*2 > 5*output_width)   filter = 5;
  else if(input_width*1 > 2*output_width)   filter = 4;
  else if(input_width*3 > 5*output_width)   filter = 3;
  else if(input_width*4 > 5*output_width)   filter = 2;
  else if(input_width*19 > 20*output_width) filter = 1;
  else filter = 0;
*/

module dscaler
#( parameter ORIGIN_WIDTH = 640, ORIGIN_HEIGHT = 480, BURST_LEN = 32 )
(
    input clk,
    input reset_n,
    // dscaler activate
    input start,
    //  scaling finish
    output finish,
    //decimation rate
    input [ 4: 0 ] down_level_in,
    //pixel I/O
    input [ 31: 0 ] pixel_in,
    output [ 31: 0 ] pixel_out,

    //master burst read/write request
    output IO_IP2Bus_MstRd_Req,
    output IO_IP2Bus_MstWrt_Req,
    // read complete
    input IO_Bus2IP_MstRd_Cmplt,
    // write complete
    input IO_Bus2IP_MstWrt_Cmplt,

    // read valid
    input bus_rd_valid,
    // write valid
    input bus_wrt_valid,

    //for debug
    output[ 4: 0 ] dbg_ds_state_c,
    output[ 4: 0 ] dbg_ds_state_n
);

function integer clogb2 ( input integer bit_depth );
    begin
        for ( clogb2 = 0; bit_depth > 0; clogb2 = clogb2 + 1 )
        begin
            bit_depth = bit_depth >> 1;
        end
    end
endfunction

//---------------------------------------------------------------------
// PARAMETER DECLARATION
//---------------------------------------------------------------------

//dscaler spec //don't modify any parameter.
localparam PIXEL_DATA_WIDTH = 8;
localparam M_AXI_DATA_WIDTH = 32; //AXI BUS DATA_WIDTH
localparam NUMBER_OF_PIXELS = M_AXI_DATA_WIDTH / PIXEL_DATA_WIDTH; // 4 pixels
localparam PING_PONG_RAM_SIZE = BURST_LEN * NUMBER_OF_PIXELS; //32*4=128 pixels, number of pixels from one burst read/write need to store
localparam PIXEL_BUFFER_SIZE = 32; //define pixel buffer size and one of ping-pong RAM ports
localparam FILTER_TAP_LENGTH = 12; //12-tap filter

//image size & filter information
localparam IN_WIDTH = ORIGIN_WIDTH - 1; //639, width of input image, origin width
localparam IN_HEIGHT = ORIGIN_HEIGHT - 1; //479, height of input image, origin height
localparam TOTAL_PIXEL = ORIGIN_WIDTH * ORIGIN_HEIGHT; //640*480, number of total pixel,  origin width*origin height
localparam WRT_RAM_ADDR_LIMIT = PING_PONG_RAM_SIZE - 1; //127, ping-pong RAM addr for write upperbound
localparam WRT_RAM_ADDR_LIMIT_FOR_INPUT_AND_OUTPUT = PING_PONG_RAM_SIZE - NUMBER_OF_PIXELS; // 128-4
localparam MAX_RD_MEM_TIMES = ( TOTAL_PIXEL / NUMBER_OF_PIXELS ) / BURST_LEN; //  640*480/4/32 = 2400
localparam PING_PONG_RAM_DEPTH = PING_PONG_RAM_SIZE / PIXEL_BUFFER_SIZE; //  128/32 = 4
localparam INTER_RAM_DEPTH = TOTAL_PIXEL / PIXEL_BUFFER_SIZE; //   640*480/32 = 9600

localparam integer PIXEL_BUFFER_SIZE_BITS_LENGTH = clogb2( PIXEL_BUFFER_SIZE );
localparam integer INTER_RAM_DEPTH_BITS_LENGTH = clogb2( INTER_RAM_DEPTH );
localparam integer WIDTH_BITS_LENGTH = clogb2( ORIGIN_WIDTH );
localparam integer HEIGHT_BITS_LENGTH = clogb2( ORIGIN_HEIGHT );
localparam integer MAX_RD_MEM_TIMES_BITS_LENGTH = clogb2( MAX_RD_MEM_TIMES );
localparam integer PING_PONG_RAM_DEPTH_BITS_LENGTH = clogb2( PING_PONG_RAM_DEPTH );
localparam integer WRT_RAM_ADDR_LIMIT_BITS_LENGTH = clogb2( WRT_RAM_ADDR_LIMIT );
localparam integer RD_RAM_ADDR_BITS_LENGTH = clogb2( ( PING_PONG_RAM_SIZE * 2 ) - 1 );
localparam integer RAM_ADDR_BITS_LENGTH = clogb2( ( PING_PONG_RAM_SIZE / PIXEL_BUFFER_SIZE ) - 1 );
localparam integer H_RD_RAM_CNT_BITS_LENGTH = clogb2( ( PING_PONG_RAM_SIZE / PIXEL_BUFFER_SIZE ) * 2 - 1 );

// ------------------ dscaler control signal ------------------------------------
//down scaling state
localparam IDLE = 0, RD_MEM = 1, RDY_FOR_H = 2, H_SCALE = 3, NXT_ROW = 4, WRT_RAM = 5, CLN_BUF = 6,
          RDY_FOR_V = 7, V_SCALE = 8, NXT_COL = 9, WRT_MEM_0 = 10,
          PRELOAD_0 = 11, PRELOAD_1 = 12, PRELOAD_2 = 13,
          H_STALL = 14, V_STALL = 15, WRT_MEM_1 = 16, CMPLT = 17, INIT = 18;

//---------------------------------------------------------------------
//   WIRE AND REG DECLARATION
//---------------------------------------------------------------------

//input signal
reg start_reg; //dscaler activate
reg [ 7: 0 ] pixel_in_reg[ 0: 3 ];
reg pixel_in_valid; //pixel input valid

//output image information
reg[ WIDTH_BITS_LENGTH - 1: 0 ] OUT_WIDTH; //output image width
reg[ HEIGHT_BITS_LENGTH - 1: 0 ] OUT_HEIGHT; //output image height

//dscaler state
reg [ 4: 0 ] state_c, state_n;

// Debug signals
assign dbg_ds_state_c = state_c;
assign dbg_ds_state_n = state_n;
assign finish = ( state_c == WRT_MEM_1 && state_n == CMPLT );

// state condition
wire preloading;
wire h_scaling, v_scaling;
wire h_end, v_end;
wire reload_pixel_state; //The signal reload_pixel_state is 1, pixel buffer need to reload.

reg [ 4: 0 ] down_level;

reg [ 4: 0 ] left_edge_detection; //detect for duplication padding
reg [ 4: 0 ] right_edge_detection; //detect for duplication padding

reg [ WIDTH_BITS_LENGTH - 1: 0 ] scaled_pixel_index; // calculate scaled_pixel_index_reg early
reg [ WIDTH_BITS_LENGTH - 1: 0 ] scaled_pixel_index_reg; // 0~output_width-1

reg [ 2: 0 ] write_last_pxl; //write last pixel
reg h_flag;
reg v_flag;

reg [ WIDTH_BITS_LENGTH - 1: 0 ] scaling_pxl_idx; // calculate scaling_pxl_idx_reg early
reg [ WIDTH_BITS_LENGTH - 1: 0 ] scaling_pxl_idx_reg; // 0~output_width-1
reg [ WIDTH_BITS_LENGTH - 1: 0 ] scaling_round_counter; //  0~output_width-1 , number of scaled round

reg rd_mem_cmlt; //read memory complete

reg reload_pixel_buffer_tail_delay;

wire[ 5: 0 ] right_index_limit[ 5: 0 ];
wire[ 5: 0 ] left_index_limit[ 5: 0 ];
reg h_scaled_pixel_valid; // horizontal scaled pixels valid
reg v_scaled_pixel_valid; // vertical scaled pixels valid

// ------------------ 12-tap sinc filter set ------------------------------------

reg [ 3: 0 ] filter_set; // 16 level pyramid
reg signed[ 8: 0 ] base_filter[ 0: FILTER_TAP_LENGTH - 1 ]; //base filter

//filter coefficient (offset)
reg signed[ 5: 0 ] f0 ;
reg signed[ 5: 0 ] f1 ;
reg signed[ 5: 0 ] f2 ;
reg signed[ 5: 0 ] f3 ;
reg signed[ 8: 0 ] f4 ;
reg signed[ 8: 0 ] f5 ;
reg signed[ 8: 0 ] f6 ;
reg signed[ 8: 0 ] f7 ;
reg signed[ 5: 0 ] f8 ;
reg signed[ 5: 0 ] f9 ;
reg signed[ 5: 0 ] f10;
reg signed[ 5: 0 ] f11;

wire[ 4: 0 ] left_index[ 0: 5 ]; //0~31
wire[ 4: 0 ] center_index; // center pixel index
wire[ 4: 0 ] right_index[ 0: 4 ]; //0~31

// ------------------ pixel buffer ------------------------------------
//============================================
//	Except scaling at new row/col head or tail using duplication padding by repeating border element,
//	each scaling cycle need to the picked center pixel, its left 6 pixels and right 5 pixels total 12 pixels to calculate.
//	So pixel_buffer_head need to store 6 pixels for border pixel_buffer[0]~[4],
//	and pixel_buffer_tail need to store 5 pixels for border pixel_buffer[26]~[31].
//============================================
reg [ 7: 0 ] pixel_buffer[ 0: PIXEL_BUFFER_SIZE - 1 ]; //store pixel that are scaling with filter
reg [ 7: 0 ] pixel_buffer_head[ 0: 6 - 1 ]; //store last round scaling pixel_buffer last 6 pixels
reg [ 7: 0 ] pixel_buffer_tail[ 0: 5 - 1 ]; //store next round scaling pixel_buffer first 5 pixels

reg reload_pixel_buffer; // load pixels to pixel buffer signal
reg[ 4: 0 ] load_buf_cnt; // load pixel buffer times
reg reload_pixel_buffer_tail;

// ------------------ reference table ------------------------------------
//============================================
//	Reference table[8:5] for pixel_number
//	Reference table[4:0] for filter_number
//	To implement the following formula
//	reference_table[index] = ( (index <<9) + ((256) - ((down_level)<<3)) +((down_level)>>1)) / ((down_level));
//============================================

reg [ WIDTH_BITS_LENGTH - 1: 0 ] limit_length; // IN_HEIGHT or IN_WIDTH
reg [ 3: 0 ] filter_number; //filter_number for select filter offset(filter coefficient)
reg [ 4: 0 ] pixel_number; // pixel_number for select a center pxiel

reg [ 3: 0 ] filter_number_reg; //filter_number for select filter offset(filter coefficient)
reg [ 4: 0 ] pixel_number_reg; // pixel_number for select a center pxiel

reg [ 8: 0 ] reference_table[ 31: 0 ]; // reference table
reg [ 4: 0 ] reference_table_index; // decided by scaling_pxl_idx_reg
reg [ 5: 0 ] pixel_counter_0; //  a counter for even index to calculate reference table
reg [ 5: 0 ] pixel_counter_1; //  a counter for odd index to calculate reference table
reg reference_table_done;

reg [ 17: 0 ] C0_0;
reg [ 17: 0 ] C0_1;
reg [ 7: 0 ] C1;
reg [ 3: 0 ] C2;

//two 16bits divider signal
wire [ 15: 0 ] dividend_0, dividend_1;
wire [ 15: 0 ] divisor;
wire [ 15: 0 ] quotient_0, quotient_1;
reg en_0, en_1;
wire divider_out_valid_0, divider_out_valid_1;

// ------------------ Bus control signal ------------------------------------
//bus state
localparam WAIT_REQ = 0, WAIT_CMPLT = 1;
//bus control signal
reg bus_state_c, bus_state_n;

//bus request
wire rd_mem_req; //read memory request
reg [ MAX_RD_MEM_TIMES_BITS_LENGTH - 1: 0 ] rd_mem_req_cnt; //0~2400(MAX_RD_MEM_TIMES) , Notice: if modify WRT_RAM_ADDR_LIMIT, rd_mem_req_cnt bits need to change.
wire wrt_mem_req; //write memory request
reg wrt_mem_last_one_req; //write memory last one request

//------------------- ping-pong RAM signal ------------------------------------
integer i;
wire [ RAM_ADDR_BITS_LENGTH - 1: 0 ] ram_addr_0; //0~3 = (PING_PONG_RAM_SIZE/PIXEL_BUFFER_SIZE)-1 , Notice: if modify WRT_RAM_ADDR_LIMIT, sram_addr bits need to change.
wire [ RAM_ADDR_BITS_LENGTH - 1: 0 ] ram_addr_1;
reg [ RD_RAM_ADDR_BITS_LENGTH - 1: 0 ] rd_ram_addr; //0~255 = (PING_PONG_RAM_SIZE*2)-1 , Notice: if modify WRT_RAM_ADDR_LIMIT, rd_ram_addr bits need to change.
reg [ 31: 0 ] ram_we_0;
reg [ 31: 0 ] ram_we_1;
wire ram_en;
wire [ 7: 0 ] data_in[ 0: PIXEL_BUFFER_SIZE - 1 ]; //ram data in
wire [ 7: 0 ] data_out_0[ 0: PIXEL_BUFFER_SIZE - 1 ]; //ram data out

//SRAM signal
localparam RD = 0, WRT = 1, STALL = 2;
wire [ 7: 0 ] data_out_1[ 0: PIXEL_BUFFER_SIZE - 1 ]; //ram data out
reg [ 1: 0 ] ram_state_c_0;
reg [ 1: 0 ] ram_state_n_0;

reg [ 1: 0 ] ram_state_c_1;
reg [ 1: 0 ] ram_state_n_1;

reg v_rd_ram_ptr; //vertical read read ram pointer
wire h_rd_ram_ptr; //horizontal read ram pointer
reg[ H_RD_RAM_CNT_BITS_LENGTH - 1: 0 ] h_rd_ram_cnt; //0~7((PING_PONG_RAM_SIZE/PIXEL_BUFFER_SIZE)*2-1) , Notice: if modify WRT_RAM_ADDR_LIMIT, h_rd_ram_cnt bits need to change.

reg [ WRT_RAM_ADDR_LIMIT_BITS_LENGTH - 1: 0 ] wrt_ram_addr_0; //0~127(PING_PONG_RAM_SIZE) , Notice: if modify WRT_RAM_ADDR_LIMIT, wrt_sram_addr_reg bits need to change.
reg [ WRT_RAM_ADDR_LIMIT_BITS_LENGTH - 1: 0 ] wrt_ram_addr_1;
reg wrt_ram_ptr; //write ram pointer
reg wrt_mem_req_reg; //write memory request

reg [ WRT_RAM_ADDR_LIMIT_BITS_LENGTH - 1: 0 ] output_cnt; //0~127(PING_PONG_RAM_SIZE) , Notice: if modify WRT_RAM_ADDR_LIMIT, output_cnt bits need to change.

reg ram_full_0; // ram full
reg ram_full_1;

//------------------- inter RAM signal ------------------------------------
wire [ INTER_RAM_DEPTH_BITS_LENGTH - 1: 0 ] inter_ram_addr; //0~9600,
reg [ INTER_RAM_DEPTH_BITS_LENGTH - 1 + PIXEL_BUFFER_SIZE_BITS_LENGTH - 1: 0 ] rd_inter_ram_addr; //for read inter ram addr
wire [ INTER_RAM_DEPTH_BITS_LENGTH - 1: 0 ] wrt_inter_ram_addr; // for write inter ram addr
reg [ 31: 0 ] inter_ram_we;
wire inter_ram_en;
wire [ 7: 0 ] inter_data_in; //ram data in
wire [ 7: 0 ] inter_data_out[ 0: PIXEL_BUFFER_SIZE - 1 ]; //ram data out

reg [ INTER_RAM_DEPTH_BITS_LENGTH - 1 + PIXEL_BUFFER_SIZE_BITS_LENGTH - 1: 0 ] base; // for inter ram base addr
reg [ INTER_RAM_DEPTH_BITS_LENGTH - 1 + PIXEL_BUFFER_SIZE_BITS_LENGTH - 1: 0 ] offset; //for inter ram addr offset

wire write_next_INTER_RAM_PORT;

reg write_next_INTER_RAM_PORT_reg;

//------------------- multiply and accumulate ------------------------------------
reg signed[ 8: 0 ] filter[ 0: 11 ];
reg signed[ 8: 0 ] pixel[ 0: 11 ];
reg signed[ 16: 0 ] multiply[ 0: 11 ];
reg signed[ 19: 0 ] accumulate[ 0: 1 ];
reg signed[ 14: 0 ] scaled_pixel;

//special case: NXT_COL and STALL are adjacent cycle need to special processing
reg [ 7: 0 ] scaled_pixel_at_ram_full;
reg nxt_col_at_ram_full;

//=================================================================================
//	IMPLEMENTATION OF dscaler
//=================================================================================

//======================================
//   dscaler state control
//======================================
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        state_c <= INIT;
    end //IDLE;
    else
    begin
        state_c <= state_n;
    end
end

always@( * )
begin
    case ( state_c )
        INIT:
            if ( start )
            begin
                state_n = IDLE;
            end
            else
            begin
                state_n = INIT;
            end
        IDLE:
            state_n = RD_MEM;

        RD_MEM:
            if ( rd_mem_cmlt && reference_table_done )
            begin
                state_n = RDY_FOR_H;
            end
            else
            begin
                state_n = RD_MEM;
            end
        RDY_FOR_H:
            state_n = PRELOAD_0;
        PRELOAD_0:
            state_n = PRELOAD_1;
        PRELOAD_1:
            state_n = PRELOAD_2;
        PRELOAD_2:
            if ( h_flag == 0 )
            begin
                state_n = H_SCALE;
            end
            else
            begin
                state_n = V_SCALE;
            end

        H_SCALE:
            if ( h_end )
            begin
                state_n = WRT_RAM;
            end
            else if ( ( rd_mem_req_cnt != MAX_RD_MEM_TIMES ) && ( ( ram_state_c_0 == WRT && h_rd_ram_ptr == 0 ) || ( ram_state_c_1 == WRT && h_rd_ram_ptr == 1 ) ) )
            begin
                state_n = H_STALL;
            end
            else if ( scaling_pxl_idx_reg == OUT_WIDTH )
            begin
                state_n = NXT_ROW;
            end
            else
            begin
                state_n = H_SCALE;
            end
        H_STALL:
            if ( h_rd_ram_ptr == 0 && ram_full_0 || h_rd_ram_ptr == 1 && ram_full_1 )
            begin
                state_n = H_SCALE;
            end
            else
            begin
                state_n = H_STALL;
            end
        NXT_ROW:
            state_n = H_SCALE;
        WRT_RAM:
            if ( write_last_pxl == 4 )
            begin
                state_n = CLN_BUF;
            end
            else
            begin
                state_n = WRT_RAM;
            end
        CLN_BUF:
            state_n = RDY_FOR_V;


        RDY_FOR_V:
            state_n = PRELOAD_0;
        V_SCALE:
            if ( v_end )
            begin
                state_n = WRT_MEM_0;
            end
            else if ( ram_state_c_1 != STALL && ( ram_state_c_0 == RD && wrt_ram_ptr == 0 ) || ( ram_state_c_1 == RD && wrt_ram_ptr == 1 ) )
            begin
                state_n = V_STALL;
            end
            else if ( scaling_pxl_idx_reg == OUT_HEIGHT )
            begin
                state_n = NXT_COL;
            end
            else
            begin
                state_n = V_SCALE;
            end
        V_STALL:
            if ( wrt_ram_ptr == 0 && ram_full_0 == 0 || wrt_ram_ptr == 1 && ram_full_1 == 0 )
            begin
                state_n = V_SCALE;
            end
            else
            begin
                state_n = V_STALL;
            end
        NXT_COL:
            state_n = V_SCALE;
        WRT_MEM_0:
            if ( write_last_pxl == 4 )
            begin
                state_n = WRT_MEM_1;
            end
            else
            begin
                state_n = WRT_MEM_0;
            end
        WRT_MEM_1:
            if ( wrt_mem_last_one_req && IO_Bus2IP_MstWrt_Cmplt )
            begin
                state_n = CMPLT;
            end
            else
            begin
                state_n = WRT_MEM_1;
            end
        CMPLT:
            if ( start == 0 )
            begin
                state_n = INIT;
            end
            else
            begin
                state_n = CMPLT;
            end
        default:
            state_n = INIT;
    endcase
end

//------------------- input ports ------------------------------------

always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        { pixel_in_reg[ 3 ], pixel_in_reg[ 2 ], pixel_in_reg[ 1 ], pixel_in_reg[ 0 ] } <= 0;
    end
    else
    begin
        { pixel_in_reg[ 0 ], pixel_in_reg[ 1 ], pixel_in_reg[ 2 ], pixel_in_reg[ 3 ] } <= pixel_in;
    end
end

//The signal bus_rd_valid is 1, pixels are valid.
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        pixel_in_valid <= 0;
    end
    else
    begin
        pixel_in_valid <= bus_rd_valid;
    end
end

always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        start_reg <= 0;
    end
    else
    begin
        start_reg <= start;
    end
end

always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        down_level <= 0;
    end
    else
    begin
        down_level <= down_level_in;
    end
end

//------------------- output ports ------------------------------------

//4 pxiels, output data
assign pixel_out[ 7: 0 ] = ( wrt_mem_req_reg ) ? ( ( v_rd_ram_ptr ) ? data_out_1[ output_cnt[ 4 : 0 ] ] : data_out_0[ output_cnt[ 4 : 0 ] ] ) : scaled_pixel;
assign pixel_out[ 15: 8 ] = ( wrt_mem_req_reg ) ? ( ( v_rd_ram_ptr ) ? data_out_1[ output_cnt[ 4 : 0 ] + 1 ] : data_out_0[ output_cnt[ 4 : 0 ] + 1 ] ) : 0;
assign pixel_out[ 23: 16 ] = ( wrt_mem_req_reg ) ? ( ( v_rd_ram_ptr ) ? data_out_1[ output_cnt[ 4 : 0 ] + 2 ] : data_out_0[ output_cnt[ 4 : 0 ] + 2 ] ) : 0;
assign pixel_out[ 31: 24 ] = ( wrt_mem_req_reg ) ? ( ( v_rd_ram_ptr ) ? data_out_1[ output_cnt[ 4 : 0 ] + 3 ] : data_out_0[ output_cnt[ 4 : 0 ] + 3 ] ) : 0;

//output data index control
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        output_cnt <= 0;
    end
    else if ( ~bus_wrt_valid )
    begin
        output_cnt <= output_cnt;
    end
    else
    begin
        output_cnt <= rd_ram_addr;
    end
end


//burst read/write request
assign IO_IP2Bus_MstRd_Req = rd_mem_req;
assign IO_IP2Bus_MstWrt_Req = wrt_mem_req;
assign rd_mem_req = ~rd_mem_cmlt && ( bus_state_c == WAIT_REQ ) && ( state_c == RD_MEM || h_scaling ) && ( ( wrt_ram_ptr == 0 && ram_full_0 == 0 ) || ( wrt_ram_ptr == 1 && ram_full_1 == 0 ) ) && ~pixel_in_valid && ( rd_mem_req_cnt != MAX_RD_MEM_TIMES );
assign wrt_mem_req = ( bus_state_c == WAIT_REQ ) && ( v_flag ) && ram_state_c_1 != STALL && ( ( ram_full_0 == 1 ) || ( ram_full_1 == 1 ) ) && ~pixel_in_valid;

always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        wrt_mem_req_reg <= 0;
    end
    else if ( IO_Bus2IP_MstWrt_Cmplt )
    begin
        wrt_mem_req_reg <= 0;
    end
    else if ( wrt_mem_req )
    begin
        wrt_mem_req_reg <= 1;
    end
    else
    begin
        wrt_mem_req_reg <= wrt_mem_req_reg;
    end
end

always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        bus_state_c <= WAIT_REQ;
    end
    else
    begin
        bus_state_c <= bus_state_n;
    end
end

always@( * )
begin
    case ( bus_state_c )
        WAIT_REQ:
            if ( rd_mem_req == 1 || wrt_mem_req )
            begin
                bus_state_n <= WAIT_CMPLT;
            end
            else
            begin
                bus_state_n <= WAIT_REQ;
            end
        WAIT_CMPLT:
            if ( IO_Bus2IP_MstRd_Cmplt == 1 || IO_Bus2IP_MstWrt_Cmplt == 1 )
            begin
                bus_state_n <= WAIT_REQ;
            end
            else
            begin
                bus_state_n <= WAIT_CMPLT;
            end

        default:
            bus_state_n <= WAIT_REQ;

    endcase
end


//scaler state
assign preloading = ( state_c == PRELOAD_0 || state_c == PRELOAD_1 || state_c == PRELOAD_2 );
assign h_scaling = ( state_c == RDY_FOR_H || state_c == H_SCALE || state_c == NXT_ROW );
assign v_scaling = ( state_c == RDY_FOR_V || state_c == V_SCALE || state_c == NXT_COL );
assign reload_pixel_state = ( state_c == NXT_COL || state_c == NXT_ROW || state_c == RDY_FOR_V || state_c == RDY_FOR_H ); //reload pixel state


always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        OUT_WIDTH <= 0;
    end
    else
    begin
        OUT_WIDTH <= ORIGIN_WIDTH[ WIDTH_BITS_LENGTH - 1: 5 ] * down_level - 1;
    end
end
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        OUT_HEIGHT <= 0;
    end
    else
    begin
        OUT_HEIGHT <= ORIGIN_HEIGHT[ HEIGHT_BITS_LENGTH - 1: 5 ] * down_level - 1;
    end
end


//**************************************
// special case execution begin
//**************************************
always@( posedge clk )
begin
    if ( ~reset_n || state_n == CMPLT )
    begin
        nxt_col_at_ram_full <= 0;
    end
    else if ( state_c == V_SCALE && state_n != V_STALL )
    begin
        nxt_col_at_ram_full <= 0;
    end
    else if ( state_c == NXT_COL && ram_full_0 && ram_full_1 )
    begin
        nxt_col_at_ram_full <= 1;
    end
    else
    begin
        nxt_col_at_ram_full <= nxt_col_at_ram_full;
    end
end

always@( posedge clk )
begin
    if ( ~reset_n || state_n == CMPLT )
    begin
        scaled_pixel_at_ram_full <= 0;
    end
    else if ( state_c == V_SCALE && state_n != V_STALL )
    begin
        scaled_pixel_at_ram_full <= 0;
    end
    else if ( state_c == NXT_COL && ram_full_0 && ram_full_1 )
    begin
        scaled_pixel_at_ram_full <= ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
    end
    else
    begin
        scaled_pixel_at_ram_full <= scaled_pixel_at_ram_full;
    end
end

//**************************************
// special case execution end
//**************************************

//read memory request count
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        rd_mem_req_cnt <= 0;
    end
    else if ( state_n == IDLE || state_n == CMPLT )
    begin
        rd_mem_req_cnt <= 0;
    end
    else if ( IO_Bus2IP_MstRd_Cmplt )
    begin
        rd_mem_req_cnt <= rd_mem_req_cnt + 1;
    end
    else
    begin
        rd_mem_req_cnt <= rd_mem_req_cnt;
    end
end

//write last memory request
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        wrt_mem_last_one_req <= 0;
    end
    else if ( state_n == IDLE || state_n == CMPLT )
    begin
        wrt_mem_last_one_req <= 0;
    end
    else if ( wrt_mem_req && state_c == WRT_MEM_1 )
    begin
        wrt_mem_last_one_req <= 1;
    end
    else
    begin
        wrt_mem_last_one_req <= wrt_mem_last_one_req;
    end
end

// row pixel index from 0 to OUT_WIDTH to calculate reference_table
always@( * )
begin
    if ( ~reset_n )
    begin
        scaling_pxl_idx <= 0;
    end
    else if ( scaling_pxl_idx_reg == OUT_WIDTH && state_c == H_SCALE || scaling_pxl_idx_reg == OUT_HEIGHT && state_c == V_SCALE )
    begin
        scaling_pxl_idx <= 0;
    end
    else if ( preloading || h_scaling || v_scaling )
    begin
        scaling_pxl_idx <= scaling_pxl_idx_reg + 1;
    end
    else
    begin
        scaling_pxl_idx <= 0;
    end
end

always@( posedge clk )
begin
    if ( ~reset_n || state_n == CMPLT )
    begin
        scaling_pxl_idx_reg <= 0;
    end
    else if ( state_c == H_STALL || state_n == H_STALL || state_n == V_STALL || state_c == V_STALL || nxt_col_at_ram_full )
    begin
        scaling_pxl_idx_reg <= scaling_pxl_idx_reg;
    end
    else
    begin
        scaling_pxl_idx_reg <= scaling_pxl_idx;
    end
end

always@( * )
begin
    if ( ~reset_n )
    begin
        scaled_pixel_index <= 0;
    end
    else if ( scaled_pixel_index_reg == OUT_WIDTH && state_c == H_SCALE || scaled_pixel_index_reg == OUT_HEIGHT && state_c == V_SCALE )
    begin
        scaled_pixel_index <= 0;
    end
    else if ( h_scaling || v_scaling )
    begin
        scaled_pixel_index <= scaled_pixel_index_reg + 1;
    end
    else
    begin
        scaled_pixel_index <= 0;
    end
end

always@( posedge clk )
begin
    if ( ~reset_n || state_n == CMPLT )
    begin
        scaled_pixel_index_reg <= 0;
    end
    else if ( state_n == H_STALL || state_c == H_STALL || state_n == V_STALL || state_c == V_STALL )
    begin
        scaled_pixel_index_reg <= scaled_pixel_index_reg;
    end
    else
    begin
        scaled_pixel_index_reg <= scaled_pixel_index;
    end
end

always@( posedge clk )
begin
    if ( ~reset_n || state_n == CMPLT )
    begin
        scaling_round_counter <= 0;
    end
    else if ( state_n == H_STALL || state_c == H_STALL || state_n == V_STALL || state_c == V_STALL || nxt_col_at_ram_full )
    begin
        scaling_round_counter <= scaling_round_counter;
    end
    else if ( state_n == WRT_RAM || state_n == WRT_MEM_0 )
    begin
        scaling_round_counter <= scaling_round_counter;
    end
    else if ( state_c == IDLE || state_c == CLN_BUF )
    begin
        scaling_round_counter <= 0;
    end
    else if ( ( scaling_pxl_idx_reg == OUT_WIDTH && state_c == H_SCALE ) || ( scaling_pxl_idx_reg == OUT_HEIGHT && state_c == V_SCALE ) )
    begin
        scaling_round_counter <= scaling_round_counter + 1;
    end
    else
    begin
        scaling_round_counter <= scaling_round_counter;
    end
end

always@( posedge clk )
begin
    if ( ~reset_n || state_n == CMPLT )
    begin
        left_edge_detection <= 0;
    end
    else if ( state_n == H_STALL || state_c == H_STALL || state_n == V_STALL || state_c == V_STALL || nxt_col_at_ram_full )
    begin
        left_edge_detection <= left_edge_detection;
    end
    else if ( ( pixel_number < 6 ) && scaling_pxl_idx_reg <= 6 )
    begin
        left_edge_detection <= 0;
    end
    else
    begin
        left_edge_detection <= 5'b11111;
    end
end

always@( posedge clk )
begin
    if ( ~reset_n || state_n == CMPLT )
    begin
        right_edge_detection <= 0;
    end
    else if ( state_n == H_STALL || state_c == H_STALL || state_n == V_STALL || state_c == V_STALL || nxt_col_at_ram_full )
    begin
        right_edge_detection <= right_edge_detection;
    end
    else if ( pixel_number > 26 && ( scaling_pxl_idx > limit_length - 5 ) || ( state_n == NXT_ROW ) || ( state_n == NXT_COL ) || ( state_n == WRT_RAM ) || ( state_n == WRT_MEM_0 ) )
    begin
        right_edge_detection <= 0;
    end
    else
    begin
        right_edge_detection <= 5'b11111;
    end
end

always@( posedge clk )
begin

    if ( ~reset_n )
    begin
        write_last_pxl <= 0;
    end
    else if ( state_n == H_STALL || state_c == H_STALL || state_n == V_STALL || state_c == V_STALL || nxt_col_at_ram_full )
    begin
        write_last_pxl <= write_last_pxl;
    end

    else if ( state_c == WRT_RAM || state_c == WRT_MEM_0 )
    begin
        write_last_pxl <= write_last_pxl + 1;
    end

    else
    begin
        write_last_pxl <= 0;
    end
end

assign h_end = ( scaling_round_counter == IN_HEIGHT && scaling_pxl_idx_reg == OUT_WIDTH );
assign v_end = ( scaling_round_counter == OUT_WIDTH && scaling_pxl_idx_reg == OUT_HEIGHT );

always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        h_flag <= 0;
    end
    else if ( state_n == H_STALL || state_c == H_STALL || state_n == V_STALL || state_c == V_STALL )
    begin
        h_flag <= h_flag;
    end
    else if ( state_n == IDLE || state_n == CMPLT )
    begin
        h_flag <= 0;
    end
    else if ( state_c == H_SCALE && h_flag == 0 )
    begin
        h_flag <= 1;
    end
    else
    begin
        h_flag <= h_flag;
    end
end

always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        v_flag <= 0;
    end
    else if ( state_n == H_STALL || state_c == H_STALL || state_n == V_STALL || state_c == V_STALL )
    begin
        v_flag <= v_flag;
    end
    else if ( state_n == IDLE || state_n == CMPLT )
    begin
        v_flag <= 0;
    end
    else if ( state_n == RDY_FOR_V && v_flag == 0 )
    begin
        v_flag <= 1;
    end
    else
    begin
        v_flag <= v_flag;
    end
end

always@( posedge clk )
    if ( ~reset_n )
    begin
        rd_mem_cmlt <= 0;
    end
    else if ( state_c == RDY_FOR_H || state_n == CMPLT )
    begin
        rd_mem_cmlt <= 0;
    end
    else if ( state_c == RD_MEM && wrt_ram_addr_0 == WRT_RAM_ADDR_LIMIT_FOR_INPUT_AND_OUTPUT )
    begin
        rd_mem_cmlt <= 1;
    end
    else
    begin
        rd_mem_cmlt <= rd_mem_cmlt;
    end

//=========================================
// implementation of ping-pong ram
// function:
// buffer input pixels
// buffer output pixels
//=========================================

//RAM_0 state
always@( posedge clk )
begin
    if ( ~reset_n || state_n == CMPLT )
    begin
        ram_state_c_0 <= WRT;
    end
    else
    begin
        ram_state_c_0 <= ram_state_n_0;
    end
end
always@( * )
begin
    case ( ram_state_c_0 )
        RD:
            if ( h_rd_ram_cnt == PING_PONG_RAM_DEPTH || v_flag && v_rd_ram_ptr == 0 && IO_Bus2IP_MstWrt_Cmplt )
            begin
                ram_state_n_0 = WRT;
            end
            else
            begin
                ram_state_n_0 = RD;
            end
        WRT:
            if ( pixel_in_valid && wrt_ram_addr_0 == WRT_RAM_ADDR_LIMIT_FOR_INPUT_AND_OUTPUT )
            begin
                ram_state_n_0 = RD;
            end
            else if ( wrt_ram_addr_0 == WRT_RAM_ADDR_LIMIT )
            begin
                ram_state_n_0 = RD;
            end
            else
            begin
                ram_state_n_0 = WRT;
            end
        default:
            ram_state_n_0 = RD;
    endcase
end

//RAM_1 state
always@( posedge clk )
begin
    if ( ~reset_n || state_n == CMPLT )
    begin
        ram_state_c_1 <= STALL;
    end
    else
    begin
        ram_state_c_1 <= ram_state_n_1;
    end
end

always@( * )
begin
    case ( ram_state_c_1 )
        RD:
            if ( h_scaling && h_rd_ram_cnt == 0 || v_flag && v_rd_ram_ptr == 1 && IO_Bus2IP_MstWrt_Cmplt )
            begin
                ram_state_n_1 = WRT;
            end
            else
            begin
                ram_state_n_1 = RD;
            end
        WRT:
            if ( state_n == CLN_BUF )
            begin
                ram_state_n_1 = STALL;
            end
            else if ( pixel_in_valid && wrt_ram_addr_1 == WRT_RAM_ADDR_LIMIT_FOR_INPUT_AND_OUTPUT )
            begin
                ram_state_n_1 = RD;
            end
            else if ( wrt_ram_addr_1 == WRT_RAM_ADDR_LIMIT )
            begin
                ram_state_n_1 = RD;
            end
            else
            begin
                ram_state_n_1 = WRT;
            end
        STALL:
            if ( pixel_in_valid && wrt_ram_addr_0 == WRT_RAM_ADDR_LIMIT_FOR_INPUT_AND_OUTPUT )
            begin
                ram_state_n_1 = WRT;
            end
            else if ( wrt_ram_addr_0 == WRT_RAM_ADDR_LIMIT )
            begin
                ram_state_n_1 = WRT;
            end
            else
            begin
                ram_state_n_1 = STALL;
            end
        default:
            ram_state_n_1 = RD;
    endcase
end

genvar PING_PONG_RAM_0_PORT;
generate
    for ( PING_PONG_RAM_0_PORT = 0;PING_PONG_RAM_0_PORT < PIXEL_BUFFER_SIZE;PING_PONG_RAM_0_PORT = PING_PONG_RAM_0_PORT + 1 )
    begin
        sram
            #( .DATA_WIDTH( 8 ), .ADDR_WIDTH( PING_PONG_RAM_DEPTH_BITS_LENGTH - 1 ), .RAM_SIZE( PING_PONG_RAM_DEPTH ) )
            ram(
                .clk( clk ),
                .en( ram_en ),
                .we( ram_we_0[ PING_PONG_RAM_0_PORT ] & ( ~wrt_ram_ptr ) & ( ram_state_c_0 == WRT ) ),
                .addr( ram_addr_0 ),
                .data_i( data_in[ PING_PONG_RAM_0_PORT ] ),
                .data_o( data_out_0[ PING_PONG_RAM_0_PORT ] )
            );
    end
endgenerate

genvar PING_PONG_RAM_1_PORT;
generate
    for ( PING_PONG_RAM_1_PORT = 0;PING_PONG_RAM_1_PORT < PIXEL_BUFFER_SIZE;PING_PONG_RAM_1_PORT = PING_PONG_RAM_1_PORT + 1 )
    begin
        sram
            #( .DATA_WIDTH( 8 ), .ADDR_WIDTH( PING_PONG_RAM_DEPTH_BITS_LENGTH - 1 ), .RAM_SIZE( PING_PONG_RAM_DEPTH ) )
            ram(
                .clk( clk ),
                .en( ram_en ),
                .we( ram_we_1[ PING_PONG_RAM_1_PORT ] & ( wrt_ram_ptr ) & ( ram_state_c_1 == WRT ) ),
                .addr( ram_addr_1 ),
                .data_i( data_in[ PING_PONG_RAM_1_PORT ] ),
                .data_o( data_out_1[ PING_PONG_RAM_1_PORT ] )
            );
    end
endgenerate

//RAM_0 & RAM_1 input ports
assign data_in[ 0 ] = ( pixel_in_valid ) ? pixel_in_reg[ 0 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 1 ] = ( pixel_in_valid ) ? pixel_in_reg[ 1 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 2 ] = ( pixel_in_valid ) ? pixel_in_reg[ 2 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 3 ] = ( pixel_in_valid ) ? pixel_in_reg[ 3 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 4 ] = ( pixel_in_valid ) ? pixel_in_reg[ 0 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 5 ] = ( pixel_in_valid ) ? pixel_in_reg[ 1 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 6 ] = ( pixel_in_valid ) ? pixel_in_reg[ 2 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 7 ] = ( pixel_in_valid ) ? pixel_in_reg[ 3 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 8 ] = ( pixel_in_valid ) ? pixel_in_reg[ 0 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 9 ] = ( pixel_in_valid ) ? pixel_in_reg[ 1 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 10 ] = ( pixel_in_valid ) ? pixel_in_reg[ 2 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 11 ] = ( pixel_in_valid ) ? pixel_in_reg[ 3 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 12 ] = ( pixel_in_valid ) ? pixel_in_reg[ 0 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 13 ] = ( pixel_in_valid ) ? pixel_in_reg[ 1 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 14 ] = ( pixel_in_valid ) ? pixel_in_reg[ 2 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 15 ] = ( pixel_in_valid ) ? pixel_in_reg[ 3 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 16 ] = ( pixel_in_valid ) ? pixel_in_reg[ 0 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 17 ] = ( pixel_in_valid ) ? pixel_in_reg[ 1 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 18 ] = ( pixel_in_valid ) ? pixel_in_reg[ 2 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 19 ] = ( pixel_in_valid ) ? pixel_in_reg[ 3 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 20 ] = ( pixel_in_valid ) ? pixel_in_reg[ 0 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 21 ] = ( pixel_in_valid ) ? pixel_in_reg[ 1 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 22 ] = ( pixel_in_valid ) ? pixel_in_reg[ 2 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 23 ] = ( pixel_in_valid ) ? pixel_in_reg[ 3 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 24 ] = ( pixel_in_valid ) ? pixel_in_reg[ 0 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 25 ] = ( pixel_in_valid ) ? pixel_in_reg[ 1 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 26 ] = ( pixel_in_valid ) ? pixel_in_reg[ 2 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 27 ] = ( pixel_in_valid ) ? pixel_in_reg[ 3 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 28 ] = ( pixel_in_valid ) ? pixel_in_reg[ 0 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 29 ] = ( pixel_in_valid ) ? pixel_in_reg[ 1 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 30 ] = ( pixel_in_valid ) ? pixel_in_reg[ 2 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;
assign data_in[ 31 ] = ( pixel_in_valid ) ? pixel_in_reg[ 3 ] : ( nxt_col_at_ram_full ) ? scaled_pixel_at_ram_full : ( scaled_pixel >= 255 ) ? 255 : ( scaled_pixel <= 0 ) ? 0 : scaled_pixel;

//that ram_full_0 signal is high means RAM_0 is full.
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        ram_full_0 <= 0;
    end
    else if ( state_n == CMPLT )
    begin
        ram_full_0 <= 0;
    end
    else if ( ram_state_c_0 == WRT && wrt_ram_addr_0 == WRT_RAM_ADDR_LIMIT_FOR_INPUT_AND_OUTPUT && pixel_in_valid )
    begin
        ram_full_0 <= 1;
    end
    else if ( ram_state_c_0 == WRT && wrt_ram_addr_0 == WRT_RAM_ADDR_LIMIT )
    begin
        ram_full_0 <= 1;
    end
    else if ( ram_state_c_0 == RD && ( h_rd_ram_cnt == PING_PONG_RAM_DEPTH || v_flag && v_rd_ram_ptr == 0 && IO_Bus2IP_MstWrt_Cmplt ) )
    begin
        ram_full_0 <= 0;
    end
    else
    begin
        ram_full_0 <= ram_full_0;
    end
end

//that ram_full_1 signal is high means RAM_1 is full.
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        ram_full_1 <= 0;
    end
    else if ( state_n == CMPLT )
    begin
        ram_full_1 <= 0;
    end
    else if ( ram_state_c_1 == WRT && wrt_ram_addr_1 == WRT_RAM_ADDR_LIMIT_FOR_INPUT_AND_OUTPUT && pixel_in_valid )
    begin
        ram_full_1 <= 1;
    end
    else if ( ram_state_c_1 == WRT && wrt_ram_addr_1 == WRT_RAM_ADDR_LIMIT )
    begin
        ram_full_1 <= 1;
    end
    else if ( ram_state_c_1 == RD && ( h_scaling && h_rd_ram_cnt == 0 || v_flag && v_rd_ram_ptr == 1 && IO_Bus2IP_MstWrt_Cmplt ) )
    begin
        ram_full_1 <= 0;
    end
    else
    begin
        ram_full_1 <= ram_full_1;
    end
end

//choose one which RAM_0 or RAM_1 to write
always@( posedge clk )
begin
    if ( ~reset_n || state_n == CMPLT )
    begin
        wrt_ram_ptr <= 0;
    end
    else if ( pixel_in_valid && ( wrt_ram_addr_0 == WRT_RAM_ADDR_LIMIT_FOR_INPUT_AND_OUTPUT || wrt_ram_addr_1 == WRT_RAM_ADDR_LIMIT_FOR_INPUT_AND_OUTPUT ) )
    begin
        wrt_ram_ptr <= wrt_ram_ptr + 1;
    end
    else if ( wrt_ram_ptr == 0 && wrt_ram_addr_0 == WRT_RAM_ADDR_LIMIT || wrt_ram_ptr == 1 && wrt_ram_addr_1 == WRT_RAM_ADDR_LIMIT )
    begin
        wrt_ram_ptr <= wrt_ram_ptr + 1;
    end
    else
    begin
        wrt_ram_ptr <= wrt_ram_ptr;
    end
end

//choose one which RAM_0 or RAM_1 to read at horizontal scaling stage
assign h_rd_ram_ptr = ( h_rd_ram_cnt >= PING_PONG_RAM_DEPTH );

always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        h_rd_ram_cnt <= 0;
    end
    else if ( state_n == IDLE || state_n == CMPLT )
    begin
        h_rd_ram_cnt <= 0;
    end
    else if ( h_scaling && reload_pixel_buffer == 1 )
    begin
        h_rd_ram_cnt <= h_rd_ram_cnt + 1;
    end
    else
    begin
        h_rd_ram_cnt <= h_rd_ram_cnt;
    end
end

//choose one which RAM_0 or RAM_1 to read at vertical scaling stage
always@( posedge clk )
begin
    if ( ~reset_n || state_n == CMPLT )
    begin
        v_rd_ram_ptr <= 0;
    end
    else
    begin
        v_rd_ram_ptr <= ( v_flag && IO_Bus2IP_MstWrt_Cmplt ) ? ~v_rd_ram_ptr : v_rd_ram_ptr;
    end
end

assign ram_en = 1; // block ram always work

//RAM_0 and RAM_1 address port
assign ram_addr_0 = ( ( pixel_in_valid || v_scaled_pixel_valid ) && ( ram_state_c_0 == WRT ) ) ? wrt_ram_addr_0[ WRT_RAM_ADDR_LIMIT_BITS_LENGTH - 1 : 5 ] : rd_ram_addr[ RD_RAM_ADDR_BITS_LENGTH - 1 : 5 ]; // each 32 pixels store from ram0~ram31 and sram_addr+1
assign ram_addr_1 = ( ( pixel_in_valid || v_scaled_pixel_valid ) && ( ram_state_c_1 == WRT ) ) ? wrt_ram_addr_1[ WRT_RAM_ADDR_LIMIT_BITS_LENGTH - 1 : 5 ] : rd_ram_addr[ RD_RAM_ADDR_BITS_LENGTH - 1 : 5 ]; // each 32 pixels store from ram0~ram31 and sram_addr+1

//RAM_0 address for writing
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        wrt_ram_addr_0 <= 0;
    end
    else if ( state_n == IDLE || state_n == CMPLT )
    begin
        wrt_ram_addr_0 <= 0;
    end
    else if ( state_n == V_STALL || state_c == V_STALL )
    begin
        wrt_ram_addr_0 <= wrt_ram_addr_0;
    end
    else if ( v_scaled_pixel_valid & ( ram_state_c_0 == WRT ) && wrt_ram_ptr == 0 )
    begin
        wrt_ram_addr_0 <= wrt_ram_addr_0 + 1;
    end
    else if ( pixel_in_valid & ( ram_state_c_0 == WRT ) && wrt_ram_ptr == 0 )
    begin
        wrt_ram_addr_0 <= wrt_ram_addr_0 + 4;
    end
    else
    begin
        wrt_ram_addr_0 <= wrt_ram_addr_0;
    end
end

//RAM_1 address for writing
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        wrt_ram_addr_1 <= 0;
    end
    else if ( state_n == IDLE || state_n == CMPLT )
    begin
        wrt_ram_addr_1 <= 0;
    end
    else if ( state_n == V_STALL || state_c == V_STALL )
    begin
        wrt_ram_addr_1 <= wrt_ram_addr_1;
    end
    else if ( v_scaled_pixel_valid & ( ram_state_c_1 == WRT ) && wrt_ram_ptr == 1 )
    begin
        wrt_ram_addr_1 <= wrt_ram_addr_1 + 1;
    end
    else if ( pixel_in_valid & ( ram_state_c_1 == WRT ) && wrt_ram_ptr == 1 )
    begin
        wrt_ram_addr_1 <= wrt_ram_addr_1 + 4;
    end
    else
    begin
        wrt_ram_addr_1 <= wrt_ram_addr_1;
    end
end

//RAM_0 and RAM_1 addresses for reading
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        rd_ram_addr <= 0;
    end
    else if ( state_n == IDLE || v_flag && IO_Bus2IP_MstWrt_Cmplt || state_n == CMPLT )
    begin
        rd_ram_addr <= 0;
    end  //0~WRT_RAM_ADDR_LIMIT in v_scaling
    else if ( bus_wrt_valid )
    begin
        rd_ram_addr <= rd_ram_addr + 4;
    end
    else if ( state_c == RDY_FOR_H || state_c == NXT_ROW )
    begin
        rd_ram_addr <= rd_ram_addr;
    end
    else if ( state_n == NXT_ROW || state_n == RDY_FOR_H || ( h_scaling && reload_pixel_buffer == 1 ) )
    begin
        rd_ram_addr[ RD_RAM_ADDR_BITS_LENGTH - 1: 5 ] <= rd_ram_addr[ RD_RAM_ADDR_BITS_LENGTH - 1: 5 ] + 1;
    end
    else
    begin
        rd_ram_addr <= rd_ram_addr;
    end
end

//RAM_0 write enable
always@( posedge clk )
begin
    if ( ~reset_n || state_n == IDLE )
    begin
        ram_we_0 <= 4'b1111;
    end
    else if ( state_n == V_STALL || state_c == V_STALL )
    begin
        ram_we_0 <= ram_we_0;
    end
    else if ( state_c == RDY_FOR_V )
    begin
        ram_we_0 <= 1'b1;
    end
    else if ( ( v_scaled_pixel_valid ) && ( wrt_ram_ptr == 0 ) && ( ram_state_c_0 == WRT ) )
    begin
        ram_we_0 <= { ram_we_0[ 30: 0 ], ram_we_0[ 31 ] };
    end
    else if ( ( pixel_in_valid ) && ( wrt_ram_ptr == 0 ) && ( ram_state_c_0 == WRT ) )
    begin
        ram_we_0 <= { ram_we_0[ 27: 0 ], ram_we_0[ 31: 28 ] };
    end
    else
    begin
        ram_we_0 <= ram_we_0;
    end
end

//RAM_1 write enable
always@( posedge clk )
begin
    if ( ~reset_n || state_n == IDLE )
    begin
        ram_we_1 <= 4'b1111;
    end
    else if ( state_n == V_STALL || state_c == V_STALL )
    begin
        ram_we_1 <= ram_we_1;
    end
    else if ( state_c == RDY_FOR_V )
    begin
        ram_we_1 <= 1'b1;
    end
    else if ( ( v_scaled_pixel_valid ) && ( wrt_ram_ptr == 1 ) && ( ram_state_c_1 == WRT ) )
    begin
        ram_we_1 <= { ram_we_1[ 30: 0 ], ram_we_1[ 31 ] };
    end
    else if ( ( pixel_in_valid ) && ( wrt_ram_ptr == 1 ) && ( ram_state_c_1 == WRT ) )
    begin
        ram_we_1 <= { ram_we_1[ 27: 0 ], ram_we_1[ 31: 28 ] };
    end
    else
    begin
        ram_we_1 <= ram_we_1;
    end
end

//=========================================
// implementation of inter_ram
// function:
// inter_ram to store ALL h scaled pixel
//=========================================

//0~31, total 32 row inter_ram
genvar INTER_RAM_PORT;
generate
    for ( INTER_RAM_PORT = 0;INTER_RAM_PORT < PIXEL_BUFFER_SIZE;INTER_RAM_PORT = INTER_RAM_PORT + 1 )
    begin
        sram
            #( .DATA_WIDTH( 8 ), .ADDR_WIDTH( INTER_RAM_DEPTH_BITS_LENGTH ), .RAM_SIZE( INTER_RAM_DEPTH ) )
            ram(
                .clk( clk ),
                .en( inter_ram_en ),
                .we( inter_ram_we[ INTER_RAM_PORT ] ),
                .addr( inter_ram_addr ),
                .data_i( inter_data_in ),
                .data_o( inter_data_out[ INTER_RAM_PORT ] )
            );
    end
endgenerate

assign wrt_inter_ram_addr = base + offset;

//when h scaled pixel is valid inter_ram work
assign inter_ram_en = h_scaled_pixel_valid || ( preloading && h_flag == 1 );
//pixel need to clip in range 0 ~ 255
assign inter_data_in = ( scaled_pixel >= 255 ) ? 255
       : ( scaled_pixel <= 0 ) ? 0
       : scaled_pixel;
// inter_ram_addr have 2 case:
// case 1: store h scaled pixel at period of h scaling define by base + offset
// case 2: load  h scaled pixel at period of v scaling define by rd_inter_ram_addr
assign inter_ram_addr = ( h_scaled_pixel_valid == 1 )
       ? wrt_inter_ram_addr : rd_inter_ram_addr[ INTER_RAM_DEPTH_BITS_LENGTH - 1 + PIXEL_BUFFER_SIZE_BITS_LENGTH - 1 : 5 ];

always@( posedge clk )
begin
    if ( ~reset_n || state_n == CMPLT )
    begin
        base <= 0;
    end
    else if ( state_c == H_STALL || state_n == H_STALL )
    begin
        base <= base;
    end
    else if ( inter_ram_we[ 31 ] && write_next_INTER_RAM_PORT_reg )
    begin
        base <= scaling_round_counter[ WIDTH_BITS_LENGTH - 1: 5 ];
    end //every h_scale 32 row need to change
    else
    begin
        base <= base;
    end
end

always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        offset <= 0;
    end
    else if ( write_next_INTER_RAM_PORT_reg == 1 || state_n == CMPLT )
    begin
        offset <= 0;
    end
    else if ( h_scaled_pixel_valid == 1 )
    begin
        offset <= offset + ORIGIN_HEIGHT[ HEIGHT_BITS_LENGTH - 1: 5 ];
    end  //ORIGIN_HEIGHT/32(32 is pixel buffer)
    else
    begin
        offset <= offset;
    end
end

always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        rd_inter_ram_addr <= 0;
    end
    else if ( state_n == CLN_BUF || state_n == CMPLT )
    begin
        rd_inter_ram_addr <= 0;
    end
    else if ( state_c == RDY_FOR_V || state_c == NXT_COL )
    begin
        rd_inter_ram_addr <= rd_inter_ram_addr;
    end
    else if ( state_n == RDY_FOR_V || state_n == NXT_COL || ( reference_table_index == 0 && reload_pixel_buffer == 1 ) )
    begin
        rd_inter_ram_addr[ INTER_RAM_DEPTH_BITS_LENGTH - 1 + PIXEL_BUFFER_SIZE_BITS_LENGTH - 1: 5 ] <= rd_inter_ram_addr[ INTER_RAM_DEPTH_BITS_LENGTH - 1 + PIXEL_BUFFER_SIZE_BITS_LENGTH - 1: 5 ] + 1;
    end //reference_table_index==0 then reload_pixel_buffer must 1
    else
    begin
        rd_inter_ram_addr <= rd_inter_ram_addr;
    end
end

assign write_next_INTER_RAM_PORT = ( state_c != IDLE && state_c != INIT ) && ( scaled_pixel_index_reg == OUT_WIDTH );

//when scaled_pixel_index_reg == OUT_WIDTH, write_next_INTER_RAM_PORT_reg is 1 and inter_ram_we shift next one
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        write_next_INTER_RAM_PORT_reg <= 0;
    end
    else if ( state_n == CLN_BUF )
    begin
        write_next_INTER_RAM_PORT_reg <= 0;
    end
    else if ( write_next_INTER_RAM_PORT )
    begin
        write_next_INTER_RAM_PORT_reg <= 1;
    end
    else
    begin
        write_next_INTER_RAM_PORT_reg <= 0;
    end
end

//choose which inter_ram to write by inter_ram_we
always@( posedge clk )
begin
    if ( ~reset_n || state_n == CMPLT )
    begin
        inter_ram_we <= 1;
    end
    else if ( state_n == CLN_BUF )
    begin
        inter_ram_we <= 0;
    end
    else if ( write_next_INTER_RAM_PORT_reg == 1 )
    begin
        inter_ram_we <= { inter_ram_we[ 30: 0 ], inter_ram_we[ 31 ] };
    end
    else
    begin
        inter_ram_we <= inter_ram_we;
    end
end

//=================================================
// implementation of reference_table
// function:
// offers filter number and pixel number
// reference_table[index] = ( (index <<9) + ((256) - ((down_level)<<3)) +((down_level)>>1)) / ((down_level));
//=================================================

always@( posedge clk )
    if ( ~reset_n )
    begin
        reference_table_done <= 0;
    end
    else
    begin
        reference_table_done <= ( reference_table[ down_level - 1 ] != 0 );
    end

always@( posedge clk )
begin
    if ( ~reset_n || state_n == CMPLT )
    begin
        pixel_counter_0 <= 0;
        en_0 <= 0;
    end
    else if ( state_c == WRT_MEM_1 && state_n == IDLE || state_n == CMPLT )
    begin
        pixel_counter_0 <= 0;
    end
    else if ( state_c == IDLE && state_n == RD_MEM )
    begin
        pixel_counter_0 <= pixel_counter_0 + 2;
        en_0 <= 1;
    end
    else if ( divider_out_valid_0 &&
              ( down_level[ 0 ] && pixel_counter_0 != ( down_level + 1 ) ||   //down level is odd.
                ~down_level[ 0 ] && pixel_counter_0 != ( down_level ) ) )
    begin //down level is even.
        pixel_counter_0 <= pixel_counter_0 + 2;
        en_0 <= 1;
    end
    else
    begin
        pixel_counter_0 <= pixel_counter_0;
        en_0 <= 0;
    end
end

always@( posedge clk )
begin
    if ( ~reset_n || state_n == CMPLT )
    begin
        pixel_counter_1 <= 1;
        en_1 <= 0;
    end
    else if ( state_c == WRT_MEM_1 && state_n == IDLE || state_n == CMPLT )
    begin
        pixel_counter_1 <= 1;

    end
    else if ( state_c == IDLE && state_n == RD_MEM )
    begin
        pixel_counter_1 <= pixel_counter_1 + 2;
        en_1 <= 1;
    end
    else if ( divider_out_valid_1 &&
              ( down_level[ 0 ] && pixel_counter_1 != ( down_level ) ||   //down level is odd.
                ~down_level[ 0 ] && pixel_counter_1 != ( down_level + 1 ) ) )
    begin //down level is even.
        pixel_counter_1 <= pixel_counter_1 + 2;
        en_1 <= 1;
    end
    else
    begin
        pixel_counter_1 <= pixel_counter_1;
        en_1 <= 0;
    end
end

assign dividend_0 = C0_0 - C1 + C2;
assign dividend_1 = C0_1 - C1 + C2;
assign divisor = down_level;
divider d0( .clk( clk ), .rst( ~reset_n ), .en( en_0 ), .dividend( dividend_0 ), .divisor( divisor ), .quotient( quotient_0 ), .out_valid( divider_out_valid_0 ) );
divider d1( .clk( clk ), .rst( ~reset_n ), .en( en_1 ), .dividend( dividend_1 ), .divisor( divisor ), .quotient( quotient_1 ), .out_valid( divider_out_valid_1 ) );

always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        C0_0 <= 0;
    end
    else
    begin
        C0_0 <= { pixel_counter_0, 1'b1, 8'b0 };
    end
end

always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        C0_1 <= 0;

    end
    else
    begin
        C0_1 <= { pixel_counter_1, 1'b1, 8'b0 };
    end
end

always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        C1 <= 0;
        C2 <= 0;
    end
    else
    begin
        C1 <= down_level << 3;
        C2 <= down_level >> 1;
    end
end

reg [ 4: 0 ] odd_index, even_index;

always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        even_index <= 0;
    end
    else if ( state_n == CMPLT )
    begin
        even_index <= 0;
    end
    else if ( en_0 )
    begin
        even_index <= pixel_counter_0 - 2;
    end
    else
    begin
        even_index <= even_index;
    end

end
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        odd_index <= 1;
    end
    else if ( state_n == CMPLT )
    begin
        odd_index <= 1;
    end
    else if ( en_1 )
    begin
        odd_index <= pixel_counter_1 - 2;
    end
    else
    begin
        odd_index <= odd_index;
    end

end
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        reference_table_index <= 0;
    end
    else if ( state_n == H_STALL || state_c == H_STALL || state_n == V_STALL || state_c == V_STALL || nxt_col_at_ram_full )
    begin
        reference_table_index <= reference_table_index;
    end
    else if ( reference_table_index == ( down_level - 1 ) || state_n == CMPLT )
    begin
        reference_table_index <= 0;
    end
    else if ( preloading || h_scaling || v_scaling )
    begin
        reference_table_index <= reference_table_index + 1;
    end
    else
    begin
        reference_table_index <= reference_table_index;
    end


end



always@( posedge clk )
begin
    if ( ~reset_n || state_n == CMPLT )
    begin
        reference_table[ 0 ] <= 0;
        reference_table[ 2 ] <= 0;
        reference_table[ 4 ] <= 0;
        reference_table[ 6 ] <= 0;
        reference_table[ 8 ] <= 0;
        reference_table[ 10 ] <= 0;
        reference_table[ 12 ] <= 0;
        reference_table[ 14 ] <= 0;
        reference_table[ 16 ] <= 0;
        reference_table[ 18 ] <= 0;
        reference_table[ 20 ] <= 0;
        reference_table[ 22 ] <= 0;
        reference_table[ 24 ] <= 0;
        reference_table[ 26 ] <= 0;
        reference_table[ 28 ] <= 0;
        reference_table[ 30 ] <= 0;

    end
    else if ( divider_out_valid_0 )
    begin

        reference_table[ even_index ] <= quotient_0;
    end
    else
    begin
        reference_table[ 0 ] <= reference_table[ 0 ];
        reference_table[ 2 ] <= reference_table[ 2 ];
        reference_table[ 4 ] <= reference_table[ 4 ];
        reference_table[ 6 ] <= reference_table[ 6 ];
        reference_table[ 8 ] <= reference_table[ 8 ];
        reference_table[ 10 ] <= reference_table[ 10 ];
        reference_table[ 12 ] <= reference_table[ 12 ];
        reference_table[ 14 ] <= reference_table[ 14 ];
        reference_table[ 16 ] <= reference_table[ 16 ];
        reference_table[ 18 ] <= reference_table[ 18 ];
        reference_table[ 20 ] <= reference_table[ 20 ];
        reference_table[ 22 ] <= reference_table[ 22 ];
        reference_table[ 24 ] <= reference_table[ 24 ];
        reference_table[ 26 ] <= reference_table[ 26 ];
        reference_table[ 28 ] <= reference_table[ 28 ];
        reference_table[ 30 ] <= reference_table[ 30 ];
    end
end
always@( posedge clk )
begin
    if ( ~reset_n || state_n == CMPLT )
    begin
        reference_table[ 1 ] <= 0;
        reference_table[ 3 ] <= 0;
        reference_table[ 5 ] <= 0;
        reference_table[ 7 ] <= 0;
        reference_table[ 9 ] <= 0;
        reference_table[ 11 ] <= 0;
        reference_table[ 13 ] <= 0;
        reference_table[ 15 ] <= 0;
        reference_table[ 17 ] <= 0;
        reference_table[ 19 ] <= 0;
        reference_table[ 21 ] <= 0;
        reference_table[ 23 ] <= 0;
        reference_table[ 25 ] <= 0;
        reference_table[ 27 ] <= 0;
        reference_table[ 29 ] <= 0;
        reference_table[ 31 ] <= 0;

    end
    else if ( divider_out_valid_1 )
    begin

        reference_table[ odd_index ] <= quotient_1;
    end
    else
    begin
        reference_table[ 1 ] <= reference_table[ 1 ];
        reference_table[ 3 ] <= reference_table[ 3 ];
        reference_table[ 5 ] <= reference_table[ 5 ];
        reference_table[ 7 ] <= reference_table[ 7 ];
        reference_table[ 9 ] <= reference_table[ 9 ];
        reference_table[ 11 ] <= reference_table[ 11 ];
        reference_table[ 13 ] <= reference_table[ 13 ];
        reference_table[ 15 ] <= reference_table[ 15 ];
        reference_table[ 17 ] <= reference_table[ 17 ];
        reference_table[ 19 ] <= reference_table[ 19 ];
        reference_table[ 21 ] <= reference_table[ 21 ];
        reference_table[ 23 ] <= reference_table[ 23 ];
        reference_table[ 25 ] <= reference_table[ 25 ];
        reference_table[ 27 ] <= reference_table[ 27 ];
        reference_table[ 29 ] <= reference_table[ 29 ];
        reference_table[ 31 ] <= reference_table[ 31 ];
    end
end

always@( * )
begin
    if ( preloading || h_scaling || v_scaling )
    begin
        { pixel_number, filter_number } <= reference_table[ reference_table_index ];
    end
    else
    begin
        { pixel_number, filter_number } <= 0;
    end
end

always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        { pixel_number_reg, filter_number_reg } <= 0;
    end
    else if ( state_n == H_STALL || state_c == H_STALL || state_n == V_STALL || state_c == V_STALL || nxt_col_at_ram_full )
    begin
        { pixel_number_reg, filter_number_reg } <= { pixel_number_reg, filter_number_reg };
    end
    else
    begin
        { pixel_number_reg, filter_number_reg } <= { pixel_number, filter_number };
    end
end


//=================================================
// implementation of pixel_buffer
// function:
// store 32 x 8 bits pixel
//=================================================

//load ram to main buf
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        for ( i = 0;i < PIXEL_BUFFER_SIZE;i = i + 1 )
        begin
            pixel_buffer[ i ] <= 0;
        end
    end
    else if ( state_c == H_STALL || state_c == V_STALL || nxt_col_at_ram_full )
    begin
        for ( i = 0;i < PIXEL_BUFFER_SIZE;i = i + 1 )
        begin
            pixel_buffer[ i ] <= pixel_buffer[ i ];
        end
    end
    else if ( h_scaling && reload_pixel_buffer == 1 )
    begin
        for ( i = 0;i < PIXEL_BUFFER_SIZE;i = i + 1 )
        begin
            pixel_buffer[ i ] <= ( h_rd_ram_ptr == 0 ) ? data_out_0[ i ] : data_out_1[ i ];
        end
    end
    else if ( reference_table_index == 0 && reload_pixel_buffer == 1 )
    begin
        for ( i = 0;i < PIXEL_BUFFER_SIZE;i = i + 1 )
        begin
            pixel_buffer[ i ] <= inter_data_out[ i ];
        end
    end
    else
    begin
        for ( i = 0;i < PIXEL_BUFFER_SIZE;i = i + 1 )
        begin
            pixel_buffer[ i ] <= pixel_buffer[ i ];
        end
    end
end

always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        reload_pixel_buffer_tail_delay <= 0;
    end
    else if ( state_c == H_STALL || state_c == V_STALL || nxt_col_at_ram_full )
    begin
        reload_pixel_buffer_tail_delay <= reload_pixel_buffer_tail_delay;
    end
    else
    begin
        reload_pixel_buffer_tail_delay <= reload_pixel_buffer_tail;
    end
end

//load ram to extend buf
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        for ( i = 0;i < 5;i = i + 1 )
        begin
            pixel_buffer_tail[ i ] <= 0;
        end
    end
    else if ( state_c == H_STALL || state_c == V_STALL || nxt_col_at_ram_full )
    begin
        for ( i = 0;i < 5;i = i + 1 )
        begin
            pixel_buffer_tail[ i ] <= pixel_buffer_tail[ i ];
        end
    end
    else if ( ( preloading && v_flag || state_c == V_SCALE ) && reload_pixel_buffer_tail_delay == 1 )
    begin
        for ( i = 0;i < 5;i = i + 1 )
        begin
            pixel_buffer_tail[ i ] <= inter_data_out[ i ];
        end
    end
    else if ( ( preloading || state_c == H_SCALE ) && reload_pixel_buffer_tail_delay == 1 )
    begin
        for ( i = 0;i < 5;i = i + 1 )
        begin
            pixel_buffer_tail[ i ] <= ( h_rd_ram_ptr == 0 ) ? data_out_0[ i ] : data_out_1[ i ];
        end
    end
    else
    begin
        for ( i = 0;i < 5;i = i + 1 )
        begin
            pixel_buffer_tail[ i ] <= pixel_buffer_tail[ i ];
        end
    end
end

//load ram to pixel buffer head
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        for ( i = 0;i < 6;i = i + 1 )
        begin
            pixel_buffer_head[ i ] <= 0;
        end
    end
    else if ( state_n == H_STALL || state_c == H_STALL || state_n == V_STALL || state_c == V_STALL || nxt_col_at_ram_full )
    begin
        for ( i = 0;i < 6;i = i + 1 )
        begin
            pixel_buffer_head[ i ] <= pixel_buffer_head[ i ];
        end
    end
    else if ( reference_table_index == 0 && reload_pixel_buffer == 1 )
    begin //reference_table_index=0 && reload_pixel_buffer is same
        for ( i = 0;i < 6;i = i + 1 )
        begin
            pixel_buffer_head[ i ] <= pixel_buffer[ 26 + i ];
        end
    end
    else
    begin
        for ( i = 0;i < 6;i = i + 1 )
        begin
            pixel_buffer_head[ i ] <= pixel_buffer_head[ i ];
        end
    end
end

always@( * )
begin
    if ( state_c != V_STALL && state_n != V_STALL && state_c != IDLE && state_c != INIT && state_c != RD_MEM && reference_table_index == 0 )
    begin
        reload_pixel_buffer <= 1;
    end
    else
    begin
        reload_pixel_buffer <= 0;
    end
end

always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        load_buf_cnt <= 0;
    end
    else if ( state_n == H_STALL || state_c == H_STALL || state_n == V_STALL || state_c == V_STALL || nxt_col_at_ram_full )
    begin
        load_buf_cnt <= load_buf_cnt;
    end
    else if ( state_n == NXT_ROW || state_n == NXT_COL || state_n == CLN_BUF || state_n == IDLE || state_n == CMPLT )
    begin
        load_buf_cnt <= 0;
    end
    else if ( reload_pixel_state )
    begin
        load_buf_cnt <= load_buf_cnt;
    end
    else if ( reload_pixel_buffer == 1 )
    begin
        load_buf_cnt <= load_buf_cnt + 1;
    end
    else
    begin
        load_buf_cnt <= load_buf_cnt;
    end
end

always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        reload_pixel_buffer_tail <= 0;
    end
    else if ( state_n == H_STALL || state_c == H_STALL || state_n == V_STALL || state_c == V_STALL || nxt_col_at_ram_full )
    begin
        reload_pixel_buffer_tail <= reload_pixel_buffer_tail;
    end
    else if ( reload_pixel_buffer == 1 )
    begin
        reload_pixel_buffer_tail <= 1;
    end
    else
    begin
        reload_pixel_buffer_tail <= 0;
    end
end

always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        limit_length <= 0;
    end

    else if ( state_n == RDY_FOR_H )
    begin
        limit_length <= OUT_WIDTH;
    end
    else if ( state_n == RDY_FOR_V )
    begin
        limit_length <= OUT_HEIGHT;
    end
    else
    begin
        limit_length <= limit_length;
    end
end

//image edge limit
assign right_index_limit[ 0 ] = ( pixel_number_reg );
assign right_index_limit[ 1 ] = ( pixel_number_reg + 1 );
assign right_index_limit[ 2 ] = ( pixel_number_reg + 2 );
assign right_index_limit[ 3 ] = ( pixel_number_reg + 3 );
assign right_index_limit[ 4 ] = ( pixel_number_reg + 4 );
assign right_index_limit[ 5 ] = ( pixel_number_reg + 5 );

assign left_index_limit[ 0 ] = ( pixel_number_reg - 1 );
assign left_index_limit[ 1 ] = ( pixel_number_reg - 2 );
assign left_index_limit[ 2 ] = ( pixel_number_reg - 3 );
assign left_index_limit[ 3 ] = ( pixel_number_reg - 4 );
assign left_index_limit[ 4 ] = ( pixel_number_reg - 5 );
assign left_index_limit[ 5 ] = ( pixel_number_reg - 6 );

//choose index of pixel buffer to do MAC
assign left_index[ 5 ] = ( ~left_edge_detection[ 0 ] && left_index_limit[ 5 ] > 5 ) ? 0 : pixel_number_reg - 6;
assign left_index[ 4 ] = ( ~left_edge_detection[ 0 ] && left_index_limit[ 4 ] > 5 ) ? 0 : pixel_number_reg - 5;
assign left_index[ 3 ] = ( ~left_edge_detection[ 0 ] && left_index_limit[ 3 ] > 5 ) ? 0 : pixel_number_reg - 4;
assign left_index[ 2 ] = ( ~left_edge_detection[ 0 ] && left_index_limit[ 2 ] > 5 ) ? 0 : pixel_number_reg - 3;
assign left_index[ 1 ] = ( ~left_edge_detection[ 0 ] && left_index_limit[ 1 ] > 5 ) ? 0 : pixel_number_reg - 2;
assign left_index[ 0 ] = ( ~left_edge_detection[ 0 ] && left_index_limit[ 0 ] > 5 ) ? 0 : pixel_number_reg - 1;

assign center_index = ( ~right_edge_detection[ 0 ] && right_index_limit[ 0 ] > 31 ) ? 31 : pixel_number_reg ;
assign right_index[ 0 ] = ( ~right_edge_detection[ 0 ] && right_index_limit[ 1 ] > 31 ) ? 31 : pixel_number_reg + 1 ;
assign right_index[ 1 ] = ( ~right_edge_detection[ 0 ] && right_index_limit[ 2 ] > 31 ) ? 31 : pixel_number_reg + 2 ;
assign right_index[ 2 ] = ( ~right_edge_detection[ 0 ] && right_index_limit[ 3 ] > 31 ) ? 31 : pixel_number_reg + 3 ;
assign right_index[ 3 ] = ( ~right_edge_detection[ 0 ] && right_index_limit[ 4 ] > 31 ) ? 31 : pixel_number_reg + 4 ;
assign right_index[ 4 ] = ( ~right_edge_detection[ 0 ] && right_index_limit[ 5 ] > 31 ) ? 31 : pixel_number_reg + 5 ;


//=================================================
// implementation of multiplier and accumulator(MAC)
// function:
// filter and pixels calculating
//=================================================

//if signal is high horizontal scaled pixel is valid.
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        h_scaled_pixel_valid <= 0;
    end
    else if ( state_n == H_STALL || state_c == H_STALL || state_n == V_STALL || state_c == V_STALL || nxt_col_at_ram_full )
    begin
        h_scaled_pixel_valid <= 0;
    end
    else if ( state_c == RDY_FOR_H || state_n == IDLE || state_n == CLN_BUF || state_n == CMPLT )
    begin
        h_scaled_pixel_valid <= 0;
    end
    else if ( state_c == H_SCALE )
    begin
        h_scaled_pixel_valid <= 1;
    end
    else
    begin
        h_scaled_pixel_valid <= h_scaled_pixel_valid;
    end

end


//if signal is high vertical scaled pixel is valid.
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        v_scaled_pixel_valid <= 0;
    end
    else if ( state_c == RDY_FOR_H || state_n == IDLE || state_n == CLN_BUF || state_n == CMPLT )
    begin
        v_scaled_pixel_valid <= 0;
    end
    else if ( state_c == V_SCALE )
    begin
        v_scaled_pixel_valid <= 1;
    end
    else
    begin
        v_scaled_pixel_valid <= v_scaled_pixel_valid;
    end

end

always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        scaled_pixel <= 0;
    end
    else if ( state_n == H_STALL || state_c == H_STALL || state_n == V_STALL || state_c == V_STALL || nxt_col_at_ram_full )
    begin
        scaled_pixel <= scaled_pixel;
    end
    else
    begin
        scaled_pixel <= ( accumulate[ 0 ] + accumulate[ 1 ] ) >>> 7;
    end


end


//accumulator
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        accumulate[ 0 ] <= 0;
        accumulate[ 1 ] <= 0;
    end
    else if ( state_n == H_STALL || state_c == H_STALL || state_n == V_STALL || state_c == V_STALL || nxt_col_at_ram_full )
    begin
        accumulate[ 0 ] <= accumulate[ 0 ];
        accumulate[ 1 ] <= accumulate[ 1 ];


    end
    else
    begin
        accumulate[ 0 ] <= multiply[ 0 ] +
                  multiply[ 1 ] +
                  multiply[ 2 ] +
                  multiply[ 3 ] +
                  multiply[ 4 ] +
                  multiply[ 5 ];
        accumulate[ 1 ] <= multiply[ 6 ] +
                  multiply[ 7 ] +
                  multiply[ 8 ] +
                  multiply[ 9 ] +
                  multiply[ 10 ] +
                  multiply[ 11 ];
    end

end

//multiplier
always@( posedge clk )
begin
    if ( ~reset_n )
    begin

        for ( i = 0;i < 12;i = i + 1 )
        begin

            multiply[ i ] <= 0;

        end
    end
    else if ( state_n == H_STALL || state_c == H_STALL || state_n == V_STALL || state_c == V_STALL || nxt_col_at_ram_full )
    begin


        multiply[ 0 ] <= multiply[ 0 ];
        multiply[ 1 ] <= multiply[ 1 ];
        multiply[ 2 ] <= multiply[ 2 ];
        multiply[ 3 ] <= multiply[ 3 ];
        multiply[ 4 ] <= multiply[ 4 ];
        multiply[ 5 ] <= multiply[ 5 ];
        multiply[ 6 ] <= multiply[ 6 ];
        multiply[ 7 ] <= multiply[ 7 ];
        multiply[ 8 ] <= multiply[ 8 ];
        multiply[ 9 ] <= multiply[ 9 ];
        multiply[ 10 ] <= multiply[ 10 ];
        multiply[ 11 ] <= multiply[ 11 ];



    end
    else if ( preloading || ( state_c != RDY_FOR_H && h_scaling ) || ( state_c != RDY_FOR_V && v_scaling ) || state_c == WRT_RAM || state_c == WRT_MEM_0 )
    begin
        multiply[ 0 ] <= filter[ 0 ] * pixel[ 0 ];
        multiply[ 1 ] <= filter[ 1 ] * pixel[ 1 ];
        multiply[ 2 ] <= filter[ 2 ] * pixel[ 2 ];
        multiply[ 3 ] <= filter[ 3 ] * pixel[ 3 ];
        multiply[ 4 ] <= filter[ 4 ] * pixel[ 4 ];
        multiply[ 5 ] <= filter[ 5 ] * pixel[ 5 ];
        multiply[ 6 ] <= filter[ 6 ] * pixel[ 6 ];
        multiply[ 7 ] <= filter[ 7 ] * pixel[ 7 ];
        multiply[ 8 ] <= filter[ 8 ] * pixel[ 8 ];
        multiply[ 9 ] <= filter[ 9 ] * pixel[ 9 ];
        multiply[ 10 ] <= filter[ 10 ] * pixel[ 10 ];
        multiply[ 11 ] <= filter[ 11 ] * pixel[ 11 ];

    end
    else
    begin


        multiply[ 0 ] <= multiply[ 0 ];
        multiply[ 1 ] <= multiply[ 1 ];
        multiply[ 2 ] <= multiply[ 2 ];
        multiply[ 3 ] <= multiply[ 3 ];
        multiply[ 4 ] <= multiply[ 4 ];
        multiply[ 5 ] <= multiply[ 5 ];
        multiply[ 6 ] <= multiply[ 6 ];
        multiply[ 7 ] <= multiply[ 7 ];
        multiply[ 8 ] <= multiply[ 8 ];
        multiply[ 9 ] <= multiply[ 9 ];
        multiply[ 10 ] <= multiply[ 10 ];
        multiply[ 11 ] <= multiply[ 11 ];



    end

end

//pixel is ready for calculating
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        pixel[ 0 ] <= 0;
        pixel[ 1 ] <= 0;
        pixel[ 2 ] <= 0;
        pixel[ 3 ] <= 0;
        pixel[ 4 ] <= 0;
        pixel[ 5 ] <= 0;

    end
    else if ( state_n == H_STALL || state_c == H_STALL || state_n == V_STALL || state_c == V_STALL || nxt_col_at_ram_full )
    begin
        pixel[ 0 ] <= pixel[ 0 ];
        pixel[ 1 ] <= pixel[ 1 ];
        pixel[ 2 ] <= pixel[ 2 ];
        pixel[ 3 ] <= pixel[ 3 ];
        pixel[ 4 ] <= pixel[ 4 ];
        pixel[ 5 ] <= pixel[ 5 ];
    end
    else
    begin
        case ( right_index[ 0 ] & left_edge_detection )
            1:
            begin
                pixel[ 0 ] <= { 1'b0, pixel_buffer_head[ 0 ][ 7: 0 ] };
                pixel[ 1 ] <= { 1'b0, pixel_buffer_head[ 1 ][ 7: 0 ] };
                pixel[ 2 ] <= { 1'b0, pixel_buffer_head[ 2 ][ 7: 0 ] };
                pixel[ 3 ] <= { 1'b0, pixel_buffer_head[ 3 ][ 7: 0 ] };
                pixel[ 4 ] <= { 1'b0, pixel_buffer_head[ 4 ][ 7: 0 ] };
                pixel[ 5 ] <= { 1'b0, pixel_buffer_head[ 5 ][ 7: 0 ] };
            end

            2:
            begin
                pixel[ 0 ] <= { 1'b0, pixel_buffer_head[ 1 ][ 7: 0 ] };
                pixel[ 1 ] <= { 1'b0, pixel_buffer_head[ 2 ][ 7: 0 ] };
                pixel[ 2 ] <= { 1'b0, pixel_buffer_head[ 3 ][ 7: 0 ] };
                pixel[ 3 ] <= { 1'b0, pixel_buffer_head[ 4 ][ 7: 0 ] };
                pixel[ 4 ] <= { 1'b0, pixel_buffer_head[ 5 ][ 7: 0 ] };
                pixel[ 5 ] <= { 1'b0, pixel_buffer[ 0 ][ 7: 0 ] };
            end

            3:
            begin
                pixel[ 0 ] <= { 1'b0, pixel_buffer_head[ 2 ][ 7: 0 ] };
                pixel[ 1 ] <= { 1'b0, pixel_buffer_head[ 3 ][ 7: 0 ] };
                pixel[ 2 ] <= { 1'b0, pixel_buffer_head[ 4 ][ 7: 0 ] };
                pixel[ 3 ] <= { 1'b0, pixel_buffer_head[ 5 ][ 7: 0 ] };
                pixel[ 4 ] <= { 1'b0, pixel_buffer[ 0 ][ 7: 0 ] };
                pixel[ 5 ] <= { 1'b0, pixel_buffer[ 1 ][ 7: 0 ] };
            end

            4:
            begin
                pixel[ 0 ] <= { 1'b0, pixel_buffer_head[ 3 ][ 7: 0 ] };
                pixel[ 1 ] <= { 1'b0, pixel_buffer_head[ 4 ][ 7: 0 ] };
                pixel[ 2 ] <= { 1'b0, pixel_buffer_head[ 5 ][ 7: 0 ] };
                pixel[ 3 ] <= { 1'b0, pixel_buffer[ 0 ][ 7: 0 ] };
                pixel[ 4 ] <= { 1'b0, pixel_buffer[ 1 ][ 7: 0 ] };
                pixel[ 5 ] <= { 1'b0, pixel_buffer[ 2 ][ 7: 0 ] };
            end

            5:
            begin
                pixel[ 0 ] <= { 1'b0, pixel_buffer_head[ 4 ][ 7: 0 ] };
                pixel[ 1 ] <= { 1'b0, pixel_buffer_head[ 5 ][ 7: 0 ] };
                pixel[ 2 ] <= { 1'b0, pixel_buffer[ 0 ][ 7: 0 ] };
                pixel[ 3 ] <= { 1'b0, pixel_buffer[ 1 ][ 7: 0 ] };
                pixel[ 4 ] <= { 1'b0, pixel_buffer[ 2 ][ 7: 0 ] };
                pixel[ 5 ] <= { 1'b0, pixel_buffer[ 3 ][ 7: 0 ] };
            end
            6:
            begin
                pixel[ 0 ] <= { 1'b0, pixel_buffer_head[ 5 ][ 7: 0 ] };
                pixel[ 1 ] <= { 1'b0, pixel_buffer[ 0 ][ 7: 0 ] };
                pixel[ 2 ] <= { 1'b0, pixel_buffer[ 1 ][ 7: 0 ] };
                pixel[ 3 ] <= { 1'b0, pixel_buffer[ 2 ][ 7: 0 ] };
                pixel[ 4 ] <= { 1'b0, pixel_buffer[ 3 ][ 7: 0 ] };
                pixel[ 5 ] <= { 1'b0, pixel_buffer[ 4 ][ 7: 0 ] };
            end



            default:
            begin
                pixel[ 0 ] <= { 1'b0, pixel_buffer[ left_index[ 5 ] ][ 7: 0 ] };
                pixel[ 1 ] <= { 1'b0, pixel_buffer[ left_index[ 4 ] ][ 7: 0 ] };
                pixel[ 2 ] <= { 1'b0, pixel_buffer[ left_index[ 3 ] ][ 7: 0 ] };
                pixel[ 3 ] <= { 1'b0, pixel_buffer[ left_index[ 2 ] ][ 7: 0 ] };
                pixel[ 4 ] <= { 1'b0, pixel_buffer[ left_index[ 1 ] ][ 7: 0 ] };
                pixel[ 5 ] <= { 1'b0, pixel_buffer[ left_index[ 0 ] ][ 7: 0 ] };
            end

        endcase
    end


end


//pixel is ready for calculating
always@( posedge clk )
begin

    if ( ~reset_n )
    begin
        pixel[ 6 ] <= 0;
        pixel[ 7 ] <= 0;
        pixel[ 8 ] <= 0;
        pixel[ 9 ] <= 0;
        pixel[ 10 ] <= 0;
        pixel[ 11 ] <= 0;

    end
    else if ( state_n == H_STALL || state_c == H_STALL || state_n == V_STALL || state_c == V_STALL || nxt_col_at_ram_full )
    begin

        pixel[ 6 ] <= pixel[ 6 ];
        pixel[ 7 ] <= pixel[ 7 ];
        pixel[ 8 ] <= pixel[ 8 ];
        pixel[ 9 ] <= pixel[ 9 ];
        pixel[ 10 ] <= pixel[ 10 ];
        pixel[ 11 ] <= pixel[ 11 ];

    end

    else
    begin

        pixel[ 6 ] <= { 1'b0, pixel_buffer[ center_index ][ 7: 0 ] };

        case ( center_index & right_edge_detection )
            27:
            begin

                pixel[ 7 ] <= { 1'b0, pixel_buffer[ 28 ][ 7: 0 ] };
                pixel[ 8 ] <= { 1'b0, pixel_buffer[ 29 ][ 7: 0 ] };
                pixel[ 9 ] <= { 1'b0, pixel_buffer[ 30 ][ 7: 0 ] };
                pixel[ 10 ] <= { 1'b0, pixel_buffer[ 31 ][ 7: 0 ] };
                pixel[ 11 ] <= { 1'b0, pixel_buffer_tail[ 0 ][ 7: 0 ] };

            end
            28:
            begin

                pixel[ 7 ] <= { 1'b0, pixel_buffer[ 29 ][ 7: 0 ] };
                pixel[ 8 ] <= { 1'b0, pixel_buffer[ 30 ][ 7: 0 ] };
                pixel[ 9 ] <= { 1'b0, pixel_buffer[ 31 ][ 7: 0 ] };
                pixel[ 10 ] <= { 1'b0, pixel_buffer_tail[ 0 ][ 7: 0 ] };
                pixel[ 11 ] <= { 1'b0, pixel_buffer_tail[ 1 ][ 7: 0 ] };

            end
            29:
            begin

                pixel[ 7 ] <= { 1'b0, pixel_buffer[ 30 ][ 7: 0 ] };
                pixel[ 8 ] <= { 1'b0, pixel_buffer[ 31 ][ 7: 0 ] };
                pixel[ 9 ] <= { 1'b0, pixel_buffer_tail[ 0 ][ 7: 0 ] };
                pixel[ 10 ] <= { 1'b0, pixel_buffer_tail[ 1 ][ 7: 0 ] };
                pixel[ 11 ] <= { 1'b0, pixel_buffer_tail[ 2 ][ 7: 0 ] };

            end
            30:
            begin

                pixel[ 7 ] <= { 1'b0, pixel_buffer[ 31 ][ 7: 0 ] };
                pixel[ 8 ] <= { 1'b0, pixel_buffer_tail[ 0 ][ 7: 0 ] };
                pixel[ 9 ] <= { 1'b0, pixel_buffer_tail[ 1 ][ 7: 0 ] };
                pixel[ 10 ] <= { 1'b0, pixel_buffer_tail[ 2 ][ 7: 0 ] };
                pixel[ 11 ] <= { 1'b0, pixel_buffer_tail[ 3 ][ 7: 0 ] };

            end
            31:
            begin

                pixel[ 7 ] <= { 1'b0, pixel_buffer_tail[ 0 ][ 7: 0 ] };
                pixel[ 8 ] <= { 1'b0, pixel_buffer_tail[ 1 ][ 7: 0 ] };
                pixel[ 9 ] <= { 1'b0, pixel_buffer_tail[ 2 ][ 7: 0 ] };
                pixel[ 10 ] <= { 1'b0, pixel_buffer_tail[ 3 ][ 7: 0 ] };
                pixel[ 11 ] <= { 1'b0, pixel_buffer_tail[ 4 ][ 7: 0 ] };


            end




            default:
            begin

                pixel[ 7 ] <= { 1'b0, pixel_buffer[ right_index[ 0 ] ][ 7: 0 ] };
                pixel[ 8 ] <= { 1'b0, pixel_buffer[ right_index[ 1 ] ][ 7: 0 ] };
                pixel[ 9 ] <= { 1'b0, pixel_buffer[ right_index[ 2 ] ][ 7: 0 ] };
                pixel[ 10 ] <= { 1'b0, pixel_buffer[ right_index[ 3 ] ][ 7: 0 ] };
                pixel[ 11 ] <= { 1'b0, pixel_buffer[ right_index[ 4 ] ][ 7: 0 ] };


            end

        endcase

    end
end

//12 taps filter
always@( posedge clk )
begin

    if ( ~reset_n )
    begin


    end
    else if ( state_n == H_STALL || state_c == H_STALL || state_n == V_STALL || state_c == V_STALL || nxt_col_at_ram_full )
    begin

        filter[ 0 ] <= filter[ 0 ];
        filter[ 1 ] <= filter[ 1 ];
        filter[ 2 ] <= filter[ 2 ];
        filter[ 3 ] <= filter[ 3 ];
        filter[ 4 ] <= filter[ 4 ];
        filter[ 5 ] <= filter[ 5 ];
        filter[ 6 ] <= filter[ 6 ];
        filter[ 7 ] <= filter[ 7 ];
        filter[ 8 ] <= filter[ 8 ];
        filter[ 9 ] <= filter[ 9 ];
        filter[ 10 ] <= filter[ 10 ];
        filter[ 11 ] <= filter[ 11 ];




    end
    else
    begin
        filter[ 0 ] <= base_filter[ 0 ] + f0 ;
        filter[ 1 ] <= base_filter[ 1 ] + f1 ;
        filter[ 2 ] <= base_filter[ 2 ] + f2 ;
        filter[ 3 ] <= base_filter[ 3 ] + f3 ;
        filter[ 4 ] <= base_filter[ 4 ] + f4 ;
        filter[ 5 ] <= base_filter[ 5 ] + f5 ;
        filter[ 6 ] <= base_filter[ 6 ] + f6 ;
        filter[ 7 ] <= base_filter[ 7 ] + f7 ;
        filter[ 8 ] <= base_filter[ 8 ] + f8 ;
        filter[ 9 ] <= base_filter[ 9 ] + f9 ;
        filter[ 10 ] <= base_filter[ 10 ] + f10;
        filter[ 11 ] <= base_filter[ 11 ] + f11;
    end
end

//=================================================
// implementation of filter sets
// function:
// H.264/SVC sine-window sinc function filter from JSVM DownConvertStatic
//=================================================

//choose filter_set by down_level
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        filter_set <= 0;
    end
    else
    begin
        case ( down_level )
            31:
                filter_set <= 0;
            30, 29, 28, 27, 26:
                filter_set <= 1;
            25, 24, 23, 22, 21, 20:
                filter_set <= 2;
            19, 18, 17, 16:
                filter_set <= 3;
            15, 14, 13:
                filter_set <= 4;
            12:
                filter_set <= 5;
            11, 10, 9:
                filter_set <= 6;
            8:
                filter_set <= 7;
            default:
                filter_set <= 0;
        endcase
    end
end


// choose base_filter by filter_set
always@( posedge clk )
begin
    if ( ~reset_n )
    begin
        { base_filter[ 0 ], base_filter[ 1 ], base_filter[ 2 ], base_filter[ 3 ], base_filter[ 4 ], base_filter[ 5 ], base_filter[ 6 ], base_filter[ 7 ],
          base_filter[ 8 ], base_filter[ 9 ], base_filter[ 10 ], base_filter[ 11 ] }
        <= { 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd128, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0 };
    end
    else if ( state_n == RDY_FOR_H )
    begin
        case ( filter_set )
            0:
            begin

                { base_filter[ 0 ], base_filter[ 1 ], base_filter[ 2 ], base_filter[ 3 ], base_filter[ 4 ], base_filter[ 5 ], base_filter[ 6 ], base_filter[ 7 ],
                  base_filter[ 8 ], base_filter[ 9 ], base_filter[ 10 ], base_filter[ 11 ] }
                <= { 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd128, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0 };


            end
            1:
            begin
                { base_filter[ 0 ], base_filter[ 1 ], base_filter[ 2 ], base_filter[ 3 ], base_filter[ 4 ], base_filter[ 5 ], base_filter[ 6 ], base_filter[ 7 ],
                  base_filter[ 8 ], base_filter[ 9 ], base_filter[ 10 ], base_filter[ 11 ] }
                <= { 9'd0, 9'd2, 9'd0, -9'd14, 9'd33, 9'd86, 9'd33, -9'd14, 9'd0, 9'd2, 9'd0, 9'd0 };
            end
            2:
            begin
                { base_filter[ 0 ], base_filter[ 1 ], base_filter[ 2 ], base_filter[ 3 ], base_filter[ 4 ], base_filter[ 5 ], base_filter[ 6 ], base_filter[ 7 ],
                  base_filter[ 8 ], base_filter[ 9 ], base_filter[ 10 ], base_filter[ 11 ] }
                <= { 9'd2, 9'd0, -9'd10, 9'd0, 9'd40, 9'd64, 9'd40, 9'd0, -9'd10, 9'd0, 9'd2, 9'd0 };
            end
            3:
            begin
                { base_filter[ 0 ], base_filter[ 1 ], base_filter[ 2 ], base_filter[ 3 ], base_filter[ 4 ], base_filter[ 5 ], base_filter[ 6 ], base_filter[ 7 ],
                  base_filter[ 8 ], base_filter[ 9 ], base_filter[ 10 ], base_filter[ 11 ] }
                <= { 9'd0, -9'd4, -9'd7, 9'd11, 9'd38, 9'd52, 9'd38, 9'd11, -9'd7, -9'd4, 9'd0, 9'd0 };
            end
            4:
            begin
                { base_filter[ 0 ], base_filter[ 1 ], base_filter[ 2 ], base_filter[ 3 ], base_filter[ 4 ], base_filter[ 5 ], base_filter[ 6 ], base_filter[ 7 ],
                  base_filter[ 8 ], base_filter[ 9 ], base_filter[ 10 ], base_filter[ 11 ] }
                <= { -9'd2, -9'd7, 9'd0, 9'd17, 9'd35, 9'd43, 9'd35, 9'd17, 9'd0, -9'd7, -9'd5, 9'd2 };
            end
            5:
            begin
                { base_filter[ 0 ], base_filter[ 1 ], base_filter[ 2 ], base_filter[ 3 ], base_filter[ 4 ], base_filter[ 5 ], base_filter[ 6 ], base_filter[ 7 ],
                  base_filter[ 8 ], base_filter[ 9 ], base_filter[ 10 ], base_filter[ 11 ] }
                <= { -9'd6, -9'd3, 9'd5, 9'd19, 9'd31, 9'd36, 9'd31, 9'd19, 9'd5, -9'd3, -9'd6, 9'd0 };
            end
            6:
            begin
                { base_filter[ 0 ], base_filter[ 1 ], base_filter[ 2 ], base_filter[ 3 ], base_filter[ 4 ], base_filter[ 5 ], base_filter[ 6 ], base_filter[ 7 ],
                  base_filter[ 8 ], base_filter[ 9 ], base_filter[ 10 ], base_filter[ 11 ] }
                <= { -9'd9, 9'd0, 9'd9, 9'd20, 9'd28, 9'd32, 9'd28, 9'd20, 9'd9, 9'd0, -9'd9, 9'd0 };
            end
            7:
            begin
                { base_filter[ 0 ], base_filter[ 1 ], base_filter[ 2 ], base_filter[ 3 ], base_filter[ 4 ], base_filter[ 5 ], base_filter[ 6 ], base_filter[ 7 ],
                  base_filter[ 8 ], base_filter[ 9 ], base_filter[ 10 ], base_filter[ 11 ] }
                <= { -9'd8, 9'd7, 9'd13, 9'd18, 9'd22, 9'd24, 9'd22, 9'd18, 9'd13, 9'd7, 9'd2, -9'd10 };
            end
            default:
            begin
                { base_filter[ 0 ], base_filter[ 1 ], base_filter[ 2 ], base_filter[ 3 ], base_filter[ 4 ], base_filter[ 5 ], base_filter[ 6 ], base_filter[ 7 ],
                  base_filter[ 8 ], base_filter[ 9 ], base_filter[ 10 ], base_filter[ 11 ] }
                <= { 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd128, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0, 9'd0 };
            end
        endcase
    end

    else
    begin
        { base_filter[ 0 ], base_filter[ 1 ], base_filter[ 2 ], base_filter[ 3 ], base_filter[ 4 ], base_filter[ 5 ], base_filter[ 6 ], base_filter[ 7 ],
          base_filter[ 8 ], base_filter[ 9 ], base_filter[ 10 ], base_filter[ 11 ] }
        <= { base_filter[ 0 ], base_filter[ 1 ], base_filter[ 2 ], base_filter[ 3 ], base_filter[ 4 ], base_filter[ 5 ], base_filter[ 6 ], base_filter[ 7 ],
             base_filter[ 8 ], base_filter[ 9 ], base_filter[ 10 ], base_filter[ 11 ] };
    end
end




//choose filter coefficient by filter_number & base_filter

always@( * )
begin

    case ( filter_set )
        0:
        begin
            case ( filter_number_reg )
                default:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 0 ;
                    f4 <= 0 ;
                    f5 <= 0 ;
                    f6 <= 0 ;
                    f7 <= 0 ;
                    f8 <= 0 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                1:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 2 ;
                    f4 <= -6 ;
                    f5 <= -1 ;
                    f6 <= 7 ;
                    f7 <= -2 ;
                    f8 <= 0 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                2:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 3 ;
                    f4 <= -12 ;
                    f5 <= -3 ;
                    f6 <= 16 ;
                    f7 <= -5 ;
                    f8 <= 1 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                3:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 4 ;
                    f4 <= -16 ;
                    f5 <= -8 ;
                    f6 <= 26 ;
                    f7 <= -7 ;
                    f8 <= 1 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                4:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 5 ;
                    f4 <= -18 ;
                    f5 <= -14 ;
                    f6 <= 36 ;
                    f7 <= -10 ;
                    f8 <= 1 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                5:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 5 ;
                    f4 <= -20 ;
                    f5 <= -21 ;
                    f6 <= 46 ;
                    f7 <= -12 ;
                    f8 <= 2 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                6:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 5 ;
                    f4 <= -21 ;
                    f5 <= -29 ;
                    f6 <= 57 ;
                    f7 <= -15 ;
                    f8 <= 3 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                7:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 5 ;
                    f4 <= -20 ;
                    f5 <= -39 ;
                    f6 <= 68 ;
                    f7 <= -18 ;
                    f8 <= 4 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                8:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 4 ;
                    f4 <= -19 ;
                    f5 <= -49 ;
                    f6 <= 79 ;
                    f7 <= -19 ;
                    f8 <= 4 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                9:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 4 ;
                    f4 <= -18 ;
                    f5 <= -60 ;
                    f6 <= 89 ;
                    f7 <= -20 ;
                    f8 <= 5 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                10:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 3 ;
                    f4 <= -15 ;
                    f5 <= -71 ;
                    f6 <= 99 ;
                    f7 <= -21 ;
                    f8 <= 5 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                11:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 2 ;
                    f4 <= -12 ;
                    f5 <= -82 ;
                    f6 <= 107 ;
                    f7 <= -20 ;
                    f8 <= 5 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                12:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 1 ;
                    f4 <= -10 ;
                    f5 <= -92 ;
                    f6 <= 114 ;
                    f7 <= -18 ;
                    f8 <= 5 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                13:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 1 ;
                    f4 <= -7 ;
                    f5 <= -102 ;
                    f6 <= 120 ;
                    f7 <= -16 ;
                    f8 <= 4 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                14:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 1 ;
                    f4 <= -5 ;
                    f5 <= -112 ;
                    f6 <= 125 ;
                    f7 <= -12 ;
                    f8 <= 3 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                15:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 0 ;
                    f4 <= -2 ;
                    f5 <= -121 ;
                    f6 <= 127 ;
                    f7 <= -6 ;
                    f8 <= 2 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end



            endcase

        end
        1:
        begin
            case ( filter_number_reg )
                default:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 0 ;
                    f4 <= 0 ;
                    f5 <= 0 ;
                    f6 <= 0 ;
                    f7 <= 0 ;
                    f8 <= 0 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                1:
                begin
                    f0 <= 0 ;
                    f1 <= -1 ;
                    f2 <= 1 ;
                    f3 <= 0 ;
                    f4 <= -4 ;
                    f5 <= -1 ;
                    f6 <= 5 ;
                    f7 <= 1 ;
                    f8 <= -1 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                2:
                begin
                    f0 <= 0 ;
                    f1 <= -1 ;
                    f2 <= 2 ;
                    f3 <= 0 ;
                    f4 <= -9 ;
                    f5 <= -2 ;
                    f6 <= 10 ;
                    f7 <= 2 ;
                    f8 <= -2 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                3:
                begin
                    f0 <= 0 ;
                    f1 <= -1 ;
                    f2 <= 2 ;
                    f3 <= 1 ;
                    f4 <= -14 ;
                    f5 <= -3 ;
                    f6 <= 15 ;
                    f7 <= 3 ;
                    f8 <= -3 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                4:
                begin
                    f0 <= 0 ;
                    f1 <= -2 ;
                    f2 <= 3 ;
                    f3 <= 1 ;
                    f4 <= -18 ;
                    f5 <= -5 ;
                    f6 <= 20 ;
                    f7 <= 4 ;
                    f8 <= -4 ;
                    f9 <= 1 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                5:
                begin
                    f0 <= 0 ;
                    f1 <= -2 ;
                    f2 <= 3 ;
                    f3 <= 2 ;
                    f4 <= -22 ;
                    f5 <= -7 ;
                    f6 <= 24 ;
                    f7 <= 6 ;
                    f8 <= -5 ;
                    f9 <= 1 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                6:
                begin
                    f0 <= 0 ;
                    f1 <= -2 ;
                    f2 <= 3 ;
                    f3 <= 3 ;
                    f4 <= -26 ;
                    f5 <= -10 ;
                    f6 <= 29 ;
                    f7 <= 9 ;
                    f8 <= -7 ;
                    f9 <= 1 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                7:
                begin
                    f0 <= 0 ;
                    f1 <= -2 ;
                    f2 <= 3 ;
                    f3 <= 4 ;
                    f4 <= -30 ;
                    f5 <= -13 ;
                    f6 <= 32 ;
                    f7 <= 12 ;
                    f8 <= -7 ;
                    f9 <= 1 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                8:
                begin
                    f0 <= 0 ;
                    f1 <= -2 ;
                    f2 <= 3 ;
                    f3 <= 5 ;
                    f4 <= -33 ;
                    f5 <= -16 ;
                    f6 <= 37 ;
                    f7 <= 14 ;
                    f8 <= -9 ;
                    f9 <= 1 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                9:
                begin
                    f0 <= 0 ;
                    f1 <= -2 ;
                    f2 <= 3 ;
                    f3 <= 7 ;
                    f4 <= -35 ;
                    f5 <= -21 ;
                    f6 <= 40 ;
                    f7 <= 17 ;
                    f8 <= -10 ;
                    f9 <= 1 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                10:
                begin
                    f0 <= 0 ;
                    f1 <= -2 ;
                    f2 <= 3 ;
                    f3 <= 7 ;
                    f4 <= -38 ;
                    f5 <= -24 ;
                    f6 <= 43 ;
                    f7 <= 21 ;
                    f8 <= -11 ;
                    f9 <= 1 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                11:
                begin
                    f0 <= 0 ;
                    f1 <= -2 ;
                    f2 <= 3 ;
                    f3 <= 9 ;
                    f4 <= -41 ;
                    f5 <= -29 ;
                    f6 <= 46 ;
                    f7 <= 25 ;
                    f8 <= -12 ;
                    f9 <= 1 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                12:
                begin
                    f0 <= 0 ;
                    f1 <= -2 ;
                    f2 <= 3 ;
                    f3 <= 10 ;
                    f4 <= -43 ;
                    f5 <= -33 ;
                    f6 <= 48 ;
                    f7 <= 29 ;
                    f8 <= -13 ;
                    f9 <= 1 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                13:
                begin
                    f0 <= 0 ;
                    f1 <= -2 ;
                    f2 <= 2 ;
                    f3 <= 11 ;
                    f4 <= -44 ;
                    f5 <= -38 ;
                    f6 <= 50 ;
                    f7 <= 33 ;
                    f8 <= -13 ;
                    f9 <= 0 ;
                    f10 <= 1 ;
                    f11 <= 0 ;
                end
                14:
                begin
                    f0 <= 0 ;
                    f1 <= -2 ;
                    f2 <= 2 ;
                    f3 <= 12 ;
                    f4 <= -45 ;
                    f5 <= -43 ;
                    f6 <= 51 ;
                    f7 <= 38 ;
                    f8 <= -14 ;
                    f9 <= 0 ;
                    f10 <= 1 ;
                    f11 <= 0 ;
                end
                15:
                begin
                    f0 <= 0 ;
                    f1 <= -2 ;
                    f2 <= 2 ;
                    f3 <= 13 ;
                    f4 <= -46 ;
                    f5 <= -48 ;
                    f6 <= 52 ;
                    f7 <= 43 ;
                    f8 <= -14 ;
                    f9 <= -1 ;
                    f10 <= 1 ;
                    f11 <= 0 ;
                end



            endcase

        end
        2:
        begin
            case ( filter_number_reg )
                default:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 0 ;
                    f4 <= 0 ;
                    f5 <= 0 ;
                    f6 <= 0 ;
                    f7 <= 0 ;
                    f8 <= 0 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                1:
                begin
                    f0 <= 0 ;
                    f1 <= 1 ;
                    f2 <= 1 ;
                    f3 <= -2 ;
                    f4 <= -3 ;
                    f5 <= 0 ;
                    f6 <= 2 ;
                    f7 <= 2 ;
                    f8 <= 0 ;
                    f9 <= -1 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                2:
                begin
                    f0 <= 0 ;
                    f1 <= 1 ;
                    f2 <= 1 ;
                    f3 <= -3 ;
                    f4 <= -6 ;
                    f5 <= 0 ;
                    f6 <= 4 ;
                    f7 <= 4 ;
                    f8 <= 0 ;
                    f9 <= -1 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                3:
                begin
                    f0 <= 0 ;
                    f1 <= 1 ;
                    f2 <= 2 ;
                    f3 <= -5 ;
                    f4 <= -9 ;
                    f5 <= -1 ;
                    f6 <= 7 ;
                    f7 <= 6 ;
                    f8 <= 0 ;
                    f9 <= -2 ;
                    f10 <= 1 ;
                    f11 <= 0 ;
                end
                4:
                begin
                    f0 <= -1 ;
                    f1 <= 2 ;
                    f2 <= 2 ;
                    f3 <= -6 ;
                    f4 <= -11 ;
                    f5 <= -2 ;
                    f6 <= 9 ;
                    f7 <= 8 ;
                    f8 <= 0 ;
                    f9 <= -2 ;
                    f10 <= 1 ;
                    f11 <= 0 ;
                end
                5:
                begin
                    f0 <= -1 ;
                    f1 <= 2 ;
                    f2 <= 3 ;
                    f3 <= -7 ;
                    f4 <= -14 ;
                    f5 <= -3 ;
                    f6 <= 12 ;
                    f7 <= 10 ;
                    f8 <= 0 ;
                    f9 <= -3 ;
                    f10 <= 1 ;
                    f11 <= 0 ;
                end
                6:
                begin
                    f0 <= -1 ;
                    f1 <= 2 ;
                    f2 <= 4 ;
                    f3 <= -8 ;
                    f4 <= -17 ;
                    f5 <= -4 ;
                    f6 <= 14 ;
                    f7 <= 13 ;
                    f8 <= 0 ;
                    f9 <= -4 ;
                    f10 <= 1 ;
                    f11 <= 0 ;
                end
                7:
                begin
                    f0 <= -1 ;
                    f1 <= 2 ;
                    f2 <= 4 ;
                    f3 <= -9 ;
                    f4 <= -20 ;
                    f5 <= -5 ;
                    f6 <= 16 ;
                    f7 <= 15 ;
                    f8 <= 0 ;
                    f9 <= -4 ;
                    f10 <= 1 ;
                    f11 <= 1 ;
                end
                8:
                begin
                    f0 <= -1 ;
                    f1 <= 2 ;
                    f2 <= 5 ;
                    f3 <= -9 ;
                    f4 <= -22 ;
                    f5 <= -7 ;
                    f6 <= 17 ;
                    f7 <= 18 ;
                    f8 <= 1 ;
                    f9 <= -5 ;
                    f10 <= 0 ;
                    f11 <= 1 ;
                end
                9:
                begin
                    f0 <= -1 ;
                    f1 <= 3 ;
                    f2 <= 6 ;
                    f3 <= -10 ;
                    f4 <= -25 ;
                    f5 <= -8 ;
                    f6 <= 19 ;
                    f7 <= 20 ;
                    f8 <= 1 ;
                    f9 <= -6 ;
                    f10 <= 0 ;
                    f11 <= 1 ;
                end
                10:
                begin
                    f0 <= -2 ;
                    f1 <= 3 ;
                    f2 <= 6 ;
                    f3 <= -10 ;
                    f4 <= -27 ;
                    f5 <= -10 ;
                    f6 <= 20 ;
                    f7 <= 23 ;
                    f8 <= 2 ;
                    f9 <= -6 ;
                    f10 <= 0 ;
                    f11 <= 1 ;
                end
                11:
                begin
                    f0 <= -2 ;
                    f1 <= 3 ;
                    f2 <= 7 ;
                    f3 <= -10 ;
                    f4 <= -30 ;
                    f5 <= -12 ;
                    f6 <= 21 ;
                    f7 <= 26 ;
                    f8 <= 3 ;
                    f9 <= -7 ;
                    f10 <= 0 ;
                    f11 <= 1 ;
                end
                12:
                begin
                    f0 <= -2 ;
                    f1 <= 3 ;
                    f2 <= 8 ;
                    f3 <= -10 ;
                    f4 <= -32 ;
                    f5 <= -15 ;
                    f6 <= 22 ;
                    f7 <= 29 ;
                    f8 <= 4 ;
                    f9 <= -8 ;
                    f10 <= 0 ;
                    f11 <= 1 ;
                end
                13:
                begin
                    f0 <= -2 ;
                    f1 <= 3 ;
                    f2 <= 8 ;
                    f3 <= -10 ;
                    f4 <= -34 ;
                    f5 <= -17 ;
                    f6 <= 23 ;
                    f7 <= 31 ;
                    f8 <= 5 ;
                    f9 <= -8 ;
                    f10 <= -1 ;
                    f11 <= 2 ;
                end
                14:
                begin
                    f0 <= -2 ;
                    f1 <= 2 ;
                    f2 <= 9 ;
                    f3 <= -10 ;
                    f4 <= -36 ;
                    f5 <= -20 ;
                    f6 <= 24 ;
                    f7 <= 34 ;
                    f8 <= 7 ;
                    f9 <= -9 ;
                    f10 <= -1 ;
                    f11 <= 2 ;
                end
                15:
                begin
                    f0 <= -2 ;
                    f1 <= 2 ;
                    f2 <= 9 ;
                    f3 <= -10 ;
                    f4 <= -38 ;
                    f5 <= -22 ;
                    f6 <= 24 ;
                    f7 <= 37 ;
                    f8 <= 8 ;
                    f9 <= -9 ;
                    f10 <= -1 ;
                    f11 <= 2 ;
                end


            endcase
        end
        3:
        begin
            case ( filter_number_reg )
                default:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 0 ;
                    f4 <= 0 ;
                    f5 <= 0 ;
                    f6 <= 0 ;
                    f7 <= 0 ;
                    f8 <= 0 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                1:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= -2 ;
                    f4 <= -1 ;
                    f5 <= -1 ;
                    f6 <= 2 ;
                    f7 <= 2 ;
                    f8 <= 1 ;
                    f9 <= -3 ;
                    f10 <= 2 ;
                    f11 <= 0 ;
                end
                2:
                begin
                    f0 <= 0 ;
                    f1 <= 1 ;
                    f2 <= 0 ;
                    f3 <= -3 ;
                    f4 <= -3 ;
                    f5 <= -1 ;
                    f6 <= 3 ;
                    f7 <= 3 ;
                    f8 <= 2 ;
                    f9 <= -3 ;
                    f10 <= 1 ;
                    f11 <= 0 ;
                end
                3:
                begin
                    f0 <= 0 ;
                    f1 <= 2 ;
                    f2 <= -1 ;
                    f3 <= -5 ;
                    f4 <= -5 ;
                    f5 <= -1 ;
                    f6 <= 4 ;
                    f7 <= 5 ;
                    f8 <= 2 ;
                    f9 <= -3 ;
                    f10 <= 2 ;
                    f11 <= 0 ;
                end
                4:
                begin
                    f0 <= 0 ;
                    f1 <= 2 ;
                    f2 <= -1 ;
                    f3 <= -6 ;
                    f4 <= -6 ;
                    f5 <= -2 ;
                    f6 <= 5 ;
                    f7 <= 7 ;
                    f8 <= 3 ;
                    f9 <= -4 ;
                    f10 <= 2 ;
                    f11 <= 0 ;
                end
                5:
                begin
                    f0 <= 0 ;
                    f1 <= 2 ;
                    f2 <= -1 ;
                    f3 <= -7 ;
                    f4 <= -8 ;
                    f5 <= -2 ;
                    f6 <= 7 ;
                    f7 <= 8 ;
                    f8 <= 4 ;
                    f9 <= -4 ;
                    f10 <= 1 ;
                    f11 <= 0 ;
                end
                6:
                begin
                    f0 <= 0 ;
                    f1 <= 3 ;
                    f2 <= -1 ;
                    f3 <= -9 ;
                    f4 <= -10 ;
                    f5 <= -3 ;
                    f6 <= 8 ;
                    f7 <= 10 ;
                    f8 <= 5 ;
                    f9 <= -4 ;
                    f10 <= 1 ;
                    f11 <= 0 ;
                end
                7:
                begin
                    f0 <= 0 ;
                    f1 <= 3 ;
                    f2 <= -1 ;
                    f3 <= -10 ;
                    f4 <= -12 ;
                    f5 <= -3 ;
                    f6 <= 9 ;
                    f7 <= 12 ;
                    f8 <= 6 ;
                    f9 <= -4 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                8:
                begin
                    f0 <= 0 ;
                    f1 <= 4 ;
                    f2 <= -1 ;
                    f3 <= -11 ;
                    f4 <= -14 ;
                    f5 <= -4 ;
                    f6 <= 10 ;
                    f7 <= 13 ;
                    f8 <= 7 ;
                    f9 <= -4 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                9:
                begin
                    f0 <= 0 ;
                    f1 <= 4 ;
                    f2 <= -1 ;
                    f3 <= -12 ;
                    f4 <= -15 ;
                    f5 <= -5 ;
                    f6 <= 11 ;
                    f7 <= 15 ;
                    f8 <= 8 ;
                    f9 <= -4 ;
                    f10 <= -1 ;
                    f11 <= 0 ;
                end
                10:
                begin
                    f0 <= 0 ;
                    f1 <= 5 ;
                    f2 <= -1 ;
                    f3 <= -13 ;
                    f4 <= -17 ;
                    f5 <= -6 ;
                    f6 <= 11 ;
                    f7 <= 17 ;
                    f8 <= 9 ;
                    f9 <= -4 ;
                    f10 <= -1 ;
                    f11 <= 0 ;
                end
                11:
                begin
                    f0 <= 0 ;
                    f1 <= 5 ;
                    f2 <= -1 ;
                    f3 <= -14 ;
                    f4 <= -19 ;
                    f5 <= -7 ;
                    f6 <= 12 ;
                    f7 <= 19 ;
                    f8 <= 11 ;
                    f9 <= -4 ;
                    f10 <= -2 ;
                    f11 <= 0 ;
                end
                12:
                begin
                    f0 <= 0 ;
                    f1 <= 6 ;
                    f2 <= -1 ;
                    f3 <= -15 ;
                    f4 <= -20 ;
                    f5 <= -9 ;
                    f6 <= 12 ;
                    f7 <= 21 ;
                    f8 <= 12 ;
                    f9 <= -4 ;
                    f10 <= -2 ;
                    f11 <= 0 ;
                end
                13:
                begin
                    f0 <= 0 ;
                    f1 <= 6 ;
                    f2 <= 0 ;
                    f3 <= -16 ;
                    f4 <= -22 ;
                    f5 <= -10 ;
                    f6 <= 13 ;
                    f7 <= 22 ;
                    f8 <= 13 ;
                    f9 <= -4 ;
                    f10 <= -2 ;
                    f11 <= 0 ;
                end
                14:
                begin
                    f0 <= 0 ;
                    f1 <= 5 ;
                    f2 <= 0 ;
                    f3 <= -16 ;
                    f4 <= -24 ;
                    f5 <= -11 ;
                    f6 <= 13 ;
                    f7 <= 24 ;
                    f8 <= 15 ;
                    f9 <= -3 ;
                    f10 <= -3 ;
                    f11 <= 0 ;
                end
                15:
                begin
                    f0 <= 0 ;
                    f1 <= 6 ;
                    f2 <= 0 ;
                    f3 <= -17 ;
                    f4 <= -25 ;
                    f5 <= -12 ;
                    f6 <= 13 ;
                    f7 <= 26 ;
                    f8 <= 16 ;
                    f9 <= -3 ;
                    f10 <= -4 ;
                    f11 <= 0 ;
                end


            endcase
        end
        4:
        begin
            case ( filter_number_reg )
                default:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 0 ;
                    f4 <= 0 ;
                    f5 <= 0 ;
                    f6 <= 0 ;
                    f7 <= 0 ;
                    f8 <= 0 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                1:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= -1 ;
                    f3 <= -1 ;
                    f4 <= -1 ;
                    f5 <= 0 ;
                    f6 <= 1 ;
                    f7 <= 1 ;
                    f8 <= 1 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                2:
                begin
                    f0 <= 1 ;
                    f1 <= 0 ;
                    f2 <= -1 ;
                    f3 <= -3 ;
                    f4 <= -2 ;
                    f5 <= 0 ;
                    f6 <= 1 ;
                    f7 <= 2 ;
                    f8 <= 1 ;
                    f9 <= 1 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                3:
                begin
                    f0 <= 1 ;
                    f1 <= 0 ;
                    f2 <= -2 ;
                    f3 <= -4 ;
                    f4 <= -3 ;
                    f5 <= -1 ;
                    f6 <= 2 ;
                    f7 <= 3 ;
                    f8 <= 3 ;
                    f9 <= 1 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                4:
                begin
                    f0 <= 2 ;
                    f1 <= 0 ;
                    f2 <= -3 ;
                    f3 <= -5 ;
                    f4 <= -4 ;
                    f5 <= -1 ;
                    f6 <= 3 ;
                    f7 <= 4 ;
                    f8 <= 3 ;
                    f9 <= 1 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                5:
                begin
                    f0 <= 2 ;
                    f1 <= 0 ;
                    f2 <= -3 ;
                    f3 <= -6 ;
                    f4 <= -5 ;
                    f5 <= -1 ;
                    f6 <= 4 ;
                    f7 <= 6 ;
                    f8 <= 4 ;
                    f9 <= 1 ;
                    f10 <= -1 ;
                    f11 <= -1 ;
                end
                6:
                begin
                    f0 <= 2 ;
                    f1 <= 0 ;
                    f2 <= -4 ;
                    f3 <= -7 ;
                    f4 <= -6 ;
                    f5 <= -1 ;
                    f6 <= 5 ;
                    f7 <= 7 ;
                    f8 <= 5 ;
                    f9 <= 1 ;
                    f10 <= -1 ;
                    f11 <= -1 ;
                end
                7:
                begin
                    f0 <= 3 ;
                    f1 <= 0 ;
                    f2 <= -4 ;
                    f3 <= -8 ;
                    f4 <= -8 ;
                    f5 <= -2 ;
                    f6 <= 5 ;
                    f7 <= 8 ;
                    f8 <= 6 ;
                    f9 <= 2 ;
                    f10 <= -1 ;
                    f11 <= -1 ;
                end
                8:
                begin
                    f0 <= 3 ;
                    f1 <= 1 ;
                    f2 <= -5 ;
                    f3 <= -10 ;
                    f4 <= -9 ;
                    f5 <= -2 ;
                    f6 <= 6 ;
                    f7 <= 9 ;
                    f8 <= 7 ;
                    f9 <= 2 ;
                    f10 <= -1 ;
                    f11 <= -1 ;
                end
                9:
                begin
                    f0 <= 3 ;
                    f1 <= 1 ;
                    f2 <= -5 ;
                    f3 <= -11 ;
                    f4 <= -10 ;
                    f5 <= -3 ;
                    f6 <= 6 ;
                    f7 <= 10 ;
                    f8 <= 9 ;
                    f9 <= 3 ;
                    f10 <= -2 ;
                    f11 <= -1 ;
                end
                10:
                begin
                    f0 <= 3 ;
                    f1 <= 1 ;
                    f2 <= -6 ;
                    f3 <= -12 ;
                    f4 <= -11 ;
                    f5 <= -3 ;
                    f6 <= 7 ;
                    f7 <= 12 ;
                    f8 <= 10 ;
                    f9 <= 3 ;
                    f10 <= -2 ;
                    f11 <= -2 ;
                end
                11:
                begin
                    f0 <= 3 ;
                    f1 <= 1 ;
                    f2 <= -6 ;
                    f3 <= -13 ;
                    f4 <= -12 ;
                    f5 <= -4 ;
                    f6 <= 7 ;
                    f7 <= 13 ;
                    f8 <= 11 ;
                    f9 <= 4 ;
                    f10 <= -2 ;
                    f11 <= -2 ;
                end
                12:
                begin
                    f0 <= 4 ;
                    f1 <= 2 ;
                    f2 <= -6 ;
                    f3 <= -14 ;
                    f4 <= -14 ;
                    f5 <= -5 ;
                    f6 <= 7 ;
                    f7 <= 14 ;
                    f8 <= 12 ;
                    f9 <= 4 ;
                    f10 <= -2 ;
                    f11 <= -2 ;
                end
                13:
                begin
                    f0 <= 4 ;
                    f1 <= 2 ;
                    f2 <= -6 ;
                    f3 <= -14 ;
                    f4 <= -15 ;
                    f5 <= -6 ;
                    f6 <= 7 ;
                    f7 <= 15 ;
                    f8 <= 13 ;
                    f9 <= 5 ;
                    f10 <= -2 ;
                    f11 <= -3 ;
                end
                14:
                begin
                    f0 <= 4 ;
                    f1 <= 2 ;
                    f2 <= -6 ;
                    f3 <= -16 ;
                    f4 <= -16 ;
                    f5 <= -7 ;
                    f6 <= 8 ;
                    f7 <= 16 ;
                    f8 <= 14 ;
                    f9 <= 6 ;
                    f10 <= -2 ;
                    f11 <= -3 ;
                end
                15:
                begin
                    f0 <= 4 ;
                    f1 <= 2 ;
                    f2 <= -7 ;
                    f3 <= -16 ;
                    f4 <= -17 ;
                    f5 <= -7 ;
                    f6 <= 8 ;
                    f7 <= 17 ;
                    f8 <= 16 ;
                    f9 <= 6 ;
                    f10 <= -2 ;
                    f11 <= -4 ;
                end


            endcase
        end
        5:
        begin
            case ( filter_number_reg )
                default:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 0 ;
                    f4 <= 0 ;
                    f5 <= 0 ;
                    f6 <= 0 ;
                    f7 <= 0 ;
                    f8 <= 0 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                1:
                begin
                    f0 <= 0 ;
                    f1 <= -1 ;
                    f2 <= -1 ;
                    f3 <= -1 ;
                    f4 <= 0 ;
                    f5 <= 1 ;
                    f6 <= 1 ;
                    f7 <= 1 ;
                    f8 <= 1 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= -1 ;
                end
                2:
                begin
                    f0 <= 0 ;
                    f1 <= -1 ;
                    f2 <= -1 ;
                    f3 <= -2 ;
                    f4 <= -1 ;
                    f5 <= 0 ;
                    f6 <= 2 ;
                    f7 <= 2 ;
                    f8 <= 2 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= -1 ;
                end
                3:
                begin
                    f0 <= 1 ;
                    f1 <= -2 ;
                    f2 <= -2 ;
                    f3 <= -3 ;
                    f4 <= -1 ;
                    f5 <= 0 ;
                    f6 <= 2 ;
                    f7 <= 3 ;
                    f8 <= 3 ;
                    f9 <= 1 ;
                    f10 <= 0 ;
                    f11 <= -2 ;
                end
                4:
                begin
                    f0 <= 1 ;
                    f1 <= -2 ;
                    f2 <= -3 ;
                    f3 <= -4 ;
                    f4 <= -2 ;
                    f5 <= 0 ;
                    f6 <= 3 ;
                    f7 <= 4 ;
                    f8 <= 4 ;
                    f9 <= 1 ;
                    f10 <= 0 ;
                    f11 <= -2 ;
                end
                5:
                begin
                    f0 <= 1 ;
                    f1 <= -2 ;
                    f2 <= -3 ;
                    f3 <= -4 ;
                    f4 <= -3 ;
                    f5 <= 0 ;
                    f6 <= 3 ;
                    f7 <= 5 ;
                    f8 <= 5 ;
                    f9 <= 1 ;
                    f10 <= 0 ;
                    f11 <= -3 ;
                end
                6:
                begin
                    f0 <= 2 ;
                    f1 <= -2 ;
                    f2 <= -4 ;
                    f3 <= -5 ;
                    f4 <= -4 ;
                    f5 <= 0 ;
                    f6 <= 4 ;
                    f7 <= 5 ;
                    f8 <= 5 ;
                    f9 <= 2 ;
                    f10 <= 0 ;
                    f11 <= -3 ;
                end
                7:
                begin
                    f0 <= 2 ;
                    f1 <= -2 ;
                    f2 <= -5 ;
                    f3 <= -6 ;
                    f4 <= -5 ;
                    f5 <= -1 ;
                    f6 <= 4 ;
                    f7 <= 6 ;
                    f8 <= 6 ;
                    f9 <= 3 ;
                    f10 <= 1 ;
                    f11 <= -3 ;
                end
                8:
                begin
                    f0 <= 2 ;
                    f1 <= -3 ;
                    f2 <= -5 ;
                    f3 <= -7 ;
                    f4 <= -5 ;
                    f5 <= 0 ;
                    f6 <= 5 ;
                    f7 <= 7 ;
                    f8 <= 7 ;
                    f9 <= 3 ;
                    f10 <= 0 ;
                    f11 <= -4 ;
                end
                9:
                begin
                    f0 <= 3 ;
                    f1 <= -2 ;
                    f2 <= -5 ;
                    f3 <= -8 ;
                    f4 <= -6 ;
                    f5 <= -1 ;
                    f6 <= 4 ;
                    f7 <= 7 ;
                    f8 <= 8 ;
                    f9 <= 3 ;
                    f10 <= 1 ;
                    f11 <= -4 ;
                end
                10:
                begin
                    f0 <= 3 ;
                    f1 <= -3 ;
                    f2 <= -6 ;
                    f3 <= -9 ;
                    f4 <= -7 ;
                    f5 <= -1 ;
                    f6 <= 5 ;
                    f7 <= 8 ;
                    f8 <= 9 ;
                    f9 <= 4 ;
                    f10 <= 1 ;
                    f11 <= -4 ;
                end
                11:
                begin
                    f0 <= 3 ;
                    f1 <= -3 ;
                    f2 <= -7 ;
                    f3 <= -9 ;
                    f4 <= -7 ;
                    f5 <= -2 ;
                    f6 <= 5 ;
                    f7 <= 9 ;
                    f8 <= 10 ;
                    f9 <= 5 ;
                    f10 <= 1 ;
                    f11 <= -5 ;
                end
                12:
                begin
                    f0 <= 4 ;
                    f1 <= -3 ;
                    f2 <= -7 ;
                    f3 <= -10 ;
                    f4 <= -8 ;
                    f5 <= -2 ;
                    f6 <= 5 ;
                    f7 <= 10 ;
                    f8 <= 10 ;
                    f9 <= 5 ;
                    f10 <= 1 ;
                    f11 <= -5 ;
                end
                13:
                begin
                    f0 <= 4 ;
                    f1 <= -3 ;
                    f2 <= -7 ;
                    f3 <= -11 ;
                    f4 <= -9 ;
                    f5 <= -3 ;
                    f6 <= 5 ;
                    f7 <= 11 ;
                    f8 <= 11 ;
                    f9 <= 6 ;
                    f10 <= 1 ;
                    f11 <= -5 ;
                end
                14:
                begin
                    f0 <= 5 ;
                    f1 <= -3 ;
                    f2 <= -8 ;
                    f3 <= -12 ;
                    f4 <= -10 ;
                    f5 <= -3 ;
                    f6 <= 5 ;
                    f7 <= 11 ;
                    f8 <= 12 ;
                    f9 <= 7 ;
                    f10 <= 2 ;
                    f11 <= -6 ;
                end
                15:
                begin
                    f0 <= 5 ;
                    f1 <= -3 ;
                    f2 <= -8 ;
                    f3 <= -13 ;
                    f4 <= -11 ;
                    f5 <= -4 ;
                    f6 <= 6 ;
                    f7 <= 12 ;
                    f8 <= 13 ;
                    f9 <= 7 ;
                    f10 <= 2 ;
                    f11 <= -6 ;
                end


            endcase
        end
        6:
        begin
            case ( filter_number_reg )
                default:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 0 ;
                    f4 <= 0 ;
                    f5 <= 0 ;
                    f6 <= 0 ;
                    f7 <= 0 ;
                    f8 <= 0 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                1:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= -1 ;
                    f3 <= -1 ;
                    f4 <= 0 ;
                    f5 <= 0 ;
                    f6 <= 1 ;
                    f7 <= 0 ;
                    f8 <= 1 ;
                    f9 <= 0 ;
                    f10 <= 5 ;
                    f11 <= -5 ;
                end
                2:
                begin
                    f0 <= 0 ;
                    f1 <= -1 ;
                    f2 <= -1 ;
                    f3 <= -2 ;
                    f4 <= 0 ;
                    f5 <= 0 ;
                    f6 <= 1 ;
                    f7 <= 1 ;
                    f8 <= 1 ;
                    f9 <= 1 ;
                    f10 <= 5 ;
                    f11 <= -5 ;
                end
                3:
                begin
                    f0 <= 0 ;
                    f1 <= -1 ;
                    f2 <= -2 ;
                    f3 <= -2 ;
                    f4 <= -1 ;
                    f5 <= 0 ;
                    f6 <= 2 ;
                    f7 <= 2 ;
                    f8 <= 2 ;
                    f9 <= 1 ;
                    f10 <= 5 ;
                    f11 <= -6 ;
                end
                4:
                begin
                    f0 <= 1 ;
                    f1 <= -2 ;
                    f2 <= -3 ;
                    f3 <= -3 ;
                    f4 <= -1 ;
                    f5 <= 0 ;
                    f6 <= 2 ;
                    f7 <= 2 ;
                    f8 <= 3 ;
                    f9 <= 2 ;
                    f10 <= 5 ;
                    f11 <= -6 ;
                end
                5:
                begin
                    f0 <= 1 ;
                    f1 <= -2 ;
                    f2 <= -3 ;
                    f3 <= -4 ;
                    f4 <= -2 ;
                    f5 <= 0 ;
                    f6 <= 3 ;
                    f7 <= 3 ;
                    f8 <= 3 ;
                    f9 <= 2 ;
                    f10 <= 5 ;
                    f11 <= -6 ;
                end
                6:
                begin
                    f0 <= 1 ;
                    f1 <= -2 ;
                    f2 <= -4 ;
                    f3 <= -4 ;
                    f4 <= -2 ;
                    f5 <= -1 ;
                    f6 <= 3 ;
                    f7 <= 3 ;
                    f8 <= 4 ;
                    f9 <= 3 ;
                    f10 <= 6 ;
                    f11 <= -7 ;
                end
                7:
                begin
                    f0 <= 1 ;
                    f1 <= -3 ;
                    f2 <= -4 ;
                    f3 <= -5 ;
                    f4 <= -3 ;
                    f5 <= -1 ;
                    f6 <= 3 ;
                    f7 <= 4 ;
                    f8 <= 5 ;
                    f9 <= 4 ;
                    f10 <= 6 ;
                    f11 <= -7 ;
                end
                8:
                begin
                    f0 <= 2 ;
                    f1 <= -3 ;
                    f2 <= -5 ;
                    f3 <= -6 ;
                    f4 <= -3 ;
                    f5 <= -1 ;
                    f6 <= 3 ;
                    f7 <= 5 ;
                    f8 <= 5 ;
                    f9 <= 4 ;
                    f10 <= 6 ;
                    f11 <= -7 ;
                end
                9:
                begin
                    f0 <= 2 ;
                    f1 <= -3 ;
                    f2 <= -5 ;
                    f3 <= -6 ;
                    f4 <= -4 ;
                    f5 <= -1 ;
                    f6 <= 3 ;
                    f7 <= 5 ;
                    f8 <= 6 ;
                    f9 <= 5 ;
                    f10 <= 6 ;
                    f11 <= -8 ;
                end
                10:
                begin
                    f0 <= 2 ;
                    f1 <= -3 ;
                    f2 <= -6 ;
                    f3 <= -7 ;
                    f4 <= -5 ;
                    f5 <= -1 ;
                    f6 <= 3 ;
                    f7 <= 6 ;
                    f8 <= 7 ;
                    f9 <= 5 ;
                    f10 <= 7 ;
                    f11 <= -8 ;
                end
                11:
                begin
                    f0 <= 3 ;
                    f1 <= -4 ;
                    f2 <= -7 ;
                    f3 <= -8 ;
                    f4 <= -5 ;
                    f5 <= -1 ;
                    f6 <= 4 ;
                    f7 <= 6 ;
                    f8 <= 7 ;
                    f9 <= 6 ;
                    f10 <= 7 ;
                    f11 <= -8 ;
                end
                12:
                begin
                    f0 <= 3 ;
                    f1 <= -4 ;
                    f2 <= -7 ;
                    f3 <= -8 ;
                    f4 <= -6 ;
                    f5 <= -2 ;
                    f6 <= 4 ;
                    f7 <= 7 ;
                    f8 <= 8 ;
                    f9 <= 6 ;
                    f10 <= 7 ;
                    f11 <= -8 ;
                end
                13:
                begin
                    f0 <= 3 ;
                    f1 <= -4 ;
                    f2 <= -8 ;
                    f3 <= -9 ;
                    f4 <= -6 ;
                    f5 <= -2 ;
                    f6 <= 4 ;
                    f7 <= 7 ;
                    f8 <= 9 ;
                    f9 <= 7 ;
                    f10 <= 8 ;
                    f11 <= -9 ;
                end
                14:
                begin
                    f0 <= 4 ;
                    f1 <= -4 ;
                    f2 <= -8 ;
                    f3 <= -10 ;
                    f4 <= -7 ;
                    f5 <= -3 ;
                    f6 <= 4 ;
                    f7 <= 8 ;
                    f8 <= 9 ;
                    f9 <= 8 ;
                    f10 <= 8 ;
                    f11 <= -9 ;
                end
                15:
                begin
                    f0 <= 4 ;
                    f1 <= -4 ;
                    f2 <= -9 ;
                    f3 <= -10 ;
                    f4 <= -8 ;
                    f5 <= -3 ;
                    f6 <= 4 ;
                    f7 <= 8 ;
                    f8 <= 10 ;
                    f9 <= 8 ;
                    f10 <= 9 ;
                    f11 <= -9 ;
                end


            endcase
        end
        7:
        begin
            case ( filter_number_reg )
                default:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 0 ;
                    f4 <= 0 ;
                    f5 <= 0 ;
                    f6 <= 0 ;
                    f7 <= 0 ;
                    f8 <= 0 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                1:
                begin
                    f0 <= 0 ;
                    f1 <= 0 ;
                    f2 <= 0 ;
                    f3 <= 0 ;
                    f4 <= 0 ;
                    f5 <= -1 ;
                    f6 <= 0 ;
                    f7 <= 1 ;
                    f8 <= 0 ;
                    f9 <= 0 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                2:
                begin
                    f0 <= 0 ;
                    f1 <= -1 ;
                    f2 <= -1 ;
                    f3 <= 0 ;
                    f4 <= 0 ;
                    f5 <= -1 ;
                    f6 <= 0 ;
                    f7 <= 1 ;
                    f8 <= 1 ;
                    f9 <= 1 ;
                    f10 <= 0 ;
                    f11 <= 0 ;
                end
                3:
                begin
                    f0 <= -1 ;
                    f1 <= -1 ;
                    f2 <= -1 ;
                    f3 <= -1 ;
                    f4 <= 0 ;
                    f5 <= -1 ;
                    f6 <= 1 ;
                    f7 <= 1 ;
                    f8 <= 1 ;
                    f9 <= 1 ;
                    f10 <= 1 ;
                    f11 <= 0 ;
                end
                4:
                begin
                    f0 <= -1 ;
                    f1 <= -1 ;
                    f2 <= -1 ;
                    f3 <= -1 ;
                    f4 <= -1 ;
                    f5 <= -1 ;
                    f6 <= 1 ;
                    f7 <= 1 ;
                    f8 <= 1 ;
                    f9 <= 2 ;
                    f10 <= 1 ;
                    f11 <= 0 ;
                end
                5:
                begin
                    f0 <= -1 ;
                    f1 <= -2 ;
                    f2 <= -2 ;
                    f3 <= -1 ;
                    f4 <= -1 ;
                    f5 <= -1 ;
                    f6 <= 1 ;
                    f7 <= 2 ;
                    f8 <= 2 ;
                    f9 <= 2 ;
                    f10 <= 1 ;
                    f11 <= 0 ;
                end
                6:
                begin
                    f0 <= -1 ;
                    f1 <= -2 ;
                    f2 <= -2 ;
                    f3 <= -2 ;
                    f4 <= -1 ;
                    f5 <= -1 ;
                    f6 <= 1 ;
                    f7 <= 2 ;
                    f8 <= 2 ;
                    f9 <= 2 ;
                    f10 <= 2 ;
                    f11 <= 0 ;
                end
                7:
                begin
                    f0 <= -1 ;
                    f1 <= -2 ;
                    f2 <= -3 ;
                    f3 <= -2 ;
                    f4 <= -1 ;
                    f5 <= -1 ;
                    f6 <= 1 ;
                    f7 <= 2 ;
                    f8 <= 2 ;
                    f9 <= 3 ;
                    f10 <= 2 ;
                    f11 <= 0 ;
                end
                8:
                begin
                    f0 <= -2 ;
                    f1 <= -2 ;
                    f2 <= -3 ;
                    f3 <= -2 ;
                    f4 <= -2 ;
                    f5 <= -1 ;
                    f6 <= 1 ;
                    f7 <= 2 ;
                    f8 <= 3 ;
                    f9 <= 3 ;
                    f10 <= 3 ;
                    f11 <= 0 ;
                end
                9:
                begin
                    f0 <= -2 ;
                    f1 <= -3 ;
                    f2 <= -3 ;
                    f3 <= -3 ;
                    f4 <= -2 ;
                    f5 <= -1 ;
                    f6 <= 1 ;
                    f7 <= 3 ;
                    f8 <= 3 ;
                    f9 <= 3 ;
                    f10 <= 3 ;
                    f11 <= 1 ;
                end
                10:
                begin
                    f0 <= -2 ;
                    f1 <= -3 ;
                    f2 <= -4 ;
                    f3 <= -3 ;
                    f4 <= -2 ;
                    f5 <= -1 ;
                    f6 <= 1 ;
                    f7 <= 3 ;
                    f8 <= 3 ;
                    f9 <= 4 ;
                    f10 <= 3 ;
                    f11 <= 1 ;
                end
                11:
                begin
                    f0 <= -2 ;
                    f1 <= -4 ;
                    f2 <= -4 ;
                    f3 <= -3 ;
                    f4 <= -2 ;
                    f5 <= -1 ;
                    f6 <= 1 ;
                    f7 <= 3 ;
                    f8 <= 4 ;
                    f9 <= 4 ;
                    f10 <= 3 ;
                    f11 <= 1 ;
                end
                12:
                begin
                    f0 <= -2 ;
                    f1 <= -4 ;
                    f2 <= -4 ;
                    f3 <= -4 ;
                    f4 <= -3 ;
                    f5 <= -1 ;
                    f6 <= 1 ;
                    f7 <= 3 ;
                    f8 <= 4 ;
                    f9 <= 5 ;
                    f10 <= 4 ;
                    f11 <= 1 ;
                end
                13:
                begin
                    f0 <= -2 ;
                    f1 <= -4 ;
                    f2 <= -5 ;
                    f3 <= -4 ;
                    f4 <= -3 ;
                    f5 <= -1 ;
                    f6 <= 1 ;
                    f7 <= 4 ;
                    f8 <= 4 ;
                    f9 <= 5 ;
                    f10 <= 4 ;
                    f11 <= 1 ;
                end
                14:
                begin
                    f0 <= -2 ;
                    f1 <= -5 ;
                    f2 <= -5 ;
                    f3 <= -4 ;
                    f4 <= -3 ;
                    f5 <= -2 ;
                    f6 <= 1 ;
                    f7 <= 4 ;
                    f8 <= 5 ;
                    f9 <= 5 ;
                    f10 <= 4 ;
                    f11 <= 2 ;
                end
                15:
                begin
                    f0 <= -2 ;
                    f1 <= -5 ;
                    f2 <= -6 ;
                    f3 <= -5 ;
                    f4 <= -3 ;
                    f5 <= -2 ;
                    f6 <= 1 ;
                    f7 <= 4 ;
                    f8 <= 5 ;
                    f9 <= 6 ;
                    f10 <= 5 ;
                    f11 <= 2 ;
                end
            endcase
        end
        default:
        begin
            f0 <= 0 ;
            f1 <= 0 ;
            f2 <= 0 ;
            f3 <= 0 ;
            f4 <= 0 ;
            f5 <= 0 ;
            f6 <= 0 ;
            f7 <= 0 ;
            f8 <= 0 ;
            f9 <= 0 ;
            f10 <= 0;
            f11 <= 0;
        end
    endcase
end

endmodule
