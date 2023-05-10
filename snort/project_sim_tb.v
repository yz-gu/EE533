////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 1995-2003 Xilinx, Inc.
// All Right Reserved.
////////////////////////////////////////////////////////////////////////////////
//   ____  ____ 
//  /   /\/   / 
// /___/  \  /    Vendor: Xilinx 
// \   \   \/     Version : 10.1
//  \   \         Application : ISE
//  /   /         Filename : ids_tb.tfw
// /___/   /\     Timestamp : Mon Apr 03 16:15:46 2023
// \   \  /  \ 
//  \___\/\___\ 
//
//Command: 
//Design Name: ids_tb
//Device: Xilinx
//
`timescale 1ns/1ps

module project_tb;
    reg [63:0] in_data = 64'b0000000000000000000000000000000000000000000000000000000000000000;
    reg [7:0] in_ctrl = 8'b00000000;
    reg in_wr = 1'b0;
    wire in_rdy;

    wire [63:0] out_data;
    wire [7:0] out_ctrl;
    wire out_wr;
    reg out_rdy = 1'b1;

    reg reg_req_in = 1'b0;
    reg reg_ack_in = 1'b0;
    reg reg_rd_wr_L_in = 1'b0;
    reg [15:0] reg_addr_in = 16'b0000000000000000;
    reg [15:0] reg_data_in = 16'b0000000000000000;
    reg [1:0] reg_src_in = 2'b00;

    wire reg_req_out;
    wire reg_ack_out;
    wire reg_rd_wr_L_out;
    wire [15:0] reg_addr_out;
    wire [15:0] reg_data_out;
    wire [1:0] reg_src_out;

    reg reset = 1'b0;
    reg clk = 1'b0;

    parameter PERIOD = 200;
    parameter real DUTY_CYCLE = 0.5;
    parameter OFFSET = 100;

    initial    // Clock process for clk
    begin
        #OFFSET;
        forever
        begin
            clk = 1'b0;
            #(PERIOD-(PERIOD*DUTY_CYCLE)) clk = 1'b1;
            #(PERIOD*DUTY_CYCLE);
        end
    end

    rv64i UUT (
        .in_data(in_data),
        .out_data(out_data),
        .in_ctrl(in_ctrl),
        .out_ctrl(out_ctrl),
        .in_wr(in_wr),
        .out_wr(out_wr),
        .in_rdy(in_rdy),
        .out_rdy(out_rdy),
        .reg_req_in(reg_req_in),
        .reg_req_out(reg_req_out),
        .reg_ack_in(reg_ack_in),
        .reg_ack_out(reg_ack_out),
        .reg_rd_wr_L_in(reg_rd_wr_L_in),
        .reg_rd_wr_L_out(reg_rd_wr_L_out),
        .reg_addr_in(reg_addr_in),
        .reg_addr_out(reg_addr_out),
        .reg_data_in(reg_data_in),
        .reg_data_out(reg_data_out),
        .reg_src_in(reg_src_in),
        .reg_src_out(reg_src_out),
        .reset(reset),
        .clk(clk));

    initial begin
        // -------------  Current Time:  185ns
        #185;
        reset = 1'b1;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000000000;
        // -------------------------------------
        // -------------  Current Time:  385ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000000001;
        // -------------------------------------
        // -------------  Current Time:  585ns
        #200;
        reset = 1'b0;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000000010;
        // -------------------------------------
        // -------------  Current Time:  785ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000000011;
        // -------------------------------------
        // -------------  Current Time:  985ns
        #200;
        in_wr = 1'b1;
        out_rdy = 1'b1;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000000100; // header
        in_ctrl = 8'b00000001;
        // -------------------------------------
        // -------------  Current Time:  1185ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000000101; // pkt 1
        in_ctrl = 8'b00000000;
        // -------------------------------------
        // -------------  Current Time:  1385ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000000110;
        // -------------------------------------
        // -------------  Current Time:  1585ns
        #200;
//        in_data = 64'b0000000000000000000000000000000000000000000000010000000000000111;
        in_data = 64'h00000a0101030000;
        // -------------------------------------
        // -------------  Current Time:  1785ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000000111;
        in_data = 64'h000000000000005c;
        // -------------------------------------
        // -------------  Current Time:  1985ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000001001;
        // -------------------------------------
        // -------------  Current Time:  2185ns
        #200;
        // in_data = 64'b0000000000000000000000000000000000000000000000010000000000001010;
        in_data = 64'h00000000099000000;
        // -------------------------------------
        // -------------  Current Time:  2385ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000001011;
        // -------------------------------------
        // -------------  Current Time:  2585ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000001100;
        // -------------------------------------
        // -------------  Current Time:  2785ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000001101;
        // -------------------------------------
        // -------------  Current Time:  2985ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000001110;
        // -------------------------------------
        // -------------  Current Time:  3185ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000001111;
        // -------------------------------------
        // -------------  Current Time:  3385ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000010000;
        // -------------------------------------
        // -------------  Current Time:  3585ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000010001;
        // -------------------------------------
        // -------------  Current Time:  3785ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000010010;
        // -------------------------------------
        // -------------  Current Time:  3985ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000010011;
        // -------------------------------------
        // -------------  Current Time:  4185ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000010100;
        // -------------------------------------
        // -------------  Current Time:  4385ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000010101;
        // -------------------------------------
        // -------------  Current Time:  4585ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000010110;
        // -------------------------------------
        // -------------  Current Time:  4785ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000010111;
        // -------------------------------------
        // -------------  Current Time:  4985ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000011000;
        // -------------------------------------
        // -------------  Current Time:  5185ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000011001;
        // -------------------------------------
        // -------------  Current Time:  5385ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000011010;
        // -------------------------------------
        // -------------  Current Time:  5585ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000011011; // end pkt
        in_ctrl = 8'b00000001;
        // -------------------------------------
        // -------------  Current Time:  5785ns
        #200;
        in_wr = 1'b0;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000011100;
        in_ctrl = 8'b00000000;
        // -------------------------------------
        // -------------  Current Time:  5985ns
        #6000;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000011101;
        // -------------------------------------
        // -------------  Current Time:  6185ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000011110;
        // -------------------------------------
        // -------------  Current Time:  6385ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000011111;
        // -------------------------------------
        // -------------  Current Time:  6585ns
        #200;
        in_wr = 1'b1;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000100000; // header
        in_ctrl = 8'b00000001;
        // -------------------------------------
        // -------------  Current Time:  6785ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000100001;
        // -------------------------------------
        // -------------  Current Time:  6985ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000100010; // pkt 1
        in_ctrl = 8'b00000000;
        // -------------------------------------
        // -------------  Current Time:  7185ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000100011;
        // -------------------------------------
        // -------------  Current Time:  7585ns
        #200;
        // in_data = 64'b0000000000000000000000000000000000000000000000010000000000100101;
        in_data = 64'h00000a0102030000;
        // -------------------------------------
        // -------------  Current Time:  7785ns
        #200;
        // in_data = 64'b0000000000000000000000000000000000000000000000010000000000100110;
        in_data = 64'h000000000000001a;
        // -------------------------------------
        // -------------  Current Time:  7985ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000100100;
        // -------------------------------------
        // -------------  Current Time:  7985ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000100111;
        // -------------------------------------
        // -------------  Current Time:  8185ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000101000;
        // -------------------------------------
        // -------------  Current Time:  8385ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000101001;
        // -------------------------------------
        // -------------  Current Time:  8585ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000101010;
        // -------------------------------------
        // -------------  Current Time:  8785ns
        #200;
        // in_data = 64'b0000000000000000000000000000000000000000000000010000000000101011;
        in_data = 64'h00000099000000000;
        // -------------------------------------
        // -------------  Current Time:  8985ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000101100;
        // -------------------------------------
        // -------------  Current Time:  9185ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000101101;
        // -------------------------------------
        // -------------  Current Time:  9385ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000101110;
        // -------------------------------------
        // -------------  Current Time:  9585ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000101111;
        // -------------------------------------
        // -------------  Current Time:  9785ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000110000; // end pkt
        in_ctrl = 8'b00000001;
        // -------------------------------------
        // -------------  Current Time:  9985ns
        #200;
        in_wr = 1'b0;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000110001;
        in_ctrl = 8'b00000000;
        // -------------------------------------
        // -------------  Current Time:  10185ns
        #6000;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000110010;
        // -------------------------------------
        // -------------  Current Time:  10385ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000110011;
        // -------------------------------------
        // -------------  Current Time:  10585ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000110100;
        // -------------------------------------
        // -------------  Current Time:  10785ns
        #200;
        in_wr = 1'b1;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000110101; // header
        in_ctrl = 8'b00000001;
        // -------------------------------------
        // -------------  Current Time:  10985ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000110110;
        in_ctrl = 8'b00000000;
        // -------------------------------------
        // -------------  Current Time:  11185ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000110111;
        // -------------------------------------
        // -------------  Current Time:  11385ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000111000;
        // -------------------------------------
        // -------------  Current Time:  11585ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000111001;
        // -------------------------------------
        // -------------  Current Time:  11785ns
        #200;
        in_data = 64'h00000a0101030000;
//        in_data = 64'b0000000000000000000000000000000000000000000000010000000000111010; // keep unchanged
        // -------------------------------------
        // -------------  Current Time:  11985ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000000000000000000111;
        // -------------------------------------
        // -------------  Current Time:  12185ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000111100;
        // -------------------------------------
        // -------------  Current Time:  12385ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000111101;
        // -------------------------------------
        // -------------  Current Time:  12585ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000111110;
        // -------------------------------------
        // -------------  Current Time:  12785ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000000111111;
        // -------------------------------------
        // -------------  Current Time:  12985ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001000000; // end pkt
        in_ctrl = 8'b00000001;
        // -------------------------------------
        // -------------  Current Time:  13185ns
        #200;
        in_wr = 1'b0;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001000001;
        in_ctrl = 8'b00000000;
        // -------------------------------------
        // -------------  Current Time:  13385ns
        #6000;
        in_wr = 1'b1;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001000010;
        in_ctrl = 8'b00000001;
        // -------------------------------------
        // -------------  Current Time:  13585ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001000011; // header
        in_ctrl = 8'b00000000;
        // -------------------------------------
        // -------------  Current Time:  13785ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001000100; // payload 0
        // -------------------------------------
        // -------------  Current Time:  13985ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001000101;
        // -------------------------------------
        // -------------  Current Time:  14185ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001000110;
        // -------------------------------------
        // -------------  Current Time:  14385ns
        #200;
//        in_data = 64'h00000a0101030000;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001000110; // keep unchanged
        // -------------------------------------
        // -------------  Current Time:  14585ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001001000;
        // -------------------------------------
        // -------------  Current Time:  14785ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001001001;
        // -------------------------------------
        // -------------  Current Time:  14985ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001001010; // end pkt
        in_ctrl = 8'b00000001;
        // -------------------------------------
        // -------------  Current Time:  15185ns
        #200;
        in_wr = 1'b0;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001001011;
        in_ctrl = 8'b00000000;
        // -------------------------------------
        // -------------  Current Time:  15385ns
        #5000;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001001100;
        // -------------------------------------
        // -------------  Current Time:  15585ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001001101;
        // -------------------------------------
        // -------------  Current Time:  15785ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001001110;
        // -------------------------------------
        // -------------  Current Time:  15985ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001001111;
        // -------------------------------------
        // -------------  Current Time:  16185ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001010000;
        // -------------------------------------
        // -------------  Current Time:  16385ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001010001;
        // -------------------------------------
        // -------------  Current Time:  16585ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001010010;
        // -------------------------------------
        // -------------  Current Time:  16785ns
        #200;
        in_wr = 1'b1;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001010011;
        in_ctrl = 8'b00000001;
        // -------------------------------------
        // -------------  Current Time:  16985ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001010100; // header
        in_ctrl = 8'b00000000;
        // -------------------------------------
        // -------------  Current Time:  17185ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001010101; // payload 0
        // -------------------------------------
        // -------------  Current Time:  17385ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001010110;
        // -------------------------------------
        // -------------  Current Time:  17585ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001010111;
        // -------------------------------------
        // -------------  Current Time:  17785ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001011000; // keep unchanged
        // -------------------------------------
        // -------------  Current Time:  17985ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001011001;
        // -------------------------------------
        // -------------  Current Time:  18185ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001011010;
        // -------------------------------------
        // -------------  Current Time:  18385ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001011011;
        // -------------------------------------
        // -------------  Current Time:  18585ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001011100;
        // -------------------------------------
        // -------------  Current Time:  18785ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001011101;
        // -------------------------------------
        // -------------  Current Time:  18985ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001011110;
        // -------------------------------------
        // -------------  Current Time:  19185ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001011111;
        // -------------------------------------
        // -------------  Current Time:  19385ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001100000;
        // -------------------------------------
        // -------------  Current Time:  19585ns
        #200;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001100001; // end pkt
        in_ctrl = 8'b00000001;
        // -------------------------------------
        // -------------  Current Time:  19785ns
        #200;
        in_wr = 1'b0;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001100010;
        in_ctrl = 8'b00000000;
        // -------------------------------------
        // -------------  Current Time:  19985ns
        #5000;
        in_data = 64'b0000000000000000000000000000000000000000000000010000000001100011;
        // -------------------------------------
    end

endmodule
