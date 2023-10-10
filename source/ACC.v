`timescale 1ns / 1ps

module ACC (
    input [63:0] in_data,
    input clk,
    input rst,
    input ce,
    output [14:0] xor_out_combined,
    output [14:0] sl_xor_out_combined
    // output [14:0] sr_xor_out_combined
);
    // xor
    wire [7:0] xor_out;
    assign xor_out[0] = ^in_data[7:0];
    assign xor_out[1] = ^in_data[15:8];
    assign xor_out[2] = ^in_data[23:16];
    assign xor_out[3] = ^in_data[31:24];    
    assign xor_out[4] = ^in_data[39:32];
    assign xor_out[5] = ^in_data[47:40];
    assign xor_out[6] = ^in_data[55:48];
    assign xor_out[7] = ^in_data[63:56];

    reg [6:0] xor_out_last;
    always @(posedge clk or posedge rst) begin
        if(rst)
            xor_out_last <= 7'b0;
        else if (!ce)
            xor_out_last <= xor_out_last;
        else
            xor_out_last <= xor_out[6:0];
    end

    assign xor_out_combined = {xor_out_last, xor_out};

    // shift left and xor
    wire [7:0] sl_xor_out;
    assign sl_xor_out[0] = ^(in_data[7:0]<<4);
    assign sl_xor_out[1] = ^(in_data[15:8]<<4);
    assign sl_xor_out[2] = ^(in_data[23:16]<<4);
    assign sl_xor_out[3] = ^(in_data[31:24]<<4);    
    assign sl_xor_out[4] = ^(in_data[39:32]<<4);
    assign sl_xor_out[5] = ^(in_data[47:40]<<4);
    assign sl_xor_out[6] = ^(in_data[55:48]<<4);
    assign sl_xor_out[7] = ^(in_data[63:56]<<4);

    reg [6:0] sl_xor_out_last;
    always @(posedge clk or posedge rst) begin
        if(rst)
            sl_xor_out_last <= 7'b0;
        else if (!ce)
            sl_xor_out_last <= sl_xor_out_last;
        else
            sl_xor_out_last <= sl_xor_out[6:0];
    end

    assign sl_xor_out_combined = {sl_xor_out_last, sl_xor_out};

    // // shift right and xor
    // wire [7:0] sr_xor_out;
    // assign sr_xor_out[0] = ^(in_data[7:0]>>4);
    // assign sr_xor_out[1] = ^(in_data[15:8]>>4);
    // assign sr_xor_out[2] = ^(in_data[23:16]>>4);
    // assign sr_xor_out[3] = ^(in_data[31:24]>>4);    
    // assign sr_xor_out[4] = ^(in_data[39:32]>>4);
    // assign sr_xor_out[5] = ^(in_data[47:40]>>4);
    // assign sr_xor_out[6] = ^(in_data[55:48]>>4);
    // assign sr_xor_out[7] = ^(in_data[63:56]>>4);

    // reg [6:0] sr_xor_out_last;
    // always @(posedge clk or posedge rst) begin
    //     if(rst)
    //         sr_xor_out_last <= 7'b0;
    //     else if (!ce)
    //         sr_xor_out_last <= sr_xor_out_last;
    //     else
    //         sr_xor_out_last <= sr_xor_out[6:0];
    // end

    // assign sr_xor_out_combined = {sr_xor_out_last, sr_xor_out};
endmodule


module Bloom_Filter (
    input [63:0] in_data,
    input clk,
    input en,
    input ce,
    input rst,
    input [7:0] pattern,
    input [7:0] mask,
    output reg [7:0] match,
    output reg [7:0] sl_match
    // output reg [7:0] sr_match
);
    wire [14:0] xor_out, sl_xor_out;
    // wire [14:0] sr_xor_out;

    ACC dut(
        .in_data(in_data),
        .clk(clk),
        .rst(rst),
        .ce(ce),
        .xor_out_combined(xor_out),
        .sl_xor_out_combined(sl_xor_out)
        // .sr_xor_out_combined(sr_xor_out)
    );
    
    reg [255:0] fm;
    always @(posedge clk) begin
        if (rst)
            fm <= 256'b0;
        else if (pattern!=8'b0)
            fm[pattern] <= 1'b1;
    end

    always @(posedge clk) begin
        if (rst) begin
            match <= 8'b0;
            sl_match <= 8'b0;
            // sr_match <= 8'b0;
        end
        else if (en) begin
            match[0] <= fm[xor_out[7:0]&mask];
            match[1] <= fm[xor_out[8:1]&mask];
            match[2] <= fm[xor_out[9:2]&mask];
            match[3] <= fm[xor_out[10:3]&mask];
            match[4] <= fm[xor_out[11:4]&mask];
            match[5] <= fm[xor_out[12:5]&mask];
            match[6] <= fm[xor_out[13:6]&mask];
            match[7] <= fm[xor_out[14:7]&mask];
            sl_match[0] <= fm[sl_xor_out[7:0]&mask];
            sl_match[1] <= fm[sl_xor_out[8:1]&mask];
            sl_match[2] <= fm[sl_xor_out[9:2]&mask];
            sl_match[3] <= fm[sl_xor_out[10:3]&mask];
            sl_match[4] <= fm[sl_xor_out[11:4]&mask];
            sl_match[5] <= fm[sl_xor_out[12:5]&mask];
            sl_match[6] <= fm[sl_xor_out[13:6]&mask];
            sl_match[7] <= fm[sl_xor_out[14:7]&mask];
            // sr_match[0] <= fm[sr_xor_out[7:0]&mask];
            // sr_match[1] <= fm[sr_xor_out[8:1]&mask];
            // sr_match[2] <= fm[sr_xor_out[9:2]&mask];
            // sr_match[3] <= fm[sr_xor_out[10:3]&mask];
            // sr_match[4] <= fm[sr_xor_out[11:4]&mask];
            // sr_match[5] <= fm[sr_xor_out[12:5]&mask];
            // sr_match[6] <= fm[sr_xor_out[13:6]&mask];
            // sr_match[7] <= fm[sr_xor_out[14:7]&mask];
        end
        else begin
            match <= 8'b0;
            sl_match <= 8'b0;
            // sr_match <= 8'b0;
        end
    end
endmodule
