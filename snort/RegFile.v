 `timescale 1ns / 1ps

module RegFile(
	input clk,
	// input rst,
    input [6:0] r0_addr,
    input [6:0] r1_addr,
    input [6:0] w_addr,
    input [63:0] w_data,
    input we,
    output [63:0] r0_data,
    output [63:0] r1_data
    );

	RFMEM rf_mem0 (.clka(clk), .dina(w_data), .addra(w_addr), .wea(we), .clkb(~clk), .addrb(r0_addr), .doutb(r0_data));
	RFMEM rf_mem1 (.clka(clk), .dina(w_data), .addra(w_addr), .wea(we), .clkb(~clk), .addrb(r1_addr), .doutb(r1_data));
endmodule
