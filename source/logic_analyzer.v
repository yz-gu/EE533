module logic_analyzer (
	// inputs
	input [63:0]	wdata,
	input [7:0]		wctrl,
	input [63:0]  	rdata,
	input [7:0]		rctrl,
	input [7:0]     waddr,
	input [7:0]     raddr,
	input           we,
	input			match_flag,
	input [1:0]     state,
	// input [7:0]		payload_start_ptr,
	input [11:0]	pc,
	// software and hardware registers
	input [7:0]    	addr,
	input [11:0]		tmp,
	input           clk,
	input	    	la_rst,
	// output [191:0]	data_out,
	output [255:0]	data_out,
	input [63:0] 	wb_rddata,
	input fiforead
);

	reg [7:0] count_r;
	reg	already_started, la_we;

	always @(posedge clk) begin
		if (la_rst) begin
			count_r <= 0;
			// already_started <= 0;
			la_we <= 0;
		end
		else if (!la_rst && count_r != 8'd255) begin
			la_we <= 1;
			// if (wctrl == 8'hff || already_started) begin
			if (state!=2'b0 || fiforead) begin
				// already_started <= 1;
				count_r <= count_r + 1;
			end
		end
		if (count_r == 8'd255) begin
			la_we <= 0;
		end
	end

	LA_MEM la_mem (
		.dina({tmp, 2'b0, state, 3'b0, we, pc, wb_rddata, rdata, wdata, rctrl, wctrl, raddr, waddr}),
		.addra(count_r),
		.wea(la_we),
		.addrb(addr),
		.clka(clk),
		.clkb(clk),
		.doutb(data_out)
	);

endmodule
