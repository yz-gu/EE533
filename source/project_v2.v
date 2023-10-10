// v2: ip block and ids

`timescale 1ns/1ps

// opcode and instructions supported
`define RTYPE   7'b0110011 // ADD, SUB
`define ITYPE   7'b0010011 // ADDI(MOV), SLLI, ORI
`define LW      7'b0000011 // LW
`define SW      7'b0100011 // SW
`define BRANCH  7'b1100011 // BEQ (XNOR), BNE (XOR), BLE (SLT), BGT (SLT)
`define JAL     7'b1101111 // JAL
`define JALR    7'b1100111 // JALR
// `define LUI     7'b0110111 // LUI
`define LR      7'b1111111 // load other data into registers

module rv64i #(
	parameter DATA_WIDTH = 64,
	parameter CTRL_WIDTH = DATA_WIDTH/8,
	parameter UDP_REG_SRC_WIDTH = 2
) (
	// --- Datastream part
	input [DATA_WIDTH-1:0]		in_data,
	input [CTRL_WIDTH-1:0]		in_ctrl,
	input 						in_wr,
	output						in_rdy,
	
	output [DATA_WIDTH-1:0]		out_data,
	output [CTRL_WIDTH-1:0]		out_ctrl,
	output						out_wr,
	input						out_rdy,
	// --- Register interface
	input                               reg_req_in,
	input                               reg_ack_in,
	input                               reg_rd_wr_L_in,
	input  	[`UDP_REG_ADDR_WIDTH-1:0]   reg_addr_in,
	input  	[`CPCI_NF2_DATA_WIDTH-1:0]  reg_data_in,
	input 	[UDP_REG_SRC_WIDTH-1:0]     reg_src_in,

	output                              reg_req_out,
	output                              reg_ack_out,
	output                              reg_rd_wr_L_out,
	output  [`UDP_REG_ADDR_WIDTH-1:0]   reg_addr_out,
	output  [`CPCI_NF2_DATA_WIDTH-1:0]  reg_data_out,
	output  [UDP_REG_SRC_WIDTH-1:0]     reg_src_out,
	// misc
	input	reset,
	input   clk
);

// Register Interface
    wire [31:0] ctrl_in, din_high, din_low, pattern_in, pattern_low, pattern_high;
    reg [31:0] dout_high, dout_low;

	reg cpu_mode;
	localparam IDLE = 2'b00;
	localparam HEADER = 2'b01;
	localparam PAYLOAD = 2'b10;
	localparam CPU = 2'b11;

    reg  new_reset;
	wire pipeline_en = ctrl_in[31] & cpu_mode;
    wire req_in = ctrl_in[30];
    wire rw_in = ctrl_in[29];
    wire rst_in = ctrl_in[28];
    wire [9:0] addr_in = ctrl_in[9:0];
    wire [63:0] din = {din_high, din_low};
	
	wire la_rst = ctrl_in[24];
	wire [7:0] la_raddr = ctrl_in[23:16];

	wire [31:0] PATTERN_IP = {16'h0a01, pattern_in[15:0]};
	wire [31:0] PATTERN_LENGTH = {16'h0, pattern_in[31:16]};

	wire [255:0] la_dout;
	reg [31:0] hw_data0, hw_data1, hw_data2, hw_data3, hw_data4, hw_data5, hw_data6, hw_data7;

	generic_regs
	#( 
		.UDP_REG_SRC_WIDTH	(UDP_REG_SRC_WIDTH),
		.TAG              (`RV64I_BLOCK_ADDR),        
		.REG_ADDR_WIDTH   (`RV64I_REG_ADDR_WIDTH), 
		.NUM_COUNTERS       (0),             
		.NUM_SOFTWARE_REGS  (6),           
		.NUM_HARDWARE_REGS  (10)             
	) module_regs (
		.reg_req_in       (reg_req_in),
		.reg_ack_in       (reg_ack_in),
		.reg_rd_wr_L_in   (reg_rd_wr_L_in),
		.reg_addr_in      (reg_addr_in),
		.reg_data_in      (reg_data_in),
		.reg_src_in       (reg_src_in),

		.reg_req_out      (reg_req_out),
		.reg_ack_out      (reg_ack_out),
		.reg_rd_wr_L_out  (reg_rd_wr_L_out),
		.reg_addr_out     (reg_addr_out),
		.reg_data_out     (reg_data_out),
		.reg_src_out      (reg_src_out),

		// --- counters interface
		.counter_updates  (),
		.counter_decrement(),

		// --- SW regs interface
		.software_regs    ({pattern_high, pattern_low, pattern_in, din_high, din_low, ctrl_in}),

		// --- HW regs interface
		.hardware_regs    ({hw_data7, hw_data6, hw_data5, hw_data4, hw_data3, hw_data2, hw_data1, hw_data0, dout_high, dout_low}),

		.clk              (clk),
		.reset            (reset)
		);

	// mem net declaration
	wire im_req, im_read, im_write;
	wire [8:0] im_addr;
	wire [31:0] im_din, im_dout;
	wire dm_req, dm_read, dm_write;
	wire dm_wea;
	wire [7:0] dm_addr;
	wire [63:0] dm_din, dm_dout;

	always @(*) begin
		{dout_high, dout_low} <= (req_in & ~rw_in)? (dm_req? dm_dout: {32'b0, im_dout}): 64'b0;
	end

// net declaration
	//four threads have their own private IMEM and DMEM
	//each thread has their own private IMEM of 128
  	//IF stage
	reg 	[63:0] 	PC0, PC1, PC2, PC3;
	reg     [1:0]   IF_thread;
	reg 	[63:0] 	IF_PC;
  	//ID stage
	wire    [1:0]   ID_thread;
  	wire 	[63:0] 	ID_PC;
	wire 	[31:0] 	ID_ins;
	wire			ID_MemtoReg, ID_RegW, ID_MemW, ID_ALUsrc0, ID_ALUsrc1, ID_Branch, ID_JAL, ID_JALR, ID_LUI, ID_LR;
	wire 	[3:0]	ID_ALUctrl;
	wire 			rf_we;
	wire	[4:0]	ID_r0, ID_r1, ID_rd;
	reg    	[63:0]	ID_imm;
	wire 	[63:0] 	ID_r0data, ID_r1data;
  	//EX stage
	wire    [1:0]   EX_thread;
  	wire 	[63:0] 	EX_PC;	
	wire			EX_MemtoReg, EX_RegW, EX_MemW, EX_ALUsrc0, EX_ALUsrc1, EX_Branch, EX_JAL, EX_JALR, EX_LUI, EX_LR;
	wire	[63:0]	EX_PC_branch;
	wire  	[63:0]	EX_imm, EX_r0data, EX_r1data;
	wire	[63:0]  ALU_in0, ALU_in1, EX_ALUresult;
	wire 	[4:0]	EX_rd;
	wire 	[3:0]	EX_ALUctrl;
	wire	[2:0]   EX_fn3;
	
	wire 	EX_equal, EX_less, EX_less_signed, EX_jump, ME_Branch_take;
	reg 	ME_equal, ME_less, ME_Branch;
	reg  	[2:0] ME_fn3;
	reg  	[63:0] ME_PC_branch;
  	//ME stage	
	wire    [1:0]   ME_thread;
  	wire 	[63:0] 	ME_PC;
	wire			ME_MemtoReg, ME_RegW, ME_MemW, ME_LR, ME_jump;
	wire 	[63:0]	ME_ALUresult, ME_r1data;
	wire 	[4:0]	ME_rd;
  	//WB stage
	wire    [1:0]   WB_thread;
  	wire 	[63:0] 	WB_PC;
	wire			WB_MemtoReg, WB_RegW, WB_LR, WB_jump;
	wire 	[4:0]	WB_rd;
	wire    [2:0]   WB_fn3;
	wire 	[63:0] 	WB_dmdout, WB_rddata, WB_ALUresult;

// tmp register
	reg 		tmp_in_use;
	reg [31:0] 	im_dout_tmp;
	reg [63:0] 	dm_dout_tmp;
	always @(posedge clk) begin
		if(reset | rst_in | new_reset | pipeline_en) begin
			tmp_in_use <= 0;
			im_dout_tmp <= 32'b0;
			dm_dout_tmp <= 64'b0;	
		end
		else begin
			tmp_in_use <= 1;
			im_dout_tmp <= tmp_in_use? im_dout_tmp: im_dout;
			dm_dout_tmp <= tmp_in_use? dm_dout_tmp: dm_dout;
		end
	end

// IF stage ----------------------------------------------------------------------------------
	always @(posedge clk) begin
		if(reset | rst_in | new_reset) begin
			IF_thread <= 2'b0;
			PC0 <= 64'h000;
			PC1 <= 64'h200;
			PC2 <= 64'h400;
			PC3 <= 64'h600;
		end
		else if(pipeline_en) begin
			PC0 <= (ME_jump & (ME_thread == 2'b00)) ? ME_ALUresult : (ME_Branch_take & (ME_thread == 2'b00)) ? ME_PC_branch : (IF_thread == 2'b00) ? (PC0 + 4) : PC0;
			PC1 <= (ME_jump & (ME_thread == 2'b01)) ? ME_ALUresult : (ME_Branch_take & (ME_thread == 2'b01)) ? ME_PC_branch : (IF_thread == 2'b01) ? (PC1 + 4) : PC1;
			PC2 <= (ME_jump & (ME_thread == 2'b10)) ? ME_ALUresult : (ME_Branch_take & (ME_thread == 2'b10)) ? ME_PC_branch : (IF_thread == 2'b10) ? (PC2 + 4) : PC2;
			PC3 <= (ME_jump & (ME_thread == 2'b11)) ? ME_ALUresult : (ME_Branch_take & (ME_thread == 2'b11)) ? ME_PC_branch : (IF_thread == 2'b11) ? (PC3 + 4) : PC3;
			IF_thread <= IF_thread + 1;
		end
	end

	always @(*) begin
		if(reset | rst_in | new_reset)
			IF_PC = 64'b0;
		else if(pipeline_en)
			case(IF_thread)
				2'b00: IF_PC = PC0;
				2'b01: IF_PC = PC1;
				2'b10: IF_PC = PC2;
				2'b11: IF_PC = PC3;
				default: IF_PC = 64'b0;
			endcase
	end

	// Instr mem request
	assign im_req = req_in & ~addr_in[9];
	assign im_read = im_req & ~rw_in;
	assign im_write = im_req & rw_in;
	assign im_addr = im_req? addr_in[8:0]: IF_PC[10:2];
	assign im_din = im_write? din[31:0]: 32'b0;

	assign ID_ins = tmp_in_use? im_dout_tmp: im_dout;

	IMEM I_Mem (
		.addr	(im_addr),
		.clk	(clk),
		.din	(im_din),
		.dout	(im_dout),
		.we		(im_write)
	);
	
	RegIFID Reg_IF_ID (
		.clk			(clk),
		.rst 			(reset | rst_in | new_reset),
		.en				(pipeline_en),
		.PC_in			(IF_PC),
		.PC_out			(ID_PC),
		.thread_in		(IF_thread),
		.thread_out		(ID_thread)
   	);

// ID stage -----------------------------------------------------------------------------------
	control Ctrl(
		.opcode		(ID_ins[6:0]),
		.fn3		(ID_ins[14:12]),
		.fn7_bit30	(ID_ins[30]),
		.Branch		(ID_Branch),
		.JAL		(ID_JAL),
		.JALR		(ID_JALR),
		.LR			(ID_LR),
		// .LUI 		(ID_LUI),
		.MemtoReg	(ID_MemtoReg),
		.RegWrite	(ID_RegW),
		.MemWrite	(ID_MemW),
		.alusrc0	(ID_ALUsrc0),
		.alusrc1	(ID_ALUsrc1),
		.aluctrl	(ID_ALUctrl)
	);

	assign ID_r0 = ID_ins[19:15];
	assign ID_r1 = ID_ins[24:20];
	assign ID_rd = ID_ins[11:7];

	RegFile Reg_file (
		.clk		(clk),
		.r0_addr	({ID_thread, ID_r0}),
		.r1_addr	({ID_thread, ID_r1}),
		.w_addr		({WB_thread, WB_rd}),
		.w_data		(WB_rddata),
		.we			(rf_we),
		.r0_data	(ID_r0data),
		.r1_data	(ID_r1data)
    );

	always @(*) begin
		case(ID_ins[6:0])
			`BRANCH: 	ID_imm = {{52{ID_ins[31]}}, ID_ins[31], ID_ins[7], ID_ins[30:25], ID_ins[11:8]};
			`ITYPE:		ID_imm = {{52{ID_ins[31]}}, ID_ins[31:20]};
			`LW:		ID_imm = {{52{ID_ins[31]}}, ID_ins[31:20]};
			`SW:		ID_imm = {{52{ID_ins[31]}}, ID_ins[31:25], ID_ins[11:7]};
			`JAL:		ID_imm = {{43{ID_ins[31]}}, ID_ins[31], ID_ins[19:12], ID_ins[20], ID_ins[30:21], 1'b0};
			`JALR:		ID_imm = {{52{ID_ins[31]}}, ID_ins[31:20]};
			// `LUI:		ID_imm = {{44{ID_ins[31]}}, ID_ins[31:12]};
			`LR:		ID_imm = {{52{ID_ins[31]}}, ID_ins[31:20]};
			default:	ID_imm = 64'b0;
		endcase
	end

	RegIDEX Reg_ID_EX (
		.clk		(clk),
		.en			(pipeline_en),
		.rst		(reset | rst_in | new_reset),
		.PC_in		(ID_PC),
		.PC_out		(EX_PC),
		.Branch_in	(ID_Branch),
		.JAL_in		(ID_JAL),
		.JALR_in	(ID_JALR),
		.LR_in		(ID_LR),
		// .LUI_in 	(ID_LUI),
		.fn3_in		(ID_ins[14:12]),
		.MemtoReg_in	(ID_MemtoReg),
		.RegWrite_in	(ID_RegW),
		.MemWrite_in	(ID_MemW),
		// .alusrc0_in		(ID_ALUsrc0),
		.alusrc1_in		(ID_ALUsrc1),
		.aluctrl_in		(ID_ALUctrl),
		.Branch_out	(EX_Branch),
		.JAL_out	(EX_JAL),
		.JALR_out	(EX_JALR),
		.LR_out		(EX_LR),
		// .LUI_out 	(EX_LUI),
		.fn3_out	(EX_fn3),
		.MemtoReg_out	(EX_MemtoReg),
		.RegWrite_out	(EX_RegW),
		.MemWrite_out	(EX_MemW),
		// .alusrc0_out	(EX_ALUsrc0),
		.alusrc1_out	(EX_ALUsrc1),
		.aluctrl_out	(EX_ALUctrl),
    	.r0data_in	(ID_r0data),
    	.r1data_in	(ID_r1data),
    	.WReg_in	(ID_rd),
		.r0data_out	(EX_r0data),
    	.r1data_out	(EX_r1data),
    	.WReg_out	(EX_rd),
		.imm_in		(ID_imm),
		.imm_out	(EX_imm),
		.thread_in		(ID_thread),
		.thread_out		(EX_thread)
    );

// EX stage -----------------------------------------------------------------------------------
	assign ALU_in0 = EX_JAL? EX_PC: EX_r0data;
	assign ALU_in1 = EX_ALUsrc1? EX_imm: EX_r1data;
	assign EX_PC_branch = EX_PC + EX_imm;

	ALU alu_64bit (
		.A(ALU_in0),
		.B(ALU_in1),
		.aluctrl(EX_ALUctrl),
		.result(EX_ALUresult),
		.equal(EX_equal),
		.less(EX_less),
		.less_signed(EX_less_signed)
	);

	assign EX_jump = EX_JAL | EX_JALR;
	always @(posedge clk) begin
		if(reset | rst_in | new_reset) begin
			ME_equal <= 1'b0;
			ME_less <= 1'b0;
			ME_Branch <= 1'b0;
			ME_fn3 <= 3'b0;
			ME_PC_branch <= 64'b0;
		end
		else if(pipeline_en) begin
			ME_equal <= EX_equal;
			ME_less <= EX_less;
			ME_Branch <= EX_Branch;
			ME_fn3 <= EX_fn3;
			ME_PC_branch <= EX_PC_branch;
		end
	end
		
	RegEXME Reg_EX_ME (
		.clk			(clk),
		.en				(pipeline_en),
		.rst			(reset | rst_in | new_reset),
		.PC_in			(EX_PC),
		.PC_out			(ME_PC),
	    .MemtoReg_in	(EX_MemtoReg),
		.MemtoReg_out	(ME_MemtoReg),
		.WRegEn_in		(EX_RegW),
    	.WMemEn_in		(EX_MemW),
		.WRegEn_out		(ME_RegW),
    	.WMemEn_out		(ME_MemW),
    	.r1data_in		(EX_r1data),
		.r1data_out		(ME_r1data),
    	.ALUresult_in	(EX_ALUresult),
		.WReg_in		(EX_rd),
    	.ALUresult_out	(ME_ALUresult),
    	.WReg_out		(ME_rd),
		.jump_in 		(EX_jump),
		.jump_out 		(ME_jump),
		.LR_in			(EX_LR),
		.LR_out			(ME_LR),
		.thread_in		(EX_thread),
		.thread_out		(ME_thread)
    );

// ME stage ------------------------------------------------------------------------------------
	assign ME_Branch_take = ME_Branch & ((ME_fn3==3'b000 & ME_equal) | (ME_fn3==3'b001 & ~ME_equal) | (ME_fn3==3'b100 & ME_less) | (ME_fn3==3'b101 & ~ME_less));

    reg process_done;
    always @(posedge clk) begin
        if (reset | rst_in | new_reset) begin
			process_done = 0;
        end
        else begin
            if(ME_thread==2'b00 & ME_PC[7:0]==8'b0)
				process_done = 0;
			if(ME_PC[9:0]==10'h218 || ME_PC[9:0]==10'h21c)
				process_done = 1;
        end
    end

	reg [1:0] state, state_next;
    reg stop_in_rdy, cpu_mode_next;
    reg begin_pkt, begin_pkt_next;
	reg	end_pkt, end_pkt_next;
    reg [7:0] readptr, writeptr, headptr, payload_start_ptr;
    reg [8:0] fifo_depth;
    reg valid;
	wire [7:0] mix_addra;

    always @(*) begin
        state_next = state;
		begin_pkt_next = begin_pkt;
		end_pkt_next = end_pkt;
		stop_in_rdy = 0;
		cpu_mode_next = 0;
		new_reset = 0;
        if (in_wr & (fifo_depth <= 9'h0fe) & (state_next != CPU)) begin
            case(state)
                IDLE: begin
					if (in_ctrl != 0) begin
						state_next = HEADER;
						begin_pkt_next = 1'b1;
						end_pkt_next = 0;
					end
                end
                HEADER: begin
					begin_pkt_next = 0;
					if (in_ctrl == 0) begin
						state_next = PAYLOAD;
						payload_start_ptr = mix_addra;
					end
                end
                PAYLOAD: begin
					if (in_ctrl != 0) begin
						end_pkt_next = 1'b1;
						stop_in_rdy = 1'b1;
						state_next = CPU;
						new_reset = 1'b1;
					end
                end
            endcase
        end
		else if (state == CPU) begin
			cpu_mode_next = 1'b1;
			stop_in_rdy = 1'b1;
			new_reset = 1'b0;
			end_pkt_next = 1'b0;
			if (process_done) begin
				state_next = IDLE;
				stop_in_rdy = 1'b0;
				cpu_mode_next = 1'b0;
			end
		end
    end
   
    reg in_wr_reg;
    reg [CTRL_WIDTH-1:0] in_ctrl_reg;
    reg [DATA_WIDTH-1:0] in_data_reg;
    always @(posedge clk) begin
        if(reset | rst_in) begin
            state <= IDLE;
            begin_pkt <= 1'b0;
            end_pkt <= 1'b0;
			in_ctrl_reg <= 8'b0;
			in_data_reg <= 64'b0;
			in_wr_reg <= 1'b0;
			cpu_mode <= 1'b0;
        end
        else begin
            state <= state_next;
            begin_pkt <= begin_pkt_next;
            end_pkt <= end_pkt_next;
			in_ctrl_reg <= in_ctrl;
			in_data_reg <= in_data;
            in_wr_reg <= in_wr;
			cpu_mode <= cpu_mode_next;
        end
    end

	wire match;
	detect7B matcher (
		.ce            (out_rdy), 
		.match_en      (out_rdy && state==PAYLOAD),     
		.clk           (clk),
		.pipe1         ({in_ctrl_reg, in_data_reg}),
		.hwregA        ({pattern_high, pattern_low}), 
		.match         (match),    
		.mrst          (reset || rst_in || state==IDLE)  
	);

	reg match_flag;
	always @(posedge clk) begin
		if (reset | rst_in | state==IDLE) begin
			match_flag <= 0;
		end
		else if (match) begin
			match_flag <= 1;
		end
	end

	// FIFO
    wire full = fifo_depth==9'd256;
    wire empty = fifo_depth==9'd0;
    wire fiforead = out_rdy & ~empty & (readptr!=writeptr) & (readptr!=headptr) & !cpu_mode;
    wire fifowrite = in_wr_reg & ~full;
    assign out_wr = valid;

    always @(posedge clk) begin
        if(reset | rst_in) begin
            readptr <= 8'b0;
            writeptr <= 8'b0;
            headptr <= 8'b0;
            valid <= 1'b0;
            fifo_depth <= 9'd0;
        end
        else if(!cpu_mode) begin
            if(begin_pkt|end_pkt)
                headptr <= writeptr;
            if(fiforead)
                readptr <= readptr + 1;
            if(fifowrite)
                writeptr <= writeptr + 1;
            if(fiforead & ~fifowrite)
                fifo_depth <= fifo_depth - 1;
            if(~fiforead & fifowrite)
                fifo_depth <= fifo_depth + 1;
		end
		valid <= fiforead;
    end

	assign in_rdy = (fifo_depth < 9'd254) & ~stop_in_rdy; // 1 cycle in advance and 1 latch slot

	// Data mem request
	assign dm_req = req_in & addr_in[9];
	assign dm_read = dm_req & ~rw_in;
	assign dm_write = dm_req & rw_in;
	assign dm_addr = dm_req? addr_in[7:0]: ME_ALUresult[10:3];
	assign dm_din = dm_write? din: ME_r1data;
	assign dm_wea = dm_write | (ME_MemW & pipeline_en);

	// memory input/output mux
    assign mix_addra = cpu_mode? dm_addr: writeptr;
    wire [7:0] mix_addrb = cpu_mode? dm_addr: readptr;
	wire mix_wea = cpu_mode? dm_wea: fifowrite;
    wire [63:0] mix_din = cpu_mode? dm_din: in_data_reg;
    wire [63:0] mix_dout;	

    DMEM fifo_sram (
        .clka   (clk),
        .clkb   (clk),
        .addra  (mix_addra),
        .addrb  (mix_addrb),
        .dina   (mix_din),
        .doutb  (mix_dout),
        .wea    (mix_wea)
    );

    assign out_data = mix_dout;

    Ctrl_mem ctrl_mem(
        .clka   (clk),
        .clkb   (clk),
        .addra  (writeptr),
        .addrb  (readptr),
        .dina   (in_ctrl_reg),
        .doutb  (out_ctrl),
        .wea    (fifowrite)
    );

	RegMEWB Reg_ME_WB(
		.en				(pipeline_en),
		.clk			(clk),
		.rst			(reset | rst_in | new_reset),
		.PC_in			(ME_PC),
		.PC_out			(WB_PC),
		.WRegEn_in		(ME_RegW),
		.WRegEn_out		(WB_RegW),
		.MemtoReg_in	(ME_MemtoReg),
		.MemtoReg_out	(WB_MemtoReg),
		.fn3_in			(ME_fn3),
		.fn3_out		(WB_fn3),
		.WReg_in		(ME_rd),
		.WReg_out		(WB_rd),
		.ALUresult_in	(ME_ALUresult),
		.ALUresult_out	(WB_ALUresult),
		.LR_in			(ME_LR),
		.LR_out			(WB_LR),
		.thread_in 		(ME_thread),
		.thread_out 	(WB_thread),
		.jump_in 		(ME_jump),
		.jump_out 		(WB_jump)
	);

	logic_analyzer la (
		.clk    (clk),
		.wdata	(mix_din),
		.wctrl  (in_ctrl_reg),
		.rdata  (mix_dout),
		.rctrl  (out_ctrl),
		.waddr  (mix_addra),
		.we    	(mix_wea),
		// .match_flag (match_flag),
		// .payload_start_ptr (payload_start_ptr),
		.raddr  (mix_addrb),
		.state  (state),
		.pc   	(ME_PC[11:0]),
		.la_rst (la_rst),
		// .tmp 	({1'b0, WB_LR, WB_MemtoReg, tmp_in_use}),
		.addr   (la_raddr),
		.data_out	(la_dout),
		.wb_rddata	(WB_rddata),
		.fiforead 	(fiforead)
   	);

	always @(*) begin
      	{hw_data7, hw_data6, hw_data5, hw_data4, hw_data3, hw_data2, hw_data1, hw_data0} = la_dout;
   	end

// WB stage ------------------------------------------------------------------------------------
	reg [63:0] DatatoReg;

	always @(*) begin
        case (WB_fn3)
            3'd0: DatatoReg = {53'b0, payload_start_ptr, 3'b0};
            3'd1: DatatoReg = {32'b0, PATTERN_IP};
			3'd2: DatatoReg = {32'b0, PATTERN_LENGTH};
			3'd3: DatatoReg = {63'b0, match_flag};
            default: DatatoReg = 64'b0;
        endcase
	end

	assign WB_dmdout = tmp_in_use? dm_dout_tmp: mix_dout;
	assign WB_rddata = WB_jump? (WB_PC+4): WB_LR? DatatoReg: WB_MemtoReg? WB_dmdout: WB_ALUresult;
	assign rf_we = WB_RegW & pipeline_en;

endmodule 
