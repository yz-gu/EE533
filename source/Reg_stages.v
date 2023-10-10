`timescale 1ns / 1ps

module RegIFID(
    input           clk,
    input           rst,
    input           en,
    input [1:0]     thread_in,
    input [63:0]    PC_in,
    output reg [1:0]    thread_out,
    output reg [63:0]   PC_out
);
	always @ (posedge clk) begin
        if(rst) begin
            PC_out <= 0;
            thread_out <=0;
        end
        else if(en) begin
            PC_out <= PC_in;
            thread_out <= thread_in;
        end
	end
endmodule


module RegIDEX(
    input           en,
    input           clk,
    input           rst,
    input [63:0]    PC_in,
    input           Branch_in,
    input           JAL_in,
    input           JALR_in,
    // input           LUI_in,
    input [2:0]     fn3_in,
    input           LR_in,

    input           MemtoReg_in,
    input           RegWrite_in,
    input           MemWrite_in,
    // input           alusrc0_in,
    input           alusrc1_in,
    input [3:0]     aluctrl_in,
    input [1:0]     thread_in,

    output reg      LR_out,
    output reg [1:0]    thread_out,

    output reg [63:0]   PC_out,  
    output reg          Branch_out,
    output reg          JAL_out,
    output reg          JALR_out,
    // output reg          LUI_out,
    output reg [2:0]    fn3_out,

    output reg          MemtoReg_out,
    output reg          RegWrite_out,
    output reg          MemWrite_out,
    // output reg          alusrc0_out,
    output reg          alusrc1_out,
    output reg [3:0]    aluctrl_out,

    input [63:0]    r0data_in,
    input [63:0]    r1data_in,
    input [4:0]     WReg_in,
    input [63:0]    imm_in,

    output reg [63:0]   r0data_out,
    output reg [63:0]   r1data_out,
    output reg [4:0]    WReg_out,
    output reg [63:0]   imm_out
);
	always @ (posedge clk) begin
        if(rst) begin
            r1data_out <= 0;
            r0data_out <= 0;
            WReg_out <= 0;
            imm_out <= 0;

            Branch_out <= 0;
            JAL_out <= 0;
            JALR_out <= 0;
            // LUI_out <= 0;
            LR_out <= 0;

            MemtoReg_out <= 0;
            RegWrite_out <= 0;
            MemWrite_out <= 0;
            // alusrc0_out <= 0;
            alusrc1_out <= 0;
            aluctrl_out <= 0;
            fn3_out <= 0;
            PC_out <= 0;
            thread_out <= 0;
        end
        else if(en) begin
            r1data_out <= r1data_in;
            r0data_out <= r0data_in;
            WReg_out <= WReg_in;
            imm_out <= imm_in;

            Branch_out <= Branch_in;
            JAL_out <= JAL_in;
            JALR_out <= JALR_in;
            // LUI_out <= LUI_in;
            LR_out <= LR_in;

            MemtoReg_out <= MemtoReg_in;
            RegWrite_out <= RegWrite_in;
            MemWrite_out <= MemWrite_in;
            // alusrc0_out <= alusrc0_in;
            alusrc1_out <= alusrc1_in;
            aluctrl_out <= aluctrl_in;
            fn3_out <= fn3_in;
            PC_out <= PC_in;
            thread_out <= thread_in;
        end
        else begin
            RegWrite_out <= 0;
            MemWrite_out <= 0;
        end
	end
endmodule


module RegEXME(
    input en,
    input clk,
    input rst,
    input [63:0] PC_in,
    input WRegEn_in,
    input WMemEn_in,
    input MemtoReg_in,
    input LR_in,

    input [63:0] r1data_in,
    input [63:0] ALUresult_in,
    input [4:0] WReg_in,
    input [1:0]     thread_in,
    output reg [1:0]    thread_out,

    output reg LR_out,
    output reg [63:0] PC_out,
    output reg MemtoReg_out,
    output reg WRegEn_out,
    output reg WMemEn_out,
    output reg [63:0] r1data_out,
    output reg [63:0] ALUresult_out,
    output reg [4:0] WReg_out,

    // input       JAL_in,
    // output reg  JAL_out,
    // input       JALR_in,
    // output reg  JALR_out

    input jump_in,
    output reg jump_out
);
	always @ (posedge clk) begin
        if(rst) begin
            MemtoReg_out <= 0;
            WRegEn_out <= 0;
            WMemEn_out <= 0;
            r1data_out <= 0;
            ALUresult_out <= 0;
            WReg_out <= 0;
            PC_out <= 0;
            // JAL_out <= 0;
            // JALR_out <= 0;
            thread_out <= 0;
            LR_out <= 0;
            jump_out <= 0;
        end
        else if(en) begin
            MemtoReg_out <= MemtoReg_in;
            WRegEn_out <= WRegEn_in;
            WMemEn_out <= WMemEn_in;
            r1data_out <= r1data_in;
            ALUresult_out <= ALUresult_in;
            WReg_out <= WReg_in;
            PC_out <= PC_in;
            // JAL_out <= JAL_in;
            // JALR_out <= JALR_in;
            thread_out <= thread_in;
            LR_out <= LR_in;
            jump_out <= jump_in;
        end
	end
endmodule


module RegMEWB(
    input en,
    input clk,
    input rst,
    input [63:0]    PC_in,
    input           WRegEn_in,
    input           MemtoReg_in,
    input [4:0]     WReg_in,
    input [63:0]    ALUresult_in,
    input [1:0]     thread_in,
    input           LR_in,
    input [2:0]     fn3_in,
    output reg [2:0] fn3_out,
    output reg [1:0]    thread_out,
    output reg [63:0]   PC_out,
    output reg [63:0]   ALUresult_out,
    output reg          MemtoReg_out,
    output reg          WRegEn_out,
    output reg [4:0]    WReg_out,
    output reg          LR_out,

    // input       JAL_in,
    // output reg  JAL_out,
    // input       JALR_in,
    // output reg  JALR_out

    input jump_in,
    output reg jump_out
);
	always @ (posedge clk) begin
        if(rst) begin
            WRegEn_out <= 0;
            WReg_out <= 0;
            MemtoReg_out <= 0;
            ALUresult_out <= 0;
            PC_out <= 0;
            // JAL_out <= 0;
            // JALR_out <= 0;
            thread_out <= 0;
            LR_out <= 0;    
            fn3_out <= 0;
            jump_out <= 0;    
        end
        else if(en) begin
            WRegEn_out <= WRegEn_in;
            WReg_out <= WReg_in;
            MemtoReg_out <= MemtoReg_in;
            ALUresult_out <= ALUresult_in;
            PC_out <= PC_in;
            // JAL_out <= JAL_in;
            // JALR_out <= JALR_in;
            thread_out <= thread_in;
            LR_out <= LR_in;
            fn3_out <= fn3_in;
            jump_out <= jump_in;
        end
	end
endmodule
