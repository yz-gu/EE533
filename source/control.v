// opcode and instructions supported
`define RTYPE   7'b0110011 // ADD, SUB
`define ITYPE   7'b0010011 // ADDI(MOV), SLLI, ORI
`define LW      7'b0000011 // LW
`define SW      7'b0100011 // SW
`define BRANCH  7'b1100011 // BEQ (XNOR), BNE (XOR), BLE (SLT), BGT (SLT)
`define JAL     7'b1101111 // JAL
`define JALR    7'b1100111 // JALR
`define LUI     7'b0110111 // LUI
`define LR      7'b1111111 // load other data into registers

`timescale 1ns/1ps

module control (
    input [6:0] opcode,
    input [2:0] fn3,
    input       fn7_bit30, // we only need bit 30 in fn7
    // output              Branch, JAL, JALR, LUI, MemtoReg, RegWrite, MemWrite, LR,
    output              Branch, JAL, JALR, MemtoReg, RegWrite, MemWrite, LR,
    output              alusrc1,
    output reg [3:0]    aluctrl
);
    
    assign Branch = (opcode==`BRANCH);
    assign JAL  = opcode==`JAL;
    assign JALR = opcode==`JALR;
    // assign LUI  = opcode==`LUI;
    assign LR   = opcode==`LR;

    assign MemWrite = opcode==`SW;
    assign MemtoReg = opcode==`LW;
    assign RegWrite = (opcode==`RTYPE | opcode==`ITYPE | opcode==`LW | opcode==`JAL | opcode==`JALR | opcode==`LR);
    // assign alusrc0  = opcode==`LUI;
    assign alusrc1  = opcode==`ITYPE | opcode==`LW | opcode==`SW | opcode==`JAL | opcode==`JALR | opcode==`LR;

	// ALU Ctrl Gen
    always @(*) begin
        case (opcode)
            `RTYPE: case (fn3)
                3'b000: aluctrl = fn7_bit30? 4'b1000: 4'b0000; // add or sub
                3'b001: aluctrl = 4'b0001; // sll
                3'b101: aluctrl = 4'b0101; // srl
                default: aluctrl = 4'b0000;
            endcase
                 // SUB & ADD
            `ITYPE: case (fn3)
                3'b000: aluctrl = 4'b0000; // addi
                3'b001: aluctrl = 4'b0001; // slli
                3'b101: aluctrl = 4'b1011; // srli
                3'b110: aluctrl = 4'b0110; // ori
                default: aluctrl = 4'b0000;
            endcase
            `LW: aluctrl = 4'b0000;
            `LR: aluctrl = 4'b0000;
            `SW: aluctrl = 4'b0000;
            // `BRANCH: case (fn3)
            //     3'b000: aluctrl = 4'b0100; // BEQ, XOR
            //     3'b001: aluctrl = 4'b0100; // BNE, XOR
            //     3'b100: aluctrl = 4'b0010; // BLT, SLT
            //     3'b101: aluctrl = 4'b0010; // BGE, SLT
            //     default: aluctrl = 4'b0000;
            // endcase
            `JAL: aluctrl = 4'b0000;
            `JALR: aluctrl = 4'b0000;
            `LUI: aluctrl = 4'b0001;
            default: aluctrl = 4'b0000;
        endcase
	end
endmodule