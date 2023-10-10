`timescale 1ns/1ps

module ALU (
    input [63:0] A,
    input [63:0] B,
    input [3:0] aluctrl,
    output reg [63:0] result,
    output equal,
    output less,
    output less_signed
);
    
    always @(*) begin
        case (aluctrl)
            4'b0000: result = A + B;
            4'b1000: result = A - B;
            4'b0110: result = A | B;
            4'b0111: result = A & B;
            4'b0100: result = A ^ B;
            4'b1100: result = A ~^ B;
            4'b0001: result = A << B;
            4'b1011: result = A >> B;
            4'b1101: result = A >>> B;
            // 4'b0010: result = {63'b0, $signed(A) < $signed(B)};
            // 4'b0011: result = {63'b0, A < B};
            default: result  = 64'b0;
        endcase
    end

    assign equal = A == B;
    assign less = A < B;
    assign less_signed = $signed(A) < $signed(B);

endmodule