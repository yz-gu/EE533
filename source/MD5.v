`timescale 1ns / 1ps

module MD5(
    input clk,
    input rst_n,
    input hash_en,
    input rst_en,
    input w_en,
    input [2:0] w_addr,
    input [63:0] w_data,
    input r_addr,
    output wire [63:0] r_data,
    output wire hash_rdy
);
    wire F_bcd;
    wire G_bcd;
    wire H_bcd;
    wire I_bcd;
    wire [31:0] f_sum;
    reg [31:0] f;
    reg [3:0] g;

    reg [1:0] state;
    reg [31:0] A;
    reg [31:0] B;
    reg [31:0] C;
    reg [31:0] D;
    reg [5:0] cnt;
    reg [31:0] M [15:0];
    reg [31:0] hash [3:0];

    localparam IDLE = 1'b00;
    localparam COMPUTE = 1'b01;
    localparam ADD = 1'b10;
    localparam reg [31:0] k [0:63] = {
        32'hd76aa478, 32'he8c7b756, 32'h242070db, 32'hc1bdceee,
        32'hf57c0faf, 32'h4787c62a, 32'ha8304613, 32'hfd469501,
        32'h698098d8, 32'h8b44f7af, 32'hffff5bb1, 32'h895cd7be,
        32'h6b901122, 32'hfd987193, 32'ha679438e, 32'h49b40821,
        32'hf61e2562, 32'hc040b340, 32'h265e5a51, 32'he9b6c7aa,
        32'hd62f105d, 32'h02441453, 32'hd8a1e681, 32'he7d3fbc8,
        32'h21e1cde6, 32'hc33707d6, 32'hf4d50d87, 32'h455a14ed,
        32'ha9e3e905, 32'hfcefa3f8, 32'h676f02d9, 32'h8d2a4c8a,
        32'hfffa3942, 32'h8771f681, 32'h6d9d6122, 32'hfde5380c,
        32'ha4beea44, 32'h4bdecfa9, 32'hf6bb4b60, 32'hbebfbc70,
        32'h289b7ec6, 32'heaa127fa, 32'hd4ef3085, 32'h04881d05,
        32'hd9d4d039, 32'he6db99e5, 32'h1fa27cf8, 32'hc4ac5665,
        32'hf4292244, 32'h432aff97, 32'hab9423a7, 32'hfc93a039,
        32'h655b59c3, 32'h8f0ccc92, 32'hffeff47d, 32'h85845dd1,
        32'h6fa87e4f, 32'hfe2ce6e0, 32'ha3014314, 32'h4e0811a1,
        32'hf7537e82, 32'hbd3af235, 32'h2ad7d2bb, 32'heb86d391
    };
    localparam reg [4:0] s [0:63] = {
        7,12,17,22,7,12,17,22,7,12,17,22,7,12,17,22,
        5,9,14,20,5,9,14,20,5,9,14,20,5,9,14,20,
        4,11,16,23,4,11,16,23,4,11,16,23,4,11,16,23,
        6,10,15,21,6,10,15,21,6,10,15,21,6,10,15,21
    };

    always @ (posedge clk, negedge rst_n) begin
        if (~rst_n) begin
            state = IDLE;
            cnt = 6'b000000;
            M = '{16{32'd0}};
            hash <= {{32'h67452301}, {32'hefcdab89}, {32'h98badcfe}, {32'h10325476}};
            A = 0;
            B = 0;
            C = 0;
            D = 0;
        end
        else begin
            case(state)
                IDLE:
                begin
                    if (w_en) begin
                        {M[{w_addr, 1'b1}], M[{w_addr, 1'b0}]} <= {w_data[7:0], w_data[15:8], w_data[23:16], w_data[31:24], w_data[39:32], w_data[47:40], w_data[55:48], w_data[63:56]};
                        // {M[{w_addr, 1'b1}], M[{w_addr, 1'b0}]} <= w_data;
                    end
                    if (hash_en) begin
                        state <= COMPUTE;
                        A <= rst_en ? 32'h67452301 : hash[3];
                        B <= rst_en ? 32'hefcdab89 : hash[2];
                        C <= rst_en ? 32'h98badcfe : hash[1];
                        D <= rst_en ? 32'h10325476 : hash[0];
                    end
                end
                COMPUTE:
                begin
                    cnt <= cnt + 1;
                    if (cnt == 63) begin
                        state <= ADD;
                    end
                    A <= D;
                    D <= C;
                    C <= B;
                    B <= B + ((f_sum << s[cnt]) | (f_sum >> (32 - s[cnt]))); // left rotate
                end
                ADD:
                begin
                    state <= IDLE;
                    hash[3] <= A + hash[3];
                    hash[2] <= B + hash[2];
                    hash[1] <= C + hash[1];
                    hash[0] <= D + hash[0];
                end
            endcase
        end
    end

    assign F_bcd = ((B & C) | ((~B) & D));
    assign G_bcd = ((B & D) | (C & (~D)));
    assign H_bcd = (B ^ C ^ D);
    assign I_bcd = (C ^ (B | (~D)));

    always @ (*) begin
        if (cnt < 16) begin
            f = F_bcd;
            g = cnt;
        end
        else if (cnt < 32) begin
            f = G_bcd;
            g = 5 * cnt + 1;
        end
        else if (cnt < 48) begin
            f = H_bcd;
            g = 3 * cnt + 5;
        end
        else begin
            f = I_bcd;
            g = 7 * cnt;
        end
    end
    assign f_sum = (f + A + K[cnt] + M[g]);

    assign hash_rdy = (state == IDLE);
    assign r_data = r_addr ? {hash[3], hash[2]} : {hash[1], hash[0]};
endmodule