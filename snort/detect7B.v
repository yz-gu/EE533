`timescale 1ns / 1ps

module detect7B(
    input ce, 
    input clk, 
    input [63:0] hwregA, 
    input match_en, 
    input mrst, 
    input [63:0] pipe1, 
    output match
);
    
    wire XLXN_6;
    wire XLXN_11;
    wire XLXN_14;
    wire [111:0] XLXN_16;
    reg  [71:0] pipe0;
    
    always @(posedge clk or posedge XLXN_6) begin
        if(XLXN_6)
            pipe0[71:0] <= 72'b0;
        else if (!ce)
            pipe0[71:0] <= pipe0[71:0];
        else
            pipe0[71:0] <= pipe1[71:0];
    end

    assign XLXN_16 = {pipe0[47:0], pipe1[63:0]};

    wordmatch XLXI_7 (.datacomp(hwregA[55:0]), 
                        .datain(XLXN_16[111:0]), 
                        .wildcard(hwregA[62:56]), 
                        .match(XLXN_11));

    assign match = XLXN_11 & match_en & ~mrst;
endmodule


module wordmatch(datacomp, 
                 datain, 
                 wildcard, 
                 match);

    input [55:0] datacomp;
    input [111:0] datain;
    input [6:0] wildcard;
    output match;
   
    wire XLXN_24;
    wire XLXN_25;
    wire XLXN_26;
    wire XLXN_27;
    wire XLXN_28;
    wire XLXN_29;
    wire XLXN_30;
    wire XLXN_31;
   
    comparator XLXI_1 (.a(datacomp[55:0]), 
                        .mask(wildcard[6:0]), 
                        .b(datain[55:0]), 
                        .match(XLXN_31));
    comparator XLXI_2 (.a(datacomp[55:0]), 
                        .mask(wildcard[6:0]), 
                        .b(datain[63:8]), 
                        .match(XLXN_30));
    comparator XLXI_3 (.a(datacomp[55:0]), 
                        .mask(wildcard[6:0]), 
                        .b(datain[71:16]), 
                        .match(XLXN_29));
    comparator XLXI_4 (.a(datacomp[55:0]), 
                        .mask(wildcard[6:0]), 
                        .b(datain[79:24]), 
                        .match(XLXN_28));
    comparator XLXI_5 (.a(datacomp[55:0]), 
                        .mask(wildcard[6:0]), 
                        .b(datain[87:32]), 
                        .match(XLXN_27));
    comparator XLXI_6 (.a(datacomp[55:0]), 
                        .mask(wildcard[6:0]), 
                        .b(datain[95:40]), 
                        .match(XLXN_26));
    comparator XLXI_7 (.a(datacomp[55:0]), 
                        .mask(wildcard[6:0]), 
                        .b(datain[103:48]), 
                        .match(XLXN_25));
    comparator XLXI_8 (.a(datacomp[55:0]), 
                        .mask(wildcard[6:0]), 
                        .b(datain[111:56]), 
                        .match(XLXN_24));

    assign match = XLXN_24 || XLXN_25 || XLXN_26 || XLXN_27 || XLXN_28 || XLXN_29 || XLXN_30 || XLXN_31;
endmodule


module comparator (
    input [55:0] a,
    input [55:0] b,
    input [6:0] mask,
    output match
);
    wire [6:0] comp8bit;
    assign comp8bit[0] = a[7:0]==b[7:0] || ~mask[0];
    assign comp8bit[1] = a[15:8]==b[15:8] || ~mask[1];
    assign comp8bit[2] = a[23:16]==b[23:16] || ~mask[2];
    assign comp8bit[3] = a[31:24]==b[31:24] || ~mask[3];
    assign comp8bit[4] = a[39:32]==b[39:32] || ~mask[4];
    assign comp8bit[5] = a[47:40]==b[47:40] || ~mask[5];
    assign comp8bit[6] = a[55:48]==b[55:48] || ~mask[6];
    assign match = comp8bit==7'b1111111;
endmodule