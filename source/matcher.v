`timescale 1ns / 1ps

module detect7B(
    input ce, 
    input clk,
    input [63:0] hwregA,
    input [63:0] hwregB,
    input match_en,
    input mrst, 
    input [63:0] pipe1, 
    output match
);
    
    wire XLXN_11, XLXN_12;
    wire [119:0] XLXN_16;
    reg  [55:0] pipe0;
    
    always @(posedge clk or posedge mrst) begin
        if(mrst)
            pipe0 <= 56'b0;
        else if (!ce)
            pipe0 <= pipe0;
        else
            pipe0 <= pipe1[55:0];
    end

    assign XLXN_16 = {pipe0, pipe1};

    wordmatch XLXI_7 (.pattern(hwregA), 
                        .datain(XLXN_16), 
                        .match(XLXN_11));
    wordmatch XLXI_71 (.pattern(hwregB), 
                        .datain(XLXN_16), 
                        .match(XLXN_12));

    assign match = (XLXN_11|XLXN_12) & match_en & ~mrst;
endmodule


module wordmatch(input [63:0] pattern, 
                 input [119:0] datain,  
                 output match);
   
    wire XLXN_24;
    wire XLXN_25;
    wire XLXN_26;
    wire XLXN_27;
    wire XLXN_28;
    wire XLXN_29;
    wire XLXN_30;
    wire XLXN_31;
   
    comparator XLXI_1 (.a(pattern),  
                        .b(datain[63:0]), 
                        .match(XLXN_31));
    comparator XLXI_2 (.a(pattern),  
                        .b(datain[71:8]), 
                        .match(XLXN_30));
    comparator XLXI_3 (.a(pattern),  
                        .b(datain[79:16]), 
                        .match(XLXN_29));
    comparator XLXI_4 (.a(pattern),  
                        .b(datain[87:24]), 
                        .match(XLXN_28));
    comparator XLXI_5 (.a(pattern),  
                        .b(datain[95:32]), 
                        .match(XLXN_27));
    comparator XLXI_6 (.a(pattern),  
                        .b(datain[103:40]), 
                        .match(XLXN_26));
    comparator XLXI_7 (.a(pattern),  
                        .b(datain[111:48]), 
                        .match(XLXN_25));
    comparator XLXI_8 (.a(pattern),  
                        .b(datain[119:56]), 
                        .match(XLXN_24));

    assign match = XLXN_24 || XLXN_25 || XLXN_26 || XLXN_27 || XLXN_28 || XLXN_29 || XLXN_30 || XLXN_31;
endmodule


module comparator (
    input [63:0] a,
    input [63:0] b,
    output match
);
    assign match = a==b;
endmodule