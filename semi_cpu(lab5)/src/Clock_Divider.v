`timescale 1ns / 1ps

module Clock_Divider(clk, slow_clk);
    input clk;
    output reg slow_clk;
    reg [19:0] counter;
    initial begin
        counter = 0;
        slow_clk = 0;
     end
    always @(posedge clk)
        if (counter == 12_500)
        begin
            slow_clk = ~slow_clk;
            counter = 0;
        end
        else
            counter = counter + 1;
endmodule
