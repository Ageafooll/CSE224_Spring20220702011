`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/15/2025 12:04:37 PM
// Design Name: 
// Module Name: ZeroToFiveCounter_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module top(clk, rst, out);
    //Inputs
    input clk, rst;
    
    //Outputs
    wire [3:0] count;
    output wire [0:6] out;
    
    ZeroToFiveCounter uut(
        .clk(clk),
        .rst(rst),
        .count(count)
    );
    DisplayOut display_out(count, out);
endmodule
        

