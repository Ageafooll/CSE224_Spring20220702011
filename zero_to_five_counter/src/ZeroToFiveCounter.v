`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/15/2025 12:03:43 PM
// Design Name: 
// Module Name: ZeroToFiveCounter
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


module ZeroToFiveCounter(
    input clk,
    input rst,
    output reg[3:0] count
    );
    always @(posedge clk) begin
        if (count == 4'd5 || rst == 1'b1) begin
            count <= 4'b0000; //Reset the counter
        end
        else begin
            count <= count + 1; //Increment the counter
        end
    end
endmodule
