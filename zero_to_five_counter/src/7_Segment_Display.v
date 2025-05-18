`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 17.04.2025 14:01:47
// Design Name: 
// Module Name: DisplayOut
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

module DisplayOut(
    input [3:0] in,
    output reg [0:6] out
    );
    always @(*) begin
        case (in)
            4'd0: out = 7'b1000000;
            4'd1: out = 7'b1111001;
            4'd2: out = 7'b0100100;
            4'd3: out = 7'b0110000;
            4'd4: out = 7'b0011001;
            4'd5: out = 7'b0010010;
            default: out = 7'b1111111; 
        endcase
    end
endmodule
