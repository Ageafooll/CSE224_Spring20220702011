`timescale 1ns / 1ps

module Control_Unit(clk, op, funct3, funct7, ALUControl, ALUSrc, RegWrite);
    input clk;
    input [6:0] op;
    input [2:0] funct3;
    input [6:0] funct7;
    output reg [2:0] ALUControl;
    output wire ALUSrc;
    output reg RegWrite;
    assign ALUSrc = ~op[5];
    initial
        RegWrite = 0;
    always @(posedge clk)
    begin
        case (funct3)
            3'b000: ALUControl = {~op[5], 3'b10};
            3'b010: ALUControl = {~op[5], 3'b11};
            3'b001: ALUControl = 3'b100;
            3'b101: ALUControl = 3'b101;
            default: ALUControl = 3'b000;
        endcase
        RegWrite = ~RegWrite;
    end
endmodule
