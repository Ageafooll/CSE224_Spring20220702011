`timescale 1ns / 1ps

module Instruction_Decoder(instr, op, funct3, funct7, A1, A2, A3, imm);
    input [31:0] instr;
    output wire [6:0] op;
    output wire [2:0] funct3;
    output wire [6:0] funct7;
    output wire [4:0] A1, A2, A3;
    output wire [11:0] imm;
    assign funct7 = instr[31:25];
    assign imm = instr[31:20];
    //assign A1 = instr[24:20];
    //assign A2 = instr[19:15];
    assign A2 = instr[24:20];
    assign A1 = instr[19:15];
    assign funct3 = instr[14:12];
    assign A3 = instr[11:7];
    assign op = instr[6:0];
endmodule
