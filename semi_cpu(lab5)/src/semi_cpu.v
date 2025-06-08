`timescale 1ns / 1ps

module Main(clk, next, rst, an, cx);
    input clk, next, rst;
    output [7:0] an, cx;
    
    reg [7:0] PC;
    wire [31:0] RD, instr;
    
    wire [6:0] op;
    wire [2:0] funct3;
    wire [6:0] funct7;
    wire [4:0] A1, A2, A3;
    wire [11:0] imm;
    wire [31:0] immExt;
    
    wire [31:0] RD1, RD2;
    wire [31:0] srcB;
    
    wire [2:0] ALUControl;
    wire ALUSrc, RegWrite;
    
    wire [31:0] ALUResult;
    
    wire slow_clk;
    
    initial begin
        PC = 0;
    end
    
    always @(posedge next or negedge rst)
        if (!rst)
            PC <= 32'd0;
        else if (next)
            PC <= PC + 1;
    
    Instruction_Memory inst_mem(PC, RD);
    
    assign instr = RD & {32{next}};
    
    Instruction_Decoder inst_dec(RD, op, funct3, funct7, A1, A2, A3, imm);
    
    assign immExt = {{20{imm[11]}}, imm};
    
    Control_Unit ctrl_unit(clk, op, funct3, funct7, ALUControl, ALUSrc, RegWrite);
    
    assign srcB = (immExt & {32{ALUSrc}}) | (RD2 & {32{~ALUSrc}});
    
    ALU alu(RD1, srcB, ALUResult, ALUControl);
    
    Register_File reg_file(clk, RegWrite, A1, A2, A3, ALUResult, RD1, RD2);
    
    Clock_Divider clk_div(clk, slow_clk);
    
    Seven_Segment_Converter seven_seg_conv(slow_clk, ALUResult, an, cx);
endmodule
