`timescale 1ns / 1ps

module ALU(srcA, srcB, ALUResult, ALUControl);
    input [31:0] srcA, srcB;
    input [2:0] ALUControl;
    output reg [31:0] ALUResult;
    always @(*)
        casex (ALUControl)
            3'bx10: ALUResult = srcA + srcB;
            3'bx11: ALUResult = srcA - srcB;
            3'b100: ALUResult = srcA << srcB;
            3'b101: ALUResult = srcA >> srcB;
            default: ;
        endcase
endmodule
