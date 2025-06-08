module topmodule(
    input [3:0] inputA, inputB,
    input [1:0] opcode,
    input clk,
    input WE3,
    input [4:0] A1, A2, A3,
    input [3:0] WD3,
    output reg [3:0] outputC,
    output [3:0] RD1, RD2
);

wire [3:0] alu_out;

ALU Module1 (
    .inputA(inputA),
    .inputB(inputB),
    .outputC(alu_out),
    .clk(clk),
    .opcode(opcode)
);

always @(posedge clk) begin
    outputC <= alu_out;
end

RegisterFile Module2 (
    .clk(clk),
    .WE3(WE3),
    .A1(A1),
    .A2(A2),
    .RD1(RD1),
    .RD2(RD2),
    .A3(A3),
    .WD3(WD3)
);

endmodule

module ALU(inputA, inputB, outputC, clk, opcode);

input[3:0] inputA, inputB;
output reg[3:0] outputC;
input clk;
input[1:0] opcode;

always@(posedge clk)
begin
case(opcode)
2'b00: outputC <= inputA + inputB;
2'b01: outputC <= inputA - inputB;
2'b10: outputC <= inputA << inputB;
2'b11: outputC <= inputA >> inputB;
endcase
end

endmodule

module RegisterFile(clk, WE3, A1, A2, RD1, RD2, A3, WD3);

output reg[3:0] RD1, RD2;
input[3:0] WD3;
input[4:0] A1,A2,A3;
input clk, WE3;

reg[3:0] RegisterFile[31:0];

always@(posedge clk)
begin

if(WE3)
begin
RegisterFile[A3] <= WD3; 
end

else
begin
RD1 <= RegisterFile[A1];
RD2 <= RegisterFile[A2];
end

end
endmodule
