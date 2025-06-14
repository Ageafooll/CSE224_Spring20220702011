module top_module(clk, control, reset, seg, an);

  input clk, control, reset;

  output [6:0] seg;
  output [3:0] an;

  wire [31:0] result;

  reg [4:0] pc;
  wire [4:0] pc_new;

  wire [31:0] instruction;

  wire [4:0] rs1, rs2, rd;
  wire signed [11:0] ri; // CHANGED from [31:0] to [11:0]
  wire [2:0] alu_optype;
  wire [3:0] alu_opcode; // CHANGED from [2:0] to [3:0]

  wire [31:0] RD1, RD2;

  wire [31:0] alu_out;

  instruction_memory f1
  (
    .clk(clk),
    .WE3(1'b0),           // CHANGED from 0 to 1'b0 (match 1-bit input)
    .A1(pc),
    .A2(5'b00000),        // CHANGED from 0 to 5'b00000 (match 5-bit input)
    .A3(5'b00000),        // CHANGED from 0 to 5'b00000 (match 5-bit input)
    .WD3(32'b0),          // CHANGED from 0 to 32'b0 (explicit width match)
    .RD1(instruction),
    .RD2()
  );

  instruction_decoder f2
  (
    .instruction(instruction),
    .outRS1(rs1),
    .outRS2(rs2),
    .outRSD(rd),
    .outImm(ri),
    .outType(alu_optype),
    .outOpcode(alu_opcode)
  );

  register_file f3
  (
    .clk(clk),
    .WE3(control),       // OK (1-bit input)
    .A1(rs1),
    .A2(rs2),
    .A3(rd),
    .WD3(alu_out),
    .RD1(RD1),
    .RD2(RD2)
  );

  ALU f4
  (
    .clk(clk),
    .opType(alu_optype),
    .opcode(alu_opcode[2:0]), // CHANGED: Truncate 4-bit to 3-bit input
    .inputA(RD1),
    .inputB(RD2),
    .inputImm(ri),           // OK now, since both are [11:0]
    .PCcurrent(pc),
    .PCnew(pc_new),
    .out(alu_out)
  );

  SevenSegmentDisplay f5 
  (
    .clk(clk),
    .reset(reset),
    .value(result[15:0]),
    .an(an),
    .seg(seg)
  );

  assign result = alu_out;

  always @(posedge clk or posedge reset) 
    begin
      if (reset)
        pc <= 0;
      else if (control)
        pc <= pc_new;
    end

endmodule



module register_file(clk, WE3, A1, A2, A3, WD3, RD1, RD2);
  
  input clk, WE3;
  input [4:0] A1, A2, A3;
  input [31:0] WD3;
  output [31:0] RD1, RD2;
  
  reg [31:0] reg_file [31:0]; 
  
  initial
    begin
      
      reg_file[0] = 0;
      
    end
  
  assign RD1 = reg_file[A1];
  assign RD2 = reg_file[A2];
  
  always @(posedge clk) 
    begin
      if (WE3)
        reg_file[A3] <= WD3;
    end
  
endmodule

module instruction_memory(clk, WE3, A1, A2, A3, WD3, RD1, RD2);
  
  input clk, WE3;
  input [4:0] A1, A2, A3;
  input [31:0] WD3;
  output reg [31:0] RD1, RD2;
  
  reg [31:0] reg_file [31:0]; 
  
  initial
    begin
      //32-bit my instruction format
      
      reg_file[0]  = 32'b01011001010000000000000010100000; // ADDI x10, x0, 10
  	  reg_file[1]  = 32'b01011001111000000000000011110000; // ADDI x15, x0, 15
      reg_file[2]  = 32'b00101011001010100111100000000000; // ADD x25, x10, x15
      reg_file[3]  = 32'b01011110100110010000000001010000; // SUBI x20, x25, 5
      reg_file[4]  = 32'b01011010101000000000000000100000; // ADDI x21, x0, 2
      reg_file[5]  = 32'b01101000000000000000000001000000; //  J 12 (PC = 12)
      reg_file[6]  = 32'b00110111110001111010100000000000; // SHIFTL x30, x7, x21
      reg_file[7]  = 32'b00100000000000000000000000000000; // NOOp
      reg_file[8]  = 32'b00100000000000000000000000000000; // NOOp
      reg_file[9]  = 32'b00100000000000000000000000000000; // NOOp
      reg_file[10] = 32'b00100000000000000000000000000000; // NOOp
      reg_file[11] = 32'b01011000100000000000000001000000; // ADDI x4, x0, 4
      reg_file[12] = 32'b00101000101000000000000000000000; // ADD x5, x0, x0
      reg_file[13] = 32'b01100100000001000000000001110000; // BEQ x4, x5, offset=7 signed
      reg_file[14] = 32'b01011000110000000000000000010000; // ADDI x6, x0, 1
      reg_file[15] = 32'b01011000111000000000000000010000; // ADDI x7, x0, 1
      reg_file[16] = 32'b00101001000001100011100000000000; // ADD x8, x6, x7
      reg_file[17] = 32'b00101000110001110000000000000000; // ADD x6, x7, x0
      reg_file[18] = 32'b00101000111010000000000000000000; // ADD x7, x8, x0
      reg_file[19] = 32'b01011000101001010000000000010000; // ADDI x5, x5, 1
      reg_file[20] = 32'b01101000000000000000000000010000; // J 14 (PC=12), offset=2 signed
      
    end
  
  always @(posedge clk) 
    begin
      if (WE3)
        reg_file[A3] <= WD3;

      RD1 <= reg_file[A1];
      RD2 <= reg_file[A2];
    end
  
endmodule

module instruction_decoder(instruction, outRS1, outRS2, outRSD, outImm, outType, outOpcode);
  //Instruction format: 32-bit
  //R-type: {[31:29] type, [28:25] opCode, [24:20] rd, [19:15] rs1, [14:10] rs2, [9:0] empty} (type = 001)
  //I-Type: {[31:29] type, [28:25] opCode, [24:20] rd, [19:15] rs1, [14:3] signed imm, [2:0] empty} (type = 010)
  //B-Type: {[31:29] type, [28:25] opCode, [24:20] rd, [19:15] rs1, [14:3] signed imm (relative), [2:0] empty} (type = 011)
  
  //R-type instruction opcodes: NOOp: 000 & 001, ADDr: 010, SUBr: 011, ASR: 100, ASL:101
  //I-type instruction opcodes: ADDi: 110, SUBi: 111
  //B-Type instruciton opcodes: BEQ: 001, J: 010
  
  input [31:0] instruction;
  output reg [2:0] outType;
  output reg [4:0] outRS1, outRS2, outRSD;
  output reg signed [11:0] outImm;
  output reg [3:0] outOpcode;
  
  
  
  always@ (*)
    begin
      
      outRS1 = 5'b0;
  	  outRS2 = 5'b0;
  	  outRSD = 5'b0;
  	  outImm = 12'b0;
      
      outType = instruction[31:29];
      outOpcode = instruction[28:25];
      
      case(instruction[31:29])
        
        3'b001:
          begin
            outRSD = instruction[24:20];
            outRS1 = instruction[19:15];
            outRS2 = instruction[14:10];
          end
        
        3'b010:
          begin
            outRSD = instruction[24:20];
            outRS1 = instruction[19:15];
            outImm = $signed(instruction[14:3]);
          end
        
        3'b011:
          begin
            outRS1 = instruction[24:20];
            outRS2 = instruction[19:15];
            outImm = $signed(instruction[14:3]);
          end
      endcase
    end

endmodule

module ALU(clk, opType, opcode, inputA, inputB, inputImm, PCcurrent, PCnew, out);
  
  input clk;
  input [4:0] PCcurrent;
  input [2:0] opcode;
  input [2:0] opType;
  input [31:0] inputA, inputB;
  input signed [11:0] inputImm;
  output reg [4:0] PCnew;
  output reg [31:0] out;
  
  always @(posedge clk)
    begin
      
      if(opType == 3'b001 || opType == 3'b010)
        begin
          
          case(opcode)
			
            3'b000, 3'b001: out <= out;
            
            3'b010: out <= inputA + inputB;

            3'b011: out <= inputA - inputB;

            3'b100: out <= inputA << inputB;

            3'b101: out <= inputA >> inputB;

            3'b110: out <= inputA + inputImm;

            3'b111: out <= inputA - inputImm;

            

          endcase
          
          PCnew <= PCcurrent + 1;
          
        end
      
      else if(opType == 3'b011)
        begin
          
          case(opcode)
            
            3'b001:
              begin
                if(inputA == inputB)
                  begin
                    PCnew <= PCcurrent + inputImm;
                  end
                else
                  begin
                    PCnew <= PCcurrent + 1;
                  end
              end
            
            3'b010:
              PCnew <= PCcurrent + inputImm;
            
          endcase
          
        end
      
    end
endmodule


module SevenSegmentDisplay(clk, reset, value, an, seg);
	
  	input clk;            
    input reset;
    input [15:0] value;    
    output reg [3:0] an;   
    output reg [6:0] seg;
  
  	
    reg [19:0] refresh_counter = 0;
    wire [1:0] digit_select;
    reg [3:0] current_digit;

    
    always @(posedge clk or posedge reset) 
      begin
        
        if (reset)
            refresh_counter <= 0;
        else
            refresh_counter <= refresh_counter + 1;
   	  end

    assign digit_select = refresh_counter[19:18];

    always @(*) 
      begin
        
        case (digit_select)
            2'b00: 
              begin
                an = 4'b1110;
                current_digit = value[3:0];
              end
          
            2'b01: 
              begin
                an = 4'b1101;
                current_digit = value[7:4];
              end
          
            2'b10: 
              begin
                an = 4'b1011;
                current_digit = value[11:8];
              end
          
            2'b11: 
              begin
                an = 4'b0111;
                current_digit = value[15:12];
              end
          
            default: 
              begin
                an = 4'b1111;
                current_digit = 4'b0000;
              end
        endcase
        
      end

    always @(*) 
      begin
        case (current_digit)
            4'h0: seg = 7'b1000000;
            4'h1: seg = 7'b1111001;
            4'h2: seg = 7'b0100100;
            4'h3: seg = 7'b0110000;
            4'h4: seg = 7'b0011001;
            4'h5: seg = 7'b0010010;
            4'h6: seg = 7'b0000010;
            4'h7: seg = 7'b1111000;
            4'h8: seg = 7'b0000000;
            4'h9: seg = 7'b0010000;
            4'hA: seg = 7'b0001000;
            4'hB: seg = 7'b0000011;
            4'hC: seg = 7'b1000110;
            4'hD: seg = 7'b0100001;
            4'hE: seg = 7'b0000110;
            4'hF: seg = 7'b0001110;
            default: seg = 7'b1111111;
        endcase
      end

endmodule