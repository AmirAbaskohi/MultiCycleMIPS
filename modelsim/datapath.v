module PC(input [31:0]in, input clk, rst, load, output [31:0]out);
    reg [31:0] pc;
    always@(posedge clk, posedge rst)begin
      if(rst) pc <= 32'b0;
      else begin
        if(load) pc <= in;
      end
    end
    assign out = pc;
endmodule

module MUX2_32(input [31:0]first, second,input select, output [31:0]out);
    assign out = (select == 1'b0) ? first : second;
endmodule

module MUX2_5(input [4:0]first, second,input select, output [4:0]out);
    assign out = (select == 1'b0) ? first : second;
endmodule

module MUX4_32(input [31:0]first, second, third, fourth,input [1:0]select, output[31:0] out);
    assign out = (select == 2'b00) ? first : (select == 2'b01) ? second :
                 (select == 2'b10) ? third : fourth;
endmodule

module ALU(input [31:0]A, B,input [2:0]ALUOperation, output zero, output reg[31:0] ALUResult);
    always@(A, B, ALUOperation)begin
      if(ALUOperation == 3'b000) ALUResult = A & B;
      else if(ALUOperation == 3'b001) ALUResult = A | B;
      else if(ALUOperation == 3'b010) ALUResult = A + B;
      else if(ALUOperation == 3'b011) ALUResult = A + (~B) + 1;
      else if(ALUOperation == 3'b100)begin
        if(A[31] != B[31])begin
            if(A[31] > B[31]) ALUResult = 1;
            else ALUResult = 0;
        end
        else begin
          if(A < B) ALUResult = 1;
          else ALUResult = 0;
        end
      end
    end
    assign zero = (ALUResult == 0) ? 1'b1 : 1'b0;
endmodule

module SignExt(input [15:0] in, output [31:0] out);
    assign out = { {16{in[15]}}, in};
endmodule

module SHL2_26_to_28(input[25:0] in, output[27:0] out);
  assign out = {in,2'b00};
endmodule

module SHL2_32_to_32(input[31:0] in, output[31:0] out);
  assign out = {in[29:0] , 2'b00};
endmodule

module RegisterWithLoad(input [31:0]in, input clk, rst, load, output[31:0]out);
  reg [31:0]Reg;
  always@(posedge clk)begin
    if(rst) Reg <= 32'b0;
    else begin
      if(load) Reg <= in;
    end
  end
  assign out = Reg;
endmodule

module RegisterWithoutLoad(input [31:0]in, input clk, rst, output[31:0]out);
  reg [31:0]Reg;
  always@(posedge clk)begin
    if(rst) Reg <= 32'b0;
    else Reg <= in;
  end
  assign out = Reg;
endmodule

module Concat4_26(input [3:0] first, input [27:0] second, output[31:0] out);
  assign out = {first,second};
endmodule

module RegFile(input[4:0] ReadReg1, ReadReg2, WriteReg, input[31:0] WriteData, input clk, RegWrite, output[31:0] ReadData1, ReadData2);
  reg [31:0] R[0:31];
  initial R[0] <= 32'b0;
  always@(posedge clk) begin
    if (RegWrite == 1)begin
      if (WriteReg != 0)
        R[WriteReg] <= WriteData;
    end
  end
  assign ReadData1 = R[ReadReg1];
  assign ReadData2 = R[ReadReg2];
endmodule

module Memory(input[31:0] Address, WriteData, input clk, MemRead, MemWrite, output[31:0] ReadData);
  reg [7:0] Mem [0:128000];
  initial $readmemh("Mem.data", Mem);
  assign ReadData = MemRead ? {Mem[Address], Mem[Address+1], Mem[Address+2], Mem[Address+3]} : 32'b0;
  always@(posedge clk)begin
    if(MemWrite == 1) begin
      Mem[Address] = WriteData[31:24];
      Mem[Address+1] = WriteData[23:16];
      Mem[Address+2] = WriteData[15:8];
      Mem[Address+3] = WriteData[7:0];
    end
  end
endmodule

module MultiCycleMIPS_DP(input[2:0] ALUOperation, input[1:0] PCSrc, ALUSrcB,
                          input PCLoad, IRWrite, IorD, MemRead, MemWrite, MemtoReg,
                          RegDst, lastReg, PCtoReg, RegWrite, ALUSrcA, clk, rst,
                          output zero, output [5:0] opcode, funccode);
    
  wire[31:0] mux1out, mux2out, mux5out, mux6out, mux7out, mux8out;
  wire[4:0] mux3out, mux4out;
  wire[31:0] pcout;
  wire[31:0] Aout,Bout;
  wire[31:0] ALUResout;
  wire[31:0] MemReadData, RegReadData1, RegReadData2;
  wire[31:0] inst, memdata;
  wire[31:0] ALUout;
  wire[31:0] concatout;
  wire[27:0] shl28out;
  wire[31:0] shl32out;
  wire[31:0] signextout;
  wire[4:0] constValue31;
  wire[31:0] constValue4;

  assign constValue4 = 32'b00000000000000000000000000000100;
  assign constValue31 = 5'b11111;
  assign funccode = inst[5:0];
  assign opcode = inst[31:26];

  PC pc(mux1out, clk, rst, PCLoad, pcout);

  Memory memory(mux2out, Bout, clk, MemRead, MemWrite, MemReadData);

  RegisterWithLoad IR(MemReadData, clk, rst, IRWrite, inst);

  RegisterWithoutLoad MDR(MemReadData, clk, rst, memdata);
  RegisterWithoutLoad A(RegReadData1, clk, rst, Aout);
  RegisterWithoutLoad B(RegReadData2, clk, rst, Bout);
  RegisterWithoutLoad ALURes(ALUout, clk, rst, ALUResout);

  ALU alu(mux7out, mux8out, ALUOperation, zero, ALUout);

  RegFile regfile(inst[25:21], inst[20:16], mux4out, mux6out, clk, RegWrite, RegReadData1, RegReadData2);

  SHL2_26_to_28 shl26to28(inst[25:0], shl28out);
  SHL2_32_to_32 shl32to32(signextout, shl32out);

  SignExt sext(inst[15:0], signextout);

  Concat4_26 concat(pcout[31:28], shl28out, concatout);

  MUX2_32 mux2(pcout, ALUResout, IorD, mux2out);
  MUX2_32 mux5(ALUResout, memdata, MemtoReg, mux5out);
  MUX2_32 mux6(mux5out, pcout, PCtoReg, mux6out);
  MUX2_32 mux7(pcout, Aout, ALUSrcA,mux7out);

  MUX2_5 mux3(inst[20:16], inst[15:11], RegDst, mux3out);
  MUX2_5 mux4(mux3out, constValue31, lastReg, mux4out);

  MUX4_32 mux1(ALUout, concatout, ALUResout, Aout, PCSrc, mux1out);
  MUX4_32 mux8(Bout, constValue4, signextout, shl32out, ALUSrcB, mux8out);
endmodule