module SignalController(input [5:0]opcode, input clk, rst, output reg IorD, MemRead, MemWrite, IRWrite, RegDst, LastReg, MemtoReg,
                        PCtoReg, RegWrite, ALUSrcA, PcWrite, beq, bne, output reg [1:0] ALUSrcB, PCSrc, AluOp);  
  reg [4:0] ns, ps;
  always@(posedge clk, posedge rst)
  begin
    if(rst) ps <= 1;
    else ps <= ns;
  end
  
  always@(ps, opcode)
  begin
    case(ps)
      1 : ns <= 2;
      2 : begin
        case(opcode)
          6'b000010 : ns <= 3;  //j
          6'b000100 : ns <= 4;  //beq
          6'b000101 : ns <= 5;  //bne
          6'b100000 : ns <= 6;  //jr
          6'b100011 : ns <= 7;  //lw
          6'b101011 : ns <= 7;  //sw
          6'b000000 : ns <= 11; //rt
          6'b001000 : ns <= 13; //addi
          6'b001100 : ns <= 15; //andi
          6'b000011 : ns <= 17; //jal
        endcase
      end
      3 : ns <= 1;
      4 : ns <= 1;
      5 : ns <= 1;
      6 : ns <= 1;
      7 : 
      begin
        if(opcode == 6'b100011) ns <= 8; //lw
        else ns <= 10; //sw
      end
      8: ns <= 9;
      9: ns <= 1;
      10: ns <= 1;
      11: ns <= 12;
      12: ns <= 1;
      13: ns <= 14;
      14: ns <= 1;
      15: ns <= 16;
      16: ns <= 1;
      17: ns <= 1;
    endcase
  end
  
  always@(ps)
  begin
    {IorD,MemRead,MemWrite,IRWrite,RegDst,LastReg,MemtoReg,PCtoReg,RegWrite,ALUSrcA,PcWrite,beq,bne} = 13'b0000000000000;
    {ALUSrcB, PCSrc, AluOp} = 6'b000000;
    case(ps)
      1: begin
        {MemRead, IRWrite, PcWrite} = 3'b111;
        ALUSrcB = 2'b01;
      end
      2: ALUSrcB = 2'b11;
      3: begin
        PCSrc = 2'b01;
        PcWrite = 1'b1;
      end
      4: begin
        {ALUSrcA, beq} = 2'b11;
        AluOp = 2'b01;
        PCSrc = 2'b10;
      end
      5: begin
        {ALUSrcA, bne} = 2'b11;
        AluOp = 2'b01;
        PCSrc = 2'b10;
      end
      6: begin
        PcWrite = 1'b1;
        PCSrc = 2'b11;
      end
      7: begin
        ALUSrcA = 1'b1;
        ALUSrcB = 2'b10;
      end
      8: {IorD, MemRead} = 2'b11;
      9: {MemtoReg, RegWrite} = 2'b11;
      10: {IorD, MemWrite} = 2'b11;
      11: begin
        ALUSrcA = 1'b1;
        AluOp = 2'b10;
      end
      12: {RegWrite, RegDst} = 2'b11;
      13: begin
        ALUSrcA = 1'b1;
        ALUSrcB = 2'b10;
      end
      14: RegWrite = 1'b1;
      15: begin
        ALUSrcA = 1'b1;
        ALUSrcB = 2'b10;
        AluOp = 2'b11;
      end
      16: RegWrite = 1'b1;
      17: begin
        {PcWrite, PCtoReg, LastReg, RegWrite} = 4'b1111;
        PCSrc = 2'b01; 
      end
    endcase
  end
endmodule

module ALUController(input [1:0] AluOp, input [5:0] funccode, output reg [2:0] ALUOperation);
    always@(AluOp, funccode)begin
      if(AluOp == 2'b10)begin
        if(funccode == 6'b100000) ALUOperation = 3'b010;
        else if(funccode == 6'b100100) ALUOperation = 3'b000;
        else if(funccode == 6'b100101) ALUOperation = 3'b001;
        else if(funccode == 6'b100010) ALUOperation = 3'b011;
        else if(funccode == 6'b101010) ALUOperation = 3'b100;
        else ALUOperation = 3'b010;
      end
      else if(AluOp == 2'b00) ALUOperation = 3'b010;
      else if(AluOp == 2'b01) ALUOperation = 3'b011;
      else if(AluOp == 2'b11) ALUOperation = 3'b000;
    end
endmodule

module MultiCycleMIPS_Controller(input [5:0] opcode, funccode, input zero, clk, rst, output IorD, MemRead, MemWrite, IRWrite, RegDst, LastReg, 
                        MemtoReg, PCtoReg, RegWrite, ALUSrcA, PCLoad, output[1:0] ALUSrcB, PCSrc, output[2:0] ALUOperation);
  wire[1:0] AluOp;
  wire PcWrite, beq, bne;
  SignalController SC (opcode, clk, rst, IorD, MemRead, MemWrite, IRWrite, RegDst, LastReg, MemtoReg,
                       PCtoReg, RegWrite, ALUSrcA, PcWrite, beq, bne, ALUSrcB, PCSrc, AluOp);
                       
  ALUController AC (AluOp, funccode, ALUOperation);
  
  assign PCLoad = PcWrite | (beq & zero) | (bne & (~zero));
endmodule