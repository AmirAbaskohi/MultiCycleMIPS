`timescale 1ns/1ns
module MultiCycleMIPS(input clk, rst);
    wire[1:0] PCSrc, ALUSrcB;

    wire[2:0] ALUOperation;

    wire[5:0] opcode, funccode;

    wire zero, IorD, MemRead, MemWrite, IRWrite, RegDst, LastReg;
    wire MemtoReg, PCtoReg, RegWrite, ALUSrcA, PCLoad;

    MultiCycleMIPS_DP datapath(ALUOperation, PCSrc, ALUSrcB, PCLoad, IRWrite, IorD, MemRead, MemWrite,
                                MemtoReg, RegDst, LastReg, PCtoReg, RegWrite, ALUSrcA, clk, rst,
                                zero, opcode, funccode);

    MultiCycleMIPS_Controller controller(opcode, funccode, zero, clk, rst, IorD, MemRead, MemWrite,
                                            IRWrite, RegDst, LastReg, MemtoReg, PCtoReg, RegWrite,
                                            ALUSrcA, PCLoad, ALUSrcB, PCSrc, ALUOperation);
endmodule

module MultiCycleMIPS_tb();
  reg clk,rst,enabled;
  MultiCycleMIPS test(clk,rst);
  initial clk = 0;
  always begin
    #50 clk = ~clk;
  end
  initial begin
    enabled = 1;
    rst = 1;
    #100 rst = 0;
  end
  always begin : loop_block
    if (enabled == 1) begin
      #100
      $display ("pc is : %b",MultiCycleMIPS_tb.test.datapath.pcout);
      $display ("opc is : %b",MultiCycleMIPS_tb.test.datapath.opcode);
      $display ("func is : %b",MultiCycleMIPS_tb.test.datapath.funccode);
      $display ("Inst is : %b",MultiCycleMIPS_tb.test.datapath.inst);
      
      $display ("R1 is : %b",MultiCycleMIPS_tb.test.datapath.regfile.R[1]);
      $display ("R2 is : %b",MultiCycleMIPS_tb.test.datapath.regfile.R[2]);
      $display ("R3 is : %b",MultiCycleMIPS_tb.test.datapath.regfile.R[3]);
      $display ("R4 is : %b",MultiCycleMIPS_tb.test.datapath.regfile.R[4]);
      $display ("R5 is : %b",MultiCycleMIPS_tb.test.datapath.regfile.R[5]);
      $display ("R6 is : %b",MultiCycleMIPS_tb.test.datapath.regfile.R[6]);
      $display ("R7 is : %b",MultiCycleMIPS_tb.test.datapath.regfile.R[7]);
      $display ("R8 is : %b",MultiCycleMIPS_tb.test.datapath.regfile.R[8]);
      $display ("R9 is : %b",MultiCycleMIPS_tb.test.datapath.regfile.R[9]);
      $display ("R10 is : %b",MultiCycleMIPS_tb.test.datapath.regfile.R[10]);
      
      $display ("mem[2000] is : %b",{MultiCycleMIPS_tb.test.datapath.memory.Mem[2000],
                                     MultiCycleMIPS_tb.test.datapath.memory.Mem[2001],
                                     MultiCycleMIPS_tb.test.datapath.memory.Mem[2002],
                                     MultiCycleMIPS_tb.test.datapath.memory.Mem[2003]});
                                     
      $display ("mem[2004] is : %b",{MultiCycleMIPS_tb.test.datapath.memory.Mem[2004],
                                     MultiCycleMIPS_tb.test.datapath.memory.Mem[2005],
                                     MultiCycleMIPS_tb.test.datapath.memory.Mem[2006],
                                     MultiCycleMIPS_tb.test.datapath.memory.Mem[2007]});
      
      $display ("present state is : %b",MultiCycleMIPS_tb.test.controller.SC.ps);
      if (MultiCycleMIPS_tb.test.datapath.opcode[0] ===1'bX) begin
        enabled = 0;
        disable loop_block;
        $stop;
      end
    end else #10000;
  end
endmodule