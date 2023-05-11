`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2023 02:46:57
// Design Name: 
// Module Name: MEM_WB
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
module MEM_WB
  (
    input clk,
    input reset,
    input [63:0] ALUResult_EXMEM,      // input ALU result from EX/MEM pipeline register
    input [63:0] Read_Data,            // input data read from memory
    input [4:0] Rd_EXMEM,              // input destination register number from EX/MEM pipeline register
    input MemtoReg_EXMEM,              // input signal indicating if Read_Data should be written to register file or ALUResult
    input RegWrite_EXMEM,              // input signal indicating if register file should be written to

    output reg [63:0] ALUResult_MEMWB, // output ALU result to be written to register file or used as address for store instruction
    output reg [63:0] Read_Data_MEMWB, // output data to be written to register file
    output reg [4:0] Rd_MEMWB,         // output destination register number
    output reg MemtoReg_MEMWB,         // output signal indicating if Read_Data or ALUResult should be written to register file
    output reg RegWrite_MEMWB          // output signal indicating if register file should be written to
  );
  
  always@(posedge clk)
  begin 
    if(reset == 1'b1)
      begin
        ALUResult_MEMWB = 64'b0;
        Read_Data_MEMWB = 64'b0;
        Rd_MEMWB = 5'b0;
        MemtoReg_MEMWB = 1'b0;
        RegWrite_MEMWB = 1'b0;
      end
   else
     begin
      ALUResult_MEMWB = ALUResult_EXMEM; // output ALU result from EX/MEM pipeline register
      Read_Data_MEMWB = Read_Data;       // output data read from memory
      Rd_MEMWB = Rd_EXMEM;               // output destination register number from EX/MEM pipeline register
      MemtoReg_MEMWB = MemtoReg_EXMEM;   // output signal indicating if Read_Data or ALUResult should be written to register file
      RegWrite_MEMWB = RegWrite_EXMEM;   // output signal indicating if register file should be written to
     end
   end
   
endmodule
