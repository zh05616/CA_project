`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2023 02:42:58
// Design Name: 
// Module Name: Data_Memory
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
module Data_Memory(
  input [63:0] Mem_Addr, [63:0] Write_Data,
  input clk, MemWrite, MemRead,
  output [63:0] Read_Data
);

  reg [7:0] memory [63:0];
  reg [63:0] temp_data;
  integer i;

  initial begin
  for (i=0 ;i<64 ; i = i + 1) begin
    memory[i] <= i;
  end
  end

  always @(*) begin
    if (MemWrite) begin
      memory[Mem_Addr] = Write_Data[7:0];
      memory[Mem_Addr+1] = Write_Data[15:8];
      memory[Mem_Addr+2] = Write_Data[23:16];
      memory[Mem_Addr+3] = Write_Data[31:24];
      memory[Mem_Addr+4] = Write_Data[39:32];
      memory[Mem_Addr+5] = Write_Data[47:40];
      memory[Mem_Addr+6] = Write_Data[55:48];
      memory[Mem_Addr+7] = Write_Data[63:56];
    end
  end

  always @(*) begin
    if (MemRead) begin
      temp_data <= {memory[Mem_Addr+7], memory[Mem_Addr+6], memory[Mem_Addr+5], memory[Mem_Addr+4], memory[Mem_Addr+3], memory[Mem_Addr+2], memory[Mem_Addr+1], memory[Mem_Addr]};
    end
  end

  assign Read_Data = temp_data;

endmodule