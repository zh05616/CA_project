`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2023 02:42:26
// Design Name: 
// Module Name: Instruction_Memory
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
module Instruction_Memory(
    input [63:0] Inst_Address,
    output [31:0] Instruction
);

  reg [7:0] memory [15:0];

  initial begin
    		memory[0] = 8'b00010011;
		memory[1] = 8'b00001011;
		memory[2] = 8'b00000000;
		memory[3] = 8'b00000000;
		memory[4] = 8'b10010011;
		memory[5] = 8'b00001011;
		memory[6] = 8'b00000000;
		memory[7] = 8'b00000000;
		memory[8] = 8'b00010011;
		memory[9] = 8'b00001100;
		memory[10] = 8'b00000000;
		memory[11] = 8'b00000000;
		memory[12] = 8'b10010011;
		memory[13] = 8'b00001100;
		memory[14] = 8'b00000000;
		memory[15] = 8'b00000000;
		memory[16] = 8'b10010011;
		memory[17] = 8'b00000010;
		memory[18] = 8'b00000000;
		memory[19] = 8'b00000000;
		memory[20] = 8'b00010011;
		memory[21] = 8'b00000011;
		memory[22] = 8'b00000000;
		memory[23] = 8'b00000000;
		memory[24] = 8'b10010011;
		memory[25] = 8'b00000011;
		memory[26] = 8'b00000000;
		memory[27] = 8'b00000000;
		memory[28] = 8'b00010011;
		memory[29] = 8'b00001101;
		memory[30] = 8'b01000000;
		memory[31] = 8'b00000000;
		memory[32] = 8'b00000011;
		memory[33] = 8'b00110011;
		memory[34] = 8'b11111100;
		memory[35] = 8'b00000000;
		memory[36] = 8'b10000011;
		memory[37] = 8'b10110011;
		memory[38] = 8'b11111100;
		memory[39] = 8'b00000000;
		memory[40] = 8'b01100011;
		memory[41] = 8'b01000100;
		memory[42] = 8'b01110011;
		memory[43] = 8'b00000010;
		memory[44] = 8'b10010011;
		memory[45] = 8'b10001100;
		memory[46] = 8'b10001100;
		memory[47] = 8'b00000000;
		memory[48] = 8'b10010011;
		memory[49] = 8'b10001011;
		memory[50] = 8'b00011011;
		memory[51] = 8'b00000000;
		memory[52] = 8'b11100011;
		memory[53] = 8'b10010110;
		memory[54] = 8'b10101011;
		memory[55] = 8'b11111111;
		memory[56] = 8'b10010011;
		memory[57] = 8'b00001011;
		memory[58] = 8'b00000000;
		memory[59] = 8'b00000000;
		memory[60] = 8'b10010011;
		memory[61] = 8'b00001100;
		memory[62] = 8'b00000000;
		memory[63] = 8'b00000000;
		memory[64] = 8'b00010011;
		memory[65] = 8'b00001011;
		memory[66] = 8'b00011011;
		memory[67] = 8'b00000000;
		memory[68] = 8'b00010011;
		memory[69] = 8'b00001100;
		memory[70] = 8'b10001100;
		memory[71] = 8'b00000000;
		memory[72] = 8'b11100011;
		memory[73] = 8'b00011100;
		memory[74] = 8'b10101011;
		memory[75] = 8'b11111101;
		memory[76] = 8'b01100011;
		memory[77] = 8'b00001010;
		memory[78] = 8'b00000000;
		memory[79] = 8'b00000000;
		memory[80] = 8'b10010011;
		memory[81] = 8'b00000010;
		memory[82] = 8'b00000011;
		memory[83] = 8'b00000000;
		memory[84] = 8'b10100011;
		memory[85] = 8'b00110111;
		memory[86] = 8'b01111100;
		memory[87] = 8'b00000000;
		memory[88] = 8'b10100011;
		memory[89] = 8'b10110111;
		memory[90] = 8'b01011100;
		memory[91] = 8'b00000000;
		memory[92] = 8'b11100011;
		memory[93] = 8'b00001000;
		memory[94] = 8'b00000000;
		memory[95] = 8'b11111100;
  end

  assign Instruction = {memory[Inst_Address+3], memory[Inst_Address+2], memory[Inst_Address+1], memory[Inst_Address]}; 

endmodule