`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2023 02:41:22
// Design Name: 
// Module Name: ALU_Control
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
module ALU_Control(
  input [1:0] ALUOp, [3:0] Funct,
  output reg [3:0] Operation
);
  reg [3:0] Op_reg;

  always @(*) begin
    case (ALUOp)
      2'b00: Operation = 4'b0010;
      2'b01: begin
        case (Funct)
          4'b0000: Operation = 4'b0110;
          4'b1001: Operation = 4'b0011;
          4'b0100: Operation = 4'b1010;
        endcase  
      end
      2'b10: begin
        case (Funct)
          4'b0000: Operation = 4'b0010;
          4'b1000: Operation = 4'b0110; 
          4'b0111: Operation = 4'b0000;
          4'b0110: Operation = 4'b0001;
        endcase
      end 
    endcase
  end
endmodule