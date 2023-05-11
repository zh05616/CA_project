`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2023 02:40:21
// Design Name: 
// Module Name: Control_Unit
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
module Control_Unit(
  input [6:0] Opcode,
  input MUX_Control,
  output reg Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite,
  output reg [1:0] ALUOp 
);
  always @(*) begin
    if (~MUX_Control)
      begin
        ALUSrc<=0;
        MemtoReg<=0;
        RegWrite<=0;
        MemRead<=0;
        MemWrite<=0;
        Branch<=0;
        ALUOp<=2'b00;
      end
    else
      begin
        case (Opcode)
          7'b0110011: begin
            ALUSrc = 0;
            MemtoReg = 0;
            RegWrite = 1;
            MemRead = 0;
            MemWrite = 0;
            Branch = 0;
            ALUOp = 2'b10;
          end
          7'b0000011: begin
            ALUSrc = 1;
            MemtoReg = 1;
            RegWrite = 1;
            MemRead = 1;
            MemWrite = 0;
            Branch = 0;
            ALUOp = 2'b00;
          end
          7'b0100011: begin
            ALUSrc = 1;
            RegWrite = 0;
            MemRead = 0;
            MemWrite = 1;
            Branch = 0;
            ALUOp = 2'b00;
          end
          7'b1100011: begin
            ALUSrc = 0;
            RegWrite = 0;
            MemRead = 0;
            MemWrite = 0;
            Branch = 1;
            ALUOp = 2'b01;
          end
          7'b0010011: begin
            ALUSrc = 1;
            RegWrite = 1;
            MemRead = 0;
            MemWrite = 0;
            Branch = 0;
            ALUOp = 2'b00;
            MemtoReg = 0;
          end
          default: begin
            ALUSrc = 0;
            RegWrite = 0;
            MemRead = 0;
            MemWrite = 0;
            Branch = 0;
            ALUOp = 2'b00;
            MemtoReg = 0;
          end
        endcase
      end
  end
endmodule