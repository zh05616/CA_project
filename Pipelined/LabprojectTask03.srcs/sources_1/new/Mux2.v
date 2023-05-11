`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.04.2023 02:47:31
// Design Name: 
// Module Name: Mux2
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
module Mux2(
  input [1:0] S, [63:0] a, [63:0] b,[63:0] c,
  output reg [63:0] data_out);
  // 2:1 multiplexer with 3 inputs a,b,c and select signal S

  always @ (S or a or b or c)
    begin
      case(S)
        2'b00:  data_out <= a; // when S is 2'b00, output is a
        2'b01:  data_out <= b; // when S is 2'b01, output is b
        2'b10:  data_out <= c; // when S is 2'b10, output is c
      endcase
    end
endmodule