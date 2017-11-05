`include "opcodes.v"
`include "mapping.v"

`define RegWrite 11
`define MemtoReg 10:9
`define MemRead 8
`define MemWrite 7
`define RegDst 6:5
`define ALUOp 4:1
`define ALUSrc 0

module Control_unit (
  input wire [3:0] opcode,
  input wire [5:0] funct,
  input wire reset_n,
  output reg [`CTRL_BUS_SIZE-1:0] controlBus
);
always @(*) begin
    if({opcode, funct}==10'b1111011100) controlBus = 12'b0xx00xx1111x;
    else if({opcode, funct}==10'b1111011001) controlBus = 12'b0xx00xx1111x;
    else if({opcode, funct}==10'b1111011010) controlBus = 12'b11000101111x;
    else if({opcode, funct}==10'b1111011101) controlBus = 12'b0xx00xx1111x;
    else if(opcode==4'b1111) controlBus = 12'b100000111110;
    else if(opcode==4'b0100) controlBus = 12'b100000001001;
    else if(opcode==4'b0101) controlBus = 12'b100000001011;
    else if(opcode==4'b0110) controlBus = 12'b100000001101;
    else if(opcode==4'b0111) controlBus = 12'b101100001111;
    else if(opcode==4'b1000) controlBus = 12'b0xx01xx10001;
    else if(opcode==4'b0000) controlBus = 12'b0xx00xx1011x;
    else if(opcode==4'b0001) controlBus = 12'b0xx00xx0001x;
    else if(opcode==4'b0010) controlBus = 12'b0xx00xx0010x;
    else if(opcode==4'b0011) controlBus = 12'b0xx00xx0011x;
    else if(opcode==4'b1001) controlBus = 12'b0xx00xx1001x;
    else if(opcode==4'b1010) controlBus = 12'b11000101010x;
end
endmodule