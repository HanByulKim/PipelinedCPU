

module Forwarding_unit(
    input wire [1:0] IF_ID_rs,
    input wire [1:0] ID_EX_rs,
    input wire [1:0] ID_EX_rt,
    input wire [1:0] ID_EX_rd,
    input wire [1:0] EX_MEM_rd,
    input wire [1:0] MEM_WB_rd,
    input wire ID_EX_RegWrite,
    input wire EX_MEM_RegWrite,
    input wire MEM_WB_RegWrite,
    output reg [1:0] forwardA,
    output reg [1:0] forwardB,
    output reg [1:0] forwardDest
);
    always @(*) begin
        if(EX_MEM_RegWrite && EX_MEM_rd == ID_EX_rs) forwardA = 2'b10;
        else if(MEM_WB_RegWrite && MEM_WB_rd == ID_EX_rs) forwardA = 2'b01;
        else forwardA=0;
        
        if(EX_MEM_RegWrite && EX_MEM_rd == ID_EX_rt) forwardB=2'b10;
        else if(MEM_WB_RegWrite && MEM_WB_rd == ID_EX_rt) forwardB = 2'b01;
        else forwardB=0;
        
        if(ID_EX_RegWrite && ID_EX_rd == IF_ID_rs) forwardDest = 2'b11;
        else if(EX_MEM_RegWrite && EX_MEM_rd == IF_ID_rs) forwardDest=2'b10;
        else if(MEM_WB_RegWrite && MEM_WB_rd == IF_ID_rs) forwardDest = 2'b01;
        else forwardDest=0;
    end        
    
endmodule
