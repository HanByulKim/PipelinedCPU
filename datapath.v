`define WORD_SIZE 16    // data and address word size
`include "mapping.v"

module Datapath(
  output wire i_readM,                       // i_read from memory  
  output wire i_writeM,                       // i_write from memory
  output wire [`WORD_SIZE-1:0] i_address,    // i_current address for data
  output wire d_readM,                       // d_read from memory  
  output wire d_writeM,                       // d_write from memory
  output wire [`WORD_SIZE-1:0] d_address,    // d_current address for data
  inout wire [`WORD_SIZE-1:0] i_data,              // data being isnput or output
  inout [`WORD_SIZE-1:0] d_data,              // data being isnput or output
  input reset_n,                            // active-low RESET signal
  input clk,                                // clock signal
  
  // for debuging/testing purpose
  output reg [`WORD_SIZE-1:0] num_inst,   // number of instruction during execution
  output wire [`WORD_SIZE-1:0] output_port, // this will be used for a "WWD" instruction
  output is_halted                      // this means cpu is halted
  
);

// latchs
reg [`LATCH_SIZE-1:0] latchs;

// IF declaration
reg [`WORD_SIZE-1:0] IMemOut;
wire [`WORD_SIZE-1:0] nextPC;
reg FlushOrStalled;

// ID declaration
wire [`CTRL_BUS_SIZE-1:0] controlBus;   // control signal
wire IF_Flush;
wire [1:0] PCSrc;
wire [3:0] ALUCtrl;                 // ALU control signal
wire [`CTRL_BUS_SIZE-1:0]nextID_EX_Ctrls;

wire [`WORD_SIZE-1:0] inst;

wire PCWrite;
wire ControlFlush;
wire IF_ID_Stall;
wire ID_isLink;

wire BCEQ;
wire BCGT;
wire BCLT;

wire [`WORD_SIZE-1:0] RFOutA;
wire [`WORD_SIZE-1:0] RFOutB;

// sign-extended immediate
wire [`WORD_SIZE-1:0] extendImm;

// EX declaration

wire [`WORD_SIZE-1:0] ALUInputA;
wire [`WORD_SIZE-1:0] ALUInputB;
wire [`WORD_SIZE-1:0] forwardedRt;
wire [`WORD_SIZE-1:0] EX_ALUOut;
wire [5:0] EX_funct;
wire [1:0] EX_rd;
wire [1:0] forwardA;
wire [1:0] forwardB;
wire [1:0] forwardDest;


//MEM declaration
wire [`WORD_SIZE-1:0] DMemOut;

//WB declaration
wire [`WORD_SIZE-1:0] WB_RFWriteData;


// Datapath module declaration
Control_unit ctrl_unit(.opcode(latchs[`IF_ID_opcode]), .funct(latchs[`IF_ID_funct]), .reset_n(reset_n), .controlBus(controlBus));
Register register(.clk(clk), .write(latchs[`MEM_WB_RegWrite]), .addr1(latchs[`IF_ID_rs]), .addr2(latchs[`IF_ID_rt]), .addr3(latchs[`MEM_WB_rd]), .data3(WB_RFWriteData), .data1(RFOutA), .data2(RFOutB));
Hazard_Detection_unit hazard_detection_unit(.opcode(latchs[`IF_ID_opcode]), .funct(latchs[`IF_ID_funct]), .IF_ID_rs(latchs[`IF_ID_rs]), .IF_ID_rt(latchs[`IF_ID_rt]), .ID_EX_rt(latchs[`ID_EX_rt]), .ID_EX_MemRead(latchs[`ID_EX_MemRead]), .BCEQ(BCEQ), .BCGT(BCGT), .BCLT(BCLT), .PCWrite(PCWrite), .IFFlush(IF_Flush), .IF_ID_Stall(IF_ID_Stall), .ControlFlush(ControlFlush), .PCSrc(PCSrc), .isLink(ID_isLink));
Forwarding_unit forwarding_unit(.IF_ID_rs(latchs[`IF_ID_rs]), .ID_EX_rs(latchs[`ID_EX_rs]), .ID_EX_rt(latchs[`ID_EX_rt]), .ID_EX_rd(EX_rd), .EX_MEM_rd(latchs[`EX_MEM_rd]), .MEM_WB_rd(latchs[`MEM_WB_rd]), 
                                .ID_EX_RegWrite(latchs[`ID_EX_RegWrite]), .EX_MEM_RegWrite(latchs[`EX_MEM_RegWrite]), .MEM_WB_RegWrite(latchs[`MEM_WB_RegWrite]), .forwardA(forwardA), .forwardB(forwardB), .forwardDest(forwardDest));

ALUctrl_unit aluctrl_unit(.ALUOp(latchs[`ID_EX_ALUOp]), .funct(EX_funct), .ALUCtrl(ALUCtrl));
ALU alu(.A(ALUInputA), .B(ALUInputB), .ALUCtrl(ALUCtrl), .C(EX_ALUOut), .output_port(output_port), .isHLT(is_halted));

// IF wiring
assign i_address = latchs[`IF_pc];
assign i_readM = PCWrite;
assign i_writeM = 0;
assign nextPC=(PCSrc<=1) ? ( (PCSrc==0) ? extendImm+latchs[`IF_ID_pc]: latchs[`IF_pc]+1 ) : ( (PCSrc==2) ? {latchs[`IF_ID_pc]>>12,latchs[`IF_ID_target]} : 
    (forwardDest==0)? RFOutA : 
    (forwardDest==1)? WB_RFWriteData : 
    (forwardDest==2)? 
        (latchs[`EX_MEM_isLink]==1)? latchs[`EX_MEM_pc]:latchs[`EX_MEM_ALUOut] : 
        (latchs[`ID_EX_isLink]==1)? latchs[`ID_EX_pc]:EX_ALUOut);

// ID wiring
assign extendImm = (latchs[`IF_ID_imm]>>7 == 1) ? {8'b11111111,latchs[`IF_ID_imm]} : {8'b00000000,latchs[`IF_ID_imm]}; // sign-extend
assign nextID_EX_Ctrls = (ControlFlush ==0) ? controlBus : 12'bz;
assign BCEQ = (RFOutA==RFOutB)? 1: 0;
assign BCGT = (RFOutA > 0)? 1:0;
assign BCLT = (RFOutA < 0)? 1:0;

// EX wiring
assign ALUInputA = (forwardA == 0)? latchs[`ID_EX_rsData] : (forwardA ==1)? WB_RFWriteData : (latchs[`EX_MEM_isLink]==1)? latchs[`EX_MEM_pc]: latchs[`EX_MEM_ALUOut];
assign forwardedRt = (forwardB == 0)? latchs[`ID_EX_rtData] : (forwardB ==1)? WB_RFWriteData : (latchs[`EX_MEM_isLink]==1)? latchs[`EX_MEM_pc]: latchs[`EX_MEM_ALUOut];
assign ALUInputB = (latchs[`ID_EX_ALUSrc]==0)? (forwardedRt) : latchs[`ID_EX_imm];
assign EX_funct = latchs[`ID_EX_imm];
assign EX_rd = (latchs[`ID_EX_RegDst]==0)? latchs[`ID_EX_rt] : (latchs[`ID_EX_RegDst]==1)? latchs[`ID_EX_rd] : 2;
// MEM wiring
assign d_address = latchs[`EX_MEM_ALUOut];
assign d_writeM = latchs[`EX_MEM_MemWrite];
assign d_readM = latchs[`EX_MEM_MemRead];
assign d_data = (d_writeM == 1)? (latchs[`EX_MEM_MemWriteData]) : 16'bz;
assign DMemOut = (d_readM ==1)? d_data: 16'bz;

// WB wiring
assign WB_RFWriteData = (latchs[`MEM_WB_MemtoReg]==0)? latchs[`MEM_WB_ALUOut] : (latchs[`MEM_WB_MemtoReg]==1)? latchs[`MEM_WB_DMemOut] : latchs[`MEM_WB_pc];

// check reset_n
always @(negedge reset_n) begin
    num_inst<=-1;
    latchs=0;
    latchs[`IF_pc] = -1;
    FlushOrStalled=0;
end

always @(negedge clk) begin
    if(latchs[`ID_EX_incNum]==1) num_inst = num_inst + 1;
end

always @(posedge clk) begin
    if(reset_n==1&&is_halted==0) begin
        if(IF_Flush==1) begin 
            latchs[`IF_ID_inst] = 0;
            latchs[`IF_ID_incNum] = 0;
        end
        if(IF_ID_Stall==1) latchs[`IF_ID_incNum] = 0;
    end
end


always@(posedge clk) begin
    if(reset_n==1&&is_halted==0) begin
        latchs[`IF_ID_incNum]<=1;
        if(PCWrite == 1) latchs[`IF_pc] <= nextPC;
        latchs[`ID_EX_incNum]<=latchs[`IF_ID_incNum];
        // IF/ID
        if(IF_ID_Stall == 0) begin
            latchs[`IF_ID_pc] <=latchs[`IF_pc] + 1; 
            latchs[`IF_ID_inst] <= IMemOut;
        end
        // ID/EX
        latchs[`ID_EX_isLink] <= ID_isLink;
        latchs[`ID_EX_controlBus] <= nextID_EX_Ctrls;
        latchs[`ID_EX_pc] <= latchs[`IF_ID_pc];
        latchs[`ID_EX_rsData] <= RFOutA;
        latchs[`ID_EX_rtData] <= RFOutB;
        latchs[`ID_EX_imm] <= extendImm;
        latchs[`ID_EX_rs] <= latchs[`IF_ID_rs];
        latchs[`ID_EX_rt] <= latchs[`IF_ID_rt];
        latchs[`ID_EX_rd] <= latchs[`IF_ID_rd];
        // EX/MEM
        latchs[`EX_MEM_isLink] <= latchs[`ID_EX_isLink];
        latchs[`EX_MEM_controlWB] <= latchs[`ID_EX_controlWB];
        latchs[`EX_MEM_controlMEM] <= latchs[`ID_EX_controlMEM];    
        latchs[`EX_MEM_pc] <= latchs[`ID_EX_pc];
        latchs[`EX_MEM_ALUOut] <= EX_ALUOut;
        latchs[`EX_MEM_MemWriteData] <= ((forwardB == 0)? latchs[`ID_EX_rtData] : (forwardB ==1)? WB_RFWriteData : latchs[`EX_MEM_ALUOut]);
        latchs[`EX_MEM_rd] <= EX_rd;
        // MEM/WB
        latchs[`MEM_WB_controlWB] <= latchs[`EX_MEM_controlWB];
        latchs[`MEM_WB_pc] <= latchs[`EX_MEM_pc];
        latchs[`MEM_WB_DMemOut] <= DMemOut;
        latchs[`MEM_WB_ALUOut] <= latchs[`EX_MEM_ALUOut];
        latchs[`MEM_WB_rd] <= latchs[`EX_MEM_rd];
    end
end
wire de_ID_EX_isLink;
assign de_ID_EX_isLink=latchs[`ID_EX_isLink];
wire de_EX_MEM_isLink;
assign de_EX_MEM_isLink=latchs[`EX_MEM_isLink];
always @(i_data) begin
    if(i_readM==1) IMemOut = i_data;
end

endmodule