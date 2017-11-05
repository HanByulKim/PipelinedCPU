`define IF_pc 15:0
`define IF_ID_pc 31:16
`define IF_ID_inst 47:32
`define IF_ID_incNum 48
`define ID_EX_ALUSrc 49
`define ID_EX_ALUOp 53:50
`define ID_EX_RegDst 55:54
`define ID_EX_MemWrite 56
`define ID_EX_MemRead 57
`define ID_EX_MemtoReg 59:58
`define ID_EX_RegWrite 60
`define ID_EX_pc 76:61
`define ID_EX_rsData 92:77
`define ID_EX_rtData 108:93
`define ID_EX_imm 124:109
`define ID_EX_rs 126:125
`define ID_EX_rt 128:127
`define ID_EX_rd 130:129
`define ID_EX_incNum 131
`define ID_EX_isLink 132
`define EX_MEM_MemWrite 133
`define EX_MEM_MemRead 134
`define EX_MEM_MemtoReg 136:135
`define EX_MEM_RegWrite 137
`define EX_MEM_pc 153:138
`define EX_MEM_ALUOut 169:154
`define EX_MEM_MemWriteData 185:170
`define EX_MEM_rd 187:186
`define EX_MEM_isLink 188
`define MEM_WB_MemtoReg 190:189
`define MEM_WB_RegWrite 191
`define MEM_WB_pc 207:192
`define MEM_WB_DMemOut 223:208
`define MEM_WB_ALUOut 239:224
`define MEM_WB_rd 241:240

`define ID_EX_controlWB 60:58
`define ID_EX_controlMEM 57:56
`define ID_EX_controlEX 55:49
`define EX_MEM_controlWB 137:135
`define EX_MEM_controlMEM 134:133
`define MEM_WB_controlWB 191:189
`define ID_EX_controlBus 60:49

`define IF_ID_opcode 47:44
`define IF_ID_rs 43:42
`define IF_ID_rt 41:40
`define IF_ID_rd 39:38
`define IF_ID_funct 37:32
`define IF_ID_target 43:32
`define IF_ID_imm 39:32

`define LATCH_SIZE 242
`define CTRL_BUS_SIZE 12