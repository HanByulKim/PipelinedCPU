
`define WORD_SIZE 16
module ALU(
    input [15:0] A,
    input [15:0] B,
    input wire [3:0] ALUCtrl,
    output reg [15:0] C,
    output reg [`WORD_SIZE-1:0] output_port,
    output reg isHLT
    );
    
    always @* begin
      isHLT=0;
      #10
      case(ALUCtrl)
          4'b0000: C = A + B;               // ADD or ADI or LWD or SWD
          4'b0001: begin                    // SUB, BEQ
            C = A+~B+1;          
            //Zero = (C == 0)? 1 : 0;
          end
          4'b0010: C = A & B;               // AND
          4'b0011: C = A | B;               // ORR or ORI
          4'b0100: C = ~A;                  // NOT
          4'b0101: C = ~A + 1;              // TCP
          4'b0110: C = A<<1;                // SHL
          4'b0111: C = A>>1;                // SHR
          4'b1000: C = {B,8'b00000000};     // LHI
          /*4'b1001: begin                    // BNE
              C = A - B; 
              //Zero = (C != 0)? 1 : 0;
          end
          4'b1010: begin                    // BGZ
              C = A; 
              //Zero = (C[15] == 0&& C!=0)? 1 : 0;
          end
          4'b1011: begin                    // BLZ
              C = A; 
              //Zero = (C[15] == 1)? 1 : 0;
          end
          4'b1100: C = A;                   // JPR or JRL*/
          4'b1101: begin
            output_port = A;
          end
          4'b1111: isHLT=1;                 // HLT
      endcase
    end
    
endmodule
