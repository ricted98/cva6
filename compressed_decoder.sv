////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2017 ETH Zurich, University of Bologna                       //
// All rights reserved.                                                       //
//                                                                            //
// This code is under development and not yet released to the public.         //
// Until it is released, the code is under the copyright of ETH Zurich        //
// and the University of Bologna, and may contain unpublished work.           //
// Any reuse/redistribution should only be under explicit permission.         //
//                                                                            //
// Bug fixes and contributions will eventually be released under the          //
// SolderPad open hardware license and under the copyright of ETH Zurich      //
// and the University of Bologna.                                             //
//                                                                            //
// Engineer:       Sven Stucki - svstucki@student.ethz.ch                     //
//                                                                            //
// Design Name:    Compressed instruction decoder                             //
// Project Name:   zero-riscy                                                 //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    Decodes RISC-V compressed instructions into their RV32     //
//                 equivalent. This module is fully combinatorial.            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


import ariane_pkg::*;

module compressed_decoder
(
  input  logic [31:0] instr_i,
  output logic [31:0] instr_o,
  output logic        is_compressed_o,
  output logic        illegal_instr_o
);

  // -------------------
  // Compressed Decoder
  // -------------------

  always_comb
  begin
    illegal_instr_o = 1'b0;
    instr_o         = '0;

    unique case (instr_i[1:0])
      // C0
      2'b00: begin
        unique case (instr_i[15:13])
          3'b000: begin
            // c.addi4spn -> addi rd', x2, imm
            instr_o = {2'b0, instr_i[10:7], instr_i[12:11], instr_i[5], instr_i[6], 2'b00, 5'h02, 3'b000, 2'b01, instr_i[4:2], OPCODE_OPIMM};
            if (instr_i[12:5] == 8'b0)  illegal_instr_o = 1'b1;
          end

          3'b010: begin
            // c.lw -> lw rd', imm(rs1')
            instr_o = {5'b0, instr_i[5], instr_i[12:10], instr_i[6], 2'b00, 2'b01, instr_i[9:7], 3'b010, 2'b01, instr_i[4:2], OPCODE_LOAD};
          end

          3'b011: begin
            // c.ld -> ld rd', imm(rs1')
            // | imm[11:0] | rs1 | funct3 | rd | opcode |
            instr_o = {4'b0, instr_i[6:5], instr_i[12:10], 3'b000, 2'b01, instr_i[9:7], 3'b011, 2'b01, instr_i[4:2], OPCODE_LOAD};
          end

          3'b110: begin
            // c.sw -> sw rs2', imm(rs1')
            instr_o = {5'b0, instr_i[5], instr_i[12], 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b010, instr_i[11:10], instr_i[6], 2'b00, OPCODE_STORE};
          end

          3'b111: begin
            // c.sd -> sd rs2', imm(rs1')
            instr_o = {4'b0, instr_i[6:5], instr_i[12], 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b011, instr_i[11:10], 3'b000, OPCODE_STORE};
          end

          default: begin
            illegal_instr_o = 1'b1;
          end
        endcase
      end

      // C1
      2'b01: begin
        unique case (instr_i[15:13])
          3'b000: begin
            // c.addi -> addi rd, rd, nzimm
            // c.nop -> addi 0, 0, 0
            instr_o = {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2], instr_i[11:7], 3'b0, instr_i[11:7], OPCODE_OPIMM};
          end

          // c.addiw -> addiw rd, rd, nzimm for RV64
          3'b001: begin
            if (instr_i[11:7] != 5'h0) // only valid if the destination is not r0
              instr_o = {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2], instr_i[11:7], 3'b0, instr_i[11:7], OPCODE_OPIMM32};
            else
              illegal_instr_o = 1'b1;
          end

          3'b101: begin
            // 101: c.j   -> jal x0, imm
            instr_o = {instr_i[12], instr_i[8], instr_i[10:9], instr_i[6], instr_i[7], instr_i[2], instr_i[11], instr_i[5:3], {9 {instr_i[12]}}, 4'b0, ~instr_i[15], OPCODE_JAL};
          end

          3'b010: begin
            // c.li -> addi rd, x0, nzimm
            instr_o = {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2], 5'b0, 3'b0, instr_i[11:7], OPCODE_OPIMM};
            if (instr_i[11:7] == 5'b0)  illegal_instr_o = 1'b1;
          end

          3'b011: begin
            // c.lui -> lui rd, imm
            instr_o = {{15 {instr_i[12]}}, instr_i[6:2], instr_i[11:7], OPCODE_LUI};

            if (instr_i[11:7] == 5'h02) begin
              // c.addi16sp -> addi x2, x2, nzimm
              instr_o = {{3 {instr_i[12]}}, instr_i[4:3], instr_i[5], instr_i[2], instr_i[6], 4'b0, 5'h02, 3'b000, 5'h02, OPCODE_OPIMM};
            end else if (instr_i[11:7] == 5'b0) begin
              illegal_instr_o = 1'b1;
            end

            if ({instr_i[12], instr_i[6:2]} == 6'b0) illegal_instr_o = 1'b1;
          end

          3'b100: begin
            unique case (instr_i[11:10])
              2'b00,
              2'b01: begin
                // 00: c.srli -> srli rd, rd, shamt
                // 01: c.srai -> srai rd, rd, shamt
                instr_o = {1'b0, instr_i[10], 5'b0, instr_i[6:2], 2'b01, instr_i[9:7], 3'b101, 2'b01, instr_i[9:7], OPCODE_OPIMM};
                if (instr_i[6:2] == 5'b0) illegal_instr_o = 1'b1;
              end

              2'b10: begin
                // c.andi -> andi rd, rd, imm
                instr_o = {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2], 2'b01, instr_i[9:7], 3'b111, 2'b01, instr_i[9:7], OPCODE_OPIMM};
              end

              2'b11: begin
                unique case ({instr_i[12], instr_i[6:5]})
                  3'b000: begin
                    // c.sub -> sub rd', rd', rs2'
                    instr_o = {2'b01, 5'b0, 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b000, 2'b01, instr_i[9:7], OPCODE_OP};
                  end

                  3'b001: begin
                    // c.xor -> xor rd', rd', rs2'
                    instr_o = {7'b0, 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b100, 2'b01, instr_i[9:7], OPCODE_OP};
                  end

                  3'b010: begin
                    // c.or  -> or  rd', rd', rs2'
                    instr_o = {7'b0, 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b110, 2'b01, instr_i[9:7], OPCODE_OP};
                  end

                  3'b011: begin
                    // c.and -> and rd', rd', rs2'
                    instr_o = {7'b0, 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b111, 2'b01, instr_i[9:7], OPCODE_OP};
                  end

                  3'b100: begin
                  // c.subw -> subw rd', rd', rs2'
                    instr_o = {2'b01, 5'b0, 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b000, 2'b01, instr_i[9:7], OPCODE_OP32};
                  end
                  3'b101: begin
                  // c.addw -> addw rd', rd', rs2'
                    instr_o = {2'b00, 5'b0, 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b000, 2'b01, instr_i[9:7], OPCODE_OP32};
                  end

                  3'b110,
                  3'b111: begin
                    // 100: c.subw
                    // 101: c.addw
                    illegal_instr_o = 1'b1;
                  end
                endcase
              end
            endcase
          end

          3'b110, 3'b111: begin
            // 0: c.beqz -> beq rs1', x0, imm
            // 1: c.bnez -> bne rs1', x0, imm
            instr_o = {{4 {instr_i[12]}}, instr_i[6:5], instr_i[2], 5'b0, 2'b01, instr_i[9:7], 2'b00, instr_i[13], instr_i[11:10], instr_i[4:3], instr_i[12], OPCODE_BRANCH};
          end

          default: begin
            illegal_instr_o = 1'b1;
          end
        endcase
      end

      // C2
      2'b10: begin
        unique case (instr_i[15:13])
          3'b000: begin
            // c.slli -> slli rd, rd, shamt
            instr_o = {7'b0, instr_i[6:2], instr_i[11:7], 3'b001, instr_i[11:7], OPCODE_OPIMM};
            if (instr_i[11:7] == 5'b0)  illegal_instr_o = 1'b1; // register not x0
            if (instr_i[6:2]  == 5'b0)  illegal_instr_o = 1'b1; // shift amount must be non zero
          end

          3'b010: begin
            // c.lwsp -> lw rd, imm(x2)
            instr_o = {4'b0, instr_i[3:2], instr_i[12], instr_i[6:4], 2'b00, 5'h02, 3'b010, instr_i[11:7], OPCODE_LOAD};
            if (instr_i[11:7] == 5'b0)  illegal_instr_o = 1'b1;
          end

          3'b011: begin
            // c.ldsp -> ld rd, imm(x2)
            instr_o = {3'b0, instr_i[4:2], instr_i[12], instr_i[6:5], 3'b000, 5'h02, 3'b011, instr_i[11:7], OPCODE_LOAD};
            if (instr_i[11:7] == 5'b0)  illegal_instr_o = 1'b1;
          end

          3'b100: begin
            if (instr_i[12] == 1'b0) begin
              // c.mv -> add rd/rs1, x0, rs2
              instr_o = {7'b0, instr_i[6:2], 5'b0, 3'b0, instr_i[11:7], OPCODE_OP};

              if (instr_i[6:2] == 5'b0) begin
                // c.jr -> jalr x0, rd/rs1, 0
                instr_o = {12'b0, instr_i[11:7], 3'b0, 5'b0, OPCODE_JALR};
              end
            end else begin
              // c.add -> add rd, rd, rs2
              instr_o = {7'b0, instr_i[6:2], instr_i[11:7], 3'b0, instr_i[11:7], OPCODE_OP};

              if (instr_i[11:7] == 5'b0) begin
                // c.ebreak -> ebreak
                instr_o = {32'h00_10_00_73};
                if (instr_i[6:2] != 5'b0)
                  illegal_instr_o = 1'b1;
              end else if (instr_i[6:2] == 5'b0) begin
                // c.jalr -> jalr x1, rs1, 0
                instr_o = {12'b0, instr_i[11:7], 3'b000, 5'b00001, OPCODE_JALR};
              end
            end
          end

          3'b110: begin
            // c.swsp -> sw rs2, imm(x2)
            instr_o = {4'b0, instr_i[8:7], instr_i[12], instr_i[6:2], 5'h02, 3'b010, instr_i[11:9], 2'b00, OPCODE_STORE};
          end

          3'b111: begin
            // c.sdsp -> sd rs2, imm(x2)
            instr_o = {3'b0, instr_i[9:7], instr_i[12], instr_i[6:2], 5'h02, 3'b011, instr_i[11:10], 3'b000, OPCODE_STORE};
          end

          default: begin
            illegal_instr_o = 1'b1;
          end
        endcase
      end

      default: begin
        // 32 bit (or more) instruction
        instr_o = instr_i;
      end
    endcase
  end

  assign is_compressed_o = (instr_i[1:0] != 2'b11);

endmodule