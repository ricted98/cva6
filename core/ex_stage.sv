
// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Florian Zaruba, ETH Zurich
// Date: 19.04.2017
// Description: Instantiation of all functional units residing in the execute stage


module ex_stage
  import ariane_pkg::*;
#(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty,
    parameter type bp_resolve_t = logic,
    parameter type branchpredict_sbe_t = logic,
    parameter type dcache_req_i_t = logic,
    parameter type dcache_req_o_t = logic,
    parameter type exception_t = logic,
    parameter type fu_data_t = logic,
    parameter type icache_areq_t = logic,
    parameter type icache_arsp_t = logic,
    parameter type icache_dreq_t = logic,
    parameter type icache_drsp_t = logic,
    parameter type lsu_ctrl_t = logic,
    parameter type x_result_t = logic,
    parameter type acc_mmu_req_t = logic,
    parameter type acc_mmu_resp_t = logic
) (
    // Subsystem Clock - SUBSYSTEM
    input logic clk_i,
    // Asynchronous reset active low - SUBSYSTEM
    input logic rst_ni,
    // Fetch flush request - CONTROLLER
    input logic flush_i,
    // Debug mode is enabled - CSR_REGFILE
    input logic debug_mode_i,
    // rs1 forwarding - ISSUE_STAGE
    input logic [CVA6Cfg.NrIssuePorts-1:0][CVA6Cfg.VLEN-1:0] rs1_forwarding_i,
    // rs2 forwarding - ISSUE_STAGE
    input logic [CVA6Cfg.NrIssuePorts-1:0][CVA6Cfg.VLEN-1:0] rs2_forwarding_i,
    // FU data useful to execute instruction - ISSUE_STAGE
    input fu_data_t [CVA6Cfg.NrIssuePorts-1:0] fu_data_i,
    // PC of the current instruction - ISSUE_STAGE
    input logic [CVA6Cfg.VLEN-1:0] pc_i,
    // Is_zcmt instruction - ISSUE_STAGE
    input logic is_zcmt_i,
    // Report whether instruction is compressed - ISSUE_STAGE
    input logic is_compressed_instr_i,
    // Report instruction encoding - ISSUE_STAGE
    input logic [CVA6Cfg.NrIssuePorts-1:0][31:0] tinst_i,
    // Fixed Latency Unit result - ISSUE_STAGE
    output logic [CVA6Cfg.XLEN-1:0] flu_result_o,
    // ID of the scoreboard entry at which a=to write back - ISSUE_STAGE
    output logic [CVA6Cfg.TRANS_ID_BITS-1:0] flu_trans_id_o,
    // Fixed Latency Unit exception - ISSUE_STAGE
    output exception_t flu_exception_o,
    // FLU is ready - ISSUE_STAGE
    output logic flu_ready_o,
    // FLU result is valid - ISSUE_STAGE
    output logic flu_valid_o,
    // ALU instruction is valid - ISSUE_STAGE
    input logic [CVA6Cfg.NrIssuePorts-1:0] alu_valid_i,
    // AES instruction is valid - ISSUE_STAGE
    input logic [CVA6Cfg.NrIssuePorts-1:0] aes_valid_i,
    // Branch unit instruction is valid - ISSUE_STAGE
    input logic [CVA6Cfg.NrIssuePorts-1:0] branch_valid_i,
    // Information of branch prediction - ISSUE_STAGE
    input branchpredict_sbe_t branch_predict_i,
    // The branch engine uses the write back from the ALU - several_modules
    output bp_resolve_t resolved_branch_o,
    // Signaling that we resolved the branch - ISSUE_STAGE
    output logic resolve_branch_o,
    // CSR instruction is valid - ISSUE_STAGE
    input logic [CVA6Cfg.NrIssuePorts-1:0] csr_valid_i,
    // CSR address to write - COMMIT_STAGE
    output logic [11:0] csr_addr_o,
    // CSR commit - COMMIT_STAGE
    input logic csr_commit_i,
    // MULT instruction is valid - ISSUE_STAGE
    input logic [CVA6Cfg.NrIssuePorts-1:0] mult_valid_i,
    // LSU is ready - ISSUE_STAGE
    output logic lsu_ready_o,
    // LSU instruction is valid - ISSUE_STAGE
    input logic [CVA6Cfg.NrIssuePorts-1:0] lsu_valid_i,
    // Load result is valid - ISSUE_STAGE
    output logic load_valid_o,
    // Load result valid - ISSUE_STAGE
    output logic [CVA6Cfg.XLEN-1:0] load_result_o,
    // Load instruction ID - ISSUE_STAGE
    output logic [CVA6Cfg.TRANS_ID_BITS-1:0] load_trans_id_o,
    // Exception generated by load instruction - ISSUE_STAGE
    output exception_t load_exception_o,
    // Store result is valid - ISSUe_STAGE
    output logic store_valid_o,
    // Store result - ISSUE_STAGE
    output logic [CVA6Cfg.XLEN-1:0] store_result_o,
    // Store instruction ID - ISSUE_STAGE
    output logic [CVA6Cfg.TRANS_ID_BITS-1:0] store_trans_id_o,
    // Exception generated by store instruction - ISSUE_STAGE
    output exception_t store_exception_o,
    // LSU commit - COMMIT_STAGE
    input logic lsu_commit_i,
    // Commit queue ready to accept another commit request - COMMIT_STAGE
    output logic lsu_commit_ready_o,
    // Commit transaction ID - COMMIT_STAGE
    input logic [CVA6Cfg.TRANS_ID_BITS-1:0] commit_tran_id_i,
    // TO_BE_COMPLETED - ACC_DISPATCHER
    input logic stall_st_pending_i,
    // TO_BE_COMPLETED - COMMIT_STAGE
    output logic no_st_pending_o,
    // Atomic result is valid - COMMIT_STAGE
    input logic amo_valid_commit_i,
    // FU is ready - ISSUE_STAGE
    output logic fpu_ready_o,
    // FPU instruction is ready - ISSUE_STAGE
    input logic [CVA6Cfg.NrIssuePorts-1:0] fpu_valid_i,
    // FPU format - ISSUE_STAGE
    input logic [1:0] fpu_fmt_i,
    // FPU rm - ISSUE_STAGE
    input logic [2:0] fpu_rm_i,
    // FPU frm - ISSUE_STAGE
    input logic [2:0] fpu_frm_i,
    // FPU precision control - CSR_REGFILE
    input logic [6:0] fpu_prec_i,
    // FPU transaction ID - ISSUE_STAGE
    output logic [CVA6Cfg.TRANS_ID_BITS-1:0] fpu_trans_id_o,
    // FPU result - ISSUE_STAGE
    output logic [CVA6Cfg.XLEN-1:0] fpu_result_o,
    // FPU valid - ISSUE_STAGE
    output logic fpu_valid_o,
    // FPU exception - ISSUE_STAGE
    output exception_t fpu_exception_o,
    // ALU2 instruction is valid - ISSUE_STAGE
    input logic [CVA6Cfg.NrIssuePorts-1:0] alu2_valid_i,
    // CVXIF instruction is valid - ISSUE_STAGE
    input logic [CVA6Cfg.NrIssuePorts-1:0] x_valid_i,
    // CVXIF is ready - ISSUE_STAGE
    output logic x_ready_o,
    // CVXIF undecoded instruction
    input logic [31:0] x_off_instr_i,
    // CVXIF transaction ID - ISSUE_STAGE
    output logic [CVA6Cfg.TRANS_ID_BITS-1:0] x_trans_id_o,
    // CVXIF exception - ISSUE_STAGE
    output exception_t x_exception_o,
    // CVXIF result - ISSUE_STAGE
    output logic [CVA6Cfg.XLEN-1:0] x_result_o,
    // CVXIF result valid - ISSUE_STAGE
    output logic x_valid_o,
    // CVXIF write enable - ISSUE_STAGE
    output logic x_we_o,
    // CVXIF destination register - ISSUE_STAGE
    output logic [4:0] x_rd_o,
    // CVXIF Result interface - SUBSYSTEM
    input logic x_result_valid_i,
    input x_result_t x_result_i,
    output logic x_result_ready_o,
    // CVXIF Issue transaction rejected -> illegal instruction - ISSUE_STAGE
    input logic x_transaction_rejected_i,
    // accelerate port result is valid - ACC_DISPATCHER
    input logic acc_valid_i,
    // Accelerator MMU access
    input acc_mmu_req_t acc_mmu_req_i,
    output acc_mmu_resp_t acc_mmu_resp_o,
    // Enable virtual memory translation - CSR_REGFILE
    input logic enable_translation_i,
    // Enable G-Stage memory translation - CSR_REGFILE
    input logic enable_g_translation_i,
    // Enable virtual memory translation for load/stores - CSR_REGFILE
    input logic en_ld_st_translation_i,
    // Enable G-Stage memory translation for load/stores - CSR_REGFILE
    input logic en_ld_st_g_translation_i,
    // Flush TLB - CONTROLLER
    input logic flush_tlb_i,
    input logic flush_tlb_vvma_i,
    input logic flush_tlb_gvma_i,
    // Privilege mode - CSR_REGFILE
    input riscv::priv_lvl_t priv_lvl_i,
    // Virtualization mode - CSR_REGFILE
    input logic v_i,
    // Privilege level at which load and stores should happen - CSR_REGFILE
    input riscv::priv_lvl_t ld_st_priv_lvl_i,
    // Virtualization mode at which load and stores should happen - CSR_REGFILE
    input logic ld_st_v_i,
    // Instruction is hypervisor load/store - CSR_REGFILE
    output logic csr_hs_ld_st_inst_o,
    // Supervisor user memory - CSR_REGFILE
    input logic sum_i,
    // Virtual Supervisor user memory - CSR_REGFILE
    input logic vs_sum_i,
    // Make executable readable - CSR_REGFILE
    input logic mxr_i,
    // Make executable readable Virtual Supervisor - CSR_REGFILE
    input logic vmxr_i,
    // TO_BE_COMPLETED - CSR_REGFILE
    input logic [CVA6Cfg.PPNW-1:0] satp_ppn_i,
    // TO_BE_COMPLETED - CSR_REGFILE
    input logic [CVA6Cfg.ASID_WIDTH-1:0] asid_i,
    // TO_BE_COMPLETED - CSR_REGFILE
    input logic [CVA6Cfg.PPNW-1:0] vsatp_ppn_i,
    // TO_BE_COMPLETED - CSR_REGFILE
    input logic [CVA6Cfg.ASID_WIDTH-1:0] vs_asid_i,
    // TO_BE_COMPLETED - CSR_REGFILE
    input logic [CVA6Cfg.PPNW-1:0] hgatp_ppn_i,
    // TO_BE_COMPLETED - CSR_REGFILE
    input logic [CVA6Cfg.VMID_WIDTH-1:0] vmid_i,
    // icache translation response - CACHE
    input icache_arsp_t icache_areq_i,
    // icache translation request - CACHE
    output icache_areq_t icache_areq_o,
    // Data cache request output - CACHE
    input dcache_req_o_t [2:0] dcache_req_ports_i,
    // Data cache request input - CACHE
    output dcache_req_i_t [2:0] dcache_req_ports_o,
    // Write buffer is empty - CACHE
    input logic dcache_wbuffer_empty_i,
    // TO_BE_COMPLETED - CACHE
    input logic dcache_wbuffer_not_ni_i,
    // AMO request - CACHE
    output amo_req_t amo_req_o,
    // AMO response - CACHE
    input amo_resp_t amo_resp_i,
    // To count the instruction TLB misses - PERF_COUNTERS
    output logic itlb_miss_o,
    // To count the data TLB misses - PERF_COUNTERS
    output logic dtlb_miss_o,
    // Report the PMP configuration - CSR_REGFILE
    input riscv::pmpcfg_t [avoid_neg(CVA6Cfg.NrPMPEntries-1):0] pmpcfg_i,
    // Report the PMP addresses - CSR_REGFILE
    input logic [avoid_neg(CVA6Cfg.NrPMPEntries-1):0][CVA6Cfg.PLEN-3:0] pmpaddr_i,
    // Information dedicated to RVFI - RVFI
    output lsu_ctrl_t rvfi_lsu_ctrl_o,
    // Information dedicated to RVFI - RVFI
    output [CVA6Cfg.PLEN-1:0] rvfi_mem_paddr_o,
    // Original instruction AES bits
    input logic [5:0] orig_instr_aes_i
);

  // -------------------------
  // Fixed Latency Units
  // -------------------------
  // all fixed latency units share a single issue port and a sing write
  // port into the scoreboard. At the moment those are:
  // 1. ALU - all operations are single cycle
  // 2. Branch unit: operation is single cycle, the ALU is needed
  //    for comparison
  // 3. CSR: This is a small buffer which saves the address of the CSR.
  //    The value is then re-fetched once the instruction retires. The buffer
  //    is only a single entry deep, hence this operation will block all
  //    other operations once this buffer is full. This should not be a major
  //    concern though as CSRs are infrequent.
  // 4. Multiplier/Divider: The multiplier has a fixed latency of 1 cycle.
  //                        The issue logic will take care of not issuing
  //                        another instruction if it will collide on the
  //                        output port. Divisions are arbitrary in length
  //                        they will simply block the issue of all other
  //                        instructions.


  logic current_instruction_is_sfence_vma;
  logic current_instruction_is_hfence_vvma;
  logic current_instruction_is_hfence_gvma;
  // These two register store the rs1 and rs2 parameters in case of `SFENCE_VMA`
  // instruction to be used for TLB flush in the next clock cycle.
  logic [CVA6Cfg.VMID_WIDTH-1:0] vmid_to_be_flushed;
  logic [CVA6Cfg.ASID_WIDTH-1:0] asid_to_be_flushed;
  logic [CVA6Cfg.VLEN-1:0] vaddr_to_be_flushed;
  logic [CVA6Cfg.GPLEN-1:0] gpaddr_to_be_flushed;

  // from ALU to branch unit
  logic alu_branch_res;  // branch comparison result
  logic [CVA6Cfg.XLEN-1:0] alu_result, csr_result, mult_result, aes_result;
  logic [CVA6Cfg.VLEN-1:0] branch_result;
  logic csr_ready, mult_ready;
  logic [CVA6Cfg.TRANS_ID_BITS-1:0] mult_trans_id;
  logic mult_valid;

  logic [CVA6Cfg.NrIssuePorts-1:0] one_cycle_select;
  assign one_cycle_select = alu_valid_i | branch_valid_i | csr_valid_i | aes_valid_i;

  fu_data_t one_cycle_data;
  logic [CVA6Cfg.VLEN-1:0] rs1_forwarding;
  logic [CVA6Cfg.VLEN-1:0] rs2_forwarding;
  always_comb begin
    // data silence operation
    one_cycle_data = one_cycle_select[0] ? fu_data_i[0] : '0;
    rs1_forwarding = rs1_forwarding_i[0];
    rs2_forwarding = rs2_forwarding_i[0];

    if (CVA6Cfg.SuperscalarEn) begin
      if (one_cycle_select[1]) begin
        one_cycle_data = fu_data_i[1];
        rs1_forwarding = rs1_forwarding_i[1];
        rs2_forwarding = rs2_forwarding_i[1];
      end
    end
  end

  // 1. ALU (combinatorial)
  alu #(
      .CVA6Cfg  (CVA6Cfg),
      .HasBranch(1'b1),
      .fu_data_t(fu_data_t)
  ) alu_i (
      .clk_i,
      .rst_ni,
      .fu_data_i       (one_cycle_data),
      .result_o        (alu_result),
      .alu_branch_res_o(alu_branch_res)
  );

  // 2. Branch Unit (combinatorial)
  // we don't silence the branch unit as this is already critical and we do
  // not want to add another layer of logic
  branch_unit #(
      .CVA6Cfg(CVA6Cfg),
      .bp_resolve_t(bp_resolve_t),
      .branchpredict_sbe_t(branchpredict_sbe_t),
      .exception_t(exception_t),
      .fu_data_t(fu_data_t)
  ) branch_unit_i (
      .clk_i,
      .rst_ni,
      .v_i,
      .debug_mode_i,
      .fu_data_i         (one_cycle_data),
      .pc_i,
      .is_zcmt_i,
      .is_compressed_instr_i,
      .branch_valid_i    (|branch_valid_i),
      .branch_comp_res_i (alu_branch_res),
      .branch_result_o   (branch_result),
      .branch_predict_i,
      .resolved_branch_o,
      .resolve_branch_o,
      .branch_exception_o(flu_exception_o)
  );

  // 3. CSR (sequential)
  csr_buffer #(
      .CVA6Cfg  (CVA6Cfg),
      .fu_data_t(fu_data_t)
  ) csr_buffer_i (
      .clk_i,
      .rst_ni,
      .flush_i,
      .fu_data_i   (one_cycle_data),
      .csr_valid_i (|csr_valid_i),
      .csr_ready_o (csr_ready),
      .csr_result_o(csr_result),
      .csr_commit_i,
      .csr_addr_o
  );

  assign flu_valid_o = |one_cycle_select | mult_valid;

  // result MUX
  always_comb begin
    // Branch result as default case
    flu_result_o   = {{CVA6Cfg.XLEN - CVA6Cfg.VLEN{1'b0}}, branch_result};
    flu_trans_id_o = one_cycle_data.trans_id;
    // ALU result
    if (|alu_valid_i) begin
      flu_result_o = alu_result;
      // CSR result
    end else if (|csr_valid_i) begin
      flu_result_o = csr_result;
    end else if (mult_valid) begin
      flu_result_o   = mult_result;
      flu_trans_id_o = mult_trans_id;
    end else if (|aes_valid_i) begin
      flu_result_o = aes_result;
    end
  end

  // ready flags for FLU
  always_comb begin
    flu_ready_o = csr_ready & mult_ready;
  end

  // 4. Multiplication (Sequential)
  fu_data_t mult_data;
  // input silencing of multiplier
  always_comb begin
    mult_data = mult_valid_i[0] ? fu_data_i[0] : '0;
    if (CVA6Cfg.SuperscalarEn) begin
      if (mult_valid_i[1]) begin
        mult_data = fu_data_i[1];
      end
    end
  end

  mult #(
      .CVA6Cfg  (CVA6Cfg),
      .fu_data_t(fu_data_t)
  ) i_mult (
      .clk_i,
      .rst_ni,
      .flush_i,
      .mult_valid_i   (|mult_valid_i),
      .fu_data_i      (mult_data),
      .result_o       (mult_result),
      .mult_valid_o   (mult_valid),
      .mult_ready_o   (mult_ready),
      .mult_trans_id_o(mult_trans_id)
  );

  // ----------------
  // FPU
  // ----------------
  logic fpu_valid;
  logic [CVA6Cfg.TRANS_ID_BITS-1:0] fpu_trans_id;
  logic [CVA6Cfg.XLEN-1:0] fpu_result;
  logic alu2_valid;
  logic [CVA6Cfg.XLEN-1:0] alu2_result;

  generate
    if (CVA6Cfg.FpPresent) begin : fpu_gen
      fu_data_t fpu_data;
      always_comb begin
        fpu_data = fpu_valid_i[0] ? fu_data_i[0] : '0;
        if (CVA6Cfg.SuperscalarEn) begin
          if (fpu_valid_i[1]) begin
            fpu_data = fu_data_i[1];
          end
        end
      end

      fpu_wrap #(
          .CVA6Cfg(CVA6Cfg),
          .exception_t(exception_t),
          .fu_data_t(fu_data_t)
      ) fpu_i (
          .clk_i,
          .rst_ni,
          .flush_i,
          .fpu_valid_i(|fpu_valid_i),
          .fpu_ready_o,
          .fu_data_i(fpu_data),
          .fpu_fmt_i,
          .fpu_rm_i,
          .fpu_frm_i,
          .fpu_prec_i,
          .fpu_trans_id_o(fpu_trans_id),
          .result_o(fpu_result),
          .fpu_valid_o(fpu_valid),
          .fpu_exception_o
      );
    end else begin : no_fpu_gen
      assign fpu_ready_o     = '0;
      assign fpu_trans_id    = '0;
      assign fpu_result      = '0;
      assign fpu_valid       = '0;
      assign fpu_exception_o = '0;
    end
  endgenerate

  // ----------------
  // ALU2
  // ----------------
  fu_data_t alu2_data;
  if (CVA6Cfg.SuperscalarEn) begin : alu2_gen
    always_comb begin
      alu2_data = alu2_valid_i[0] ? fu_data_i[0] : '0;
      if (alu2_valid_i[1]) begin
        alu2_data = fu_data_i[1];
      end
    end

    alu #(
        .CVA6Cfg  (CVA6Cfg),
        .HasBranch(1'b0),
        .fu_data_t(fu_data_t)
    ) alu2_i (
        .clk_i,
        .rst_ni,
        .fu_data_i       (alu2_data),
        .result_o        (alu2_result),
        .alu_branch_res_o(  /* this ALU does not handle branching */)
    );
  end else begin
    assign alu2_data   = '0;
    assign alu2_result = '0;
  end

  // result MUX
  // This is really explicit so that synthesis tools can elide unused signals
  if (CVA6Cfg.SuperscalarEn) begin
    if (CVA6Cfg.FpPresent) begin
      assign fpu_valid_o    = fpu_valid || |alu2_valid_i;
      assign fpu_result_o   = fpu_valid ? fpu_result   : alu2_result;
      assign fpu_trans_id_o = fpu_valid ? fpu_trans_id : alu2_data.trans_id;
    end else begin
      assign fpu_valid_o    = |alu2_valid_i;
      assign fpu_result_o   = alu2_result;
      assign fpu_trans_id_o = alu2_data.trans_id;
    end
  end else begin
    if (CVA6Cfg.FpPresent) begin
      assign fpu_valid_o    = fpu_valid;
      assign fpu_result_o   = fpu_result;
      assign fpu_trans_id_o = fpu_trans_id;
    end else begin
      assign fpu_valid_o    = '0;
      assign fpu_result_o   = '0;
      assign fpu_trans_id_o = '0;
    end
  end

  // ----------------
  // Load-Store Unit
  // ----------------
  fu_data_t lsu_data;
  logic [31:0] lsu_tinst;
  always_comb begin
    lsu_data  = lsu_valid_i[0] ? fu_data_i[0] : '0;
    lsu_tinst = tinst_i[0];

    if (CVA6Cfg.SuperscalarEn) begin
      if (lsu_valid_i[1]) begin
        lsu_data  = fu_data_i[1];
        lsu_tinst = tinst_i[1];
      end
    end
  end

  load_store_unit #(
      .CVA6Cfg   (CVA6Cfg),
      .dcache_req_i_t(dcache_req_i_t),
      .dcache_req_o_t(dcache_req_o_t),
      .exception_t(exception_t),
      .fu_data_t (fu_data_t),
      .icache_areq_t(icache_areq_t),
      .icache_arsp_t(icache_arsp_t),
      .icache_dreq_t(icache_dreq_t),
      .icache_drsp_t(icache_drsp_t),
      .lsu_ctrl_t(lsu_ctrl_t),
      .acc_mmu_req_t(acc_mmu_req_t),
      .acc_mmu_resp_t(acc_mmu_resp_t)
  ) lsu_i (
      .clk_i,
      .rst_ni,
      .flush_i,
      .stall_st_pending_i,
      .no_st_pending_o,
      .fu_data_i             (lsu_data),
      .lsu_ready_o,
      .lsu_valid_i           (|lsu_valid_i),
      .load_trans_id_o,
      .load_result_o,
      .load_valid_o,
      .load_exception_o,
      .store_trans_id_o,
      .store_result_o,
      .store_valid_o,
      .store_exception_o,
      .commit_i              (lsu_commit_i),
      .commit_ready_o        (lsu_commit_ready_o),
      .commit_tran_id_i,
      .enable_translation_i,
      .enable_g_translation_i,
      .en_ld_st_translation_i,
      .en_ld_st_g_translation_i,
      .acc_mmu_req_i,
      .acc_mmu_resp_o,
      .icache_areq_i,
      .icache_areq_o,
      .priv_lvl_i,
      .v_i,
      .ld_st_priv_lvl_i,
      .ld_st_v_i,
      .csr_hs_ld_st_inst_o,
      .sum_i,
      .vs_sum_i,
      .mxr_i,
      .vmxr_i,
      .satp_ppn_i,
      .vsatp_ppn_i,
      .hgatp_ppn_i,
      .asid_i,
      .vs_asid_i,
      .asid_to_be_flushed_i  (asid_to_be_flushed),
      .vmid_i,
      .vmid_to_be_flushed_i  (vmid_to_be_flushed),
      .vaddr_to_be_flushed_i (vaddr_to_be_flushed),
      .gpaddr_to_be_flushed_i(gpaddr_to_be_flushed),
      .flush_tlb_i,
      .flush_tlb_vvma_i,
      .flush_tlb_gvma_i,
      .itlb_miss_o,
      .dtlb_miss_o,
      .dcache_req_ports_i,
      .dcache_req_ports_o,
      .dcache_wbuffer_empty_i,
      .dcache_wbuffer_not_ni_i,
      .amo_valid_commit_i,
      .amo_req_o,
      .amo_resp_i,
      .tinst_i               (lsu_tinst),
      .pmpcfg_i,
      .pmpaddr_i,
      .rvfi_lsu_ctrl_o,
      .rvfi_mem_paddr_o
  );

  if (CVA6Cfg.CvxifEn) begin : gen_cvxif
    fu_data_t cvxif_data;
    always_comb begin
      cvxif_data = x_valid_i[0] ? fu_data_i[0] : '0;
      if (CVA6Cfg.SuperscalarEn) begin
        if (x_valid_i[1]) begin
          cvxif_data = fu_data_i[1];
        end
      end
    end

    cvxif_fu #(
        .CVA6Cfg(CVA6Cfg),
        .exception_t(exception_t),
        .x_result_t(x_result_t)
    ) cvxif_fu_i (
        .clk_i,
        .rst_ni,
        .v_i,
        .x_valid_i(|x_valid_i),
        .x_trans_id_i(cvxif_data.trans_id),
        .x_illegal_i(x_transaction_rejected_i),
        .x_off_instr_i,
        .x_ready_o,
        .x_trans_id_o,
        .x_exception_o,
        .x_result_o,
        .x_valid_o,
        .x_we_o,
        .x_rd_o,
        .result_valid_i(x_result_valid_i),
        .result_i(x_result_i),
        .result_ready_o(x_result_ready_o)
    );
  end else begin : gen_no_cvxif
    assign x_result_ready_o = '0;
    assign x_trans_id_o     = '0;
    assign x_exception_o    = '0;
    assign x_result_o       = '0;
    assign x_valid_o        = '0;
  end

  if (CVA6Cfg.RVS) begin
    if (CVA6Cfg.RVH) begin
      always_ff @(posedge clk_i or negedge rst_ni) begin
        if (~rst_ni) begin
          current_instruction_is_sfence_vma  <= 1'b0;
          current_instruction_is_hfence_vvma <= 1'b0;
          current_instruction_is_hfence_gvma <= 1'b0;
        end else begin
          // TODO handle this with superscalar (issue only one instruction in this case?)
          if (flush_i) begin
            current_instruction_is_sfence_vma  <= 1'b0;
            current_instruction_is_hfence_vvma <= 1'b0;
            current_instruction_is_hfence_gvma <= 1'b0;
          end else if ((fu_data_i[0].operation == SFENCE_VMA && !v_i) && |csr_valid_i) begin
            current_instruction_is_sfence_vma <= 1'b1;
          end else if (((fu_data_i[0].operation == SFENCE_VMA && v_i) || fu_data_i[0].operation == HFENCE_VVMA) && |csr_valid_i) begin
            current_instruction_is_hfence_vvma <= 1'b1;
          end else if ((fu_data_i[0].operation == HFENCE_GVMA) && |csr_valid_i) begin
            current_instruction_is_hfence_gvma <= 1'b1;
          end
        end
      end
    end else begin
      assign current_instruction_is_hfence_vvma = 1'b0;
      assign current_instruction_is_hfence_gvma = 1'b0;
      always_ff @(posedge clk_i or negedge rst_ni) begin
        if (~rst_ni) begin
          current_instruction_is_sfence_vma <= 1'b0;
        end else begin
          if (flush_i) begin
            current_instruction_is_sfence_vma <= 1'b0;
          end else if (fu_data_i[0].operation == SFENCE_VMA && |csr_valid_i) begin
            current_instruction_is_sfence_vma <= 1'b1;
          end
        end
      end
    end
    if (CVA6Cfg.RVH) begin
      // This process stores the rs1 and rs2 parameters of a SFENCE_VMA instruction.
      always_ff @(posedge clk_i or negedge rst_ni) begin
        if (~rst_ni) begin
          vmid_to_be_flushed   <= '0;
          asid_to_be_flushed   <= '0;
          vaddr_to_be_flushed  <= '0;
          gpaddr_to_be_flushed <= '0;
          // if the current instruction in EX_STAGE is a sfence.vma, in the next cycle no writes will happen
        end else if ((~(current_instruction_is_sfence_vma || current_instruction_is_hfence_vvma || current_instruction_is_hfence_gvma)) && (~((fu_data_i[0].operation == SFENCE_VMA || fu_data_i[0].operation == HFENCE_VVMA || fu_data_i[0].operation == HFENCE_GVMA ) && |csr_valid_i))) begin
          vaddr_to_be_flushed  <= rs1_forwarding;
          gpaddr_to_be_flushed <= {2'b00, rs1_forwarding[CVA6Cfg.GPLEN-1:2]};
          asid_to_be_flushed   <= rs2_forwarding[CVA6Cfg.ASID_WIDTH-1:0];
          vmid_to_be_flushed   <= rs2_forwarding[CVA6Cfg.VMID_WIDTH-1:0];
        end
      end
    end else begin
      assign vmid_to_be_flushed   = '0;
      assign gpaddr_to_be_flushed = '0;
      // This process stores the rs1 and rs2 parameters of a SFENCE_VMA instruction.
      always_ff @(posedge clk_i or negedge rst_ni) begin
        if (~rst_ni) begin
          asid_to_be_flushed  <= '0;
          vaddr_to_be_flushed <= '0;
          // if the current instruction in EX_STAGE is a sfence.vma, in the next cycle no writes will happen
        end else if ((~current_instruction_is_sfence_vma) && (~((fu_data_i[0].operation == SFENCE_VMA) && |csr_valid_i))) begin
          vaddr_to_be_flushed <= rs1_forwarding;
          asid_to_be_flushed  <= rs2_forwarding[CVA6Cfg.ASID_WIDTH-1:0];
        end
      end
    end
  end else begin
    assign current_instruction_is_sfence_vma  = 1'b0;
    assign current_instruction_is_hfence_vvma = 1'b0;
    assign current_instruction_is_hfence_gvma = 1'b0;
    assign asid_to_be_flushed                 = '0;
    assign vaddr_to_be_flushed                = '0;
    assign vmid_to_be_flushed                 = '0;
    assign gpaddr_to_be_flushed               = '0;
  end

  // ----------------
  // Scalar Cryptography Unit
  // ----------------
  generate
    if (CVA6Cfg.ZKN) begin : aes_gen
      aes #(
          .CVA6Cfg  (CVA6Cfg),
          .fu_data_t(fu_data_t)
      ) aes_i (
          .clk_i,
          .rst_ni,
          .fu_data_i     (one_cycle_data),
          .result_o      (aes_result),
          .orig_instr_aes(orig_instr_aes_i)
      );
    end else begin : no_aes_gen
      assign aes_result = '0;
    end
  endgenerate

endmodule
