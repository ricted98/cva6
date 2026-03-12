// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Riccardo Tedeschi <riccardo.tedeschi6@unibo.it>
// Author: Stefan Odermatt   <soderma@student.ethz.ch>
// Author: Ivan Herger       <iherger@student.ethz.ch>

`include "rvfi_types.svh"
`include "cvxif_types.svh"
`include "hpdcache_typedef.svh"

module cva6_top
  import ariane_pkg::*;
#(
    // Top configurations
    parameter bit EnableDMR = 1'b0,
    parameter bit EnableHMR = 1'b0,

    localparam int unsigned NumPhysicalCores = EnableDMR ? 2 : 1,
    localparam int unsigned NumLogicalCores  = EnableHMR ? NumPhysicalCores : 1,
    // CVA6 config
    parameter config_pkg::cva6_cfg_t CVA6Cfg = build_config_pkg::build_config(
        cva6_config_pkg::cva6_cfg
    ),

    // RVFI PROBES
    parameter type rvfi_probes_instr_t = `RVFI_PROBES_INSTR_T(CVA6Cfg),
    parameter type rvfi_probes_csr_t = `RVFI_PROBES_CSR_T(CVA6Cfg),
    parameter type rvfi_probes_t = struct packed {
      rvfi_probes_csr_t   csr;
      rvfi_probes_instr_t instr;
    },

    // branchpredict scoreboard entry
    // this is the struct which we will inject into the pipeline to guide the various
    // units towards the correct branch decision and resolve
    localparam type branchpredict_sbe_t = struct packed {
      cf_t                     cf;               // type of control flow prediction
      logic [CVA6Cfg.VLEN-1:0] predict_address;  // target address at which to jump, or not
    },

    parameter type exception_t = struct packed {
      logic [CVA6Cfg.XLEN-1:0] cause;  // cause of exception
      logic [CVA6Cfg.XLEN-1:0] tval;  // additional information of causing exception (e.g.: instruction causing it),
      // address of LD/ST fault
      logic [CVA6Cfg.GPLEN-1:0] tval2;  // additional information when the causing exception in a guest exception
      logic [31:0] tinst;  // transformed instruction information
      logic gva;  // signals when a guest virtual address is written to tval
      logic valid;
    },

    // cache request ports
    // I$ address translation requests
    localparam type icache_areq_t = struct packed {
      logic                    fetch_valid;      // address translation valid
      logic [CVA6Cfg.PLEN-1:0] fetch_paddr;      // physical address in
      exception_t              fetch_exception;  // exception occurred during fetch
    },
    localparam type icache_arsp_t = struct packed {
      logic                    fetch_req;    // address translation request
      logic [CVA6Cfg.VLEN-1:0] fetch_vaddr;  // virtual address out
    },

    // I$ data requests
    localparam type icache_dreq_t = struct packed {
      logic                    req;      // we request a new word
      logic                    kill_s1;  // kill the current request
      logic                    kill_s2;  // kill the last request
      logic                    spec;     // request is speculative
      logic [CVA6Cfg.VLEN-1:0] vaddr;    // 1st cycle: 12 bit index is taken for lookup
    },
    localparam type icache_drsp_t = struct packed {
      logic                                ready;  // icache is ready
      logic                                valid;  // signals a valid read
      logic [CVA6Cfg.FETCH_WIDTH-1:0]      data;   // 2+ cycle out: tag
      logic [CVA6Cfg.FETCH_USER_WIDTH-1:0] user;   // User bits
      logic [CVA6Cfg.VLEN-1:0]             vaddr;  // virtual address out
      exception_t                          ex;     // we've encountered an exception
    },

    // IF/ID Stage
    // store the decompressed instruction
    localparam type fetch_entry_t = struct packed {
      logic [CVA6Cfg.VLEN-1:0] address;  // the address of the instructions from below
      logic [31:0] instruction;  // instruction word
      branchpredict_sbe_t     branch_predict; // this field contains branch prediction information regarding the forward branch path
      exception_t             ex;             // this field contains exceptions which might have happened earlier, e.g.: fetch exceptions
    },
    //JVT struct{base,mode}
    localparam type jvt_t = struct packed {
      logic [CVA6Cfg.XLEN-7:0] base;
      logic [5:0] mode;
    },

    // ID/EX/WB Stage
    localparam type scoreboard_entry_t = struct packed {
      logic [CVA6Cfg.VLEN-1:0] pc;  // PC of instruction
      logic [CVA6Cfg.TRANS_ID_BITS-1:0] trans_id;      // this can potentially be simplified, we could index the scoreboard entry
      // with the transaction id in any case make the width more generic
      fu_t fu;  // functional unit to use
      fu_op op;  // operation to perform in each functional unit
      logic [REG_ADDR_SIZE-1:0] rs1;  // register source address 1
      logic [REG_ADDR_SIZE-1:0] rs2;  // register source address 2
      logic [REG_ADDR_SIZE-1:0] rd;  // register destination address
      logic [CVA6Cfg.XLEN-1:0] result;  // for unfinished instructions this field also holds the immediate,
      // for unfinished floating-point that are partly encoded in rs2, this field also holds rs2
      // for unfinished floating-point fused operations (FMADD, FMSUB, FNMADD, FNMSUB)
      // this field holds the address of the third operand from the floating-point register file
      logic valid;  // is the result valid
      logic use_imm;  // should we use the immediate as operand b?
      logic use_zimm;  // use zimm as operand a
      logic use_pc;  // set if we need to use the PC as operand a, PC from exception
      exception_t ex;  // exception has occurred
      branchpredict_sbe_t bp;  // branch predict scoreboard data structure
      logic                     is_compressed; // signals a compressed instructions, we need this information at the commit stage if
                                               // we want jump accordingly e.g.: +4, +2
      logic is_macro_instr;  // is an instruction executed as predefined sequence of instructions called macro definition
      logic is_last_macro_instr;  // is last decoded 32bit instruction of macro definition
      logic is_double_rd_macro_instr;  // is double move decoded 32bit instruction of macro definition
      logic vfp;  // is this a vector floating-point instruction?
      logic is_zcmt;  //is a zcmt instruction
    },
    localparam type writeback_t = struct packed {
      logic valid;  // wb data is valid
      logic [CVA6Cfg.XLEN-1:0] data;  //wb data
      logic ex_valid;  // exception from WB
      logic [CVA6Cfg.TRANS_ID_BITS-1:0] trans_id;  //transaction ID
    },

    // branch-predict
    // this is the struct we get back from ex stage and we will use it to update
    // all the necessary data structures
    // bp_resolve_t
    localparam type bp_resolve_t = struct packed {
      logic                    valid;           // prediction with all its values is valid
      logic [CVA6Cfg.VLEN-1:0] pc;              // PC of predict or mis-predict
      logic [CVA6Cfg.VLEN-1:0] target_address;  // target address at which to jump, or not
      logic                    is_mispredict;   // set if this was a mis-predict
      logic                    is_taken;        // branch is taken
      cf_t                     cf_type;         // Type of control flow change
    },

    // All information needed to determine whether we need to associate an interrupt
    // with the corresponding instruction or not.
    localparam type irq_ctrl_t = struct packed {
      logic [CVA6Cfg.XLEN-1:0] mie;
      logic [CVA6Cfg.XLEN-1:0] mip;
      logic [CVA6Cfg.XLEN-1:0] mideleg;
      logic [CVA6Cfg.XLEN-1:0] hideleg;
      logic                    sie;
      logic                    global_enable;
    },

    localparam type lsu_ctrl_t = struct packed {
      logic                             valid;
      logic [CVA6Cfg.VLEN-1:0]          vaddr;
      logic [31:0]                      tinst;
      logic                             hs_ld_st_inst;
      logic                             hlvx_inst;
      logic                             overflow;
      logic                             g_overflow;
      logic [CVA6Cfg.XLEN-1:0]          data;
      logic [(CVA6Cfg.XLEN/8)-1:0]      be;
      fu_t                              fu;
      fu_op                             operation;
      logic [CVA6Cfg.TRANS_ID_BITS-1:0] trans_id;
    },


    localparam type cbo_t = logic [7:0],

    localparam type fu_data_t = struct packed {
      fu_t                              fu;
      fu_op                             operation;
      logic [CVA6Cfg.XLEN-1:0]          operand_a;
      logic [CVA6Cfg.XLEN-1:0]          operand_b;
      logic [CVA6Cfg.XLEN-1:0]          imm;
      logic [CVA6Cfg.TRANS_ID_BITS-1:0] trans_id;
    },

    localparam type icache_req_t = struct packed {
      logic [CVA6Cfg.ICACHE_SET_ASSOC_WIDTH-1:0] way;  // way to replace
      logic [CVA6Cfg.PLEN-1:0] paddr;  // physical address
      logic nc;  // noncacheable
      logic [CVA6Cfg.MEM_TID_WIDTH-1:0] tid;  // thread id (used as transaction id in Ariane)
    },
    localparam type icache_rtrn_t = struct packed {
      wt_cache_pkg::icache_in_t rtype;  // see definitions above
      logic [CVA6Cfg.ICACHE_LINE_WIDTH-1:0] data;  // full cache line width
      logic [CVA6Cfg.ICACHE_USER_LINE_WIDTH-1:0] user;  // user bits
      struct packed {
        logic                                      vld;  // invalidate only affected way
        logic                                      all;  // invalidate all ways
        logic [CVA6Cfg.ICACHE_INDEX_WIDTH-1:0]     idx;  // physical address to invalidate
        logic [CVA6Cfg.ICACHE_SET_ASSOC_WIDTH-1:0] way;  // way to invalidate
      } inv;  // invalidation vector
      logic [CVA6Cfg.MEM_TID_WIDTH-1:0] tid;  // thread id (used as transaction id in Ariane)
    },

    // D$ data requests
    localparam type dcache_req_i_t = struct packed {
      logic [CVA6Cfg.DCACHE_INDEX_WIDTH-1:0] address_index;
      logic [CVA6Cfg.DCACHE_TAG_WIDTH-1:0]   address_tag;
      logic [CVA6Cfg.XLEN-1:0]               data_wdata;
      logic [CVA6Cfg.DCACHE_USER_WIDTH-1:0]  data_wuser;
      logic                                  data_req;
      logic                                  data_we;
      logic [(CVA6Cfg.XLEN/8)-1:0]           data_be;
      logic [1:0]                            data_size;
      logic [CVA6Cfg.DcacheIdWidth-1:0]      data_id;
      logic                                  kill_req;
      logic                                  tag_valid;
      cbo_t                                  cbo_op;
    },

    localparam type dcache_req_o_t = struct packed {
      logic                                 data_gnt;
      logic                                 data_rvalid;
      logic [CVA6Cfg.DcacheIdWidth-1:0]     data_rid;
      logic [CVA6Cfg.XLEN-1:0]              data_rdata;
      logic [CVA6Cfg.DCACHE_USER_WIDTH-1:0] data_ruser;
    },

    // Accelerator - CVA6
    parameter type accelerator_req_t  = logic,
    parameter type accelerator_resp_t = logic,

    // Accelerator - CVA6's MMU
    parameter type acc_mmu_req_t  = logic,
    parameter type acc_mmu_resp_t = logic,

    // AXI types
    parameter type axi_ar_chan_t = struct packed {
      logic [CVA6Cfg.AxiIdWidth-1:0]   id;
      logic [CVA6Cfg.AxiAddrWidth-1:0] addr;
      axi_pkg::len_t                   len;
      axi_pkg::size_t                  size;
      axi_pkg::burst_t                 burst;
      logic                            lock;
      axi_pkg::cache_t                 cache;
      axi_pkg::prot_t                  prot;
      axi_pkg::qos_t                   qos;
      axi_pkg::region_t                region;
      logic [CVA6Cfg.AxiUserWidth-1:0] user;
    },
    parameter type axi_aw_chan_t = struct packed {
      logic [CVA6Cfg.AxiIdWidth-1:0]   id;
      logic [CVA6Cfg.AxiAddrWidth-1:0] addr;
      axi_pkg::len_t                   len;
      axi_pkg::size_t                  size;
      axi_pkg::burst_t                 burst;
      logic                            lock;
      axi_pkg::cache_t                 cache;
      axi_pkg::prot_t                  prot;
      axi_pkg::qos_t                   qos;
      axi_pkg::region_t                region;
      axi_pkg::atop_t                  atop;
      logic [CVA6Cfg.AxiUserWidth-1:0] user;
    },
    parameter type axi_w_chan_t = struct packed {
      logic [CVA6Cfg.AxiDataWidth-1:0]     data;
      logic [(CVA6Cfg.AxiDataWidth/8)-1:0] strb;
      logic                                last;
      logic [CVA6Cfg.AxiUserWidth-1:0]     user;
    },
    parameter type b_chan_t = struct packed {
      logic [CVA6Cfg.AxiIdWidth-1:0]   id;
      axi_pkg::resp_t                  resp;
      logic [CVA6Cfg.AxiUserWidth-1:0] user;
    },
    parameter type r_chan_t = struct packed {
      logic [CVA6Cfg.AxiIdWidth-1:0]   id;
      logic [CVA6Cfg.AxiDataWidth-1:0] data;
      axi_pkg::resp_t                  resp;
      logic                            last;
      logic [CVA6Cfg.AxiUserWidth-1:0] user;
    },
    parameter type noc_req_t = struct packed {
      axi_aw_chan_t aw;
      logic         aw_valid;
      axi_w_chan_t  w;
      logic         w_valid;
      logic         b_ready;
      axi_ar_chan_t ar;
      logic         ar_valid;
      logic         r_ready;
    },
    parameter type noc_resp_t = struct packed {
      logic    aw_ready;
      logic    ar_ready;
      logic    w_ready;
      logic    b_valid;
      b_chan_t b;
      logic    r_valid;
      r_chan_t r;
    },
    //
    parameter type acc_cfg_t = logic,
    parameter acc_cfg_t AccCfg = '0,
    // CVXIF Types
    parameter type readregflags_t = `READREGFLAGS_T(CVA6Cfg),
    parameter type writeregflags_t = `WRITEREGFLAGS_T(CVA6Cfg),
    parameter type id_t = `ID_T(CVA6Cfg),
    parameter type hartid_t = `HARTID_T(CVA6Cfg),
    parameter type x_compressed_req_t = `X_COMPRESSED_REQ_T(CVA6Cfg, hartid_t),
    parameter type x_compressed_resp_t = `X_COMPRESSED_RESP_T(CVA6Cfg),
    parameter type x_issue_req_t = `X_ISSUE_REQ_T(CVA6Cfg, hartid_t, id_t),
    parameter type x_issue_resp_t = `X_ISSUE_RESP_T(CVA6Cfg, writeregflags_t, readregflags_t),
    parameter type x_register_t = `X_REGISTER_T(CVA6Cfg, hartid_t, id_t, readregflags_t),
    parameter type x_commit_t = `X_COMMIT_T(CVA6Cfg, hartid_t, id_t),
    parameter type x_result_t = `X_RESULT_T(CVA6Cfg, hartid_t, id_t, writeregflags_t),
    parameter type cvxif_req_t =
    `CVXIF_REQ_T(CVA6Cfg, x_compressed_req_t, x_issue_req_t, x_register_t, x_commit_t),
    parameter type cvxif_resp_t =
    `CVXIF_RESP_T(CVA6Cfg, x_compressed_resp_t, x_issue_resp_t, x_result_t)
) (
    // Subsystem Clock - SUBSYSTEM
    input logic clk_i,
    // Asynchronous reset active low - SUBSYSTEM
    input logic rst_ni,
    // DMR or independent mode
    input logic dmr_mode_active_i,
    // Input of DMR checker were not identical
    output logic dmr_failure_o,
    // Reset boot address - SUBSYSTEM
    input logic [CVA6Cfg.VLEN-1:0] boot_addr_i,
    // Hard ID reflected as CSR - SUBSYSTEM
    input logic [CVA6Cfg.XLEN-1:0] hart_id_i,
    // Level sensitive (async) interrupts - SUBSYSTEM
    input logic [1:0] irq_i,
    // Inter-processor (async) interrupt - SUBSYSTEM
    input logic ipi_i,
    // Timer (async) interrupt - SUBSYSTEM
    input logic time_irq_i,
    // Debug (async) request - SUBSYSTEM
    input logic debug_req_i,
    // Probes to build RVFI, can be left open when not used - RVFI
    output rvfi_probes_t rvfi_probes_o,
    // CVXIF request - SUBSYSTEM
    output cvxif_req_t cvxif_req_o,
    // CVXIF response - SUBSYSTEM
    input cvxif_resp_t cvxif_resp_i,
    // noc request, can be AXI or OpenPiton - SUBSYSTEM
    output noc_req_t noc_req_o,
    // noc response, can be AXI or OpenPiton - SUBSYSTEM
    input noc_resp_t noc_resp_i
);

  // HPDcache configuration computed from CVA6 parameters
  localparam int NumPorts = 4;
  localparam hpdcache_pkg::hpdcache_cfg_t HPDcacheCfg =
      cva6_hpdcache_cfg_pkg::hpdcacheBuildCfg(CVA6Cfg, NumPorts);

  // D$ external SRAM types derived from HPDcacheCfg
  `HPDCACHE_TYPEDEF_EXT_SRAM_REQ_T(dcache_ext_sram_req_t, HPDcacheCfg);
  `HPDCACHE_TYPEDEF_EXT_SRAM_RESP_T(dcache_ext_sram_resp_t, HPDcacheCfg);

  // RAM types needed to instantiate hpdcache_memwrap
  `HPDCACHE_TYPEDEF_RAM_TYPES_T(sram, HPDcacheCfg);
  typedef logic unsigned [HPDcacheCfg.u.ways-1:0] sram_way_vector_t;
  typedef struct packed {
    logic valid;
    logic wback;
    logic dirty;
    logic fetch;
    logic [HPDcacheCfg.tagWidth-1:0] tag;
  } sram_dir_entry_t;

  typedef struct packed {
    logic [CVA6Cfg.VLEN-1:0] boot_addr;
    logic [CVA6Cfg.XLEN-1:0] hart_id;
    logic [1:0]              irq;
    logic                    ipi;
    logic                    time_irq;
    logic                    debug_req;
  } cva6_inputs_t;

  // Intermediate signals: cores <-> HMR <-> system
  cva6_inputs_t [NumPhysicalCores-1:0] sys2hmr, hmr2core;

  noc_req_t  [NumPhysicalCores-1:0] noc_req_core2hmr, noc_req_hmr2sys;
  noc_resp_t [NumPhysicalCores-1:0] noc_resp_sys2hmr, noc_resp_hmr2core;

  dcache_ext_sram_req_t  [NumPhysicalCores-1:0] ext_sram_req_core2hmr, ext_sram_req_hmr2sys;
  dcache_ext_sram_resp_t [NumPhysicalCores-1:0] ext_sram_resp_sys2hmr, ext_sram_resp_hmr2core;

  // System input binding (same inputs for all physical cores)
  for (genvar i = 0; i < NumPhysicalCores; i++) begin : gen_sys_inputs
    assign sys2hmr[i].boot_addr = boot_addr_i;
    assign sys2hmr[i].hart_id   = hart_id_i;
    assign sys2hmr[i].irq       = irq_i;
    assign sys2hmr[i].ipi       = ipi_i;
    assign sys2hmr[i].time_irq  = time_irq_i;
    assign sys2hmr[i].debug_req = debug_req_i;
  end

  // System output binding (only index 0 carries the active output)
  assign noc_req_o = noc_req_hmr2sys[0];
  assign noc_resp_sys2hmr[0] = noc_resp_i;
  if (NumPhysicalCores > 1) begin : gen_tie_off_sys
    assign noc_resp_sys2hmr[1]     = '0;
    assign ext_sram_resp_sys2hmr[1] = '0;
  end

  // D$ SRAM macros – driven by the checked ext_sram request from HMR index [0]
  hpdcache_memwrap #(
      .HPDcacheCfg                  (HPDcacheCfg),
      .hpdcache_way_vector_t        (sram_way_vector_t),
      .hpdcache_dir_addr_t          (sram_dir_addr_t),
      .hpdcache_dir_entry_t         (sram_dir_entry_t),
      .hpdcache_data_addr_t         (sram_data_addr_t),
      .hpdcache_data_enable_t       (sram_data_enable_t),
      .hpdcache_data_be_entry_t     (sram_data_be_entry_t),
      .hpdcache_data_entry_t        (sram_data_entry_t),
      .hpdcache_data_row_enable_t   (sram_data_row_enable_t),
      .hpdcache_data_ram_word_sel_t (sram_data_ram_word_sel_t)
  ) i_dcache_sram (
      .clk_i (clk_i),
      .rst_ni(rst_ni),
      // Directory interface
      .dir_cs_i          (ext_sram_req_hmr2sys[0].dir_cs),
      .dir_we_i          (ext_sram_req_hmr2sys[0].dir_we),
      .dir_addr_i        (ext_sram_req_hmr2sys[0].dir_addr),
      .dir_wentry_i      (ext_sram_req_hmr2sys[0].dir_wentry),
      .dir_rentry_o      (ext_sram_resp_sys2hmr[0].dir_rentry),
      .dir_err_cor_o     (ext_sram_resp_sys2hmr[0].dir_err_cor),
      .dir_err_unc_o     (ext_sram_resp_sys2hmr[0].dir_err_unc),
      .dir_err_valid_o   (ext_sram_resp_sys2hmr[0].dir_err_valid),
      .dir_err_dirty_o   (ext_sram_resp_sys2hmr[0].dir_err_dirty),
      // Data interface
      .data_addr_i       (ext_sram_req_hmr2sys[0].data_addr),
      .data_cs_i         (ext_sram_req_hmr2sys[0].data_cs),
      .data_we_i         (ext_sram_req_hmr2sys[0].data_we),
      .data_wbyteenable_i(ext_sram_req_hmr2sys[0].data_wbyteenable),
      .data_wentry_i     (ext_sram_req_hmr2sys[0].data_wentry),
      .data_rentry_o     (ext_sram_resp_sys2hmr[0].data_rentry),
      .data_err_cor_o    (ext_sram_resp_sys2hmr[0].data_err_cor),
      .data_err_unc_o    (ext_sram_resp_sys2hmr[0].data_err_unc)
  );

  // Core instantiation
  for (genvar i = 0; i < NumPhysicalCores; i++) begin : gen_cva6_core
    cva6 #(
        // CVA6 config
        .CVA6Cfg(CVA6Cfg),
        // RVFI PROBES
        .rvfi_probes_instr_t(rvfi_probes_instr_t),
        .rvfi_probes_csr_t(rvfi_probes_csr_t),
        .rvfi_probes_t(rvfi_probes_t),
        .exception_t(exception_t),
        // Accelerator - CVA6
        .accelerator_req_t(accelerator_req_t),
        .accelerator_resp_t(accelerator_resp_t),
        // Accelerator - CVA6's MMU
        .acc_mmu_req_t(acc_mmu_req_t),
        .acc_mmu_resp_t(acc_mmu_resp_t),
        // AXI types
        .axi_ar_chan_t(axi_ar_chan_t),
        .axi_aw_chan_t(axi_aw_chan_t),
        .axi_w_chan_t(axi_w_chan_t),
        .b_chan_t(b_chan_t),
        .r_chan_t(r_chan_t),
        .noc_req_t(noc_req_t),
        .noc_resp_t(noc_resp_t),
        //
        .acc_cfg_t(acc_cfg_t),
        .AccCfg(AccCfg),
        // CVXIF Types
        .readregflags_t(readregflags_t),
        .writeregflags_t(writeregflags_t),
        .id_t(id_t),
        .hartid_t(hartid_t),
        .x_compressed_req_t(x_compressed_req_t),
        .x_compressed_resp_t(x_compressed_resp_t),
        .x_issue_req_t(x_issue_req_t),
        .x_issue_resp_t(x_issue_resp_t),
        .x_register_t(x_register_t),
        .x_commit_t(x_commit_t),
        .x_result_t(x_result_t),
        .cvxif_req_t(cvxif_req_t),
        .cvxif_resp_t(cvxif_resp_t),
        .dcache_ext_sram_req_t(dcache_ext_sram_req_t),
        .dcache_ext_sram_resp_t(dcache_ext_sram_resp_t)
    ) i_cva6_core (
        // Subsystem Clock - SUBSYSTEM
        .clk_i       (clk_i),
        // Asynchronous reset active low - SUBSYSTEM
        .rst_ni      (rst_ni),
        // Reset boot address - SUBSYSTEM
        .boot_addr_i (hmr2core[i].boot_addr),
        // Hard ID reflected as CSR - SUBSYSTEM
        .hart_id_i   (hmr2core[i].hart_id),
        // Level sensitive (async) interrupts - SUBSYSTEM
        .irq_i       (hmr2core[i].irq),
        // Inter-processor (async) interrupt - SUBSYSTEM
        .ipi_i       (hmr2core[i].ipi),
        // Timer (async) interrupt - SUBSYSTEM
        .time_irq_i  (hmr2core[i].time_irq),
        // Debug (async) request - SUBSYSTEM
        .debug_req_i (hmr2core[i].debug_req),
        // Probes to build RVFI, can be left open when not used - RVFI
        .rvfi_probes_o (),
        // CVXIF request - SUBSYSTEM
        .cvxif_req_o (),
        // CVXIF response - SUBSYSTEM
        .cvxif_resp_i ('0),
        // noc request, can be AXI or OpenPiton - SUBSYSTEM
        .noc_req_o   (noc_req_core2hmr[i]),
        // noc response, can be AXI or OpenPiton - SUBSYSTEM
        .noc_resp_i  (noc_resp_hmr2core[i]),
        // D$ external SRAM interface
        .dcache_ext_sram_req_o (ext_sram_req_core2hmr[i]),
        .dcache_ext_sram_resp_i(ext_sram_resp_hmr2core[i])
    );
  end

  if (NumPhysicalCores == 1) begin : gen_single_core
    // No DMR – straight passthrough
    assign dmr_failure_o = 1'b0;
    assign hmr2core[0]              = sys2hmr[0];
    assign noc_req_hmr2sys[0]       = noc_req_core2hmr[0];
    assign noc_resp_hmr2core[0]     = noc_resp_sys2hmr[0];
    assign ext_sram_req_hmr2sys[0]  = ext_sram_req_core2hmr[0];
    assign ext_sram_resp_hmr2core[0] = ext_sram_resp_sys2hmr[0];

  end else if (NumPhysicalCores == 2) begin : gen_hmr

    hmr_unit #(
        .CVA6Cfg       (CVA6Cfg),
        .all_inputs_t  (cva6_inputs_t),
        .bus_outputs_t (noc_req_t),
        .bus_inputs_t  (noc_resp_t),
        .ext_sram_req_t (dcache_ext_sram_req_t),
        .ext_sram_resp_t(dcache_ext_sram_resp_t)
    ) i_hmr_unit (
        .clk_i (clk_i),
        .rst_ni(rst_ni),
        .dmr_mode_active_i(dmr_mode_active_i),
        .dmr_failure_o    (dmr_failure_o),
        // System side
        .sys_inputs_i      (sys2hmr),
        .sys_bus_outputs_o (noc_req_hmr2sys),
        .sys_bus_inputs_i  (noc_resp_sys2hmr),
        .sys_ext_sram_req_o (ext_sram_req_hmr2sys),
        .sys_ext_sram_resp_i(ext_sram_resp_sys2hmr),
        // Core side
        .core_inputs_o      (hmr2core),
        .core_bus_outputs_i (noc_req_core2hmr),
        .core_bus_inputs_o  (noc_resp_hmr2core),
        .core_ext_sram_req_i (ext_sram_req_core2hmr),
        .core_ext_sram_resp_o(ext_sram_resp_hmr2core)
    );

  end else begin
    $error("Unsupported number of cores.");
  end
endmodule
