// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Stefan Odermatt   <soderma@student.ethz.ch>
// Author: Ivan Herger       <iherger@student.ethz.ch>
// Author: Riccardo Tedeschi <riccardo.tedeschi6@unibo.it>

`include "rvfi_types.svh"
`include "cvxif_types.svh"

module cva6_top
  import ariane_pkg::*;
#(
    // Number of cores
    parameter logic [1:0] NumCores = 2'd1,

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

    localparam type cbo_t = logic [7:0],

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

    localparam int unsigned NumPorts = 4,

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
    // Synchronous reset active high - SUBSYSTEM
    input logic clear_i,
    // DMR or independent mode
    input logic dmr_mode_active_i,
    // Input of DMR checker were not identical
    output logic dmr_failure_o,
    // Reset boot address - SUBSYSTEM
    input logic [CVA6Cfg.VLEN-1:0] boot_addr_i,
    // Hard ID reflected as CSR - SUBSYSTEM
    input logic [NumCores-1:0][CVA6Cfg.XLEN-1:0] hart_id_i,
    // Level sensitive (async) interrupts - SUBSYSTEM
    input logic [NumCores-1:0][1:0] irq_i,
    // Inter-processor (async) interrupt - SUBSYSTEM
    input logic [NumCores-1:0] ipi_i,
    // Timer (async) interrupt - SUBSYSTEM
    input logic [NumCores-1:0] time_irq_i,
    // Debug (async) request - SUBSYSTEM
    input logic [NumCores-1:0] debug_req_i,
    // Probes to build RVFI, can be left open when not used - RVFI
    output rvfi_probes_t [NumCores-1:0] rvfi_probes_o,
    // CVXIF request - SUBSYSTEM
    output cvxif_req_t [NumCores-1:0] cvxif_req_o,
    // CVXIF response - SUBSYSTEM
    input cvxif_resp_t [NumCores-1:0] cvxif_resp_i,
    // noc request, can be AXI or OpenPiton - SUBSYSTEM
    output noc_req_t [NumCores-1:0] noc_req_o,
    // noc response, can be AXI or OpenPiton - SUBSYSTEM
    input noc_resp_t [NumCores-1:0] noc_resp_i
);

  typedef struct packed {
    logic [CVA6Cfg.XLEN-1:0] hart_id;
    logic [1:0]              irq;
    logic                    ipi;
    logic                    time_irq;
    logic                    debug_req;
  } cva6_inputs_t;

  typedef logic cva6_outputs_t;

  cva6_inputs_t [NumCores-1:0] sys2hmr, hmr2core;
  cva6_outputs_t [NumCores-1:0] hmr2sys, core2hmr;
  noc_req_t [NumCores-1:0] noc_req_core2hmr, noc_req_hmr2sys;
  noc_resp_t [NumCores-1:0] noc_resp_sys2hmr, noc_resp_hmr2core;
  dcache_req_i_t [NumCores-1:0][NumPorts-1:0] dcache_req_core2hmr, dcache_req_hmr2core;
  dcache_req_o_t [NumCores-1:0][NumPorts-1:0] dcache_rsp_core2hmr, dcache_rsp_hmr2core;
  icache_areq_t [NumCores-1:0] icache_areq_core2hmr, icache_areq_hmr2core;
  icache_arsp_t [NumCores-1:0] icache_arsp_core2hmr, icache_arsp_hmr2core;
  icache_dreq_t [NumCores-1:0] icache_dreq_core2hmr, icache_dreq_hmr2core;
  icache_drsp_t [NumCores-1:0] icache_drsp_core2hmr, icache_drsp_hmr2core;
  // D-cache wbuffer signals
  logic [NumCores-1:0] wbuffer_empty_core2hmr, wbuffer_empty_hmr2core;
  logic [NumCores-1:0] wbuffer_not_ni_core2hmr, wbuffer_not_ni_hmr2core;
  // AMO request and response
  ariane_pkg::amo_req_t [NumCores-1:0] dcache_amo_req_core2hmr, dcache_amo_req_hmr2core;
  ariane_pkg::amo_resp_t [NumCores-1:0] dcache_amo_resp_core2hmr, dcache_amo_resp_hmr2core;
  // D-cache flush signals
  logic [NumCores-1:0] dcache_flush_core2hmr, dcache_flush_hmr2core;
  logic [NumCores-1:0] dcache_flush_ack_core2hmr, dcache_flush_ack_hmr2core;

  for (genvar i = 0; i < NumCores; i++) begin : gen_cva6_core

    // Tieoff for core outputs to HMR
    assign core2hmr[i]          = '0;

    // Bind system inputs to HMR
    assign sys2hmr[i].hart_id   = hart_id_i[i];
    assign sys2hmr[i].irq       = irq_i[i];
    assign sys2hmr[i].ipi       = ipi_i[i];
    assign sys2hmr[i].time_irq  = time_irq_i[i];
    assign sys2hmr[i].debug_req = debug_req_i[i];

    // Bind HMR outputs to system.
    assign noc_req_o[i]         = noc_req_hmr2sys[i];
    assign noc_resp_sys2hmr[i]  = noc_resp_i[i];

    cva6 #(
        // CVA6 config
        .CVA6Cfg(CVA6Cfg),
        // RVFI PROBES
        .rvfi_probes_instr_t(rvfi_probes_instr_t),
        .rvfi_probes_csr_t(rvfi_probes_csr_t),
        .rvfi_probes_t(rvfi_probes_t),
        .exception_t(exception_t),
        // cache request ports
        // I$ address translation requests
        .icache_areq_t(icache_areq_t),
        .icache_arsp_t(icache_arsp_t),
        // I$ data requests
        .icache_dreq_t(icache_dreq_t),
        .icache_drsp_t(icache_drsp_t),

        // D$ data requests
        .dcache_req_i_t(dcache_req_i_t),
        .dcache_req_o_t(dcache_req_o_t),
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
        .NumPorts(NumPorts)
    ) i_cva6_core (
        // Subsystem Clock - SUBSYSTEM
        .clk_i(clk_i),
        // Asynchronous reset active low - SUBSYSTEM
        .rst_ni(rst_ni),
        // Reset boot address - SUBSYSTEM
        .clear_i(clear_i),
        .boot_addr_i(boot_addr_i),
        // Hard ID reflected as CSR - SUBSYSTEM
        .hart_id_i(hmr2core[i].hart_id),
        // Level sensitive (async) interrupts - SUBSYSTEM
        .irq_i(hmr2core[i].irq),
        // Inter-processor (async) interrupt - SUBSYSTEM
        .ipi_i(hmr2core[i].ipi),
        // Timer (async) interrupt - SUBSYSTEM
        .time_irq_i(hmr2core[i].time_irq),
        // Debug (async) request - SUBSYSTEM
        .debug_req_i(hmr2core[i].debug_req),
        // Probes to build RVFI, can be left open when not used - RVFI
        .rvfi_probes_o(rvfi_probes_o[i]),
        // CVXIF request - SUBSYSTEM
        .cvxif_req_o(cvxif_req_o[i]),
        // CVXIF response - SUBSYSTEM
        .cvxif_resp_i(cvxif_resp_i[i]),
        // noc request, can be AXI or OpenPiton - SUBSYSTEM
        .noc_req_o(noc_req_core2hmr[i]),
        // noc response, can be AXI or OpenPiton - SUBSYSTEM
        .noc_resp_i(noc_resp_hmr2core[i]),
        // HMR unit internal signal extraction
        // D-Cache, requests to cache
        .dcache_req_to_cache_i(dcache_req_hmr2core[i]),
        .dcache_req_to_cache_o(dcache_req_core2hmr[i]),
        // D-cache, response from cache
        .dcache_req_from_cache_i(dcache_rsp_hmr2core[i]),
        .dcache_req_from_cache_o(dcache_rsp_core2hmr[i]),
        // D-cache wbuffer signals
        .dcache_wbuffer_empty_i(wbuffer_empty_hmr2core[i]),
        .dcache_wbuffer_empty_o(wbuffer_empty_core2hmr[i]),
        .dcache_wbuffer_not_ni_i(wbuffer_not_ni_hmr2core[i]),
        .dcache_wbuffer_not_ni_o(wbuffer_not_ni_core2hmr[i]),
        // AMO signals
        .dcache_amo_req_i(dcache_amo_req_hmr2core[i]),
        .dcache_amo_req_o(dcache_amo_req_core2hmr[i]),
        .dcache_amo_resp_i(dcache_amo_resp_hmr2core[i]),
        .dcache_amo_resp_o(dcache_amo_resp_core2hmr[i]),
        // D-cache flush signals
        .dcache_flush_i(dcache_flush_hmr2core[i]),
        .dcache_flush_o(dcache_flush_core2hmr[i]),
        .dcache_flush_ack_i(dcache_flush_ack_hmr2core[i]),
        .dcache_flush_ack_o(dcache_flush_ack_core2hmr[i]),
        // I-cache, address resolve response from MMU
        .icache_areq_ex_cache_i(icache_areq_hmr2core[i]),
        .icache_areq_ex_cache_o(icache_areq_core2hmr[i]),
        // I-cache, address resolve request to MMU
        .icache_areq_cache_ex_i(icache_arsp_hmr2core[i]),
        .icache_areq_cache_ex_o(icache_arsp_core2hmr[i]),
        // I-cache, data request from fetch stage
        .icache_dreq_if_cache_i(icache_dreq_hmr2core[i]),
        .icache_dreq_if_cache_o(icache_dreq_core2hmr[i]),
        // I-cache, data response to fetch stage
        .icache_dreq_cache_if_i(icache_drsp_hmr2core[i]),
        .icache_dreq_cache_if_o(icache_drsp_core2hmr[i])
    );
  end

  if (NumCores == 1) begin : gen_single_core
    assign dmr_failure_o = 1'b0;
    assign hmr2sys = core2hmr;
    assign noc_req_hmr2sys = noc_req_core2hmr;
    assign hmr2core = sys2hmr;
    assign noc_resp_hmr2core = noc_resp_sys2hmr;
    assign dcache_req_hmr2core = dcache_req_core2hmr;
    assign dcache_rsp_hmr2core = dcache_rsp_core2hmr;
    assign wbuffer_empty_hmr2core = wbuffer_empty_core2hmr;
    assign wbuffer_not_ni_hmr2core = wbuffer_not_ni_core2hmr;
    assign dcache_amo_req_hmr2core = dcache_amo_req_core2hmr;
    assign dcache_amo_resp_hmr2core = dcache_amo_resp_core2hmr;
    assign dcache_flush_hmr2core = dcache_flush_core2hmr;
    assign dcache_flush_ack_hmr2core = dcache_flush_ack_core2hmr;
    assign icache_areq_hmr2core = icache_areq_core2hmr;
    assign icache_arsp_hmr2core = icache_arsp_core2hmr;
    assign icache_dreq_hmr2core = icache_dreq_core2hmr;
    assign icache_drsp_hmr2core = icache_drsp_core2hmr;
  end else if (NumCores == 2) begin : gen_hmr
    hmr_unit #(
        // CVA6 config
        .CVA6Cfg(CVA6Cfg),
        // Number of dcache ports
        .NumPorts(NumPorts),
        // General core inputs wrapping struct
        .all_inputs_t(cva6_inputs_t),
        // General core outputs wrapping struct
        .nominal_outputs_t(cva6_outputs_t),
        // Bus outputs wrapping struct (requires SeparateData)
        .bus_outputs_t(noc_req_t),
        .bus_inputs_t(noc_resp_t),
        // Type for requests to data cache
        .dcache_req_t(dcache_req_i_t),
        // Type for response from data cache
        .dcache_rsp_t(dcache_req_o_t),
        // Type for areq to instruction cache
        .icache_areq_t(icache_areq_t),
        .icache_arsp_t(icache_arsp_t),
        .icache_dreq_t(icache_dreq_t),
        .icache_drsp_t(icache_drsp_t)
    ) i_relcva6_hmr (
        .clk_i(clk_i),
        .rst_ni(rst_ni),
        // 1: dmr mode, 0: independent mode
        .dmr_mode_active_i(dmr_mode_active_i),
        // DMR signals
        // Indicates if the DMR group has multiple mismatches
        .dmr_failure_o(dmr_failure_o),
        // Signals between HMR unit and system
        .sys_inputs_i(sys2hmr),
        .sys_nominal_outputs_o(hmr2sys),
        .sys_bus_outputs_o(noc_req_hmr2sys),
        .sys_bus_inputs_i(noc_resp_sys2hmr),
        // Signal between HMR unit and core
        .core_inputs_o(hmr2core),
        .core_nominal_outputs_i(core2hmr),
        .core_bus_outputs_i(noc_req_core2hmr),
        .core_bus_inputs_o(noc_resp_hmr2core),
        // Signals between HMR unit and data cache
        .dcache_req_core2hmr_i(dcache_req_core2hmr[NumCores-1:0]),
        .dcache_req_hmr2core_o(dcache_req_hmr2core[NumCores-1:0]),
        .dcache_rsp_core2hmr_i(dcache_rsp_core2hmr[NumCores-1:0]),
        .dcache_rsp_hmr2core_o(dcache_rsp_hmr2core[NumCores-1:0]),
        // Data cache write buffer signals
        .dcache_wbuffer_empty_core2hmr_i(wbuffer_empty_core2hmr),
        .dcache_wbuffer_empty_hmr2core_o(wbuffer_empty_hmr2core),
        .dcache_wbuffer_not_ni_core2hmr_i(wbuffer_not_ni_core2hmr),
        .dcache_wbuffer_not_ni_hmr2core_o(wbuffer_not_ni_hmr2core),
        // AMO signals
        .dcache_amo_req_core2hmr_i(dcache_amo_req_core2hmr),
        .dcache_amo_req_hmr2core_o(dcache_amo_req_hmr2core),
        .dcache_amo_resp_core2hmr_i(dcache_amo_resp_core2hmr),
        .dcache_amo_resp_hmr2core_o(dcache_amo_resp_hmr2core),
        // D-cache flush signals
        .dcache_flush_core2hmr_i(dcache_flush_core2hmr),
        .dcache_flush_hmr2core_o(dcache_flush_hmr2core),
        .dcache_flush_ack_core2hmr_i(dcache_flush_ack_core2hmr),
        .dcache_flush_ack_hmr2core_o(dcache_flush_ack_hmr2core),
        // Signals between HMR unit and instruction cache
        .icache_areq_core2hmr_i(icache_areq_core2hmr),
        .icache_areq_hmr2core_o(icache_areq_hmr2core),
        .icache_arsp_core2hmr_i(icache_arsp_core2hmr),
        .icache_arsp_hmr2core_o(icache_arsp_hmr2core),
        .icache_dreq_core2hmr_i(icache_dreq_core2hmr),
        .icache_dreq_hmr2core_o(icache_dreq_hmr2core),
        .icache_drsp_core2hmr_i(icache_drsp_core2hmr),
        .icache_drsp_hmr2core_o(icache_drsp_hmr2core)
    );
  end else begin
    $error("Unsupported number of cores.");
  end
endmodule
