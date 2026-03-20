// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Riccardo Tedeschi <riccardo.tedeschi6@unibo.it>
// Author: Stefan Odermatt   <soderma@student.ethz.ch>
// Author: Ivan Herger       <iherger@student.ethz.ch>


// Note: Currently the HMR unit only implements DMR
//
// In the new architecture each physical core has its own complete cache
// hierarchy with externalized SRAMs. The DMR comparison happens at two
// boundaries:
//   1. NOC (AXI bus) requests  – compared before reaching the system
//   2. Ext SRAM requests       – compared before reaching the SRAM macros
//
// The system side is dimensioned by NumLogicalCores for NOC and inputs
// (1 in DMR, 2 in split). Ext SRAM is always scalar (single memwrap).
// The core side is always 2 (NumPhysicalCores).
module hmr_unit #(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = build_config_pkg::build_config(
        cva6_config_pkg::cva6_cfg
    ),
    parameter int unsigned NumLogicalCores = 1,
    // General core inputs wrapping struct
    parameter type all_inputs_t = logic,
    // Bus outputs wrapping struct (NOC request)
    parameter type bus_outputs_t = logic,
    // Bus input wrapping struct (NOC response)
    parameter type bus_inputs_t = logic,
    // D$ external SRAM request type
    parameter type dcache_ext_sram_req_t = logic,
    // D$ external SRAM response type
    parameter type dcache_ext_sram_resp_t = logic
) (
    input logic clk_i,
    input logic rst_ni,

    input logic dmr_mode_active_i,  // 1: dmr mode, 0: independent mode

    // DMR signals
    output logic dmr_failure_o,

    // Signals between HMR unit and system
    input  all_inputs_t              [NumLogicalCores-1:0] sys_inputs_i,
    output bus_outputs_t             [NumLogicalCores-1:0] sys_bus_outputs_o,
    input  bus_inputs_t              [NumLogicalCores-1:0] sys_bus_inputs_i,
    output dcache_ext_sram_req_t     sys_dcache_ext_sram_req_o,
    input  dcache_ext_sram_resp_t    sys_dcache_ext_sram_resp_i,

    // Signals between HMR unit and cores (always 2 physical cores)
    output all_inputs_t              [1:0] core_inputs_o,
    input  bus_outputs_t             [1:0] core_bus_outputs_i,
    output bus_inputs_t              [1:0] core_bus_inputs_o,
    input  dcache_ext_sram_req_t     [1:0] core_dcache_ext_sram_req_i,
    output dcache_ext_sram_resp_t    [1:0] core_dcache_ext_sram_resp_o
);

  // DMR checked signals & failure flags
  bus_outputs_t  dmr_bus_outputs;
  dcache_ext_sram_req_t dmr_dcache_ext_sram_req;

  logic dmr_failure_bus;
  logic dmr_failure_dcache_ext_sram;

  assign dmr_failure_o = dmr_mode_active_i & (dmr_failure_bus | dmr_failure_dcache_ext_sram);

  // ---------------------------------------------------------------------------
  // Core Inputs
  // DMR: both cores receive sys_inputs[0] (identical stimuli)
  // Independent: each core receives its own sys_inputs[i]
  // ---------------------------------------------------------------------------
  for (genvar i = 0; i < 2; i++) begin : gen_core_inputs
    always_comb begin
      if (dmr_mode_active_i) begin
        core_inputs_o[i] = sys_inputs_i[0];
      end else begin
        core_inputs_o[i] = sys_inputs_i[i];
      end
    end
  end

  // ---------------------------------------------------------------------------
  // NOC (AXI bus) outputs – request path
  // DMR: compare both cores, forward checked result on sys[0]
  // Independent: passthrough per logical core
  // ---------------------------------------------------------------------------
  DMR_checker #(
      .check_bus_t(bus_outputs_t),
      .AxiBus     (1'b1)
  ) i_dmr_bus_checker (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .inp_a_i(core_bus_outputs_i[0]),
      .inp_b_i(core_bus_outputs_i[1]),
      .check_o(dmr_bus_outputs),
      .error_o(dmr_failure_bus)
  );

  for (genvar i = 0; i < NumLogicalCores; i++) begin : gen_bus_outputs
    always_comb begin
      if (dmr_mode_active_i) begin
        sys_bus_outputs_o[i] = dmr_bus_outputs;
      end else begin
        sys_bus_outputs_o[i] = core_bus_outputs_i[i];
      end
    end
  end

  // ---------------------------------------------------------------------------
  // NOC (AXI bus) inputs – response path
  // DMR: both cores receive sys_bus_inputs[0]
  // Independent: passthrough per logical core
  // ---------------------------------------------------------------------------
  for (genvar i = 0; i < 2; i++) begin : gen_bus_inputs
    always_comb begin
      if (dmr_mode_active_i) begin
        core_bus_inputs_o[i] = sys_bus_inputs_i[0];
      end else begin
        core_bus_inputs_o[i] = sys_bus_inputs_i[i];
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Ext D$ SRAM – request path (scalar, single memwrap)
  // Compare both cores, forward checked result to system
  // ---------------------------------------------------------------------------
  DMR_checker #(
      .check_bus_t(dcache_ext_sram_req_t)
  ) i_dmr_dcache_ext_sram_checker (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .inp_a_i(core_dcache_ext_sram_req_i[0]),
      .inp_b_i(core_dcache_ext_sram_req_i[1]),
      .check_o(dmr_dcache_ext_sram_req),
      .error_o(dmr_failure_dcache_ext_sram)
  );

  always_comb begin
    if (dmr_mode_active_i) begin
      sys_dcache_ext_sram_req_o = dmr_dcache_ext_sram_req;
    end else begin
      sys_dcache_ext_sram_req_o = core_dcache_ext_sram_req_i[0];
    end
  end

  // ---------------------------------------------------------------------------
  // Ext D$ SRAM – response path (scalar, single memwrap)
  // Both cores receive the same response
  // ---------------------------------------------------------------------------
  for (genvar i = 0; i < 2; i++) begin : gen_dcache_ext_sram_resp
    assign core_dcache_ext_sram_resp_o[i] = sys_dcache_ext_sram_resp_i;
  end

endmodule
