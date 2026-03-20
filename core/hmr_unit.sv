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
// NumLogicalCores controls the mode:
//   1 → fixed DMR (no independent/split mode, dmr_mode_active_i ignored)
//   2 → HMR      (runtime switchable between DMR and independent mode)
//
// The system side is dimensioned by NumLogicalCores for NOC and inputs.
// Ext SRAM is always scalar (single memwrap).
// The core side is always 2 (NumPhysicalCores).
module hmr_unit #(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = build_config_pkg::build_config(
        cva6_config_pkg::cva6_cfg
    ),
    parameter bit EnableDMR = 1'b1,
    parameter bit EnableHMR = 1'b0,
    localparam int unsigned NumLogicalCores = EnableHMR ? 2 : 1,
    // General core inputs wrapping struct
    parameter type all_inputs_t = logic,
    // Bus outputs wrapping struct (NOC request)
    parameter type bus_outputs_t = logic,
    // Bus input wrapping struct (NOC response)
    parameter type bus_inputs_t = logic,
    // D$ external SRAM request type
    parameter type dcache_ext_sram_req_t = logic,
    // D$ external SRAM response type
    parameter type dcache_ext_sram_resp_t = logic,
    // I$ external SRAM request type
    parameter type icache_ext_sram_req_t = logic,
    // I$ external SRAM response type
    parameter type icache_ext_sram_resp_t = logic
) (
    input logic clk_i,
    input logic rst_ni,

    input logic dmr_mode_active_i,  // 1: dmr mode, 0: independent mode (only used when NumLogicalCores > 1)

    // DMR signals
    output logic dmr_failure_o,

    // Signals between HMR unit and system
    input  all_inputs_t              [NumLogicalCores-1:0] sys_inputs_i,
    output bus_outputs_t             [NumLogicalCores-1:0] sys_bus_outputs_o,
    input  bus_inputs_t              [NumLogicalCores-1:0] sys_bus_inputs_i,
    output dcache_ext_sram_req_t     sys_dcache_ext_sram_req_o,
    input  dcache_ext_sram_resp_t    sys_dcache_ext_sram_resp_i,
    output icache_ext_sram_req_t     sys_icache_ext_sram_req_o,
    input  icache_ext_sram_resp_t    sys_icache_ext_sram_resp_i,

    // Signals between HMR unit and cores (always 2 physical cores)
    output all_inputs_t              [1:0] core_inputs_o,
    input  bus_outputs_t             [1:0] core_bus_outputs_i,
    output bus_inputs_t              [1:0] core_bus_inputs_o,
    input  dcache_ext_sram_req_t     [1:0] core_dcache_ext_sram_req_i,
    output dcache_ext_sram_resp_t    [1:0] core_dcache_ext_sram_resp_o,
    input  icache_ext_sram_req_t     [1:0] core_icache_ext_sram_req_i,
    output icache_ext_sram_resp_t    [1:0] core_icache_ext_sram_resp_o
);

  // Effective DMR active signal: in fixed DMR mode (!EnableHMR),
  // always active; in HMR mode, controlled at runtime.
  logic dmr_active;
  if (!EnableHMR) begin : gen_fixed_dmr
    assign dmr_active = 1'b1;
  end else begin : gen_hmr_mode
    assign dmr_active = dmr_mode_active_i;
  end

  // DMR checked signals & failure flags
  bus_outputs_t  dmr_bus_outputs;
  dcache_ext_sram_req_t dmr_dcache_ext_sram_req;
  icache_ext_sram_req_t dmr_icache_ext_sram_req;

  logic dmr_failure_bus;
  logic dmr_failure_dcache_ext_sram;
  logic dmr_failure_icache_ext_sram;

  assign dmr_failure_o = dmr_active & (dmr_failure_bus | dmr_failure_dcache_ext_sram | dmr_failure_icache_ext_sram);

  // ---------------------------------------------------------------------------
  // Core Inputs
  // DMR: both cores receive sys_inputs[0] (identical stimuli)
  // Independent (HMR only): each core receives its own sys_inputs[i]
  // ---------------------------------------------------------------------------
  if (!EnableHMR) begin : gen_core_inputs_dmr
    // Fixed DMR: both cores always get sys_inputs[0]
    for (genvar i = 0; i < 2; i++) begin : gen_core_inputs
      assign core_inputs_o[i] = sys_inputs_i[0];
    end
  end else begin : gen_core_inputs_hmr
    for (genvar i = 0; i < 2; i++) begin : gen_core_inputs
      always_comb begin
        if (dmr_active)
          core_inputs_o[i] = sys_inputs_i[0];
        else
          core_inputs_o[i] = sys_inputs_i[i];
      end
    end
  end

  // ---------------------------------------------------------------------------
  // NOC (AXI bus) outputs – request path
  // DMR: compare both cores, forward checked result on sys[0]
  // Independent (HMR only): passthrough per logical core
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

  if (!EnableHMR) begin : gen_bus_outputs_dmr
    assign sys_bus_outputs_o[0] = dmr_bus_outputs;
  end else begin : gen_bus_outputs_hmr
    for (genvar i = 0; i < NumLogicalCores; i++) begin : gen_bus_outputs
      always_comb begin
        if (dmr_active)
          sys_bus_outputs_o[i] = dmr_bus_outputs;
        else
          sys_bus_outputs_o[i] = core_bus_outputs_i[i];
      end
    end
  end

  // ---------------------------------------------------------------------------
  // NOC (AXI bus) inputs – response path
  // DMR: both cores receive sys_bus_inputs[0]
  // Independent (HMR only): passthrough per logical core
  // ---------------------------------------------------------------------------
  if (!EnableHMR) begin : gen_bus_inputs_dmr
    for (genvar i = 0; i < 2; i++) begin : gen_bus_inputs
      assign core_bus_inputs_o[i] = sys_bus_inputs_i[0];
    end
  end else begin : gen_bus_inputs_hmr
    for (genvar i = 0; i < 2; i++) begin : gen_bus_inputs
      always_comb begin
        if (dmr_active)
          core_bus_inputs_o[i] = sys_bus_inputs_i[0];
        else
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

  if (!EnableHMR) begin : gen_dcache_ext_sram_req_dmr
    assign sys_dcache_ext_sram_req_o = dmr_dcache_ext_sram_req;
  end else begin : gen_dcache_ext_sram_req_hmr
    always_comb begin
      if (dmr_active)
        sys_dcache_ext_sram_req_o = dmr_dcache_ext_sram_req;
      else
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

  // ---------------------------------------------------------------------------
  // Ext I$ SRAM – request path (scalar, single memwrap)
  // Compare both cores, forward checked result to system
  // ---------------------------------------------------------------------------
  DMR_checker #(
      .check_bus_t(icache_ext_sram_req_t)
  ) i_dmr_icache_ext_sram_checker (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .inp_a_i(core_icache_ext_sram_req_i[0]),
      .inp_b_i(core_icache_ext_sram_req_i[1]),
      .check_o(dmr_icache_ext_sram_req),
      .error_o(dmr_failure_icache_ext_sram)
  );

  if (!EnableHMR) begin : gen_icache_ext_sram_req_dmr
    assign sys_icache_ext_sram_req_o = dmr_icache_ext_sram_req;
  end else begin : gen_icache_ext_sram_req_hmr
    always_comb begin
      if (dmr_active)
        sys_icache_ext_sram_req_o = dmr_icache_ext_sram_req;
      else
        sys_icache_ext_sram_req_o = core_icache_ext_sram_req_i[0];
    end
  end

  // ---------------------------------------------------------------------------
  // Ext I$ SRAM – response path (scalar, single memwrap)
  // Both cores receive the same response
  // ---------------------------------------------------------------------------
  for (genvar i = 0; i < 2; i++) begin : gen_icache_ext_sram_resp
    assign core_icache_ext_sram_resp_o[i] = sys_icache_ext_sram_resp_i;
  end

endmodule
