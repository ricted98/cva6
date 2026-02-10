// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Stefan Odermatt  <soderma@student.ethz.ch>
// Author: Ivan Herger      <iherger@student.ethz.ch>


// Note: Currently the HMR unit only implements DMR
module hmr_unit #(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = build_config_pkg::build_config(
        cva6_config_pkg::cva6_cfg
    ),
    // General core inputs wrapping struct
    parameter type all_inputs_t = logic,
    // General core outputs wrapping struct
    parameter type nominal_outputs_t = logic,
    // Bus outputs wrapping struct (requires SeparateData)
    parameter type bus_outputs_t = logic,
    // Bus input wrapping struct
    parameter type bus_inputs_t = logic,
    // Default nominal outputs when output ports are disabled
    parameter nominal_outputs_t DefaultNominalOutputs = '{default: '0},
    // Default bus outputs when output ports are disabled (requires SeparateData)
    parameter bus_outputs_t DefaultBusOutputs = '{default: '0},
    // Number of dcache ports
    parameter int unsigned NumPorts = 3,
    // Type for requests to data cache
    parameter type dcache_req_t = logic,
    // Type for response from data cache
    parameter type dcache_rsp_t = logic,
    // Type for areq to instruction cache
    parameter type icache_areq_t = logic,
    parameter type icache_arsp_t = logic,
    parameter type icache_dreq_t = logic,
    parameter type icache_drsp_t = logic
) (
    input logic clk_i,
    input logic rst_ni,

    input logic dmr_mode_active_i,  // 1: dmr mode, 0: independent mode

    // DMR signals
    // Indicates if the DMR group has multiple mismatches
    output logic dmr_failure_o,

    // Signals between HMR unit and system
    input  all_inputs_t      [1:0] sys_inputs_i,
    output nominal_outputs_t [1:0] sys_nominal_outputs_o,
    output bus_outputs_t     [1:0] sys_bus_outputs_o,
    input  bus_inputs_t      [1:0] sys_bus_inputs_i,

    // Signal between HMR unit and core
    output all_inputs_t      [1:0] core_inputs_o,
    input  nominal_outputs_t [1:0] core_nominal_outputs_i,
    input  bus_outputs_t     [1:0] core_bus_outputs_i,
    output bus_inputs_t      [1:0] core_bus_inputs_o,

    // Signals between HMR unit and data cache
    input  dcache_req_t      [1:0][NumPorts-1:0] dcache_req_core2hmr_i, // dcache req signal, sent from core to the hmr unit
    output dcache_req_t      [1:0][NumPorts-1:0] dcache_req_hmr2core_o, // checked req signal, gets sent back to core
    input  dcache_rsp_t      [1:0][NumPorts-1:0] dcache_rsp_core2hmr_i, // dcache resp signal, sent from core to the hmr unit
    output dcache_rsp_t [1:0][NumPorts-1:0] dcache_rsp_hmr2core_o,
    // Dcache write buffer signals
    input logic [1:0] dcache_wbuffer_empty_core2hmr_i,
    output logic [1:0] dcache_wbuffer_empty_hmr2core_o,
    input logic [1:0] dcache_wbuffer_not_ni_core2hmr_i,
    output logic [1:0] dcache_wbuffer_not_ni_hmr2core_o,
    // AMO signals
    input ariane_pkg::amo_req_t [1:0] dcache_amo_req_core2hmr_i,
    output ariane_pkg::amo_req_t [1:0] dcache_amo_req_hmr2core_o,
    input ariane_pkg::amo_resp_t [1:0] dcache_amo_resp_core2hmr_i,
    output ariane_pkg::amo_resp_t [1:0] dcache_amo_resp_hmr2core_o,
    // D-cache flush signals
    input logic [1:0] dcache_flush_core2hmr_i,
    output logic [1:0] dcache_flush_hmr2core_o,
    input logic [1:0] dcache_flush_ack_core2hmr_i,
    output logic [1:0] dcache_flush_ack_hmr2core_o,

    // Signals between HMR unit and instruction cache
    input  icache_areq_t [1:0] icache_areq_core2hmr_i,
    output icache_areq_t [1:0] icache_areq_hmr2core_o,
    input  icache_arsp_t [1:0] icache_arsp_core2hmr_i,
    output icache_arsp_t [1:0] icache_arsp_hmr2core_o,
    input  icache_dreq_t [1:0] icache_dreq_core2hmr_i,
    output icache_dreq_t [1:0] icache_dreq_hmr2core_o,
    input  icache_drsp_t [1:0] icache_drsp_core2hmr_i,
    output icache_drsp_t [1:0] icache_drsp_hmr2core_o
);

  nominal_outputs_t                    dmr_nominal_outputs;
  ariane_pkg::amo_req_t                dmr_amo_req;
  logic                                dmr_dcache_flush;
  icache_dreq_t                        dmr_icache_dreq;
  dcache_req_t          [NumPorts-1:0] dmr_dcache_req;

  logic                                dmr_failure_main;
  logic                                dmr_failure_amo_req;
  logic                                dmr_failure_dcache_flush;
  logic                                dmr_failure_icache_dreq;
  logic                 [NumPorts-1:0] dmr_failure_dcache_req;

  logic                                dmr_failure;

  assign dmr_failure = |{
    dmr_failure_main,
    dmr_failure_amo_req,
    dmr_failure_dcache_flush,
    dmr_failure_icache_dreq,
    dmr_failure_dcache_req
    };

  // Binding DMR outputs to zero for now
  assign dmr_failure_o = (dmr_mode_active_i) ? dmr_failure : 1'b0;

  // Generate Core inputs
  for (genvar i = 0; i < 2; i++) begin : gen_core_inputs
    always_comb begin
      if (dmr_mode_active_i) begin : dmr_mode
        core_inputs_o[i] = sys_inputs_i[0];  // core[0] is always the main core
      end else begin : independent_mode
        core_inputs_o[i] = sys_inputs_i[i];
      end
    end
  end

  // Generate core outputs
  for (genvar i = 0; i < 2; i++) begin : gen_core_outputs
    always_comb begin
      if (dmr_mode_active_i) begin : dmr_mode
        if (i == 0) begin : is_dmr_main_core
          sys_nominal_outputs_o[i] = dmr_nominal_outputs;
        end else begin : disable_core  // Assign disable
          sys_nominal_outputs_o[i] = '0;
        end
      end else begin : independent_mode
        sys_nominal_outputs_o[i] = core_nominal_outputs_i[i];
      end
    end
  end

  /*********************
    * DMR Core Checkers *
    *********************/
  DMR_checker #(
      .check_bus_t(nominal_outputs_t)
  ) dmr_core_checker_main (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .inp_a_i(core_nominal_outputs_i[0]),
      .inp_b_i(core_nominal_outputs_i[1]),
      .check_o(dmr_nominal_outputs),
      .error_o(dmr_failure_main)
  );

  // Axi bus multiplexing
  // Axi bus request handling
  // Check the requests the caches made, they have to be the same
  for (genvar i = 0; i < 2; i++) begin : gen_axi_req
    always_comb begin
      if (dmr_mode_active_i) begin : dmr_mode
        if (i == 0) begin : is_dmr_main_core
          // Only send bus request from main core
          sys_bus_outputs_o[i] = core_bus_outputs_i[0];
        end else begin : disable_bus_output  // Assign disable
          sys_bus_outputs_o[i]         = '0;
          sys_bus_outputs_o[i].r_ready = 1'b1;
          sys_bus_outputs_o[i].b_ready = 1'b1;
        end
      end else begin : independent_mode
        sys_bus_outputs_o[i] = core_bus_outputs_i[i];
      end
    end
  end
  // Axi bus response handling
  for (genvar i = 0; i < 2; i++) begin : gen_axi_resp
    always_comb begin
      if (dmr_mode_active_i) begin : dmr_mode
        if (i == 0) begin : is_dmr_main_core
          // Only send bus response to cache 0
          core_bus_inputs_o[i] = sys_bus_inputs_i[0];
        end else begin : disable_bus_output  // Assign disable
          core_bus_inputs_o[i] = '0;
        end
      end else begin : independent_mode
        // Have independent responses
        core_bus_inputs_o[i] = sys_bus_inputs_i[i];
      end
    end
  end

  /*********************
    *   DMR for Cache    *
    *********************/

  // Data cache write buffer signal handling
  for (genvar i = 0; i < 2; i++) begin : gen_wbuffer_signals
    always_comb begin
      if (dmr_mode_active_i) begin : dmr_mode
        // Both cores receive the wbuffer flags from cache 0
        dcache_wbuffer_empty_hmr2core_o[i]  = dcache_wbuffer_empty_core2hmr_i[0];
        dcache_wbuffer_not_ni_hmr2core_o[i] = dcache_wbuffer_not_ni_core2hmr_i[0];
      end else begin : independent_mode
        // Wbuffer responses are independent
        dcache_wbuffer_empty_hmr2core_o[i]  = dcache_wbuffer_empty_core2hmr_i[i];
        dcache_wbuffer_not_ni_hmr2core_o[i] = dcache_wbuffer_not_ni_core2hmr_i[i];
      end
    end
  end

  // Data cache amo request and response handling
  for (genvar i = 0; i < 2; i++) begin : gen_amo_handling
    always_comb begin
      if (dmr_mode_active_i) begin : dmr_mode
        if (i == 0) begin : is_dmr_main_core
          // Only cache 0 receives the amo request
          dcache_amo_req_hmr2core_o[i] = dmr_amo_req;
        end else begin : disable_core  // Assign disable
          dcache_amo_req_hmr2core_o[i] = '0;
        end
        // Both cores receive the response from cache 0
        dcache_amo_resp_hmr2core_o[i] = dcache_amo_resp_core2hmr_i[0];
      end else begin : independent_mode
        dcache_amo_req_hmr2core_o[i]  = dcache_amo_req_core2hmr_i[i];
        dcache_amo_resp_hmr2core_o[i] = dcache_amo_resp_core2hmr_i[i];
      end
    end
  end

  // Always compare the amo request to the caches
  DMR_checker #(
      .check_bus_t(ariane_pkg::amo_req_t)
  ) dmr_amo_req_checker (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .inp_a_i(dcache_amo_req_core2hmr_i[0]),
      .inp_b_i(dcache_amo_req_core2hmr_i[1]),
      .check_o(dmr_amo_req),
      .error_o(dmr_failure_amo_req)
  );
  // Data cache flush handling
  // Only cache 0 is used
  for (genvar i = 0; i < 2; i++) begin : gen_flush_handling
    always_comb begin
      if (dmr_mode_active_i) begin : dmr_mode
        if (i == 0) begin : is_dmr_main_core
          // Only cache 0 receives the flush request
          dcache_flush_hmr2core_o[i] = dmr_dcache_flush;
        end else begin : disable_core  // Assign disable
          dcache_flush_hmr2core_o[i] = '0;
        end
        // Both cores receive the flush ack from cache 0
        dcache_flush_ack_hmr2core_o[i] = dcache_flush_ack_core2hmr_i[0];
      end else begin : independent_mode
        dcache_flush_hmr2core_o[i] = dcache_flush_core2hmr_i[i];
        dcache_flush_ack_hmr2core_o[i] = dcache_flush_ack_core2hmr_i[i];
      end
    end
  end

  DMR_checker #(
      .check_bus_t(logic)
  ) dmr_dcache_flush_checker (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .inp_a_i(dcache_flush_core2hmr_i[0]),
      .inp_b_i(dcache_flush_core2hmr_i[1]),
      .check_o(dmr_dcache_flush),
      .error_o(dmr_failure_dcache_flush)
  );

  // Data cache request and response handling

  // Generate the DMR checker for the responses and requests
  for (genvar port_index = 0; port_index < NumPorts; port_index++) begin : gen_dcache_dmr_checker
    // Always check the requests to the cache
    DMR_checker #(
        .check_bus_t(dcache_req_t)
    ) dmr_dcache_req_checker (
        .clk_i  (clk_i),
        .rst_ni (rst_ni),
        .inp_a_i(dcache_req_core2hmr_i[0][port_index]),
        .inp_b_i(dcache_req_core2hmr_i[1][port_index]),
        .check_o(dmr_dcache_req[port_index]),            // In case of error the request is dropped
        .error_o(dmr_failure_dcache_req[port_index])
    );
  end

  // Implement a DMR checker for every port to the data cache
  for (genvar port_index = 0; port_index < NumPorts; port_index++) begin : gen_port_index
    for (genvar i = 0; i < 2; i++) begin : gen_dcache_resp_req_handling
      always_comb begin
        if (dmr_mode_active_i) begin : dmr_mode
          if (i == 0) begin : is_dmr_main_core
            // Only cache 0 receives the request
            dcache_req_hmr2core_o[i][port_index] = dmr_dcache_req[port_index];
          end else begin : disable_core  // Assign disable
            dcache_req_hmr2core_o[i][port_index] = '0;
          end
          // Both cores receive the response from cache 0
          dcache_rsp_hmr2core_o[i][port_index] = dcache_rsp_core2hmr_i[0][port_index];
        end else begin : independent_mode
          // Both request and response independent
          dcache_req_hmr2core_o[i][port_index] = dcache_req_core2hmr_i[i][port_index];
          dcache_rsp_hmr2core_o[i][port_index] = dcache_rsp_core2hmr_i[i][port_index];
        end
      end
    end
  end

  // Instruction cache request and response handling
  for (genvar i = 0; i < 2; i++) begin : gen_icache_resp_req_handling
    always_comb begin
      if (dmr_mode_active_i) begin : dmr_mode
        if (i == 0) begin : is_dmr_main_core
          // Only cache 0 receives the request
          icache_dreq_hmr2core_o[i] = dmr_icache_dreq;
        end else begin : disable_core  // Assign disable
          icache_dreq_hmr2core_o[i] = '0;
        end
        // Both caches receive the response from cache 0
        icache_drsp_hmr2core_o[i] = icache_drsp_core2hmr_i[0];
      end else begin : independent_mode
        // Both request and response independent
        icache_dreq_hmr2core_o[i] = icache_dreq_core2hmr_i[i];
        icache_drsp_hmr2core_o[i] = icache_drsp_core2hmr_i[i];
      end

      // For now we ignore the address transaction requests and responses
      icache_areq_hmr2core_o[i] = icache_areq_core2hmr_i[i];
      icache_arsp_hmr2core_o[i] = icache_arsp_core2hmr_i[i];
    end
  end

  DMR_checker #(
      .check_bus_t(icache_dreq_t)
  ) dmr_icache_dreq_checker (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .inp_a_i(icache_dreq_core2hmr_i[0]),
      .inp_b_i(icache_dreq_core2hmr_i[1]),
      .check_o(dmr_icache_dreq),            // In case of error the request is dropped
      .error_o(dmr_failure_icache_dreq)
  );
endmodule
