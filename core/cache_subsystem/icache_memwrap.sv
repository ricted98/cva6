// Copyright 2025 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Riccardo Tedeschi <riccardo.tedeschi6@unibo.it>
//
// Description: I$ SRAM wrapper. Instantiates per-way tag and data sram_cache
//              macros and exposes a flat struct-based request/response
//              interface for external SRAM integration.

module icache_memwrap #(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty,
    parameter type icache_sram_req_t = logic,
    parameter type icache_sram_resp_t = logic
) (
    input logic clk_i,
    input logic rst_ni,

    input  icache_sram_req_t  req_i,
    output icache_sram_resp_t resp_o
);

  localparam int unsigned ICACHE_OFFSET_WIDTH = $clog2(CVA6Cfg.ICACHE_LINE_WIDTH / 8);
  localparam int unsigned ICACHE_NUM_WORDS    = 2 ** (CVA6Cfg.ICACHE_INDEX_WIDTH - ICACHE_OFFSET_WIDTH);

  for (genvar i = 0; i < CVA6Cfg.ICACHE_SET_ASSOC; i++) begin : gen_sram
    // Tag RAM (tag + valid bit)
    sram_cache #(
        .DATA_WIDTH (CVA6Cfg.ICACHE_TAG_WIDTH + 1),
        .BYTE_ACCESS(0),
        .TECHNO_CUT (CVA6Cfg.TechnoCut),
        .NUM_WORDS  (ICACHE_NUM_WORDS)
    ) tag_sram (
        .clk_i  (clk_i),
        .rst_ni (rst_ni),
        .req_i  (req_i.tag_req[i]),
        .we_i   (req_i.tag_we),
        .addr_i (req_i.tag_addr),
        .wuser_i('0),
        .wdata_i(req_i.tag_wdata[i]),
        .be_i   ('1),
        .ruser_o(),
        .rdata_o(resp_o.tag_rdata[i])
    );

    // Data RAM
    sram_cache #(
        .USER_WIDTH (CVA6Cfg.ICACHE_USER_LINE_WIDTH),
        .DATA_WIDTH (CVA6Cfg.ICACHE_LINE_WIDTH),
        .USER_EN    (CVA6Cfg.FETCH_USER_EN),
        .BYTE_ACCESS(0),
        .TECHNO_CUT (CVA6Cfg.TechnoCut),
        .NUM_WORDS  (ICACHE_NUM_WORDS)
    ) data_sram (
        .clk_i  (clk_i),
        .rst_ni (rst_ni),
        .req_i  (req_i.data_req[i]),
        .we_i   (req_i.data_we),
        .addr_i (req_i.data_addr),
        .wuser_i(req_i.data_wuser),
        .wdata_i(req_i.data_wdata),
        .be_i   ('1),
        .ruser_o(resp_o.data_ruser[i]),
        .rdata_o(resp_o.data_rdata[i])
    );
  end

endmodule : icache_memwrap
