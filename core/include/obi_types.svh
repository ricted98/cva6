// Copyright 2025 Thales DIS France SAS
//
// Licensed under the Solderpad Hardware Licence, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.0
// You may obtain a copy of the License at https://solderpad.org/licenses/
//
// Description: CVA6-specific helpers on top of the generic `OBI_DECL_*` macros
//              provided by pulp-platform/obi. They shape an obi_pkg::obi_cfg_t
//              into the anonymous structs, so that a whole OBI bus can be
//              declared in one line, either in a parameter port list or in a
//              module body.

`ifndef OBI_CVA6_TYPES_SVH
`define OBI_CVA6_TYPES_SVH

`include "obi/typedef.svh"

// Config-shaped wrappers around the generic obi `OBI_DECL_*` macros.
`define OBI_CFG_ALL_A_OPTIONAL_T(cfg) \
  `OBI_DECL_ALL_A_OPTIONAL(cfg.OptionalCfg.AUserWidth, cfg.OptionalCfg.WUserWidth, cfg.OptionalCfg.MidWidth, cfg.OptionalCfg.AChkWidth)
`define OBI_CFG_A_CHAN_T(cfg, a_optional_t) \
  `OBI_DECL_A_CHAN_T(cfg.AddrWidth, cfg.DataWidth, cfg.IdWidth, a_optional_t)
`define OBI_CFG_INTEGRITY_REQ_T(a_chan_t) \
  `OBI_DECL_INTEGRITY_REQ_T(a_chan_t)
`define OBI_CFG_ALL_R_OPTIONAL_T(cfg) \
  `OBI_DECL_ALL_R_OPTIONAL(cfg.OptionalCfg.RUserWidth, cfg.OptionalCfg.RChkWidth)
`define OBI_CFG_R_CHAN_T(cfg, r_optional_t) \
  `OBI_DECL_R_CHAN_T(cfg.DataWidth, cfg.IdWidth, r_optional_t)
`define OBI_CFG_INTEGRITY_RSP_T(r_chan_t) \
  `OBI_DECL_INTEGRITY_RSP_T(r_chan_t)

// Declare the six types of one OBI bus in one go. `keyword` selects the
// declaration kind and `separator` how the declarations are joined, so that the
// same body serves both a parameter port list (comma) and a module body
// (semicolon).
`define OBI_GENERIC_PARAM_TYPE_ALL(obi_t, cfg, keyword, separator)                          \
  keyword type obi_t``_a_optional_t = `OBI_CFG_ALL_A_OPTIONAL_T(cfg) separator              \
  keyword type obi_t``_a_chan_t = `OBI_CFG_A_CHAN_T(cfg, obi_t``_a_optional_t) separator    \
  keyword type obi_t``_req_t = `OBI_CFG_INTEGRITY_REQ_T(obi_t``_a_chan_t) separator         \
  keyword type obi_t``_r_optional_t = `OBI_CFG_ALL_R_OPTIONAL_T(cfg) separator              \
  keyword type obi_t``_r_chan_t = `OBI_CFG_R_CHAN_T(cfg, obi_t``_r_optional_t) separator    \
  keyword type obi_t``_rsp_t = `OBI_CFG_INTEGRITY_RSP_T(obi_t``_r_chan_t)

`define OBI_SEPARATOR_COMMA ,
`define OBI_SEPARATOR_SEMICOLON ;

// For a parameter port list: no trailing separator, caller adds the comma.
`define OBI_PARAM_TYPE_GLOBAL_ALL(obi_t, cfg) \
  `OBI_GENERIC_PARAM_TYPE_ALL(obi_t, cfg, parameter, `OBI_SEPARATOR_COMMA)
`define OBI_LOCALPARAM_TYPE_GLOBAL_ALL(obi_t, cfg) \
  `OBI_GENERIC_PARAM_TYPE_ALL(obi_t, cfg, parameter, `OBI_SEPARATOR_COMMA)
// For a module body: no trailing semicolon, caller adds it.
`define OBI_LOCALPARAM_TYPE_ALL(obi_t, cfg) \
  `OBI_GENERIC_PARAM_TYPE_ALL(obi_t, cfg, localparam, `OBI_SEPARATOR_SEMICOLON)

`endif  // OBI_CVA6_TYPES_SVH
