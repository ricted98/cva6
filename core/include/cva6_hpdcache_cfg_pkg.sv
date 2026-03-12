// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Description: Package to compute the HPDcache configuration from the
//              CVA6 configuration. This allows any module in the hierarchy
//              to obtain HPDcacheCfg without duplicating the config logic.

package cva6_hpdcache_cfg_pkg;

  function automatic int unsigned __minu(int unsigned x, int unsigned y);
    return x < y ? x : y;
  endfunction

  function automatic int unsigned __maxu(int unsigned x, int unsigned y);
    return y < x ? x : y;
  endfunction

  // Build the hpdcache user configuration from CVA6 parameters.
  // NumPorts is the number of D$ request ports (typically 4 in CVA6).
  // HPDcache requesters layout:
  //    0: Page-Table Walk (PTW)
  //    1: Load unit
  //    2: Accelerator load
  //    3: Store/AMO
  //    .
  //    .
  //    .
  //    NumPorts: CMO
  //    NumPorts + 1: Hardware Memory Prefetcher (hwpf)
  function automatic hpdcache_pkg::hpdcache_user_cfg_t hpdcacheSetUserConfig(
      config_pkg::cva6_cfg_t CVA6Cfg,
      int NumPorts
  );
    int HPDCACHE_NREQUESTERS = NumPorts + 2;
    hpdcache_pkg::hpdcache_user_cfg_t userCfg;
    userCfg.nRequesters = HPDCACHE_NREQUESTERS;
    userCfg.paWidth = CVA6Cfg.PLEN;
    userCfg.wordWidth = CVA6Cfg.XLEN;
    userCfg.sets = CVA6Cfg.DCACHE_NUM_WORDS;
    userCfg.ways = CVA6Cfg.DCACHE_SET_ASSOC;
    userCfg.clWords = CVA6Cfg.DCACHE_LINE_WIDTH / CVA6Cfg.XLEN;
    userCfg.reqWords = 1;
    userCfg.reqTransIdWidth = CVA6Cfg.DcacheIdWidth;
    userCfg.reqSrcIdWidth = 3;  // Up to 8 requesters
    userCfg.victimSel = hpdcache_pkg::HPDCACHE_VICTIM_RANDOM;
    userCfg.dataWaysPerRamWord = __minu(CVA6Cfg.DCACHE_SET_ASSOC, 128 / CVA6Cfg.XLEN);
    userCfg.dataSetsPerRam = CVA6Cfg.DCACHE_NUM_WORDS;
    userCfg.dataRamByteEnable = 1'b1;
    userCfg.accessWords = __maxu(CVA6Cfg.AxiDataWidth / CVA6Cfg.XLEN, userCfg.reqWords);
    userCfg.mshrSets = CVA6Cfg.NrLoadBufEntries < 16 ? 1 : CVA6Cfg.NrLoadBufEntries / 2;
    userCfg.mshrWays = CVA6Cfg.NrLoadBufEntries < 16 ? CVA6Cfg.NrLoadBufEntries : 2;
    userCfg.mshrWaysPerRamWord = CVA6Cfg.NrLoadBufEntries < 16 ? CVA6Cfg.NrLoadBufEntries : 2;
    userCfg.mshrSetsPerRam = CVA6Cfg.NrLoadBufEntries < 16 ? 1 : CVA6Cfg.NrLoadBufEntries / 2;
    userCfg.mshrRamByteEnable = 1'b1;
    userCfg.mshrUseRegbank = (CVA6Cfg.NrLoadBufEntries < 16);
    /*FIXME we should add additional CVA6 config parameters (cbufEntries)*/
    userCfg.cbufEntries = 4;
    userCfg.refillCoreRspFeedthrough = 1'b1;
    userCfg.refillFifoDepth = 2 * (CVA6Cfg.DCACHE_LINE_WIDTH / CVA6Cfg.AxiDataWidth);
    userCfg.wbufDirEntries = CVA6Cfg.WtDcacheWbufDepth;
    userCfg.wbufDataEntries = CVA6Cfg.WtDcacheWbufDepth;
    userCfg.wbufWords = 1;
    userCfg.wbufTimecntWidth = 3;
    userCfg.rtabEntries = 4;
    /*FIXME we should add additional CVA6 config parameters (flushEntries)*/
    userCfg.flushEntries = CVA6Cfg.WtDcacheWbufDepth;
    /*FIXME we should add additional CVA6 config parameters (flushFifoDepth)*/
    userCfg.flushFifoDepth = CVA6Cfg.WtDcacheWbufDepth;
    userCfg.memAddrWidth = CVA6Cfg.AxiAddrWidth;
    userCfg.memIdWidth = CVA6Cfg.MEM_TID_WIDTH;
    userCfg.memDataWidth = CVA6Cfg.AxiDataWidth;
    userCfg.wtEn =
        (CVA6Cfg.DCacheType == config_pkg::HPDCACHE_WT) ||
        (CVA6Cfg.DCacheType == config_pkg::HPDCACHE_WT_WB);
    userCfg.wbEn =
        (CVA6Cfg.DCacheType == config_pkg::HPDCACHE_WB) ||
        (CVA6Cfg.DCacheType == config_pkg::HPDCACHE_WT_WB);
    userCfg.lowLatency = 1'b1;
    userCfg.externalSram = 1'b1;
    return userCfg;
  endfunction

  // Convenience function: compute the full HPDcacheCfg in one call.
  function automatic hpdcache_pkg::hpdcache_cfg_t hpdcacheBuildCfg(
      config_pkg::cva6_cfg_t CVA6Cfg,
      int NumPorts
  );
    return hpdcache_pkg::hpdcacheBuildConfig(hpdcacheSetUserConfig(CVA6Cfg, NumPorts));
  endfunction

endpackage
