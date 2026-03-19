// Copyright 2026 ETH Zurich, University of Bologna
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//      https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package cva6_hpdcache_subsystem_pkg;

  function automatic int unsigned __minu(int unsigned x, int unsigned y);
    return x < y ? x : y;
  endfunction

  function automatic int unsigned __maxu(int unsigned x, int unsigned y);
    return y < x ? x : y;
  endfunction

  //    0: Page-Table Walk (PTW)
  //    1: Load unit
  //    2: Accelerator load
  //    3: Store/AMO
  //    .
  //    .
  //    .
  //    NumPorts: CMO
  //    NumPorts + 1: Hardware Memory Prefetcher (hwpf)
  localparam int unsigned HPDCACHE_EXTRA_REQUESTERS = 2;

  function automatic hpdcache_pkg::hpdcache_user_cfg_t hpdcacheSetUserConfig(
      config_pkg::cva6_cfg_t CVA6Cfg,
      int unsigned NumPorts,
      logic externalSram = 1'b0
  );
    hpdcache_pkg::hpdcache_user_cfg_t userCfg;
    userCfg.nRequesters = NumPorts + HPDCACHE_EXTRA_REQUESTERS;
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
    userCfg.eccEn = 1'b0;
    userCfg.eccScrubberEn = 1'b0;
    userCfg.externalSram = externalSram;
    return userCfg;
  endfunction

  function automatic hpdcache_pkg::hpdcache_cfg_t hpdcacheBuildCfg(
      config_pkg::cva6_cfg_t CVA6Cfg,
      int unsigned NumPorts,
      logic externalSram = 1'b0
  );
    return hpdcache_pkg::hpdcacheBuildConfig(
        hpdcacheSetUserConfig(CVA6Cfg, NumPorts, externalSram)
    );
  endfunction

endpackage : cva6_hpdcache_subsystem_pkg
