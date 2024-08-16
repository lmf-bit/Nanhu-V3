/***************************************************************************************
* Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
* Copyright (c) 2020-2021 Peng Cheng Laboratory
*
* XiangShan is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/

package xiangshan.mem

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import utils._
import xs.utils._
import xiangshan.ExceptionNO._
import xiangshan._
import xiangshan.backend.execute.fu.FuConfigs.lduCfg
import xiangshan.backend.execute.fu._
import xiangshan.backend.execute.fu.csr.SdtrigExt
import xiangshan.backend.issue.{EarlyWakeUpInfo, RSFeedback, RSFeedbackType, RsIdx}
import xiangshan.backend.rob.RobPtr
import xiangshan.cache._
import xiangshan.cache.mmu.{TlbCmd, TlbReq, TlbRequestIO, TlbResp}
import xs.utils.perf.HasPerfLogging

class LoadToLsqIO(implicit p: Parameters) extends XSBundle {
  val s1_lduMMIOPAddr = ValidIO(new LoadMMIOPaddrWriteBundle)
  val s2_excepWb2LQ = ValidIO(new LqWriteBundle)
  val s2_queryAndUpdateLQ = ValidIO(new LoadQueueDataUpdateBundle)

  val forwardFromSQ = new PipeLoadForwardFromSQ
  val loadViolationQuery = new LoadViolationQueryIO
  val trigger = Flipped(new LqTriggerIO)
}

class LoadUnitTriggerIO(implicit p: Parameters) extends XSBundle {
  val tdata2 = Input(UInt(64.W))
  val matchType = Input(UInt(2.W))
  val tEnable = Input(Bool()) // timing is calculated before this
  val addrHit = Output(Bool())
  val lastDataHit = Output(Bool())
}

class LoadUnit(implicit p: Parameters) extends XSModule
  with HasLoadHelper
  with HasPerfEvents
  with HasDCacheParameters
  with SdtrigExt
  with HasPerfLogging
  with HasL1PrefetchSourceParameter
  with HasCircularQueuePtrHelper {
  val io = IO(new Bundle() {

    // S0: reservationStation issueIn
    val rsIssueIn = Flipped(DecoupledIO(new ExuInput))
    val rsIdx = Input(new RsIdx)
    val isFirstIssue = Input(Bool())
    // S0: replayQueue issueIn
    val replayQIssueIn = Flipped(DecoupledIO(new ReplayQueueIssueBundle))
    // S0: fastReplay from LoadS1
    val fastReplayIn = Flipped(DecoupledIO(new LsPipelineBundle))
    // S0: specialLoad for timing
    val auxValid = Input(Bool())
    val vmEnable = Input(Bool())
    // S0/S1: tlb query and response in next cycle
    val tlb = new TlbRequestIO(nRespDups=2)

    // S1: fastReplay to LoadS0
    val fastReplayOut = DecoupledIO(new LsPipelineBundle)
    // S1/S2: cache query and response in next cycle
    val dcache = new DCacheLoadIO
    val lduForwardMSHR = new LduForwardFromMSHR
    // S1/S2: forward query to sbuffer and response in next cycle
    val forwardFromSBuffer = new LoadForwardQueryIO
    // S1/S2: FDI req and response in next cycle
    val fdiReq = ValidIO(new  FDIReqBundle())
    val fdiResp = Flipped(new FDIRespBundle())
    // S1/S2/S3 : forward query to lsq, update lsq, writeback from lsq
    val lsq = new LoadToLsqIO

    // S2: mshrID which handled load miss req
    val loadReqHandledResp = Flipped(ValidIO(UInt(log2Up(cfg.nMissEntries).W)) )
    // S2: pmp query response
    val pmp = Flipped(new PMPRespBundle()) // arrive same to tlb now
    // S2: preftech train output
    val prefetch_train = ValidIO(new LsPipelineBundle())
    val prefetch_train_l1 = ValidIO(new LsPipelineBundle())
    val hit_prefetch = Output(Bool())

    // S2: load enq LoadRAWQueue
    val enqRAWQueue = new LoadEnqRAWBundle
    // S2,S3: store violation query
    val storeViolationQuery = Vec(StorePipelineWidth, Flipped(Valid(new storeRAWQueryBundle)))
    // S3: feedback reservationStation to replay
    val feedbackSlow = ValidIO(new RSFeedback)
    val feedbackFast = ValidIO(new RSFeedback) // todo: will be deleted soon
    // S3: replay inst enq replayQueue
    val s3_enq_replayQueue = DecoupledIO(new LoadToReplayQueueBundle)
    // S3: load writeback
    val ldout = Decoupled(new ExuOutput)
    // S3: mmio writeback
    val mmioWb = Flipped(DecoupledIO(new ExuOutput))

    // Global: redirect flush all pipeline
    val redirect = Flipped(ValidIO(new Redirect))
    // Global: debug trigger
    val trigger = Vec(TriggerNum, new LoadUnitTriggerIO)
    // Global: csr control
    val csrCtrl = Flipped(new CustomCSRCtrlIO)
    // Load EarlyWakeUp
    val earlyWakeUp = Output(new Bundle() {
      val cancel = Bool() //s2 cancel
      val wakeUp = Valid(new EarlyWakeUpInfo) //s1 wakeup
    })
  })

  //redirect register fanout
  private val redirectUseName = List("loadS0",  "loadS1",  "loadS2",  "loadS3", "fastRep")
  private val redirectReg = RedirectRegDup(redirectUseName,io.redirect)

  /*
    LOAD S0: arb 2 input; generate vaddr; req to TLB
  */
  val fastReplayIn = io.fastReplayIn
  val rsIssueIn = io.rsIssueIn
  val replayIssueIn = io.replayQIssueIn
  assert(!(fastReplayIn.fire && rsIssueIn.fire ||
    fastReplayIn.fire && replayIssueIn.fire ||
    rsIssueIn.fire && replayIssueIn.fire) ,"3 input port can't fire same time")
  /*
    3 ports ready always true, use ldStop to block rs, use replayStop to block replayQ
    fastRep > replayQ > rsIssue
  */
  io.rsIssueIn.ready := true.B
  io.replayQIssueIn.ready := true.B
  io.fastReplayIn.ready := true.B
  io.dcache.pf_can_use_mqArb := !fastReplayIn.valid && !replayIssueIn.valid

  val s0_rsIssue = Wire(new LoadPipelineBundleS0)
  s0_rsIssue.uop := rsIssueIn.bits.uop
  s0_rsIssue.vm := rsIssueIn.bits.vm
  s0_rsIssue.rsIdx := io.rsIdx
  s0_rsIssue.isFirstIssue := io.isFirstIssue
  s0_rsIssue.vaddr := rsIssueIn.bits.src(0) + SignExt(rsIssueIn.bits.uop.ctrl.imm(11,0), VAddrBits)
  s0_rsIssue.replayCause.foreach(_ := false.B)
  s0_rsIssue.schedIndex := 0.U
  s0_rsIssue.isReplayQReplay := false.B
  s0_rsIssue.debugCause := 0.U

  val s0_replayQIssue = Wire(new LoadPipelineBundleS0)
  s0_replayQIssue.uop := replayIssueIn.bits.uop
  s0_replayQIssue.vm := 0.U
  s0_replayQIssue.rsIdx := DontCare
  s0_replayQIssue.isFirstIssue := false.B
  s0_replayQIssue.vaddr := replayIssueIn.bits.vaddr
  s0_replayQIssue.replayCause.foreach(_ := false.B)
  s0_replayQIssue.schedIndex := replayIssueIn.bits.schedIndex
  s0_replayQIssue.isReplayQReplay := true.B
  s0_replayQIssue.debugCause := replayIssueIn.bits.debugCause

  val s0_fastRepIssue = Wire(new LoadPipelineBundleS0)
  s0_fastRepIssue.uop := fastReplayIn.bits.uop
  s0_fastRepIssue.vm := 0.U
  s0_fastRepIssue.rsIdx := fastReplayIn.bits.rsIdx
  s0_fastRepIssue.isFirstIssue := false.B
  s0_fastRepIssue.vaddr := fastReplayIn.bits.vaddr
  s0_fastRepIssue.replayCause.foreach(_ := false.B)
  s0_fastRepIssue.schedIndex := fastReplayIn.bits.replay.schedIndex
  s0_fastRepIssue.isReplayQReplay := fastReplayIn.bits.replay.isReplayQReplay
  s0_fastRepIssue.debugCause := (1<<(LoadReplayCauses.C_FR-1)).U
  dontTouch(s0_fastRepIssue)
  val s0_src_selector = Seq(
    fastReplayIn.valid,
    replayIssueIn.valid,
    rsIssueIn.valid)
  val s0_src = Seq(
    s0_fastRepIssue,
    s0_replayQIssue,
    s0_rsIssue
  )
  val s0_sel_src = Wire(new LoadPipelineBundleS0)
  s0_sel_src := ParallelPriorityMux(s0_src_selector, s0_src)

  val s0_imm12 = s0_sel_src.uop.ctrl.imm(11,0)
  val s0_vaddr = s0_sel_src.vaddr
  val s0_mask  = WireInit(genWmask(s0_vaddr, s0_sel_src.uop.ctrl.fuOpType(1,0)))
  val s0_uop   = WireInit(s0_sel_src.uop)
  val s0_auxValid = s0_src_selector.reduce(_ || _)
  val s0_valid = s0_src_selector.reduce(_ || _)
  val s0_EnableMem = s0_sel_src.uop.loadStoreEnable

  val s0_req_tlb = io.tlb.req
  s0_req_tlb := DontCare
  s0_req_tlb.valid := s0_auxValid
  s0_req_tlb.bits.vaddr := s0_vaddr
  s0_req_tlb.bits.cmd := TlbCmd.read
  s0_req_tlb.bits.size := LSUOpType.size(s0_uop.ctrl.fuOpType)
  s0_req_tlb.bits.robIdx := s0_uop.robIdx
  s0_req_tlb.bits.debug.pc := s0_uop.cf.pc

  val s0_req_dcache = io.dcache.req
  val s0_isSoftPrefetch = Mux(replayIssueIn.valid,false.B,LSUOpType.isPrefetch(s0_uop.ctrl.fuOpType))
  val s0_isSoftPrefetchRead = s0_uop.ctrl.fuOpType === LSUOpType.prefetch_r
  val s0_isSoftPrefetchWrite = s0_uop.ctrl.fuOpType === LSUOpType.prefetch_w
  s0_req_dcache.valid := s0_auxValid
  when (s0_isSoftPrefetchRead) {
    s0_req_dcache.bits.cmd  := MemoryOpConstants.M_PFR
  }.elsewhen (s0_isSoftPrefetchWrite) {
    s0_req_dcache.bits.cmd  := MemoryOpConstants.M_PFW
  }.otherwise {
    s0_req_dcache.bits.cmd  := MemoryOpConstants.M_XRD
  }
  s0_req_dcache.bits.robIdx := s0_sel_src.uop.robIdx
  s0_req_dcache.bits.addr := s0_vaddr
  s0_req_dcache.bits.mask := s0_mask
  s0_req_dcache.bits.data := DontCare
  s0_req_dcache.bits.instrtype := Mux(s0_isSoftPrefetch,SOFT_PREFETCH.U,LOAD_SOURCE.U)
  s0_req_dcache.bits.id := DontCare
  val s0_cancel = !s0_req_dcache.ready

  val s0_addrAligned = LookupTree(s0_uop.ctrl.fuOpType(1, 0), List(
    "b00".U   -> true.B,                   //b
    "b01".U   -> (s0_vaddr(0)    === 0.U), //h
    "b10".U   -> (s0_vaddr(1, 0) === 0.U), //w
    "b11".U   -> (s0_vaddr(2, 0) === 0.U)  //d
  ))

  val s0_out = Wire(Decoupled(new LsPipelineBundle))
  s0_out.valid := s0_valid
  s0_out.bits := DontCare
  s0_out.bits.replay.schedIndex := s0_sel_src.schedIndex
  s0_out.bits.replay.isReplayQReplay := s0_sel_src.isReplayQReplay
  s0_out.bits.replay.schedIndex := s0_sel_src.schedIndex
  s0_out.bits.vaddr := s0_vaddr
  s0_out.bits.mask := s0_mask
  s0_out.bits.uop := s0_uop

  private val s0_vaddr2 = SignExt(s0_sel_src.vaddr, XLEN)
  dontTouch(s0_vaddr)
  private val illegalAddr = s0_vaddr2(XLEN - 1, VAddrBits - 1) =/= 0.U && s0_vaddr2(XLEN - 1, VAddrBits - 1) =/= Fill(XLEN - VAddrBits + 1, 1.U(1.W))
  s0_out.bits.uop.cf.exceptionVec(loadAddrMisaligned) := !s0_addrAligned && s0_EnableMem && !s0_isSoftPrefetch
  s0_out.bits.uop.cf.exceptionVec(loadPageFault) := illegalAddr && s0_EnableMem & io.vmEnable && !s0_isSoftPrefetch
  s0_out.bits.rsIdx := s0_sel_src.rsIdx
  s0_out.bits.isFirstIssue := s0_sel_src.isFirstIssue
  s0_out.bits.replay.replayCause.foreach(_ := false.B)
  s0_out.bits.isSoftPrefetch := s0_isSoftPrefetch
  s0_out.bits.debugCause := s0_sel_src.debugCause

  /*
    LOAD S1: process TLB response data; sbuffer/lsq forward query; ld-ld violation query
  */
  val s1_in = Wire(Decoupled(new LsPipelineBundle))
  val s1_out = Wire(Decoupled(new LsPipelineBundle))

  PipelineConnect(s0_out, s1_in, true.B, s0_out.bits.uop.robIdx.needFlush(redirectReg("loadS1")))

  s1_out.bits := s1_in.bits // todo: replace this way of coding!
  //store load violation from storeUnit S1
  val s1_stldViolationVec = Wire(Vec(StorePipelineWidth, Bool()))
  s1_stldViolationVec := io.storeViolationQuery.map({ case req =>
    s1_out.valid && req.valid &&
    isAfter(s1_out.bits.uop.robIdx, req.bits.robIdx) &&
    s1_out.bits.paddr(PAddrBits - 1, 3) === req.bits.paddr &&
      (s1_out.bits.mask & req.bits.mask).orR
  })
  val s1_hasStLdViolation = s1_stldViolationVec.reduce(_ | _)
  dontTouch(s1_hasStLdViolation)

  val s1_dtlbResp = io.tlb.resp
  s1_dtlbResp.ready := true.B

  val s1_uop = s1_in.bits.uop
  val s1_paddr_dup_lsu = s1_dtlbResp.bits.paddr(0)
  val s1_paddr_dup_dcache = s1_dtlbResp.bits.paddr(1)
  io.dcache.s1_paddr_dup_lsu := s1_paddr_dup_lsu
  io.dcache.s1_paddr_dup_dcache := s1_paddr_dup_dcache

  val s1_enableMem = s1_in.bits.uop.loadStoreEnable
  val s1_hasException = Mux(s1_enableMem && s1_in.valid, ExceptionNO.selectByFu(s1_out.bits.uop.cf.exceptionVec, lduCfg).asUInt.orR, false.B)
  val s1_tlb_miss = s1_dtlbResp.bits.miss
  val s1_bank_conflict = io.dcache.s1_bank_conflict
  val s1_cancel_inner = RegEnable(s0_cancel,s0_out.fire)
  val s1_isSoftPrefetch = s1_in.bits.isSoftPrefetch
  val s1_dcacheKill = s1_in.valid && (s1_tlb_miss || s1_hasException || s1_cancel_inner || (!s1_enableMem))
  io.dcache.s1_kill := s1_dcacheKill

  val s1_sbufferForwardReq = io.forwardFromSBuffer
  val s1_lsqForwardReq = io.lsq.forwardFromSQ
  val s1_MSHRForwardReq = io.lduForwardMSHR.req
  val s1_fdiReq = io.fdiReq

  s1_MSHRForwardReq.valid := s1_in.valid && !(s1_hasException || s1_tlb_miss) && s1_enableMem
  s1_MSHRForwardReq.bits := s1_paddr_dup_lsu

  s1_sbufferForwardReq.valid := s1_in.valid && !(s1_hasException || s1_tlb_miss) && s1_enableMem
  s1_sbufferForwardReq.vaddr := s1_in.bits.vaddr
  s1_sbufferForwardReq.paddr := s1_paddr_dup_lsu
  s1_sbufferForwardReq.uop := s1_uop
  s1_sbufferForwardReq.sqIdx := s1_uop.sqIdx
  s1_sbufferForwardReq.mask := s1_in.bits.mask
  s1_sbufferForwardReq.pc := s1_uop.cf.pc

  s1_lsqForwardReq.valid := s1_in.valid && !(s1_hasException || s1_tlb_miss) && s1_enableMem
  s1_lsqForwardReq.vaddr := s1_in.bits.vaddr
  s1_lsqForwardReq.paddr := s1_paddr_dup_lsu
  s1_lsqForwardReq.uop := s1_uop
  s1_lsqForwardReq.sqIdx := s1_uop.sqIdx
  s1_lsqForwardReq.sqIdxMask := DontCare
  s1_lsqForwardReq.mask := s1_in.bits.mask
  s1_lsqForwardReq.pc := s1_uop.cf.pc
  io.lsq.forwardFromSQ.sqIdxMask := UIntToMask(s1_uop.sqIdx.value, StoreQueueSize)

  s1_fdiReq.valid := s1_out.fire
  s1_fdiReq.bits.addr := s1_out.bits.vaddr
  s1_fdiReq.bits.inUntrustedZone := s1_out.bits.uop.fdiUntrusted
  s1_fdiReq.bits.operation := FDIOp.read

  /* Generate feedback signal caused by:    1.dcache bank conflict    2.need redo ld-ld violation check */
  val s1_csrCtrl_ldld_vio_check_enable = io.csrCtrl.ldld_vio_check_enable
//  val s1_needLdVioCheckRedo = s1_ldViolationQueryReq.valid && !s1_ldViolationQueryReq.ready &&
//    RegNext(s1_csrCtrl_ldld_vio_check_enable)
  val s1_needLdVioCheckRedo = false.B

  s1_out.valid        := s1_in.valid && s1_enableMem
  s1_out.bits.paddr   := s1_paddr_dup_lsu
  s1_out.bits.tlbMiss := s1_tlb_miss
  when(!s1_tlb_miss){
    s1_out.bits.uop.cf.exceptionVec(loadPageFault) := (s1_dtlbResp.bits.excp(0).pf.ld || s1_in.bits.uop.cf.exceptionVec(loadPageFault)) && s1_enableMem && !s1_isSoftPrefetch
    s1_out.bits.uop.cf.exceptionVec(loadAccessFault) := (s1_dtlbResp.bits.excp(0).af.ld || s1_in.bits.uop.cf.exceptionVec(loadAccessFault)) && s1_enableMem && !s1_isSoftPrefetch
  }
  s1_out.bits.ptwBack := s1_dtlbResp.bits.ptwBack
  s1_out.bits.rsIdx   := s1_in.bits.rsIdx
  s1_out.bits.isSoftPrefetch := s1_isSoftPrefetch
  s1_in.ready := !s1_in.valid || s1_out.ready

  val s1_causeReg = WireInit(0.U.asTypeOf(new ReplayInfo))
  val s1_cause_can_transfer = !(ExceptionNO.selectByFu(s1_out.bits.uop.cf.exceptionVec, lduCfg).asUInt.orR) && (!s1_isSoftPrefetch) && s1_enableMem
  s1_causeReg.schedIndex := s1_out.bits.replay.schedIndex
  s1_causeReg.isReplayQReplay := s1_out.bits.replay.isReplayQReplay
  s1_causeReg.full_fwd := false.B
  s1_causeReg.fwd_data_sqIdx := 0.U.asTypeOf(new SqPtr)
  s1_causeReg.tlb_miss := s1_tlb_miss  // tlb resp miss
  s1_causeReg.raw_nack := false.B
  s1_causeReg.raw_violation := s1_hasStLdViolation
  s1_causeReg.bank_conflict := false.B  // bankConflict & DcacheNotReady go fastReplay
  dontTouch(s1_causeReg)

  io.earlyWakeUp.wakeUp.valid := s1_in.valid && !s1_causeReg.need_rep
//  io.earlyWakeUp.wakeUp.valid := false.B
  io.earlyWakeUp.wakeUp.bits.lpv := "b00010".U
  io.earlyWakeUp.wakeUp.bits.pdest := s1_in.bits.uop.pdest
  io.earlyWakeUp.wakeUp.bits.destType := MuxCase(SrcType.default, Seq(
    s1_in.bits.uop.ctrl.rfWen -> SrcType.reg,
    s1_in.bits.uop.ctrl.fpWen -> SrcType.fp,
  ))
  io.earlyWakeUp.wakeUp.bits.robPtr := s1_in.bits.uop.robIdx

  /*
    LOAD S2: cache miss control; forward data merge; mmio check; feedback to reservationStation
  */
  val s2_in = Wire(Decoupled(new LsPipelineBundle))
  val s2_out = Wire(Decoupled(new LsPipelineBundle))
  PipelineConnect(s1_out, s2_in, true.B, s1_out.bits.uop.robIdx.needFlush(redirectReg("loadS1")))

  s2_out.valid := s2_in.valid && !s2_in.bits.uop.robIdx.needFlush(redirectReg("loadS2"))
  s2_out.bits := s2_in.bits

  val s2_tlb_miss = s2_in.bits.tlbMiss
  //store load violation from storeUnit S1
  val s2_stldViolationVec = Wire(Vec(StorePipelineWidth, Bool()))
  s2_stldViolationVec := io.storeViolationQuery.map({ case req =>
    s2_in.valid && req.valid &&
      isAfter(s2_in.bits.uop.robIdx, req.bits.robIdx) &&
      s2_in.bits.paddr(PAddrBits - 1, 3) === req.bits.paddr &&
      (s2_in.bits.mask & req.bits.mask).orR
  })
  val s2_hasStLdViolation = s2_stldViolationVec.reduce(_ | _)
  val s2_enqRAWFail = io.enqRAWQueue.s2_enq.valid && !io.enqRAWQueue.s2_enqSuccess
  val s2_allStLdViolation = s2_hasStLdViolation | RegNext(s1_hasStLdViolation,false.B)
  dontTouch(s2_hasStLdViolation)

  val pmp = WireInit(io.pmp)

  val s2_cancel_inner = RegEnable(s1_cancel_inner,s1_out.fire)
  val s2_enableMem = s2_in.bits.uop.loadStoreEnable && s2_in.valid
  val s2_isSoftPrefetch = s2_in.bits.isSoftPrefetch
  when(!s2_tlb_miss){
    s2_out.bits.uop.cf.exceptionVec(loadAccessFault) := (s2_in.bits.uop.cf.exceptionVec(loadAccessFault) || pmp.ld) && s2_enableMem && !s2_isSoftPrefetch
  }
  //don't need tlb
  s2_out.bits.uop.cf.exceptionVec(fdiULoadAccessFault) := (io.fdiResp.fdi_fault === FDICheckFault.UReadDascisFault) && s2_enableMem  //FDI load access fault

  val s2_hasException = Mux(s2_enableMem, ExceptionNO.selectByFu(s2_out.bits.uop.cf.exceptionVec, lduCfg).asUInt.orR,false.B)
  val s2_dcacheResp = io.dcache.resp
  val s2_dcacheMshrID = io.loadReqHandledResp
  dontTouch(s2_dcacheMshrID)

  val s2_LSQ_LoadForwardQueryIO = Wire(new LoadForwardQueryIO)
  val s2_SB_LoadForwardQueryIO = Wire(new LoadForwardQueryIO)
  val s2_MSHR_LoadForwardData = Wire(new LoadForwardQueryIO)
//  val s2_loadViolationQueryResp = Wire(ValidIO(new LoadViolationQueryResp))

  s2_LSQ_LoadForwardQueryIO := DontCare
  s2_SB_LoadForwardQueryIO := DontCare
  s2_MSHR_LoadForwardData := DontCare
  s2_dcacheResp.ready := true.B

  private val dataMSHRSplit = Wire(Vec(8, UInt(8.W)))
  for(i <- 0 until 8){
    dataMSHRSplit(i) := io.lduForwardMSHR.resp.bits(8 * (i + 1) - 1, 8 * i)
  }
  s2_MSHR_LoadForwardData.forwardData := dataMSHRSplit
  s2_MSHR_LoadForwardData.forwardMask := Fill(8, io.lduForwardMSHR.resp.valid).asBools

  s2_LSQ_LoadForwardQueryIO.forwardData := io.lsq.forwardFromSQ.forwardData
  s2_LSQ_LoadForwardQueryIO.forwardMask := io.lsq.forwardFromSQ.forwardMask
  s2_LSQ_LoadForwardQueryIO.dataInvalid := io.lsq.forwardFromSQ.dataInvalid
  s2_LSQ_LoadForwardQueryIO.matchInvalid := io.lsq.forwardFromSQ.matchInvalid

  s2_SB_LoadForwardQueryIO.forwardData := io.forwardFromSBuffer.forwardData
  s2_SB_LoadForwardQueryIO.forwardMask := io.forwardFromSBuffer.forwardMask
  s2_SB_LoadForwardQueryIO.dataInvalid := io.forwardFromSBuffer.dataInvalid // always false
  s2_SB_LoadForwardQueryIO.matchInvalid := io.forwardFromSBuffer.matchInvalid

//  s2_loadViolationQueryResp := io.lsq.loadViolationQuery.s2_resp

  val s2_actually_mmio = pmp.mmio && !s2_tlb_miss
  val s2_mmio = !s2_isSoftPrefetch && s2_actually_mmio && !s2_hasException
  val s2_cache_miss = s2_dcacheResp.bits.miss && s2_dcacheResp.valid
  val s2_bank_conflict = RegNext(s1_bank_conflict)
  val s2_cache_replay = s2_dcacheResp.bits.replay
  val s2_cache_replay_with_mshrFull = s2_dcacheResp.bits.replay && s2_dcacheResp.bits.mshr_full
  val s2_cache_replay_with_conflict = s2_dcacheResp.bits.replay && s2_dcacheResp.bits.miss_req_enq_confilct
  val s2_cache_replay_with_reject = s2_dcacheResp.bits.replay && (!s2_dcacheResp.bits.mshr_full && !s2_dcacheResp.bits.miss_req_enq_confilct) 

//  val s2_ldld_violation = s2_loadViolationQueryResp.valid && s2_loadViolationQueryResp.bits.have_violation &&
//    RegNext(io.csrCtrl.ldld_vio_check_enable)
  val s2_ldld_violation = false.B
  val s2_data_invalid = s2_LSQ_LoadForwardQueryIO.dataInvalid && !s2_ldld_violation && !s2_hasException

  val s2_dcache_kill = pmp.ld || pmp.mmio // move pmp resp kill to outside
  // to kill mmio resp which are redirected
  io.dcache.s2_kill := s2_dcache_kill

  val s2_dcacheShouldResp = !(s2_tlb_miss || s2_hasException || s2_mmio || s2_isSoftPrefetch)
  assert(!(s2_enableMem && (!s2_cancel_inner && s2_dcacheShouldResp && !s2_dcacheResp.valid)), "DCache response got lost")

  // merge forward result, lsq has higher priority than sbuffer
  val s2_forwardMask = Wire(Vec(8, Bool()))
  val s2_forwardData = Wire(Vec(8, UInt(8.W)))
  for (i <- 0 until XLEN / 8) {
    s2_forwardMask(i) := s2_LSQ_LoadForwardQueryIO.forwardMask(i) || s2_SB_LoadForwardQueryIO.forwardMask(i) || s2_MSHR_LoadForwardData.forwardMask(i)
    s2_forwardData(i) := Mux(s2_LSQ_LoadForwardQueryIO.forwardMask(i), s2_LSQ_LoadForwardQueryIO.forwardData(i),
                                                                      Mux(s2_SB_LoadForwardQueryIO.forwardMask(i), s2_SB_LoadForwardQueryIO.forwardData(i),
                                                                                                                  s2_MSHR_LoadForwardData.forwardData(i)))
  }
  val s2_fullForward = s2_out.valid && !s2_tlb_miss && ((~s2_forwardMask.asUInt).asUInt & s2_in.bits.mask) === 0.U && !s2_LSQ_LoadForwardQueryIO.dataInvalid

  dontTouch(s2_fullForward)
  val s2_dataFromDCache = s2_out.valid && !s2_fullForward && !s2_cache_miss

  // dcache load data
  val s2_loadDataFromDcache = Wire(new LoadDataFromDcacheBundle)
  s2_loadDataFromDcache.load_data := Mux(s2_dcacheResp.bits.miss, 0.U, s2_dcacheResp.bits.load_data) //to cut X-prop
  s2_loadDataFromDcache.forwardMask := s2_forwardMask
  s2_loadDataFromDcache.forwardData := s2_forwardData
  s2_loadDataFromDcache.uop := s2_out.bits.uop
  s2_loadDataFromDcache.addrOffset := s2_in.bits.paddr(2, 0)
  dontTouch(s2_loadDataFromDcache)
  // real miss: dcache miss as well as can't forward
  s2_out.bits.miss := s2_cache_miss && !s2_hasException && !s2_fullForward && !s2_ldld_violation &&
    !s2_isSoftPrefetch && s2_enableMem
  s2_out.bits.uop.ctrl.fpWen := s2_in.bits.uop.ctrl.fpWen && !s2_hasException
  s2_out.bits.mmio := s2_mmio && s2_enableMem

  s2_out.bits.forwardMask := s2_forwardMask
  s2_out.bits.forwardData := s2_forwardData // data from dcache is not included in io.out.bits.forwardData
  s2_in.ready := s2_out.ready || !s2_in.valid

  val s2_dataForwarded = (s2_cache_miss || s2_cache_replay || s2_bank_conflict || s2_cancel_inner) && !s2_hasException && s2_fullForward

  val s2_rsFeedback = Wire(ValidIO(new RSFeedback))
  s2_rsFeedback.valid := s2_in.valid && (!s2_in.bits.replay.isReplayQReplay)
  s2_rsFeedback.bits.rsIdx := s2_in.bits.rsIdx
  s2_rsFeedback.bits.sourceType := Mux(!io.s3_enq_replayQueue.ready, RSFeedbackType.replayQFull,RSFeedbackType.success)
  dontTouch(s2_rsFeedback)

  // provide paddr for lq
  io.lsq.s1_lduMMIOPAddr.valid := s1_out.valid && !s1_tlb_miss
  io.lsq.s1_lduMMIOPAddr.bits.lqIdx := s1_out.bits.uop.lqIdx
  io.lsq.s1_lduMMIOPAddr.bits.paddr := s1_paddr_dup_lsu

  // todo: delete feedback fast
  io.feedbackFast.valid := false.B
  io.feedbackFast.bits := DontCare

  // provide prefetcher train data
  io.prefetch_train.bits := s2_in.bits
  io.prefetch_train.bits.miss := io.dcache.resp.bits.miss
  io.prefetch_train.valid := s2_in.fire && !s2_out.bits.mmio && !s2_in.bits.tlbMiss

  io.prefetch_train_l1.valid := s2_out.valid && !s2_out.bits.mmio
  io.prefetch_train_l1.bits := s2_out.bits
  io.prefetch_train_l1.bits.miss := io.dcache.resp.bits.miss
  io.hit_prefetch := isFromL1Prefetch(io.dcache.resp.bits.meta_prefetch)


  val hasDcacheErrButForwarded = !(s2_cache_miss || s2_cache_replay || s2_bank_conflict || s2_cancel_inner) || s2_fullForward
  val exceptionWb = s2_hasException
  val normalWb = !s2_tlb_miss && hasDcacheErrButForwarded && !s2_data_invalid && !s2_mmio && !s2_allStLdViolation && !s2_enqRAWFail

  val s2_wb_valid = s2_in.valid && !s2_in.bits.uop.robIdx.needFlush(redirectReg("loadS2")) &&
   (exceptionWb || normalWb)

  // writeback to LSQ, Load queue will be updated at s2 for both hit/miss int/fp load
  //update exceptionGen
  io.lsq.s2_excepWb2LQ.valid := s2_out.valid
  io.lsq.s2_excepWb2LQ.bits.fromLsPipelineBundle(s2_out.bits) // generate LqWriteBundle from LsPipelineBundle
  io.lsq.s2_excepWb2LQ.bits.has_writeback := s2_wb_valid

  io.lsq.s2_queryAndUpdateLQ.valid := s2_wb_valid
  io.lsq.s2_queryAndUpdateLQ.bits.lqPtr := s2_out.bits.uop.lqIdx
  io.lsq.s2_queryAndUpdateLQ.bits.dataIsFromDCache := s2_dataFromDCache
  io.lsq.s2_queryAndUpdateLQ.bits.wayIdx := s2_dcacheResp.bits.wayIdx
  io.lsq.s2_queryAndUpdateLQ.bits.paddr := s2_out.bits.paddr
  io.lsq.s2_queryAndUpdateLQ.bits.debug_mmio := s2_out.bits.mmio

  when(s2_in.valid) {
    assert(!(s2_tlb_miss && s2_fullForward),"when s2_tlb_miss,s2_fullForward must be false!!")
    when(s2_tlb_miss){
      assert(!(s2_out.bits.uop.cf.exceptionVec(loadAccessFault) || s2_out.bits.uop.cf.exceptionVec(loadPageFault)))
    }
  }

  // Int load, if hit, will be writebacked at s2
  val hitLoadOut = Wire(Valid(new ExuOutput))
  hitLoadOut := DontCare
  hitLoadOut.valid := s2_wb_valid
  hitLoadOut.bits.uop := s2_out.bits.uop
  hitLoadOut.bits.data := s2_out.bits.data
  hitLoadOut.bits.redirectValid := false.B
  hitLoadOut.bits.redirect := DontCare
  hitLoadOut.bits.debug.isMMIO := s2_out.bits.mmio
  hitLoadOut.bits.debug.isPerfCnt := false.B
  hitLoadOut.bits.debug.paddr := s2_out.bits.paddr
  hitLoadOut.bits.debug.vaddr := s2_out.bits.vaddr
  hitLoadOut.bits.fflags := DontCare

  s2_out.ready := true.B
  val s2_causeReg = WireInit(0.U.asTypeOf(new ReplayInfo))
  val debugS2CauseReg = RegInit(0.U.asTypeOf(new ReplayInfo))
  debugS2CauseReg := Mux(s1_cause_can_transfer && s1_out.valid, s1_causeReg, 0.U.asTypeOf(new ReplayInfo))
  val s2_cause_can_transfer = (!ExceptionNO.selectByFu(s2_out.bits.uop.cf.exceptionVec, lduCfg).asUInt.orR) && !s2_in.bits.isSoftPrefetch && !s2_mmio && s2_enableMem
  s2_causeReg := debugS2CauseReg
  s2_causeReg.full_fwd := s2_fullForward
  s2_causeReg.fwd_data_sqIdx := Mux(s2_data_invalid, io.lsq.forwardFromSQ.dataInvalidSqIdx, 0.U.asTypeOf(new SqPtr))
  s2_causeReg.dcache_miss := s2_cache_miss || debugS2CauseReg.dcache_miss
  s2_causeReg.fwd_fail    := s2_data_invalid || debugS2CauseReg.fwd_fail
  // s2_causeReg.dcache_rep  := (s2_cache_replay && !s2_tlb_miss) || debugS2CauseReg.dcache_rep
  s2_causeReg.dcache_rep  := (s2_cache_replay_with_mshrFull && !s2_tlb_miss) || debugS2CauseReg.dcache_rep
  s2_causeReg.raw_violation := s2_hasStLdViolation || debugS2CauseReg.raw_violation
  s2_causeReg.raw_nack := s2_enqRAWFail
  dontTouch(s2_causeReg)

  //missq full do not go to Fastreplay
  val needFastRep = (s2_bank_conflict || s2_cancel_inner || s2_cache_replay_with_conflict || s2_cache_replay_with_reject) && !s2_fullForward
  val hasNoOtherCause = !(s2_causeReg.replayCause.reduce(_ || _))
  io.fastReplayOut.valid := s2_in.valid && s2_enableMem && hasNoOtherCause && needFastRep && !exceptionWb && !s2_mmio && !s2_in.bits.uop.robIdx.needFlush(redirectReg("fastRep"))
  io.fastReplayOut.bits  := s2_in.bits
  // lpv cancel feedback to reservationStation
  io.earlyWakeUp.cancel := s2_in.valid && !s2_wb_valid

  /*
    LOAD S3: writeback data merge; writeback control
  */
  val s3_in = Wire(Decoupled(new LsPipelineBundle))
  s3_in.ready := true.B
  PipelineConnect(s2_out, s3_in, true.B, s2_out.bits.uop.robIdx.needFlush(redirectReg("loadS2")))

  val s3_ldld_violation = io.lsq.loadViolationQuery.s3_resp.valid &&
    io.lsq.loadViolationQuery.s3_resp.bits.have_violation &&
    RegNext(io.csrCtrl.ldld_vio_check_enable)

  // mmio data from load queue
  val s3_mmioDataFromLq = RegEnable(io.mmioWb.bits.data, io.mmioWb.valid)
  // data from dcache hit
  val s3_loadDataFromDcache = RegEnable(s2_loadDataFromDcache, s2_in.valid)
  val s3_rdataDcache = s3_loadDataFromDcache.mergedData() //merge Data from forward and DCache

  private val hitLoadOutValidReg = RegNext(hitLoadOut.valid, false.B)
  val s3_uop = s3_loadDataFromDcache.uop
  val s3_offset = s3_loadDataFromDcache.addrOffset
  val s3_rdata_dup = WireInit(VecInit(List.fill(8)(0.U(64.W))))
  s3_rdata_dup.zipWithIndex.foreach({case(d,i) => {
    d := s3_rdataDcache
  }})

  val s3_sel_rdata = LookupTree(s3_offset,List(
    "b000".U -> s3_rdata_dup(0)(63, 0),
    "b001".U -> s3_rdata_dup(1)(63, 8),
    "b010".U -> s3_rdata_dup(2)(63, 16),
    "b011".U -> s3_rdata_dup(3)(63, 24),
    "b100".U -> s3_rdata_dup(4)(63, 32),
    "b101".U -> s3_rdata_dup(5)(63, 40),
    "b110".U -> s3_rdata_dup(6)(63, 48),
    "b111".U -> s3_rdata_dup(7)(63, 56)
  ))
  val s3_rdataPartialLoad = rdataHelper(s3_uop,s3_sel_rdata)

  private val s3_lsqMMIOOutputValid = RegNext(io.mmioWb.valid && (!io.mmioWb.bits.uop.robIdx.needFlush(redirectReg("loadS3"))),false.B)
  io.ldout.valid := hitLoadOutValidReg || s3_lsqMMIOOutputValid
  val s3_load_wb_meta_reg = RegEnable(Mux(hitLoadOut.valid,hitLoadOut.bits,io.mmioWb.bits), hitLoadOut.valid | io.mmioWb.valid)
  io.ldout.bits := s3_load_wb_meta_reg
  io.ldout.bits.data := Mux(hitLoadOutValidReg, s3_rdataPartialLoad, s3_load_wb_meta_reg.data)

  val s3_alreadyFastRep = RegNext(io.fastReplayOut.valid)
  dontTouch(s3_alreadyFastRep)
  io.feedbackSlow.valid := s3_in.valid &&
    !s3_in.bits.replay.isReplayQReplay &&
    !s3_in.bits.uop.robIdx.needFlush(redirectReg("loadS3")) &&
    !s3_alreadyFastRep

  io.feedbackSlow.bits.rsIdx := s3_in.bits.rsIdx
  io.feedbackSlow.bits.sourceType :=  Mux(!hitLoadOutValidReg && !io.s3_enq_replayQueue.ready || RegNext(io.fastReplayOut.valid),
    RSFeedbackType.replayQFull, RSFeedbackType.success)

  // load forward_fail/ldld_violation check, mcheck for inst in load pipeline
  val s3_forward_fail = RegNext(io.lsq.forwardFromSQ.matchInvalid || io.forwardFromSBuffer.matchInvalid)
  val s3_need_replay_from_fetch = s3_forward_fail || s3_ldld_violation
  val s3_can_replay_from_fetch = RegEnable(s2_out.bits.mmio && !s2_out.bits.isSoftPrefetch && s2_out.bits.tlbMiss, s2_out.valid)

  when (RegNext(hitLoadOut.valid)) {
    io.ldout.bits.uop.ctrl.replayInst := s3_need_replay_from_fetch
  }

  io.mmioWb.ready := !hitLoadOut.valid

  val lastValidData = RegEnable(io.ldout.bits.data, io.ldout.fire)
  val hitLoadAddrTriggerHitVec = Wire(Vec(TriggerNum, Bool()))
  val lqLoadAddrTriggerHitVec = io.lsq.trigger.lqLoadAddrTriggerHitVec
  (0 until TriggerNum).foreach{ i => {
    val tdata2 = io.trigger(i).tdata2
    val matchType = io.trigger(i).matchType
    val tEnable = io.trigger(i).tEnable

    hitLoadAddrTriggerHitVec(i) := TriggerCmp(s2_out.bits.vaddr, tdata2, matchType, tEnable)
    io.trigger(i).addrHit := RegNext(Mux(hitLoadOut.valid, hitLoadAddrTriggerHitVec(i), lqLoadAddrTriggerHitVec(i)))
    io.trigger(i).lastDataHit := TriggerCmp(lastValidData, tdata2, matchType, tEnable)
  }}
  io.lsq.trigger.hitLoadAddrTriggerHitVec := hitLoadAddrTriggerHitVec

  val s3_causeReg = RegInit(0.U.asTypeOf(new ReplayInfo))
  s3_causeReg := Mux( s2_cause_can_transfer && s2_out.fire && !s2_wb_valid, s2_causeReg, 0.U.asTypeOf(new ReplayInfo))
  dontTouch(s3_causeReg)
  val s3_dcacheMshrID = RegEnable(s2_dcacheMshrID.bits, s2_dcacheMshrID.valid)

  dontTouch(s3_dcacheMshrID)
  //write back control info to replayQueue in S3
  io.s3_enq_replayQueue.valid := s3_in.valid && !s3_in.bits.uop.robIdx.needFlush(redirectReg("loadS3")) && RegNext(!io.fastReplayOut.valid)
  io.s3_enq_replayQueue.bits.vaddr := s3_in.bits.vaddr
  io.s3_enq_replayQueue.bits.paddr := s3_in.bits.paddr
  io.s3_enq_replayQueue.bits.isMMIO := s3_in.bits.mmio
  io.s3_enq_replayQueue.bits.paddr := s3_in.bits.paddr
  io.s3_enq_replayQueue.bits.replay.isReplayQReplay := s3_in.bits.replay.isReplayQReplay
  io.s3_enq_replayQueue.bits.replay.replayCause := DontCare
  io.s3_enq_replayQueue.bits.replay.replayCause(LoadReplayCauses.C_TM) := s3_causeReg.tlb_miss
  io.s3_enq_replayQueue.bits.replay.replayCause(LoadReplayCauses.C_FF) := s3_causeReg.fwd_fail
  io.s3_enq_replayQueue.bits.replay.replayCause(LoadReplayCauses.C_DR) := s3_causeReg.dcache_rep
  io.s3_enq_replayQueue.bits.replay.replayCause(LoadReplayCauses.C_DM) := s3_causeReg.dcache_miss
  io.s3_enq_replayQueue.bits.replay.replayCause(LoadReplayCauses.C_BC) := s3_causeReg.bank_conflict
  io.s3_enq_replayQueue.bits.replay.replayCause(LoadReplayCauses.C_RAW) := s3_causeReg.raw_violation
  io.s3_enq_replayQueue.bits.replay.replayCause(LoadReplayCauses.C_NK) := s3_causeReg.raw_nack
  io.s3_enq_replayQueue.bits.replay.schedIndex := s3_in.bits.replay.schedIndex
  io.s3_enq_replayQueue.bits.replay.fwd_data_sqIdx := s3_causeReg.fwd_data_sqIdx
  io.s3_enq_replayQueue.bits.replay.full_fwd := RegNext(s2_dataForwarded)
  io.s3_enq_replayQueue.bits.mshrMissIDResp := s3_dcacheMshrID
  io.s3_enq_replayQueue.bits.uop := s3_in.bits.uop
  io.s3_enq_replayQueue.bits.mask := s3_in.bits.mask
  io.s3_enq_replayQueue.bits.tlbMiss := false.B
  assert(!((io.s3_enq_replayQueue.valid &&io.s3_enq_replayQueue.bits.replay.need_rep) && s3_alreadyFastRep), "FastRep and ReplayQ will handle independent casue")
  assert(!(RegNext(hitLoadOut.valid,false.B) && s3_alreadyFastRep),"FastRep and Wirteback cant happened same time")
  assert(!(RegNext(hitLoadOut.valid,false.B) && io.s3_enq_replayQueue.bits.replay.replayCause.reduce(_|_)),"when load" +  " wb," + "replayCause must be 0!!")


  val s3_needReplay = io.s3_enq_replayQueue.valid && io.s3_enq_replayQueue.bits.replay.replayCause.reduce(_|_)

  val s2_canEnqRAW = s2_in.valid &&  !s2_in.bits.uop.robIdx.needFlush(redirectReg("loadS2")) && !s2_hasException &&
    !s2_tlb_miss && hasDcacheErrButForwarded && !s2_data_invalid && !s2_mmio && !s2_allStLdViolation

  io.enqRAWQueue.s2_enq.valid := s2_canEnqRAW
  io.enqRAWQueue.s2_enq.bits.paddr := s2_out.bits.paddr(PAddrBits - 1, 3)
  io.enqRAWQueue.s2_enq.bits.mask := s2_out.bits.mask
  io.enqRAWQueue.s2_enq.bits.sqIdx := s2_out.bits.uop.sqIdx
  io.enqRAWQueue.s2_enq.bits.robIdx := s2_out.bits.uop.robIdx
  io.enqRAWQueue.s2_enq.bits.ftqPtr := s2_out.bits.uop.cf.ftqPtr
  io.enqRAWQueue.s2_enq.bits.ftqOffset := s2_out.bits.uop.cf.ftqOffset
  io.enqRAWQueue.s3_cancel := RegNext(io.enqRAWQueue.s2_enq.valid && io.enqRAWQueue.s2_enqSuccess,false.B) && s3_needReplay

  val perfEvents = Seq(
    ("load_s0_in_fire         ", s0_valid),
    ("load_to_load_forward    ", s1_out.valid),
    ("stall_dcache            ", s0_out.valid && s0_out.ready && !s0_req_dcache.ready),
    ("load_s1_in_fire         ", s1_in.fire),
    ("load_s1_tlb_miss        ", s1_in.fire && s1_dtlbResp.bits.miss),
    ("load_s2_in_fire         ", s2_in.fire),
    ("load_s2_dcache_miss     ", s2_in.fire && s2_dcacheResp.bits.miss),
    ("load_s2_replay          ", s2_rsFeedback.valid),
    ("load_s2_replay_tlb_miss ", s2_rsFeedback.valid && s2_in.bits.tlbMiss),
    ("load_s2_replay_cache    ", s2_rsFeedback.valid && !s2_in.bits.tlbMiss && s2_dcacheResp.bits.miss),
  )
  generatePerfEvent()


  XSPerfAccumulate("NHV5_load_issueFromRs", io.rsIssueIn.fire)
  XSPerfAccumulate("NHV5_load_issueFromReplay", io.replayQIssueIn.fire)
  XSPerfAccumulate("NHV5_MSHR_forward_valid", io.lduForwardMSHR.resp.valid)

  XSPerfAccumulate("NHV5_load_s0_fromRs_requireTLB", io.tlb.req.fire && !s0_out.bits.replay.isReplayQReplay)
  XSPerfAccumulate("NHV5_load_s0_fromRs_requireDcache", io.dcache.req.fire && !s0_out.bits.replay.isReplayQReplay)
  XSPerfAccumulate("NHV5_load_s0_fromRq_requireTLB", io.tlb.req.fire && s0_out.bits.replay.isReplayQReplay)
  XSPerfAccumulate("NHV5_load_s0_fromRq_requireDcache", io.dcache.req.fire && s0_out.bits.replay.isReplayQReplay)

  val s1_perfValidCounting = s1_out.valid && !(ExceptionNO.selectByFu(s1_out.bits.uop.cf.exceptionVec, lduCfg).asUInt.orR) &&
  (!s1_isSoftPrefetch) && s1_enableMem && !s1_out.bits.uop.robIdx.needFlush(redirectReg("loadS1"))
  XSPerfAccumulate("NHV5_load_s1_fromRs_TLBMiss", s1_tlb_miss && s1_perfValidCounting && !s1_out.bits.replay.isReplayQReplay)
  XSPerfAccumulate("NHV5_load_s1_fromRs_HasRawVio", s1_hasStLdViolation && s1_perfValidCounting && !s1_out.bits.replay.isReplayQReplay)
  XSPerfAccumulate("NHV5_load_s1_fromRs_RarCheckRedo", s1_needLdVioCheckRedo && s1_perfValidCounting && !s1_out.bits.replay.isReplayQReplay)
  XSPerfAccumulate("NHV5_load_s1_fromRs_BankConflict", s1_bank_conflict && s1_perfValidCounting && !s1_out.bits.replay.isReplayQReplay)
  XSPerfAccumulate("NHV5_load_s1_fromRs_DcacheNotRdy", s1_cancel_inner && s1_perfValidCounting && !s1_out.bits.replay.isReplayQReplay)

  XSPerfAccumulate("NHV5_load_s1_fromRq_TLBMiss", s1_tlb_miss && s1_perfValidCounting && s1_out.bits.replay.isReplayQReplay)
  XSPerfAccumulate("NHV5_load_s1_fromRq_HasRawVio", s1_hasStLdViolation && s1_perfValidCounting && s1_out.bits.replay.isReplayQReplay)
  XSPerfAccumulate("NHV5_load_s1_fromRq_RarCheckRedo", s1_needLdVioCheckRedo && s1_perfValidCounting && s1_out.bits.replay.isReplayQReplay)
  XSPerfAccumulate("NHV5_load_s1_fromRq_BankConflict", s1_bank_conflict && s1_perfValidCounting && s1_out.bits.replay.isReplayQReplay)
  XSPerfAccumulate("NHV5_load_s1_fromRq_DcacheNotRdy", s1_cancel_inner && s1_perfValidCounting && s1_out.bits.replay.isReplayQReplay)


  val s2_perfValidCounting = s2_out.valid && (!ExceptionNO.selectByFu(s2_out.bits.uop.cf.exceptionVec, lduCfg).asUInt.orR) &&
  !s2_in.bits.isSoftPrefetch && !s2_mmio && s2_enableMem && !s2_out.bits.uop.robIdx.needFlush(redirectReg("loadS2"))
  XSPerfAccumulate("NHV5_load_s2_fromRs_DcacheMiss", s2_cache_miss && !s2_bank_conflict && s2_perfValidCounting && !s2_out.bits.replay.isReplayQReplay)
  XSPerfAccumulate("NHV5_load_s2_fromRs_FwdFail", s2_data_invalid && s2_perfValidCounting && !s2_out.bits.replay.isReplayQReplay)
  XSPerfAccumulate("NHV5_load_s2_fromRs_DcacheMshrFull", s2_cache_replay && s2_perfValidCounting && !s2_out.bits.replay.isReplayQReplay)
  XSPerfAccumulate("NHV5_load_s2_fromRs_HasRawVio", s2_hasStLdViolation && s2_perfValidCounting && !s2_out.bits.replay.isReplayQReplay)

  XSPerfAccumulate("dcache_mshrFull", s2_cache_replay && s2_cache_replay_with_mshrFull && s2_out.valid)
  XSPerfAccumulate("dcache_enq_conflict", s2_cache_replay && s2_cache_replay_with_conflict &&  s2_out.valid)
  XSPerfAccumulate("dcache_enq_reject", s2_cache_replay && s2_cache_replay_with_reject &&  s2_out.valid)
  XSPerfAccumulate("dcache_replay_go_to_fastReplay", io.fastReplayOut.valid &&  (s2_cache_replay_with_conflict || s2_cache_replay_with_reject))

  XSPerfAccumulate("dcache_enq_reject_but_forwardMshr_data",s2_cache_replay && s2_cache_replay_with_reject &&  s2_out.valid && io.lduForwardMSHR.resp.valid && s2_wb_valid) 

  XSPerfAccumulate("NHV5_load_s2_fromRq_DcacheMiss", s2_cache_miss && !s2_bank_conflict && s2_perfValidCounting && s2_out.bits.replay.isReplayQReplay)
  XSPerfAccumulate("NHV5_load_s2_fromRq_FwdFail", s2_data_invalid && s2_perfValidCounting && s2_out.bits.replay.isReplayQReplay)
  XSPerfAccumulate("NHV5_load_s2_fromRq_DcacheMshrFull", s2_cache_replay && s2_perfValidCounting && s2_out.bits.replay.isReplayQReplay)
  XSPerfAccumulate("NHV5_load_s2_fromRq_HasRawVio", s2_hasStLdViolation && s2_perfValidCounting && s2_out.bits.replay.isReplayQReplay)

  XSPerfAccumulate("NHV5_load_s2_fromRs_isMMio", s2_out.valid && (!ExceptionNO.selectByFu(s2_out.bits.uop.cf.exceptionVec, lduCfg).asUInt.orR) && !s2_in.bits.isSoftPrefetch && s2_mmio && s2_enableMem )

  val s3_perfValidCounting = hitLoadOutValidReg
  XSPerfAccumulate("NHV5_load_s3_fromRs_normalWbOnceSuccess",    io.ldout.valid && !s3_in.bits.replay.isReplayQReplay && !s3_in.bits.mmio)
  XSPerfAccumulate("NHV5_load_s3_fromRq_ReplayWb", io.ldout.valid && s3_in.bits.replay.isReplayQReplay && !s3_in.bits.mmio)
  XSPerfAccumulate("NHV5_load_s3_fromRq_mmioWb", s3_lsqMMIOOutputValid)
  XSPerfAccumulate("NHV5_load_s3_replayToRs_by_replayQFull ", io.feedbackSlow.valid && io.feedbackSlow.bits.sourceType === RSFeedbackType.replayQFull)
  XSPerfAccumulate("NHV5_load_s3_success_release_rs", io.feedbackSlow.valid && io.feedbackSlow.bits.sourceType === RSFeedbackType.success)

  val s3NeedReplay = io.s3_enq_replayQueue.fire && io.s3_enq_replayQueue.bits.replay.need_rep && !s3_in.bits.uop.robIdx.needFlush(redirectReg("loadS3"))
  XSPerfAccumulate("NHV5_load_s3_needReplay_cause_TlbMiss", s3NeedReplay && io.s3_enq_replayQueue.bits.replay.replayCause(LoadReplayCauses.C_TM))
  XSPerfAccumulate("NHV5_load_s3_needReplay_cause_FwdFail", s3NeedReplay && io.s3_enq_replayQueue.bits.replay.replayCause(LoadReplayCauses.C_FF))
  XSPerfAccumulate("NHV5_load_s3_needReplay_cause_MshrFull", s3NeedReplay && io.s3_enq_replayQueue.bits.replay.replayCause(LoadReplayCauses.C_DR))
  XSPerfAccumulate("NHV5_load_s3_needReplay_cause_DcacheMiss", s3NeedReplay && io.s3_enq_replayQueue.bits.replay.replayCause(LoadReplayCauses.C_DM))
  XSPerfAccumulate("NHV5_load_s3_needReplay_cause_BankConflict", s3NeedReplay && io.s3_enq_replayQueue.bits.replay.replayCause(LoadReplayCauses.C_BC))
  XSPerfAccumulate("NHV5_load_s3_needReplay_cause_Raw", s3NeedReplay && io.s3_enq_replayQueue.bits.replay.replayCause(LoadReplayCauses.C_RAW))

  XSPerfAccumulate("NHV5_load_s3_needReplay_lastTimeHas_TlbMiss", s3NeedReplay && s3_in.bits.debugCause(LoadReplayCauses.C_TM))
  XSPerfAccumulate("NHV5_load_s3_needReplay_lastTimeHas_FwdFail", s3NeedReplay && s3_in.bits.debugCause(LoadReplayCauses.C_FF))
  XSPerfAccumulate("NHV5_load_s3_needReplay_lastTimeHas_MshrFull", s3NeedReplay && s3_in.bits.debugCause(LoadReplayCauses.C_DR))
  XSPerfAccumulate("NHV5_load_s3_needReplay_lastTimeHas_DcacheMiss", s3NeedReplay && s3_in.bits.debugCause(LoadReplayCauses.C_DM))
  XSPerfAccumulate("NHV5_load_s3_needReplay_lastTimeHas_BankConflict", s3NeedReplay && s3_in.bits.debugCause(LoadReplayCauses.C_BC))
  XSPerfAccumulate("NHV5_load_s3_needReplay_lastTimeHas_Raw", s3NeedReplay && s3_in.bits.debugCause(LoadReplayCauses.C_RAW))

  val s3CauseMerge = (s3_in.bits.debugCause & io.s3_enq_replayQueue.bits.replay.replayCause.asUInt)
  XSPerfAccumulate("NHV5_load_s3_sameCause_TlbMiss", s3NeedReplay && s3CauseMerge(LoadReplayCauses.C_TM))
  XSPerfAccumulate("NHV5_load_s3_sameCause_FwdFail", s3NeedReplay && s3CauseMerge(LoadReplayCauses.C_FF))
  XSPerfAccumulate("NHV5_load_s3_sameCause_MshrFull", s3NeedReplay && s3CauseMerge(LoadReplayCauses.C_DR))
  XSPerfAccumulate("NHV5_load_s3_sameCause_DcacheMiss", s3NeedReplay && s3CauseMerge(LoadReplayCauses.C_DM))
  XSPerfAccumulate("NHV5_load_s3_sameCause_BankConflict", s3NeedReplay && s3CauseMerge(LoadReplayCauses.C_BC))
  XSPerfAccumulate("NHV5_load_s3_sameCause_Raw", s3NeedReplay && s3CauseMerge(LoadReplayCauses.C_RAW))

  XSPerfAccumulate("NHV5_load_notIssue", (!io.rsIssueIn.valid && !io.replayQIssueIn.valid) && (io.rsIssueIn.ready || io.replayQIssueIn.ready))
  XSPerfAccumulate("NHV5_load_notWb", !io.ldout.valid && io.ldout.ready)

  val debug_NHV5_load_s3_sameCause_MshrFull = RegInit(false.B)
  debug_NHV5_load_s3_sameCause_MshrFull := s3NeedReplay && s3CauseMerge(LoadReplayCauses.C_DR)
  dontTouch(debug_NHV5_load_s3_sameCause_MshrFull)
  when(io.ldout.fire){
    XSDebug("ldout %x\n", io.ldout.bits.uop.cf.pc)
  }

  val debugModule = Module(new LoadUnitDebugInfo)
  debugModule.io.infoIn.s0_valid := s0_valid
  debugModule.io.infoIn.s1_valid := s1_in.valid
  debugModule.io.infoIn.s2_valid := s2_in.valid
  debugModule.io.infoIn.s3_valid := s3_in.valid
  debugModule.io.infoIn.wb_valid := io.ldout.valid
  debugModule.io.infoIn.rsIdx := io.rsIdx
  debugModule.io.infoIn.robIdx := s0_sel_src.uop.robIdx
}

class LdDebugBundle(implicit p: Parameters) extends XSBundle {
  val s0_valid = Bool()
  val s1_valid = Bool()
  val s2_valid = Bool()
  val s3_valid = Bool()
  val wb_valid = Bool()
  val rsIdx = new RsIdx()
  val robIdx = new RobPtr()
}

class LoadUnitDebugInfo(implicit p: Parameters) extends XSModule{
  val io = IO(new Bundle(){
    val infoIn = Input(new LdDebugBundle)
  })

  val debugReg = RegInit(0.U.asTypeOf(new LdDebugBundle))
  debugReg := io.infoIn
  dontTouch(debugReg)
}

