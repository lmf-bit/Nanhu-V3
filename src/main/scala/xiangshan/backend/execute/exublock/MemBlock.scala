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

package xiangshan.backend.execute.exublock

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy.{BundleBridgeSource, LazyModule, LazyModuleImp}
import freechips.rocketchip.tile.HasFPUParameters
import coupledL2.PrefetchRecv
import coupledL2.prefetch.PrefetchReceiverParams
import utils._
import xs.utils._
import xiangshan._
import xiangshan.backend.execute.exu.{ExuConfig, ExuInputNode, ExuOutputMultiSinkNode, ExuOutputNode, ExuType}
import xiangshan.backend.execute.exucx.ExuComplexIssueNode
import xiangshan.backend.execute.fu._
import xiangshan.backend.execute.fu.csr.CSRConst.ModeS
import xiangshan.backend.execute.fu.{FuConfigs, FunctionUnit, PMP, PMPChecker, PMPCheckerv2}
import xiangshan.backend.execute.fu.csr.{PFEvent, SdtrigExt}
import xiangshan.backend.execute.fu.fence.{FenceToSbuffer, SfenceBundle}
import xiangshan.backend.issue.EarlyWakeUpInfo
import xiangshan.backend.rob.RobLsqIO
import xiangshan.cache._
import xiangshan.cache.mmu.{BTlbPtwIO, HasTlbConst, PtwSectorResp, TLB_backend, TlbHintIO, TlbIO, TlbReplace}
import xiangshan.mem._
import xiangshan.mem.prefetch._
import xs.utils.mbist.MBISTPipeline
import xs.utils.perf.HasPerfLogging
import xs.utils.{DelayN, ParallelPriorityMux, RegNextN, ValidIODelay}

class Std(implicit p: Parameters) extends XSModule {
  val io = IO(new Bundle{
    val in = Flipped(DecoupledIO(new ExuInput))
    val out = DecoupledIO(new ExuOutput)
    val redirect = Input(Valid(new Redirect))
  })
  private val validReg = RegNext(io.in.valid, false.B)
  private val bitsReg = RegEnable(io.in.bits, io.in.valid)
  io.in.ready := true.B
  io.out.valid := validReg && !bitsReg.uop.robIdx.needFlush(io.redirect)
  io.out.bits := DontCare
  io.out.bits.uop := bitsReg.uop
  io.out.bits.data := bitsReg.src(1)
}
class MemIssueRouter(implicit p: Parameters) extends LazyModule{
  val node = new ExuComplexIssueNode
  lazy val module = new LazyModuleImp(this){
    require(node.in.length == 1)
    private val ib = node.in.head._1
    for((ob,oe) <- node.out) {
      ob.issue.valid := ib.issue.valid && ib.issue.bits.uop.ctrl.fuType === oe._2.fuConfigs.head.fuType
      ob.issue.bits := ib.issue.bits
      ib.issue.ready := true.B
      assert(ob.issue.ready === true.B)
      ob.rsIdx := ib.rsIdx
      ob.auxValid := ib.auxValid && ib.issue.bits.uop.ctrl.fuType === oe._2.fuConfigs.head.fuType
      if (oe._2.fuConfigs.head.name == "sta") {
        ib.rsFeedback.feedbackSlowStore := ob.rsFeedback.feedbackSlowStore
      }
//      if (oe._2.fuConfigs.head.name == "ldu") {
//        ib.rsFeedback.feedbackFastLoad := ob.rsFeedback.feedbackFastLoad
//        ib.rsFeedback.feedbackSlowLoad := ob.rsFeedback.feedbackSlowLoad
//      } else if (oe._2.fuConfigs.head.name == "sta") {
//        ib.rsFeedback.feedbackSlowStore := ob.rsFeedback.feedbackSlowStore
//      }
    }
  }
}

class MemBlock(val parentName:String = "Unknown")(implicit p: Parameters) extends BasicExuBlock
  with HasXSParameter{

  private val lduParams = Seq.tabulate(exuParameters.LduCnt)(idx => {
    ExuConfig(
      name = "LduExu",
      id = idx,
      complexName = "MemComplex",
      fuConfigs = Seq(FuConfigs.lduCfg),
      exuType = ExuType.ldu,
      writebackToRob = true,
      writebackToVms = false
    )
  })
  private val staParams = Seq.tabulate(exuParameters.StuCnt)(idx => {
    ExuConfig(
      name = "StaExu",
      id = idx,
      complexName = "MemComplex",
      fuConfigs = Seq(FuConfigs.staCfg),
      exuType = ExuType.sta,
      writebackToRob = true,
      writebackToVms = false
    )
  })
  private val stdParams = Seq.tabulate(exuParameters.StuCnt)(idx => {
    ExuConfig(
      name = "StdExu",
      id = idx,
      complexName = "MemComplex",
      fuConfigs = Seq(FuConfigs.stdCfg),
      exuType = ExuType.std,
      writebackToRob = true,
      writebackToVms = false
    )
  })
//  private val slduParams = Seq.tabulate(exuParameters.LduCnt)(idx => {ExuConfig(
//    name = "SpecialLduExu",
//    id = idx,
//    complexName = "MemComplex",
//    fuConfigs = Seq(FuConfigs.specialLduCfg),
//    exuType = ExuType.sldu,
//    writebackToRob = false,
//    writebackToVms = false
//  )})

  private val vstdParams = Seq.tabulate(exuParameters.StuCnt)(idx => {
    ExuConfig(
      name = "vStdExu",
      id = idx,
      complexName = "MemComplex",
      fuConfigs = Seq(FuConfigs.stdCfg),
      exuType = ExuType.std,
      writebackToRob = false,
      writebackToVms = true
    )
  })
  private val vstaParams = Seq.tabulate(exuParameters.StuCnt)(idx => {
    ExuConfig(
      name = "vStaExu",
      id = idx,
      complexName = "MemComplex",
      fuConfigs = Seq(FuConfigs.staCfg),
      exuType = ExuType.sta,
      writebackToRob = false,
      writebackToVms = true
    )
  })

  private val vlduParams = Seq.tabulate(exuParameters.LduCnt)(idx => {
    ExuConfig(
      name = "vLduExu",
      id = idx,
      complexName = "MemComplex",
      fuConfigs = Seq(FuConfigs.lduCfg),
      exuType = ExuType.ldu,
      writebackToRob = false,
      writebackToVms = true
    )
  })

//  val lduIssueNodes: Seq[ExuInputNode] = lduParams.zipWithIndex.map(e => new ExuInputNode(e._1))
//  val slduIssueNodes: Seq[MemoryBlockIssueNode] = slduParams.zipWithIndex.map(e => new MemoryBlockIssueNode(e._1, e._2))
  val lduIssueNodes: Seq[MemoryBlockIssueNode] = lduParams.zipWithIndex.map(e => new MemoryBlockIssueNode(e._1, e._2))
  val staIssueNodes: Seq[ExuInputNode] = staParams.zipWithIndex.map(e => new ExuInputNode(e._1))
  val stdIssueNodes: Seq[ExuInputNode] = stdParams.zipWithIndex.map(e => new ExuInputNode(e._1))

  val lduWritebackNodes: Seq[ExuOutputNode] = lduParams.map(e => new ExuOutputNode(e))
  val vlduWritebackNodes: Seq[ExuOutputNode] = vlduParams.map(e => new ExuOutputNode(e))
  val staWritebackNodes: Seq[ExuOutputNode] = staParams.map(new ExuOutputNode(_))
  val vstaWritebackNodes: Seq[ExuOutputNode] = vstaParams.map(new ExuOutputNode(_))
  val stdWritebackNodes: Seq[ExuOutputNode] = stdParams.map(new ExuOutputNode(_))
  val vstdWritebackNodes: Seq[ExuOutputNode] = vstdParams.map(new ExuOutputNode(_))

  val memIssueRouters: Seq[MemIssueRouter] = Seq.fill(2)(LazyModule(new MemIssueRouter))
  memIssueRouters.zip(staIssueNodes).zip(stdIssueNodes).foreach({case((mir, sta), std) =>
//    ldu :*= mir.node
    sta :*= mir.node
    std :*= mir.node
  })

  private val allWritebackNodes = lduWritebackNodes ++ staWritebackNodes ++ stdWritebackNodes

  memIssueRouters.foreach(mir => mir.node :*= issueNode)
  lduIssueNodes.foreach(_ :*= issueNode)
//  slduIssueNodes.foreach(_ :*= issueNode) //???
  allWritebackNodes.foreach(onode => writebackNode :=* onode)

  val dcache = LazyModule(new DCacheWrapper(parentName = parentName + "dcache_"))
  val uncache = LazyModule(new Uncache())
  val pf_sender_opt = coreParams.prefetcher match  {
    case Some(receive : SMSParams) => Some(BundleBridgeSource(() => new PrefetchRecv))
    // case sms_sender_hyper : HyperPrefetchParams => Some(BundleBridgeSource(() => new PrefetchRecv))
    case _ => None
  }

  lazy val module = new MemBlockImp(this)

}

class MemBlockImp(outer: MemBlock) extends BasicExuBlockImp(outer)
  with HasXSParameter
  with HasFPUParameters
  with HasPerfEvents
  with SdtrigExt
  with HasPerfLogging
  with HasTlbConst
{
  private val lduIssues = outer.lduIssueNodes.map(iss => {
    require(iss.in.length == 1)
    iss.in.head._1
  })

//  private val slduIssues = outer.slduIssueNodes.map(iss => {
//    require(iss.in.length == 1)
//    iss.in.head._1
//  })
  private val staIssues = outer.staIssueNodes.map(iss => {
    require(iss.in.length == 1)
    iss.in.head._1
  })
  private val stdIssues = outer.stdIssueNodes.map(iss => {
    require(iss.in.length == 1)
    iss.in.head._1
  })

  private val lduWritebacks = outer.lduWritebackNodes.map(wb => {
    require(wb.out.length == 1)
    wb.out.head._1
  })
  private val vlduWritebacks = outer.vlduWritebackNodes.map(wb => {
    require(wb.out.length == 1)
    wb.out.head._1
  })
  private val staWritebacks = outer.staWritebackNodes.map(wb => {
    require(wb.out.length == 1)
    wb.out.head._1
  })
  private val vstaWritebacks = outer.vstaWritebackNodes.map(wb => {
    require(wb.out.length == 1)
    wb.out.head._1
  })
  private val stdWritebacks = outer.stdWritebackNodes.map(wb => {
    require(wb.out.length == 1)
    wb.out.head._1
  })
  private val vstdWritebacks = outer.vstdWritebackNodes.map(wb => {
    require(wb.out.length == 1)
    wb.out.head._1
  })

  val io = IO(new Bundle {
    val hartId = Input(UInt(8.W))
    // in
    val stIssuePtr = Output(new SqPtr())
    // misc
    val stIn = Vec(exuParameters.StuCnt, ValidIO(new ExuInput))
    val ptw = new BTlbPtwIO(ld_tlb_ports + exuParameters.StuCnt)
    val tlb_hint = Flipped(new TlbHintIO)
    val tlb_wakeUp = Flipped(ValidIO(new PtwSectorResp))
    val sfence = Input(new SfenceBundle)
    val tlbCsr = Input(new TlbCsrBundle)
    val fenceToSbuffer = Flipped(new FenceToSbuffer)
    val enqLsq = new LsqEnqIO
    // val memPredUpdate = Vec(exuParameters.StuCnt, Input(new MemPredUpdateReq))
    val lsqio = new Bundle {
      val exceptionAddr = new ExceptionAddrIO // to csr
      val rob = Flipped(new RobLsqIO) // rob to lsq
    }
    val csrCtrl = Flipped(new CustomCSRCtrlIO)
    val csrUpdate = new DistributedCSRUpdateReq
    val error = new L1CacheErrorInfo
    val memInfo = new Bundle {
      val sqFull = Output(Bool())
      val lqFull = Output(Bool())
      val dcacheMSHRFull = Output(Bool())
    }

//    val earlyWakeUpCancel = Output(Vec(3, Vec(lduIssues.length, Bool())))
    val issueToMou = Flipped(Decoupled(new ExuInput))
    val writebackFromMou = Decoupled(new ExuOutput)

    val sqFull = Output(Bool())
    val lqFull = Output(Bool())
    val perfEventsPTW = Input(Vec(19, new PerfEvent))
    val lqCancelCnt = Output(UInt(log2Up(LoadQueueSize + 1).W))
    val sqCancelCnt = Output(UInt(log2Up(StoreQueueSize + 1).W))
    val sqDeq = Output(UInt(2.W))

    val lsqVecDeqCnt = Output(new LsqVecDeqIO)
    val lqDeq = Output(UInt(log2Up(CommitWidth + 1).W))
    val ldStopMemBlock = Output(Vec(LoadPipelineWidth, Bool()))
    val lduEarlyWakeUp = Output(Vec(loadUnitNum, new Bundle() {
      val cancel = Bool()
      val wakeUp = Valid(new EarlyWakeUpInfo)
    }))
    val l2_hint = Input(new DCacheTLDBypassLduIO)
  })
  io.lsqVecDeqCnt := DontCare

  val dcache = outer.dcache.module
  val uncache = outer.uncache.module

  val csrCtrl = DelayN(io.csrCtrl, 2)
  dcache.io.l2_hint := io.l2_hint
  dcache.io.csr.distribute_csr <> csrCtrl.distribute_csr
  dcache.io.l2_pf_store_only := RegNext(io.csrCtrl.l2_pf_store_only, false.B)
  io.csrUpdate := RegNext(dcache.io.csr.update)
  io.error <> RegNext(RegNext(dcache.io.error))
  when(!csrCtrl.cache_error_enable){
    io.error.report_to_beu := false.B
    io.error.valid := false.B
  }
  private val vmEnable = io.tlbCsr.priv.dmode <= ModeS && io.tlbCsr.satp.mode.orR

  private val loadUnits = Seq.fill(exuParameters.LduCnt)(Module(new LoadUnit))
  private val storeUnits = Seq.fill(exuParameters.StuCnt)(Module(new StoreUnit))
  private val stdUnits = Seq.fill(exuParameters.StuCnt)(Module(new Std))
  private val stData = stdUnits.map(_.io.out)
  
  val l1_pf_req = Wire(Decoupled(new L1PrefetchReq()))
  val prefetcherOpt: Option[BasePrefecher] = coreParams.prefetcher match {
    case Some(sms_sender: SMSParams) =>
      val sms = Module(new SMSPrefetcher(parentName = outer.parentName + "sms_"))
      sms.io_agt_en := RegNextN(io.csrCtrl.l1D_pf_enable_agt, 2, Some(false.B))
      sms.io_pht_en := RegNextN(io.csrCtrl.l1D_pf_enable_pht, 2, Some(false.B))
      sms.io_act_threshold := RegNextN(io.csrCtrl.l1D_pf_active_threshold, 2, Some(12.U))
      sms.io_act_stride := RegNextN(io.csrCtrl.l1D_pf_active_stride, 2, Some(30.U))
      sms.io_stride_en := RegNextN(io.csrCtrl.l1D_pf_enable_stride, 2, Some(true.B))
      Some(sms)
    // case Some(sms_sender_hyper : HyperPrefetchParams) =>
    //   val sms = Module(new SMSPrefetcher(parentName = outer.parentName + "sms_"))
    //   sms.io_agt_en := RegNextN(io.csrCtrl.l1D_pf_enable_agt, 2, Some(false.B))
    //   sms.io_pht_en := RegNextN(io.csrCtrl.l1D_pf_enable_pht, 2, Some(false.B))
    //   sms.io_act_threshold := RegNextN(io.csrCtrl.l1D_pf_active_threshold, 2, Some(12.U))
    //   sms.io_act_stride := RegNextN(io.csrCtrl.l1D_pf_active_stride, 2, Some(30.U))
    //   sms.io_stride_en := RegNextN(io.csrCtrl.l1D_pf_enable_stride, 2, Some(true.B))
    //   Some(sms)
    case _ => None
  }
  val hartId = p(XSCoreParamsKey).HartId
  val l1PrefetcherOpt: Option[L1Prefetcher] = coreParams.prefetcher.map {
    case _ =>
      val l1Prefetcher = Module(new L1Prefetcher())
      // l1Prefetcher.io.enable := Constantin.createRecord(s"enableL1StreamPrefetcher$hartId", initValue = true.B) 
      l1Prefetcher.io.enable := true.B
      l1Prefetcher.pf_ctrl.dynamic_depth := 32.U
      l1Prefetcher.pf_ctrl.flush := false.B
      l1Prefetcher.pf_ctrl.enable := true.B
      l1Prefetcher.pf_ctrl.confidence := 0.U
      l1Prefetcher.l2PfqBusy := false.B

      l1Prefetcher
  }
  val pfQueueSize = 1
  val l1PfReqQ  = Module(new Queue(new L1PrefetchReq(), pfQueueSize, flow = true, pipe = true))
  l1PrefetcherOpt match {
    // case Some(pf) => l1PfReqQ.io.enq <> Pipeline(in = pf.io.l1_req, depth = 1, pipe = false, name = Some("pf_queue_to_ldu_reg"))
    case Some(pf) => l1PfReqQ.io.enq <> pf.io.l1_req
    case None =>
      l1PfReqQ.io.enq.valid := false.B
      l1PfReqQ.io.enq.bits := DontCare
  }
  l1_pf_req.valid := l1PfReqQ.io.deq.valid
  l1_pf_req.bits := l1PfReqQ.io.deq.bits
  l1PfReqQ.io.deq.ready := Mux(l1PfReqQ.io.count === pfQueueSize.U && l1PfReqQ.io.enq.valid, true.B, l1_pf_req.ready)

  dcache.io.pf_req <> l1_pf_req
  
  prefetcherOpt.foreach{ pf => pf.io.l1_req.ready := false.B }
  prefetcherOpt match {
    case Some(sms) => // memblock can only have sms or not
      outer.pf_sender_opt match{
        case Some(sender) =>
        val pf_to_l2 = Pipe(sms.io.l2_req, 2)
        sender.out.head._1.addr_valid := pf_to_l2.valid
        sender.out.head._1.addr := pf_to_l2.bits.addr
        sender.out.head._1.l2_pf_en := RegNextN(io.csrCtrl.l2_pf_enable, 2, Some(true.B))
        sender.out.head._1.l2_pf_ctrl := RegNextN(io.csrCtrl.l2_pf_ctrl,2,Some(0.U(Csr_PfCtrlBits.W)))
        sms.io.enable := RegNextN(io.csrCtrl.l1D_pf_enable, 2, Some(false.B))
        case None => assert(cond = false, "Maybe from Config fault: Open SMS but dont have sms sender&recevier")
      }
    case None =>
  }
  private val pf_train_on_hit = RegNextN(io.csrCtrl.l1D_pf_train_on_hit, 2, Some(true.B))

  loadUnits.zipWithIndex.map(x => x._1.suggestName("LoadUnit_"+x._2))
  storeUnits.zipWithIndex.map(x => x._1.suggestName("StoreUnit_"+x._2))

  private val atomicsUnit = Module(new AtomicsUnit)

  io.writebackFromMou.valid := RegNext(atomicsUnit.io.out.valid, false.B)
  io.writebackFromMou.bits := RegEnable(atomicsUnit.io.out.bits, atomicsUnit.io.out.valid)
  atomicsUnit.io.out.ready := true.B

  private val stOut = staWritebacks

  val lsq     = Module(new LsqWrappper)
  val sbuffer = Module(new Sbuffer)

  lsq.io.tlb_hint <> io.tlb_hint

  io.lqDeq := lsq.io.lqDeq

  for(i <- 0 until LoadPipelineWidth){
    lsq.io.fastReplayStop(i) := loadUnits(i).io.fastReplayOut.fire
  }

  io.ldStopMemBlock.zip(lsq.io.replayQLdStop).zip(loadUnits.map(_.io.fastReplayOut.fire)).foreach({ case ((out, replayQ), fastReplay) =>
    out := RegNext(replayQ || fastReplay, false.B)
  })

  // if you wants to stress test dcache store, use FakeSbuffer
  // val sbuffer = Module(new FakeSbuffer)

  io.stIssuePtr := lsq.io.issuePtrExt

  dcache.io.hartId := io.hartId
  lsq.io.hartId := io.hartId
  sbuffer.io.hartId := io.hartId
  atomicsUnit.io.hartId := io.hartId

  private val redirectInDelay = Pipe(redirectIn)
  private val ldExeWbReqs = loadUnits.map(_.io.ldout)
  private val staExeWbReqs = storeUnits.map(_.io.stout)
  private val stdExeWbReqs = stdUnits.map(_.io.out)
  (lduWritebacks ++ staWritebacks ++ stdWritebacks)
    .zip(ldExeWbReqs ++ staExeWbReqs ++ stdExeWbReqs)
    .foreach({case(wb, out) =>
      wb.valid := out.valid && !out.bits.uop.ctrl.isVector
      wb.bits := out.bits
      wb.bits.wakeupValid := true.B
      out.ready := true.B
  })

  (vlduWritebacks ++ vstaWritebacks ++ vstdWritebacks)
    .zip(ldExeWbReqs ++ staExeWbReqs ++ stdExeWbReqs)
    .foreach({case(vwb, vout) =>
      vwb.valid := vout.valid && vout.bits.uop.ctrl.isVector
      vwb.bits := vout.bits
      vwb.bits.wakeupValid := false.B
      vout.ready := true.B
    })

  lduWritebacks.zip(ldExeWbReqs).foreach({case(wb, out) =>
    val redirect_wb = wb.bits.redirect
    val uop_out = out.bits.uop
    wb.bits.redirectValid := out.fire && out.bits.uop.ctrl.replayInst
    redirect_wb.robIdx := uop_out.robIdx
    redirect_wb.ftqIdx := uop_out.cf.ftqPtr
    redirect_wb.ftqOffset := uop_out.cf.ftqOffset
    redirect_wb.level := RedirectLevel.flush
    redirect_wb.interrupt := false.B
    redirect_wb.cfiUpdate := DontCare
    redirect_wb.cfiUpdate.isMisPred := false.B
    redirect_wb.isException := false.B
    redirect_wb.isLoadStore := false.B
    redirect_wb.isLoadLoad := true.B
    redirect_wb.isXRet := false.B
    redirect_wb.isFlushPipe := false.B
    redirect_wb.isPreWalk := false.B
  })

  vlduWritebacks.foreach(vwb => {
    vwb.bits.redirectValid := false.B
    vwb.bits.redirect := DontCare
  })


  // dtlb
  val total_tlb_ports = ld_tlb_ports + exuParameters.StuCnt
  val NUMSfenceDup = 3
  val NUMTlbCsrDup = 8
  val sfence_dup = Seq.fill(NUMSfenceDup)(Pipe(io.sfence))
  val tlbcsr_dup = Seq.fill(NUMTlbCsrDup)(RegNext(io.tlbCsr))


  val dtlb_ld_st = VecInit(Seq.fill(1) {
    val dtlb = Module(new TLB_backend(ld_tlb_ports + exuParameters.StuCnt, 2, OnedtlbParams))
    dtlb.io
  })
  if(!UseOneDtlb){
    dtlb_ld_st := DontCare
  }

  val dtlb_ld : Seq[TlbIO] = if(UseOneDtlb) {
    dtlb_ld_st.take(ld_tlb_ports)
  } else {
    VecInit(Seq.fill(1) {
      val tlb_ld =  Module(new TLB_backend(ld_tlb_ports, 2, ldtlbParams))
      tlb_ld.io // let the module have name in waveform
    })
  }

  val dtlb_st : Seq[TlbIO] = if(UseOneDtlb){
    dtlb_ld_st.drop(ld_tlb_ports)
  } else {
      VecInit(Seq.fill(1) {
      val tlb_st = Module(new TLB_backend(exuParameters.StuCnt, 1, sttlbParams))
      tlb_st.io // let the module have name in waveform
    })
  }

  private val loadTlbWakeup = Wire(Valid(new LoadTLBWakeUpBundle))
  private val contiguousVpn = WireInit(0.U((log2Up(tlbContiguous)).W))
  when(io.tlb_wakeUp.valid){
    contiguousVpn := OHToUInt(io.tlb_wakeUp.bits.pteidx)
    assert(PopCount(io.tlb_wakeUp.bits.pteidx) <= 1.U)
  }

  loadTlbWakeup.valid := io.tlb_wakeUp.valid
  loadTlbWakeup.bits.vpn := Cat(io.tlb_wakeUp.bits.entry.tag, contiguousVpn)
  loadTlbWakeup.bits.level := io.tlb_wakeUp.bits.entry.level.get
  lsq.io.tlbWakeup := loadTlbWakeup
  require(contiguousVpn.getWidth + io.tlb_wakeUp.bits.entry.tag.getWidth == loadTlbWakeup.bits.vpn.getWidth)

  val dtlb = dtlb_ld ++ dtlb_st
  val dtlb_reqs = dtlb.flatMap(_.requestor)
  val dtlb_pmps = dtlb.flatMap(_.pmp)
  dtlb.zip(sfence_dup.take(2)).foreach{ case (d,s) => d.sfence := s }
  dtlb.zip(tlbcsr_dup.take(2)).foreach{ case (d,c) => d.csr := c }
  if (refillBothTlb) {
    require(ldtlbParams.outReplace == sttlbParams.outReplace)
    require(ldtlbParams.outReplace)

    val replace = Module(new TlbReplace(total_tlb_ports, ldtlbParams))
    replace.io.apply_sep(dtlb_ld.map(_.replace) ++ dtlb_st.map(_.replace), io.ptw.resp.bits.data.entry.tag)
  } else {
    if(!UseOneDtlb){
      if (ldtlbParams.outReplace) {
        val replace_ld = Module(new TlbReplace(ld_tlb_ports, ldtlbParams))
        replace_ld.io.apply_sep(dtlb_ld.map(_.replace), io.ptw.resp.bits.data.entry.tag)
      }
      if (sttlbParams.outReplace) {
        val replace_st = Module(new TlbReplace(exuParameters.StuCnt, sttlbParams))
        replace_st.io.apply_sep(dtlb_st.map(_.replace), io.ptw.resp.bits.data.entry.tag)
      }
    }
  }

  val ptw_resp_next = RegEnable(io.ptw.resp.bits, io.ptw.resp.valid)
  val ptw_resp_v = RegNext(
    io.ptw.resp.valid && !(RegNext(sfence_dup.last.valid && tlbcsr_dup.last.satp.changed)),
    init = false.B
  )
  io.ptw.resp.ready := true.B

  dtlb.flatMap(a => a.ptw.req)
    .zipWithIndex
    .foreach{ case (tlb, i) =>
    tlb <> io.ptw.req(i)
    val vector_hit = if (refillBothTlb) Cat(ptw_resp_next.vector).orR
      else if (i < ld_tlb_ports) Cat(ptw_resp_next.vector.take(ld_tlb_ports)).orR
      else Cat(ptw_resp_next.vector.drop(ld_tlb_ports)).orR
    io.ptw.req(i).valid := tlb.valid && !(ptw_resp_v && vector_hit &&
      ptw_resp_next.data.hit(tlb.bits.vpn, RegNext(tlbcsr_dup(i).satp.asid), allType = true, ignoreAsid = true))
  }
  dtlb.foreach(_.ptw.resp.bits := ptw_resp_next.data)
  if (refillBothTlb || UseOneDtlb) {
    dtlb.foreach(_.ptw.resp.valid := ptw_resp_v && Cat(ptw_resp_next.vector).orR)
  } else {
    dtlb_ld.foreach(_.ptw.resp.valid := ptw_resp_v && Cat(ptw_resp_next.vector.take(ld_tlb_ports)).orR)
    dtlb_st.foreach(_.ptw.resp.valid := ptw_resp_v && Cat(ptw_resp_next.vector.drop(ld_tlb_ports)).orR)
  }
  dtlb.map(_.flushPipe.map(a => a := false.B)) // non-block doesn't need

  // fdi memory access check
  val fdi = Module(new MemFDI())
  fdi.io.distribute_csr <> csrCtrl.distribute_csr

  private val fdiCheckers = Seq.fill(exuParameters.LduCnt + exuParameters.StuCnt)(Module(new FDIMemChecker()))
  private val fdiCheckersIOs = fdiCheckers.map(_.io)

  val memFDIReq  = storeUnits.map(_.io.fdiReq) ++ loadUnits.map(_.io.fdiReq)
  val memFDIResp = storeUnits.map(_.io.fdiResp) ++ loadUnits.map(_.io.fdiResp)

  for( (dchecker,index) <- fdiCheckersIOs.zipWithIndex){
     dchecker.enableFDI := fdi.io.enableFDI
     dchecker.resource := fdi.io.entries
     dchecker.req := memFDIReq(index)
     memFDIResp(index) := dchecker.resp
  }

  // pmp
  val pmp = Module(new PMP())
  pmp.io.distribute_csr <> csrCtrl.distribute_csr

  val pmp_check = VecInit(Seq.fill(total_tlb_ports)(
    Module(new PMPChecker(3, leaveHitMux = true)).io
  ))
  val tlbcsr_pmp = tlbcsr_dup.drop(2).map(RegNext(_))
  for (((p,d),i) <- (pmp_check zip dtlb_pmps).zipWithIndex) {
    p.apply(tlbcsr_pmp(i).priv.dmode, tlbcsr_pmp(i).satp.mode, pmp.io.pmp, pmp.io.pma, d,
            pmp.io.spmp, tlbcsr_pmp(i).priv.sum, csrCtrl.spmp_enable)
    require(p.req.bits.size.getWidth == d.bits.size.getWidth)
  }

  val tdata = RegInit(VecInit(Seq.fill(TriggerNum)(0.U.asTypeOf(new MatchTriggerIO))))
  val tEnable = RegInit(VecInit(Seq.fill(TriggerNum)(false.B)))
  tEnable := csrCtrl.mem_trigger.tEnableVec
  when(csrCtrl.mem_trigger.tUpdate.valid) {
    tdata(csrCtrl.mem_trigger.tUpdate.bits.addr) := csrCtrl.mem_trigger.tUpdate.bits.tdata
  }

  val backendTriggerTimingVec = tdata.map(_.timing)
  val backendTriggerChainVec  = tdata.map(_.chain)

  XSDebug(tEnable.asUInt.orR, "Debug Mode: At least one store trigger is enabled\n")

  private def PrintTriggerInfo(enable: Bool, trigger: MatchTriggerIO)(implicit p: Parameters) = {
    XSDebug(enable, p"Debug Mode: Match Type is ${trigger.matchType}; select is ${trigger.select};" +
      p"timing is ${trigger.timing}; action is ${trigger.action}; chain is ${trigger.chain};" +
      p"tdata2 is ${Hexadecimal(trigger.tdata2)}")
  }
  for(j <- 0 until TriggerNum)
    PrintTriggerInfo(tEnable(j), tdata(j))

  for (i <- 0 until exuParameters.LduCnt) {
    loadUnits(i).io.redirect := redirectIn  //pipe inside
    lduIssues(i).rsFeedback.feedbackSlowLoad := loadUnits(i).io.feedbackSlow
    lduIssues(i).rsFeedback.feedbackFastLoad := loadUnits(i).io.feedbackFast

    lsq.io.loadEnqRAW(i) <> loadUnits(i).io.enqRAWQueue
    lsq.io.lduqueryAndUpdate(i) := loadUnits(i).io.lsq.s2_queryAndUpdateLQ

    val bnpi = outer.lduIssueNodes(i).in.head._2._1.bankNum / exuParameters.LduCnt
//    slduIssues(i).rsFeedback := DontCare
//    val selSldu = slduIssues(i).auxValid
//    val slduValid = slduIssues(i).issue.valid && !slduIssues(i).issue.bits.uop.robIdx.needFlush(loadUnits(i).io.redirect)
    val lduValid = lduIssues(i).issue.valid && !lduIssues(i).issue.bits.uop.robIdx.needFlush(loadUnits(i).io.redirect)
//    loadUnits(i).io.rsIdx := Mux(selSldu, slduIssues(i).rsIdx, lduIssues(i).rsIdx)
    loadUnits(i).io.rsIdx := lduIssues(i).rsIdx
    // get input form dispatch
//    loadUnits(i).io.rsIssueIn.valid := Mux(selSldu, slduValid, lduValid)
//    loadUnits(i).io.rsIssueIn.bits := Mux(selSldu, slduIssues(i).issue.bits, lduIssues(i).issue.bits)
//    loadUnits(i).io.auxValid := Mux(selSldu, slduIssues(i).auxValid, lduIssues(i).auxValid)
    loadUnits(i).io.rsIssueIn.valid := lduValid
    loadUnits(i).io.rsIssueIn.bits := lduIssues(i).issue.bits
    loadUnits(i).io.auxValid := lduIssues(i).auxValid
//    slduIssues(i).issue.ready := loadUnits(i).io.rsIssueIn.ready
    lduIssues(i).issue.ready := loadUnits(i).io.rsIssueIn.ready
//    when(selSldu){assert(lduIssues(i).issue.valid === false.B)}
    // dcache access
    loadUnits(i).io.dcache <> dcache.io.lsu.load(i)
    loadUnits(i).io.lduForwardMSHR <> dcache.io.lsu.lduForwardMSHR(i)
    loadUnits(i).io.loadReqHandledResp <> dcache.io.lsu.loadReqHandledResp

    dcache.io.lsu.load(i).req.valid := loadUnits(i).io.dcache.req.valid && !loadUnits(i).io.dcache.req.bits.robIdx.needFlush(Pipe(redirectIn))
    // forward
    loadUnits(i).io.lsq.forwardFromSQ <> lsq.io.forward(i)
    loadUnits(i).io.forwardFromSBuffer <> sbuffer.io.forward(i)
    // ld-ld violation check
    loadUnits(i).io.lsq.loadViolationQuery <> lsq.io.loadViolationQuery(i)
    loadUnits(i).io.csrCtrl       <> csrCtrl
    loadUnits(i).io.vmEnable := RegNext(vmEnable, false.B)
    // dtlb
    loadUnits(i).io.tlb <> dtlb_reqs.take(exuParameters.LduCnt)(i)
    dtlb_reqs.take(exuParameters.LduCnt)(i).req.valid := loadUnits(i).io.tlb.req.valid && !loadUnits(i).io.tlb.req.bits.robIdx.needFlush(Pipe(redirectIn))
    // pmp
    loadUnits(i).io.pmp <> pmp_check(i).resp
    //replayQueue
    lsq.io.replayQEnq(i) <> loadUnits(i).io.s3_enq_replayQueue
    loadUnits(i).io.replayQIssueIn <> lsq.io.replayQIssue(i)

    loadUnits(i).io.fastReplayIn.valid := RegNext(loadUnits(i).io.fastReplayOut.valid)
    loadUnits(i).io.fastReplayIn.bits := RegEnable(loadUnits(i).io.fastReplayOut.bits, loadUnits(i).io.fastReplayOut.fire)
    loadUnits(i).io.fastReplayOut.ready := RegNext(loadUnits(i).io.fastReplayIn.ready)
//    //cancel
//    io.earlyWakeUpCancel.foreach(w => w(i) := RegNext(loadUnits(i).io.cancel,false.B))
    //earlyWakeup and cancel
    io.lduEarlyWakeUp(i).cancel := RegNext(loadUnits(i).io.earlyWakeUp.cancel, false.B)
    io.lduEarlyWakeUp(i).wakeUp := loadUnits(i).io.earlyWakeUp.wakeUp
    // prefetch
    val pcDelay1Valid = RegNext(loadUnits(i).io.rsIssueIn.fire, false.B)
    val pcDelay1Bits = RegEnable(loadUnits(i).io.rsIssueIn.bits.uop.cf.pc, loadUnits(i).io.rsIssueIn.fire)
    val pcDelay2Bits = RegEnable(pcDelay1Bits, pcDelay1Valid)
    prefetcherOpt.foreach(pf => {
      pf.io.ld_in(i).valid := Mux(pf_train_on_hit,
        loadUnits(i).io.prefetch_train.valid,
        loadUnits(i).io.prefetch_train.valid && loadUnits(i).io.prefetch_train.bits.miss
      )
      pf.io.ld_in(i).bits := loadUnits(i).io.prefetch_train.bits
      pf.io.ld_in(i).bits.uop.cf.pc := pcDelay2Bits
    })
    l1PrefetcherOpt.foreach(pf => {
      // stride will train on miss or prefetch hit
      val source = loadUnits(i).io.prefetch_train_l1
      val hit_prefetch = loadUnits(i).io.hit_prefetch
      // pf.stride_train(i).valid := source.valid && source.bits.isFirstIssue && (
      //   source.bits.miss || hit_prefetch
      // )
      pf.stride_train(i).valid := source.valid && (
        source.bits.miss || hit_prefetch
      )
      pf.stride_train(i).bits := source.bits
      pf.stride_train(i).bits.uop.cf.pc := pcDelay2Bits
      // pf.io.ld_in(i).valid := source.valid && source.bits.isFirstIssue
      pf.io.ld_in(i).valid := source.valid
      pf.io.ld_in(i).bits := source.bits
    })

    lsq.io.loadExcepWbInfo(i) <> loadUnits(i).io.lsq.s2_excepWb2LQ
    lsq.io.trigger(i) <> loadUnits(i).io.lsq.trigger

    // --------------------------------
    // Load Triggers
    // --------------------------------
    val frontendTriggerTimingVec  = lduWritebacks(i).bits.uop.cf.trigger.frontendTiming
    val frontendTriggerChainVec   = lduWritebacks(i).bits.uop.cf.trigger.frontendChain
    val frontendTriggerHitVec     = lduWritebacks(i).bits.uop.cf.trigger.frontendHit
    val loadTriggerHitVec         = Wire(Vec(TriggerNum, Bool()))

    val triggerTimingVec  = VecInit(backendTriggerTimingVec.zip(frontendTriggerTimingVec).map { case (b, f) => b || f } )
    val triggerChainVec   = VecInit(backendTriggerChainVec.zip(frontendTriggerChainVec).map { case (b, f) => b || f } )
    val triggerHitVec     = VecInit(loadTriggerHitVec.zip(frontendTriggerHitVec).map { case (b, f) => b || f })

    val triggerCanFireVec = Wire(Vec(TriggerNum, Bool()))

    for (j <- 0 until TriggerNum) {
      loadUnits(i).io.trigger(j).tdata2     := tdata(j).tdata2
      loadUnits(i).io.trigger(j).matchType  := tdata(j).matchType
      loadUnits(i).io.trigger(j).tEnable    := tEnable(j) && tdata(j).load
      // Just let load triggers that match data unavailable
      loadTriggerHitVec(j) := loadUnits(i).io.trigger(j).addrHit && !tdata(j).select
    }
    TriggerCheckCanFire(TriggerNum, triggerCanFireVec, triggerHitVec, triggerTimingVec, triggerChainVec)

    lduWritebacks(i).bits.uop.cf.trigger.backendHit      := triggerHitVec
    lduWritebacks(i).bits.uop.cf.trigger.backendCanFire  := triggerCanFireVec
    XSDebug(lduWritebacks(i).bits.uop.cf.trigger.getBackendCanFire && lduWritebacks(i).valid, p"Debug Mode: Load Inst No.${i}" +
    p"has trigger fire vec ${lduWritebacks(i).bits.uop.cf.trigger.backendCanFire}\n")
  }

  //mmio writeback
  val mmioCanWbVec = loadUnits.map(_.io.mmioWb.ready)
  val mmioCanWb = mmioCanWbVec.reduce(_|_)
  val mmioWritePortIdx = WireInit(0.U(log2Up(LoadPipelineWidth + 1).W))
  when(mmioCanWb){
    mmioWritePortIdx := PriorityEncoder(mmioCanWbVec)
  }

  loadUnits.map(_.io.mmioWb).zipWithIndex.foreach({ case (wb, idx) =>
    when(idx.U === mmioWritePortIdx && mmioCanWb) {
      wb.valid := lsq.io.mmioWb.valid
      wb.bits := lsq.io.mmioWb.bits
    }.otherwise{
      wb.valid := false.B
      wb.bits := 0.U.asTypeOf(new ExuOutput)
    }
  })

  lsq.io.mmioWb.ready := mmioCanWb

  // Prefetcher
  prefetcherOpt.foreach(pf => {
    dtlb_reqs(ld_tlb_ports - 2) <> pf.io.tlb_req
  })
  l1PrefetcherOpt.foreach(pf => {
    dtlb_reqs(ld_tlb_ports - 1) <> pf.io.tlb_req
  })

  // StoreUnit
  for (i <- 0 until exuParameters.StuCnt) {
    val stu = storeUnits(i)

    stdUnits(i).io.in <> stdIssues(i).issue
    stdUnits(i).io.redirect := Pipe(redirectIn)

    loadUnits.foreach({req =>
      req.io.storeViolationQuery(i) := stu.io.storeViolationQuery
    })

    stu.io.redirect     <> Pipe(redirectIn)
    stu.io.redirect_dup.foreach({ case d => {d <> Pipe(redirectIn)}})
    staIssues(i).rsFeedback.feedbackSlowStore := stu.io.feedbackSlow
    stu.io.rsIdx        :=  staIssues(i).rsIdx
    // NOTE: just for dtlb's perf cnt
    stu.io.vmEnable := RegNext(vmEnable, false.B)
    stu.io.stin         <> staIssues(i).issue
    stu.io.lsq          <> lsq.io.storeIn(i)
    stu.io.lsq_replenish <> lsq.io.storeInRe(i)
    lsq.io.storeViolationQuery(i) := stu.io.storeViolationQuery
    // dtlb
    stu.io.tlb          <> dtlb_reqs.drop(ld_tlb_ports)(i)
    stu.io.pmp          <> pmp_check(i+ld_tlb_ports).resp
    stu.io.stout        <> lsq.io.storeAddrIn(i)
    // Lsq to sta unit
    lsq.io.storeMaskIn(i) <> stu.io.storeMaskOut

    // Lsq to std unit's rs
    lsq.io.storeDataIn(i) := stData(i)
    stData(i).ready := true.B

    // 1. sync issue info to store set LFST
    // 2. when store issue, broadcast issued sqPtr to wake up the following insts
    io.stIn(i).valid := staIssues(i).issue.valid
    io.stIn(i).bits := staIssues(i).issue.bits

    stu.io.stout.ready := true.B

    // -------------------------
    // Store Triggers
    // -------------------------
    val frontendTriggerTimingVec  = stOut(i).bits.uop.cf.trigger.frontendTiming
    val frontendTriggerChainVec   = stOut(i).bits.uop.cf.trigger.frontendChain
    val frontendTriggerHitVec     = stOut(i).bits.uop.cf.trigger.frontendHit

    val storeTriggerHitVec        = WireInit(VecInit(Seq.fill(TriggerNum)(false.B)))

    val triggerTimingVec  = VecInit(backendTriggerTimingVec.zip(frontendTriggerTimingVec).map { case (b, f) => b || f } )
    val triggerChainVec   = VecInit(backendTriggerChainVec.zip(frontendTriggerChainVec).map { case (b, f) => b || f } )
    val triggerHitVec     = VecInit(storeTriggerHitVec.zip(frontendTriggerHitVec).map { case (b, f) => b || f })

    val triggerCanFireVec = WireInit(VecInit(Seq.fill(TriggerNum)(false.B)))

    when(stOut(i).fire){
      for (j <- 0 until TriggerNum) {
        storeTriggerHitVec(j) := !tdata(j).select && TriggerCmp(
          stOut(i).bits.debug.vaddr,
          tdata(j).tdata2,
          tdata(j).matchType,
          tEnable(j) && tdata(j).store
        )
      }
      TriggerCheckCanFire(TriggerNum, triggerCanFireVec, triggerHitVec, triggerTimingVec, triggerChainVec)

      stOut(i).bits.uop.cf.trigger.backendHit := triggerHitVec
      stOut(i).bits.uop.cf.trigger.backendCanFire := triggerCanFireVec
    }
  }

  // mmio store writeback will use store writeback port 0
  lsq.io.mmioStout.ready := false.B
  when (lsq.io.mmioStout.valid && !staExeWbReqs(0).valid) {
    stOut(0).valid := true.B
    stOut(0).bits  := lsq.io.mmioStout.bits
    lsq.io.mmioStout.ready := true.B
  }

  // Lsq
  lsq.io.rob            <> io.lsqio.rob
  lsq.io.enq            <> io.enqLsq
  lsq.io.brqRedirect    <> Pipe(redirectIn)
  lsq.io.tlDchannelWakeup := dcache.io.lsu.tl_d_channel
  lsq.io.mshrFull := dcache.io.mshrFull
  staWritebacks.head.bits.redirectValid := lsq.io.rollback.valid
  staWritebacks.head.bits.redirect := lsq.io.rollback.bits
  staWritebacks.tail.foreach(e => {
    e.bits.redirectValid := false.B
    e.bits.redirect := DontCare
  })
  vstaWritebacks.foreach(e => {
    e.bits.redirectValid := false.B
    e.bits.redirect := DontCare
  })
  AddPipelineReg(lsq.io.uncache.req, uncache.io.lsq.req, false.B)
  AddPipelineReg(uncache.io.lsq.resp, lsq.io.uncache.resp, false.B)
  // delay dcache refill for 1 cycle for better timing
  // TODO: remove RegNext after fixing refill paddr timing
//  lsq.io.dcache.valid := RegNext(dcache.io.lsu.lsq.valid)
//  lsq.io.dcache.bits := RegEnable(dcache.io.lsu.lsq.bits,dcache.io.lsu.lsq.valid)
  lsq.io.release        := dcache.io.lsu.release
  lsq.io.lqCancelCnt <> io.lqCancelCnt
  lsq.io.sqCancelCnt <> io.sqCancelCnt
  lsq.io.sqDeq <> io.sqDeq
  for (i <- 0 until StorePipelineWidth){
    lsq.io.storeDataWbPtr(i).valid := stData(i).fire
    lsq.io.storeDataWbPtr(i).bits  := stData(i).bits.uop.sqIdx
  }
  // LSQ to store buffer
  lsq.io.sbuffer        <> sbuffer.io.in
  lsq.io.sqempty        <> sbuffer.io.sqempty

  // Sbuffer
  sbuffer.io.csrCtrl    <> csrCtrl
  sbuffer.io.dcache     <> dcache.io.lsu.store


  // TODO: if dcache sbuffer resp needs to ne delayed
  // sbuffer.io.dcache.pipe_resp.valid := RegNext(dcache.io.lsu.store.pipe_resp.valid)
  // sbuffer.io.dcache.pipe_resp.bits := RegNext(dcache.io.lsu.store.pipe_resp.bits)

  // flush sbuffer
  val fenceFlush = io.fenceToSbuffer.flushSb
  val atomicsFlush = atomicsUnit.io.flush_sbuffer.valid
  io.fenceToSbuffer.sbIsEmpty := RegNext(sbuffer.io.flush.empty)
  // if both of them tries to flush sbuffer at the same time
  // something must have gone wrong
  assert(!(fenceFlush && atomicsFlush))
  sbuffer.io.flush.valid := RegNext(fenceFlush || atomicsFlush)

  // AtomicsUnit: AtomicsUnit will override other control signials,
  // as atomics insts (LR/SC/AMO) will block the pipeline
  val s_normal :: s_atomics :: Nil = Enum(2)
  val state = RegInit(s_normal)

  atomicsUnit.io.in.valid := io.issueToMou.valid
  atomicsUnit.io.in.bits  := io.issueToMou.bits
  io.issueToMou.ready := atomicsUnit.io.in.ready
  atomicsUnit.io.redirect := Pipe(redirectIn)

  // TODO: complete amo's pmp support
  private val amoTlb = dtlb_ld(0).requestor(0)
  atomicsUnit.io.dtlb.resp.valid := false.B
  atomicsUnit.io.dtlb.resp.bits  := DontCare
  atomicsUnit.io.dtlb.req.ready  := amoTlb.req.ready
  atomicsUnit.io.pmpResp := pmp_check(0).resp

  atomicsUnit.io.dcache <> dcache.io.lsu.atomics
  atomicsUnit.io.flush_sbuffer.empty := sbuffer.io.flush.empty

  atomicsUnit.io.csrCtrl := csrCtrl

  // for atomicsUnit, it uses loadUnit(0)'s TLB port
  when(state === s_normal){
    when(atomicsUnit.io.in.valid){
      state := s_atomics
    }
  }

  when (state === s_atomics) {
    atomicsUnit.io.dtlb <> amoTlb
    when(atomicsUnit.io.out.fire){
      state := s_normal
    }
  }

  lsq.io.exceptionAddr.isStore := io.lsqio.exceptionAddr.isStore
  // Exception address is used several cycles after flush.
  // We delay it by 10 cycles to ensure its flush safety.
  val atomicsException = RegInit(false.B)
  when (DelayN(redirectIn.valid, 10) && atomicsException) {
    atomicsException := false.B
  }.elsewhen (atomicsUnit.io.exceptionAddr.valid) {
    atomicsException := true.B
  }
  val atomicsExceptionAddress = RegEnable(atomicsUnit.io.exceptionAddr.bits, atomicsUnit.io.exceptionAddr.valid)
  io.lsqio.exceptionAddr.vaddr := RegEnable(Mux(atomicsException, atomicsExceptionAddress, lsq.io.exceptionAddr.vaddr),
                                  atomicsUnit.io.exceptionAddr.valid || lsq.io.exceptionAddrValid)
  XSError(atomicsException && atomicsUnit.io.in.valid, "new instruction before exception triggers\n")

  io.memInfo.sqFull := RegNext(lsq.io.sqFull)
  io.memInfo.lqFull := RegNext(lsq.io.lqFull)
  io.memInfo.dcacheMSHRFull := RegNext(dcache.io.mshrFull)

  val mbistPipeline = if(coreParams.hasMbist && coreParams.hasShareBus) {
    MBISTPipeline.PlaceMbistPipeline(2, s"${outer.parentName}_mbistPipe", true)
  } else {
    None
  }

  io.lqFull := lsq.io.lqFull
  io.sqFull := lsq.io.sqFull

  val ldDeqCount = PopCount(lduIssues.map(_.issue.valid))
  val stDeqCount = PopCount(staIssues.map(_.issue.valid))
  val rsDeqCount = ldDeqCount + stDeqCount
  XSPerfAccumulate("load_rs_deq_count", ldDeqCount)
  XSPerfHistogram("load_rs_deq_count", ldDeqCount, true.B, 1, 2, 1)
  XSPerfAccumulate("store_rs_deq_count", stDeqCount)
  XSPerfHistogram("store_rs_deq_count", stDeqCount, true.B, 1, 2, 1)
  XSPerfAccumulate("ls_rs_deq_count", rsDeqCount)

  val pfevent = Module(new PFEvent)
  pfevent.io.distribute_csr := csrCtrl.distribute_csr
  val csrevents = pfevent.io.hpmevent.slice(16,24)

  val memBlockPerfEvents = Seq(
    ("ldDeqCount", ldDeqCount),
    ("stDeqCount", stDeqCount),
  )

  val perfFromUnits = (loadUnits ++ Seq(sbuffer, lsq, dcache)).flatMap(_.getPerfEvents)
  val perfFromIO    = io.perfEventsPTW.map(x => ("perfEventsPTW", x.value))
  val perfBlock     = Seq(("ldDeqCount", ldDeqCount),
                          ("stDeqCount", stDeqCount))
  // let index = 0 be no event
  val allPerfEvents = Seq(("noEvent", 0.U)) ++ perfFromUnits ++ perfFromIO ++ perfBlock

  if (printEventCoding) {
    for (((name, inc), i) <- allPerfEvents.zipWithIndex) {
      println("MemBlock perfEvents Set", name, inc, i)
    }
  }

  val allPerfInc = allPerfEvents.map(_._2.asTypeOf(new PerfEvent))
  val perfEvents = HPerfMonitor(csrevents, allPerfInc).getPerfEvents
  generatePerfEvent()

  val clock_debug = RegInit(false.B)
  clock_debug := ~clock_debug
  dontTouch(clock_debug)
}
