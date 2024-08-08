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
import xiangshan._
import xiangshan.backend.execute.fu.fpu.FPU
import xiangshan.backend.rob.RobPtr
import xiangshan.cache._
import xiangshan.frontend.FtqPtr
import xiangshan.ExceptionNO._
import xiangshan.backend.execute.fu.FuConfigs
import xiangshan.backend.issue.SelectPolicy
import xiangshan.mem.lsqueue.{LSQExceptionGen, LoadRAWQueueDataModule}
import xs.utils.perf.HasPerfLogging
import xiangshan.cache.mmu.TlbHintIO

class LqPtr(implicit p: Parameters) extends CircularQueuePtr[LqPtr](
  p => p(XSCoreParamsKey).LoadQueueSize
){
}

object LqPtr {
  def apply(f: Bool, v: UInt)(implicit p: Parameters): LqPtr = {
    val ptr = Wire(new LqPtr)
    ptr.flag := f
    ptr.value := v
    ptr
  }
}

trait HasLoadHelper { this: XSModule =>
  def rdataHelper(uop: MicroOp, rdata: UInt): UInt = {
    val fpWen = uop.ctrl.fpWen
    LookupTree(uop.ctrl.fuOpType, List(
      LSUOpType.lb   -> SignExt(rdata(7, 0) , XLEN),
      LSUOpType.lh   -> SignExt(rdata(15, 0), XLEN),
      /*
          riscv-spec-20191213: 12.2 NaN Boxing of Narrower Values
          Any operation that writes a narrower result to an f register must write
          all 1s to the uppermost FLEN−n bits to yield a legal NaN-boxed value.
      */
      LSUOpType.lw   -> Mux(fpWen, FPU.box(rdata, FPU.S), SignExt(rdata(31, 0), XLEN)),
      LSUOpType.ld   -> Mux(fpWen, FPU.box(rdata, FPU.D), SignExt(rdata(63, 0), XLEN)),
      LSUOpType.lbu  -> ZeroExt(rdata(7, 0) , XLEN),
      LSUOpType.lhu  -> ZeroExt(rdata(15, 0), XLEN),
      LSUOpType.lwu  -> ZeroExt(rdata(31, 0), XLEN),
    ))
  }
}

class LqEnqIO(implicit p: Parameters) extends XSBundle {
  val canAccept = Output(Bool())
  val sqCanAccept = Input(Bool())
  val needAlloc = Vec(exuParameters.LsExuCnt, Input(Bool()))
  val req = Vec(exuParameters.LsExuCnt, Flipped(ValidIO(new MicroOp)))
  val resp = Vec(exuParameters.LsExuCnt, Output(new LqPtr))
  val reqNum = Input((UInt(exuParameters.LsExuCnt.W)))
}

class LoadMMIOPaddrWriteBundle(implicit p: Parameters) extends XSBundle {
  val paddr = Output(UInt(PAddrBits.W))
  val lqIdx = Output(new LqPtr)
}

class ReleaseEntryBundle(implicit p: Parameters) extends XSBundle with HasDCacheParameters{
  val sig = UInt((loadLoadViolationCheckBits - log2Up(DCacheSets) - log2Up(DCacheWays)).W)
  val setIdx = UInt(log2Up(DCacheSets).W)
  val wayIdx = UInt(log2Up(DCacheWays).W)
}


class LoadQueueDataUpdateBundle(implicit p: Parameters) extends XSBundle with HasDCacheParameters{
  val lqPtr = new LqPtr
  //for load load violation
  val dataIsFromDCache = Bool()
  val wayIdx = UInt(log2Up(DCacheWays).W)

  val paddr = UInt(PAddrBits.W)
  val debug_mmio = Bool()
}

class LqTriggerIO(implicit p: Parameters) extends XSBundle {
  val hitLoadAddrTriggerHitVec = Input(Vec(TriggerNum, Bool()))
  val lqLoadAddrTriggerHitVec = Output(Vec(TriggerNum, Bool()))
}

// Load Queue
class LoadQueue(implicit p: Parameters) extends XSModule
  with HasDCacheParameters
  with HasCircularQueuePtrHelper
  with HasLoadHelper
  with HasPerfEvents
  with HasPerfLogging
{
  val io = IO(new Bundle() {
    //dispatch enqueue
    val enq = new LqEnqIO
    //replayQueue
    val replayQIssue = Vec(LoadPipelineWidth, DecoupledIO(new ReplayQueueIssueBundle))
    val replayQEnq = Vec(LoadPipelineWidth, Flipped(DecoupledIO(new LoadToReplayQueueBundle)))
    val replayQFull = Output(Bool())
    val replayQLdStop = Output(Vec(LoadPipelineWidth, Bool()))
    val fastReplayStop = Input(Vec(LoadPipelineWidth, Bool()))
    //loadUnit update
    val loadExcepWbInfo = Vec(LoadPipelineWidth, Flipped(Valid(new LqWriteBundle)))  //from loadUnit S2
    //mmio
    val mmioWb = DecoupledIO(new ExuOutput)
    val uncache = new UncacheWordIO
    //wakeup info
    val tlbWakeup = Flipped(ValidIO(new LoadTLBWakeUpBundle))
    val tlDchannelWakeup = Input(Vec(2, new DCacheTLDBypassLduIO))
    val mshrFull = Input(Bool())
    //store load violation
    val loadEnqRAW = Vec(LoadPipelineWidth, Flipped(new LoadEnqRAWBundle)) //LoadUnit S2 enq
    val stLdViolationQuery = Vec(StorePipelineWidth, Flipped(Valid(new storeRAWQueryBundle))) //storeUnit S1
    //load load violation
    val lduqueryAndUpdate = Vec(LoadPipelineWidth, Flipped(ValidIO(new LoadQueueDataUpdateBundle))) //from loadUnit S2
    val ldLdViolationResp = Vec(LoadPipelineWidth, Flipped(new LoadViolationQueryIO))
    //storeQueue info
    val stAddrReadyPtr = Input(new SqPtr)
    val stAddrAllReady = Input(Bool())
    val stDataReadyVec = Input(Vec(StoreQueueSize, Bool()))
    val sqEmpty = Input(Bool())
    val stDataReadySqPtr = Input(new SqPtr)
    val storeDataWbPtr = Vec(StorePipelineWidth, Flipped(Valid(new SqPtr)))
    //other
    val trigger = Vec(LoadPipelineWidth, new LqTriggerIO)
    val rollback = Output(Valid(new Redirect))
    val brqRedirect = Flipped(ValidIO(new Redirect))
    val lqSafeDeq = Input(new RobPtr)
    val robHead = Input(new RobPtr)
    val release = Flipped(ValidIO(new Release))
    val exceptionAddr = ValidIO(new ExceptionAddrIO)
    val lqFull = Output(Bool())
    val lqCancelCnt = Output(UInt(log2Up(LoadQueueSize + 1).W))
    val lqDeq = Output(UInt(log2Up(CommitWidth + 1).W))
    val replayQFreeNum = Output(UInt(log2Up(LoadReplayQueueSize + 1).W))
    //debug info
    val debug_deqPtr = Input(new RobPtr)
    val debug_enqPtr = Input(new RobPtr)
  })

  private val replayQueue = Module(new LoadReplayQueue(enablePerf = true))
  private val rawQueue = Module(new LoadRAWQueue)

  println("LoadQueue: size:" + LoadQueueSize)

  private val uop = Reg(Vec(LoadQueueSize, new MicroOp))
  private val releaseDataModule = Module(new LQReleaseDataModule(LoadQueueSize, LoadPipelineWidth, LoadPipelineWidth))
  releaseDataModule.io := DontCare

  private val exceptionGen = Module(new LSQExceptionGen(LoadPipelineWidth, FuConfigs.lduCfg))
  private val exceptionInfo = exceptionGen.io.out

  private val allocated = RegInit(VecInit(List.fill(LoadQueueSize)(false.B))) // lq entry has been allocated
  private val readyToLeave = RegInit(VecInit(List.fill(LoadQueueSize)(false.B))) // lq entry is ready to leave
  private val writebacked = RegInit(VecInit(List.fill(LoadQueueSize)(false.B))) // inst has been writebacked to CDB
  private val released = RegInit(VecInit(List.fill(LoadQueueSize)(false.B))) // load data has been released by dcache
  private val dataFromDCache = RegInit(VecInit(List.fill(LoadQueueSize)(false.B)))

  private val debug_mmio = RegInit(VecInit(List.fill(LoadQueueSize)(false.B))) // mmio: inst is an mmio inst
  private val debug_paddr = RegInit(VecInit(List.fill(LoadQueueSize)(0.U(PAddrBits.W)))) // mmio: inst is an mmio inst

  private val enqPtrExt = RegInit(VecInit((0 until io.enq.req.length).map(_.U.asTypeOf(new LqPtr))))
  private val deqPtrExt = RegInit(0.U.asTypeOf(new LqPtr))
  private val deqPtrExtNext = Wire(new LqPtr)
  dontTouch(deqPtrExt)

  private val enqPtr = enqPtrExt(0).value
  private val deqPtr = deqPtrExt.value

  private val validCount = distanceBetween(enqPtrExt(0), deqPtrExt)
  private val allowEnqueue = validCount <= (LoadQueueSize - 2).U

  private val deqMask = UIntToMask(deqPtr, LoadQueueSize)
  private val enqMask = UIntToMask(enqPtr, LoadQueueSize)

  private val release1cycle = io.release
  private val release2cycle_valid = RegNext(io.release.valid)
  private val release2cycle_paddr = RegEnable(io.release.bits.paddr, io.release.valid)
  private val release2cycle_wayIdx = RegEnable(io.release.bits.wayIdx, io.release.valid)

  replayQueue.io.redirect := io.brqRedirect
  replayQueue.io.enq <> io.replayQEnq
  replayQueue.io.replayQIssue.foreach(_.ready := true.B)
  replayQueue.io.tlDchannelWakeup := io.tlDchannelWakeup
  replayQueue.io.stDataReadyVec := io.stDataReadyVec
  replayQueue.io.storeDataWbPtr := io.storeDataWbPtr
  replayQueue.io.sqEmpty := io.sqEmpty
  replayQueue.io.stDataReadySqPtr := io.stDataReadySqPtr
  replayQueue.io.mshrFull := io.mshrFull
  replayQueue.io.tlbWakeup := io.tlbWakeup
  replayQueue.io.loadDeqPtr := deqPtrExt
  replayQueue.io.robHead := io.robHead
  replayQueue.io.stAddrReadyPtr := io.stAddrReadyPtr
  io.replayQFreeNum := replayQueue.io.freeNum


  io.mmioWb <> replayQueue.io.mmioWb
  io.uncache <> replayQueue.io.mmioReq
  io.replayQLdStop := replayQueue.io.replayQLdStop
  replayQueue.io.fastReplayStop := io.fastReplayStop
  io.replayQIssue <> replayQueue.io.replayQIssue
  io.replayQFull := replayQueue.io.replayQFull
  replayQueue.io.degbugInfo.debug_deqPtr := io.debug_deqPtr
  replayQueue.io.degbugInfo.debug_enqPtr := io.debug_enqPtr

  XSPerfAccumulate("replayq", replayQueue.io.replayQIssue(0).fire || replayQueue.io.replayQIssue(1).fire)
  println("LoadQueue: size:" + LoadQueueSize)

  rawQueue.io.redirect := io.brqRedirect
  rawQueue.io.stAddrReadyPtr := io.stAddrReadyPtr
  rawQueue.io.storeQuery := io.stLdViolationQuery
  rawQueue.io.loadEnq <> io.loadEnqRAW
  rawQueue.io.stAddrAllReady := io.stAddrAllReady
  io.rollback := rawQueue.io.rollback

  replayQueue.io.rawIsFull := rawQueue.io.isFull

  /**
    * Enqueue at dispatch
    *
    * Currently, LoadQueue only allows enqueue when #emptyEntries > EnqWidth
    */
  io.enq.canAccept := allowEnqueue

  //for uop(index).robidx clock-gating
  private val wen_robIdx = Wire(Vec(LoadQueueSize, Bool()))
  private val canEnqueue = io.enq.req.map(_.valid)
  private val enqCancel = io.enq.req.map(_.bits.robIdx.needFlush(io.brqRedirect))

  (0 until LoadQueueSize).foreach(i => {
    val req_valid = Wire(Vec(exuParameters.LsExuCnt, Bool()))
    (0 until exuParameters.LsExuCnt).foreach(j => {
      req_valid(j) := canEnqueue(j) && !enqCancel(j) && (io.enq.req(j).bits.lqIdx.value === i.U)
    })
    wen_robIdx(i) := req_valid.asUInt.orR
    val w_addr = OHToUInt(req_valid)
    val w_data = io.enq.req(w_addr).bits.robIdx
    when(wen_robIdx(i)) {
      uop(i).robIdx := w_data
    }
  })

  for (i <- 0 until io.enq.req.length) {
    val offset = if (i == 0) 0.U else PopCount(io.enq.needAlloc.take(i))
    val lqIdx = enqPtrExt(offset)
    val index = io.enq.req(i).bits.lqIdx.value
    when (canEnqueue(i) && !enqCancel(i)) {
      allocated(index) := true.B
      readyToLeave(index) := false.B
      writebacked(index) := false.B
      dataFromDCache(index) := false.B
      released(index) := false.B

      XSError(!io.enq.canAccept || !io.enq.sqCanAccept, s"must accept $i\n")
      XSError(index =/= lqIdx.value, s"must be the same entry $i\n")
    }
    io.enq.resp(i) := lqIdx
  }
  XSDebug(p"(ready, valid): ${io.enq.canAccept}, ${Binary(Cat(io.enq.req.map(_.valid)))}\n")

  private val lastCycleRedirect_valid = RegNext(io.brqRedirect.valid)
  private val lastCycleRedirect_bits = RegEnable(io.brqRedirect.bits, io.brqRedirect.valid)
  private val lastlastCycleRedirect_valid = RegNext(lastCycleRedirect_valid)
  private val lastlastCycleRedirect_bits = RegEnable(lastCycleRedirect_bits, lastCycleRedirect_valid)

  private val lastCycleRedirect = Wire(io.brqRedirect.cloneType)
  lastCycleRedirect.valid := lastCycleRedirect_valid
  lastCycleRedirect.bits := lastCycleRedirect_bits

  private val lastlastCycleRedirect = Wire(io.brqRedirect.cloneType)
  lastlastCycleRedirect.valid := lastlastCycleRedirect_valid
  lastlastCycleRedirect.bits := lastlastCycleRedirect_bits

  private def ReleasePAddrHash(paddr: UInt, wayIdx: UInt, targetWidth: Int = loadLoadViolationCheckBits): ReleaseEntryBundle = {
    require(paddr.getWidth == PAddrBits)
    require(wayIdx.getWidth == log2Up(DCacheWays))
    val opWidth: Int = PAddrBits - untagBits
    val setIdxWidth: Int = untagBits - blockOffBits

    require((opWidth + wayIdx.getWidth + setIdxWidth) >= targetWidth)
    val setIdx = get_idx(paddr)

    val eachWidth: Int = targetWidth - setIdxWidth - wayIdx.getWidth
    val num = opWidth / eachWidth

    val slice = Wire(Vec(num, UInt(eachWidth.W)))

    for(i <- 0 until num){
      slice(i) := paddr(untagBits + eachWidth * (i + 1) - 1,untagBits + eachWidth * i)
    }

    val hashTag: UInt = slice.reduce(_^_)
    val targetBits = Wire(new ReleaseEntryBundle)

    targetBits.setIdx := setIdx
    targetBits.sig := hashTag
    targetBits.wayIdx := wayIdx

    targetBits
  }

  for (i <- 0 until LoadPipelineWidth) {
    releaseDataModule.io.write(i).wen := false.B
    val wbIdx = io.lduqueryAndUpdate(i).bits.lqPtr.value
    val wbPaddr = io.lduqueryAndUpdate(i).bits.paddr
    val wbWayIdx = io.lduqueryAndUpdate(i).bits.wayIdx
    val wbReleaseData = ReleasePAddrHash(wbPaddr,wbWayIdx)

    when(io.lduqueryAndUpdate(i).fire){
      writebacked(wbIdx) := true.B
      dataFromDCache(wbIdx) := io.lduqueryAndUpdate(i).bits.dataIsFromDCache

      debug_mmio(wbIdx) := io.lduqueryAndUpdate(i).bits.debug_mmio
      debug_paddr(wbIdx) := io.lduqueryAndUpdate(i).bits.paddr

      releaseDataModule.io.write(i).wen := true.B
      releaseDataModule.io.write(i).entryAddr := wbIdx
      releaseDataModule.io.write(i).data := wbReleaseData

      released(wbIdx) := release2cycle_valid && (ReleasePAddrHash(wbPaddr, wbWayIdx) === ReleasePAddrHash(release2cycle_paddr,release2cycle_wayIdx)) ||
        release1cycle.valid && (ReleasePAddrHash(wbPaddr, wbWayIdx) === ReleasePAddrHash(release1cycle.bits.paddr,release1cycle.bits.wayIdx))
    }

    when(replayQueue.io.mmioWb.fire){
      val lqIdx = replayQueue.io.mmioWb.bits.uop.lqIdx.value
      writebacked(lqIdx) := true.B
    }
  }

  //when scalar load is committed and index-ordered instruction is committed, allocated flag is false
  private val readyToDeq = Reg(Vec(LoadQueueSize, Bool()))
  for (i <- 0 until LoadQueueSize) {
    readyToDeq(i) := readyToLeave(i) & allocated(i) & writebacked(i) & !io.brqRedirect.valid
  }
  private val deqWindow = Seq.tabulate(CommitWidth)(idx => (deqPtrExt + idx.U).value)
  private val deqVec = deqWindow.map(addr => readyToDeq(addr) & !io.brqRedirect.valid)
  private val deqBlocked = deqVec.map(!_) :+ true.B
  private val deqNum = PriorityEncoder(deqBlocked)
  io.lqDeq := RegNext(deqNum)
  uop.zip(readyToLeave).zipWithIndex.foreach({case((u, r), idx) =>
    val updateReadyToLeave = Wire(Bool())
    updateReadyToLeave := u.robIdx <= io.lqSafeDeq && allocated(idx) && !r
    when(updateReadyToLeave){
      r := true.B
    }
  })
  (0 until CommitWidth).map(i => {
    when(deqNum > i.U){
      allocated((deqPtrExt+i.U).value) := false.B
      XSError(!allocated((deqPtrExt+i.U).value), s"why release invalid entry $i?\n")
    }
  })


  // Load-Load Memory violation query
  (0 until LoadPipelineWidth).foreach(i => {
    val req = io.lduqueryAndUpdate(i)
    val resp = io.ldLdViolationResp(i).s3_resp

    //s2 load load violation Req
    val reqPaddr = req.bits.paddr
    val reqWayIdx = req.bits.wayIdx
    val reqReleaseEntry = ReleasePAddrHash(reqPaddr, reqWayIdx)
    releaseDataModule.io.load_query(i).req_data := reqReleaseEntry
    val startIndex = req.bits.lqPtr.value
    val lqIdxMask = UIntToMask(startIndex, LoadQueueSize)
    val xorMask = lqIdxMask ^ enqMask
    val sameFlag = req.bits.lqPtr.flag
    val ldToEnqPtrMask = Mux(sameFlag, xorMask, ~xorMask)

    val s0_shouldCheck = WireInit(VecInit((0 until LoadQueueSize).map({ j =>
        ldToEnqPtrMask(j) &&
        allocated(j) &&
        dataFromDCache(j) &&
        released(j)
    })))

    val s0_addrMatch = WireInit(VecInit((0 until LoadQueueSize).map({j =>
      releaseDataModule.io.load_query(i).resp_mask(j)
    })))

    val s1_shouldCheck = RegEnable(s0_shouldCheck, req.valid)
    val s1_addrMatch = RegEnable(s0_addrMatch, req.valid)
    val llv_mask = s1_shouldCheck.asUInt & s1_addrMatch.asUInt
    llv_mask.suggestName("ldldViolationMask_" + i)

    //s3 load load violation resp
    resp.valid := RegNext(req.valid, false.B)
    resp.bits.have_violation := llv_mask.orR

    dontTouch(s0_shouldCheck)
    dontTouch(s0_addrMatch)
    dontTouch(s1_shouldCheck)
    dontTouch(s1_addrMatch)
  })

  when(release1cycle.valid){
    releaseDataModule.io.release_query.req_data := ReleasePAddrHash(release1cycle.bits.paddr, release1cycle.bits.wayIdx)
  }

  private val release_flag = WireInit(VecInit((0 until LoadQueueSize).map({ i =>
    RegNext(releaseDataModule.io.release_query.resp_mask(i) &&
      allocated(i) &&
      dataFromDCache(i) &&
      release1cycle.valid)
  })).asUInt)
  dontTouch(release_flag)

  (0 until LoadQueueSize).map(i => {
    when(release_flag(i)){
      released(i) := true.B
    }
  })

  // Read vaddr for mem exception
  exceptionGen.io.redirect := io.brqRedirect
  exceptionGen.io.in.zipWithIndex.foreach({ case (d, i) =>
    val validCond = io.loadExcepWbInfo(i).valid && !io.loadExcepWbInfo(i).bits.uop.robIdx.needFlush(io.brqRedirect)
    val writebackCond = !io.loadExcepWbInfo(i).bits.miss && !io.loadExcepWbInfo(i).bits.mmio
    val ffIgnoreCond = io.loadExcepWbInfo(i).bits.uop.vctrl.ff && io.loadExcepWbInfo(i).bits.uop.segIdx =/= 0.U && writebackCond
    d.bits.robIdx := RegEnable(io.loadExcepWbInfo(i).bits.uop.robIdx, validCond)
    d.bits.vaddr := RegEnable(io.loadExcepWbInfo(i).bits.vaddr, validCond)
    d.bits.eVec := RegEnable(io.loadExcepWbInfo(i).bits.uop.cf.exceptionVec, validCond)
    d.bits.uopIdx := RegEnable(io.loadExcepWbInfo(i).bits.uop.uopIdx, validCond)
    d.valid := RegNext(validCond & !ffIgnoreCond, false.B)
  })
  private val mmioEvec = Wire(ExceptionVec())
  mmioEvec.foreach(_ := false.B)
  mmioEvec(loadAccessFault) := true.B
  exceptionGen.io.mmioUpdate.valid := io.uncache.resp.fire && io.uncache.resp.bits.error
  exceptionGen.io.mmioUpdate.bits.eVec := mmioEvec
  exceptionGen.io.mmioUpdate.bits.robIdx := io.robHead
  exceptionGen.io.mmioUpdate.bits.vaddr := replayQueue.io.mmioPaddr
  exceptionGen.io.mmioUpdate.bits.uopIdx := uop(deqPtr).uopIdx

//  private val ffCleanConds = io.ldout.map(lo => {
//    val wbCond = lo.fire && exceptionInfo.valid
//    val excptHitCond = lo.bits.uop.uopIdx === exceptionInfo.bits.uopIdx && lo.bits.uop.robIdx === exceptionInfo.bits.robIdx
//    val ffIgnoreCond = lo.bits.uop.vctrl.ff && lo.bits.uop.segIdx =/= 0.U
//    wbCond && excptHitCond && ffIgnoreCond
//  })
//  exceptionGen.io.clean := ffCleanConds.reduce(_ || _)
  exceptionGen.io.clean := false.B  //todo
  io.exceptionAddr.valid := exceptionInfo.valid
  io.exceptionAddr.bits.vaddr := exceptionInfo.bits.vaddr
  io.exceptionAddr.bits.isStore := DontCare

  (0 until LoadPipelineWidth).foreach(i => {
    io.trigger(i).lqLoadAddrTriggerHitVec := DontCare //todo
  })

  // misprediction recovery / exception redirect
  // invalidate lq term using robIdx
  private val needCancel = Wire(Vec(LoadQueueSize, Bool()))
  for (i <- 0 until LoadQueueSize) {
    needCancel(i) := uop(i).robIdx.needFlush(io.brqRedirect) && allocated(i)
    when (needCancel(i)) {
      allocated(i) := false.B
    }
  }

  private val splitNum = 5
  require(LoadQueueSize % splitNum == 0)
  private val needCancelCntSub = Seq.fill(splitNum)(RegInit(0.U(log2Up(LoadQueueSize).W)))

  (0 until splitNum).foreach({ case i => {
    needCancelCntSub(i) := PopCount(needCancel.asUInt((i + 1) * (LoadQueueSize / splitNum) - 1, i * (LoadQueueSize / splitNum)))
  }})
  private val needCancelCnt = needCancelCntSub.reduce(_ + _)
  private val lastCycleCancelCount = needCancelCnt


  /**
    * update pointers
    */
  private val lastEnqCancel = PopCount(RegNext(VecInit(canEnqueue.zip(enqCancel).map(x => x._1 && x._2))))
  private val enqNumber = Mux(io.enq.canAccept && io.enq.sqCanAccept, PopCount(io.enq.req.map(_.valid)), 0.U)
  private val enqNumber_enq = Mux(io.enq.canAccept && io.enq.sqCanAccept, io.enq.reqNum, 0.U)

  XSError(enqNumber =/= enqNumber_enq,"enqNumber and enqNumber_enq must be equal\n")

  when (lastCycleRedirect.valid) {
    // we recover the pointers in the next cycle after redirect
    enqPtrExt := VecInit(enqPtrExt.map(_ - (lastCycleCancelCount + lastEnqCancel)))
  }.otherwise {
    enqPtrExt := VecInit(enqPtrExt.map(_ + enqNumber_enq))
  }

  deqPtrExtNext := deqPtrExt + deqNum
  deqPtrExt := deqPtrExtNext

  io.lqCancelCnt := RegNext(lastCycleCancelCount + lastEnqCancel)

  /**
    * misc
    */
  // perf counter
  QueuePerf(LoadQueueSize, validCount, !allowEnqueue)
  io.lqFull := !allowEnqueue
  XSPerfAccumulate("rollback", io.rollback.valid) // rollback redirect generated
//  XSPerfAccumulate("mmioCycle", uncache_Order_State =/= s_idle) // lq is busy dealing with uncache req
  XSPerfAccumulate("mmioCnt", io.uncache.req.fire)

  val perfValidCount = RegNext(validCount)

  val perfEvents = Seq(
    ("rollback         ", io.rollback.valid),
    ("mmio_Cnt         ", io.uncache.req.fire),
    ("ltq_1_4_valid    ", (perfValidCount < (LoadQueueSize.U/4.U))),
    ("ltq_2_4_valid    ", (perfValidCount > (LoadQueueSize.U/4.U)) & (perfValidCount <= (LoadQueueSize.U/2.U))),
    ("ltq_3_4_valid    ", (perfValidCount > (LoadQueueSize.U/2.U)) & (perfValidCount <= (LoadQueueSize.U*3.U/4.U))),
    ("ltq_4_4_valid    ", (perfValidCount > (LoadQueueSize.U*3.U/4.U)))
  )
  generatePerfEvent()

  // debug info
  XSDebug("enqPtrExt %d:%d deqPtrExt %d:%d\n", enqPtrExt(0).flag, enqPtr, deqPtrExt.flag, deqPtr)

  def PrintFlag(flag: Bool, name: String): Unit = {
    when(flag) {
      XSDebug(false, true.B, name)
    }.otherwise {
      XSDebug(false, true.B, " ")
    }
  }

  for (i <- 0 until LoadQueueSize) {
    XSDebug(i + " pc %x pa %x ", uop(i).cf.pc, debug_paddr(i))
    PrintFlag(allocated(i), "a")
    PrintFlag(allocated(i) && writebacked(i), "w")
    XSDebug(false, true.B, "\n")
  }

}
