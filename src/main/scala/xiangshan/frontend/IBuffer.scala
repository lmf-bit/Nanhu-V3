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

package xiangshan.frontend

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan._
import utils._
import xs.utils._
import xiangshan.ExceptionNO._
import xs.utils.perf.HasPerfLogging

class IbufPtr(implicit p: Parameters) extends CircularQueuePtr[IbufPtr](
  p => p(XSCoreParamsKey).IBufSize
){
}

class IBufferIO(implicit p: Parameters) extends XSBundle {
  val flush = Input(Bool())
  val in = Flipped(DecoupledIO(new FetchToIBuffer))
  val out = Vec(DecodeWidth, DecoupledIO(new CtrlFlow))
  val full = Output(Bool())
}

class IBufEntry(implicit p: Parameters) extends XSBundle {
  val inst = UInt(32.W)
  val pc = UInt(VAddrBits.W)
  val foldpc = UInt(MemPredPCWidth.W)
  val pd = new PreDecodeInfo
  val pred_taken = Bool()
  val ftqPtr = new FtqPtr
  val ftqOffset = UInt(log2Ceil(PredictWidth).W)
  val ipf = Bool()
  val acf = Bool()
  val crossPageIPFFix = Bool()
  val triggered = new TriggerCf
  val fdiUntrusted = Bool()
  val fetchTime = UInt(XLEN.W)

  def fromFetch(fetch: FetchToIBuffer, i: Int): IBufEntry = {
    inst   := fetch.instrs(i)
    pc     := fetch.pc(i)
    foldpc := fetch.foldpc(i)
    pd     := fetch.pd(i)
    pred_taken := fetch.ftqOffset(i).valid
    ftqPtr := fetch.ftqPtr
    ftqOffset := fetch.ftqOffset(i).bits
    ipf := fetch.ipf(i)
    acf := fetch.acf(i)
    crossPageIPFFix := fetch.crossPageIPFFix(i)
    triggered := fetch.triggered(i)
    fdiUntrusted := fetch.fdiUntrusted(i)
    val time = GTimer()
    fetchTime := time
    this
  }

  def toCtrlFlow: CtrlFlow = {
    val cf = Wire(new CtrlFlow)
    cf.instr := inst
    cf.pc := pc
    cf.foldpc := foldpc
    cf.exceptionVec := 0.U.asTypeOf(ExceptionVec())
    cf.exceptionVec(instrPageFault) := ipf
    cf.exceptionVec(instrAccessFault) := acf
    cf.trigger := triggered
    cf.pd := pd
    cf.pred_taken := pred_taken
    cf.crossPageIPFFix := crossPageIPFFix
    cf.storeSetHit := DontCare
    cf.waitForRobIdx := DontCare
    cf.loadWaitBit := DontCare
    cf.loadWaitStrict := DontCare
    cf.ssid := DontCare
    cf.ftqPtr := ftqPtr
    cf.ftqOffset := ftqOffset
    cf.fdiUntrusted := fdiUntrusted
    cf.predebugInfo.fetchTime := fetchTime
    cf.predebugInfo.decodeTime := DontCare
    cf
  }
}

class IBuffer(implicit p: Parameters) extends XSModule with HasCircularQueuePtrHelper with HasPerfEvents with HasPerfLogging {
  val io = IO(new IBufferIO)

  private val ibuf: Vec[IBufEntry] = RegInit(VecInit.fill(IBufSize)(0.U.asTypeOf(new IBufEntry)))

  val enqPtrVec = RegInit(VecInit.tabulate(PredictWidth)(_.U.asTypeOf(new IbufPtr)))
  val deqPtrVec = RegInit(VecInit.tabulate(DecodeWidth)(_.U.asTypeOf(new IbufPtr)))
  val deqPtrNextVec = Wire(Vec(DecodeWidth, new IbufPtr))
  val enqPtr    = enqPtrVec(0)
  val deqPtr    = deqPtrVec(0)
  deqPtrVec := deqPtrNextVec
  
  val bypassEntries = WireDefault(VecInit.fill(DecodeWidth)(0.U.asTypeOf(Valid(new IBufEntry))))
  val deqEntries    = WireDefault(VecInit.fill(DecodeWidth)(0.U.asTypeOf(Valid(new IBufEntry))))
  val outEntries    = RegInit(VecInit.fill(DecodeWidth)(0.U.asTypeOf(Valid(new IBufEntry))))

  // bypass
  val backendCanAcc = io.out.head.ready
  val useBypassOut    = RegInit(false.B)
  val useBypassRemain = RegInit(0.U)
  val useBypassRemainNext = Wire(UInt())
  val isEnqBypass = (enqPtr === deqPtr) && backendCanAcc && io.in.fire &&
                    (useBypassRemain === 0.U || (useBypassOut && useBypassRemainNext === 0.U))
  val enqOffsetVec = VecInit.tabulate(PredictWidth)(i => PopCount(io.in.bits.valid.asBools.take(i)))
  val enqDataVec   = VecInit.tabulate(PredictWidth)(i => Wire(new IBufEntry).fromFetch(io.in.bits, i))
  val numBypass = PopCount(bypassEntries.map(_.valid))
  val numOut = PriorityMuxDefault(io.out.map(x => !x.ready) zip (0 until DecodeWidth).map(_.U), DecodeWidth.U)
  bypassEntries.zipWithIndex.foreach {
    case(entry, idx) =>
      val validOH = Range(0, PredictWidth).map {
        i => io.in.bits.valid(i) && io.in.bits.enqEnable(i) && enqOffsetVec(i) === idx.asUInt
      }
      entry.valid := io.in.fire && !io.flush && validOH.reduce(_||_)
      entry.bits  := Mux1H(validOH, enqDataVec)
  }
  
  // enq
  val numIn = Mux(io.in.fire, PopCount(io.in.bits.enqEnable), 0.U)
  val numEnq = WireDefault(0.U)
  val numDeq = WireDefault(0.U)
  val allowEnq = RegInit(true.B)
  val numValid = distanceBetween(enqPtr, deqPtr)
  val numValidReg = Reg(UInt())
  numValidReg := numValid
  val numValidNext = Mux(backendCanAcc, numValid + numEnq - numDeq, numValid + numEnq) 
  when(io.in.fire) {
    when(isEnqBypass) {
      numEnq := Mux(numIn > DecodeWidth.U, numIn - DecodeWidth.U, 0.U)
    }.otherwise {
      numEnq := numIn
    }
  }
  val numFire = PriorityMuxDefault(io.out.map(x => !x.fire) zip (0 until DecodeWidth).map(_.U), DecodeWidth.U)
  when(backendCanAcc) {
    when(isEnqBypass) {
      numDeq := Mux(numOut > numBypass,
        Mux((numOut - numBypass) >= numValid, numValid, numOut - numBypass) , 0.U)
    }.elsewhen(useBypassOut) {
      numDeq := 0.U
    }.otherwise {
      numDeq := numFire
    }
  }
  allowEnq := (IBufSize.U - numValidNext) >= PredictWidth.U
  io.in.ready := allowEnq
  ibuf.zipWithIndex.foreach {
    case(entry, idx) => {
      val validOH = Range(0, PredictWidth).map {
        i =>
          val bypassMask = (enqOffsetVec(i) >= DecodeWidth.U) &&
            enqPtrVec(enqOffsetVec(i) - DecodeWidth.U).value === idx.asUInt
          val normalMask = enqPtrVec(enqOffsetVec(i)).value === idx.asUInt
          val mask = Mux(isEnqBypass, bypassMask, normalMask)

          io.in.bits.valid(i) && io.in.bits.enqEnable(i) && mask
      }
      val wen = io.in.fire && !io.flush && validOH.reduce(_||_)
      val writeEntry = Mux1H(validOH, enqDataVec)
      entry := Mux(wen, writeEntry, entry)
    }
  }
  
  when(io.in.fire && !io.flush) {
    enqPtrVec := VecInit(enqPtrVec.map(_ + numEnq))
  }

  // deq
  val numValidAfterDeq = numValid - numDeq
  val deqValidVec = Mux(numValidAfterDeq >= DecodeWidth.U, ((1 << DecodeWidth) - 1).U,
    UIntToMask(numValidAfterDeq(log2Ceil(DecodeWidth) - 1, 0), DecodeWidth))
  for(a <-  0 until DecodeWidth) {
    deqEntries(a).valid := deqValidVec(a)
    deqEntries(a).bits  := ibuf(deqPtrNextVec(a).value)
  }
  when(io.out.head.ready && !io.flush) {
    deqPtrNextVec := VecInit(deqPtrVec.map(_ + numDeq))
  }.otherwise {
    deqPtrNextVec := deqPtrVec
  }

  // out
  useBypassRemainNext := Mux(useBypassRemain > numFire, useBypassRemain - numFire, 0.U)
  when(backendCanAcc && isEnqBypass) {
    useBypassOut    := true.B
    useBypassRemain := numBypass
  }.elsewhen(useBypassOut && useBypassRemain =/= 0.U) {
    useBypassOut    := Mux(useBypassRemain > numFire, true.B, false.B)
    useBypassRemain := useBypassRemainNext
  }.otherwise {
    useBypassOut    := false.B
    useBypassRemain := 0.U
  }
  
  (outEntries zip bypassEntries zip deqEntries).zipWithIndex.foreach {
    case(((out, bypass), deq), idx) => {
      when(backendCanAcc) {
        when(isEnqBypass) {
          out := bypass
        }.elsewhen(useBypassOut && useBypassRemainNext =/= 0.U) {
          out := Mux(idx.U < useBypassRemainNext, outEntries(numFire + idx.U), 0.U.asTypeOf(out)) 
        }.otherwise {
          out := deq
        }
      }
    }
  }
  io.out zip outEntries foreach {
    case(out, reg) => {
      out.valid := reg.valid
      out.bits  := reg.bits.toCtrlFlow
    }
  }

  // flush
  when(io.flush) {
    allowEnq := true.B
    deqPtrVec := deqPtrVec.indices.map(_.U.asTypeOf(new IbufPtr))
    enqPtrVec := enqPtrVec.indices.map(_.U.asTypeOf(new IbufPtr))
    outEntries.foreach(_.valid := false.B)
    useBypassOut    := false.B 
    useBypassRemain := 0.U
  }
  io.full := !allowEnq

  //perf
  XSDebug(io.flush, "IBuffer Flushed\n")
  when(io.in.fire) {
    XSDebug("Enque:\n")
    XSDebug(p"MASK=${Binary(io.in.bits.valid)}\n")
    for(i <- 0 until PredictWidth){
      XSDebug(p"PC=${Hexadecimal(io.in.bits.pc(i))} ${Hexadecimal(io.in.bits.instrs(i))}\n")
    }
  }
  for (i <- 0 until DecodeWidth) {
    XSDebug(io.out(i).fire,
      p"deq: ${Hexadecimal(io.out(i).bits.instr)} PC=${Hexadecimal(io.out(i).bits.pc)}" +
      p"v=${io.out(i).valid} r=${io.out(i).ready} " +
      p"excpVec=${Binary(io.out(i).bits.exceptionVec.asUInt)} crossPageIPF=${io.out(i).bits.crossPageIPFFix}\n")
  }

  XSDebug(p"ValidEntries: ${numValid}\n")
  XSDebug(p"EnqNum: ${numEnq}\n")
  XSDebug(p"DeqNum: ${numDeq}\n")

  val afterInit  = RegInit(false.B)
  val headBubble = RegInit(false.B)
  when (io.in.fire) { afterInit := true.B }
  when (io.flush) {
    headBubble := true.B
  } .elsewhen(numValid =/= 0.U) {
    headBubble := false.B
  }
  val instrHungry = afterInit && (numValid === 0.U) && !headBubble

  QueuePerf(IBufSize, numValid, !allowEnq)
  XSPerfAccumulate("flush", io.flush)
  XSPerfAccumulate("hungry", instrHungry)
  if (env.EnableTopDown) {
    val ibuffer_IDWidth_hvButNotFull = afterInit && (numValid =/= 0.U) && (numValid < DecodeWidth.U) && !headBubble
    XSPerfAccumulate("ibuffer_IDWidth_hvButNotFull", ibuffer_IDWidth_hvButNotFull)
  }

  val perfEvents = Seq(
    ("IBuffer_Flushed  ", io.flush                                                                     ),
    ("IBuffer_hungry   ", instrHungry                                                                  ),
    ("IBuffer_1_4_valid", (numValid >  (0*(IBufSize/4)).U) & (numValid < (1*(IBufSize/4)).U)   ),
    ("IBuffer_2_4_valid", (numValid >= (1*(IBufSize/4)).U) & (numValid < (2*(IBufSize/4)).U)   ),
    ("IBuffer_3_4_valid", (numValid >= (2*(IBufSize/4)).U) & (numValid < (3*(IBufSize/4)).U)   ),
    ("IBuffer_4_4_valid", (numValid >= (3*(IBufSize/4)).U) & (numValid < (4*(IBufSize/4)).U)   ),
    ("IBuffer_full     ",  numValid.andR                                                           ),
    ("Front_Bubble     ", PopCount((0 until DecodeWidth).map(i => io.out(i).ready && !io.out(i).valid)))
  )
  generatePerfEvent()
}

