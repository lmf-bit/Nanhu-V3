package xiangshan.backend
import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan._
import xiangshan.backend.decode.FusionDecodeInfo
import xs.utils.CircularQueuePtr
import xiangshan.backend.rob.RobPtr
import xs.utils._
import xs.utils.perf.HasPerfLogging

class RouterQueueEntry(implicit p: Parameters) extends XSBundle {
  val uop = new MicroOp
  def initMicroOp: MicroOp = {
    val umop = Wire(new MicroOp)
    umop := uop
    umop.srcState(0) := DontCare
    umop.srcState(1) := DontCare
    umop.srcState(2) := DontCare
    umop.debugInfo := DontCare
    umop.lqIdx := DontCare
    umop.sqIdx := DontCare
    umop.lpv := DontCare
    umop.vCsrInfo := DontCare
    umop.vctrl := DontCare
    umop.vctrl.ordered := false.B
    umop.uopIdx := 0.U
    umop.vmState := DontCare
    umop.isTail := DontCare
    umop.isPrestart := DontCare
    umop.vtypeRegIdx := DontCare
    umop.uopNum := 0.U
    umop.vm := DontCare
    umop.canRename := DontCare
    umop.mergeIdx := DontCare
    umop.loadStoreEnable := DontCare
    umop.segIdx := 0.U
    umop.elmIdx := 0.U
    umop
  }
}

class RouterQueue(vecLen:Int, outNum:Int, size:Int)(implicit p: Parameters) extends XSModule with HasCircularQueuePtrHelper with HasPerfLogging {
  val io = IO(new Bundle {
    val in = Flipped(Vec(vecLen, Decoupled(new CfCtrl)))
    val out = Vec(outNum, Vec(vecLen, Decoupled(new MicroOp)))
    val allowOut = Output(Vec(outNum, Bool()))
    val redirect = Flipped(ValidIO(new Redirect))
    val flush = Input(Bool())
    val robempty = Input(Bool())
    val singleStep = Input(Bool())
  })

  //reg
  private class RouterQueuePtr extends CircularQueuePtr[RouterQueuePtr](size)

  private val dataModule = Reg(Vec(size, new RouterQueueEntry))

  private val enqPtrVec = Seq.tabulate(vecLen)(i => RegInit(i.U.asTypeOf(new RouterQueuePtr)))
  private val enqPtr = enqPtrVec.head
  private val enqPtrVecNext = enqPtrVec.map(WireInit(_))

  private val deqPtrVec = Seq.tabulate(vecLen)(i => RegInit(i.U.asTypeOf(new RouterQueuePtr)))
  private val deqPtr = deqPtrVec.head
  private val deqPtrVecNext = deqPtrVec.map(WireInit(_))

  private val singleStepStatus = RegInit(false.B)

  private val allowOut = Wire(Vec(outNum, Bool()))
  allowOut.zipWithIndex.foreach({ case (a, i) =>
    a := io.out.zipWithIndex.filterNot(_._2 == i).map(_._1.head.ready).reduce(_ & _)
  })

  //enq
  for (i <- 0 until vecLen) {
    when(io.in(i).fire && !(io.redirect.valid || io.flush)) {
      val currentAddr = enqPtr + PopCount(io.in.take(i).map(_.valid))
      dataModule(currentAddr.value).uop.ctrl := io.in(i).bits.ctrl
      dataModule(currentAddr.value).uop.cf := io.in(i).bits.cf
      dataModule(currentAddr.value).uop.vctrl := io.in(i).bits.vctrl
      dataModule(currentAddr.value).uop.vCsrInfo := io.in(i).bits.vCsrInfo
    }
  }

  private val enqCount    = PopCount(io.in.map(_.valid))
  private val numValidEntries = distanceBetween(enqPtr, deqPtr)
  private val numEmptyEntries = size.U - numValidEntries
  private val emptyEntriesNext = Wire(UInt(log2Ceil(size + 1).W))
  private val actualEnqNum = PopCount(io.in.map(_.fire))
  private val actualDeqNum = PopCount(io.out(0).map(_.fire))
  when(io.redirect.valid) {
    emptyEntriesNext := size.U
  }.otherwise {
    emptyEntriesNext := numEmptyEntries +& actualDeqNum - actualEnqNum
  }
  assert(deqPtr <= enqPtr)
  assert(actualEnqNum <= numEmptyEntries)
  private val allowEnqueue = RegNext(numValidEntries + enqCount <= (size - vecLen).U, true.B)
  // enq ready
  private val canAcceptRegs = Seq.fill(vecLen)(RegInit(true.B))
  canAcceptRegs.zipWithIndex.foreach({case (acepReg, i) => acepReg := i.U < emptyEntriesNext})
  io.in.zip(canAcceptRegs).foreach({case (in, acep) => in.ready := acep})

  // singelStepStatus
  private val inst0actualOut = io.out(0)(0).valid
  when(io.redirect.valid) {
    singleStepStatus := false.B
  }.elsewhen(io.singleStep && io.in(0).fire && inst0actualOut) {
    singleStepStatus := true.B
  }

  //deq
  private val deqEntries = WireInit(VecInit.fill(vecLen)(0.U.asTypeOf(new RouterQueueEntry)))
  deqEntries.zip(deqPtrVec).foreach({case (deqdata, addr) =>
    deqdata := dataModule(addr.value)
  })
  val isBlockBackward = VecInit(deqEntries.map(req => req.uop.ctrl.blockBackward))
  val isNoSpecExec = VecInit(deqEntries.map(req => req.uop.ctrl.noSpecExec))
  val thisIsBlocked = VecInit((0 until vecLen).map(i => {
    // for i > 0, when Rob is empty but dispatch1 have valid instructions to enqueue, it's blocked
    if (i > 0) isNoSpecExec(i)
    else isNoSpecExec(i) && !io.robempty
  }))
  dontTouch(thisIsBlocked)
  val nextCanOut = VecInit((0 until vecLen).map(i =>
    (!isNoSpecExec(i) && !isBlockBackward(i))
  ))
  val notBlockedByPrevious = VecInit((0 until vecLen).map(i =>
    if (i == 0) true.B
    else Cat((0 until i).map(j => nextCanOut(j))).andR
  ))
  val thisCanActualOut = (0 until vecLen).map(i => !thisIsBlocked(i) && notBlockedByPrevious(i))
  // dontTouch(thisCanActualOut)
  val updatedCommitType = Wire(Vec(vecLen, CommitType()))
  val updatedUop = Wire(Vec(RenameWidth, new MicroOp))
  io.out.zipWithIndex.foreach({case(out, idx) =>
    out.zipWithIndex.zip(deqEntries).foreach({case ((o, i), deq) =>
      o.valid := thisCanActualOut(i) && (i.U < numValidEntries) && !(io.redirect.valid || io.flush)
      updatedUop(i) := deq.uop
      updatedUop(i).ctrl.singleStep := io.singleStep && (if (i == 0) singleStepStatus else true.B)
      updatedCommitType(i) := Cat(FuType.isLoadStore(deq.uop.ctrl.fuType), (FuType.isStore(deq.uop.ctrl.fuType) && !FuType.isAMO(deq.uop.ctrl.fuType)) | (FuType.isJumpExu(deq.uop.ctrl.fuType) || !deq.uop.cf.pd.notCFI ))
      when(!CommitType.isFused(deq.uop.ctrl.commitType)) {
        updatedUop(i).ctrl.commitType := updatedCommitType(i)
      }.otherwise {
        XSError(o.valid && updatedCommitType(i) =/= CommitType.NORMAL, "why fused?\n")
      }
      o.bits := updatedUop(i)
    })
  })
  io.allowOut := allowOut

  //pointers update
  enqPtrVec.zipWithIndex.foreach({case (ptr, i) =>
    when(io.redirect.valid) {
      ptr := i.U.asTypeOf(new RouterQueuePtr)
    }.elsewhen(io.flush) {
      ptr := i.U.asTypeOf(new RouterQueuePtr)
    }.elsewhen(actualEnqNum =/= 0.U) {
      ptr := ptr + actualEnqNum
    }
  })
  deqPtrVec.zipWithIndex.foreach({ case (ptr, i) =>
    when(io.redirect.valid) {
      ptr := i.U.asTypeOf(new RouterQueuePtr)
    }.elsewhen(io.out(0).map(_.fire).reduce(_ || _)) {
      ptr := ptr + actualDeqNum
    }
  })

  //perf counter
  XSPerfAccumulate("decqueue_in_fire", actualEnqNum)
  XSPerfAccumulate("decqueue_in_valid", PopCount(io.in.map(_.valid)))
  XSPerfAccumulate("waitInstr", PopCount((0 until vecLen).map(i => io.in(i).valid && !io.in(i).ready)))
  XSPerfAccumulate("stall_cycle_out_ready", PopCount((0 until vecLen).map(i => io.out(0)(i).valid && !io.out(0)(i).ready)))
  XSPerfAccumulate("stall_cycle_nospecexec", PopCount((0 until vecLen).map(i => io.out(0)(i).bits.ctrl.noSpecExec)))
  XSPerfAccumulate("stall_cycle_scala", allowOut(1))
  XSPerfAccumulate("stall_cycle_vector", allowOut(0))
  XSPerfAccumulate("flush_time", io.flush)
}