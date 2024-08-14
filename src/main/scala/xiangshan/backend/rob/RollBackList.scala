package xiangshan.backend.rob

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan._
import xs.utils._

class RollBackListBufferPtr(size: Int) extends CircularQueuePtr[RollBackListBufferPtr](size) {
  def this()(implicit p: Parameters) = this(p(XSCoreParamsKey).RblSize)
}

object RollBackListBufferPtr {
  def apply(flag: Boolean = false, v: Int = 0)(implicit p: Parameters): RollBackListBufferPtr = {
    val ptr = Wire(new RollBackListBufferPtr(p(XSCoreParamsKey).RblSize))
    ptr.flag := flag.B
    ptr.value := v.U
    ptr
  }
}

class RollBackListCommitInfo(implicit p: Parameters) extends XSBundle {
  val ldest = UInt(5.W)
  val pdest = UInt(PhyRegIdxWidth.W)
  val rfWen = Bool()
  val fpWen = Bool()
  val vecWen = Bool()

  def connectEntryData(data: MicroOp) = {
    ldest := data.ctrl.ldest
    rfWen := data.ctrl.rfWen
    fpWen := data.ctrl.fpWen
    vecWen := data.ctrl.vdWen
    pdest := data.pdest
  }
}

class RollBackListCommitIO(implicit p: Parameters) extends XSBundle {
  val isCommit = Bool()
  val commitValid = Vec(CommitWidth, Bool())

  val isWalk = Bool()
  // valid bits optimized for walk
  val walkValid = Vec(CommitWidth, Bool())

  val info = Vec(CommitWidth, new RollBackListCommitInfo)

  def hasWalkInstr: Bool = isWalk && walkValid.asUInt.orR
  def hasCommitInstr: Bool = isCommit && commitValid.asUInt.orR
}

class RollBackListBufferEntry(implicit p: Parameters) extends XSBundle {
  val info = new RollBackListCommitInfo
}

class RollBackList(size: Int)(implicit p: Parameters) extends XSModule with HasCircularQueuePtrHelper {
  val io = IO(new Bundle {
    val enq = new RblEnqIO
    val fromRob = new RobToRblIO
    val commits = Output(new RollBackListCommitIO)
    val walkEnd = Output(Bool())
    val redirect = Input(Bool())
  })

  // pointers define
  private val enqPtrVec = Wire(Vec(RenameWidth, new RollBackListBufferPtr))
  private val enqPtr = enqPtrVec.head
  private val enqPtrVecNext = Wire(enqPtrVec.cloneType)

  private val deqPtrVec = Wire(Vec(CommitWidth, new RollBackListBufferPtr))
  private val deqPtr = deqPtrVec(0)
  private val deqPtrOH = RegInit(1.U(size.W))
  private val deqPtrOHShift = CircularShift(deqPtrOH)
  private val deqPtrOHVec = VecInit.tabulate(CommitWidth + 1)(deqPtrOHShift.left)
  private val deqPtrVecNext = Wire(deqPtrVec.cloneType)

  private val walkPtrVec = Reg(Vec(CommitWidth, new RollBackListBufferPtr))
  private val walkPtr = walkPtrVec(0)
  private val walkPtrOH = walkPtr.toOH
  private val walkPtrOHVec = VecInit.tabulate(CommitWidth + 1)(CircularShift(walkPtrOH).left)
  private val walkPtrNext = Wire(new RollBackListBufferPtr)

  private val isEmpty = (enqPtr === deqPtr)

  // Regs
  private val rblBuffer = Reg(Vec(size, new RollBackListBufferEntry))
  private val rblBufferEntries = VecInit((0 until size) map (i => rblBuffer(i)))

  private val s_idle :: s_walk :: Nil = Enum(2)
  private val state = RegInit(s_idle)
  private val stateNext = WireInit(state) // otherwise keep state value
  private val commitSize = RegInit(0.U(log2Up(size).W))
  private val walkSize = RegInit(0.U(log2Up(size).W))

  //enq
  private val realNeedEnq = io.enq.req.map(req => req.valid && req.bits.ctrl.needWriteRf)
  private val enqCount    = PopCount(realNeedEnq)
  private val allocatePtrVec = VecInit((0 until RenameWidth).map(i => enqPtrVec(PopCount(realNeedEnq.take(i))).value))
  allocatePtrVec.zip(io.enq.req).zip(realNeedEnq).map { case ((allocatePtr, req), realNeedEnq) =>
    when(realNeedEnq) {
      rblBuffer(allocatePtr).info.connectEntryData(req.bits)
    }
  }
  private val numValidEntries = distanceBetween(enqPtr, deqPtr)
  private val allowEnqueue = RegNext(numValidEntries + enqCount <= (size - RenameWidth).U, true.B)
  // enq ready
  io.enq.canAccept := allowEnqueue && state === s_idle
  io.enq.isEmpty := isEmpty

  //deq
  private val realcommitSize = Mux(io.redirect, 0.U, io.fromRob.commitSize)
  private val realwalkSize = Mux(io.redirect, 0.U, io.fromRob.walkSize)

  private val commitCandidates = VecInit(deqPtrOHVec.map(addr => Mux1H(addr, rblBufferEntries)))
  private val walkCandidates = VecInit(walkPtrOHVec.map(addr => Mux1H(addr, rblBufferEntries)))

  for (i <- 0 until CommitWidth) {
    io.commits.commitValid(i) := state === s_idle && i.U < realcommitSize
    io.commits.walkValid(i) := state === s_walk && i.U < realwalkSize
    // special walk use commitPtr
    io.commits.info(i) := Mux(state === s_idle, commitCandidates(i).info, walkCandidates(i).info)
  }

  // change state
  private val commitNum = Wire(UInt(3.W))
  private val walkNum = Wire(UInt(3.W))
  commitNum := Mux(io.commits.commitValid(0), PriorityMux((0 until 6).map(
    i => io.commits.commitValid(5 - i) -> (6 - i).U
  )), 0.U)
  walkNum := Mux(io.commits.walkValid(0), PriorityMux((0 until 6).map(
    i => io.commits.walkValid(5 - i) -> (6 - i).U
  )), 0.U)
  private val commitdeqCount = Mux(io.commits.isCommit && !io.commits.isWalk, commitNum, 0.U)
  private val walkdeqCount = Mux(io.commits.isWalk && !io.commits.isCommit, walkNum, 0.U)
  private val commitSizeNxt = commitSize + realcommitSize - commitdeqCount
  private val walkSizeNxt = walkSize + realwalkSize - walkdeqCount
  private val walkEndNext = walkSizeNxt === 0.U
  commitSize := Mux(io.redirect, 0.U, commitSizeNxt)
  walkSize := Mux(io.redirect, 0.U, walkSizeNxt)
  state := stateNext
  when(io.redirect) {
    stateNext := s_walk
  }.otherwise {
    // change stateNext
    switch(state) {
      // this transaction is not used actually, just list all states
      is(s_idle) {
        stateNext := s_idle
      }
      is(s_walk) {
        when(walkEndNext) {
          stateNext := s_idle
        }
      }
    }
  }
  // update enq pointer
  val enqPtrNext = Mux(
    state === s_walk && stateNext === s_idle,
    walkPtrNext,
    enqPtr + enqCount
  )
  enqPtr := enqPtrNext
  enqPtrVecNext.zipWithIndex.map { case (ptr, i) => ptr := enqPtrNext + i.U }
  enqPtrVec := enqPtrVecNext

  val deqPtrSteps = Mux1H(Seq(
    (state === s_idle) -> commitdeqCount,
  ))

  // update deq pointer
  val deqPtrNext = deqPtr + deqPtrSteps
  val deqPtrOHNext = deqPtrOHVec(deqPtrSteps)
  deqPtr := deqPtrNext
  deqPtrOH := deqPtrOHNext
  deqPtrVecNext.zipWithIndex.map { case (ptr, i) => ptr := deqPtrNext + i.U }
  deqPtrVec := deqPtrVecNext
}