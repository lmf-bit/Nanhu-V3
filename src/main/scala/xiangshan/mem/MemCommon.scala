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
import chisel3.experimental.prefix
import chisel3.util._
import xiangshan._
import xs.utils._
import xiangshan.backend.issue.RsIdx
import xiangshan.cache._
import xs.utils.{CircularQueuePtr, LookupTree, UIntToMask}
import xiangshan.mem.prefetch.PrefetchReqBundle
import xiangshan.mem.HasL1PrefetchSourceParameter

object genWmask {
  def apply(addr: UInt, sizeEncode: UInt): UInt = {
    (LookupTree(sizeEncode, List(
      "b00".U -> 0x1.U, //0001 << addr(2:0)
      "b01".U -> 0x3.U, //0011
      "b10".U -> 0xf.U, //1111
      "b11".U -> 0xff.U //11111111
    )) << addr(2, 0)).asUInt
  }
}

object GatedValidRegNext {
  // 3 is the default minimal width of EDA inserted clock gating cells.
  // so using `GatedValidRegNext` to signals whoes width is less than 3 may not help.

  // It is useless to clockgate only one bit, so change to RegNext here
  def apply(next: Bool, init: Bool = false.B): Bool = {
    val last = WireInit(false.B)
    last := RegNext(next, init)
    last
  }

  def apply(last: Vec[Bool]): Vec[Bool] = {
    val next = VecInit(Seq.fill(last.size)(false.B))
    next := RegEnable(last, VecInit(Seq.fill(last.size)(false.B)), last.asUInt =/= next.asUInt)
    next
  }
}

object genWdata {
  def apply(data: UInt, sizeEncode: UInt): UInt = {
    LookupTree(sizeEncode, List(
      "b00".U -> Fill(8, data(7, 0)),
      "b01".U -> Fill(4, data(15, 0)),
      "b10".U -> Fill(2, data(31, 0)),
      "b11".U -> data
    ))
  }
}

object getFirstOne{
  def apply(mask: Vec[Bool], startMask: UInt): UInt = {
    val length = mask.length
    val highBits = (0 until length).map(i => mask(i) & (~startMask(i)).asBool)
    val highBitsUint = Cat(highBits.reverse)
    PriorityEncoder(Mux(highBitsUint.orR, highBitsUint, mask.asUInt))
  }
}

object getOldestInTwo {
  def apply(valid: Seq[Bool], uop: Seq[MicroOp]): MicroOp = {
    assert(valid.length == uop.length)
    assert(valid.length == 2)
    Mux(valid(0) && valid(1),
      Mux(uop(0).robIdx > uop(1).robIdx, uop(1), uop(0)),
      Mux(valid(0) && !valid(1), uop(0), uop(1)))
  }
}

object getAfterMask{
  def apply(valid: Seq[Bool], uop: Seq[MicroOp]): IndexedSeq[IndexedSeq[Bool]] = {
    assert(valid.length == uop.length)
    val length = valid.length
    (0 until length).map(i => {
      (0 until length).map(j => {
        Mux(valid(i) && valid(j),
          uop(i).robIdx > uop(j).robIdx,
          Mux(!valid(i), true.B, false.B))
      })
    })
  }
}





class LsPipelineBundle(implicit p: Parameters) extends XSBundle with HasDCacheParameters{
  val vaddr = UInt(VAddrBits.W)
  val paddr = UInt(PAddrBits.W)
  // val func = UInt(6.W)
  val mask = UInt(8.W)
  val data = UInt((XLEN+1).W)
  val uop = new MicroOp
  val wlineflag = Bool() // store write the whole cache line

  val miss = Bool()
  val tlbMiss = Bool()
  val ptwBack = Bool()
  val mmio = Bool()
  val rsIdx = new RsIdx

  val forwardMask = Vec(8, Bool())
  val forwardData = Vec(8, UInt(8.W))

  //rs has feedback
  val rsHasFeedback = Bool()

  //softprefetch
  val isSoftPrefetch = Bool()

  // replayInfo
  val replay = new ReplayInfo
  // debug
  val debugCause = UInt(LoadReplayCauses.allCauses.W)
  // For debug usage
  val isFirstIssue = Bool()

  def asPrefetchReqBundle(): PrefetchReqBundle = {
    val res = Wire(new PrefetchReqBundle)
    res.vaddr := this.vaddr
    res.paddr := this.paddr
    res.pc    := this.uop.cf.pc
    res.miss  := this.miss
    res
  }
}

class LqWriteBundle(implicit p: Parameters) extends XSBundle {
  // queue entry data, except flag bits, will be updated if writeQueue is true,
  // valid bit in LqWriteBundle will be ignored
  val has_writeback = Bool()
  val miss = Bool()
  val mmio = Bool()
  val uop = new MicroOp
  val vaddr = UInt(VAddrBits.W)

  def fromLsPipelineBundle(input: LsPipelineBundle) : Unit = {
    uop := input.uop
    miss := input.miss
    mmio := input.mmio
    vaddr := input.vaddr
  }
}


class LoadPipelineBundleS0(implicit p: Parameters) extends XSBundle {
  //EXUInput
  val uop = new MicroOp
  val vm = UInt(VLEN.W)

  //replayQ
  val isReplayQReplay = Bool()
  val replayCause = Vec(LoadReplayCauses.allCauses, Bool())
  val schedIndex = UInt(log2Up(LoadReplayQueueSize).W)
  val vaddr = UInt(VAddrBits.W)

  //Rs
  val rsIdx = new RsIdx

  // debug
  val debugCause = UInt(LoadReplayCauses.allCauses.W)
}

class LoadForwardQueryIO(implicit p: Parameters) extends XSBundle {
  val vaddr = Output(UInt(VAddrBits.W))
  val paddr = Output(UInt(PAddrBits.W))
  val mask = Output(UInt(8.W))
  val uop = Output(new MicroOp) // for replay
  val pc = Output(UInt(VAddrBits.W)) //for debug
  val valid = Output(Bool())

//  val forwardMaskFast = Input(Vec(8, Bool()))
  val forwardMask = Input(Vec(8, Bool())) // resp to load_s2
  val forwardData = Input(Vec(8, UInt(8.W))) // resp to load_s2

  val sqIdx = Output(new SqPtr)

  // dataInvalid suggests store to load forward found forward should happen,
  // but data is not available for now. If dataInvalid, load inst should
  // be replayed from RS. Feedback type should be RSFeedbackType.dataInvalid
  val dataInvalid = Input(Bool()) // Addr match, but data is not valid for now

  // matchInvalid suggests in store to load forward logic, paddr cam result does
  // to equal to vaddr cam result. If matchInvalid, a microarchitectural exception
  // should be raised to flush SQ and committed sbuffer.
  val matchInvalid = Input(Bool()) // resp to load_s2
}

// PipeLoadForwardFromSQ is used in loadUnit stage_1 to require StoreQueue to forward data
class PipeLoadForwardFromSQ(implicit p: Parameters) extends LoadForwardQueryIO {
  val sqIdxMask = Output(UInt(StoreQueueSize.W))
  val dataInvalidSqIdx = Input(new SqPtr) // resp to load_s2, sqIdx
}

// Query load queue for ld-ld violation
// 
// Req should be send in load_s1
// Resp will be generated 1 cycle later
//
// Note that query req may be !ready, as dcache is releasing a block
// If it happens, a replay from rs is needed.

// class LoadViolationQueryReq(implicit p: Parameters) extends XSBundle {
//   val paddr = UInt(PAddrBits.W)
//   val uop = new MicroOp // provide lqIdx
// }

class LoadViolationQueryResp(implicit p: Parameters) extends XSBundle {
  val have_violation = Bool()
}

class LoadViolationQueryIO(implicit p: Parameters) extends XSBundle {
  val s3_resp = Flipped(Valid(new LoadViolationQueryResp))
}

// Store byte valid mask write bundle
//
// Store byte valid mask write to SQ takes 2 cycles
class StoreMaskBundle(implicit p: Parameters) extends XSBundle {
  val sqIdx = new SqPtr
  val mask = UInt(8.W)
}

class LoadDataFromDcacheBundle(implicit p: Parameters) extends DCacheBundle {
  val load_data = UInt(64.W)

  val forwardMask = Vec(8, Bool())
  val forwardData = Vec(8, UInt(8.W))
  val uop = new MicroOp // for data selection, only fwen and fuOpType are used
  val addrOffset = UInt(3.W) // for data selection

  def dcacheData(): UInt = {
    load_data
  }

  def mergedData(): UInt = {
    val rdataVec = VecInit((0 until XLEN / 8).map(j =>
      Mux(forwardMask(j), forwardData(j), dcacheData()(8*(j+1)-1, 8*j))
    ))
    rdataVec.asUInt
  }
}

// Load writeback data from load queue (refill)
class LoadDataFromLQBundle(implicit p: Parameters) extends XSBundle {
  val lqData = UInt(64.W) // load queue has merged data
  val uop = new MicroOp // for data selection, only fwen and fuOpType are used
  val addrOffset = UInt(3.W) // for data selection

  def mergedData(): UInt = {
    lqData
  }
}

object RedirectRegDup{
  def apply(name: List[String], redirect: ValidIO[Redirect]): Map[String, ValidIO[Redirect]] = {
    val len = name.length
    var map: Map[String, ValidIO[Redirect]] = Map()

    for (i <- 0 until len) {
      map += (name(i) -> prefix("dup_" + name(i)){Pipe(redirect)})
    }
    map
  }
}

object AddPipelineReg {
  class PipelineRegModule[T <: Data](gen: T) extends Module {
    val io = IO(new Bundle() {
      val in = Flipped(DecoupledIO(gen.cloneType))
      val out = DecoupledIO(gen.cloneType)
      val isFlush = Input(Bool())
    })

    val valid = RegInit(false.B)
    valid.suggestName("pipeline_reg_valid")
    when (io.out.fire) { valid := false.B }
    when (io.in.fire) { valid := true.B }
    when (io.isFlush) { valid := false.B }

    io.in.ready := !valid || io.out.ready
    io.out.bits := RegEnable(io.in.bits, io.in.fire)
    io.out.valid := valid //&& !isFlush
  }

  def apply[T <: Data]
  (left: DecoupledIO[T], right: DecoupledIO[T], isFlush: Bool,
   moduleName: Option[String] = None
  ){
    val pipelineReg = Module(new PipelineRegModule[T](left.bits.cloneType))
    if(moduleName.nonEmpty) pipelineReg.suggestName(moduleName.get)
    pipelineReg.io.in <> left
    right <> pipelineReg.io.out
    pipelineReg.io.isFlush := isFlush
  }
}
