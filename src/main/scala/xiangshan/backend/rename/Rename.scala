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

package xiangshan.backend.rename

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.CSRs
import utils._
import xs.utils.GTimer
import xiangshan._
import xiangshan.backend.decode.{FusionDecodeInfo, Imm_I, Imm_LUI_LOAD, Imm_U}
import xiangshan.backend.rename.freelist._
import xiangshan.backend.execute.fu.jmp.JumpOpType
import xiangshan.backend.execute.fu.FuOutput
import xiangshan.backend.execute.fu.csr.CSROpType
import xiangshan.backend.execute.fu.csr.vcsr.VCSRWithVtypeRenameIO
import xiangshan.backend.rob.{RobEnqIO, RobPtr}
import xiangshan.mem.mdp._
import xiangshan.vector.SIRenameInfo
import xiangshan.vector.vtyperename.{VtpToVCtl, VtypeRename}
import xs.utils.perf.HasPerfLogging
import xs.utils.HasCircularQueuePtrHelper
import xiangshan.ExceptionNO.selectFrontend
import xiangshan.ExceptionNO.illegalInstr

class Rename(implicit p: Parameters) extends XSModule  with HasCircularQueuePtrHelper with HasPerfEvents with HasPerfLogging {
  val io = IO(new Bundle() {
    val redirect = Flipped(ValidIO(new Redirect))
    val robCommits = Flipped(new RobCommitIO)
    val rabCommits = Flipped(new RabCommitIO)
    // from decode
    val in = Vec(RenameWidth, Flipped(DecoupledIO(new MicroOp)))
    val allowIn = Input(Bool())
//    val fusionInfo = Vec(DecodeWidth - 1, Flipped(new FusionDecodeInfo))
    // ssit read result
    val ssit = Flipped(Vec(RenameWidth, Output(new SSITEntry)))
    // waittable read result
    val waittable = Flipped(Vec(RenameWidth, Output(Bool())))
    // RAT read
    val intRat = Vec(RenameWidth, Vec(3, Flipped(new RatReadPort)))
    val fpRat = Vec(RenameWidth, Vec(4, Flipped(new RatReadPort)))
    // to rename table
    val intReadPorts  = Vec(RenameWidth, Vec(3, Input(UInt(PhyRegIdxWidth.W))))
    val fpReadPorts   = Vec(RenameWidth, Vec(4, Input(UInt(PhyRegIdxWidth.W))))
    val intRenamePorts  = Vec(RenameWidth, Output(new RatWritePort))
    val fpRenamePorts   = Vec(RenameWidth, Output(new RatWritePort))
    // from rename table
    val int_old_pdest = Vec(RabCommitWidth, Input(UInt(PhyRegIdxWidth.W)))
    val int_need_free = Vec(RabCommitWidth, Input(Bool()))
    val fp_old_pdest = Vec(RabCommitWidth, Input(UInt(PhyRegIdxWidth.W)))
    // to dispatch1
    val out = Vec(RenameWidth, DecoupledIO(new MicroOp))
    // enq Rob
    val enqRob = Flipped(new RobEnqIO)
    // to vector
    val toVCtl = Output(Vec(RenameWidth, new VtpToVCtl))
    val vcsrio  = Flipped(new VCSRWithVtypeRenameIO)
    // for snapshots
    val snpt = Input(new SnapshotPort)
    val snptLastEnq = Flipped(ValidIO(new RobPtr))
    val snptIsFull= Input(Bool())
    val vlUpdate = Input(Valid(UInt(log2Ceil(VLEN + 1).W)))
    val dispatchIn = Vec(RenameWidth, Input(Valid(new RobPtr)))
    val topdown = new Bundle{
      val ctrlRecStall = Output(Bool())
      val mvioRecStall = Output(Bool())
      val otherRecStall = Output(Bool())
      val intFlStall = Output(Bool())
      val fpFlStall = Output(Bool())
      val vtypeRenameStall = Output(Bool())
      val multiFlStall = Output(Bool())
    }

  })

  // create free list and rat
  val intFreeList = Module(new MEFreeList(NRPhyRegs))
  val fpFreeList = Module(new StdFreeList(NRPhyRegs - 32))
  val vtyperename = Module(new VtypeRename)
  vtyperename.io.redirect := io.redirect
  vtyperename.io.robCommits := io.robCommits
  vtyperename.io.vlUpdate := io.vlUpdate
  vtyperename.io.dispatchIn := io.dispatchIn
  io.toVCtl := vtyperename.io.toVCtl

  // decide if given instruction needs allocating a new physical register (CfCtrl: from decode; RobCommitInfo: from rob)
  def needDestReg[T <: CfCtrl](fp: Boolean, x: T): Bool = {
      if(fp) x.ctrl.fpWen
      else (x.ctrl.rfWen && (x.ctrl.ldest =/= 0.U))
  }

  // connect [redirect + walk] ports for __float point__ & __integer__ free list
  Seq((fpFreeList), (intFreeList)).foreach {
    case (fl) =>
      fl.io.redirect := io.redirect.valid
      fl.io.walk := io.rabCommits.isWalk
      fl.io.rabCommit := io.rabCommits
  }
  for (i <- 0 until RabCommitWidth) {
    intFreeList.io.walkReq(i) := io.rabCommits.walkValid(i) && io.rabCommits.info(i).rfWen && !io.rabCommits.info(i).isMove && io.rabCommits.info(i).ldest =/= 0.U
    fpFreeList.io.walkReq(i) := io.rabCommits.walkValid(i) && io.rabCommits.info(i).fpWen
  }
  // walk has higher priority than allocation and thus we don't use isWalk here
  // only when both fp and int free list and dispatch1 has enough space can we do allocation
  intFreeList.io.doAllocate := fpFreeList.io.canAllocate && vtyperename.io.canAccept && io.out(0).ready && io.enqRob.canAccept || io.rabCommits.isWalk
  fpFreeList.io.doAllocate := intFreeList.io.canAllocate && vtyperename.io.canAccept && io.out(0).ready && io.enqRob.canAccept || io.rabCommits.isWalk

  // dispatch1 ready ++ float point free list ready ++ int free list ready ++ not walk ++ rob canaccept
  val canOut = io.out(0).ready && fpFreeList.io.canAllocate && intFreeList.io.canAllocate && !io.rabCommits.isWalk && !io.robCommits.isWalk && vtyperename.io.canAccept && io.enqRob.canAccept

  // compressUnit: decode instructions guidelines to the ROB allocation logic
  val compressUnit = Module(new CompressUnit())
    compressUnit.io.in.zip(io.in).foreach{ case(sink, source) =>
    sink.valid := source.valid
    sink.bits := source.bits
  }
  val needRobFlags = compressUnit.io.out.needRobFlags
  val instrSizesVec = compressUnit.io.out.instrSizes
  val compressMasksVec = compressUnit.io.out.masks
  dontTouch(needRobFlags);dontTouch(instrSizesVec);dontTouch(compressMasksVec)
  // speculatively assign the instruction with an robIdx
 val validCount = PopCount(io.in.map(_.valid).zip(needRobFlags).map{
  case(valid, needRob) => valid && needRob}) // number of instructions waiting to enter rob (from decode)
 val robIdxHead = RegInit(0.U.asTypeOf(new RobPtr))
 val lastCycleMisprediction = RegNext(io.redirect.valid && !io.redirect.bits.flushItself())
 val robIdxHeadNext = Mux(io.redirect.valid, io.redirect.bits.robIdx, // redirect: move ptr to given rob index
        Mux(lastCycleMisprediction, robIdxHead + 1.U, // mis-predict: not flush robIdx itself
                        Mux(canOut & io.allowIn, robIdxHead + validCount, // instructions successfully entered next stage: increase robIdx
                     /* default */  robIdxHead))) // no instructions passed by this cycle: stick to old value
 robIdxHead := robIdxHeadNext

  /**
    * Rename: allocate free physical register and update rename table
    */
  val uops = Wire(Vec(RenameWidth, new MicroOp))
  uops.foreach( uop => {
    uop.srcState(0) := DontCare
    uop.srcState(1) := DontCare
    uop.srcState(2) := DontCare
    uop.robIdx := DontCare
    uop.debugInfo := DontCare
    uop.lqIdx := DontCare
    uop.sqIdx := DontCare
    uop.lpv := DontCare
    uop.vCsrInfo := DontCare
    uop.vctrl := DontCare
    uop.vctrl.ordered := false.B
    uop.uopIdx := 0.U
    uop.vmState := DontCare
    uop.isTail := DontCare
    uop.isPrestart := DontCare
    uop.vtypeRegIdx := DontCare
    uop.uopNum := 0.U
    uop.vm := DontCare
    uop.canRename := DontCare
    uop.mergeIdx := DontCare
    uop.loadStoreEnable := DontCare
    uop.segIdx := 0.U
    uop.elmIdx := 0.U
    uop.snapshot := DontCare
  })

  val needFpDest  = Wire(Vec(RenameWidth, Bool()))
  val needIntDest = Wire(Vec(RenameWidth, Bool()))
  val hasValid    = Cat(io.in.map(_.valid)).orR
  val hasExceptionVec = Wire(Vec(RenameWidth, Bool()))
  val needFlushPipeVec = Wire(Vec(RenameWidth, Bool()))
  dontTouch(hasExceptionVec)
  dontTouch(needFlushPipeVec)
  val isMove = io.in.map(_.bits.ctrl.isMove)

  val intSpecWen = Wire(Vec(RenameWidth, Bool()))
  val fpSpecWen = Wire(Vec(RenameWidth, Bool()))

  // uop calculation
  for (i <- 0 until RenameWidth) {
    uops(i).cf := io.in(i).bits.cf
    uops(i).ctrl := io.in(i).bits.ctrl
    uops(i).fdiUntrusted := io.in(i).bits.cf.fdiUntrusted

    // We use the lsrc/ldest before fusion decoder to read RAT for better timing.
    io.intRat(i)(0).addr := uops(i).ctrl.lsrc(0)
    io.intRat(i)(1).addr := uops(i).ctrl.lsrc(1)
    io.intRat(i)(2).addr := uops(i).ctrl.ldest
    io.intRat(i).foreach(_.hold := !io.out(i).ready)

    // Floating-point instructions can not be fused now.
    io.fpRat(i)(0).addr := uops(i).ctrl.lsrc(0)
    io.fpRat(i)(1).addr := uops(i).ctrl.lsrc(1)
    io.fpRat(i)(2).addr := uops(i).ctrl.lsrc(2)
    io.fpRat(i)(3).addr := uops(i).ctrl.ldest
    io.fpRat(i).foreach(_.hold := !io.out(i).ready)

    // update cf according to ssit result
    uops(i).cf.storeSetHit := io.ssit(i).valid
    uops(i).cf.loadWaitStrict := io.ssit(i).strict && io.ssit(i).valid
    uops(i).cf.ssid := io.ssit(i).ssid

    // update cf according to waittable result
    uops(i).cf.loadWaitBit := io.waittable(i)

    // alloc a new phy reg
    needFpDest(i) := io.in(i).valid && needDestReg(fp = true, io.in(i).bits) && io.allowIn
    needIntDest(i) := io.in(i).valid && needDestReg(fp = false, io.in(i).bits) && io.allowIn
    fpFreeList.io.allocateReq(i) := needFpDest(i)
    intFreeList.io.allocateReq(i) := needIntDest(i) && !isMove(i)

    // no valid instruction from decode stage || all resources (dispatch1 + both free lists) ready
    io.in(i).ready := !hasValid || canOut

    uops(i).robIdx  := robIdxHead + PopCount(io.in.take(i).map(_.valid).zip(needRobFlags).map{
      case(valid, needRob) => valid && needRob})
    uops(i).psrc(0) := Mux(uops(i).ctrl.srcType(0) === SrcType.reg, io.intReadPorts(i)(0), io.fpReadPorts(i)(0))
    uops(i).psrc(1) := Mux(uops(i).ctrl.srcType(1) === SrcType.reg, io.intReadPorts(i)(1), io.fpReadPorts(i)(1))
    uops(i).psrc(2)         := io.fpReadPorts(i)(2)
    uops(i).old_pdest       := Mux(uops(i).ctrl.rfWen, io.intReadPorts(i).last, io.fpReadPorts(i).last)
    uops(i).eliminatedMove  := isMove(i)
  
    uops(i).compressInstNum := instrSizesVec(i)
    uops(i).compressMask    := compressMasksVec(i)
    hasExceptionVec(i) := Cat(selectFrontend(uops(i).cf.exceptionVec) :+ uops(i).cf.exceptionVec(illegalInstr)).orR 
    needFlushPipeVec(i) := hasExceptionVec(i) || uops(i).cf.trigger.getFrontendCanFire
    uops(i).compressWbNum := instrSizesVec(i) - PopCount(compressMasksVec(i) & (Cat(isMove.reverse) | Cat(needFlushPipeVec.reverse)))

    uops(i).lastUop := needRobFlags(i)
    if(i == 0){uops(i).firstUop := true.B}else{uops(i).firstUop := needRobFlags(i - 1)}

    when(io.out(i).valid){
      assert(instrSizesVec(i)>=1.U, "uop num at least is 1")
      when(instrSizesVec(i)===1.U){
        assert(uops(i).lastUop && uops(i).firstUop,"when compress num is 1, must be first and last uop")
      }.otherwise{
        assert(!(uops(i).lastUop && uops(i).firstUop),"when compress num great than 1, first and last uop can't be same")}
      }
    // update pdest
    uops(i).pdest := Mux(needIntDest(i), intFreeList.io.allocatePhyReg(i), // normal int inst
      // normal fp inst
      Mux(needFpDest(i), fpFreeList.io.allocatePhyReg(i),
        /* default */0.U))

    // Assign performance counters
    uops(i).debugInfo.renameTime := GTimer()

    vtyperename.io.needAlloc(i) := io.in(i).valid && io.in(i).bits.ctrl.isVtype
    vtyperename.io.in(i).bits := uops(i)
    vtyperename.io.in(i).valid := io.in(i).fire && io.in(i).bits.ctrl.isVtype && io.allowIn
    vtyperename.io.vcsr <> io.vcsrio

    //out
    io.out(i).valid := io.in(i).valid && intFreeList.io.canAllocate && fpFreeList.io.canAllocate && !io.robCommits.isWalk && vtyperename.io.canAccept && io.allowIn && io.enqRob.canAccept
    io.out(i).bits  := uops(i)
    io.enqRob.needAlloc(i) := io.in(i).valid && uops(i).firstUop
    io.enqRob.req(i).valid := io.in(i).valid && intFreeList.io.canAllocate && fpFreeList.io.canAllocate && !io.robCommits.isWalk && vtyperename.io.canAccept && io.allowIn && io.out(i).ready
    io.enqRob.req(i).bits := io.out(i).bits
    // dirty code for fence. The lsrc is passed by imm.
    when (io.out(i).bits.ctrl.fuType === FuType.fence) {
      io.out(i).bits.ctrl.imm := Cat(io.in(i).bits.ctrl.lsrc(1), io.in(i).bits.ctrl.lsrc(0))
    }.elsewhen(io.out(i).bits.ctrl.fuOpType === CSROpType.vsetvl || io.out(i).bits.ctrl.fuOpType === CSROpType.vsetvli || io.out(i).bits.ctrl.fuOpType === CSROpType.vsetivli) {
      io.out(i).bits.ctrl.imm := vtyperename.io.out(i).bits.ctrl.imm
    }
    
    when (io.in(i).bits.ctrl.isSoftPrefetch) {
      // dirty code for SoftPrefetch (prefetch.r/prefetch.w)
      io.out(i).bits.ctrl.fuType    := Mux(io.in(i).bits.ctrl.lsrc(0) === 1.U, FuType.ldu, FuType.jmp)
      io.out(i).bits.ctrl.fuOpType  := Mux(io.in(i).bits.ctrl.lsrc(0) === 1.U,
        Mux(io.in(i).bits.ctrl.lsrc(1) === 1.U, LSUOpType.prefetch_r, LSUOpType.prefetch_w),
        JumpOpType.prefetch_i
      )
      io.out(i).bits.ctrl.selImm  := SelImm.IMM_S
      io.out(i).bits.ctrl.imm     := Cat(io.in(i).bits.ctrl.imm(io.in(i).bits.ctrl.imm.getWidth - 1, 5), 0.U(5.W))
    }

    // write speculative rename table
    // we update rat later inside commit code
    intSpecWen(i) := needIntDest(i) && intFreeList.io.canAllocate && intFreeList.io.doAllocate && !io.robCommits.isWalk && !io.redirect.valid && io.allowIn && io.enqRob.canAccept
    fpSpecWen(i)  := needFpDest(i) && fpFreeList.io.canAllocate && fpFreeList.io.doAllocate && !io.robCommits.isWalk && !io.redirect.valid && io.allowIn && io.enqRob.canAccept

  }

  /**
    * How to set psrc:
    * - bypass the pdest to psrc if previous instructions write to the same ldest as lsrc
    * - default: psrc from RAT
    * How to set pdest:
    * - Mux(isMove, psrc, pdest_from_freelist).
    *
    * The critical path of rename lies here:
    * When move elimination is enabled, we need to update the rat with psrc.
    * However, psrc maybe comes from previous instructions' pdest, which comes from freelist.
    *
    * If we expand these logic for pdest(N):
    * pdest(N) = Mux(isMove(N), psrc(N), freelist_out(N))
    *          = Mux(isMove(N), Mux(bypass(N, N - 1), pdest(N - 1),
    *                           Mux(bypass(N, N - 2), pdest(N - 2),
    *                           ...
    *                           Mux(bypass(N, 0),     pdest(0),
    *                                                 rat_out(N))...)),
    *                           freelist_out(N))
    */
  // a simple functional model for now
  io.out(0).bits.pdest := Mux(isMove(0), uops(0).psrc.head, uops(0).pdest)
  io.out(0).bits.vtypeRegIdx := vtyperename.io.out(0).bits.vtypeRegIdx
  io.toVCtl(0).psrc := io.out(0).bits.psrc
  io.toVCtl(0).pdest := io.out(0).bits.pdest
  io.toVCtl(0).old_pdest := io.out(0).bits.old_pdest
  io.toVCtl(0).robIdx := io.out(0).bits.robIdx
  val bypassCond = Wire(Vec(4, MixedVec(List.tabulate(RenameWidth-1)(i => UInt((i+1).W)))))
  for (i <- 1 until RenameWidth) {
    val fpCond = io.in(i).bits.ctrl.srcType.map(_ === SrcType.fp) :+ needFpDest(i)
    val intCond = io.in(i).bits.ctrl.srcType.map(_ === SrcType.reg) :+ needIntDest(i)
    val target = io.in(i).bits.ctrl.lsrc :+ io.in(i).bits.ctrl.ldest
    for ((((cond1, cond2), t), j) <- fpCond.zip(intCond).zip(target).zipWithIndex) {
      val destToSrc = io.in.take(i).zipWithIndex.map { case (in, j) =>
        val indexMatch = in.bits.ctrl.ldest === t
        val writeMatch =  cond2 && needIntDest(j) || cond1 && needFpDest(j)
        indexMatch && writeMatch
      }
      bypassCond(j)(i - 1) := VecInit(destToSrc).asUInt
    }
    io.out(i).bits.psrc(0) := io.out.take(i).map(_.bits.pdest).zip(bypassCond(0)(i-1).asBools).foldLeft(uops(i).psrc(0)) {
      (z, next) => Mux(next._2, next._1, z)
    }
    io.out(i).bits.psrc(1) := io.out.take(i).map(_.bits.pdest).zip(bypassCond(1)(i-1).asBools).foldLeft(uops(i).psrc(1)) {
      (z, next) => Mux(next._2, next._1, z)
    }
    io.out(i).bits.psrc(2) := io.out.take(i).map(_.bits.pdest).zip(bypassCond(2)(i-1).asBools).foldLeft(uops(i).psrc(2)) {
      (z, next) => Mux(next._2, next._1, z)
    }
    io.out(i).bits.old_pdest := io.out.take(i).map(_.bits.pdest).zip(bypassCond(3)(i-1).asBools).foldLeft(uops(i).old_pdest) {
      (z, next) => Mux(next._2, next._1, z)
    }
    io.out(i).bits.pdest := Mux(isMove(i), io.out(i).bits.psrc(0), uops(i).pdest)

    // For fused-lui-load, load.src(0) is replaced by the imm.
    val last_is_lui = io.in(i - 1).bits.ctrl.selImm === SelImm.IMM_U && io.in(i - 1).bits.ctrl.srcType(0) =/= SrcType.pc
    val this_is_load = io.in(i).bits.ctrl.fuType === FuType.ldu && !io.in(i).bits.ctrl.isVector
    val lui_to_load = io.in(i - 1).valid && io.in(i - 1).bits.ctrl.ldest === io.in(i).bits.ctrl.lsrc(0)
    val fused_lui_load = last_is_lui && this_is_load && lui_to_load
    val setVlBypass = io.out(i).bits.ctrl.fuType === FuType.csr &&
      (io.out(i).bits.ctrl.fuOpType === CSROpType.vsetvl || io.out(i).bits.ctrl.fuOpType === CSROpType.vsetvli) &&
      io.out(i).bits.ctrl.lsrc(0) === 0.U && io.out(i).bits.ctrl.ldest === 0.U
    when (fused_lui_load) {
      // The first LOAD operand (base address) is replaced by LUI-imm and stored in {psrc, imm}
      val lui_imm = io.in(i - 1).bits.ctrl.imm
      val ld_imm = io.in(i).bits.ctrl.imm
      io.out(i).bits.ctrl.srcType(0) := SrcType.imm
      io.out(i).bits.ctrl.imm := Imm_LUI_LOAD().immFromLuiLoad(lui_imm, ld_imm)
      val psrcWidth = uops(i).psrc.head.getWidth
      val lui_imm_in_imm = uops(i).ctrl.imm.getWidth - Imm_I().len
      val left_lui_imm = Imm_U().len - lui_imm_in_imm
      require(2 * psrcWidth >= left_lui_imm, "cannot fused lui and load with psrc")
      io.out(i).bits.psrc(0) := lui_imm(lui_imm_in_imm + psrcWidth - 1, lui_imm_in_imm)
      io.out(i).bits.psrc(1) := lui_imm(lui_imm.getWidth - 1, lui_imm_in_imm + psrcWidth)
    }.elsewhen(setVlBypass){
      io.out(i).bits.psrc(0) := vtyperename.io.out(i).bits.psrc(0)
    }
    io.out(i).bits.vtypeRegIdx := vtyperename.io.out(i).bits.vtypeRegIdx
    io.toVCtl(i).psrc := io.out(i).bits.psrc
    io.toVCtl(i).pdest := io.out(i).bits.pdest
    io.toVCtl(i).old_pdest := io.out(i).bits.old_pdest
    io.toVCtl(i).robIdx := io.out(i).bits.robIdx
  }
  // snap generate
  val genSnapshot = Cat(io.out.map(out => out.fire && out.bits.snapshot)).orR
  val lastCycleCreateSnpt = RegInit(false.B)
  lastCycleCreateSnpt := genSnapshot && !io.snptIsFull
  val sameSnptDistance = (CommitWidth * 4).U
  // notInSameSnpt: 1.robidxHead - snapLastEnq >= sameSnptDistance 2.no snap
  val notInSameSnpt = RegNext(distanceBetween(robIdxHeadNext, io.snptLastEnq.bits) >= sameSnptDistance || !io.snptLastEnq.valid)
  val allowSnpt = if (EnableRenameSnapshot) notInSameSnpt && !lastCycleCreateSnpt && io.out.head.bits.firstUop else false.B
  io.out.zip(io.in).foreach{ case (out, in) => out.bits.snapshot := allowSnpt && (!in.bits.cf.pd.notCFI || FuType.isJumpExu(in.bits.ctrl.fuType)) && in.fire }

  val setVlBypass0 = io.out(0).bits.ctrl.fuType === FuType.csr &&
                    (io.out(0).bits.ctrl.fuOpType === CSROpType.vsetvl || io.out(0).bits.ctrl.fuOpType === CSROpType.vsetvli) &&
                    io.out(0).bits.ctrl.lsrc(0) === 0.U && io.out(0).bits.ctrl.ldest === 0.U
  when(setVlBypass0) {
    io.out(0).bits.psrc(0) := vtyperename.io.out(0).bits.psrc(0)
  }

  /**
    * Instructions commit: update freelist and rename table
    */
  for (i <- 0 until CommitWidth) {
    val commitValid = io.rabCommits.isCommit && io.rabCommits.commitValid(i)
    val walkValid = io.rabCommits.isWalk && io.rabCommits.walkValid(i)

    Seq((io.intRenamePorts, false), (io.fpRenamePorts, true)) foreach { case (rat, fp) =>
      /*
      I. RAT Update
       */

      // walk back write - restore spec state : ldest => old_pdest
      if (fp && i < RenameWidth) {
        // When redirect happens (mis-prediction), don't update the rename table
        rat(i).wen := fpSpecWen(i)
        rat(i).addr := uops(i).ctrl.ldest
        rat(i).data := fpFreeList.io.allocatePhyReg(i)
      } else if (!fp && i < RenameWidth) {
        rat(i).wen := intSpecWen(i)
        rat(i).addr := uops(i).ctrl.ldest
        rat(i).data := io.out(i).bits.pdest
      }

      /*
      II. Free List Update
       */
      if (fp) { // Float Point free list
        fpFreeList.io.freeReq(i)  := RegNext(commitValid && io.rabCommits.info(i).fpWen)
        fpFreeList.io.freePhyReg(i) := io.fp_old_pdest(i)
      } else { // Integer free list
        intFreeList.io.freeReq(i) := io.int_need_free(i)
        intFreeList.io.freePhyReg(i) := RegNext(io.int_old_pdest(i))
      }
    }

  }
  intFreeList.io.snpt := io.snpt
  fpFreeList.io.snpt := io.snpt
  intFreeList.io.snpt.snptEnq := genSnapshot
  fpFreeList.io.snpt.snptEnq := genSnapshot

  /*
  Debug and performance counters
   */
  def printRenameInfo(in: DecoupledIO[CfCtrl], out: DecoupledIO[MicroOp]) = {
    XSInfo(out.fire, p"pc:${Hexadecimal(in.bits.cf.pc)} in(${in.valid},${in.ready}) " +
      p"lsrc(0):${in.bits.ctrl.lsrc(0)} -> psrc(0):${out.bits.psrc(0)} " +
      p"lsrc(1):${in.bits.ctrl.lsrc(1)} -> psrc(1):${out.bits.psrc(1)} " +
      p"lsrc(2):${in.bits.ctrl.lsrc(2)} -> psrc(2):${out.bits.psrc(2)} " +
      p"ldest:${in.bits.ctrl.ldest} -> pdest:${out.bits.pdest} " +
      p"old_pdest:${out.bits.old_pdest}\n"
    )
  }

  for((x,y) <- io.in.zip(io.out)){
    printRenameInfo(x, y)
  }

  XSDebug(io.robCommits.isWalk, p"Walk Recovery Enabled\n")
  XSDebug(io.robCommits.isWalk, p"validVec:${Binary(io.robCommits.walkValid.asUInt)}\n")
  for (i <- 0 until CommitWidth) {
    val info = io.robCommits.info(i)
    XSDebug(io.robCommits.isWalk && io.robCommits.walkValid(i), p"[#$i walk info] pc:${Hexadecimal(info.pc)} " +
      p"ldest:${info.ldest} rfWen:${info.rfWen} fpWen:${info.fpWen} " +
      p"pdest:${info.pdest} old_pdest:${info.old_pdest}\n")
  }

  XSDebug(p"inValidVec: ${Binary(Cat(io.in.map(_.valid)))}\n")

  XSPerfAccumulate("in", Mux(RegNext(io.in(0).ready), PopCount(io.in.map(_.valid)), 0.U))
  XSPerfAccumulate("utilization", PopCount(io.in.map(_.valid)))
  XSPerfAccumulate("waitInstr", PopCount((0 until RenameWidth).map(i => io.in(i).valid && !io.in(i).ready)))
  XSPerfAccumulate("stall_cycle_dispatch", hasValid && !io.out(0).ready && fpFreeList.io.canAllocate && intFreeList.io.canAllocate && !io.robCommits.isWalk && io.enqRob.canAccept)
  XSPerfAccumulate("stall_cycle_fp", hasValid && io.out(0).ready && !fpFreeList.io.canAllocate && intFreeList.io.canAllocate && !io.robCommits.isWalk && io.enqRob.canAccept)
  XSPerfAccumulate("stall_cycle_int", hasValid && io.out(0).ready && fpFreeList.io.canAllocate && !intFreeList.io.canAllocate && !io.robCommits.isWalk && io.enqRob.canAccept)
  XSPerfAccumulate("stall_cycle_robfull", hasValid && io.out(0).ready && fpFreeList.io.canAllocate && intFreeList.io.canAllocate && !io.robCommits.isWalk && !io.enqRob.canAccept)
  XSPerfAccumulate("stall_cycle_walk", hasValid && io.out(0).ready && fpFreeList.io.canAllocate && intFreeList.io.canAllocate && io.robCommits.isWalk && io.enqRob.canAccept)
  XSPerfAccumulate("recovery_bubbles", PopCount(io.in.map(_.valid && io.out(0).ready && fpFreeList.io.canAllocate && intFreeList.io.canAllocate && io.robCommits.isWalk && io.enqRob.canAccept)))
  XSPerfAccumulate("compress_instr", PopCount(io.out.map(o => o.fire && o.bits.compressInstNum > 1.U)))
  XSPerfAccumulate("move_instr_count", PopCount(io.out.map(out => out.fire && out.bits.ctrl.isMove)))
  val is_fused_lui_load = io.out.map(o => o.fire && o.bits.ctrl.fuType === FuType.ldu && o.bits.ctrl.srcType(0) === SrcType.imm)
  XSPerfAccumulate("fused_lui_load_instr_count", PopCount(is_fused_lui_load))


  val renamePerf = Seq(
    ("rename_in                  ", PopCount(io.in.map(_.valid & io.in(0).ready ))                                                               ),
    ("rename_waitinstr           ", PopCount((0 until RenameWidth).map(i => io.in(i).valid && !io.in(i).ready))                                  ),
    ("rename_stall_cycle_dispatch", hasValid && !io.out(0).ready &&  fpFreeList.io.canAllocate &&  intFreeList.io.canAllocate && !io.robCommits.isWalk),
    ("rename_stall_cycle_fp      ", hasValid &&  io.out(0).ready && !fpFreeList.io.canAllocate &&  intFreeList.io.canAllocate && !io.robCommits.isWalk),
    ("rename_stall_cycle_int     ", hasValid &&  io.out(0).ready &&  fpFreeList.io.canAllocate && !intFreeList.io.canAllocate && !io.robCommits.isWalk),
    ("rename_stall_cycle_walk    ", hasValid &&  io.out(0).ready &&  fpFreeList.io.canAllocate &&  intFreeList.io.canAllocate &&  io.robCommits.isWalk)
  )
  val intFlPerf = intFreeList.getPerfEvents
  val fpFlPerf = fpFreeList.getPerfEvents
  val perfEvents = renamePerf ++ intFlPerf ++ fpFlPerf
  generatePerfEvent()

  // bad speculation
  val debugRedirect = RegEnable(io.redirect.bits, io.redirect.valid)
  val recStall = io.redirect.valid || io.rabCommits.isWalk
  val ctrlRecStall = Mux(io.redirect.valid, io.redirect.bits.debugIsCtrl, io.rabCommits.isWalk && debugRedirect.debugIsCtrl)
  val mvioRecStall = Mux(io.redirect.valid, io.redirect.bits.debugIsMemVio, io.rabCommits.isWalk && debugRedirect.debugIsMemVio)
  val otherRecStall = recStall && !(ctrlRecStall || mvioRecStall)
  XSPerfAccumulate("recovery_stall", recStall)
  XSPerfAccumulate("control_recovery_stall", ctrlRecStall)
  XSPerfAccumulate("mem_violation_recovery_stall", mvioRecStall)
  XSPerfAccumulate("other_recovery_stall", otherRecStall)
  // freelist stall
  private val inHeadValid = io.in.head.valid
  val notRecStall = !io.out.head.valid && !recStall
  val intFlStall = notRecStall && inHeadValid && fpFreeList.io.canAllocate && vtyperename.io.canAccept && !intFreeList.io.canAllocate
  val fpFlStall = notRecStall && inHeadValid && intFreeList.io.canAllocate && vtyperename.io.canAccept && !fpFreeList.io.canAllocate
  val vtypeRenameStall = notRecStall && inHeadValid && intFreeList.io.canAllocate && fpFreeList.io.canAllocate && vtyperename.io.canAccept
  val multiFlStall = notRecStall && inHeadValid && (PopCount(Cat(
    !intFreeList.io.canAllocate,
    !fpFreeList.io.canAllocate,
    !vtyperename.io.canAccept)) > 1.U)
  // other stall
  val otherStall = notRecStall && !intFlStall && !fpFlStall && !vtypeRenameStall

  io.topdown.ctrlRecStall     := ctrlRecStall
  io.topdown.mvioRecStall     := mvioRecStall
  io.topdown.otherRecStall    := otherRecStall
  io.topdown.intFlStall       := intFlStall
  io.topdown.fpFlStall        := fpFlStall
  io.topdown.vtypeRenameStall := vtypeRenameStall
  io.topdown.multiFlStall     := multiFlStall
}
