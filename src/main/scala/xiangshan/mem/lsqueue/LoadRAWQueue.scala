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
import xiangshan._
import xiangshan.backend.rob.RobPtr
import xiangshan.frontend.FtqPtr
import xiangshan.mem.{LsqFreeList, SqPtr}
import xiangshan.mem.lsqueue.LoadRAWQueueDataModule
import xs.utils.HasCircularQueuePtrHelper
import xs.utils.perf.HasPerfLogging


class RAWEnqBundle(implicit p: Parameters) extends XSBundle {
  val robIdx = new RobPtr
  val sqIdx = new SqPtr
  val mask = UInt((XLEN/8).W)
  val paddr = UInt((PAddrBits - 3).W)
//  val data_valid = Bool()
  val ftqPtr = new FtqPtr
  val ftqOffset = UInt(log2Up(PredictWidth).W)
}

class LoadEnqRAWBundle(implicit p: Parameters) extends XSBundle {
  val s2_enq = Output(Valid(new RAWEnqBundle))
  val s2_enqSuccess = Input(Bool())
  val s3_cancel = Output(Bool())
}


class storeRAWQueryBundle(implicit p: Parameters) extends XSBundle {
  val paddr = UInt((PAddrBits - 3).W)
  val mask = UInt(8.W)
  val robIdx = new RobPtr

  val stFtqPtr = new FtqPtr
  val stFtqOffset = UInt(log2Up(PredictWidth).W)
}

class RAWInfoUop(implicit p: Parameters) extends XSBundle {
//  val uop = new MicroOp
  val robIdx = new RobPtr
  val sqIdx = new SqPtr
  val ftqPtr = new FtqPtr
  val ftqOffset = UInt(log2Up(PredictWidth).W)
}


class LoadRAWQueue(implicit p: Parameters) extends XSModule
  with HasPerfLogging
  with HasCircularQueuePtrHelper{
  val io = IO(new Bundle() {
    val redirect = Flipped(ValidIO(new Redirect))
    val loadEnq = Vec(LoadPipelineWidth, Flipped(new LoadEnqRAWBundle)) //Load S2 enq //todo
    val stAddrReadyPtr = Input(new SqPtr)
    val stAddrAllReady = Input(Bool())
    val storeQuery = Vec(StorePipelineWidth,Flipped(ValidIO(new storeRAWQueryBundle))) //store S1 query
    val rollback = Output(Valid(new Redirect))
    val isFull = Output(Bool())
  })

  private val freeWidth = 4

  private val allocatedReg = RegInit(VecInit(List.fill(LoadRAWQueueSize)(false.B)))
  private val uopReg = Reg(Vec(LoadRAWQueueSize,new RAWInfoUop))
  private val dataModule = Module(new LoadRAWQueueDataModule(
    queryWidth = StorePipelineWidth,
    entrySize = LoadRAWQueueSize,
    writeNum = LoadPipelineWidth,
    numWBanks = 1,
    nextWrite = false
  ))
  dataModule.io := DontCare
  dataModule.io.write.foreach(_.wen := false.B)

  private val freeList = Module(new LsqFreeList(
    size = LoadRAWQueueSize,
    allocWidth = LoadPipelineWidth,
    freeWidth = freeWidth,
    enablePreAlloc = true,
    moduleName = "LoadRAWQueue freelist"
  ))
  freeList.io := DontCare
  assert(freeList.io.validCount <= LoadRAWQueueSize.U)

  private val needAllocateEntryVec = Wire(Vec(LoadPipelineWidth,Valid(UInt(log2Up(LoadRAWQueueSize).W)))) //actually enqueue Idx
  dontTouch(needAllocateEntryVec)

  //allocate entry
  needAllocateEntryVec.map(_.valid).zip(io.loadEnq).foreach({case (valid,req) =>
    valid := req.s2_enq.valid && !req.s2_enq.bits.robIdx.needFlush(io.redirect) &&
      Mux(io.stAddrAllReady,false.B,(io.stAddrReadyPtr < req.s2_enq.bits.sqIdx))
  })

  for ((enq, idx) <- io.loadEnq.zipWithIndex) {
    freeList.io.doAllocate(idx) := false.B
    freeList.io.allocateReq(idx) := true.B

    val offset = PopCount(needAllocateEntryVec.map(_.valid).take(idx))
    val canAccept = freeList.io.canAllocate(offset)

    needAllocateEntryVec(idx).bits := freeList.io.allocateSlot(offset)
    enq.s2_enqSuccess := Mux(needAllocateEntryVec(idx).valid, canAccept, true.B)
  }

  private val reqWriteValidVec = Wire(Vec(LoadPipelineWidth,Bool()))
  reqWriteValidVec.zipWithIndex.foreach({case(reqWriteValid,idx) =>
    reqWriteValid := needAllocateEntryVec(idx).valid && io.loadEnq(idx).s2_enqSuccess

    when(reqWriteValid){
      freeList.io.doAllocate(idx) := true.B
      dataModule.io.write(idx).wen := true.B
      dataModule.io.write(idx).entryAddr := needAllocateEntryVec(idx).bits
      dataModule.io.write(idx).paddr := io.loadEnq(idx).s2_enq.bits.paddr
      dataModule.io.write(idx).mask := io.loadEnq(idx).s2_enq.bits.mask
    }
  })

  (0 until LoadRAWQueueSize).foreach({ rawIdx =>
    val w_enVec = Wire(Vec(LoadPipelineWidth,Bool()))
    w_enVec.zipWithIndex.foreach({case (en,idx) =>
      en := reqWriteValidVec(idx) && needAllocateEntryVec(idx).bits === rawIdx.U
    })
    val w_addr = OHToUInt(w_enVec)
    val w_data = io.loadEnq(w_addr).s2_enq.bits
    val w_en = w_enVec.reduce(_|_)

    when(w_en) {
      allocatedReg(rawIdx) := true.B
      uopReg(rawIdx).robIdx := w_data.robIdx
      uopReg(rawIdx).sqIdx := w_data.sqIdx

      uopReg(rawIdx).ftqPtr := w_data.ftqPtr
      uopReg(rawIdx).ftqOffset := w_data.ftqOffset
    }
  })


  needAllocateEntryVec.zipWithIndex.foreach({ case (self, idx) =>
    needAllocateEntryVec.zipWithIndex.filterNot(_._2 == idx).foreach({ other =>
      assert(!(reqWriteValidVec(idx) && reqWriteValidVec(other._2) && self.bits === other._1.bits),
        "needAllocateEntryVec must be different!")
    })
  })


  allocatedReg.zipWithIndex.foreach({ case (self, idx) =>
    allocatedReg.zipWithIndex.filterNot(_._2 == idx).foreach({ other =>
      assert(!(other._1 && self && uopReg(idx).robIdx === uopReg(other._2).robIdx), "has same rob!")
    })
  })

  //free entry
  //1. when the stAddrReadyPtr is after uop.sqIdx
  //2. redirect
  //3. when the inst enqueue replayQueue, the enqueue should be cancel in last cycle
  private val normalFreeVec = WireInit(VecInit((0 until LoadRAWQueueSize).map(j => false.B)))
  private val redirectFreeVec = WireInit(VecInit((0 until LoadRAWQueueSize).map(j => false.B)))
  private val cancelFreeVec = WireInit(VecInit((0 until LoadRAWQueueSize).map(j => false.B)))
  private val freeMaskVec = WireInit(VecInit((0 until LoadRAWQueueSize).map(j => false.B)))

  dontTouch(normalFreeVec)
  dontTouch(redirectFreeVec)
  dontTouch(cancelFreeVec)
  dontTouch(freeMaskVec)

  needAllocateEntryVec.zipWithIndex.foreach({case(entry,idx) =>
    val lastEnq = RegEnable(entry,io.loadEnq(idx).s2_enq.valid)
    val cancel = io.loadEnq(idx).s3_cancel
    when(lastEnq.valid && cancel){
      cancelFreeVec(lastEnq.bits) := true.B
    }
  })

  freeMaskVec.zipWithIndex.foreach({case(free,idx) =>
    normalFreeVec(idx) := Mux(io.stAddrAllReady, true.B, !isBefore(io.stAddrReadyPtr, uopReg(idx).sqIdx))
    redirectFreeVec(idx) := uopReg(idx).robIdx.needFlush(io.redirect)
    free := (normalFreeVec(idx) | redirectFreeVec(idx) | cancelFreeVec(idx)) && allocatedReg(idx)
  })

  freeList.io.free := freeMaskVec.asUInt

  freeMaskVec.zipWithIndex.foreach({case(free,idx) =>
    when(free){   //todo
      allocatedReg(idx) := false.B
    }
  })

  //detect store load violation
  private val s0_stFtq = Wire(Vec(StorePipelineWidth, new Bundle() {
    val stFtqIdx = new FtqPtr
    val stFtqOffset = UInt(log2Up(PredictWidth).W)
  }))
  private val s1_stFtq = WireInit(0.U.asTypeOf(s0_stFtq))


  //  val violationOldestVec = Wire(Vec(StorePipelineWidth,Valid(new RobPtr)))
  private val violationOldestEntryIdxVec = Wire(Vec(StorePipelineWidth,Valid(UInt(LoadRAWQueueSize.W))))
  dontTouch(violationOldestEntryIdxVec) //violationOldestEntryIdxVec.bits is OneHot
  require(io.storeQuery.length == dataModule.io.violation.length)

  io.storeQuery.zipWithIndex.foreach({case (query,idx) =>
    //S0: store_s1 req
    s0_stFtq(idx).stFtqIdx := query.bits.stFtqPtr
    s0_stFtq(idx).stFtqOffset := query.bits.stFtqOffset

    dataModule.io.violation(idx).paddr := query.bits.paddr
    dataModule.io.violation(idx).mask := query.bits.mask
    val s0_needCheck = VecInit((0 until LoadRAWQueueSize).map(j => {
      query.valid && allocatedReg(j) &&
        !uopReg(j).robIdx.needFlush(io.redirect) &&
        isAfter(uopReg(j).robIdx,query.bits.robIdx)
    }))
    s0_needCheck.suggestName(s"s0_needCheck_${idx}")
    val s0_addrMaskMatch = dataModule.io.violation(idx).violationMask.asUInt

    //S1: store_s2
    //Do not add "init" parameters randomly!!!!
    s1_stFtq(idx) := RegEnable(s0_stFtq(idx), query.valid)
    val s1_addrMaskMatch = RegEnable(s0_addrMaskMatch, query.valid)
    val s1_needCheck = RegEnable(s0_needCheck, 0.U.asTypeOf(s0_needCheck), query.valid)
    val s1_violationValid = VecInit((0 until LoadRAWQueueSize).map(j => {
      s1_addrMaskMatch(j) && s1_needCheck(j)
    }))
    val uopRob = uopReg.map(_.robIdx)

    val selModule = Module(new RAWQueueSelectPolicy(LoadRAWQueueSize, true, 8,idx))
    selModule.io.in.zipWithIndex.foreach({case (in,i) =>
      in.valid := s1_violationValid(i) && RegNext(query.valid,false.B) && !freeMaskVec(i)
      in.bits := uopRob(i)
    })

    //output is OneHot
    violationOldestEntryIdxVec(idx) := selModule.io.out
    when(violationOldestEntryIdxVec(idx).valid){
      assert(PopCount(violationOldestEntryIdxVec(idx).bits) <= 1.U,s"violationOldestEntryIdxVec${idx} must be OneHot!")
    }
  })

  private val violationSelector = Module(new ViolationSelector(StorePipelineWidth,true))
  require(violationSelector.io.in.length == violationOldestEntryIdxVec.length)

  violationSelector.io.in.zipWithIndex.foreach({case (in,idx) =>
    val entryIdx = OHToUInt(violationOldestEntryIdxVec(idx).bits)
    val rob = uopReg(entryIdx).robIdx
    in.valid := violationOldestEntryIdxVec(idx).valid && RegNext(io.storeQuery(idx).valid,false.B) && !freeMaskVec(entryIdx)//can't ignore
    in.bits := rob

    when(in.valid){ assert(allocatedReg(entryIdx) || freeMaskVec(entryIdx)) }
  })

  private val rollbackStIdx = OHToUInt(violationSelector.io.chosen)
  private val rollbackEntryIdx = OHToUInt(violationOldestEntryIdxVec(rollbackStIdx).bits)
  private val rollbackRes = Wire(Valid(new Redirect))
  private val rollbackRob = violationSelector.io.out.bits
  rollbackRes.valid := violationSelector.io.out.valid && !rollbackRob.needFlush(io.redirect)
  rollbackRes.bits.robIdx := rollbackRob
  rollbackRes.bits.ftqIdx := uopReg(rollbackEntryIdx).ftqPtr
  rollbackRes.bits.ftqOffset := uopReg(rollbackEntryIdx).ftqOffset
  rollbackRes.bits.stFtqIdx := s1_stFtq(rollbackStIdx).stFtqIdx
  rollbackRes.bits.stFtqOffset := s1_stFtq(rollbackStIdx).stFtqOffset
  rollbackRes.bits.level := RedirectLevel.flush
  rollbackRes.bits.interrupt := false.B
  rollbackRes.bits.cfiUpdate := DontCare
  rollbackRes.bits.cfiUpdate.target := DontCare //no use pc
  rollbackRes.bits.isException := false.B
  rollbackRes.bits.isLoadStore := true.B
  rollbackRes.bits.isLoadLoad := false.B
  rollbackRes.bits.isXRet := false.B
  rollbackRes.bits.isFlushPipe := false.B
  rollbackRes.bits.isPreWalk := false.B

  io.rollback := Pipe(rollbackRes)


  io.isFull := freeList.io.empty
  dontTouch(io.isFull)

  XSPerfAccumulate("memBlock_rawQueue_rollback",   io.rollback.valid)

}