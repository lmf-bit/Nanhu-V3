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

package xiangshan.backend.rename.freelist

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan._
import utils._
import xs.utils.CircularShift
import xs.utils.perf.HasPerfLogging


class MEFreeList(size: Int)(implicit p: Parameters) extends BaseFreeList(size) with HasPerfEvents with HasPerfLogging{
  val freeList = RegInit(VecInit(
    // originally {1, 2, ..., size - 1} are free. Register 0-31 are mapped to x0.
    Seq.tabulate(size - 1)(i => (i + 1).U(PhyRegIdxWidth.W)) :+ 0.U(PhyRegIdxWidth.W)))


  val tailPtr = RegInit(FreeListPtr(false, size - 1))

  dontTouch(redirectedHeadPtr)
  // consider during the walk has another redirect
  val alreadyInWalk = RegNext(io.walk)
  val walkHasRedirect = alreadyInWalk && lastCycleRedirect
  // consider the successive redirect come 
  val headPtrCanMove = io.redirect && io.walk
  dontTouch(headPtrCanMove)
  val doWalkRename = io.walk && io.doAllocate && !io.redirect
  val doNormalRename = io.canAllocate && io.doAllocate && !io.redirect
  val doRename = doWalkRename || doNormalRename

  /**
    * Allocation: from freelist (same as StdFreelist)
    */
  val phyRegCandidates = VecInit(headPtrOHVec.map(sel => Mux1H(sel, freeList)))
  dontTouch(phyRegCandidates)
  for (i <- 0 until RenameWidth) {
    // enqueue instr, is move elimination
    io.allocatePhyReg(i) := phyRegCandidates(PopCount(io.allocateReq.take(i)))
  }
  // update arch head pointer
  val archAlloc = io.rabCommit.commitValid zip io.rabCommit.info map {
    case (valid, info) => valid && info.rfWen && !info.isMove && info.ldest =/= 0.U
  }
  val numArchAllocate = PopCount(archAlloc)
  val archHeadPtrNew  = archHeadPtr + numArchAllocate
  val archHeadPtrNext = Mux(io.rabCommit.isCommit, archHeadPtrNew, archHeadPtr)
  archHeadPtr := archHeadPtrNext
  // update head pointer
  val numAllocate = Mux(io.walk, PopCount(io.walkReq), PopCount(io.allocateReq))
  val headPtrNext = Mux(lastCycleRedirect, redirectedHeadPtr, headPtr + numAllocate)
  val headPtrOHNext = Mux(lastCycleRedirect, redirectedHeadPtrOH, headPtrOHVec(numAllocate))
  headPtr := Mux(doRename, headPtrNext, headPtr)
  headPtrOH := Mux(doRename, headPtrOHNext, headPtrOH)
  dontTouch(headPtr)


  /**
    * Deallocation: when refCounter becomes zero, the register can be released to freelist
    */
  for (i <- 0 until CommitWidth) {
    when (io.freeReq(i)) {
      val freePtr = tailPtr + PopCount(io.freeReq.take(i))
      freeList(freePtr.value) := io.freePhyReg(i)
    }
  }

  // update tail pointer
  val tailPtrNext = tailPtr + PopCount(io.freeReq)
  tailPtr := tailPtrNext

  val freeRegCntReg = RegInit(0.U(log2Up(size + 1).W))
  freeRegCntReg := Mux(doWalkRename && !lastCycleRedirect, distanceBetween(tailPtrNext, headPtr) - PopCount(io.walkReq),
                   Mux(doNormalRename,                     distanceBetween(tailPtrNext, headPtr) - PopCount(io.allocateReq),
                                                           distanceBetween(tailPtrNext, headPtr)))
  io.canAllocate := freeRegCntReg >= RenameWidth.U

  val perfEvents = Seq(
    ("me_freelist_1_4_valid", freeRegCntReg <  (size / 4).U                                     ),
    ("me_freelist_2_4_valid", freeRegCntReg >= (size / 4).U && freeRegCntReg <= (size / 2).U    ),
    ("me_freelist_3_4_valid", freeRegCntReg >= (size / 2).U && freeRegCntReg <= (size * 3 / 4).U),
    ("me_freelist_4_4_valid", freeRegCntReg >= (size * 3 / 4).U                                 ),
  )
  generatePerfEvent()
}
