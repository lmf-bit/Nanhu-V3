/***************************************************************************************
 * Copyright (c) 2020-2023 Institute of Computing Technology, Chinese Academy of Sciences
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

/*--------------------------------------------------------------------------------------
    Author: GMX
    Date: 2023-06-28
    email: guanmingxing@bosc.ac.cn

    Free List for allocating free physical reg id
---------------------------------------------------------------------------------------*/

package xiangshan.vector.virename


import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._

import xiangshan._
import utils._
import xs.utils.{CircularQueuePtr, HasCircularQueuePtrHelper, CircularShift}
import xiangshan.vector._
import freechips.rocketchip.jtag.JtagState

class VIFreeList(implicit p: Parameters) extends VectorBaseModule with HasCircularQueuePtrHelper {
  val io = IO(new Bundle {
    //be different from int and float rename module
    //freeEntry num, determines whether the requests can enter RenameModule
    //connects with RAT, offer PhyRegIndex
    val needAlloc = Input(Vec(VIRenameWidth, Bool()))
    val canAccept = Output(Bool())
    val allocatePhyReg = Vec(VIRenameWidth, UInt(VIPhyRegIdxWidth.W))
    //connects with RollBackList
    val releasePhyReg = Vec(8, Flipped(ValidIO(UInt(VIPhyRegIdxWidth.W))))
    // for snapshots
    val snpt = Input(new SnapshotPort)
    // redirect
    val redirect = Flipped(ValidIO(new Redirect))
  })

  class VIFreeListPtr extends CircularQueuePtr[VIFreeListPtr](VIPhyRegsNum - 32)
  object VIFreeListPtr {
    def apply(f: Boolean, v: Int): VIFreeListPtr = {
      val ptr = Wire(new VIFreeListPtr)
      ptr.flag := f.B
      ptr.value := v.U
      ptr
    }
  }
  // free list
  // FreeList init value is: [32, 33, 34, ..., 61, 62]
  require(VIPhyRegsNum > 32)
  private val freeList_ds = VecInit(Seq.tabulate(VIPhyRegsNum - 32)(i => (i + 32).U(PhyRegIdxWidth.W)))
  private val freeList = RegInit(freeList_ds)

  //head and tail pointer
  private val allocatePtr = RegInit(VIFreeListPtr(false, 0))
  private val releasePtr = RegInit(VIFreeListPtr(true, 0))
  assert(releasePtr >= allocatePtr, "Unexpected V phy regs are released!")

  //Allocate
  private val allocateNum = PopCount(io.needAlloc)
  private val freeEntryNum = distanceBetween(releasePtr, allocatePtr)
  io.canAccept := allocateNum <= freeEntryNum
  io.allocatePhyReg.zipWithIndex.foreach({case(a, i) =>
    if(i == 0){
      a := freeList(allocatePtr.value)
    } else {
      val addend = PopCount(io.needAlloc.take(i))
      a := freeList((allocatePtr + addend).value)
    }
  })
  //snapshot
  val snapshots = SnapshotGenerator(allocatePtr, io.snpt.snptEnq, io.snpt.snptDeq, io.redirect.valid, io.snpt.flushVec)

  private val doAlloc = io.needAlloc.map(_ && io.canAccept).reduce(_|_)


  //use snapshot to update allocatePtr
  private val allocatePtrNext=Mux(io.redirect.valid, snapshots(io.snpt.snptSelect), allocatePtr + allocateNum)
  when(doAlloc) {
    allocatePtr := allocatePtrNext
  }

  //Release
  private val releaseNum = PopCount(io.releasePhyReg.map(_.valid))
  private val doRelease = io.releasePhyReg.map(_.valid).reduce(_|_)
  when(doRelease){
    releasePtr := releasePtr + releaseNum
  }

  for((rls, i) <- io.releasePhyReg.zipWithIndex) {
    val ptr = if(i == 0) {
      releasePtr
    } else {
      releasePtr + PopCount(io.releasePhyReg.take(i).map(_.valid))
    }
    when(rls.valid) {
      freeList(ptr.value) := rls.bits
    }
  }
}
