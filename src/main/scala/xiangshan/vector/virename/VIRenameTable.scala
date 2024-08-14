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

---------------------------------------------------------------------------------------*/

package xiangshan.vector.virename

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters

import xiangshan._
import utils._
import xs.utils.CircularShift

import xiangshan.vector._
import xiangshan.backend.rob.RobPtr

class RatUpdateEntry(implicit p: Parameters) extends VectorBaseBundle {
  val robIdx = new RobPtr
  val lvd = UInt(5.W)
  val pvd = UInt(VIPhyRegIdxWidth.W)
}
class VIRatRenameIO(implicit p: Parameters) extends VectorBaseBundle {
  val lvd = Input(UInt(5.W))
  val lvs = Input(Vec(4, UInt(5.W)))
  val pvd = Output(UInt(VIPhyRegIdxWidth.W))
  val pvs = Output(Vec(4, UInt(VIPhyRegIdxWidth.W)))
}

class VIRatCommitPort(implicit p: Parameters) extends VectorBaseBundle {
  val doCommit    = Bool()
  val doWalk      = Bool()
  val mask        = Vec(8, Bool())
  val lrIdx       = Vec(8, UInt(5.W))
  val prIdxOld    = Vec(8, UInt(VIPhyRegIdxWidth.W))
  val prIdxNew    = Vec(8, UInt(VIPhyRegIdxWidth.W))

  def Pipe:VIRatCommitPort = {
    val pipe = Wire(new VIRatCommitPort)
    pipe.doCommit := RegNext(this.doCommit, false.B)
    pipe.doWalk := RegNext(this.doWalk, false.B)
    for(i <- 0 until 8){
      pipe.mask(i) := RegNext(this.mask(i), false.B)
      pipe.lrIdx(i) := RegEnable(this.lrIdx(i), this.mask(i))
      pipe.prIdxOld(i) := RegEnable(this.prIdxOld(i), this.mask(i))
      pipe.prIdxNew(i) := RegEnable(this.prIdxNew(i), this.mask(i))
    }
    pipe
  }
}

class VIRenameTable(implicit p: Parameters) extends VectorBaseModule {
  val io = IO(new Bundle{
    val rename = Vec(VIRenameWidth, new VIRatRenameIO)
    val update = Input(Vec(VIRenameWidth, Valid(new Bundle {
      val addr = UInt(3.W)
      val data = new RatUpdateEntry
    })))
    val doUpdate = Input(Bool())
    val commit      = Input(new VIRatCommitPort)
    val redirect = Flipped(ValidIO(new Redirect))
    val debug  = Output(Vec(32, UInt(VIPhyRegIdxWidth.W))) //for difftest
  })
  //RAT
  private val rat_ds = VecInit.tabulate(32)(i => i.U(VIPhyRegIdxWidth.W))
  private val sRAT = RegInit(rat_ds)
  private val aRAT = RegInit(rat_ds)
  private val updateEntryValid = RegInit(VecInit(Seq.fill(8)(false.B)))
  private val updateEntryBits = Reg(Vec(8, new RatUpdateEntry))
  private val updateEntryValidNext = WireInit(updateEntryValid)
  private val updateEntryBitsNext = WireInit(updateEntryBits)

  io.debug := aRAT

  /***********************************************************************************
   * Vector Rename take uops belong to the same instruction at the same cycle.
   * Every vector uop in the same instruction sees a same RAT, no need to bypass.
   * For uops that do not allocate new reg, used the same pdest of matching ldest in the update regs.
   * All RAT updates within one vector instruction will store in update regs.
   * Updates will be postponed until when all uops of one instruction have been renamed.
   ************************************************************************************/

  for(r <- io.rename){
    val pvdSelFromUpdate = updateEntryValidNext.zip(updateEntryBitsNext).map(e => {e._1 && e._2.lvd === r.lvd})
    val pvdFromUpdate = Mux1H(pvdSelFromUpdate, updateEntryBitsNext.map(_.pvd))
    val pvdFromUpdateHit = pvdSelFromUpdate.reduce(_|_)
    r.pvd := Mux(pvdFromUpdateHit, pvdFromUpdate, sRAT(r.lvd))
    for((p, l) <- r.pvs.zip(r.lvs)){
      p := sRAT(l)
    }
  }
  private def UpdateWrite(validVec:Vec[Bool], bitsVec:Vec[RatUpdateEntry]):Unit = {
    for (((vn, bn), idx) <- validVec.zip(bitsVec).zipWithIndex) {
      val wSel = io.update.map(u => u.valid && u.bits.addr === idx.U)
      val wData = Mux1H(wSel, io.update.map(_.bits.data))
      assert(PopCount(wSel) < 2.U, "There should be no two hits in the same period")
      val wHit = wSel.reduce(_ | _)
      when(wHit){
        vn := true.B
        bn := wData
      }
    }
  }
  UpdateWrite(updateEntryValid, updateEntryBits)
  UpdateWrite(updateEntryValidNext, updateEntryBitsNext)

  updateEntryBits.zip(updateEntryValid).zipWithIndex.foreach {
    case ((e, v), i) => {
      when(io.redirect.valid && e.robIdx.needFlush(io.redirect) || io.doUpdate) {
        v := false.B
      }
    }
  }

  //Update and walk
  for((e, idx) <- sRAT.zipWithIndex){
    val uSel = updateEntryValidNext.zip(updateEntryBitsNext).map(e => e._1 && e._2.lvd === idx.U)
    val uData = Mux1H(uSel, updateEntryBitsNext.map(_.pvd))
    val uHit = uSel.reduce(_ | _)
    when(uHit && io.doUpdate){
      e := uData
    }
  }
  //Walk write has priority: write with bigger idx will overwrite ones with smaller.
  for (i <- 0 until 8) {
    when(io.commit.doWalk && io.commit.mask(i)) {
      sRAT(io.commit.lrIdx(i)) := io.commit.prIdxNew(i)
    }
  }
  //Commit write has priority: write with bigger idx will overwrite ones with smaller.
  for(i <- 0 until 8){
    when(io.commit.doCommit && io.commit.mask(i)){
      aRAT(io.commit.lrIdx(i)) := io.commit.prIdxNew(i)
    }
  }
}
