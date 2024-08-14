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
import xiangshan.backend.rob.RobPtr
import xiangshan.vector._

class VmMemoryEntry(implicit p: Parameters) extends XSBundle {
  val robIdx = new RobPtr
  val pvm = UInt(PhyRegIdxWidth.W)
}

class VIRename(implicit p: Parameters) extends VectorBaseModule {
  val io = IO(new Bundle{
    val redirect = Flipped(ValidIO(new Redirect))
    //rename, from waitqueue
    val rename = Vec(VIRenameWidth, new Bundle {
      val in = Flipped(DecoupledIO(new MicroOp))
      val out = DecoupledIO(new MicroOp)
    })
    //commit, from ROB
    val commit = Flipped(new RobCommitIO)
    val rabCommit = Input(new RabCommitIO)
    val exception = Input(Valid(new ExceptionInfo))
    val debug = Output(Vec(32, UInt(VIPhyRegIdxWidth.W)))
  })

  val freeList        = Module(new VIFreeList)
  val renameTable     = Module(new VIRenameTable)
  //val rollBackList    = Module(new VIRollBackList)
  val vrob = Module(new VRob)
  vrob.io.redirect := io.redirect

  io.debug := renameTable.io.debug

  //-------------------------------------------- Rename --------------------------------------------
  private val stopRename = io.redirect.valid || io.commit.isWalk || RegNext(io.commit.isWalk, false.B) || vrob.io.blockRename
  io.rename.map(_.in).zip(io.rename.map(_.out)).foreach {
    case (rin, ro) =>
      rin.ready := ro.ready && freeList.io.canAccept && !stopRename
      ro.valid := rin.fire
  }

  /*  Rename
  *------------------------------------------------
  * FreeList: allocate Pdest
  * RAT: write sRAT, and read old pdest
  * RollBackList: write
  * Pvm should be the same when robIdx is the same.
  *------------------------------------------------
  */
  private val updateEntryAllocator = RegInit(0.U(3.W))
  private val updateAddrs = Wire(Vec(VIRenameWidth, UInt(3.W)))
  for((a, i) <- updateAddrs.zipWithIndex){
    if(i == 0){
      a := updateEntryAllocator
    } else {
      a := updateEntryAllocator + PopCount(io.rename.take(i).map(r => r.in.fire && r.in.bits.canRename))
    }
  }
  private val allocateNew = io.rename.map(r => r.in.fire && r.in.bits.canRename)
  private val last = io.rename.map(r => r.in.fire && r.in.bits.uopIdx === (r.in.bits.uopNum - 1.U)).reduce(_|_)
  when(last){
    updateEntryAllocator := 0.U
  }.elsewhen(allocateNew.reduce(_|_)){
    updateEntryAllocator := updateEntryAllocator + PopCount(allocateNew)
  }
  renameTable.io.doUpdate := last
  renameTable.io.redirect := io.redirect
  io.rename.map(_.out).zip(io.rename.map(_.in)).zipWithIndex.foreach {
    case ((resp, req), i) => {
      val renameEn = req.fire && req.bits.canRename && req.bits.ctrl.vdWen
      val allocPhyIdx = freeList.io.allocatePhyReg(i)
      freeList.io.needAlloc(i) := req.valid && req.bits.canRename && req.bits.ctrl.vdWen && resp.ready && !stopRename
      // reanme write rat
      renameTable.io.rename(i).lvd := req.bits.ctrl.ldest
      renameTable.io.rename(i).lvs(0) := req.bits.ctrl.lsrc(0)
      renameTable.io.rename(i).lvs(1) := req.bits.ctrl.lsrc(1)
      renameTable.io.rename(i).lvs(2) := req.bits.ctrl.ldest
      renameTable.io.rename(i).lvs(3) := 0.U

      renameTable.io.update(i).valid := renameEn
      renameTable.io.update(i).bits.addr := updateAddrs(i)
      renameTable.io.update(i).bits.data.lvd := req.bits.ctrl.ldest
      renameTable.io.update(i).bits.data.pvd := allocPhyIdx
      renameTable.io.update(i).bits.data.robIdx := req.bits.robIdx

      resp.bits := req.bits
      resp.bits.pdest := Mux(req.bits.canRename && req.bits.ctrl.vdWen, allocPhyIdx, Mux(req.bits.ctrl.vdWen, renameTable.io.rename(i).pvd, req.bits.pdest))
      resp.bits.psrc(0) := Mux(req.bits.ctrl.srcType(0) === SrcType.vec, renameTable.io.rename(i).pvs(0), req.bits.psrc(0))
      resp.bits.psrc(1) := Mux(req.bits.ctrl.srcType(1) === SrcType.vec, renameTable.io.rename(i).pvs(1), req.bits.psrc(1))
      resp.bits.psrc(2) := renameTable.io.rename(i).pvs(2)
      resp.bits.vm := renameTable.io.rename(i).pvs(3)
      resp.bits.old_pdest := DontCare
    }
  }

  // write rollbacklist
  io.rename.map(_.in).zip(vrob.io.enq).zipWithIndex.foreach {
    case ((req, rlb), i) => {
      rlb.valid := req.fire && req.bits.canRename && req.bits.ctrl.vdWen
      rlb.bits.robIdx := req.bits.robIdx
      rlb.bits.logicRegIdx := req.bits.ctrl.ldest
      rlb.bits.oldPhyRegIdx := renameTable.io.rename(i).pvs(2)
      rlb.bits.newPhyRegIdx := freeList.io.allocatePhyReg(i)
    }
  }

  //-------------------------------------------- TODO: commit & walk --------------------------------------------
  private val rollbackDelay = vrob.io.commit.rat.Pipe
  vrob.io.commit.rob <> io.commit
  vrob.io.commit.rab <> io.rabCommit
  vrob.io.exception := Pipe(io.exception)
  renameTable.io.commit := rollbackDelay

  for((rls, i) <- freeList.io.releasePhyReg.zipWithIndex) {
    rls.valid := rollbackDelay.mask(i) && (rollbackDelay.doCommit || rollbackDelay.doWalk)
    rls.bits := Mux(rollbackDelay.doCommit, rollbackDelay.prIdxOld(i), rollbackDelay.prIdxNew(i))
  }
}
