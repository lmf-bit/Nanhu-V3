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
    Date: 2023-08-23
    email: guanmingxing@bosc.ac.cn
---------------------------------------------------------------------------------------*/

package xiangshan.backend.rob

import chisel3.{UInt, _}
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy.{LazyModule, LazyModuleImp}
import difftest._
import utils._
import xs.utils._
import xiangshan._
import xiangshan.frontend.FtqPtr
import xiangshan.backend.execute.exu.{ExuConfig, ExuType}
import xiangshan.backend.writeback._
import xiangshan.vector._

class RobPtr(implicit p: Parameters) extends CircularQueuePtr[RobPtr](
  p => p(XSCoreParamsKey).RobSize
) with HasCircularQueuePtrHelper {

  def needFlush(redirect: Valid[Redirect]): Bool = {
    val flushItself = redirect.bits.flushItself() && this === redirect.bits.robIdx
    redirect.valid && (flushItself || isAfter(this, redirect.bits.robIdx))
  }

  def needFlush(redirect: Seq[Valid[Redirect]]): Bool = VecInit(redirect.map(needFlush)).asUInt.orR
}

object RobPtr {
  def apply(f: Bool, v: UInt)(implicit p: Parameters): RobPtr = {
    val ptr = Wire(new RobPtr)
    ptr.flag := f
    ptr.value := v
    ptr
  }
}

class RobCSRIO(implicit p: Parameters) extends XSBundle {
  val intrBitSet = Input(Bool())
  val wfiEvent   = Input(Bool())

  val fflags     = Output(Valid(UInt(5.W)))
  val vstart     = Output(Valid(UInt(7.W)))
  val vxsat      = Output(ValidIO(Bool()))
  val dirty_fs   = Output(Bool())
  val dirty_vs   = Output(Bool())
  val perfinfo   = new Bundle {
    val retiredInstr = Output(UInt(3.W))
  }
}

class RobLsqIO(implicit p: Parameters) extends XSBundle {
  val lcommit = Output(UInt(log2Up(CommitWidth + 1).W))
  val scommit = Output(UInt(log2Up(CommitWidth + 1).W))
  val pendingld = Output(Bool())
  val pendingst = Output(Bool())
  val commit = Output(Bool())
  val pendingOrdered = Output(Bool())
  val pendingInst = Output(new RobPtr)
  val lqSafeDeq = Output(new RobPtr)
  val debug_deqPtr = Output(new RobPtr)
  val debug_enqPtr = Output(new RobPtr)
}

class RobEnqIO(implicit p: Parameters) extends XSBundle {
  val canAccept = Output(Bool())
  val canAccept_dup = Vec(RenameWidth - 1, Output(Bool()))
  val isEmpty = Output(Bool())
  // valid vector, for robIdx gen and walk
  val needAlloc = Vec(RenameWidth, Input(Bool()))
  val req = Vec(RenameWidth, Flipped(ValidIO(new MicroOp)))
  val resp = Vec(RenameWidth, Output(new RobPtr))
}

class RobFlushInfo(implicit p: Parameters) extends XSBundle {
  val ftqIdx = new FtqPtr
  val robIdx = new RobPtr
  val ftqOffset = UInt(log2Up(PredictWidth).W)
  val replayInst = Bool()
}

class RblEnqIO(implicit p: Parameters) extends XSBundle {
  val canAccept = Output(Bool())
  val isEmpty = Output(Bool())
  val needAlloc = Vec(RenameWidth, Input(Bool()))
  val req = Vec(RenameWidth, Flipped(ValidIO(new MicroOp)))
}

class RobToRblIO(implicit p: Parameters) extends XSBundle {
  val commitValid = Input(Bool())
  val commitSize = Input(UInt(log2Up(RblSize).W))
  val walkValid = Input(Bool())
  val walkSize = Input(UInt(log2Up(RblSize).W))
  val robwalkEnd = Input(Bool())
}
