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

package xiangshan.vector

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import utils._
import freechips.rocketchip.diplomacy.{LazyModule, LazyModuleImp}
import xiangshan._
import xiangshan.backend.issue.DqDispatchNode
import xiangshan.backend.rob._
import xiangshan.vector._
import xs.utils._
import xiangshan.vector.videcode._
import xiangshan.vector.vtyperename._
import xiangshan.vector.viwaitqueue._
import xiangshan.vector.virename._
import xiangshan.vector.dispatch._



class SIRenameInfo(implicit p: Parameters) extends VectorBaseBundle  {
  val psrc = Vec(3, UInt(PhyRegIdxWidth.W))
  val pdest = UInt(PhyRegIdxWidth.W)
  val old_pdest = UInt(PhyRegIdxWidth.W)
}


class VectorCtrlBlock(implicit p: Parameters) extends LazyModule {

  lazy val module = new VICtrlImp(this)
  val dispatchNode = new DqDispatchNode

}

class VICtrlImp(outer: VectorCtrlBlock)(implicit p: Parameters) extends LazyModuleImp(outer)
  with HasVectorParameters
  with HasXSParameter
{

  val io = IO(new Bundle {
    val hartId = Input(UInt(8.W))
    val cpu_halt = Output(Bool())
    //in
    //from ctrl decode
    val in = Vec(DecodeWidth, Flipped(DecoupledIO(new CfCtrl)))
    //from ctrl rename
    val vtypein = Vec(VIDecodeWidth, Flipped(ValidIO(new VtypeReg))) //to waitqueue
    val SIRenameIn = Vec(VIDecodeWidth, Flipped(ValidIO(new SIRenameInfo)))//to waitqueue
    //from ctrl rob
    val allowdeq = Vec(VIDecodeWidth, Flipped(ValidIO(new RobPtr))) //to wait queue
    val vtypewriteback = Vec(VIDecodeWidth, Flipped(ValidIO(new ExuOutput))) //to wait queue
    val MergeIdAllocate = Vec(VIDecodeWidth, Flipped(DecoupledIO(UInt(log2Up(VectorMergeStationDepth).W)))) //to wait queue
    val commit = new VIRobIdxQueueEnqIO // to rename
    val redirect = Flipped(ValidIO(new Redirect))
    //from csr vstart
    val vstart = Input(UInt(7.W))

    //out
    //to exu
    val vecDispatch2Rs = Vec(vecDispatch._2.bankNum * 2, DecoupledIO(new MicroOp))

    //to sictrlblock
    val vecDispatch2Ctrl = Vec(vecDispatch._2.bankNum, DecoupledIO(new MicroOp))

  })

  private val vecDispatch = outer.dispatchNode.out.filter(_._2._1.isVecRs).map(e => (e._1, e._2._1)).head
  private val vecDeq = vecDispatch._1

  val videcode = Module(new VIDecodeUnit)
  val waitqueue = Module(new VIWaitQueue)
  val virename = Module(new VIRenameWrapper)
  val dispatch = Module(new VectorDispatchWrapper(vecDispatch._2.bankNum))

  for (i <- 0 until VIDecodeWidth) {
    val DecodePipe = PipelineNext(io.in(i), videcode.io.in(i).ready,
      io.redirect.valid)
    DecodePipe.ready := videcode.io.in(i).ready
    videcode.io.in(i).valid := DecodePipe.valid
    videcode.io.in(i).bits := DecodePipe.bits
  }

  videcode.io.canOut := waitqueue.io.enq.canAccept
  for (i <- 0 until VIDecodeWidth) {
    when(io.vtypein(i).valid && videcode.io.out(i).valid && io.SIRenameIn(i).valid) {
      waitqueue.io.enq.req(i).valid := videcode.io.out(i).valid
      waitqueue.io.enq.needAlloc(i) := videcode.io.out(i).valid
      val CurrentData = Wire(new VIMop)
      CurrentData.MicroOp <> videcode.io.out(i).bits
      CurrentData.MicroOp.pdest <> io.SIRenameIn(i).bits.pdest
      CurrentData.MicroOp.psrc <> io.SIRenameIn(i).bits.psrc
      CurrentData.MicroOp.old_pdest <> io.SIRenameIn(i).bits.old_pdest
      CurrentData.MicroOp.vCsrInfo <> io.vtypein(i).bits.vCsrInfo
      CurrentData.MicroOp.robIdx := io.vtypein(i).bits.robIdx
      CurrentData.state := io.vtypein(i).bits.state
      waitqueue.io.enq.req(i).bits := CurrentData
    }
  }

  waitqueue.io.vstart <> io.vstart
  waitqueue.io.vtypeWbData <> io.vtypewriteback
  waitqueue.io.robin <> io.allowdeq
  waitqueue.io.MergeId <> io.MergeIdAllocate
  waitqueue.io.canRename <> virename.io.canAccept
  waitqueue.io.redirect <> io.redirect

  virename.io.uopIn <> waitqueue.io.out
  virename.io.redirect <> io.redirect
  virename.io.commit <> io.commit
  virename.io.redirect <> io.redirect

  dispatch.io.req.uop := virename.io.uopOut
  for((rp, dp) <- virename.io.uopOut zip dispatch.io.req.uop) {
    rp.ready := dispatch.io.req.canDispatch
    dp := rp.bits
  }
  dispatch.io.req.mask := virename.io.uopOut.map(_.valid).asUInt
  
  dispatch.io.redirect <> io.redirect

  io.vecDispatch2Ctrl <>  dispatch.io.toMem2RS

  vecDeq <> dispatch.io.toVectorPermuRS ++ dispatch.io.toVectorCommonRS

}
