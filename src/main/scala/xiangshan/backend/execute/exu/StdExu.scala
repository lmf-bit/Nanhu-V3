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

package xiangshan.backend.execute.exu

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan._
import xiangshan.backend.execute.fu._
import xiangshan.backend.execute.exublock.Std

class StdExu(id: Int, complexName: String, val bypassInNum: Int, val cfg: ExuConfig)(implicit p:Parameters) extends BasicExu {
  // private val cfg  = ExuConfig(
  //   name = "StdExu",
  //   id = id,
  //   complexName = complexName,
  //   fuConfigs = Seq(FuConfigs.stdCfg),
  //   exuType = ExuType.std,
  //   writebackToRob = true,
  //   writebackToVms = false
  // )
  val issueNode = new ExuInputNode(cfg)
  val writebackNode = new ExuOutputNode(cfg)

  lazy val module = new StdExuImpl(this, cfg)
}

class StdExuImpl(outer:StdExu, exuCfg:ExuConfig)(implicit p:Parameters) extends BasicExuImpl(outer){
  val io = IO(new Bundle{
    val bypassIn = Input(Vec(outer.bypassInNum, Valid(new ExuOutput)))
    val writebackToSQ = DecoupledIO(new ExuOutput)
  })
  private val issuePort = outer.issueNode.in.head._1
  private val writebackPort = outer.writebackNode.out.head._1

  issuePort.issue.ready := true.B
  private val finalIssueSignals = bypassSigGen(io.bypassIn, issuePort, outer.bypassInNum > 0)

  private val std = Module(new Std)
  std.io.in.valid := issuePort.issue.valid && (issuePort.issue.bits.uop.ctrl.fuType === FuType.std)
  std.io.in.bits := issuePort.issue.bits
  issuePort.issue.ready := true.B

  std.io.redirect := redirectIn
  writebackPort.valid <> std.io.out.valid
  writebackPort.bits := std.io.out.bits

  io.writebackToSQ <> std.io.out
}

