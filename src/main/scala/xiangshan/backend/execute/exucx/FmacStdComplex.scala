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
/***************************************************************************************
 * Author: Liang Sen
 * E-mail: liangsen20z@ict.ac.cn
 * Date: 2023-06-19
 ****************************************************************************************/
package xiangshan.backend.execute.exucx

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy.LazyModule
import xiangshan.backend.execute.exu.{FmacExu, StdExu}
import xiangshan.ExuOutput
import xiangshan.backend.execute.exu._
import xiangshan.backend.execute.fu._

class FmacStdComplex(id: Int)(implicit p:Parameters) extends BasicExuComplex{
  private val cfg_std  = ExuConfig(
    name = "StdExu",
    id = id,
    complexName = "FmacStdComplex",
    fuConfigs = Seq(FuConfigs.stdfCfg),
    exuType = ExuType.stdf,
    writebackToRob = true,
    writebackToVms = false
  )
  val fmac = LazyModule(new FmacExu(id,"FmacStdComplex"))
  val fstd = LazyModule(new StdExu(id, "FmacStdComplex", 0, cfg_std))
  fmac.issueNode :*= issueNode
  writebackNode :=* fmac.writebackNode

  fstd.issueNode :*= issueNode
  writebackNode :=* fstd.writebackNode
  lazy val module = new FmacStdCxIpm(this)
}
class FmacStdCxIpm(outer:FmacStdComplex)(implicit p:Parameters) extends BasicExuComplexImp(outer, 0){
  val csr_frm: UInt = IO(Input(UInt(3.W)))
  val io = IO(new Bundle {
    val writebackToSQ = DecoupledIO(new ExuOutput)
  })
  private val issueIn = outer.issueNode.in.head._1
  private val issueRouted = outer.issueNode.out.map(_._1)
  issueRouted.foreach(_ <> issueIn)
  outer.fmac.module.redirectIn := redirectIn
  outer.fmac.module.csr_frm := csr_frm

  outer.fstd.module.redirectIn := redirectIn
  io.writebackToSQ <> outer.fstd.module.io.writebackToSQ
}
