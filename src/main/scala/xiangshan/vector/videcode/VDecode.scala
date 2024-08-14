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

package xiangshan.vector.videcode

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util.{BitPat, _}
import freechips.rocketchip.rocket.Instructions
import freechips.rocketchip.util.uintToBitPat
import xs.utils.SignExt
import xiangshan._
import xiangshan.vector._
import freechips.rocketchip.rocket.Instructions._
/**
 * IO bundle for the Decode unit
 */
class VDecodeIO(implicit p: Parameters) extends VectorBaseBundle {
  val in = Vec(VIDecodeWidth, Flipped(DecoupledIO(new MicroOp)))
  // to Rename
  val out = Vec(VIDecodeWidth, ValidIO(new MicroOp))
  val canOut = Input(Bool())
}

/**
 * Decode unit that takes in a single CtrlFlow and generates a MicroOp.
 */
class VDecode(implicit p: Parameters) extends VectorBaseModule with VDecodeUnitConstants {
  val io = IO(new VDecodeIO)
  for ( i <- 0 until VIDecodeWidth ) {
    io.in(i).ready := io.canOut
    val decoder = Module(new VDecodeUnit)
    decoder.io.in := io.in(i).bits
    io.out(i).bits := decoder.io.out
    io.out(i).valid := io.in(i).valid
  }
}