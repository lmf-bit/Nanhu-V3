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
package xiangshan.frontend

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan._
import utils._
import xiangshan.cache.mmu.CAMTemplate
import xs.utils.CircularQueuePtr
import xs.utils.perf.HasPerfLogging

class WrBypass[T <: Data](gen: T, val numEntries: Int, val idxWidth: Int,
val numWays: Int = 1, val tagWidth: Int = 0)(implicit p: Parameters) 
extends XSModule with HasPerfLogging {

  require(numEntries >= 0)
  require(idxWidth > 0)
  require(numWays >= 1)
  require(tagWidth >= 0)

  def hasTag = tagWidth > 0
  def multipleWays = numWays > 1

  val io = IO(new Bundle {
    val wen          = Input(Bool())
    val writeIdx     = Input(UInt(idxWidth.W))
    val writeTag     = if (hasTag) Some(Input(UInt(tagWidth.W))) else None
    val writeData    = Input(Vec(numWays, gen))
    val writeWayMask = if (multipleWays) Some(Input(Vec(numWays, Bool()))) else None

    val hit     = Output(Bool())
    val hitData = Vec(numWays, Valid(gen))
  })

  class WrBypassPtr extends CircularQueuePtr[WrBypassPtr](numEntries)
  class IdxTag extends Bundle {
    val idx = UInt(idxWidth.W)
    val tag = if (hasTag) Some(UInt(tagWidth.W)) else None
    def apply(idx: UInt, tag: UInt) = {
      this.idx := idx
      this.tag.map(_ := tag)
    }
  }

  val idxTagCam   = Module(new CAMTemplate(new IdxTag, numEntries, 1))
  val dataMem     = Mem(numEntries, Vec(numWays, gen))
  val valids      = RegInit(0.U.asTypeOf(Vec(numEntries, Vec(numWays, Bool()))))
  val everWritten = RegInit(0.U.asTypeOf(Vec(numEntries, Bool())))

  // read cam
  idxTagCam.io.r.req(0)(io.writeIdx, io.writeTag.getOrElse(0.U))
  val hitsOH = idxTagCam.io.r.resp(0).zip(everWritten).map {case (resp, ew) => resp && ew}
  val hitIdx = OHToUInt(hitsOH)
  val isHit  = hitsOH.reduce(_||_)
  // wrbypass resp
  io.hit := isHit
  for (i <- 0 until numWays) {
    io.hitData(i).valid := Mux1H(hitsOH, valids)(i)
    io.hitData(i).bits  := dataMem.read(hitIdx)(i)
  }

  // write
  val enqPtr = RegInit(0.U.asTypeOf(new WrBypassPtr))
  val enqIdx = enqPtr.value
  val camWriteEna = io.wen && !isHit
  enqPtr := enqPtr + camWriteEna
  // cam
  idxTagCam.io.w.valid      := camWriteEna
  idxTagCam.io.w.bits.index := enqIdx
  idxTagCam.io.w.bits.data(io.writeIdx, io.writeTag.getOrElse(0.U))
  // dataMem
  val fullMask = Fill(numWays, 1.U(1.W)).asTypeOf(Vec(numWays, Bool()))
  val writeDataMemWayMask = io.writeWayMask.getOrElse(fullMask)
  when(io.wen) {
    val writeDataMemIdx = Mux(isHit, hitIdx, enqIdx)
    dataMem.write(writeDataMemIdx, io.writeData, writeDataMemWayMask)
  }
  // valids, everWritten
  for (i <- 0 until numWays) {
    when(io.wen) {
      when(isHit) {
        when(writeDataMemWayMask(i)) {
          valids(hitIdx)(i) := true.B
        }
      }.otherwise {
        everWritten(enqIdx) := true.B
        valids(enqIdx)(i)   := false.B
        when(writeDataMemWayMask(i)) {
          valids(enqIdx)(i) := true.B
        }
      }
    }
  }

  // perf counters
  XSPerfAccumulate("wrbypass_hit",  io.wen &&  isHit)
  XSPerfAccumulate("wrbypass_miss", io.wen && !isHit)

  XSDebug(io.wen && isHit,  p"wrbypass hit entry #${hitIdx}, idx ${io.writeIdx}" +
    p"tag ${io.writeTag.getOrElse(0.U)}data ${io.writeData}\n")
  XSDebug(io.wen && !isHit, p"wrbypass enq entry #${enqIdx}, idx ${io.writeIdx}" +
    p"tag ${io.writeTag.getOrElse(0.U)}data ${io.writeData}\n")

}