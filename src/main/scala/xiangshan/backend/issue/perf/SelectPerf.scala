package xiangshan.backend.issue.perf

import chisel3._
import org.chipsalliance.cde.config.Parameters
import chisel3.util._
import xiangshan.backend.issue._
import xiangshan.backend.rob.RobPtr
import xs.utils.perf.HasPerfLogging
import xiangshan.backend.issue.perf._
import xiangshan._

class SelectPerf(entryNumPerBank: Int, issueWidth: Int)(implicit p: Parameters) extends XSModule with HasPerfLogging {
  val io = IO(new Bundle {
    val selectInfo = Input(Vec(4, Vec(entryNumPerBank, Valid(new RobPtr))))
    val finalSelectInfo = Input(Vec(issueWidth, Valid(new RobPtr)))
  })
  
  val shouldSelVec = Wire(Vec(issueWidth, Valid(new RobPtr)))
  val selectInfoVec = io.selectInfo(0) ++ io.selectInfo(1) ++ io.selectInfo(2) ++ io.selectInfo(3)
  println(s"selectInfoVec's length is ${selectInfoVec.length}")

  val ageMatrix = Wire(Vec(selectInfoVec.length, Vec(selectInfoVec.length, Bool())))
  val entryNum = selectInfoVec.length
  for(i <- 0 until entryNum) {
    for(j <- 0 until entryNum) {
      ageMatrix(i)(j) := (selectInfoVec(i).valid && !selectInfoVec(j).valid) || (selectInfoVec(i).valid && selectInfoVec(j).valid && selectInfoVec(i).bits <= selectInfoVec(j).bits)
    }
  }
  shouldSelVec.zipWithIndex.foreach {
    case (sel, i) => {
      val hitVec = Wire(Vec(entryNum, Bool()))
      for(i <- 0 until entryNum) {
        hitVec(i) := PopCount(ageMatrix(i)) === entryNum.U - i.U
      }
      sel.bits := Mux1H(hitVec, selectInfoVec.map(_.bits))
      sel.valid := PopCount(hitVec) === 1.U
    }
  }

  val selOldest = Wire(Vec(issueWidth, Bool()))
  selOldest.zip(io.finalSelectInfo).foreach {
    case (sel, fsel) => {
      sel := shouldSelVec.map(s => s.valid && fsel.valid && s.bits === fsel.bits).reduce(_||_)
    }
  }
  for(i <- 0 to issueWidth) {
    XSPerfAccumulate(s"iss_${i}", PopCount(selOldest) === i.U)
  }
}
