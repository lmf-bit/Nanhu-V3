package xiangshan.backend.execute.exucx

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy.LazyModule
import xiangshan.backend.execute.exu.{AluExu, StdExu}
import xiangshan.{ExuOutput, FuType}
import xiangshan.backend.execute.exu._
import xiangshan.backend.execute.fu._

class AluStdComplex(id: Int, bypassNum: Int)(implicit p: Parameters) extends BasicExuComplex {
  private val cfg_std  = ExuConfig(
    name = "StdExu",
    id = id,
    complexName = "AluStdComplex",
    fuConfigs = Seq(FuConfigs.stdiCfg),
    exuType = ExuType.stdi,
    writebackToRob = true,
    writebackToVms = false
  )
  val alu = LazyModule(new AluExu(id, "AluStdComplex", bypassNum))
  val std = LazyModule(new StdExu(id, "AluStdComplex", bypassNum, cfg_std))
  alu.issueNode :*= issueNode
  writebackNode :=* alu.writebackNode

  std.issueNode :*= issueNode
  writebackNode :=* std.writebackNode

  lazy val module = new AluStdCxImp(this, id, bypassNum)
}
class AluStdCxImp(outer:AluStdComplex, id:Int, bypassNum:Int) extends BasicExuComplexImp(outer, bypassNum){

  val io = IO(new Bundle {
    val writebackToSQ = DecoupledIO(new ExuOutput)
  })

  private val issueIn = outer.issueNode.in.head._1
  private val issueRouted = outer.issueNode.out.map(_._1)
  issueRouted.foreach(_ <> issueIn)

  outer.alu.module.io.bypassIn := bypassIn
  outer.alu.module.redirectIn := redirectIn

  outer.std.module.io.bypassIn := bypassIn
  outer.std.module.redirectIn := redirectIn
  io.writebackToSQ <> outer.std.module.io.writebackToSQ

  private val issueFuHit = outer.issueNode.in.head._2._2.exuConfigs.flatMap(_.fuConfigs).map(_.fuType === issueIn.issue.bits.uop.ctrl.fuType).reduce(_ | _)
  when(issueIn.issue.valid) {
    assert(issueFuHit)
  }
}
