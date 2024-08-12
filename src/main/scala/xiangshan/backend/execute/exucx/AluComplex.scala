package xiangshan.backend.execute.exucx

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util.Valid
import freechips.rocketchip.diplomacy.LazyModule
import xiangshan.backend.execute.exu.{AluExu, ExuType, MulExu}
import xiangshan.{ExuOutput, FuType}

class AluComplex(id: Int, bypassNum: Int)(implicit p: Parameters) extends BasicExuComplex{
  val alu = LazyModule(new AluExu(id, "AluComplex", bypassNum))
  alu.issueNode :*= issueNode
  writebackNode :=* alu.writebackNode

  lazy val module = new AluCxImp(this, id, bypassNum)
}
class AluCxImp(outer:AluComplex, id:Int, bypassNum:Int) extends BasicExuComplexImp(outer, bypassNum){
  private val issueIn = outer.issueNode.in.head._1
  private val issueOut = outer.issueNode.out.head._1

  issueOut <> issueIn
  outer.alu.module.io.bypassIn := bypassIn
  outer.alu.module.redirectIn := redirectIn

  issueIn.issue.ready := issueOut.issue.ready
  private val issueFuHit = outer.issueNode.in.head._2._2.exuConfigs.flatMap(_.fuConfigs).map(_.fuType === issueIn.issue.bits.uop.ctrl.fuType).reduce(_ | _)
  when(issueIn.issue.valid) {
    assert(issueFuHit)
  }
}
