package xiangshan.backend.issue
import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan.backend.execute.exu.ExuConfig
import xiangshan.backend.rob.RobPtr
import xiangshan.{Redirect, XSBundle, XSModule}
import xs.utils.perf.HasPerfLogging
import xs.utils.{LogicShiftRight, ParallelOperationN}

class HybridSelectInfo(implicit p: Parameters) extends XSBundle {
  val robPtr = new RobPtr
  val lpv = Vec(loadUnitNum, UInt(LpvLength.W))
}

class PrioritySelectPolicy(inputNum:Int)(implicit p: Parameters) extends XSModule {
  val io = IO(new Bundle {
    val in = Input(Vec(inputNum, Valid(new HybridSelectInfo)))
    val out = Output(Valid(UInt(inputNum.W)))
  })
  private val validMask = Cat(io.in.map(_.valid).reverse)
  io.out.valid := validMask.orR
  io.out.bits := PriorityEncoderOH(validMask)
}

class OldestSelectPolicy(inputNum:Int, haveEqual:Boolean)(implicit p: Parameters) extends XSModule {
  val io = IO(new Bundle {
    val in = Input(Vec(inputNum, Valid(new HybridSelectInfo)))
    val out = Output(Valid(UInt(inputNum.W)))
  })
  private def ReductionFunc(in: Seq[(Valid[HybridSelectInfo], UInt)]):(Valid[HybridSelectInfo], UInt) = {
    val selectPolicy = Module(new SelectPolicy(in.length, true, haveEqual))
    val interRes = Wire(Valid(new HybridSelectInfo))
    selectPolicy.io.in.zip(in).foreach({case(a, b) =>
      a.valid := b._1.valid
      a.bits := b._1.bits.robPtr
    })
    interRes.valid := selectPolicy.io.out.valid
    interRes.bits := Mux1H(selectPolicy.io.out.bits, in.map(_._1.bits))
    val idx = Mux1H(selectPolicy.io.out.bits, in.map(_._2))
    (interRes, idx)
  }

  private val res = ParallelOperationN(io.in.zipWithIndex.map(in => (in._1, (1L << in._2).U(inputNum.W))), 8, ReductionFunc)
  io.out.valid := res._1.valid
  io.out.bits := res._2
}

class HybridSelectNetwork(bankNum:Int, entryNum:Int, issueNum:Int, val cfg:ExuConfig, haveEqual:Boolean, name:Option[String] = None)(implicit p: Parameters)
  extends XSModule with HasPerfLogging {
  require(issueNum <= bankNum && 0 < issueNum && bankNum % issueNum == 0, "Illegal number of issue ports are supported now!")
  private val fuTypeList = cfg.fuConfigs.map(_.fuType)
  val io = IO(new Bundle {
    val redirect = Input(Valid(new Redirect))
    val selectInfo = Input(Vec(bankNum, Vec(entryNum, Valid(new SelectInfo))))
    val issueInfo = Vec(issueNum, Decoupled(new SelectResp(bankNum, entryNum)))
    val earlyWakeUpCancel = Input(Vec(loadUnitNum, Bool()))
  })
  override val desiredName: String = name.getOrElse("SelectNetwork")
  private val finalSelectResult = Wire(Vec(issueNum, Valid(new SelectResp(bankNum, entryNum))))

  private val selectInputPerBank = io.selectInfo.zipWithIndex.map({ case (si, bidx) =>
    si.zipWithIndex.map({ case (in, eidx) =>
      val selInfo = Wire(Valid(new SelectResp(bankNum, entryNum)))
      selInfo.valid := in.valid && fuTypeList.map(_ === in.bits.fuType).reduce(_ | _)
      selInfo.bits.info := in.bits
      selInfo.bits.bankIdxOH := (1 << bidx).U(bankNum.W)
      selInfo.bits.entryIdxOH := (1 << eidx).U(entryNum.W)
      selInfo
    })
  })
  private val selLen = bankNum / issueNum
  private val selectInputPerIssue = selectInputPerBank.grouped(selLen).map(_.flatten).toSeq
  for(i <- 0 until issueNum) {
    val inSeq = selectInputPerIssue(i)
    val iss = finalSelectResult(i)
    val pSelector = Module(new PrioritySelectPolicy(inSeq.length))
    val oSelector = Module(new OldestSelectPolicy(inSeq.length, haveEqual))
    val pSel = pSelector.io.out
    val oSel = Wire(Valid(UInt(inSeq.length.W)))
    val oSelV = RegNext(oSelector.io.out.valid, false.B)
    val oSelB = RegEnable(oSelector.io.out.bits, oSelector.io.out.valid)
    oSel.valid := oSelV & (oSelB & Cat(inSeq.map(_.valid).reverse)).orR
    oSel.bits := oSelB
    val selOH = Mux(oSel.valid, oSel.bits, pSel.bits)

    pSelector.io.in.zip(inSeq).foreach({case(a, b) =>
      a.valid := b.valid
      a.bits.robPtr := b.bits.info.robPtr
      a.bits.lpv := b.bits.info.lpv
    })
    oSelector.io.in.zip(inSeq).zip(selOH.asBools).foreach({ case ((a, b), s) =>
      a.valid := b.valid
      a.bits.robPtr := b.bits.info.robPtr
      a.bits.lpv := b.bits.info.lpv
    })
    finalSelectResult(i).valid := pSel.valid | oSel.valid
    finalSelectResult(i).bits := Mux1H(selOH, inSeq.map(_.bits))
    XSPerfAccumulate(s"sel_${i}_oldest", oSel.valid)
    XSPerfAccumulate(s"sel_${i}_regular", pSel.valid & !oSel.valid)
  }

  for (((outPort, driver), idx) <- io.issueInfo.zip(finalSelectResult).zipWithIndex) {
    val cancelCond = driver.bits.info.lpv.zip(io.earlyWakeUpCancel).map({case(l, c) => l(0) & c}).reduce(_|_)
    outPort.valid := driver.valid & !cancelCond & !io.redirect.valid
    outPort.bits.bankIdxOH := driver.bits.bankIdxOH
    outPort.bits.entryIdxOH := driver.bits.entryIdxOH
    outPort.bits.info := driver.bits.info
    outPort.bits.info.lpv.zip(driver.bits.info.lpv).foreach({ case (o, i) => o := LogicShiftRight(i, 1)})
    XSPerfAccumulate(s"sel_${idx}_lpv_sel", driver.valid & Cat(driver.bits.info.lpv).orR)
    XSPerfAccumulate(s"sel_${idx}_cancelled", driver.valid & cancelCond)
  }
}


class OldestSelectNetwork_TMP(bankNum:Int, entryNum:Int, issueNum:Int, val cfg:ExuConfig, haveEqual:Boolean, name:Option[String] = None)(implicit p: Parameters)
  extends XSModule with HasPerfLogging {
  require(issueNum <= bankNum && 0 < issueNum && bankNum % issueNum == 0, "Illegal number of issue ports are supported now!")
  require(issueNum == 2)  //tmp
  private val fuTypeList = cfg.fuConfigs.map(_.fuType)
  val io = IO(new Bundle {
    val redirect = Input(Valid(new Redirect))
    val selectInfo = Input(Vec(bankNum, Vec(entryNum, Valid(new SelectInfo))))
    val issueInfo = Vec(issueNum, Decoupled(new SelectResp(bankNum, entryNum)))
    val earlyWakeUpCancel = Input(Vec(loadUnitNum, Bool()))
  })
  override val desiredName: String = name.getOrElse("SelectNetwork")
  private val finalSelectResult = Wire(Vec(issueNum, Valid(new SelectResp(bankNum, entryNum))))
  private val selectMask = Wire(Vec(bankNum * entryNum, Bool()))

  private val selectInputPerBank = io.selectInfo.zipWithIndex.map({ case (si, bidx) =>
    si.zipWithIndex.map({ case (in, eidx) =>
      val selInfo = Wire(Valid(new SelectResp(bankNum, entryNum)))
      selInfo.valid := in.valid && fuTypeList.map(_ === in.bits.fuType).reduce(_ | _)
      selInfo.bits.info := in.bits
      selInfo.bits.bankIdxOH := (1 << bidx).U(bankNum.W)
      selInfo.bits.entryIdxOH := (1 << eidx).U(entryNum.W)
      selInfo
    })
  })
  private val selLen = bankNum / issueNum
  private val inSeq = selectInputPerBank.flatten

  //oldest
//    val iss = finalSelectResult(i)
  val oSelector = Module(new OldestSelectPolicy(inSeq.length, haveEqual))
  oSelector.io.in.zip(inSeq).zipWithIndex.foreach({ case((in,info),idx) => {
    in.valid := info.valid && !io.redirect.valid
    in.bits.robPtr := info.bits.info.robPtr
    in.bits.lpv := info.bits.info.lpv
  }})

  val oldestValid = oSelector.io.out.valid
  val oldestRes = oSelector.io.out.bits

  //second oldest
  val secondOSelector = Module(new OldestSelectPolicy(inSeq.length, haveEqual))
  secondOSelector.io.in.zip(inSeq).zipWithIndex.foreach({ case((in,info),idx) => {
    in.valid := info.valid && !(oldestValid && oldestRes(idx)) && !io.redirect.valid
    in.bits.robPtr := info.bits.info.robPtr
    in.bits.lpv := info.bits.info.lpv
  }})

  val secondOldestValid = secondOSelector.io.out.valid
  val secondOldestRes = secondOSelector.io.out.bits


//  val oldestValidRegOut = RegNext(oldestValid, false.B)
//  val oldestBitsRegOut = RegEnable(oldestRes, oldestValid)
//
//  val secondOldestValidRegOut = RegNext(secondOldestValid, false.B)
//  val secondOldestBitsRegOut = RegEnable(secondOldestRes, secondOldestValid)

  val oldestValidRegOut = oldestValid
  val oldestBitsRegOut = oldestRes

  val secondOldestValidRegOut = secondOldestValid
  val secondOldestBitsRegOut = secondOldestRes

  val oldestSelectMask = oldestBitsRegOut.asBools.map(_ & oldestValidRegOut)
  val secondSelectMask = secondOldestBitsRegOut.asBools.map(_ & secondOldestValidRegOut)
  selectMask := oldestSelectMask.zip(secondSelectMask).map({case(a,b) => {a | b}})

//  finalSelectResult(0).valid := oldestValidRegOut
//  finalSelectResult(0).bits := Mux1H(oldestBitsRegOut, inSeq.map(_.bits))
//
//  finalSelectResult(1).valid := secondOldestValidRegOut
//  finalSelectResult(1).bits := Mux1H(secondOldestBitsRegOut, inSeq.map(_.bits))

  finalSelectResult(0).valid := oldestValidRegOut
  finalSelectResult(0).bits := Mux1H(oldestBitsRegOut, inSeq.map(_.bits))

  finalSelectResult(1).valid := secondOldestValidRegOut
  finalSelectResult(1).bits := Mux1H(secondOldestBitsRegOut, inSeq.map(_.bits))

  for (((outPort, driver), idx) <- io.issueInfo.zip(finalSelectResult).zipWithIndex) {
    val cancelCond = driver.bits.info.lpv.zip(io.earlyWakeUpCancel).map({case(l, c) => l(0) & c}).reduce(_|_)
    outPort.valid := driver.valid & !cancelCond & !io.redirect.valid
    outPort.bits.bankIdxOH := driver.bits.bankIdxOH
    outPort.bits.entryIdxOH := driver.bits.entryIdxOH
    outPort.bits.info := driver.bits.info
    outPort.bits.info.lpv.zip(driver.bits.info.lpv).foreach({ case (o, i) => o := LogicShiftRight(i, 1)})
    XSPerfAccumulate(s"sel_${idx}_lpv_sel", driver.valid & Cat(driver.bits.info.lpv).orR)
    XSPerfAccumulate(s"sel_${idx}_cancelled", driver.valid & cancelCond)
  }
}