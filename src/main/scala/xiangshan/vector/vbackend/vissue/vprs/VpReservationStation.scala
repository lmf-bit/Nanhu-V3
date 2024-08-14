package xiangshan.vector.vbackend.vissue.vprs

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan.{HasXSParameter, MicroOp, Redirect, SrcState, SrcType, XSBundle}
import freechips.rocketchip.diplomacy.{LazyModule, LazyModuleImp}
import xiangshan.backend.issue._
import xiangshan.backend.rename.BusyTable
import xiangshan.backend.writeback.{WriteBackSinkNode, WriteBackSinkParam, WriteBackSinkType}

class VprsIssueBundle(implicit p: Parameters) extends XSBundle{
  val uop: MicroOp = new MicroOp
  val pdest:Vec[UInt] = Vec(8, UInt(PhyRegIdxWidth.W))
  val prs: UInt = UInt(PhyRegIdxWidth.W)
  val prsType: UInt = SrcType()
  val rsRen: Bool = Bool()
  val pvs1: Vec[UInt] = Vec(8, UInt(PhyRegIdxWidth.W))
  val pvs2: Vec[UInt] = Vec(8, UInt(PhyRegIdxWidth.W))
  val pov: Vec[UInt] = Vec(8, UInt(PhyRegIdxWidth.W))
  val pvm: UInt = UInt(PhyRegIdxWidth.W)
}

class VpReservationStation(implicit p: Parameters) extends LazyModule with HasXSParameter{
  private val entryNum = vectorParameters.vPRsDepth
  private val wbNodeParam = WriteBackSinkParam(name = "Vector Permutation RS", sinkType = WriteBackSinkType.vecPermRs)
  private val rsParam = RsParam(name = "Vector RS", RsType.vperm, entryNum, 1)
  require(entryNum % rsParam.bankNum == 0)
  val wakeupNode = new WriteBackSinkNode(wbNodeParam)
  val dispatchNode = new RsDispatchNode(rsParam)

  lazy val module = new VpReservationStationImpl(this, rsParam)
}

class VpReservationStationImpl(outer:VpReservationStation, param:RsParam) extends LazyModuleImp(outer) with HasXSParameter {
  require(param.bankNum == 1)
  require(param.entriesNum % param.bankNum == 0)
  private val wbIn = outer.wakeupNode.in.head
  private val wakeup = wbIn._1.zip(wbIn._2._1)

  val io = IO(new Bundle{
    val redirect = Input(Valid(new Redirect))
    val intAllocPregs = Vec(RenameWidth, Flipped(ValidIO(UInt(PhyRegIdxWidth.W))))
    val fpAllocPregs = Vec(RenameWidth, Flipped(ValidIO(UInt(PhyRegIdxWidth.W))))
    val vecAllocPregs = Vec(vectorParameters.vRenameWidth, Flipped(ValidIO(UInt(PhyRegIdxWidth.W))))
    val issue = Decoupled(new VprsIssueBundle)
  })
  require(outer.dispatchNode.in.length == 1)
  private val enq = outer.dispatchNode.in.map(_._1).head

  private val intWkps = wakeup.filter(wkp => wkp._2.writeIntRf)
  private val fpWkps = wakeup.filter(wkp => wkp._2.writeFpRf)
  private val vecWkps = wakeup.filter(wkp => wkp._2.writeVecRf && wkp._2.throughVectorRf)

  private val intWakeupSignals = VecInit(intWkps.map(_._1).map(elm =>{
    val wkp = Wire(Valid(new WakeUpInfo))
    wkp.valid := elm.valid && elm.bits.wakeupValid
    wkp.bits.pdest := elm.bits.uop.pdest
    wkp.bits.robPtr := elm.bits.uop.robIdx
    wkp.bits.lpv := 0.U.asTypeOf(wkp.bits.lpv)
    wkp.bits.destType := Mux(elm.bits.uop.ctrl.rfWen, SrcType.reg, SrcType.default)
    wkp
  }))

  private val fpWakeupSignals = VecInit(fpWkps.map(_._1).map(elm => {
    val wkp = Wire(Valid(new WakeUpInfo))
    wkp.valid := elm.valid && elm.bits.wakeupValid
    wkp.bits.pdest := elm.bits.uop.pdest
    wkp.bits.robPtr := elm.bits.uop.robIdx
    wkp.bits.lpv := 0.U.asTypeOf(wkp.bits.lpv)
    wkp.bits.destType := Mux(elm.bits.uop.ctrl.fpWen, SrcType.fp, SrcType.default)
    wkp
  }))

  private val vectorWakeupSignals = VecInit(vecWkps.map(_._1).map(elm => {
    val wkp = Wire(Valid(new WakeUpInfo))
    wkp.valid := elm.valid && elm.bits.wakeupValid
    wkp.bits.pdest := elm.bits.uop.pdest
    wkp.bits.robPtr := elm.bits.uop.robIdx
    wkp.bits.lpv := 0.U.asTypeOf(wkp.bits.lpv)
    wkp.bits.destType := Mux(elm.bits.uop.ctrl.vdWen, SrcType.vec, SrcType.default)
    wkp
  }))
  private val scalarWakeupSignals = VecInit(intWakeupSignals ++ fpWakeupSignals)

  private val integerBusyTable = Module(new BusyTable(NRPhyRegs, param.bankNum, intWkps.length, RenameWidth, true))
  integerBusyTable.io.allocPregs := io.intAllocPregs
  integerBusyTable.io.wbPregs.zip(intWakeupSignals).foreach({case(bt, wb) =>
    bt.valid := wb.valid && wb.bits.destType === SrcType.reg
    bt.bits := wb.bits.pdest
  })
  private val floatingBusyTable = Module(new BusyTable(NRPhyRegs, param.bankNum, fpWkps.length, RenameWidth, true))
  floatingBusyTable.io.allocPregs := io.fpAllocPregs
  floatingBusyTable.io.wbPregs.zip(fpWakeupSignals).foreach({ case (bt, wb) =>
    bt.valid := wb.valid && wb.bits.destType === SrcType.fp
    bt.bits := wb.bits.pdest
  })
  private val vectorRfSize = coreParams.vectorParameters.vPhyRegsNum
  private val vRenameWidth = coreParams.vectorParameters.vRenameWidth
  private val vectorBusyTable = Module(new BusyTable(vectorRfSize, param.bankNum * 4, vecWkps.length, vRenameWidth))
  vectorBusyTable.io.allocPregs := io.vecAllocPregs
  vectorBusyTable.io.wbPregs.zip(vectorWakeupSignals).foreach({ case (bt, wb) =>
    bt.valid := wb.valid && wb.bits.destType === SrcType.vec
    bt.bits := wb.bits.pdest
  })

  private val arrayWrapper = Module(new VprsArrayWrapper(scalarWakeupSignals.length, vectorWakeupSignals.length))
  private val selectNetwork = Module(new VprsSelectNetwork(param.entriesNum, Some(s"VectorPermutationSelectNetwork")))
  private val issueDriver = Module(new VprsDecoupledPipeline)

  arrayWrapper.io.enq <> enq.head
  arrayWrapper.io.enq.valid := enq.head.valid && !io.redirect.valid
  integerBusyTable.io.read.head.req := enq.head.bits.psrc(0)
  floatingBusyTable.io.read.head.req := enq.head.bits.psrc(0)
  vectorBusyTable.io.read.head.req := enq.head.bits.psrc(0)
  vectorBusyTable.io.read(1).req := enq.head.bits.psrc(1)
  vectorBusyTable.io.read(2).req := enq.head.bits.psrc(2)
  vectorBusyTable.io.read(3).req := enq.head.bits.vm

  arrayWrapper.io.enq.bits.srcState(0) := MuxCase(SrcState.rdy, Seq(
    (enq.head.bits.ctrl.srcType(0) === SrcType.reg) -> integerBusyTable.io.read.head.resp,
    (enq.head.bits.ctrl.srcType(0) === SrcType.fp) -> floatingBusyTable.io.read.head.resp,
    (enq.head.bits.ctrl.srcType(0) === SrcType.vec) -> vectorBusyTable.io.read.head.resp
  ))
  arrayWrapper.io.enq.bits.srcState(1) := vectorBusyTable.io.read(1).resp
  arrayWrapper.io.enq.bits.srcState(2) := vectorBusyTable.io.read(2).resp
  arrayWrapper.io.enq.bits.vmState := vectorBusyTable.io.read(3).resp

  selectNetwork.io.redirect := io.redirect
  selectNetwork.io.selectInfo := arrayWrapper.io.selectInfo
  arrayWrapper.io.issueOH.valid := selectNetwork.io.issueInfo.fire
  arrayWrapper.io.issueOH.bits := selectNetwork.io.issueInfo.bits.entryIdxOH
  arrayWrapper.io.scalarWakeUps := scalarWakeupSignals
  arrayWrapper.io.vectorWakeUps := vectorWakeupSignals
  arrayWrapper.io.redirect := io.redirect

  private val issueInfo = selectNetwork.io.issueInfo.bits.info
  selectNetwork.io.issueInfo.ready := issueDriver.io.enq.ready
  issueDriver.io.enq.valid := selectNetwork.io.issueInfo.valid
  issueDriver.io.enq.bits.uop := arrayWrapper.io.issueUop
  issueDriver.io.enq.bits.pdest := arrayWrapper.io.issuePdest
  issueDriver.io.enq.bits.pov := issueInfo.pov
  issueDriver.io.enq.bits.prs := issueInfo.prs
  issueDriver.io.enq.bits.prsType := issueInfo.prsType
  issueDriver.io.enq.bits.pvm := issueInfo.pvm
  issueDriver.io.enq.bits.pvs1 := issueInfo.pvs1
  issueDriver.io.enq.bits.pvs2 := issueInfo.pvs2
  issueDriver.io.enq.bits.rsRen := DontCare
  issueDriver.io.redirect := io.redirect

  io.issue <> issueDriver.io.deq


  println("\nVector Permutation Reservation Wake Up Ports Config:")
  wakeup.zipWithIndex.foreach({case((_, cfg), idx) =>
    println(s"Wake Port $idx ${cfg.name} of ${cfg.complexName} #${cfg.id}")
  })
}