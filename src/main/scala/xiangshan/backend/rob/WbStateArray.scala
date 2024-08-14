package xiangshan.backend.rob
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xiangshan.{ExuOutput, Redirect, XSBundle, XSModule}
import freechips.rocketchip.util.SeqToAugmentedSeq
class EnqReq(implicit p: Parameters) extends XSBundle {
  val robIdx = new RobPtr
  val data = UInt(log2Ceil(RenameWidth + 1).W)
  val block = Bool()
}
class WbReq(implicit p: Parameters) extends XSBundle {
  val robIdx = new RobPtr
  val data = UInt(log2Ceil(RenameWidth + 1).W)
}
class WbStateArray(enqNum:Int, wbNum:Int, redirectNum:Int)(implicit p: Parameters) extends XSModule{
  val io = IO(new Bundle {
    val enqIn = Input(Vec(enqNum, Valid(new EnqReq)))
    val wbIn = Input(Vec(wbNum, Valid(new WbReq)))
    val redirectIn = Input(Vec(redirectNum, Valid(new WbReq)))
    val out = Output(Vec(RobSize, Bool()))
  })
  private val enq = io.enqIn
  private val wb = io.wbIn
  private val rdc = io.redirectIn
  private val maxWb = (1 << (RenameWidth + 1) - 1).asUInt
  private val writebackNumReg = RegInit(VecInit(Seq.fill(RobSize)(maxWb)))
  private val currentWbNum = writebackNumReg
  private val mayBeFlushed = RegInit(VecInit(Seq.fill(RobSize)(false.B)))

  for(i <- writebackNumReg.indices) {
    val enqSel = enq.map(r => r.valid && r.bits.robIdx.value === i.U)
    assert(PopCount(enqSel) <= 1.U, "only 1 entry can be selected whether compress or not")
    val enqData = Mux1H(enqSel, enq.map(_.bits.data))
    val enqBlock = Mux1H(enqSel, enq.map(_.bits.block))
    val wbSel = wb.map(r => r.valid && r.bits.robIdx.value === i.U && (r.bits.data === 1.U))
    val wbDataNum = PopCount(wbSel)
    val rdcSel = rdc.map(r => r.valid && r.bits.robIdx.value === i.U)
    val rdcData = Mux1H(rdcSel, rdc.map(_.bits.data))
    val debugRdcSel = VecInit(rdcSel).asUInt
    dontTouch(debugRdcSel)
    dontTouch(rdcData)
    val enqHit = Cat(enqSel).orR
    val wbHit = Cat(wbSel).orR
    val rdcHit = Cat(rdcSel).orR
    when(enqHit) {
      writebackNumReg(i) := Mux(enqBlock, maxWb, enqData)
      mayBeFlushed(i) := false.B
    }.elsewhen(rdcHit) {
      writebackNumReg(i) := Mux(rdcData===1.U, 0.U, maxWb)
      mayBeFlushed(i) := true.B
    }.elsewhen(wbHit) {
      writebackNumReg(i) := Mux(mayBeFlushed(i), maxWb, currentWbNum(i) - wbDataNum)
    }
  }
  io.out.zip(writebackNumReg).foreach({case(out, wbNum) => out := wbNum === 0.U})
  for(i <- writebackNumReg.indices){assert(writebackNumReg(i) <= maxWb, "writeBackReg %d num max is %d", i.U, maxWb)}

  private var enqIdx = 0
  private var wbIdx = 0
  private var rdcIdx = 0

  def enqueue(v: Bool, addr: RobPtr, block: Bool, data: UInt): Unit = {
    this.io.enqIn(enqIdx).valid := v
    this.io.enqIn(enqIdx).bits.robIdx := addr
    this.io.enqIn(enqIdx).bits.data := data
    this.io.enqIn(enqIdx).bits.block := block
    enqIdx = enqIdx + 1
  }

  def writeback(v:Bool, addr:RobPtr, data:Bool):Unit = {
    this.io.wbIn(wbIdx).valid := v
    this.io.wbIn(wbIdx).bits.robIdx := addr
    this.io.wbIn(wbIdx).bits.data := data.asUInt
    wbIdx = wbIdx + 1
  }

  def redirect(v:Bool, addr:RobPtr, data:Bool):Unit = {
    this.io.redirectIn(rdcIdx).valid := v
    this.io.redirectIn(rdcIdx).bits.robIdx := addr
    this.io.redirectIn(rdcIdx).bits.data := data.asUInt
    dontTouch(this.io.redirectIn)
    rdcIdx = rdcIdx + 1
  }
}
