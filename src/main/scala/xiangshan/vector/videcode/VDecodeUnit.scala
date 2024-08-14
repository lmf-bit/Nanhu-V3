package xiangshan.vector.videcode
import chisel3._
import org.chipsalliance.cde.config.Parameters
import xiangshan.ExceptionNO.illegalInstr
import xiangshan.vector._
import xiangshan._
trait VDecodeUnitConstants
{
  // abstract out instruction decode magic numbers
  val VD_MSB = 11
  val VD_LSB = 7
  val VS1_MSB = 19
  val VS1_LSB = 15
  val VS2_MSB = 24
  val VS2_LSB = 20
  val F6_MSB = 31
  val F6_LSB = 26
  val F3_MSB = 14
  val F3_LSB = 12
  val VM_LSB = 25
  val NF_MSB = 31
  val NF_LSB = 29
}
class VDecodeUnit(implicit p: Parameters) extends XSModule with VDecodeUnitConstants {
  val io = IO(new Bundle{
    val in = Input(new MicroOp)
    val out = Output(new MicroOp)
  })

  private val decodeTable = VLDecode.table ++ VSDecode.table ++ VADecode.table ++
    VWDecode.table ++ VNDecode.table

  private val uop = Wire(new MicroOp)
  uop := DontCare
  uop.cf := io.in.cf
  uop.ctrl := io.in.ctrl
  uop.vctrl := Wire(new VCtrlSignals()).decode(io.in.cf.instr, decodeTable)
  uop.ctrl.lsrc(2) := io.in.cf.instr(VD_MSB, VD_LSB)
  uop.ctrl.ldest := io.in.cf.instr(VD_MSB, VD_LSB)
  uop.ctrl.srcType(2) := Mux(io.in.ctrl.vdWen, SrcType.vec, SrcType.DC)
  uop.vctrl.funct6 := io.in.cf.instr(F6_MSB, F6_LSB)
  uop.vctrl.funct3 := io.in.cf.instr(F3_MSB, F3_LSB)
  uop.vctrl.nf := Mux(uop.vctrl.emulType === EmulType.lmul, io.in.cf.instr(NF_MSB, NF_LSB) +& 1.U(1.W), 1.U)
  uop.vctrl.vm := !io.in.cf.instr(VM_LSB)
  private val isIndexedSegLoad = uop.ctrl.fuType === FuType.ldu && uop.ctrl.srcType(1) === SrcType.vec && io.in.cf.instr(NF_MSB, NF_LSB).orR
  when(isIndexedSegLoad) {
    uop.vctrl.notOverlay := true.B
  }
  when(io.in.ctrl.selImm =/= SelImm.IMM_X) {
    uop.ctrl.imm := io.in.cf.instr(VS1_MSB, VS1_LSB)
  }
  io.out := uop
}