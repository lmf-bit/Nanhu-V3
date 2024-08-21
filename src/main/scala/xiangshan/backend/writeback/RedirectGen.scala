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
package xiangshan.backend.writeback

import chisel3._
import chisel3.util._
import RedirectGen._
import org.chipsalliance.cde.config.Parameters
import xiangshan._
import xiangshan.backend.decode.ImmUnion
import xiangshan.backend.issue.SelectPolicy
import xiangshan.backend.rob.RobPtr
import xiangshan.frontend.{BrType, FtqPCEntry}
import xs.utils.{SignExt, XORFold}
import xs.utils.perf.HasPerfLogging

class RedirectSelectBundle(idxWidth:Int)(implicit p: Parameters) extends Bundle{
  val robIdx = new RobPtr
  val idxOH = UInt(idxWidth.W)
}

object RedirectGen{
  private def getRedirect(exuOut: Valid[ExuOutput], p:Parameters): ValidIO[Redirect] = {
    val redirect = Wire(Valid(new Redirect()(p)))
    val ri = exuOut.bits.redirect
    redirect.valid := exuOut.bits.redirectValid && (ri.cfiUpdate.isMisPred || ri.isException || ri.isLoadStore || ri.isLoadLoad || ri.isFlushPipe)
    redirect.bits := exuOut.bits.redirect
    redirect
  }

  private def selectOldest(in: Seq[Valid[Redirect]], p: Parameters): (Valid[Redirect], UInt) = {
    val selector = Module(new SelectPolicy(in.length, true, true)(p))
    selector.io.in.zip(in).foreach({ case (a, b) =>
      a.valid := b.valid
      a.bits := b.bits.robIdx
    })
    val res = Wire(Valid(new Redirect()(p)))
    res.valid := selector.io.out.valid
    res.bits := Mux1H(selector.io.out.bits, in.map(_.bits))
    (res, selector.io.out.bits)
  }
}

class RedirectGen(jmpRedirectNum:Int, aluRedirectNum:Int, memRedirectNum:Int)(implicit p: Parameters) extends XSModule with HasPerfLogging{
  val io = IO(new Bundle{
    val jmpWbIn = Input(Vec(jmpRedirectNum, Flipped(ValidIO(new ExuOutput))))
    val aluWbIn = Input(Vec(aluRedirectNum, Flipped(ValidIO(new ExuOutput))))
    val memWbIn = Input(Vec(memRedirectNum, Flipped(ValidIO(new ExuOutput))))
    val pcReadAddr = Output(Vec(2, UInt(log2Ceil(FtqSize).W)))
    val pcReadData = Input(Vec(2, new FtqPCEntry))
    val redirectIn = Input(Valid(new Redirect))
    val redirectOut = Output(Valid(new Redirect))
    val preWalk = Output(Valid(new Redirect))
    val memPredUpdate = Output(Valid(new MemPredUpdateReq))
  })

  // S1: select oldest redirect, read pc
  private val s1_allWb = Wire(Vec(jmpRedirectNum + aluRedirectNum + memRedirectNum, Valid(new ExuOutput)))
  s1_allWb.zip(io.jmpWbIn ++ io.aluWbIn ++ io.memWbIn).foreach({case(s, wb) =>
    s := DontCare
    val redirectValidCond = wb.bits.redirectValid && !wb.bits.redirect.robIdx.needFlush(io.redirectIn)
    s.valid := RegNext(wb.valid, false.B)
    s.bits.redirectValid := RegNext(redirectValidCond, false.B)
    s.bits.uop := RegEnable(wb.bits.uop, wb.valid)
    s.bits.redirect := RegEnable(wb.bits.redirect, wb.bits.redirectValid)
  })
  private val s1_allRedirect = s1_allWb.map(getRedirect(_, p))
  private val (s1_redirectSel, s1_redirectIdxOH) = selectOldest(s1_allRedirect, p)
  private val s1_redirectValid = s1_redirectSel.valid && !s1_redirectSel.bits.robIdx.needFlush(io.redirectIn)
  private val s1_exuOutSel = Mux1H(s1_redirectIdxOH, s1_allWb)
  private val s1_target = Mux1H(s1_redirectIdxOH(jmpRedirectNum - 1, 0), s1_allWb.take(jmpRedirectNum).map(_.bits.redirect.cfiUpdate.target))

  private val s1_uopReg = Wire(new MicroOp)
  s1_uopReg := s1_exuOutSel.bits.uop
  when(s1_redirectValid && s1_redirectSel.bits.isException) {
    s1_uopReg.cf.pd.valid := false.B
    s1_uopReg.cf.pd.isRVC := false.B
    s1_uopReg.cf.pd.brType := BrType.notCFI
    s1_uopReg.cf.pd.isCall := false.B
    s1_uopReg.cf.pd.isRet := false.B
    s1_uopReg.ctrl.imm := 0.U
  }

  private var addrIdx = 0
  private val isJmp = s1_redirectIdxOH(jmpRedirectNum + addrIdx - 1, addrIdx).orR
  addrIdx = addrIdx + jmpRedirectNum
  private val isAlu = s1_redirectIdxOH(aluRedirectNum + addrIdx - 1, addrIdx).orR
  addrIdx = addrIdx + aluRedirectNum
  private val isMem = s1_redirectIdxOH(memRedirectNum + addrIdx - 1, addrIdx).orR
  addrIdx = addrIdx + memRedirectNum

  io.pcReadAddr(0) := s1_redirectSel.bits.ftqIdx.value

  // S2: get pc，send redirect out
  private val s2_isJmpReg = RegEnable(isJmp, s1_redirectValid)
  private val s2_isMemReg = RegEnable(isMem, s1_redirectValid)
  private val s2_pcReadReg = RegEnable(io.pcReadData(0).getPc(s1_redirectSel.bits.ftqOffset), s1_redirectValid)
  private val s2_jmpTargetReg = RegEnable(s1_target, s1_redirectValid)
  private val s2_imm12Reg = RegEnable(s1_uopReg.ctrl.imm(11, 0), s1_redirectValid)
  private val s2_pdReg = RegEnable(s1_uopReg.cf.pd, s1_redirectValid)
  private val s2_robIdxReg = RegEnable(s1_redirectSel.bits.robIdx, s1_redirectValid)
  private val s2_redirectBitsReg = RegEnable(s1_redirectSel.bits, s1_redirectValid)
  private val s2_redirectValidReg = RegNext(s1_redirectValid, false.B)
  private val s2_redirectValid = s2_redirectValidReg && !s2_redirectBitsReg.robIdx.needFlush(io.redirectIn)

  private val branchTarget = s2_pcReadReg + SignExt(ImmUnion.B.toImm32(s2_imm12Reg), XLEN)
  private val snpc = s2_pcReadReg + Mux(s2_pdReg.isRVC, 2.U, 4.U)
  private val redirectTarget = WireInit(snpc)
  when(s2_isMemReg){
    redirectTarget := s2_pcReadReg
  }.elsewhen(s2_redirectBitsReg.isException || s2_redirectBitsReg.isXRet){
    redirectTarget := s2_jmpTargetReg
  }.elsewhen(s2_redirectBitsReg.cfiUpdate.taken){
    redirectTarget := Mux(s2_isJmpReg, s2_jmpTargetReg, branchTarget)
  }
  io.redirectOut.valid := s2_redirectValidReg && !s2_robIdxReg.needFlush(io.redirectIn)
  io.redirectOut.bits := s2_redirectBitsReg
  io.redirectOut.bits.cfiUpdate.pc := s2_pcReadReg
  io.redirectOut.bits.cfiUpdate.pd := s2_pdReg
  io.redirectOut.bits.cfiUpdate.target := redirectTarget
  io.redirectOut.bits.isPreWalk := false.B

  io.preWalk.bits := DontCare
  io.preWalk.valid := s1_redirectSel.valid & !s2_redirectValidReg & !s2_redirectValidReg
  io.preWalk.bits.robIdx := PriorityMux(s1_allRedirect.map(_.valid), s1_allRedirect.map(_.bits.robIdx))
  io.preWalk.bits.level := RedirectLevel.flushAfter
  io.preWalk.bits.isException := false.B
  io.preWalk.bits.isLoadStore := false.B
  io.preWalk.bits.isLoadLoad := false.B
  io.preWalk.bits.isFlushPipe := false.B
  io.preWalk.bits.cfiUpdate.isMisPred := false.B
  io.preWalk.bits.isPreWalk := true.B

  // get pc from PcMem
  // valid only if redirect is caused by load violation
  // store_pc is used to update store set
  io.pcReadAddr(1) := s2_redirectBitsReg.stFtqIdx.value
  private val shouldUpdateMdp = s2_isMemReg && s2_redirectValidReg && s2_redirectBitsReg.isLoadStore
  private val storePc = RegEnable(io.pcReadData(1).getPc(s2_redirectBitsReg.stFtqOffset), shouldUpdateMdp)

  // update load violation predictor if load violation redirect triggered
  io.memPredUpdate.valid := RegNext(shouldUpdateMdp, init = false.B)
  // update wait table
  io.memPredUpdate.bits.waddr := RegEnable(XORFold(s2_pcReadReg(VAddrBits - 1, 1), MemPredPCWidth), shouldUpdateMdp)
  io.memPredUpdate.bits.wdata := true.B
  // update store set
  io.memPredUpdate.bits.ldpc := RegEnable(XORFold(s2_pcReadReg(VAddrBits - 1, 1), MemPredPCWidth), shouldUpdateMdp)
  // store pc is ready 1 cycle after s3_isReplay is judged
  io.memPredUpdate.bits.stpc := XORFold(storePc(VAddrBits - 1, 1), MemPredPCWidth)


  private val s2_uopReg = RegEnable(s1_exuOutSel.bits.uop, s1_redirectValid)
  XSPerfAccumulate("total_redirect_num", io.redirectOut.valid)
  XSPerfAccumulate("miss_pred_redirect_num", io.redirectOut.valid && io.redirectOut.bits.cfiUpdate.isMisPred)
  XSPerfAccumulate("bad_taken_redirect_num", io.redirectOut.valid && io.redirectOut.bits.cfiUpdate.isMisPred && io.redirectOut.bits.cfiUpdate.taken)
  XSPerfAccumulate("bad_not_taken_redirect_num", io.redirectOut.valid && io.redirectOut.bits.cfiUpdate.isMisPred && !io.redirectOut.bits.cfiUpdate.taken)
  XSPerfAccumulate("load_store_redirect_num", io.redirectOut.valid && io.redirectOut.bits.isLoadStore)
  XSPerfAccumulate("load_load_redirect_num", io.redirectOut.valid && io.redirectOut.bits.isLoadLoad)
  XSPerfAccumulate("exception_redirect_num", io.redirectOut.valid && io.redirectOut.bits.isException)
  XSPerfAccumulate("flush_redirect_num", io.redirectOut.valid && io.redirectOut.bits.isFlushPipe)
  XSPerfAccumulate("redirect_s2_new_s1", s2_redirectValid && s1_redirectValid && s1_exuOutSel.bits.uop.robIdx < s2_uopReg.robIdx)
  XSPerfAccumulate("redirect_s3_new_s1", s1_redirectValid && io.redirectOut.bits.isLoadLoad && s1_exuOutSel.bits.uop.robIdx < io.redirectOut.bits.robIdx)
  XSPerfAccumulate("redirect_s3_new_s2", io.redirectOut.valid && s2_redirectValid && s2_uopReg.robIdx < io.redirectOut.bits.robIdx)
}
