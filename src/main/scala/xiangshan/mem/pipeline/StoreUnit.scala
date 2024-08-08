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

package xiangshan.mem

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import utils._
import xs.utils._
import xiangshan.ExceptionNO._
import xiangshan._
import xiangshan.backend.execute.fu.FuConfigs.staCfg
import xiangshan.backend.execute.fu._
import xiangshan.backend.issue.{RSFeedback, RSFeedbackType, RsIdx}
import xiangshan.cache.mmu.{TlbCmd, TlbReq, TlbRequestIO, TlbResp}
import xs.utils.perf.HasPerfLogging

class StoreUnit(implicit p: Parameters) extends XSModule with HasPerfLogging {
  val io = IO(new Bundle() {
    val stin = Flipped(Decoupled(new ExuInput))
    val redirect = Flipped(ValidIO(new Redirect))
    val redirect_dup = Flipped(Vec(3,ValidIO(new Redirect)))
    val feedbackSlow = ValidIO(new RSFeedback)
    val tlb = new TlbRequestIO(if(UseOneDtlb) 2 else 1)
    val pmp = Flipped(new PMPRespBundle())
    val rsIdx = Input(new RsIdx)
    val vmEnable = Input(Bool())
    val lsq = ValidIO(new LsPipelineBundle)
    val lsq_replenish = Output(new LsPipelineBundle())
    val stout = DecoupledIO(new ExuOutput) // writeback store
    // store mask, send to sq in store_s0
    val storeMaskOut = Valid(new StoreMaskBundle)
    //FDI
    val fdiReq = ValidIO(new FDIReqBundle())
    val fdiResp = Flipped(new FDIRespBundle())
    val storeViolationQuery = ValidIO(new storeRAWQueryBundle)
  })
  io.tlb := DontCare
  val s0_in = io.stin
  val s0_out = Wire(Decoupled(new LsPipelineBundle))

  val s0_imm12 = WireInit(s0_in.bits.uop.ctrl.imm(11,0))
  val s0_saddr_lo = s0_in.bits.src(0)(11,0) + Cat(0.U(1.W), s0_imm12)
  val s0_saddr_hi = Mux(s0_saddr_lo(12),
    Mux(s0_imm12(11), s0_in.bits.src(0)(VAddrBits - 1, 12), s0_in.bits.src(0)(VAddrBits - 1, 12) + 1.U),
    Mux(s0_imm12(11), s0_in.bits.src(0)(VAddrBits - 1, 12) + SignExt(1.U, VAddrBits - 12), s0_in.bits.src(0)(VAddrBits - 1, 12)),
  )
  val s0_saddr = Cat(s0_saddr_hi, s0_saddr_lo(11, 0))

  io.tlb.req := DontCare
  io.tlb.req.bits.vaddr := s0_saddr
  io.tlb.req.valid := s0_in.valid
  io.tlb.req.bits.cmd := TlbCmd.write
  io.tlb.req.bits.size := LSUOpType.size(s0_in.bits.uop.ctrl.fuOpType)
  io.tlb.req.bits.robIdx := s0_in.bits.uop.robIdx
  io.tlb.req.bits.debug.pc := s0_in.bits.uop.cf.pc

  s0_out.bits := DontCare
  s0_out.bits.vaddr := s0_saddr

  // Now data use its own io
  // io.out.bits.data := genWdata(io.in.bits.src(1), io.in.bits.uop.ctrl.fuOpType(1,0))
  s0_out.bits.data := s0_in.bits.src(1) // FIXME: remove data from pipeline
  s0_out.bits.uop := s0_in.bits.uop
  s0_out.bits.miss := DontCare
  s0_out.bits.rsIdx := io.rsIdx
  s0_out.bits.mask := genWmask(s0_out.bits.vaddr, s0_in.bits.uop.ctrl.fuOpType(1, 0))
  s0_out.bits.wlineflag := s0_in.bits.uop.ctrl.fuOpType === LSUOpType.cbo_zero
  s0_out.valid := s0_in.valid
  s0_in.ready := s0_out.ready

  // exception check
  val s0_addrAligned = LookupTree(s0_in.bits.uop.ctrl.fuOpType(1, 0), List(
    "b00".U -> true.B, //b
    "b01".U -> (s0_out.bits.vaddr(0) === 0.U), //h
    "b10".U -> (s0_out.bits.vaddr(1, 0) === 0.U), //w
    "b11".U -> (s0_out.bits.vaddr(2, 0) === 0.U) //d
  ))
  private val s0_vaddr_inner = s0_in.bits.src(0) + SignExt(s0_in.bits.uop.ctrl.imm(11, 0), XLEN)
  dontTouch(s0_vaddr_inner)
  private val illegalAddr = s0_vaddr_inner(XLEN - 1, VAddrBits - 1) =/= 0.U && s0_vaddr_inner(XLEN - 1, VAddrBits - 1) =/= Fill(XLEN - VAddrBits + 1, 1.U(1.W))
  s0_out.bits.uop.cf.exceptionVec(storeAddrMisaligned) := !s0_addrAligned && s0_in.bits.uop.loadStoreEnable
  s0_out.bits.uop.cf.exceptionVec(storePageFault) := Mux(s0_in.bits.uop.loadStoreEnable & io.vmEnable, illegalAddr, false.B)

  val s1_in = Wire(Decoupled(new LsPipelineBundle))
  val s1_out = Wire(Decoupled(new LsPipelineBundle))
  val s1_enableMem = s1_in.bits.uop.loadStoreEnable

  // mmio cbo decoder
  val s1_is_mmio_cbo = s1_in.bits.uop.ctrl.fuOpType === LSUOpType.cbo_clean ||
    s1_in.bits.uop.ctrl.fuOpType === LSUOpType.cbo_flush ||
    s1_in.bits.uop.ctrl.fuOpType === LSUOpType.cbo_inval

  val s1_paddr = io.tlb.resp.bits.paddr(0)
  val s1_tlb_miss = io.tlb.resp.bits.miss && s1_enableMem
  val s1_mmio = s1_is_mmio_cbo
  val s1_exception = Mux(s1_enableMem, ExceptionNO.selectByFu(s1_out.bits.uop.cf.exceptionVec, staCfg).asUInt.orR, false.B)

  //FDI check
  io.fdiReq.valid := s1_out.fire //TODO: temporarily assignment
  io.fdiReq.bits.addr := s1_out.bits.vaddr //TODO: need for alignment?
  io.fdiReq.bits.inUntrustedZone := s1_out.bits.uop.fdiUntrusted
  io.fdiReq.bits.operation := FDIOp.write

  s1_in.ready := true.B
  io.tlb.resp.ready := true.B

  val s1_rsFeedback = Wire(ValidIO(new RSFeedback))
  s1_rsFeedback.valid := s1_in.valid && s1_tlb_miss
  s1_rsFeedback.bits.rsIdx := s1_in.bits.rsIdx
  s1_rsFeedback.bits.sourceType := RSFeedbackType.tlbMiss
//  io.feedbackSlow := Pipe(s1_rsFeedback)

  io.feedbackSlow.valid := RegNext(s1_in.valid,false.B)
  io.feedbackSlow.bits.rsIdx := RegEnable(s1_rsFeedback.bits.rsIdx, s1_in.valid)
  io.feedbackSlow.bits.sourceType := RegEnable(Mux(s1_rsFeedback.valid,s1_rsFeedback.bits.sourceType,RSFeedbackType.success), s1_in.valid)

  XSDebug(s1_rsFeedback.valid,
    "S1 Store: tlbHit: %d rsBank: %d rsIdx: %d\n",
    s1_in.valid && !s1_tlb_miss,
    s1_rsFeedback.bits.rsIdx.bankIdxOH,
    s1_rsFeedback.bits.rsIdx.entryIdxOH
  )

  // get paddr from dtlb, check if rollback is needed
  // writeback store inst to lsq
  s1_out.valid := s1_in.valid && !s1_tlb_miss
  s1_out.bits := s1_in.bits
  s1_out.bits.paddr := s1_paddr
  s1_out.bits.miss := false.B
  s1_out.bits.mmio := s1_mmio && s1_enableMem
  s1_out.bits.uop.cf.exceptionVec(storePageFault) := (io.tlb.resp.bits.excp(0).pf.st || s1_in.bits.uop.cf.exceptionVec(storePageFault)) && s1_enableMem
  s1_out.bits.uop.cf.exceptionVec(storeAccessFault) := io.tlb.resp.bits.excp(0).af.st && s1_enableMem

  //store load violation query
  io.lsq.valid := s1_in.valid
  io.lsq.bits := s1_out.bits
  io.lsq.bits.miss := s1_tlb_miss

  io.storeViolationQuery.valid := s1_in.valid && !s1_tlb_miss && !s1_in.bits.uop.robIdx.needFlush(io.redirect)
  io.storeViolationQuery.bits.robIdx := s1_out.bits.uop.robIdx
  io.storeViolationQuery.bits.paddr := s1_out.bits.paddr(PAddrBits - 1,3)
  io.storeViolationQuery.bits.mask := s1_out.bits.mask
  io.storeViolationQuery.bits.stFtqPtr := s1_out.bits.uop.cf.ftqPtr
  io.storeViolationQuery.bits.stFtqOffset := s1_out.bits.uop.cf.ftqOffset

  val s2_in = Wire(Decoupled(new LsPipelineBundle))
  val s2_out = Wire(Decoupled(new LsPipelineBundle))

  val s2_enableMem = s2_in.bits.uop.loadStoreEnable
  val s2_pmp = WireInit(io.pmp)

  val s2_exception = ExceptionNO.selectByFu(s2_out.bits.uop.cf.exceptionVec, staCfg).asUInt.orR && s2_enableMem
  val s2_is_mmio = (s2_in.bits.mmio || s2_pmp.mmio) && s2_enableMem

  s2_in.ready := true.B
  s2_out.bits := s2_in.bits
  s2_out.bits.mmio := s2_is_mmio && !s2_exception
  s2_out.bits.uop.cf.exceptionVec(storeAccessFault) := (s2_in.bits.uop.cf.exceptionVec(storeAccessFault) || s2_pmp.st) && s2_enableMem
  s2_out.valid := s2_in.valid && (!s2_is_mmio || s2_exception)
  //FDI store access fault
  s2_out.bits.uop.cf.exceptionVec(fdiUStoreAccessFault) := io.fdiResp.fdi_fault === FDICheckFault.UWriteFDIFault

  io.storeMaskOut.valid := s0_in.valid
  io.storeMaskOut.bits.mask := s0_out.bits.mask
  io.storeMaskOut.bits.sqIdx := s0_out.bits.uop.sqIdx

  PipelineConnect(s0_out, s1_in, true.B, s0_out.bits.uop.robIdx.needFlush(io.redirect_dup(0)))
  PipelineConnect(s1_out, s2_in, true.B, s1_out.bits.uop.robIdx.needFlush(io.redirect_dup(1)))

  io.lsq_replenish := s2_out.bits // mmio and exception

  val s3_in = Wire(Decoupled(new LsPipelineBundle))
  val s3_out = Wire(Decoupled(new ExuOutput))
  s3_in.ready := true.B

  s3_out := DontCare
  s3_out.valid := s3_in.valid && !s3_in.bits.uop.robIdx.needFlush(io.redirect)
  s3_out.bits.uop := s3_in.bits.uop
  s3_out.bits.data := DontCare
  s3_out.bits.redirectValid := false.B
  s3_out.bits.redirect := DontCare
  s3_out.bits.debug.isMMIO := s3_in.bits.mmio
  s3_out.bits.debug.paddr := s3_in.bits.paddr
  s3_out.bits.debug.vaddr := s3_in.bits.vaddr
  s3_out.bits.debug.isPerfCnt := false.B
  s3_out.bits.fflags := DontCare

  io.stout <> s3_out

  PipelineConnect(s2_out, s3_in, true.B, s2_out.bits.uop.robIdx.needFlush(io.redirect_dup(2)))
  private def printPipeLine(pipeline: LsPipelineBundle, cond: Bool, name: String): Unit = {
    XSDebug(cond,
      p"$name" + p" pc ${Hexadecimal(pipeline.uop.cf.pc)} " +
        p"addr ${Hexadecimal(pipeline.vaddr)} -> ${Hexadecimal(pipeline.paddr)} " +
        p"op ${Binary(pipeline.uop.ctrl.fuOpType)} " +
        p"data ${Hexadecimal(pipeline.data)} " +
        p"mask ${Hexadecimal(pipeline.mask)}\n"
    )
  }

  printPipeLine(s0_out.bits, s0_out.valid, "S0")
  printPipeLine(s1_out.bits, s1_out.valid, "S1")
}
