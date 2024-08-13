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

package xiangshan.cache.mmu

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan._
import utils._
import xiangshan.backend.execute.fu.csr.HasCSRConst
import xiangshan.backend.execute.fu.fence.SfenceBundle
import xs.utils._
import xs.utils.perf.HasPerfLogging



class TLB_frontend(Width: Int, nRespDups: Int = 1, q: TLBParameters)(Block: Seq[Boolean] = Seq.fill(Width)(false))(implicit p: Parameters) extends TlbModule
  with HasCSRConst with HasPerfEvents with HasPerfLogging {
  val io = IO(new TlbIO(Width, nRespDups, q))

  val req = io.requestor.map(_.req)
  val resp = io.requestor.map(_.resp)
  val ptw = io.ptw
  val pmp = io.pmp
  val ptw_resp = ptw.resp.bits
  val ptw_resp_v = ptw.resp.valid

  val req_out = req.map(a => RegEnable(a.bits, a.fire))
  val req_out_v = (0 until Width).map(i => ValidHold(req(i).fire, resp(i).fire, io.flushPipe(i)))  //&& !req(i).bits.kill

  val mode_tmp = if (q.useDmode) io.csr.priv.dmode else io.csr.priv.imode
  val mode_dup = Seq.fill(Width)(RegNext(mode_tmp))
  val vmEnable_tmp = if (EnbaleTlbDebug) (io.csr.satp.mode === 8.U)
    else (io.csr.satp.mode === 8.U && (mode_tmp < ModeM))
  val vmEnable_dup = Seq.fill(Width)(RegNext(vmEnable_tmp))
  val csr_dup = Seq.fill(Width)(RegNext(io.csr))
  val csr_dup_2 = RegNext(io.csr)

  val sfence_valid = RegNext(io.sfence.valid)
  val sfence_bits = RegEnable(io.sfence.bits, io.sfence.valid)
  val sfence = Wire(new SfenceBundle)
  sfence.valid := sfence_valid
  sfence.bits := sfence_bits

  val satp = csr_dup.head.satp
  val priv = csr_dup.head.priv
  val ifecth = if (q.fetchi) true.B else false.B

  val reqAddr = req.map(_.bits.vaddr.asTypeOf(new VaBundle))
  val reqValid = req.map(_.valid)
  val vpn = reqAddr.map(_.vpn)
  val cmd = req.map(_.bits.cmd)

  val tlbfa = TlbStorage(
    name = "tlbfa",
    sameCycle = false,
    ports = Width,
    nDups = nRespDups,
    nSets = 1,
    nWays = q.nWays,
    saveLevel = q.saveLevel,
    normalPage = true,
    superPage = true,
  )

  for (i <- 0 until Width) {
    tlbfa.r_req_apply(
      valid = io.requestor(i).req.valid,
      vpn = vpn(i),
      asid = csr_dup(i).satp.asid,
      i = i,
    )
  }

  val refill_idx = if (q.outReplace) {
    io.replace.page.access <> tlbfa.access
    io.replace.page.chosen_set := DontCare // only fa
    io.replace.page.refillIdx
  } else {
    val re = ReplacementPolicy.fromString(q.replacer, q.nWays)
    re.access(tlbfa.access.map(_.touch_ways))
    re.way
  }

  val refill_now = ptw_resp_v
  val refill = ptw_resp_v && !sfence.valid && !satp.changed

  tlbfa.w_apply(
    valid = refill,
    wayIdx = refill_idx,
    data = ptw_resp,
  )

  tlbfa.sfence <> sfence
  tlbfa.csr <> csr_dup_2

  val readResult = (0 until Width).map(TLBRead)
  val hitVec = readResult.map(_._1)
  val missVec = readResult.map(_._2)
  val validRegVec = readResult.map(_._3)
  val pmp_addr = readResult.map(_._4)
  val perm = readResult.map(_._5)

  (0 until Width).foreach { i =>
    pmp_check(pmp_addr(i), req_out(i).size, req_out(i).cmd, i)
    for (d <- 0 until nRespDups) {
      perm_check(perm(i)(d), req_out(i).cmd, i, d)
    }
  }

  // handle block or non-block io
  // for non-block io, just return the above result, send miss to ptw
  // for block io, hold the request, send miss to ptw,
  //   when ptw back, return the result
  (0 until Width).foreach{ i =>
    if (Block(i)) handle_block(i)
    else handle_nonblock(i)
  }

  io.ptw.resp.ready := true.B

  def TLBRead(i: Int) = {
    val (hit, ppn, perm_) = tlbfa.r_resp_apply(i)
    // assert(!(normal_hit && super_hit && vmEnable_dup(i) && RegNext(req(i).valid, init = false.B)))

    val validReg = RegNext(reqValid(i), init = false.B)
    val offReg = RegEnable(reqAddr(i).off, 0.U, reqValid(i))

    /** *************** next cycle when two cycle is false******************* */
    val miss = !hit && vmEnable_dup(i)
    hit.suggestName(s"hit_${i}")
    miss.suggestName(s"miss_${i}")

    val vaddr = SignExt(req(i).bits.vaddr, PAddrBits)
    resp(i).bits.miss := miss
    resp(i).bits.ptwBack := ptw.resp.fire

    val perm = WireInit(VecInit(Seq.fill(nRespDups)(0.U.asTypeOf(new TlbPermBundle))))
    // duplicate resp part
    for (d <- 0 until nRespDups) {
      // ppn and perm from tlbfa resp
      perm(d) := perm_(d)
      val paddr = Cat(ppn(d), offReg)
      resp(i).bits.paddr(d) := Mux(vmEnable_dup(i), paddr, RegEnable(vaddr,reqValid(i)))
    }

    val pmp_paddr = resp(i).bits.paddr.head

    (hit, miss, validReg, pmp_paddr, perm)
  }

  def pmp_check(addr: UInt, size: UInt, cmd: UInt, idx: Int): Unit = {
    pmp(idx).valid := resp(idx).valid
    pmp(idx).bits.addr := addr
    pmp(idx).bits.size := size
    pmp(idx).bits.cmd := cmd
  }

  def perm_check(perm: TlbPermBundle, cmd: UInt, idx: Int, nDups: Int) = {
    val pf = perm.pf
    val af = perm.af
    val ldUpdate = !perm.a && TlbCmd.isRead(cmd) && !TlbCmd.isAmo(cmd) // update A/D through exception
    val stUpdate = (!perm.a || !perm.d) && (TlbCmd.isWrite(cmd) || TlbCmd.isAmo(cmd)) // update A/D through exception
    val instrUpdate = !perm.a && TlbCmd.isExec(cmd) // update A/D through exception
    val modeCheck = !(mode_dup(idx) === ModeU && !perm.u || mode_dup(idx) === ModeS && perm.u && (!priv.sum || ifecth))
    val ldPermFail = !(modeCheck && (perm.r || priv.mxr && perm.x))
    val stPermFail = !(modeCheck && perm.w)
    val instrPermFail = !(modeCheck && perm.x)
    val ldPf = (ldPermFail || pf) && (TlbCmd.isRead(cmd) && !TlbCmd.isAmo(cmd))
    val stPf = (stPermFail || pf) && (TlbCmd.isWrite(cmd) || TlbCmd.isAmo(cmd))
    val instrPf = (instrPermFail || pf) && TlbCmd.isExec(cmd)
    val fault_valid = vmEnable_dup(idx)
    resp(idx).bits.excp(nDups).pf.ld := (ldPf || ldUpdate) && fault_valid && !af
    resp(idx).bits.excp(nDups).pf.st := (stPf || stUpdate) && fault_valid && !af
    resp(idx).bits.excp(nDups).pf.instr := (instrPf || instrUpdate) && fault_valid && !af
    // NOTE: pf need && with !af, page fault has higher priority than access fault
    // but ptw may also have access fault, then af happens, the translation is wrong.
    // In this case, pf has lower priority than af

    // for tlb without sram, tlb will miss, pm should be ignored outsize
    resp(idx).bits.excp(nDups).af.ld := af && TlbCmd.isRead(cmd) && fault_valid
    resp(idx).bits.excp(nDups).af.st := af && TlbCmd.isWrite(cmd) && fault_valid
    resp(idx).bits.excp(nDups).af.instr := af && TlbCmd.isExec(cmd) && fault_valid
  }

  def handle_nonblock(idx: Int): Unit = {
    io.requestor(idx).resp.valid := req_out_v(idx)
    io.requestor(idx).req.ready := io.requestor(idx).resp.ready // should always be true
    dontTouch(io.requestor(idx).req)
//    XSError(!io.requestor(idx).resp.ready, s"${q.name} port ${idx} is non-block, resp.ready must be true.B")

    io.ptw.req(idx).valid :=  RegNext(validRegVec(idx) && missVec(idx), false.B) && !RegNext(refill, init = false.B) && !RegNext(RegNext(refill, init = false.B), init = false.B)
//    when(RegEnable(io.requestor(idx).req_kill, RegNext(io.requestor(idx).req.fire))) {
//      io.ptw.req(idx).valid := false.B
//    }
    io.ptw.req(idx).bits.vpn := RegNext(RegEnable( reqAddr(idx).vpn, reqValid(idx)))
  }

  def handle_block(idx: Int): Unit = {
    io.requestor(idx).req.ready := !req_out_v(idx) || io.requestor(idx).resp.fire
    val miss_req_vpn = req_out(idx).vaddr.asTypeOf(new VaBundle).vpn

    val hit = io.ptw.resp.bits.hit(miss_req_vpn, io.csr.satp.asid, allType = true) && io.ptw.resp.valid

    def GatedValidRegNext(next: Bool, init: Bool = false.B): Bool = {
      val last = WireInit(false.B)
      last := RegNext(next, false.B)
      last
    }

    val new_coming_valid = WireInit(false.B)
    new_coming_valid := req(idx).fire && !io.flushPipe(idx) // && !req(idx).bits.kill
    val new_coming = RegNext(new_coming_valid, false.B)
    val miss_wire = new_coming && missVec(idx)
    val miss_v = ValidHoldBypass(miss_wire, resp(idx).fire, io.flushPipe(idx))
    val miss_req_v = ValidHoldBypass(miss_wire || (miss_v && !sfence.valid && !satp.changed),
      io.ptw.req(idx).fire || resp(idx).fire, io.flushPipe(idx))

    resp(idx).valid := req_out_v(idx) && !(miss_v && vmEnable_dup(idx))
    when (io.ptw.resp.fire && req_out_v(idx)){
      io.requestor(idx).resp.valid := true.B
      io.requestor(idx).resp.bits.miss := false.B

      val ptwRest = io.ptw.resp.bits
      val paddr = Cat(ptwRest.genPPN(req_out(idx).vaddr.asTypeOf(new VaBundle).vpn), req_out(idx).vaddr.asTypeOf(new VaBundle).off)

      for (d <- 0 until nRespDups) {
        resp(idx).bits.paddr(d) := paddr
        perm_check(ptwRest, req_out(idx).cmd, idx, d)
      }
      pmp_check(resp(idx).bits.paddr(0), req_out(idx).size, req_out(idx).cmd, idx)
    }

    val ptw_req = io.ptw.req(idx)
    ptw_req.valid := miss_req_v
    ptw_req.bits.vpn := miss_req_vpn
  }

  if (!q.shouldBlock) {
    for (i <- 0 until Width) {
      XSPerfAccumulate("first_access" + Integer.toString(i, 10), validRegVec(i) && vmEnable_dup.head && RegNext(req(i).bits.debug.isFirstIssue))
      XSPerfAccumulate("access" + Integer.toString(i, 10), validRegVec(i) && vmEnable_dup.head)
    }
    for (i <- 0 until Width) {
      XSPerfAccumulate("first_miss" + Integer.toString(i, 10), validRegVec(i) && vmEnable_dup.head && missVec(i) && RegNext(req(i).bits.debug.isFirstIssue))
      XSPerfAccumulate("miss" + Integer.toString(i, 10), validRegVec(i) && vmEnable_dup.head && missVec(i))
    }
  } else {
    // NOTE: ITLB is blocked, so every resp will be valid only when hit
    // every req will be ready only when hit
    for (i <- 0 until Width) {
      XSPerfAccumulate(s"access${i}", io.requestor(i).req.fire && vmEnable_dup.head)
      XSPerfAccumulate(s"miss${i}", ptw.req(i).fire)
    }

  }
  //val reqCycleCnt = Reg(UInt(16.W))
  //reqCycleCnt := reqCycleCnt + BoolStopWatch(ptw.req(0).fire, ptw.resp.fire || sfence.valid)
  //XSPerfAccumulate("ptw_req_count", ptw.req.fire)
  //XSPerfAccumulate("ptw_req_cycle", Mux(ptw.resp.fire, reqCycleCnt, 0.U))
  XSPerfAccumulate("ptw_resp_count", ptw.resp.fire)
  XSPerfAccumulate("ptw_resp_pf_count", ptw.resp.fire && ptw.resp.bits.pf)
  XSPerfAccumulate("ptw_resp_sp_count", ptw.resp.fire && !ptw.resp.bits.pf && (ptw.resp.bits.entry.level.get === 0.U || ptw.resp.bits.entry.level.get === 1.U))

  // Log
  for(i <- 0 until Width) {
    XSDebug(req(i).valid, p"req(${i.U}): (${req(i).valid} ${req(i).ready}) ${req(i).bits}\n")
    XSDebug(resp(i).valid, p"resp(${i.U}): (${resp(i).valid} ${resp(i).ready}) ${resp(i).bits}\n")
  }

  XSDebug(sfence.valid, p"Sfence: ${sfence}\n")
  XSDebug(ParallelOR(reqValid)|| ptw.resp.valid, p"CSR: ${csr_dup.head}\n")
  XSDebug(ParallelOR(reqValid) || ptw.resp.valid, p"vmEnable:${vmEnable_dup.head} hit:${Binary(VecInit(hitVec).asUInt)} miss:${Binary(VecInit(missVec).asUInt)}\n")
  for (i <- ptw.req.indices) {
    XSDebug(ptw.req(i).fire, p"L2TLB req:${ptw.req(i).bits}\n")
  }
  XSDebug(ptw.resp.valid, p"L2TLB resp:${ptw.resp.bits} (v:${ptw.resp.valid}r:${ptw.resp.ready}) \n")

  println(s"${q.name}: page: ${q.nWays} Ways fully-associative ${q.replacer.get}")

//   // NOTE: just for simple tlb debug, comment it after tlb's debug
  // assert(!io.ptw.resp.valid || io.ptw.resp.bits.entry.tag === io.ptw.resp.bits.entry.ppn, "Simple tlb debug requires vpn === ppn")

  val perfEvents = if(!q.shouldBlock) {
    Seq(
      ("access", PopCount((0 until Width).map(i => vmEnable_dup.head && validRegVec(i)))              ),
      ("miss  ", PopCount((0 until Width).map(i => vmEnable_dup.head && validRegVec(i) && missVec(i)))),
    )
  } else {
    Seq(
      ("access", PopCount((0 until Width).map(i => io.requestor(i).req.fire))),
      ("miss  ", PopCount((0 until Width).map(i => ptw.req(i).fire))         ),
    )
  }
  generatePerfEvent()
}



class TLB_backend(Width: Int, nRespDups: Int = 1, q: TLBParameters)(implicit p: Parameters) extends TlbModule
  with HasCSRConst with HasPerfEvents with HasPerfLogging {
  val io = IO(new TlbIO(Width, nRespDups, q))

  // require(q.superAssociative == "fa")
  // if (q.sameCycle || q.missSameCycle) {
  //   require(q.normalAssociative == "fa")
  // }

  val req = io.requestor.map(_.req)
  val resp = io.requestor.map(_.resp)
  val ptw = io.ptw
  val pmp = io.pmp
  val ptw_resp = ptw.resp.bits
  val ptw_resp_v = ptw.resp.valid

  val mode_tmp = if (q.useDmode) io.csr.priv.dmode else io.csr.priv.imode
  val mode_dup = Seq.fill(Width)(RegNext(mode_tmp))
  val vmEnable_tmp = if (EnbaleTlbDebug) (io.csr.satp.mode === 8.U)
  else (io.csr.satp.mode === 8.U && (mode_tmp < ModeM))
  val vmEnable_dup = Seq.fill(Width)(RegNext(vmEnable_tmp))
  //  val sfence_dup = Seq.fill(2)(RegNext(io.sfence))
  val csr_dup = Seq.fill(Width)(RegNext(io.csr))
  val csr_dup_2 = RegNext(io.csr)

  val sfence_valid = RegNext(io.sfence.valid)
  val sfence_bits = RegEnable(io.sfence.bits, io.sfence.valid)
  val sfence = Wire(new SfenceBundle)
  sfence.valid := sfence_valid
  sfence.bits := sfence_bits

  val satp = csr_dup.head.satp
  val priv = csr_dup.head.priv
  val ifecth = if (q.fetchi) true.B else false.B

  val reqAddr = req.map(_.bits.vaddr.asTypeOf(new VaBundle))
  val reqValid = req.map(_.valid)
  val vpn = reqAddr.map(_.vpn)
  val cmd = req.map(_.bits.cmd)

  def widthMapSeq[T <: Seq[Data]](f: Int => T) = (0 until Width).map(f)

  def widthMap[T <: Data](f: Int => T) = (0 until Width).map(f)


  val tlbfa = TlbStorage(
    name = "tlbfa",
    sameCycle = false,
    ports = Width,
    nDups = nRespDups,
    nSets = 1,
    nWays = q.nWays,
    saveLevel = q.saveLevel,
    normalPage = true,
    superPage = true,
  )


  for (i <- 0 until Width) {
    tlbfa.r_req_apply(
      valid = io.requestor(i).req.valid,
      vpn = vpn(i),
      asid = csr_dup(i).satp.asid,
      i = i,
    )
  }

  tlbfa.sfence <> sfence
  tlbfa.csr <> csr_dup_2

  val refill_now = ptw_resp_v
  def TLBRead(i: Int) = {
    val (hit, ppn_, perm_) = tlbfa.r_resp_apply(i)
    // assert(!(normal_hit && super_hit && vmEnable_dup(i) && RegNext(req(i).valid, init = false.B)))

    val cmdReg = RegEnable(cmd(i), reqValid(i))
    val validReg = RegNext(reqValid(i))
    val offReg = RegEnable(reqAddr(i).off, reqValid(i))
    val sizeReg = RegEnable(req(i).bits.size, reqValid(i))

    /** *************** next cycle when two cycle is false******************* */
    val miss = !hit && vmEnable_dup(i)
//    val miss_sameCycle = (!hit_sameCycle || refill_now) && vmEnable_dup(i)
    hit.suggestName(s"hit_${i}")
    miss.suggestName(s"miss_${i}")

    val vaddr = SignExt(req(i).bits.vaddr, PAddrBits)
    req(i).ready := resp(i).ready
    resp(i).valid := validReg
//    resp(i).bits.miss := { if (q.missSameCycle) miss_sameCycle else miss }
    resp(i).bits.miss := miss
    resp(i).bits.ptwBack := ptw.resp.fire

    // for timing optimization, pmp check is divided into dynamic and static
    // dynamic: superpage (or full-connected reg entries) -> check pmp when translation done
    // static: 4K pages (or sram entries) -> check pmp with pre-checked results
    //    val pmp_paddr = Mux(vmEnable_dup(i), Cat(super_ppn(0), offReg), if (!q.sameCycle) RegNext(vaddr) else vaddr)
    val pmp_paddr = Mux(vmEnable_dup(i), Cat(ppn_(0), offReg), RegEnable(vaddr,reqValid(i)))
    pmp(i).valid := resp(i).valid
    pmp(i).bits.addr := pmp_paddr
    pmp(i).bits.size := sizeReg
    pmp(i).bits.cmd := cmdReg

    // duplicate resp part
    for (d <- 0 until nRespDups) {
      // ppn and perm from tlbfa resp
      val ppn = ppn_(d)
      val perm = perm_(d)

      val pf = perm.pf
      val af = perm.af
      val paddr = Cat(ppn, offReg)
      resp(i).bits.paddr(d) := Mux(vmEnable_dup(i), paddr, RegEnable(vaddr,reqValid(i)))

      val ldUpdate = !perm.a && TlbCmd.isRead(cmdReg) && !TlbCmd.isAmo(cmdReg) // update A/D through exception
      val stUpdate = (!perm.a || !perm.d) && (TlbCmd.isWrite(cmdReg) || TlbCmd.isAmo(cmdReg)) // update A/D through exception
      val instrUpdate = !perm.a && TlbCmd.isExec(cmdReg) // update A/D through exception
      val modeCheck = !(mode_dup(i) === ModeU && !perm.u || mode_dup(i) === ModeS && perm.u && (!priv.sum || ifecth))
      val ldPermFail = !(modeCheck && (perm.r || priv.mxr && perm.x))
      val stPermFail = !(modeCheck && perm.w)
      val instrPermFail = !(modeCheck && perm.x)
      val ldPf = (ldPermFail || pf) && (TlbCmd.isRead(cmdReg) && !TlbCmd.isAmo(cmdReg))
      val stPf = (stPermFail || pf) && (TlbCmd.isWrite(cmdReg) || TlbCmd.isAmo(cmdReg))
      val instrPf = (instrPermFail || pf) && TlbCmd.isExec(cmdReg)
      val fault_valid = vmEnable_dup(i)
      resp(i).bits.excp(d).pf.ld := (ldPf || ldUpdate) && fault_valid && !af
      resp(i).bits.excp(d).pf.st := (stPf || stUpdate) && fault_valid && !af
      resp(i).bits.excp(d).pf.instr := (instrPf || instrUpdate) && fault_valid && !af
      // NOTE: pf need && with !af, page fault has higher priority than access fault
      // but ptw may also have access fault, then af happens, the translation is wrong.
      // In this case, pf has lower priority than af

      // for tlb without sram, tlb will miss, pm should be ignored outsize
      resp(i).bits.excp(d).af.ld    := af && TlbCmd.isRead(cmdReg) && fault_valid
      resp(i).bits.excp(d).af.st    := af && TlbCmd.isWrite(cmdReg) && fault_valid
      resp(i).bits.excp(d).af.instr := af && TlbCmd.isExec(cmdReg) && fault_valid
    }

    (hit, miss, validReg)
  }

  val readResult = (0 until Width).map(TLBRead(_))
  val hitVec = readResult.map(_._1)
  val missVec = readResult.map(_._2)
  val validRegVec = readResult.map(_._3)

  // replacement
  def get_access(one_hot: UInt, valid: Bool): Valid[UInt] = {
    val res = Wire(Valid(UInt(log2Up(one_hot.getWidth).W)))
    res.valid := Cat(one_hot).orR && valid
    res.bits := OHToUInt(one_hot)
    res
  }


  val refill_idx = if (q.outReplace) {
    io.replace.page.access <> tlbfa.access
    io.replace.page.chosen_set := DontCare  // only fa
    io.replace.page.refillIdx
  } else {
    val re = ReplacementPolicy.fromString(q.replacer, q.nWays)
    re.access(tlbfa.access.map(_.touch_ways))
    re.way
  }

  val refill = ptw_resp_v && !sfence.valid && !satp.changed

  tlbfa.w_apply(
    valid = refill,
    wayIdx = refill_idx,
    data = ptw_resp,
  )

  // if sameCycle, just req.valid
  // if !sameCycle, add one more RegNext based on !sameCycle's RegNext
  // because sram is too slow and dtlb is too distant from dtlbRepeater
  for (i <- 0 until Width) {
    io.ptw.req(i).valid :=  need_RegNextInit(true, validRegVec(i) && missVec(i), false.B) &&
      !RegNext(refill, init = false.B) &&
      param_choose(true, !RegNext(RegNext(refill, init = false.B), init = false.B), true.B)
    //    io.ptw.req(i).bits.vpn := need_RegNext(!q.sameCycle, need_RegNext(!q.sameCycle, reqAddr(i).vpn))
    io.ptw.req(i).bits.vpn := need_RegNext(true, need_RegEnable(true, reqAddr(i).vpn, reqValid(i)))
  }
  io.ptw.resp.ready := true.B

  def need_RegNext[T <: Data](need: Boolean, data: T): T = {
    if (need) RegNext(data)
    else data
  }
  def need_RegEnable[T <: Data](need: Boolean, data: T, enable: Bool): T = {
    if (need) RegEnable(data,enable)
    else data
  }
  def need_RegNextInit[T <: Data](need: Boolean, data: T, init_value: T): T = {
    if (need) RegNext(data, init = init_value)
    else data
  }

  def param_choose[T <: Data](need: Boolean, truedata: T, falsedata: T): T = {
    if (need) truedata
    else falsedata
  }

  if (!q.shouldBlock) {
    for (i <- 0 until Width) {
      XSPerfAccumulate("first_access" + Integer.toString(i, 10), validRegVec(i) && vmEnable_dup.head && RegNext(req(i).bits.debug.isFirstIssue))
      XSPerfAccumulate("access" + Integer.toString(i, 10), validRegVec(i) && vmEnable_dup.head)
    }
    for (i <- 0 until Width) {
      XSPerfAccumulate("first_miss" + Integer.toString(i, 10), validRegVec(i) && vmEnable_dup.head && missVec(i) && RegNext(req(i).bits.debug.isFirstIssue))
      XSPerfAccumulate("miss" + Integer.toString(i, 10), validRegVec(i) && vmEnable_dup.head && missVec(i))
    }
  } else {
    // NOTE: ITLB is blocked, so every resp will be valid only when hit
    // every req will be ready only when hit
    for (i <- 0 until Width) {
      XSPerfAccumulate(s"access${i}", io.requestor(i).req.fire && vmEnable_dup.head)
      XSPerfAccumulate(s"miss${i}", ptw.req(i).fire)
    }

  }
  //val reqCycleCnt = Reg(UInt(16.W))
  //reqCycleCnt := reqCycleCnt + BoolStopWatch(ptw.req(0).fire, ptw.resp.fire || sfence.valid)
  //XSPerfAccumulate("ptw_req_count", ptw.req.fire)
  //XSPerfAccumulate("ptw_req_cycle", Mux(ptw.resp.fire, reqCycleCnt, 0.U))
  XSPerfAccumulate("ptw_resp_count", ptw.resp.fire)
  XSPerfAccumulate("ptw_resp_pf_count", ptw.resp.fire && ptw.resp.bits.pf)
  XSPerfAccumulate("ptw_resp_sp_count", ptw.resp.fire && !ptw.resp.bits.pf && (ptw.resp.bits.entry.level.get === 0.U || ptw.resp.bits.entry.level.get === 1.U))

  // Log
  for(i <- 0 until Width) {
    XSDebug(req(i).valid, p"req(${i.U}): (${req(i).valid} ${req(i).ready}) ${req(i).bits}\n")
    XSDebug(resp(i).valid, p"resp(${i.U}): (${resp(i).valid} ${resp(i).ready}) ${resp(i).bits}\n")
  }

  XSDebug(sfence.valid, p"Sfence: ${sfence}\n")
  XSDebug(ParallelOR(reqValid)|| ptw.resp.valid, p"CSR: ${csr_dup.head}\n")
  XSDebug(ParallelOR(reqValid) || ptw.resp.valid, p"vmEnable:${vmEnable_dup.head} hit:${Binary(VecInit(hitVec).asUInt)} miss:${Binary(VecInit(missVec).asUInt)}\n")
  for (i <- ptw.req.indices) {
    XSDebug(ptw.req(i).fire, p"L2TLB req:${ptw.req(i).bits}\n")
  }
  XSDebug(ptw.resp.valid, p"L2TLB resp:${ptw.resp.bits} (v:${ptw.resp.valid}r:${ptw.resp.ready}) \n")

  println(s"${q.name}: page: ${q.nWays} Ways fully-associative ${q.replacer.get}")

  //   // NOTE: just for simple tlb debug, comment it after tlb's debug
  // assert(!io.ptw.resp.valid || io.ptw.resp.bits.entry.tag === io.ptw.resp.bits.entry.ppn, "Simple tlb debug requires vpn === ppn")

  val perfEvents = if(!q.shouldBlock) {
    Seq(
      ("access", PopCount((0 until Width).map(i => vmEnable_dup.head && validRegVec(i)))              ),
      ("miss  ", PopCount((0 until Width).map(i => vmEnable_dup.head && validRegVec(i) && missVec(i)))),
    )
  } else {
    Seq(
      ("access", PopCount((0 until Width).map(i => io.requestor(i).req.fire))),
      ("miss  ", PopCount((0 until Width).map(i => ptw.req(i).fire))         ),
    )
  }
  generatePerfEvent()
}


class TlbReplace(Width: Int, q: TLBParameters)(implicit p: Parameters) extends TlbModule {
  val io = IO(new TlbReplaceIO(Width, q))

  // no sa
  val re = ReplacementPolicy.fromString(q.replacer, q.nWays)
  re.access(io.page.access.map(_.touch_ways))
  io.page.refillIdx := re.way
}