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
import utils._
import xs.utils._
import xs.utils.perf.HasPerfLogging

import scala.math.min


class TLBFA(
  sameCycle: Boolean,
  ports: Int,
  nDups: Int,
  nSets: Int,
  nWays: Int,
  saveLevel: Boolean = false,
  normalPage: Boolean,
  superPage: Boolean
)(implicit p: Parameters) extends TlbModule with HasPerfEvents with HasPerfLogging {
  require(!(sameCycle && saveLevel))

  val io = IO(new TlbStorageIO(nSets, nWays, ports, nDups))
  io.r.req.map(_.ready := true.B)

  val v = RegInit(VecInit(Seq.fill(nWays)(false.B)))
  val entries = Reg(Vec(nWays, new TlbSectorEntry(normalPage, superPage)))
  val g = entries.map(_.perm.g)

  val isSuperPage = Wire(Vec(ports, Bool()))

  for (i <- 0 until ports) {
    val req = io.r.req(i)
    val resp = io.r.resp(i)
    val access = io.access(i)

    val vpn = req.bits.vpn
    val vpn_reg = if (sameCycle) vpn else RegEnable(vpn, req.fire)
    val vpn_gen_ppn = if(sameCycle || saveLevel) vpn else vpn_reg

    val refill_mask = if (sameCycle) 0.U(nWays.W) else Mux(io.w.valid, UIntToOH(io.w.bits.wayIdx), 0.U(nWays.W))
    val hitVec = VecInit((entries.zipWithIndex).zip(v zip refill_mask.asBools).map{case (e, m) => e._1.hit(vpn, io.csr.satp.asid) && m._1 && !m._2 })

    hitVec.suggestName("hitVec")

    val hitVecReg = if (sameCycle) hitVec else RegEnable(hitVec, req.fire)

    resp.valid := { if (sameCycle) req.valid else RegNext(req.valid) }
    resp.bits.hit := Cat(hitVecReg).orR
    if (nWays == 1) {
      for (d <- 0 until nDups) {
        resp.bits.ppn(d) := entries(0).genPPN(saveLevel, req.valid)(vpn_gen_ppn)
        resp.bits.perm(d) := entries(0).perm
      }
      isSuperPage(i) := entries(0).isSuperPage()
    } else {
      for (d <- 0 until nDups) {
        resp.bits.ppn(d) := ParallelMux(hitVecReg zip entries.map(_.genPPN(saveLevel, req.valid)(vpn_gen_ppn)))
        resp.bits.perm(d) := ParallelMux(hitVecReg zip entries.map(_.perm))
      }
      isSuperPage(i) := ParallelMux(hitVecReg zip entries.map(_.isSuperPage()))
    }

    access.sets := get_set_idx(vpn_reg(vpn_reg.getWidth-1, sectorTlbWidth), nSets) // no use
    access.touch_ways.valid := resp.valid && Cat(hitVecReg).orR
    access.touch_ways.bits := OHToUInt(hitVecReg)

    resp.bits.hit.suggestName("hit")
    resp.bits.ppn.suggestName("ppn")
    resp.bits.perm.suggestName("perm")
  }

  when (io.w.valid) {
    v(io.w.bits.wayIdx) := true.B
    entries(io.w.bits.wayIdx).apply(io.w.bits.data, io.csr.satp.asid)
  }

//  val refill_vpn_reg = RegNext(io.w.bits.data.entry.tag)
//  val refill_wayIdx_reg = RegNext(io.w.bits.wayIdx)
  val refill_vpn_reg = RegEnable(io.w.bits.data.entry.tag, io.w.valid)
  val refill_wayIdx_reg = RegEnable(io.w.bits.wayIdx, io.w.valid)
  when (RegNext(io.w.valid)) {
    io.access.map { access =>
      access.sets := get_set_idx(refill_vpn_reg(refill_vpn_reg.getWidth - 1, sectorTlbWidth), nSets) // no use in FA
      access.touch_ways.valid := true.B
      access.touch_ways.bits := refill_wayIdx_reg
    }
  }

  val sfence = io.sfence
  val sfence_vpn = sfence.bits.addr.asTypeOf(new VaBundle().cloneType).vpn
  val sfenceHit = entries.map(_.hit(sfence_vpn, sfence.bits.asid))
  val sfenceHit_noasid = entries.map(_.hit(sfence_vpn, sfence.bits.asid, ignoreAsid = true))
  when (io.sfence.valid) {
    when (sfence.bits.rs1) { // virtual address *.rs1 <- (rs1===0.U)
      when (sfence.bits.rs2) { // asid, but i do not want to support asid, *.rs2 <- (rs2===0.U)
        // all addr and all asid
        v.map(_ := false.B)
      }.otherwise {
        // all addr but specific asid
        v.zipWithIndex.map{ case (a,i) => a := a & (g(i) | !(entries(i).asid === sfence.bits.asid)) }
      }
    }.otherwise {
      when (sfence.bits.rs2) {
        // specific addr but all asid
        v.zipWithIndex.map{ case (a,i) => a := a & !sfenceHit_noasid(i) }
      }.otherwise {
        // specific addr and specific asid
        v.zipWithIndex.map{ case (a,i) => a := a & !(sfenceHit(i) && !g(i)) }
      }
    }
  }


  XSPerfAccumulate(s"access", io.r.resp.map(_.valid.asUInt).fold(0.U)(_ + _))
  XSPerfAccumulate(s"hit", io.r.resp.map(a => a.valid && a.bits.hit).fold(0.U)(_.asUInt + _.asUInt))
  for (i <- 0 until ports) {
    XSPerfAccumulate(s"accessPorts$i", io.r.resp(i).valid)
    XSPerfAccumulate(s"hitPorts$i", io.r.resp(i).valid && io.r.resp(i).bits.hit)
    XSPerfAccumulate(s"hitspPorts$i", io.r.resp(i).valid && io.r.resp(i).bits.hit && isSuperPage(i))
  }

  for (i <- 0 until nWays) {
    XSPerfAccumulate(s"access${i}", io.r.resp.zip(io.access.map(acc => UIntToOH(acc.touch_ways.bits))).map{ case (a, b) =>
      a.valid && a.bits.hit && b(i)}.fold(0.U)(_.asUInt + _.asUInt))
  }
  for (i <- 0 until nWays) {
    XSPerfAccumulate(s"refill${i}", io.w.valid && io.w.bits.wayIdx === i.U)
  }

  val perfEvents = Seq(
    ("tlbstore_access", io.r.resp.map(_.valid.asUInt).fold(0.U)(_ + _)                            ),
    ("tlbstore_hit   ", io.r.resp.map(a => a.valid && a.bits.hit).fold(0.U)(_.asUInt + _.asUInt)),
  )
  generatePerfEvent()

  println(s"tlb_fa: nSets${nSets} nWays:${nWays}")
}


object TlbStorage {
  def apply
  (
    name: String,
    sameCycle: Boolean,
    ports: Int,
    nDups: Int = 1,
    nSets: Int = 1,
    nWays: Int,
    saveLevel: Boolean = false,
    normalPage: Boolean,
    superPage: Boolean
  )(implicit p: Parameters) = {
    val storage = Module(new TLBFA(sameCycle, ports, nDups, nSets, nWays, saveLevel, normalPage, superPage))
    storage.suggestName(s"tlb_${name}_fa")
    storage.io
  }
}
