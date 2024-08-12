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

package xiangshan.cache

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.tilelink.TLPermissions._
import freechips.rocketchip.tilelink.{TLArbiter, TLBundleC, TLBundleD, TLEdgeOut}
import coupledL2.DirtyKey
import utils.{HasPerfEvents, HasTLDump}
import xs.utils.perf.HasPerfLogging
import xs.utils.sram.SRAMTemplate
import xiangshan.mem.AddPipelineReg

class WritebackReqCtrl(implicit p: Parameters) extends DCacheBundle {
  val param  = UInt(cWidth.W)
  val voluntary = Bool()
  val hasData = Bool()
  val dirty = Bool()

}

class WritebackReqWodata(implicit p: Parameters) extends WritebackReqCtrl {
  val addr = UInt(PAddrBits.W)

  def dump() = {
  }
}

class WritebackReqData(implicit p: Parameters) extends DCacheBundle {
  val data = UInt((cfg.blockBytes * 8).W)
}

class WritebackReq(implicit p: Parameters) extends WritebackReqWodata {
  val data = UInt((cfg.blockBytes * 8).W)
  val wayIdx = UInt(log2Up(DCacheWays).W)

  override def dump() = {
  }

  def toWritebackReqWodata(): WritebackReqWodata = {
    val out = Wire(new WritebackReqWodata)
    out.addr := addr
    out.param := param
    out.voluntary := voluntary
    out.hasData := hasData
    out.dirty := dirty
    out
  }

  def toWritebackReqCtrl(): WritebackReqCtrl = {
    val out = Wire(new WritebackReqCtrl)
    out.param := param
    out.voluntary := voluntary
    out.hasData := hasData
    out.dirty := dirty
    out
  }

  def toWritebackReqData(): WritebackReqData = {
    val out = Wire(new WritebackReqData)
    out.data := data
    out
  }
}

// While a Release sleeps and waits for a refill to wake it up,
// main pipe might update meta & data during this time.
// So the meta & data to be released need to be updated too.
class ReleaseUpdate(implicit p: Parameters) extends DCacheBundle {
  // only consider store here
  val addr = UInt(PAddrBits.W)
  val mask = UInt(DCacheBanks.W)
  val data = UInt((cfg.blockBytes * 8).W)
}

// To reduce fanout, writeback queue entry data is updated 1 cycle
// after ReleaseUpdate.fire
class WBQEntryReleaseUpdate(implicit p: Parameters) extends DCacheBundle {
  // only consider store here
  val addr = UInt(PAddrBits.W)
  val mask_delayed = UInt(DCacheBanks.W)
  val data_delayed = UInt((cfg.blockBytes * 8).W)
  val mask_orr = Bool()
}

// When a probe TtoB req enter dcache main pipe, check if that cacheline
// is waiting for release. If it is so, change TtoB to TtoN, set dcache
// coh to N.
class ProbeToBCheckReq(implicit p: Parameters) extends DCacheBundle {
  val addr = UInt(PAddrBits.W) // paddr from mainpipe s1
}

class ProbeToBCheckResp(implicit p: Parameters) extends DCacheBundle {
  val toN = Bool() // need to set dcache coh to N
}

class WritebackEntry(edge: TLEdgeOut)(implicit p: Parameters) extends DCacheModule with HasTLDump with HasPerfLogging
{
  val io = IO(new Bundle {
    val id = Input(UInt())
    // allocate this entry for new req
    val primary_valid = Input(Bool())
    // this entry is free and can be allocated to new reqs
    val primary_ready = Output(Bool())
    val primary_ready_dup = Vec(nDupWbReady, Output(Bool()))
    // this entry is busy, but it can merge the new req
    val secondary_valid = Input(Bool())
    val secondary_ready = Output(Bool())
    val req = Flipped(DecoupledIO(new WritebackReqWodata))
    // val req_data = Input(new WritebackReqData)

    val hasData = Output(Bool())

    val mem_release = DecoupledIO(new TLBundleC(edge.bundle))
    val mem_grant = Flipped(DecoupledIO(new TLBundleD(edge.bundle)))

    val block_addr  = Output(Valid(UInt()))

    val probe_ttob_check_req = Flipped(ValidIO(new ProbeToBCheckReq))
    val probe_ttob_check_resp = ValidIO(new ProbeToBCheckResp)
  })

  val s_invalid :: s_sleep :: s_release_req :: s_release_resp :: Nil = Enum(4)
  // ProbeAck:               s_invalid ->            s_release_req
  // ProbeAck merge Release: s_invalid ->            s_release_req  //有
  // Release:                s_invalid -> s_sleep -> s_release_req -> s_release_resp
  // Release merge ProbeAck: s_invalid -> s_sleep -> s_release_req  //无此种情况了
  //                        (change Release into ProbeAck when Release is not fired)
  //                     or: s_invalid -> s_sleep -> s_release_req -> s_release_resp -> s_release_req
  //                        (send a ProbeAck after Release transaction is over)
  val state = RegInit(s_invalid)
  val state_dup_0 = RegInit(s_invalid)
  val state_dup_1 = RegInit(s_invalid)
  val state_dup_for_mp = RegInit(VecInit(Seq.fill(nDupWbReady)(s_invalid)))

  // internal regs
  // remaining beats
  val remain = RegInit(0.U(refillCycles.W))
  val remain_dup_0 = RegInit(0.U(refillCycles.W))
  val remain_dup_1 = RegInit(0.U(refillCycles.W))
  val remain_set = WireInit(0.U(refillCycles.W))
  val remain_clr = WireInit(0.U(refillCycles.W))
  remain := (remain | remain_set) & ~remain_clr
  remain_dup_0 := (remain_dup_0 | remain_set) & ~remain_clr
  remain_dup_1 := (remain_dup_1 | remain_set) & ~remain_clr

  // writeback queue data
  // val data = Reg(UInt((cfg.blockBytes * 8).W))

  // writeback queue paddr
  val paddr_dup_0 = Reg(UInt(PAddrBits.W))
  val paddr_dup_1 = Reg(UInt(PAddrBits.W))
  val paddr_dup_2 = Reg(UInt(PAddrBits.W))

  // pending data write
  // !s_data_override means there is an in-progress data write
  val s_data_override = RegInit(true.B) 

  // there are valid request that can be sent to release bus
  val busy = remain.orR && s_data_override // have remain beats and data write finished

  val req  = Reg(new WritebackReqCtrl)

  // assign default signals to output signals
  io.req.ready := false.B
  io.mem_release.valid := false.B
  io.mem_release.bits  := DontCare
  io.mem_grant.ready   := false.B
  io.block_addr.valid  := state =/= s_invalid
  io.block_addr.bits   := paddr_dup_0

  s_data_override := true.B // data_override takes only 1 cycle

  when (state =/= s_invalid) {
    XSDebug("WritebackEntry: %d state: %d block_addr: %x\n", io.id, state, io.block_addr.bits)
  }

  def mergeData(old_data: UInt, new_data: UInt, wmask: UInt): UInt = {
    val full_wmask = FillInterleaved(64, wmask)
    (~full_wmask & old_data | full_wmask & new_data)
  }

  // --------------------------------------------------------------------------------
  // s_invalid: receive requests
  // new req entering
  when (io.req.valid && io.primary_valid && io.primary_ready) {
    assert (remain === 0.U)
    req := io.req.bits
    s_data_override := false.B
    // only update paddr when allocate a new missqueue entry
    paddr_dup_0 := io.req.bits.addr
    paddr_dup_1 := io.req.bits.addr
    paddr_dup_2 := io.req.bits.addr

    state := s_release_req
    state_dup_0 := s_release_req
    state_dup_1 := s_release_req
    state_dup_for_mp.foreach(_ := s_release_req)
    remain_set := Mux(io.req.bits.hasData, ~0.U(refillCycles.W), 1.U(refillCycles.W))
  }

  val merge = io.secondary_valid && io.secondary_ready


  // --------------------------------------------------------------------------------
  // while there beats remaining to be sent, we keep sending
  // which beat to send in this cycle?
  val beat = PriorityEncoder(remain_dup_0)

  val probeResponse = edge.ProbeAck(
    fromSource = io.id,
    toAddress = paddr_dup_1,
    lgSize = log2Ceil(cfg.blockBytes).U,
    reportPermissions = req.param
  )

  val probeResponseData = edge.ProbeAck(
    fromSource = io.id,
    toAddress = paddr_dup_1,
    lgSize = log2Ceil(cfg.blockBytes).U,
    reportPermissions = req.param,
    data = 0.U
  )

  val voluntaryRelease = edge.Release(
    fromSource = io.id,
    toAddress = paddr_dup_2,
    lgSize = log2Ceil(cfg.blockBytes).U,
    shrinkPermissions = req.param
  )._2

  val voluntaryReleaseData = edge.Release(
    fromSource = io.id,
    toAddress = paddr_dup_2,
    lgSize = log2Ceil(cfg.blockBytes).U,
    shrinkPermissions = req.param,
    data = 0.U
  )._2

  // voluntaryReleaseData.echo.lift(DirtyKey).foreach(_ := req.dirty)  //???
  voluntaryRelease.echo.lift(DirtyKey).foreach(_ := req.dirty)
  when(busy) {
    assert(!req.dirty || req.hasData)
  }

  io.mem_release.valid := busy
  io.mem_release.bits  := Mux(req.voluntary,
    Mux(req.hasData, voluntaryReleaseData, voluntaryRelease),
    Mux(req.hasData, probeResponseData, probeResponse))
  
  io.hasData := req.hasData

  when (io.mem_release.fire) { remain_clr := PriorityEncoderOH(remain_dup_1) }

  val (_, _, release_done, _) = edge.count(io.mem_release)


  // Because now wbq merges a same-addr req unconditionally, when the req to be merged comes too late,
  // the previous req might not be able to merge. Thus we have to handle the new req later after the
  // previous one finishes.
  // TODO: initiate these
  val release_later = RegInit(false.B)
  val c_already_sent = RegInit(false.B)
  def tmp_req() = new Bundle {
    val param = UInt(cWidth.W)
    val voluntary = Bool()
    val hasData = Bool()
    val dirty = Bool()

    def toWritebackReqCtrl = {
      val r = Wire(new WritebackReqCtrl())
      r.param := param
      r.voluntary := voluntary
      r.hasData := hasData
      r.dirty := dirty
      r
    }
  }
  val req_later = Reg(tmp_req())

  when (state_dup_0 === s_release_req) {
    when (io.mem_release.fire) {
      c_already_sent := !release_done
    }

    when (req.voluntary) {
      // The previous req is Release
      when (release_done) {
        state := s_release_resp
        state_dup_0 := s_release_resp
        state_dup_1 := s_release_resp
        state_dup_for_mp.foreach(_ := s_release_resp)
      }
      // merge a ProbeAck
      when (merge) {
        when (io.mem_release.fire || c_already_sent) {
          // too late to merge, handle the ProbeAck later
          release_later := true.B
          req_later.param := io.req.bits.param
          req_later.voluntary := io.req.bits.voluntary
          req_later.hasData := io.req.bits.hasData
          req_later.dirty := io.req.bits.dirty
        }.otherwise {
          // Release hasn't been sent out yet, change Release to ProbeAck
          req.voluntary := false.B
          req.hasData := req.hasData || io.req.bits.hasData
          req.dirty := req.dirty || io.req.bits.dirty
          remain_set := Mux(req.hasData || io.req.bits.hasData, ~0.U(refillCycles.W), 1.U(refillCycles.W))
        }
      }
    }.otherwise {
      // The previous req is ProbeAck
      when (merge) {
        release_later := true.B
        req_later.param := io.req.bits.param
        req_later.voluntary := io.req.bits.voluntary
        req_later.hasData := io.req.bits.hasData
        req_later.dirty := io.req.bits.dirty
      }

      when (release_done) {
        //probe --> release，state will not change
        when (merge) { 
          req := io.req.bits
          remain_set := Mux(req_later.hasData, ~0.U(refillCycles.W), 1.U(refillCycles.W))
          remain_clr := 0.U
          release_later := false.B
        }.elsewhen (release_later) {
          req := req_later.toWritebackReqCtrl
          remain_set := Mux(req_later.hasData, ~0.U(refillCycles.W), 1.U(refillCycles.W))
          remain_clr := 0.U
          release_later := false.B
        }.otherwise {
          state := s_invalid
          state_dup_0 := s_invalid
          state_dup_1 := s_invalid
          state_dup_for_mp.foreach(_ := s_invalid)
          release_later := false.B
        }
     }
    }
  }

  // --------------------------------------------------------------------------------
  // receive ReleaseAck for Releases
  when (state_dup_0 === s_release_resp) {
    io.mem_grant.ready := true.B

    when (merge) {
      release_later := true.B
      req_later.param := io.req.bits.param
      req_later.voluntary := io.req.bits.voluntary
      req_later.hasData := io.req.bits.hasData
      req_later.dirty := io.req.bits.dirty
    }
    when (io.mem_grant.fire) {
      // release finish --> probe?
      when (merge) {
        state := s_release_req
        state_dup_0 := s_release_req
        state_dup_1 := s_release_req
        state_dup_for_mp.foreach(_ := s_release_req)
        req := io.req.bits
        remain_set := Mux(io.req.bits.hasData, ~0.U(refillCycles.W), 1.U(refillCycles.W))
        release_later := false.B
      }.elsewhen(release_later) {
        state := s_release_req
        state_dup_0 := s_release_req
        state_dup_1 := s_release_req
        state_dup_for_mp.foreach(_ := s_release_req)
        req := req_later.toWritebackReqCtrl
        remain_set := Mux(req_later.hasData, ~0.U(refillCycles.W), 1.U(refillCycles.W))
        release_later := false.B
      }.otherwise {
        state := s_invalid
        state_dup_0 := s_invalid
        state_dup_1 := s_invalid
        state_dup_for_mp.foreach(_ := s_invalid)
        release_later := false.B
      }
    }
  }

  // When does this entry merge a new req?
  // 1. When this entry is free
  // 2. When this entry wants to release while still waiting for release_wakeup signal,
  //    and a probe req with the same addr comes. In this case we merge probe with release,
  //    handle this probe, so we don't need another release.
  io.primary_ready := state_dup_1 === s_invalid
  io.primary_ready_dup.zip(state_dup_for_mp).foreach { case (rdy, st) => rdy := st === s_invalid }
  io.secondary_ready := state_dup_1 =/= s_invalid && io.req.bits.addr === paddr_dup_0

  io.probe_ttob_check_resp.valid := RegNext(io.probe_ttob_check_req.valid) // for debug only
  io.probe_ttob_check_resp.bits.toN := state_dup_1 === s_sleep &&
    RegEnable(io.probe_ttob_check_req.bits.addr,io.probe_ttob_check_req.valid) === paddr_dup_0 &&
    RegNext(io.probe_ttob_check_req.valid)   //invalid check?


  when (!s_data_override && req.hasData) {
    // data := io.req_data.data
  }

  // assert(!RegNext(!s_data_merge && !s_data_override))

  // performance counters
  XSPerfAccumulate("wb_req", io.req.fire)
  XSPerfAccumulate("wb_release", state === s_release_req && release_done && req.voluntary)
  XSPerfAccumulate("wb_probe_resp", state_dup_0 === s_release_req && release_done && !req.voluntary)
  XSPerfAccumulate("wb_probe_ttob_fix", io.probe_ttob_check_resp.valid && io.probe_ttob_check_resp.bits.toN)
  XSPerfAccumulate("penalty_blocked_by_channel_C", io.mem_release.valid && !io.mem_release.ready)
  XSPerfAccumulate("penalty_waiting_for_channel_D", io.mem_grant.ready && !io.mem_grant.valid && state_dup_1 === s_release_resp)
}

class WritebackQueue(edge: TLEdgeOut)(implicit p: Parameters) extends DCacheModule with HasTLDump with HasPerfEvents with HasPerfLogging with HasDCacheParameters {
  val io = IO(new Bundle {
    val req = Flipped(DecoupledIO(new WritebackReq))
    val req_ready_dup = Vec(nDupWbReady, Output(Bool()))
    val mem_release = DecoupledIO(new TLBundleC(edge.bundle))
    val mem_grant = Flipped(DecoupledIO(new TLBundleD(edge.bundle)))

    val probe_ttob_check_req = Flipped(ValidIO(new ProbeToBCheckReq))
    val probe_ttob_check_resp = ValidIO(new ProbeToBCheckResp)

    val miss_req = Flipped(Valid(UInt()))
    val block_miss_req = Output(Bool())
  })

  require(cfg.nReleaseEntries > cfg.nMissEntries)

  val primary_ready_vec = Wire(Vec(cfg.nReleaseEntries, Bool()))
  val secondary_ready_vec = Wire(Vec(cfg.nReleaseEntries, Bool()))
  val accept = Cat(primary_ready_vec).orR
  val merge = Cat(secondary_ready_vec).orR
  val alloc = accept && !merge
  // When there are empty entries, merge or allocate a new entry.
  // When there is no empty entry, reject it even if it can be merged.
  io.req.ready := accept

  // assign default values to output signals
  io.mem_release.valid := false.B
  io.mem_release.bits  := DontCare
  io.mem_grant.ready   := false.B

  val beat_data = Reg(UInt(beatBits.W))
  val req_data = io.req.bits.toWritebackReqData().data

  val data = Module(new WbDuplicatedDataArray(nway = wbqWays))

  val out_mem_release_ctrl = Wire(DecoupledIO(new TLBundleC(edge.bundle)))
  
    data.io.read.valid := false.B
    data.io.read.bits := DontCare

    data.io.write.valid := false.B
    data.io.write.bits := DontCare
  


  require(isPow2(cfg.nMissEntries))
  val grant_source = io.mem_grant.bits.source
  val entries = Seq.fill(cfg.nReleaseEntries)(Module(new WritebackEntry(edge)))
  entries.zipWithIndex.foreach {
    case (entry, i) =>
      val former_primary_ready = if(i == 0)
        false.B
      else
        Cat((0 until i).map(j => entries(j).io.primary_ready)).orR
      val entry_id = (i + releaseIdBase).U

      entry.io.id := entry_id

      // entry req
      entry.io.req.valid := io.req.valid
      primary_ready_vec(i)   := entry.io.primary_ready
      secondary_ready_vec(i) := entry.io.secondary_ready
      entry.io.req.bits  := io.req.bits

      entry.io.primary_valid := alloc &&
        !former_primary_ready &&
        entry.io.primary_ready
      entry.io.secondary_valid := io.req.valid && accept

      entry.io.mem_grant.valid := (entry_id === grant_source) && io.mem_grant.valid
      entry.io.mem_grant.bits  := io.mem_grant.bits

      entry.io.probe_ttob_check_req := io.probe_ttob_check_req
      //when enq.valid, write data to sram
      when(entry.io.primary_valid && io.req.fire){
        data.io.write.valid := true.B
        data.io.write.bits.idx := i.U
        if(wbqWays == 1){
          data.io.write.bits.data := req_data
        } else {
          for(j <- 0 until wbqWays){
            data.io.write.bits.data(j) := req_data((j + 1) * beatBits - 1, j * beatBits)
          }
        }
      }
  }
  
  io.req_ready_dup.zipWithIndex.foreach { case (rdy, i) =>
    rdy := Cat(entries.map(_.io.primary_ready_dup(i))).orR
  }

  io.probe_ttob_check_resp.valid := RegNext(io.probe_ttob_check_req.valid) // for debug only
  io.probe_ttob_check_resp.bits.toN := VecInit(entries.map(e => e.io.probe_ttob_check_resp.bits.toN)).asUInt.orR

  assert(RegNext(!(io.mem_grant.valid && !io.mem_grant.ready)))
  io.mem_grant.ready := true.B

  val miss_req_conflict = VecInit(entries.map(e => e.io.block_addr.valid && e.io.block_addr.bits === io.miss_req.bits)).asUInt.orR
  io.block_miss_req := io.miss_req.valid && miss_req_conflict

  TLArbiter.robin(edge, out_mem_release_ctrl, entries.map(_.io.mem_release):_*)
  val (_, _, release_done, release_count) = edge.count(io.mem_release)

  AddPipelineReg(out_mem_release_ctrl, io.mem_release, !(data.io.read.ready || (io.mem_release.fire && !release_done)))

  out_mem_release_ctrl.ready := io.mem_release.ready && (data.io.read.ready || (io.mem_release.fire && !release_done)) //second beat can sent

  val hasDataVec = VecInit(entries.map(e => e.io.hasData))
  val id = UIntToOH(out_mem_release_ctrl.bits.source - releaseIdBase.U)
  val hasData = Mux1H(id, hasDataVec)
  val hasDataReg = RegNext(hasData)
  val ren = RegInit(true.B)
  assert(PopCount(id.asUInt) <= 1.U)

  //read data
  when(hasData && out_mem_release_ctrl.valid && ren){
      data.io.read.valid := true.B
      data.io.read.bits.idx := OHToUInt(id)
  }

  //data resp and save second beat
  when(RegNext(data.io.read.fire)){
    if (wbqWays == 1) {
      beat_data := data.io.resp(0)(refillCycles * beatBits - 1, beatBits)
    } else {
      beat_data := data.io.resp(1)
    }
  }

  //io.mem_release send req.data
  when(hasDataReg && io.mem_release.valid){
    when(release_count === 0.U){  //frist beat
      if (wbqWays == 1) {
        io.mem_release.bits.data := data.io.resp(0)(beatBits - 1, 0)
      } else {
      io.mem_release.bits.data := data.io.resp(0) //secode beat
      }
      ren := true.B
    }.elsewhen(release_count === (refillCycles - 1).U){
      io.mem_release.bits.data := beat_data
    }
  }

  when(data.io.read.fire){
    ren := false.B
  }
  // sanity check
  // print all input/output requests for debug purpose
  // print req
  when (io.req.fire) {
    io.req.bits.dump()
  }

  when (io.mem_release.fire) {
    io.mem_release.bits.dump
  }

  when (io.mem_grant.fire) {
    io.mem_grant.bits.dump
  }

  when (io.miss_req.valid) {
    XSDebug("miss_req: addr: %x\n", io.miss_req.bits)
  }

  when (io.block_miss_req) {
    XSDebug("block_miss_req\n")
  }

  // performance counters
  XSPerfAccumulate("wb_req", io.req.fire)

  // 添加entry的使用情况统计
  val free_count = PopCount(entries.map(_.io.primary_ready))
  val valid_count = 18.U - free_count
  for (i <- 0 to 18) {
    XSPerfAccumulate(s"wbq_${i}_valid", valid_count === i.U)
  }
  // 统计写回队列满了以后有请求想要发但失败的情况
  XSPerfAccumulate("wbq_full_req_failed", io.req.valid && !io.req.ready)


  val perfValidCount = RegNext(PopCount(entries.map(e => e.io.block_addr.valid)))
  val perfEvents = Seq(
    ("dcache_wbq_req      ", io.req.fire),
    ("dcache_wbq_1_4_valid", (perfValidCount < (cfg.nReleaseEntries.U/4.U))),
    ("dcache_wbq_2_4_valid", (perfValidCount > (cfg.nReleaseEntries.U/4.U)) & (perfValidCount <= (cfg.nReleaseEntries.U/2.U))),
    ("dcache_wbq_3_4_valid", (perfValidCount > (cfg.nReleaseEntries.U/2.U)) & (perfValidCount <= (cfg.nReleaseEntries.U*3.U/4.U))),
    ("dcache_wbq_4_4_valid", (perfValidCount > (cfg.nReleaseEntries.U*3.U/4.U))),
  )
  generatePerfEvent()
}

class DataReadReq(implicit p: Parameters) extends DCacheBundle {
  val idx = UInt((log2Up(cfg.nReleaseEntries)).W)
}

class DataWriteReq(implicit p: Parameters) extends DataReadReq with HasDCacheParameters {
  val data = Vec(wbqWays, UInt((CacheLineSize / wbqWays).W))
}

class WbDuplicatedDataArray(nway: Int, parentName: String = "Unkonw")(implicit p: Parameters) extends DCacheModule  with HasDCacheParameters{
  val io = IO(new Bundle() {
    val read = Flipped(DecoupledIO(new DataReadReq))
    val resp = Output(Vec(nway, UInt((CacheLineSize / nway).W)))
    val write = Flipped(DecoupledIO(new (DataWriteReq)))
  })

  val array = Module(new SRAMTemplate(UInt((CacheLineSize / nway).W), 
    set = cfg.nReleaseEntries, 
    way = nway,
    shouldReset = false, holdRead = false, singlePort = true,
    hasMbist = coreParams.hasMbist,
    hasShareBus = coreParams.hasShareBus,
    parentName = parentName + s"array_"
    ))
  
    
    array.io.w.req.valid := io.write.valid
    array.io.w.req.bits.apply(
      setIdx = io.write.bits.idx,
      data = io.write.bits.data,
      waymask = ~0.U(nway.W)
    )

    array.io.r.req.valid := io.read.fire
    array.io.r.req.bits.apply(setIdx = io.read.bits.idx)
    io.resp := array.io.r.resp.data
    io.read.ready := !io.write.valid
    io.write.ready := true.B 

}