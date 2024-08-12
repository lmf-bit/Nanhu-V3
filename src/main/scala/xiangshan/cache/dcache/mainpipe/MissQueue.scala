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
import xiangshan._
import utils._
import xs.utils._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tilelink.ClientStates._
import freechips.rocketchip.tilelink.MemoryOpCategories._
import freechips.rocketchip.tilelink.TLPermissions._
import difftest._
import coupledL2.{AliasKey, DirtyKey, PrefetchKey}
import xs.utils.FastArbiter
import mem.AddPipelineReg
import xs.utils.perf.HasPerfLogging
import freechips.rocketchip.util.SeqToAugmentedSeq

class MissReqWoStoreData(implicit p: Parameters) extends DCacheBundle {
  val source = UInt(sourceTypeWidth.W)
  val cmd = UInt(M_SZ.W)
  val addr = UInt(PAddrBits.W)
  val vaddr = UInt(VAddrBits.W)
  //prefetch
  val pf_source = UInt(L1PfSourceBits.W)
  // store
  val full_overwrite = Bool()

  // which word does amo work on?
  val word_idx = UInt(log2Up(blockWords).W)
  val amo_data = UInt(DataBits.W)
  val amo_mask = UInt((DataBits / 8).W)

  val req_coh = new ClientMetadata
  val id = UInt(reqIdWidth.W)

  // For now, miss queue entry req is actually valid when req.valid && !cancel
  // * req.valid is fast to generate
  // * cancel is slow to generate, it will not be used until the last moment
  //
  // cancel may come from the following sources:
  // 1. miss req blocked by writeback queue: 
  //      a writeback req of the same address is in progress
  // 2. pmp check failed
  val cancel = Bool() // cancel is slow to generate, it will cancel missreq.valid

  def isLoad = source === LOAD_SOURCE.U
  def isStore = source === STORE_SOURCE.U
  def isAMO = source === AMO_SOURCE.U
  def hit = req_coh.isValid()
}

// class MissReqStoreData(implicit p: Parameters) extends DCacheBundle {
//   // store data and store mask will be written to miss queue entry 
//   // 1 cycle after req.fire and meta write
//   val store_data = UInt((cfg.blockBytes * 8).W)
//   val store_mask = UInt(cfg.blockBytes.W)
// }

class MissReq(implicit p: Parameters) extends MissReqWoStoreData {
  // store data and store mask will be written to miss queue entry 
  // 1 cycle after req.fire and meta write
  // val store_data = UInt((cfg.blockBytes * 8).W)
  // val store_mask = UInt(cfg.blockBytes.W)

  // def toMissReqStoreData(): MissReqStoreData = {
  //   val out = Wire(new MissReqStoreData)
  //   out.store_data := store_data
  //   out.store_mask := store_mask
  //   out
  // }

  def toMissReqWoStoreData(): MissReqWoStoreData = {
    val out = Wire(new MissReqWoStoreData)
    out.source := source
    out.cmd := cmd
    out.addr := addr
    out.vaddr := vaddr
    out.pf_source := pf_source
    out.full_overwrite := full_overwrite
    out.word_idx := word_idx
    out.amo_data := amo_data
    out.amo_mask := amo_mask
    out.req_coh := req_coh
    out.id := id
    out.cancel := cancel
    out
  }
}

class MissEntry(edge: TLEdgeOut)(implicit p: Parameters) extends DCacheModule with HasPerfLogging {
  val io = IO(new Bundle() {
    // MSHR ID
    val id = Input(UInt(log2Up(cfg.nMissEntries).W))
    // client requests
    // MSHR update request, MSHR state and addr will be updated when req.fire
    val req = Flipped(ValidIO(new MissReqWoStoreData))
    // store data and mask will be write to miss queue entry 1 cycle after req.fire
    // val req_data = Input(new MissReqStoreData)
    // allocate this entry for new req
    val primary_valid = Input(Bool())
    // this entry is free and can be allocated to new reqs
    val primary_ready = Output(Bool())
    // this entry is busy, but it can merge the new req
    val secondary_ready = Output(Bool())
    // this entry is busy and it can not merge the new req
    val secondary_reject = Output(Bool())

    val refill_to_ldq = ValidIO(new Refill)

    val sbuffer_id = Output(UInt(reqIdWidth.W))
    val req_source = Output(UInt(sourceTypeWidth.W))
    val need_refill_ldq = Output(Bool())

    // bus
    val mem_acquire = DecoupledIO(new TLBundleA(edge.bundle))
    val mem_grant = Flipped(DecoupledIO(new TLBundleD(edge.bundle)))
    val mem_finish = DecoupledIO(new TLBundleE(edge.bundle))

    // replace pipe
    val replace_pipe_req = DecoupledIO(new MainPipeReq)
    val replace_pipe_resp = Input(Bool())

    // main pipe: amo miss
    val main_pipe_req = DecoupledIO(new MainPipeReq)
    val main_pipe_resp = Input(Bool())

    val block_addr = ValidIO(UInt(PAddrBits.W))

    val debug_early_replace = ValidIO(new Bundle() {
      // info about the block that has been replaced
      val idx = UInt(idxBits.W) // vaddr
      val tag = UInt(tagBits.W) // paddr
    })
    val l2_pf_store_only = Input(Bool())
  })

  assert(!RegNext(io.primary_valid && !io.primary_ready))

  val req = Reg(new MissReqWoStoreData)
  val req_store_mask = Reg(UInt(cfg.blockBytes.W))
  val req_valid = RegInit(false.B)
  val set = addr_to_dcache_set(req.vaddr)

  val s_acquire = RegInit(true.B)
  val s_grantack = RegInit(true.B)
  val s_replace_req = RegInit(true.B)
  val s_mainpipe_req = RegInit(true.B)

  val w_grantfirst = RegInit(true.B)
  val w_grantlast = RegInit(true.B)
  val w_replace_resp = RegInit(true.B)
  val w_mainpipe_resp = RegInit(true.B)

  val release_entry = s_grantack && w_replace_resp && w_mainpipe_resp

  val acquire_not_sent = !s_acquire && !io.mem_acquire.ready
  val data_not_refilled = !w_grantfirst

  val error = RegInit(false.B)

  val should_refill_data_reg =  Reg(Bool())
  val should_refill_data = WireInit(should_refill_data_reg)

  // val full_overwrite = req.isStore && req_store_mask.andR
  val full_overwrite = Reg(Bool())

  val (_, _, refill_done, refill_count) = edge.count(io.mem_grant)
  val grant_param = Reg(UInt(TLPermissions.bdWidth.W))

  // refill data with store data, this reg will be used to store:
  // 1. store data (if needed), before l2 refill data
  // 2. store data and l2 refill data merged result (i.e. new cacheline taht will be write to data array)
  // val refill_and_store_data = Reg(Vec(blockRows, UInt(rowBits.W)))
  // raw data refilled to l1 by l2
  val refill_data_raw = Reg(Vec(blockBytes/beatBytes, UInt(beatBits.W)))

  // allocate current miss queue entry for a miss req
  val primary_fire = WireInit(io.req.valid && io.primary_ready && io.primary_valid && !io.req.bits.cancel)
  // merge miss req to current miss queue entry
  val secondary_fire = WireInit(io.req.valid && io.secondary_ready && !io.req.bits.cancel)

  when (release_entry && req_valid) {
    req_valid := false.B
  }

  // when (!s_write_storedata && req_valid) {
  //   // store data will be write to miss queue entry 1 cycle after req.fire
  //   s_write_storedata := true.B
  //   assert(RegNext(primary_fire || secondary_fire))
  // }

  when (primary_fire) {
    req_valid := true.B
    req := io.req.bits
    req.addr := get_block_addr(io.req.bits.addr)

    s_acquire := false.B
    s_grantack := false.B

    w_grantfirst := false.B
    w_grantlast := false.B

    full_overwrite := io.req.bits.isStore && io.req.bits.full_overwrite

    //replace: refill & replace
    when (!io.req.bits.isAMO) {
      s_replace_req := false.B
      w_replace_resp := false.B
    }

    when (io.req.bits.isAMO) {
      s_mainpipe_req := false.B
      w_mainpipe_resp := false.B
    }

    should_refill_data_reg := io.req.bits.isLoad
    error := false.B
  }

  when (secondary_fire) {
    assert(io.req.bits.req_coh.state <= req.req_coh.state)
    assert(!(io.req.bits.isAMO || req.isAMO))
    // use the most uptodate meta
    req.req_coh := io.req.bits.req_coh

    when (io.req.bits.isStore) {
      req := io.req.bits
      req.addr := get_block_addr(io.req.bits.addr)
      full_overwrite := io.req.bits.isStore && io.req.bits.full_overwrite
    }

    should_refill_data := should_refill_data_reg || io.req.bits.isLoad
    should_refill_data_reg := should_refill_data
  }

  when (io.mem_acquire.fire) {
    s_acquire := true.B
  }

  val hasData = RegInit(true.B)
  val isDirty = RegInit(false.B)
  when (io.mem_grant.fire) {
    w_grantfirst := true.B
    grant_param := io.mem_grant.bits.param
    when (edge.hasData(io.mem_grant.bits)) {
      // GrantData
      w_grantlast := w_grantlast || refill_done
      hasData := true.B
    }.otherwise {
      // Grant
      assert(full_overwrite)
      w_grantlast := true.B
      hasData := false.B
    }

    error := io.mem_grant.bits.denied || io.mem_grant.bits.corrupt || error
    isDirty := io.mem_grant.bits.echo.lift(DirtyKey).getOrElse(false.B)
  }

  when (io.mem_finish.fire) {
    s_grantack := true.B
  }

  when (io.replace_pipe_req.fire) {
    s_replace_req := true.B
  }

  when (io.replace_pipe_resp) {
    w_replace_resp := true.B
  }

  when (io.main_pipe_req.fire) {
    s_mainpipe_req := true.B
  }

  when (io.main_pipe_resp) {
    w_mainpipe_resp := true.B
  }

  def before_read_sent_can_merge(new_req: MissReqWoStoreData): Bool = {
    acquire_not_sent && req.isLoad && (new_req.isLoad || new_req.isStore)
  }

  def before_data_refill_can_merge(new_req: MissReqWoStoreData): Bool = {
    data_not_refilled && (req.isLoad || req.isStore) && new_req.isLoad
  }

  def is_alias_match(vaddr0: UInt, vaddr1: UInt): Bool = {
    require(vaddr0.getWidth == VAddrBits && vaddr1.getWidth == VAddrBits)
    if (blockOffBits + idxBits > pgIdxBits) {
      vaddr0(blockOffBits + idxBits - 1, pgIdxBits) === vaddr1(blockOffBits + idxBits - 1, pgIdxBits)
    } else {
      // no alias problem
      true.B
    }
  }

  def should_merge(new_req: MissReqWoStoreData): Bool = {
    val block_match = get_block(req.addr) === get_block(new_req.addr)
    val alias_match = is_alias_match(req.vaddr, new_req.vaddr)
    block_match && alias_match &&
    (
      before_read_sent_can_merge(new_req) ||
      before_data_refill_can_merge(new_req)
    )
  }

  // store can be merged before io.mem_acquire.fire
  // store can not be merged the cycle that io.mem_acquire.fire
  // load can be merged before io.mem_grant.fire
  //
  // TODO: merge store if possible? mem_acquire may need to be re-issued,
  // but sbuffer entry can be freed
  def should_reject(new_req: MissReqWoStoreData): Bool = {
    val block_match = get_block(req.addr) === get_block(new_req.addr)
    // val set_match = set === addr_to_dcache_set(new_req.vaddr)
    val alias_match = is_alias_match(req.vaddr, new_req.vaddr)

      req_valid && block_match &&
        (!before_read_sent_can_merge(new_req) &&
          !before_data_refill_can_merge(new_req) || !alias_match)

      
  }

  io.primary_ready := !req_valid
  io.secondary_ready := should_merge(io.req.bits)
  io.secondary_reject := should_reject(io.req.bits)

  // should not allocate, merge or reject at the same time
  when(io.req.valid){
    assert(PopCount(Seq(io.primary_ready, io.secondary_ready, io.secondary_reject)) <= 1.U)
  }
  //need to do
  io.refill_to_ldq.valid := RegNext(!w_grantlast && io.mem_grant.fire) && should_refill_data_reg
  io.refill_to_ldq.bits.addr := req.addr + RegNext((refill_count << refillOffBits))
  io.refill_to_ldq.bits.data := DontCare
  io.refill_to_ldq.bits.error := RegNext(io.mem_grant.bits.corrupt || io.mem_grant.bits.denied)
  io.refill_to_ldq.bits.refill_done := RegNext(refill_done && io.mem_grant.fire)
  io.refill_to_ldq.bits.hasdata := hasData
  io.refill_to_ldq.bits.data_raw := refill_data_raw.asUInt

  io.mem_acquire.valid := !s_acquire
  val grow_param = req.req_coh.onAccess(req.cmd)._2
  val acquireBlock = edge.AcquireBlock(
    fromSource = io.id,
    toAddress = req.addr,
    lgSize = (log2Up(cfg.blockBytes)).U,
    growPermissions = grow_param
  )._2
  val acquirePerm = edge.AcquirePerm(
    fromSource = io.id,
    toAddress = req.addr,
    lgSize = (log2Up(cfg.blockBytes)).U,
    growPermissions = grow_param
  )._2
  io.mem_acquire.bits := Mux(full_overwrite, acquirePerm, acquireBlock)
  private val prefecthBit = Mux(io.l2_pf_store_only, req.isStore, true.B)
  io.mem_acquire.bits.data := Cat(req.vaddr(13, 12), prefecthBit)
  require(nSets <= 256)

  io.mem_grant.ready := !w_grantlast && s_acquire

  val grantack = RegEnable(edge.GrantAck(io.mem_grant.bits), io.mem_grant.fire)
  assert(RegNext(!io.mem_grant.fire || edge.isRequest(io.mem_grant.bits)))
  io.mem_finish.valid := !s_grantack && w_grantfirst
  io.mem_finish.bits := grantack

  io.replace_pipe_req.valid := !s_replace_req && w_grantlast
  val replace = io.replace_pipe_req.bits
  replace := DontCare
  replace.miss := false.B
  replace.miss_id := io.id
  replace.miss_param := grant_param
  replace.miss_dirty := isDirty
  replace.probe := false.B
  replace.probe_need_data := false.B
  replace.source := req.source
  replace.cmd := req.cmd
  replace.vaddr := req.vaddr 
  replace.addr := req.addr  
  replace.store_mask := ~0.U(blockBytes.W)
  replace.refill := true.B
  replace.error := error
  replace.id := req.id


  io.main_pipe_req.valid := !s_mainpipe_req && w_grantlast
  io.main_pipe_req.bits := DontCare
  io.main_pipe_req.bits.miss := true.B
  io.main_pipe_req.bits.miss_id := io.id
  io.main_pipe_req.bits.miss_param := grant_param
  io.main_pipe_req.bits.miss_dirty := isDirty
  // io.main_pipe_req.bits.miss_way_en := req.way_en
  io.main_pipe_req.bits.probe := false.B
  io.main_pipe_req.bits.source := req.source
  io.main_pipe_req.bits.cmd := req.cmd
  io.main_pipe_req.bits.vaddr := req.vaddr
  io.main_pipe_req.bits.addr := req.addr
  io.main_pipe_req.bits.store_data := DontCare
  io.main_pipe_req.bits.store_mask := ~0.U(blockBytes.W)
  io.main_pipe_req.bits.word_idx := DontCare
  io.main_pipe_req.bits.amo_data := DontCare
  io.main_pipe_req.bits.amo_mask := DontCare
  io.main_pipe_req.bits.error := error
  io.main_pipe_req.bits.id := req.id

  // io.block_addr.valid := req_valid && w_grantlast && !w_refill_resp
  io.block_addr.valid := req_valid && w_grantlast && !w_replace_resp
  io.block_addr.bits := req.addr

  io.sbuffer_id := req.id
  io.req_source := req.source
  io.need_refill_ldq := should_refill_data

  // io.debug_early_replace.valid := BoolStopWatch(io.replace_pipe_resp, io.refill_pipe_req.fire)
  io.debug_early_replace.valid := io.replace_pipe_resp
  // io.debug_early_replace.bits.idx := addr_to_dcache_set(req.vaddr)
  // io.debug_early_replace.bits.tag := req.replace_tag
  io.debug_early_replace.bits := DontCare

  XSPerfAccumulate("miss_req_primary", primary_fire)
  XSPerfAccumulate("miss_req_merged", secondary_fire)
  XSPerfAccumulate("load_miss_penalty_to_use",
    should_refill_data &&
      BoolStopWatch(primary_fire, io.refill_to_ldq.valid, true)
  )
  XSPerfAccumulate("main_pipe_penalty", BoolStopWatch(io.main_pipe_req.fire, io.main_pipe_resp))
  XSPerfAccumulate("penalty_blocked_by_channel_A", io.mem_acquire.valid && !io.mem_acquire.ready)
  XSPerfAccumulate("penalty_waiting_for_channel_D", s_acquire && !w_grantlast && !io.mem_grant.valid)
  XSPerfAccumulate("penalty_waiting_for_channel_E", io.mem_finish.valid && !io.mem_finish.ready)
  XSPerfAccumulate("penalty_from_grant_to_refill", !w_replace_resp && w_grantlast)
  XSPerfAccumulate("soft_prefetch_number", primary_fire && io.req.bits.source === SOFT_PREFETCH.U)

  val (mshr_penalty_sample, mshr_penalty) = TransactionLatencyCounter(RegNext(primary_fire), release_entry)
  XSPerfHistogram("miss_penalty", mshr_penalty, mshr_penalty_sample, 0, 20, 1, true, true)
  XSPerfHistogram("miss_penalty", mshr_penalty, mshr_penalty_sample, 20, 100, 10, true, false)

  val load_miss_begin = primary_fire && io.req.bits.isLoad
  val refill_finished = RegNext(!w_grantlast && refill_done) && should_refill_data
  val (load_miss_penalty_sample, load_miss_penalty) = TransactionLatencyCounter(load_miss_begin, refill_finished) // not real refill finish time
  XSPerfHistogram("load_miss_penalty_to_use", load_miss_penalty, load_miss_penalty_sample, 0, 20, 1, true, true)
  XSPerfHistogram("load_miss_penalty_to_use", load_miss_penalty, load_miss_penalty_sample, 20, 100, 10, true, false)

  val (a_to_d_penalty_sample, a_to_d_penalty) = TransactionLatencyCounter(io.mem_acquire.fire, io.mem_grant.fire && refill_done)
  XSPerfHistogram("a_to_d_penalty", a_to_d_penalty, a_to_d_penalty_sample, 0, 20, 1, true, true)
  XSPerfHistogram("a_to_d_penalty", a_to_d_penalty, a_to_d_penalty_sample, 20, 100, 10, true, false)
}

class MissQueue(edge: TLEdgeOut)(implicit p: Parameters) extends DCacheModule with HasPerfEvents with HasPerfLogging {
  val io = IO(new Bundle {
    val hartId = Input(UInt(8.W))
    val req = Flipped(DecoupledIO(new MissReq))
    val refill_to_ldq = ValidIO(new Refill)

    val mem_acquire = DecoupledIO(new TLBundleA(edge.bundle))
    val mem_grant = Flipped(DecoupledIO(new TLBundleD(edge.bundle)))
    val mem_finish = DecoupledIO(new TLBundleE(edge.bundle))

    val refill_to_sbuffer = ValidIO(new RefillToSbuffer)

    val replace_pipe_req = DecoupledIO(new MainPipeReq)
    val replace_pipe_resp = Flipped(ValidIO(UInt(log2Up(cfg.nMissEntries).W)))

    val main_pipe_req = DecoupledIO(new MainPipeReq)
    val main_pipe_resp = Flipped(ValidIO(new AtomicsResp))

    // block probe
    val probe_addr = Input(UInt(PAddrBits.W))
    val probe_block = Output(Bool())

    val full = Output(Bool())

    // only for performance counter
    // This is valid when an mshr has finished replacing a block (w_replace_resp),
    // but hasn't received Grant from L2 (!w_grantlast)
    val debug_early_replace = Vec(cfg.nMissEntries, ValidIO(new Bundle() {
      // info about the block that has been replaced
      val idx = UInt(idxBits.W) // vaddr
      val tag = UInt(tagBits.W) // paddr
    }))
    val l2_pf_store_only = Input(Bool())

    val loadReqHandledResp = ValidIO(UInt(log2Up(cfg.nMissEntries).W))
  })
  
  // 128KBL1: FIXME: provide vaddr for l2

  val entries = Seq.fill(cfg.nMissEntries)(Module(new MissEntry(edge)))

  val refill_data_raw = Reg(Vec(blockBytes/beatBytes, UInt(beatBits.W)))
  val refill_ldq_data_raw = Reg(Vec(blockBytes/beatBytes, UInt(beatBits.W)))
  val difftest_data_raw = Reg(Vec(blockBytes/beatBytes, UInt(beatBits.W)))

  // val req_data_gen = io.req.bits.toMissReqStoreData()
  // val req_data_buffer = RegEnable(req_data_gen, io.req.valid)

  val primary_ready_vec = entries.map(_.io.primary_ready)
  val primary_valid_vec = entries.map(_.io.primary_valid)
  val secondary_ready_vec = entries.map(_.io.secondary_ready)
  val secondary_reject_vec = entries.map(_.io.secondary_reject)
  val probe_block_vec = entries.map { case e => e.io.block_addr.valid && e.io.block_addr.bits === io.probe_addr }

  val merge = Cat(secondary_ready_vec).orR
  val reject = Cat(secondary_reject_vec).orR
  val alloc = !reject && !merge && Cat(primary_ready_vec).orR
  val accept = alloc || merge

  val should_block_d = WireInit(false.B)
  val should_block_d_reg = RegNext(should_block_d, false.B)

  when(io.req.valid){
    assert(PopCount(secondary_ready_vec) <= 1.U)
  }
//  assert(RegNext(PopCount(secondary_reject_vec) <= 1.U))
  // It is possible that one mshr wants to merge a req, while another mshr wants to reject it.
  // That is, a coming req has the same paddr as that of mshr_0 (merge),
  // while it has the same set and the same way as mshr_1 (reject).
  // In this situation, the coming req should be merged by mshr_0
//  assert(RegNext(PopCount(Seq(merge, reject)) <= 1.U))

  def select_valid_one[T <: Bundle](
    in: Seq[DecoupledIO[T]],
    out: DecoupledIO[T],
    name: Option[String] = None): Unit = {

    if (name.nonEmpty) { out.suggestName(s"${name.get}_select") }
    out.valid := Cat(in.map(_.valid)).orR
    out.bits := ParallelMux(in.map(_.valid) zip in.map(_.bits))
    in.map(_.ready := out.ready) 
    assert(!RegNext(out.valid && PopCount(Cat(in.map(_.valid))) > 1.U))
  }

  io.mem_grant.ready := false.B
  io.refill_to_sbuffer.valid := false.B
  io.refill_to_sbuffer.bits := DontCare

  val refill_row_data = io.mem_grant.bits.data
  val (_, _, refill_done, refill_count) = edge.count(io.mem_grant)

  when(io.req.bits.isAMO && io.req.valid){
    refill_data_raw(0) := Cat(io.req.bits.amo_mask, io.req.bits.word_idx)
    refill_data_raw(1) := io.req.bits.amo_data
  }
  val hasData = edge.hasData(io.mem_grant.bits)

  entries.zipWithIndex.foreach {
    case (e, i) =>
      val former_primary_ready = if(i == 0)
        false.B 
      else
        Cat((0 until i).map(j => entries(j).io.primary_ready)).orR
      
      e.io.id := i.U
      e.io.l2_pf_store_only := io.l2_pf_store_only
      e.io.req.valid := io.req.valid
      e.io.primary_valid := io.req.valid && 
        !merge && 
        !reject && 
        !former_primary_ready &&
        e.io.primary_ready
      e.io.req.bits := io.req.bits.toMissReqWoStoreData()
      // e.io.req_data := req_data_buffer

      e.io.mem_grant.valid := false.B
      e.io.mem_grant.bits := DontCare
      when (io.mem_grant.bits.source === i.U) {
        when (e.io.req_source === STORE_SOURCE.U || e.io.req_source === AMO_SOURCE.U){
          io.mem_grant.ready := e.io.mem_grant.ready
          when(io.mem_grant.fire && hasData){
            io.refill_to_sbuffer.valid := true.B
            io.refill_to_sbuffer.bits.data := refill_row_data
            io.refill_to_sbuffer.bits.id := e.io.sbuffer_id
            io.refill_to_sbuffer.bits.refill_count := refill_count
            when(e.io.need_refill_ldq) {
              refill_ldq_data_raw(refill_count) := refill_row_data
              difftest_data_raw(refill_count) := refill_row_data
            }
          }
        }.elsewhen(e.io.req_source === LOAD_SOURCE.U) {
          io.mem_grant.ready := !should_block_d && e.io.mem_grant.ready
          when(io.mem_grant.fire && hasData){
            refill_data_raw(refill_count) := refill_row_data
            difftest_data_raw(refill_count) := refill_row_data
            refill_ldq_data_raw(refill_count) := refill_row_data

            when(refill_done){
              should_block_d_reg := true.B
            }
          }
        }
        e.io.mem_grant.bits := io.mem_grant.bits
        e.io.mem_grant.valid := Mux(io.mem_grant.ready, io.mem_grant.valid, false.B)
      }

      e.io.replace_pipe_resp := io.replace_pipe_resp.valid && io.replace_pipe_resp.bits === i.U
      e.io.main_pipe_resp := io.main_pipe_resp.valid && io.main_pipe_resp.bits.ack_miss_queue && io.main_pipe_resp.bits.miss_id === i.U

      io.debug_early_replace(i) := e.io.debug_early_replace
  }

  val loadReqHandledOH = primary_ready_vec.zip(primary_valid_vec).zip(secondary_ready_vec).map{
    case((priVld, priRdy), secRdy) => (priVld && priRdy) || secRdy
  }
  assert(PopCount(loadReqHandledOH) <= 1.U, "has more than 1 entry handled req")
  io.loadReqHandledResp.valid := loadReqHandledOH.reduce(_ || _) && io.req.valid && !io.req.bits.cancel
  io.loadReqHandledResp.bits := OHToUInt(VecInit(loadReqHandledOH).asUInt)

  io.req.ready := accept
  io.refill_to_ldq.valid := Cat(entries.map(_.io.refill_to_ldq.valid)).orR
  io.refill_to_ldq.bits := ParallelMux(entries.map(_.io.refill_to_ldq.valid) zip entries.map(_.io.refill_to_ldq.bits))
  io.refill_to_ldq.bits.data := refill_ldq_data_raw(RegNext(refill_count))

  TLArbiter.lowest(edge, io.mem_acquire, entries.map(_.io.mem_acquire):_*)
  TLArbiter.lowest(edge, io.mem_finish, entries.map(_.io.mem_finish):_*)

  // arbiter_with_pipereg_N_dup(entries.map(_.io.refill_pipe_req), io.refill_pipe_req,
  // io.refill_pipe_req_dup,
  // Some("refill_pipe_req"))
  // val out_refill_pipe_req = Wire(Decoupled(new RefillPipeReq))
  // val out_refill_pipe_req_ctrl = Wire(Decoupled(new RefillPipeReqCtrl))
  // out_refill_pipe_req_ctrl.valid := out_refill_pipe_req.valid
  // out_refill_pipe_req_ctrl.bits := out_refill_pipe_req.bits.getCtrl
  // out_refill_pipe_req.ready := out_refill_pipe_req_ctrl.ready
  // arbiter(entries.map(_.io.refill_pipe_req), out_refill_pipe_req, Some("refill_pipe_req"))
  // for (dup <- io.refill_pipe_req_dup) {
  //   AddPipelineReg(out_refill_pipe_req_ctrl, dup, false.B)
  // }
  // AddPipelineReg(out_refill_pipe_req, io.refill_pipe_req, false.B)

  fastArbiter(entries.map(_.io.replace_pipe_req), io.replace_pipe_req, Some("replace_pipe_req"))
  io.replace_pipe_req.bits.store_data := refill_data_raw.asUInt
  //load MSHR should wait replace.fire
  when(io.replace_pipe_req.fire && io.replace_pipe_req.bits.source === LOAD_SOURCE.U){
    should_block_d := false.B
  }.otherwise {
    should_block_d := should_block_d_reg
  }



  fastArbiter(entries.map(_.io.main_pipe_req), io.main_pipe_req, Some("main_pipe_req"))
  io.main_pipe_req.bits.word_idx := refill_data_raw(0)(log2Up(blockWords) - 1, 0)
  io.main_pipe_req.bits.amo_mask := refill_data_raw(0)(log2Up(blockWords) + (DataBits / 8) - 1, log2Up(blockWords))
  io.main_pipe_req.bits.amo_data := refill_data_raw(1)(DataBits - 1, 0)

  io.probe_block := Cat(probe_block_vec).orR

  io.full := ~Cat(entries.map(_.io.primary_ready)).andR

  if (env.EnableDifftest) {
    val difftest = DifftestModule(new DiffRefillEvent)
    difftest.coreid := io.hartId
    difftest.index := 1.U
    difftest.idtfr := 1.U
    difftest.valid := io.refill_to_ldq.valid && io.refill_to_ldq.bits.hasdata && io.refill_to_ldq.bits.refill_done
    // difftest.valid := false.B
    difftest.addr := io.refill_to_ldq.bits.addr
    difftest.data := difftest_data_raw.asUInt.asTypeOf(difftest.data)
  }

  XSPerfAccumulate("miss_req", io.req.fire)
  XSPerfAccumulate("miss_req_allocate", io.req.fire && alloc)
  XSPerfAccumulate("miss_req_merge_load", io.req.fire && merge && io.req.bits.isLoad)
  XSPerfAccumulate("miss_req_reject_load", io.req.valid && reject && io.req.bits.isLoad)
  XSPerfAccumulate("probe_blocked_by_miss", io.probe_block)
  val max_inflight = RegInit(0.U((log2Up(cfg.nMissEntries) + 1).W))
  val num_valids = PopCount(~Cat(primary_ready_vec).asUInt)
  when (num_valids > max_inflight) {
    max_inflight := num_valids
  }
  // max inflight (average) = max_inflight_total / cycle cnt
  XSPerfAccumulate("max_inflight", max_inflight)
  QueuePerf(cfg.nMissEntries, num_valids, num_valids === cfg.nMissEntries.U)
  io.full := num_valids === cfg.nMissEntries.U
  XSPerfHistogram("num_valids", num_valids, true.B, 0, cfg.nMissEntries, 1)

  val perfValidCount = RegNext(PopCount(entries.map(entry => (!entry.io.primary_ready))))
  val perfEvents = Seq(
    ("dcache_missq_req      ", io.req.fire),
    ("dcache_missq_1_4_valid", (perfValidCount < (cfg.nMissEntries.U/4.U))),
    ("dcache_missq_2_4_valid", (perfValidCount > (cfg.nMissEntries.U/4.U)) & (perfValidCount <= (cfg.nMissEntries.U/2.U))),
    ("dcache_missq_3_4_valid", (perfValidCount > (cfg.nMissEntries.U/2.U)) & (perfValidCount <= (cfg.nMissEntries.U*3.U/4.U))),
    ("dcache_missq_4_4_valid", (perfValidCount > (cfg.nMissEntries.U*3.U/4.U))),
  )
  generatePerfEvent()
}
