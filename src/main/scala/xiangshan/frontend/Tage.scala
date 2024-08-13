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

package xiangshan.frontend

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan._
import utils._
import xs.utils._
import xs.utils.mbist.MBISTPipeline
import xs.utils.sram.{FoldedSRAMTemplate, SRAMTemplate}

import scala.math.min
import scala.util.matching.Regex
import scala.{Tuple2 => &}
import os.followLink
import xs.utils.perf.HasPerfLogging

trait TageParams extends HasBPUConst with HasXSParameter {
  // println(BankTageTableInfos)
  val TageNTables = TageTableInfos.size
  // val BankTageNTables = BankTageTableInfos.map(_.size) // Number of tage tables
  // val UBitPeriod = 256
  val TageCtrBits = 3
  val TickWidth = 7

  val USE_ALT_ON_NA_WIDTH = 4
  val NUM_USE_ALT_ON_NA = 128
  def use_alt_idx(pc: UInt) = (pc >> instOffsetBits)(log2Ceil(NUM_USE_ALT_ON_NA)-1, 0)

  val TotalBits = TageTableInfos.map {
    case (s, h, t) => {
      s * (1+t+TageCtrBits+1)
    }
  }.sum

  def posUnconf(ctr: UInt) = ctr === (1 << (ctr.getWidth - 1)).U
  def negUnconf(ctr: UInt) = ctr === ((1 << (ctr.getWidth - 1)) - 1).U

  def unconf(ctr: UInt) = posUnconf(ctr) || negUnconf(ctr)

}

trait HasFoldedHistory {
  val histLen: Int
  def compute_folded_hist(hist: UInt, l: Int)(histLen: Int) = {
    if (histLen > 0) {
      val nChunks = (histLen + l - 1) / l
      val hist_chunks = (0 until nChunks) map {i =>
        hist(min((i+1)*l, histLen)-1, i*l)
      }
      ParallelXOR(hist_chunks)
    }
    else 0.U
  }
  val compute_folded_ghist = compute_folded_hist(_: UInt, _: Int)(histLen)
}

abstract class TageBundle(implicit p: Parameters)
  extends XSBundle with TageParams with BPUUtils

abstract class TageModule(implicit p: Parameters)
  extends XSModule with TageParams with BPUUtils
  {}



class TageReq(implicit p: Parameters) extends TageBundle {
  val pc = UInt(VAddrBits.W)
  val ghist = UInt(HistoryLength.W)
  val foldedHist = new AllFoldedHistories(foldedGHistInfos)
}

class TageResp_meta(implicit p: Parameters) extends TageBundle with TageParams {
  val ctr = UInt(TageCtrBits.W)
  val u = Bool()
}

class TageResp(implicit p: Parameters) extends TageResp_meta {
  // val ctr = UInt(TageCtrBits.W)
  // val u = Bool()
  val unconf = Bool()
}


class TageUpdate(implicit p: Parameters) extends TageBundle {
  val pc = UInt(VAddrBits.W)
  val ghist = UInt(HistoryLength.W)
  // update tag and ctr
  val mask = Bool()
  val takens = Bool()
  val alloc = Bool()
  val oldCtrs = UInt(TageCtrBits.W)
  // update u
  val uMask = Bool()
  val us = Bool()
  val reset_u = Bool()
}

class TageMeta(implicit p: Parameters)
  extends TageBundle with HasSCParameter
{
  val providers = ValidUndirectioned(UInt(log2Ceil(TageNTables).W))
  val providerResps = new TageResp_meta
  // val altProviders = Vec(numBr, ValidUndirectioned(UInt(log2Ceil(TageNTables).W)))
  // val altProviderResps = Vec(numBr, new TageResp)
  val altUsed = Bool()
  //val altDiffers = Bool()
  val basecnts = UInt(2.W)
  val allocates = UInt(TageNTables.W)
  //val takens = Bool()
  val scMeta = if (EnableSC) Some(new SCMeta(SCNTables)) else None
  val predCycle = if (!env.FPGAPlatform) Some(UInt(64.W)) else None
  val use_alt_on_na = if (!env.FPGAPlatform) Some(Bool()) else None

  def altPreds = basecnts(1)
  def allocateValid = allocates.orR

  def altDiffers = basecnts(1) =/= providerResps.ctr(TageCtrBits - 1)
  def takens = Mux(altUsed, basecnts(1), providerResps.ctr(TageCtrBits-1))
}

trait TBTParams extends HasXSParameter with TageParams {
  val BtSize = 2048
  val bypassEntries = 8
}


class TageBTable(parentName:String = "Unknown")(implicit p: Parameters) extends XSModule with TBTParams with BPUUtils {
  val io = IO(new Bundle {
    //#2410
    // val s0_fire = Input(Bool())
    // val s0_pc   = Input(UInt(VAddrBits.W))
    //#2410
    val req = Flipped(DecoupledIO(UInt(VAddrBits.W))) // s0_pc
    val s1_cnt     = Output(UInt(2.W))
    val update_mask = Input(Bool())
    val update_pc = Input(UInt(VAddrBits.W))
    val update_cnt  = Input(UInt(2.W))
    val update_takens = Input(Bool())
   // val update  = Input(new TageUpdate)
  })

  val bimAddr = new TableAddr(log2Up(BtSize), instOffsetBits)

  val bt = Module(new SRAMTemplate(UInt(2.W), set = BtSize, shouldReset = true, holdRead = true, bypassWrite = true,
    hasMbist = coreParams.hasMbist,
    hasShareBus = coreParams.hasShareBus,
    parentName = parentName
  ))
  val mbistPipeline = if(coreParams.hasMbist && coreParams.hasShareBus) {
    MBISTPipeline.PlaceMbistPipeline(1, s"${parentName}_mbistPipe", true)
  } else {
    None
  }

  val doing_reset = RegInit(true.B)
  val resetRow = RegInit(0.U(log2Ceil(BtSize).W))
  resetRow := resetRow + doing_reset
  when (resetRow === (BtSize-1).U) { doing_reset := false.B }

  //#2410
  io.req.ready := !doing_reset
  // val s0_idx = bimAddr.getIdx(io.s0_pc)
  // bt.io.r.req.valid := io.s0_fire
  val s0_pc = io.req.bits
  val s0_fire = io.req.valid
  val s0_idx = bimAddr.getIdx(s0_pc)
  bt.io.r.req.valid := s0_fire
  bt.io.r.req.bits.setIdx := s0_idx

  val s1_read = bt.io.r.resp.data.head

  //#2410
  // val s1_idx = RegEnable(s0_idx, io.s0_fire)
  val s1_idx = RegEnable(s0_idx, s0_fire)


  val per_br_ctr = s1_read
  io.s1_cnt := per_br_ctr

  // Update logic

  val u_idx = bimAddr.getIdx(io.update_pc)

  val newCtrs = Wire(UInt(2.W)) // physical bridx

  val wrbypass = Module(new WrBypass(UInt(2.W), bypassEntries, log2Up(BtSize))) // logical bridx
  wrbypass.io.wen := io.update_mask
  wrbypass.io.write_idx := u_idx
  wrbypass.io.write_data.head := newCtrs

  val oldCtrs =
    Mux(wrbypass.io.hit && wrbypass.io.hit_data.head.valid,
      wrbypass.io.hit_data.head.bits, io.update_cnt)

  newCtrs := satUpdate(oldCtrs, 2, io.update_takens)

  val updateWayMask = io.update_mask.asUInt

  bt.io.w.apply(
    valid = io.update_mask || doing_reset,
    data = Mux(doing_reset, 2.U(2.W), newCtrs),
    setIdx = Mux(doing_reset, resetRow, u_idx),
    waymask = Mux(doing_reset, 1.U(1.W), updateWayMask)
  )

}


class TageTable
(
  val nRows: Int, val histLen: Int, val tagLen: Int, val tableIdx: Int, parentName:String = "Unknown"
)(implicit p: Parameters)
  extends TageModule with HasFoldedHistory with HasPerfLogging {
  val io = IO(new Bundle() {
    val req = Flipped(DecoupledIO(new TageReq))
    val resps = Output(Valid(new TageResp))
    val update = Input(new TageUpdate)
  })

  class TageEntry() extends TageBundle {
    val valid = Bool()
    val tag = UInt(tagLen.W)
    val ctr = UInt(TageCtrBits.W)
  }

  val SRAM_SIZE = 256 // physical size
  require(nRows % SRAM_SIZE == 0)
  require(isPow2(numBr))
  val nRowsPerBr = nRows / numBr
  val nBanks = 8
  val bankSize = nRowsPerBr / nBanks
  val bankFoldWidth = if (bankSize >= SRAM_SIZE) bankSize / SRAM_SIZE else 1
  val uFoldedWidth = nRowsPerBr / SRAM_SIZE
  if (bankSize < SRAM_SIZE) {
    println(f"warning: tage table $tableIdx has small sram depth of $bankSize")
  }
  val bankIdxWidth = log2Ceil(nBanks)
  def get_bank_mask(idx: UInt) = VecInit((0 until nBanks).map(idx(bankIdxWidth-1, 0) === _.U))
  def get_bank_idx(idx: UInt) = idx >> bankIdxWidth
  def get_way_in_bank(idx: UInt) =
    if (log2Ceil(bankFoldWidth) > 0)
      (idx >> bankIdxWidth)(log2Ceil(bankFoldWidth)-1, 0)
    else
      0.U(1.W)


  // bypass entries for tage update
  val perBankWrbypassEntries = 8

  val idxFhInfo = (histLen, min(log2Ceil(nRowsPerBr), histLen))
  val tagFhInfo = (histLen, min(histLen, tagLen))
  val altTagFhInfo = (histLen, min(histLen, tagLen-1))
  val allFhInfos = Seq(idxFhInfo, tagFhInfo, altTagFhInfo)

  def getFoldedHistoryInfo = allFhInfos.filter(_._1 >0).toSet
  def compute_tag_and_hash(unhashed_idx: UInt, allFh: AllFoldedHistories) = {
    val idx_fh = allFh.getHistWithInfo(idxFhInfo).foldedHist
    val tag_fh = allFh.getHistWithInfo(tagFhInfo).foldedHist
    val alt_tag_fh = allFh.getHistWithInfo(altTagFhInfo).foldedHist
    // require(idx_fh.getWidth == log2Ceil(nRows))
    val idx = (unhashed_idx ^ idx_fh)(log2Ceil(nRowsPerBr)-1, 0)
    val tag = (unhashed_idx ^ tag_fh ^ (alt_tag_fh << 1).asUInt) (tagLen - 1, 0)
    (idx, tag)
  }

  def inc_ctr(ctr: UInt, taken: Bool): UInt = satUpdate(ctr, TageCtrBits, taken)
  
  if (EnableGHistDiff) {
    val idx_history = compute_folded_ghist(io.req.bits.ghist, log2Ceil(nRowsPerBr))
    val idx_fh = io.req.bits.foldedHist.getHistWithInfo(idxFhInfo)
    XSError(idx_history =/= idx_fh.foldedHist, p"tage table $tableIdx has different fh," +
      p" ghist: ${Binary(idx_history)}, fh: ${Binary(idx_fh.foldedHist)}\n")
  }
  // pc is start address of basic block, most 2 branch inst in block
  // def getUnhashedIdx(pc: UInt) = pc >> (instOffsetBits+log2Ceil(TageBanks))
  def getUnhashedIdx(pc: UInt): UInt = (pc >> instOffsetBits).asUInt
  // val s1_pc = io.req.bits.pc
  val req_unhashed_idx = getUnhashedIdx(io.req.bits.pc)

  val us = Module(new FoldedSRAMTemplate(Bool(), set=nRowsPerBr, width=uFoldedWidth, shouldReset=true, extraReset=true, holdRead=true, singlePort=true,
    hasMbist = coreParams.hasMbist,
    hasShareBus = coreParams.hasShareBus,
    parentName = parentName + "us_"
  ))
  us.extra_reset.get := io.update.reset_u


  val table_banks = Seq.tabulate(nBanks)(idx =>
    Module(new FoldedSRAMTemplate(new TageEntry, set=bankSize, width=bankFoldWidth, shouldReset=true, holdRead=true, singlePort=true,
      hasMbist = coreParams.hasMbist,
      hasShareBus = coreParams.hasShareBus,
      parentName = parentName + s"table${idx}_"
    )))

  val mbistTablePipeline = if(coreParams.hasMbist && coreParams.hasShareBus) {
    MBISTPipeline.PlaceMbistPipeline(1, s"${parentName}_mbistTablePipe")
  } else {
    None
  }


  val (s0_idx, s0_tag) = compute_tag_and_hash(req_unhashed_idx, io.req.bits.foldedHist)
  val s0_bank_req_1h = get_bank_mask(s0_idx)

    for (b <- 0 until nBanks) {
      table_banks(b).io.r.req.valid := io.req.fire && s0_bank_req_1h(b)
      table_banks(b).io.r.req.bits.setIdx := get_bank_idx(s0_idx)
    }

  us.io.r.req.valid := io.req.fire
  us.io.r.req.bits.setIdx := s0_idx


  val s1_unhashed_idx = RegEnable(req_unhashed_idx, io.req.fire)
  val s1_idx = RegEnable(s0_idx, io.req.fire)
  val s1_tag = RegEnable(s0_tag, io.req.fire)
  val s1_pc  = RegEnable(io.req.bits.pc, io.req.fire)
  val s1_bank_req_1h = RegEnable(s0_bank_req_1h, io.req.fire)
  val s1_bank_has_write_on_this_req = RegEnable(VecInit(table_banks.map(_.io.w.req.valid)), io.req.valid)

  val resp_invalid_by_write = Wire(Bool())
  
  val tables_r = table_banks.map(_.io.r.resp.data.head) // s1
  val unconfs = tables_r.map(r => WireInit(unconf(r.ctr))) // do unconf cal in parallel
  val hits = tables_r.map(r => r.tag === s1_tag && r.valid && !resp_invalid_by_write) // do tag compare in parallel
  
  val resp_selected = Mux1H(s1_bank_req_1h, tables_r)
  val unconf_selected = Mux1H(s1_bank_req_1h, unconfs)
  val hit_selected = Mux1H(s1_bank_req_1h, hits)
  resp_invalid_by_write := Mux1H(s1_bank_req_1h, s1_bank_has_write_on_this_req)


  val per_br_resp = resp_selected
  val per_br_unconf = unconf_selected
  val per_br_hit = hit_selected
  val per_br_u   = us.io.r.resp.data.head

  io.resps.valid := per_br_hit
  io.resps.bits.ctr := per_br_resp.ctr
  io.resps.bits.u := per_br_u
  io.resps.bits.unconf := per_br_unconf


//  if (EnableGHistDiff) {
//    val update_idx_history = compute_folded_ghist(io.update.ghist, log2Ceil(nRowsPerBr))
//    val update_idx_fh = io.update.foldedHist.getHistWithInfo(idxFhInfo)
//    XSError(update_idx_history =/= update_idx_fh.foldedHist && io.update.mask,
//      p"tage table $tableIdx has different fh when update," +
//      p" ghist: ${Binary(update_idx_history)}, fh: ${Binary(update_idx_fh.foldedHist)}\n")
//  }

  // Use fetchpc to compute hash
val updateFoldedHist = WireInit(0.U.asTypeOf(new AllFoldedHistories(foldedGHistInfos)))
  updateFoldedHist.getHistWithInfo(idxFhInfo).foldedHist := compute_folded_ghist(io.update.ghist, log2Ceil(nRowsPerBr))
  updateFoldedHist.getHistWithInfo(tagFhInfo).foldedHist := compute_folded_ghist(io.update.ghist, tagLen)
  updateFoldedHist.getHistWithInfo(altTagFhInfo).foldedHist := compute_folded_ghist(io.update.ghist, tagLen - 1)
  val per_bank_update_wdata = Wire(Vec(nBanks, new TageEntry)) // corresponds to physical branches

  val update_unhashed_idx = getUnhashedIdx(io.update.pc)
  val (update_idx, update_tag) = compute_tag_and_hash(update_unhashed_idx, updateFoldedHist)
  val update_req_bank_1h = get_bank_mask(update_idx)
  val update_idx_in_bank = get_bank_idx(update_idx)
  
  val per_bank_not_silent_update = Wire(Vec(nBanks, Bool())) // corresponds to physical branches
  val per_bank_update_way_mask = per_bank_not_silent_update.map(_ && io.update.mask)

  for (b <- 0 until nBanks) {
    table_banks(b).io.w.apply(
      valid   = per_bank_update_way_mask(b).orR && update_req_bank_1h(b),
      data    = per_bank_update_wdata(b),
      setIdx  = update_idx_in_bank.asUInt,
      waymask = per_bank_update_way_mask(b)
    )
  }

  //#2410
  // Power-on reset
  val powerOnResetState = RegInit(true.B)
  when(us.io.r.req.ready && table_banks.map(_.io.r.req.ready).reduce(_ && _)) {
    // When all the SRAM first reach ready state, we consider power-on reset is done
    powerOnResetState := false.B
  }
  // Do not use table banks io.r.req.ready directly
  // All the us & table_banks are single port SRAM, ready := !wen
  // We do not want write request block the whole BPU pipeline
  io.req.ready := !powerOnResetState
  val bank_conflict = (0 until nBanks).map(b => table_banks(b).io.w.req.valid && s0_bank_req_1h(b)).reduce(_||_)
  // io.req.ready := true.B //#2410
  // io.req.ready := !(io.update.mask && not_silent_update)
  // io.req.ready := !bank_conflict
  XSPerfAccumulate(f"tage_table_bank_conflict", bank_conflict)

  val update_u_idx = update_idx
  val update_u_way_mask = io.update.uMask
  val update_u_wdata = io.update.us

  us.io.w.apply(io.update.uMask, update_u_wdata, update_u_idx, update_u_way_mask)
  
  // remove silent updates
  def silentUpdate(ctr: UInt, taken: Bool) = {
    ctr.andR && taken || !ctr.orR && !taken
  }

  val bank_wrbypasses = Seq.fill(nBanks)(
    Module(new WrBypass(UInt(TageCtrBits.W), perBankWrbypassEntries, 1, tagWidth=tagLen))
  ) // let it corresponds to logical brIdx

  for (b <- 0 until nBanks) {
    val wrbypass_ctr = bank_wrbypasses(b).io.hit_data.head.bits
    val wrbypass_data_valid = bank_wrbypasses(b).io.hit && bank_wrbypasses(b).io.hit_data.head.valid

    per_bank_update_wdata(b).valid := true.B
    per_bank_update_wdata(b).tag := update_tag
    per_bank_update_wdata(b).ctr :=
      Mux(io.update.alloc,
        Mux(io.update.takens, 4.U, 3.U),
        Mux(wrbypass_data_valid,
          inc_ctr(wrbypass_ctr,      io.update.takens),
          inc_ctr(io.update.oldCtrs, io.update.takens)
        )
      )
    per_bank_not_silent_update(b) :=
      Mux(wrbypass_data_valid,
        !silentUpdate(wrbypass_ctr,      io.update.takens),
        !silentUpdate(io.update.oldCtrs, io.update.takens)) ||
      io.update.alloc

    bank_wrbypasses(b).io.wen := io.update.mask && update_req_bank_1h(b)
    bank_wrbypasses(b).io.write_idx := get_bank_idx(update_idx)
    bank_wrbypasses(b).io.write_tag.foreach(_ := update_tag)
    bank_wrbypasses(b).io.write_data.head := per_bank_update_wdata(b).ctr
  }

  for (b <- 0 until nBanks) {
    val wrbypass = bank_wrbypasses(b)
    XSPerfAccumulate(f"tage_table_bank_${b}_wrbypass_enq", io.update.mask && update_req_bank_1h(b) && !wrbypass.io.hit)
    XSPerfAccumulate(f"tage_table_bank_${b}_wrbypass_hit", io.update.mask && update_req_bank_1h(b) &&  wrbypass.io.hit)
  }


  for (b <- 0 until nBanks) {
    val not_silent_update = per_bank_not_silent_update(b)
    XSPerfAccumulate(f"tage_table_bank_${b}_real_updates",
      io.update.mask && update_req_bank_1h(b) && not_silent_update)
    XSPerfAccumulate(f"tage_table_bank_${b}_silent_updates_eliminated",
      io.update.mask && update_req_bank_1h(b) && !not_silent_update)
  }

  XSPerfAccumulate("tage_table_hits", PopCount(io.resps.valid))
  
  for (b <- 0 until nBanks) {
    XSPerfAccumulate(f"tage_table_bank_${b}_update_req", io.update.mask && update_req_bank_1h(b))
  }

  val u = io.update
  val b = PriorityEncoder(u.mask)
  val ub = PriorityEncoder(u.uMask)
  XSDebug(io.req.fire,
    p"tableReq: pc=0x${Hexadecimal(io.req.bits.pc)}, " +
    p"idx=$s0_idx, tag=$s0_tag\n")
  XSDebug(RegNext(io.req.fire) && per_br_hit,
    p"TageTableResp_br: idx=$s1_idx, hit:${per_br_hit}, " +
    p"ctr:${io.resps.bits.ctr}, u:${io.resps.bits.u}\n")
  XSDebug(io.update.mask,
    p"update Table_br: pc:${Hexadecimal(u.pc)}}, " +
    p"taken:${u.takens}, alloc:${u.alloc}, oldCtrs:${u.oldCtrs}\n")
  val bank = OHToUInt(update_req_bank_1h.asUInt, nBanks)
  XSDebug(io.update.mask,
    p"update Table: writing tag:$update_tag, " +
    p"ctr: ${per_bank_update_wdata(bank).ctr} in idx ${update_idx}\n")
  XSDebug(RegNext(io.req.fire) && !per_br_hit, p"TageTableResp: not hit!\n")


  // ------------------------------Debug-------------------------------------
  val valids = RegInit(VecInit(Seq.fill(nRows)(false.B)))
  when (io.update.mask) { valids(update_idx) := true.B }
  XSDebug("Table usage:------------------------\n")
  XSDebug("%d out of %d rows are valid\n", PopCount(valids), nRows.U)

}

abstract class BaseTage(implicit p: Parameters) extends BasePredictor with TageParams with BPUUtils {
}

class Tage(val parentName:String = "Unknown")(implicit p: Parameters) extends BaseTage {

  val resp_meta = Wire(new TageMeta)
  override val meta_size = resp_meta.getWidth
  val tables = TageTableInfos.zipWithIndex.map {
    case ((nRows, histLen, tagLen), i) => {
      val t = Module(new TageTable(nRows, histLen, tagLen, i, parentName = parentName + s"tagtable${i}_"))
      t.io.req.valid := io.s0_fire(1)
      t.io.req.bits.pc := s0_pc_dup(1)
      t.io.req.bits.foldedHist := io.in.bits.foldedHist(1)
      t.io.req.bits.ghist := io.in.bits.ghist
      t
    }
  }
  val bt = Module (new TageBTable(parentName = parentName + "bttable_"))
  // bt.io.s0_fire := io.s0_fire(1)
  // bt.io.s0_pc   := s0_pc_dup(1)
  //#2410
  bt.io.req.valid := io.s0_fire(1)
  bt.io.req.bits := s0_pc_dup(1)

  // #2462
  // val bankTickCtrDistanceToTops = Seq.fill(numBr)(RegInit((1 << (TickWidth-1)).U(TickWidth.W)))
  val bankTickCtrDistanceToTops = RegInit(((1 << TickWidth) - 1).U(TickWidth.W))
  val bankTickCtrs = RegInit(0.U(TickWidth.W))
  val useAltOnNaCtrs = RegInit(
    VecInit(Seq.fill(NUM_USE_ALT_ON_NA)((1 << (USE_ALT_ON_NA_WIDTH-1)).U(USE_ALT_ON_NA_WIDTH.W)))
  )

  val tage_fh_info = tables.map(_.getFoldedHistoryInfo).reduce(_++_).toSet
  override def getFoldedHistoryInfo = Some(tage_fh_info)

  val s1_resps = VecInit(tables.map(_.io.resps))

  //val s1_bim = io.in.bits.resp_in(0).s1.fullPred
  // val s2_bim = RegEnable(s1_bim, io.s1_fire)

  val debug_pc_s0 = s0_pc_dup(1)
  val debug_pc_s1 = RegEnable(s0_pc_dup(1), io.s0_fire(1))
  val debug_pc_s2 = RegEnable(debug_pc_s1, io.s1_fire(1))

  val s1_provideds        = Wire(Bool())
  val s1_providers        = Wire(UInt(log2Ceil(TageNTables).W))
  val s1_providerResps    = Wire(new TageResp)
  val s1_altUsed          = Wire(Bool())
  val s1_tageTakens       = Wire(Bool())
  //val s1_finalAltPreds    = Wire(Bool())
  val s1_basecnts         = Wire(UInt(2.W))
  val s1_useAltOnNa       = Wire(Bool())

  val s2_provideds        = RegEnable(s1_provideds, io.s1_fire(1))
  val s2_providers        = RegEnable(s1_providers, io.s1_fire(1))
  val s2_providerResps    = RegEnable(s1_providerResps, io.s1_fire(1))
  val s2_altUsed          = RegEnable(s1_altUsed, io.s1_fire(1))
  val s2_tageTakens_dup   = io.s1_fire.map(f => RegEnable(s1_tageTakens, f))
  //val s2_finalAltPreds    = RegEnable(s1_finalAltPreds, io.s1_fire(1))
  val s2_basecnts         = RegEnable(s1_basecnts, io.s1_fire(1))
  val s2_useAltOnNa       = RegEnable(s1_useAltOnNa, io.s1_fire(1))

  val s3_tageTakens_dup = RegEnable(VecInit(s2_tageTakens_dup), io.s2_fire(1))

  io.out := io.in.bits.resp_in(0)
  io.out.lastStageMeta := resp_meta.asUInt

  val resp_s2 = io.out.s2
  val resp_s3 = io.out.s3

  // Update logic
  val u_valid = io.update(dupForTageSC).valid
  val update = io.update(dupForTageSC).bits
  val updateValids = update.ftbEntry.brValid && u_valid && !update.ftbEntry.alwaysTaken
//  val updateFHist = update.specInfo.foldedHist

  val updateMeta = update.meta.asTypeOf(new TageMeta)

  val updateMask    = WireInit(0.U.asTypeOf(Vec(TageNTables, Bool())))
  val updateUMask   = WireInit(0.U.asTypeOf(Vec(TageNTables, Bool())))
  val updateResetU  = WireInit(0.U.asTypeOf(Bool())) // per predictor
  val updateTakens  = Wire(Vec(TageNTables, Bool()))
  val updateAlloc   = WireInit(0.U.asTypeOf(Vec(TageNTables, Bool())))
  val updateOldCtrs = Wire(Vec(TageNTables, UInt(TageCtrBits.W)))
  val updateU       = Wire(Vec(TageNTables, Bool()))
  val updatebcnt    = Wire(UInt(2.W))
  val baseupdate    = WireInit(0.U.asTypeOf(Bool()))
  val bUpdateTakens = Wire(Bool())
  updateTakens  := DontCare
  updateOldCtrs  := DontCare
  updateU       := DontCare

  val updateMisPreds = update.mispred_mask

  class TageTableInfo(implicit p: Parameters) extends XSBundle {
    val resp = new TageResp
    val tableIdx = UInt(log2Ceil(TageNTables).W)
    val use_alt_on_unconf = Bool()
  }
  // access tag tables and output meta info

  val useAltCtr = Mux1H(UIntToOH(use_alt_idx(s1_pc_dup(1)), NUM_USE_ALT_ON_NA), useAltOnNaCtrs)
  val useAltOnNa = useAltCtr(USE_ALT_ON_NA_WIDTH-1) // highest bit

  val inputRes = s1_resps.zipWithIndex.map{case (r, idx) =>
    val tableInfo = Wire(new TageTableInfo)
    tableInfo.resp := r.bits
    tableInfo.use_alt_on_unconf := r.bits.unconf && useAltOnNa
    tableInfo.tableIdx := idx.U(log2Ceil(TageNTables).W)
    (r.valid, tableInfo)
  }
  val providerInfo = ParallelPriorityMux(inputRes.reverse)
  val provided = inputRes.map(_._1).reduce(_||_)

  s1_provideds      := provided
  s1_providers      := providerInfo.tableIdx
  s1_providerResps  := providerInfo.resp

  resp_meta.providers.valid    := RegEnable(s2_provideds, io.s2_fire(1))
  resp_meta.providers.bits     := RegEnable(s2_providers, io.s2_fire(1))
  resp_meta.providerResps      := RegEnable(s2_providerResps, io.s2_fire(1))

  resp_meta.predCycle.foreach(_ := RegEnable(GTimer(), io.s2_fire(1)))
  resp_meta.use_alt_on_na.foreach(_ := RegEnable(s2_useAltOnNa, io.s2_fire(1)))

  // Create a mask fo tables which did not hit our query, and also contain useless entries
  // and also uses a longer history than the provider
  val allocatableSlots =
    RegEnable(
      VecInit(s1_resps.map(r => !r.valid && !r.bits.u)).asUInt &
        (~(LowerMask(UIntToOH(s1_providers), TageNTables) & Fill(TageNTables, s1_provideds))).asUInt,
      io.s1_fire(1)
    )

  resp_meta.allocates := RegEnable(allocatableSlots, io.s2_fire(1))

  s1_altUsed       := !provided || providerInfo.use_alt_on_unconf
  s1_tageTakens    := Mux(s1_altUsed , bt.io.s1_cnt(1), providerInfo.resp.ctr(TageCtrBits-1))
  //s1_finalAltPreds := bt.io.s1_cnt(1)
  s1_basecnts      := bt.io.s1_cnt
  s1_useAltOnNa    := providerInfo.use_alt_on_unconf

  resp_meta.altUsed    := RegEnable(s2_altUsed, io.s2_fire(1))
  //resp_meta.altDiffers := RegEnable(s2_finalAltPreds =/= s2_providerResps.ctr(TageCtrBits - 1), io.s2_fire(1)) // alt != provider
  //resp_meta.takens     := RegEnable(s2_tageTakens_dup(0), io.s2_fire(1))
  resp_meta.basecnts   := RegEnable(s2_basecnts, io.s2_fire(1))

  val tage_enable_dup = RegNext(dup(io.ctrl.tage_enable))
  for (tage_enable & fp & s2_tageTakens <- tage_enable_dup zip resp_s2.fullPred zip s2_tageTakens_dup) {
    when (tage_enable) {
      fp.br_taken := s2_tageTakens
    }
    dontTouch(tage_enable)
  }
  for (tage_enable & fp & s3_tageTakens <- tage_enable_dup zip resp_s3.fullPred zip s3_tageTakens_dup) {
    when (tage_enable) {
      fp.br_taken := s3_tageTakens
    }
  }

  //---------------- update logics below ------------------//
  val hasUpdate = updateValids
  val updateMispred = updateMisPreds.head
  val updateTaken = hasUpdate && update.br_taken

  val updateProvided     = updateMeta.providers.valid
  val updateProvider     = updateMeta.providers.bits
  val updateProviderResp = updateMeta.providerResps
  val updateProviderCorrect = updateProviderResp.ctr(TageCtrBits-1) === updateTaken
  val updateUseAlt = updateMeta.altUsed
  val updateAltDiffers = updateMeta.altDiffers
  val updateAltIdx = use_alt_idx(update.pc)
  val updateUseAltCtr = Mux1H(UIntToOH(updateAltIdx, NUM_USE_ALT_ON_NA), useAltOnNaCtrs)
  val updateAltPred = updateMeta.altPreds
  val updateAltCorrect = updateAltPred === updateTaken

  val updateProviderWeakTaken = posUnconf(updateProviderResp.ctr)
  val updateProviderWeaknotTaken = negUnconf(updateProviderResp.ctr)
  val updateProviderWeak = unconf(updateProviderResp.ctr)

  when (hasUpdate) {
    when (updateProvided && updateProviderWeak && updateAltDiffers) {
      val newCtr = satUpdate(updateUseAltCtr, USE_ALT_ON_NA_WIDTH, updateAltCorrect)
      useAltOnNaCtrs(updateAltIdx) := newCtr
    }
  }

  XSPerfAccumulate(f"tage_bank_use_alt_pred", hasUpdate && updateUseAlt)
  XSPerfAccumulate(f"tage_bank_alt_correct", hasUpdate && updateUseAlt && updateAltCorrect)
  XSPerfAccumulate(f"tage_bank_alt_wrong", hasUpdate && updateUseAlt && !updateAltCorrect)
  XSPerfAccumulate(f"tage_bank_alt_differs", hasUpdate && updateAltDiffers)
  XSPerfAccumulate(f"tage_bank_use_alt_on_na_ctr_updated", hasUpdate && updateAltDiffers && updateProvided && updateProviderWeak)
  XSPerfAccumulate(f"tage_bank_use_alt_on_na_ctr_inc", hasUpdate && updateAltDiffers && updateProvided && updateProviderWeak &&  updateAltCorrect)
  XSPerfAccumulate(f"tage_bank_use_alt_on_na_ctr_dec", hasUpdate && updateAltDiffers && updateProvided && updateProviderWeak && !updateAltCorrect)

  XSPerfAccumulate(f"tage_bank_na", hasUpdate && updateProvided && updateProviderWeak)
  XSPerfAccumulate(f"tage_bank_use_na_correct", hasUpdate && updateProvided && updateProviderWeak && !updateUseAlt && !updateMispred)
  XSPerfAccumulate(f"tage_bank_use_na_wrong",   hasUpdate && updateProvided && updateProviderWeak && !updateUseAlt &&  updateMispred)

  updateMeta.use_alt_on_na.foreach(uaon => XSPerfAccumulate(f"tage_bank_use_alt_on_na", hasUpdate && uaon))

  when (hasUpdate && updateProvided) {
    updateMask(updateProvider) := true.B
    updateUMask(updateProvider) := updateAltDiffers
    updateU(updateProvider) := updateProviderCorrect
    updateTakens(updateProvider) := updateTaken
    updateOldCtrs(updateProvider) := updateProviderResp.ctr
    updateAlloc(updateProvider) := false.B
  }

  // update base table if used base table to predict
  baseupdate := hasUpdate && updateUseAlt
  updatebcnt := updateMeta.basecnts
  bUpdateTakens := updateTaken

  val needToAllocate = hasUpdate && updateMispred && !(updateUseAlt && updateProviderCorrect && updateProvided)
  val allocatableMask = updateMeta.allocates
  val canAllocate = updateMeta.allocateValid

  val allocLFSR = LFSR64()(TageNTables - 1, 0)
  val longerHistoryTableMask = (~(LowerMask(UIntToOH(updateProvider), TageNTables) & Fill(TageNTables, updateProvided.asUInt))).asUInt
  val canAllocMask = allocatableMask & longerHistoryTableMask
  val allocFailureMask = (~allocatableMask).asUInt & longerHistoryTableMask
  val tickInc = PopCount(allocFailureMask) > PopCount(canAllocMask)
  val tickDec = PopCount(canAllocMask) > PopCount(allocFailureMask)
  val tickIncVal = PopCount(allocFailureMask) - PopCount(canAllocMask)
  val tickDecVal = PopCount(canAllocMask) - PopCount(allocFailureMask)
  val tickToPosSat = tickIncVal >= bankTickCtrDistanceToTops && tickInc
  val tickToNegSat = tickDecVal >= bankTickCtrs && tickDec

  val firstEntry = PriorityEncoder(canAllocMask)
  val maskedEntry = PriorityEncoder(canAllocMask & allocLFSR)
  val allocate = Mux(canAllocMask(maskedEntry), maskedEntry, firstEntry)


  when (needToAllocate) {
    // val allocate = updateMeta.allocates(i).bits
    when (tickInc) {
      when (tickToPosSat) {
        bankTickCtrs := ((1 << TickWidth) - 1).U
        bankTickCtrDistanceToTops := 0.U
      }.otherwise {
        bankTickCtrs := bankTickCtrs + tickIncVal
        bankTickCtrDistanceToTops := bankTickCtrDistanceToTops - tickIncVal
      }
    }.elsewhen (tickDec) {
      when (tickToNegSat) {
        bankTickCtrs := 0.U
        bankTickCtrDistanceToTops := ((1 << TickWidth) - 1).U
      }.otherwise {
        bankTickCtrs := bankTickCtrs - tickDecVal
        bankTickCtrDistanceToTops := bankTickCtrDistanceToTops + tickDecVal
      }
    }
    when (canAllocate) {
      updateMask(allocate) := true.B
      updateTakens(allocate) := updateTaken
      updateAlloc(allocate) := true.B
      updateUMask(allocate) := true.B
      updateU(allocate) := false.B
    }
    when (bankTickCtrs === ((1 << TickWidth) - 1).U) {
      bankTickCtrs := 0.U
      bankTickCtrDistanceToTops := ((1 << TickWidth) - 1).U
      updateResetU := true.B
    }
  }
  XSPerfAccumulate(f"tage_bank_update_allocate_failure", needToAllocate && !canAllocate)
  XSPerfAccumulate(f"tage_bank_update_allocate_success", needToAllocate &&  canAllocate)
  XSPerfAccumulate(s"tage_bank_mispred", hasUpdate && updateMispred)
  XSPerfAccumulate(s"tage_bank_reset_u", updateResetU)
  for (t <- 0 to TageNTables) {
    XSPerfAccumulate(f"tage_bank_tick_inc_${t}", needToAllocate && tickInc && tickIncVal === t.U)
    XSPerfAccumulate(f"tage_bank_tick_dec_${t}", needToAllocate && tickDec && tickDecVal === t.U)
  }


  for (i <- 0 until TageNTables) {
    tables(i).io.update.mask    := RegNext(updateMask(i), false.B)
    tables(i).io.update.takens  := RegEnable(updateTakens(i), false.B, updateValids)
    tables(i).io.update.alloc   := RegEnable(updateAlloc(i), false.B, updateValids)
    tables(i).io.update.oldCtrs := RegEnable(updateOldCtrs(i), 0.U, updateValids)

    tables(i).io.update.uMask   := RegNext(updateUMask(i), false.B)
    tables(i).io.update.us      := RegNext(updateU(i), false.B)
    tables(i).io.update.reset_u := RegNext(updateResetU, false.B)
    // use fetch pc instead of instruction pc
    tables(i).io.update.pc       := RegEnable(update.pc, 0.U, updateValids)
    tables(i).io.update.ghist := RegEnable(io.update(dupForTageSC).bits.ghist, 0.U, updateValids)
  }

  bt.io.update_mask := RegNext(baseupdate)
  bt.io.update_cnt := RegEnable(updatebcnt, updateValids(0))
  bt.io.update_pc := RegEnable(update.pc, updateValids(0))
  bt.io.update_takens := RegEnable(bUpdateTakens, updateValids(0))

  // all should be ready for req
  // io.s1_ready := tables.map(_.io.req.ready).reduce(_&&_)
  //#2410
  io.s1_ready := tables.map(_.io.req.ready).reduce(_ && _) && bt.io.req.ready

  XSPerfAccumulate(f"tage_write_blocks_read", !io.s1_ready)

  def pred_perf(name: String, cnt: UInt)   = XSPerfAccumulate(s"${name}_at_pred", cnt)
  def commit_perf(name: String, cnt: UInt) = XSPerfAccumulate(s"${name}_at_commit", cnt)
  def tage_perf(name: String, pred_cnt: UInt, commit_cnt: UInt) = {
    pred_perf(name, pred_cnt)
    commit_perf(name, commit_cnt)
  }

  // Debug and perf info
  for (i <- 0 until TageNTables) {
    val pred_i_provided = s2_provideds && s2_providers === i.U
    val commit_i_provided = updateProvided && updateProvider === i.U && updateValids
    tage_perf(
      s"bank_tage_table_${i}_provided",
      PopCount(pred_i_provided),
      PopCount(commit_i_provided)
    )
  }
  tage_perf(
    s"bank_tage_use_bim",
    PopCount(!s2_provideds),
    PopCount(!updateProvided && updateValids)
  )
  tage_perf(
    s"bank_tage_use_altpred",
    PopCount(s2_provideds && unconf(s2_providerResps.ctr)),
    PopCount(updateProvided &&
      unconf(updateMeta.providerResps.ctr) && updateValids)
  )
  tage_perf(
    s"bank_tage_provided",
    PopCount(s2_provideds),
    PopCount(updateProvided && updateValids)
  )

  XSDebug(updateValids, "update: pc=%x, cycle=%d, taken:%b, misPred:%d, bimctr:%d, pvdr(%d):%d, altDiff:%d, pvdrU:%d, pvdrCtr:%d, alloc:%b\n",
    update.pc, 0.U, update.br_taken, update.mispred_mask.head,
    0.U, updateMeta.providers.valid, updateMeta.providers.bits, updateMeta.altDiffers, updateMeta.providerResps.u,
    updateMeta.providerResps.ctr, updateMeta.allocates
  )

  val s2_resps = RegEnable(s1_resps, io.s1_fire(1))
  XSDebug("req: v=%d, pc=0x%x\n", io.s0_fire(1), s0_pc_dup(1))
  XSDebug("s1_fire:%d, resp: pc=%x\n", io.s1_fire(1), debug_pc_s1)
  XSDebug("s2_fireOnLastCycle: resp: pc=%x, target=%x, hits=%b, takens=%b\n",
    debug_pc_s2, io.out.s2.target(1), s2_provideds.asUInt, s2_tageTakens_dup(0).asUInt)


  for (i <- 0 until TageNTables) {
    XSDebug("bank_tage_table(%d): valid:%b, resp_ctr:%d, resp_us:%d\n",
      i.U, s2_resps(i).valid, s2_resps(i).bits.ctr, s2_resps(i).bits.u)
  }
}


class Tage_SC(parentName:String = "Unknown")(implicit p: Parameters) extends Tage(parentName) with HasSC {}
