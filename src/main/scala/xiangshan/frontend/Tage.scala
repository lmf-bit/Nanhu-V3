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
import javax.management.MBeanRegistrationException

trait TageParams extends HasBPUConst with HasXSParameter {
  val TageNTables = TageTableInfos.size
  val TageCtrBits = 3
  val TickWidth = 7

  val USE_ALT_ON_NA_WIDTH = 4
  val NUM_USE_ALT_ON_NA = 128
  def useAltIdx(pc: UInt) = (pc >> instOffsetBits)(log2Ceil(NUM_USE_ALT_ON_NA)-1, 0)

  val nBanks = 8
  val SRAM_SIZE = 256
  val bankIdxWidth = log2Ceil(nBanks)
  val altCtrsNum = 128
  val alterCtrBits = 4

  val TotalBits = TageTableInfos.map {
    case (s, h, t) => {
      s * (1+t+TageCtrBits+1)
    }
  }.reduce(_+_)

  def posUnconf(ctr: UInt) = ctr === (1 << (ctr.getWidth - 1)).U
  def negUnconf(ctr: UInt) = ctr === ((1 << (ctr.getWidth - 1)) - 1).U
  def unconf(ctr: UInt) = posUnconf(ctr) || negUnconf(ctr)

  def get_phy_br_idx(unhashed_idx: UInt, br_lidx: Int)  = 0.U
  def get_lgc_br_idx(unhashed_idx: UInt, br_pidx: UInt) = 0.U
  def updateCtr(oldCtr: UInt, ctrWidth: Int, taken: Bool): UInt = {
    val maxCtr = (((1 << ctrWidth) - 1).U)(ctrWidth-1, 0)
    val minCtr = 0.U
    val isNotUpdate = (oldCtr === maxCtr && taken) || (oldCtr === minCtr && !taken)
    val newCtr = MuxCase(oldCtr, Array(
      isNotUpdate -> oldCtr,
      taken       -> (oldCtr + 1.U),
      !taken      -> (oldCtr - 1.U)
    ))
    newCtr
  }
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
  val pc         = UInt(VAddrBits.W)
  val ghist      = UInt(HistoryLength.W)
  val foldedHist = new AllFoldedHistories(foldedGHistInfos)
}

class TageRespMeta(implicit p: Parameters) extends TageBundle with TageParams {
  val ctr = UInt(TageCtrBits.W)
  val u = Bool()
}

class TageResp(implicit p: Parameters) extends TageRespMeta {
  // val ctr    = UInt(TageCtrBits.W)
  // val u      = Bool()
  val unconf = Bool()
  val wayIdx = UInt(2.W)
}

class TageUpdate(implicit p: Parameters) extends TageBundle {
  val pc         = UInt(VAddrBits.W)
  val ghist      = UInt(HistoryLength.W)
  val mask       = Bool()
  val takens     = Bool()
  val alloc      = Bool()
  val oldCtrs    = UInt(TageCtrBits.W)
  val wayIdx     = UInt(2.W)
  val uMask      = Bool()
  val us         = Bool()
  val reset_u    = Bool()
}

class TageMeta(implicit p: Parameters) extends TageBundle with HasSCParameter
{
  val providers     = ValidUndirectioned(UInt(log2Ceil(TageNTables).W))
  val providerResps = new TageRespMeta
  val altUsed       = Bool()
  // val altDiffers    = Bool()
  val basecnts      = UInt(2.W)
  val allocates     = UInt(TageNTables.W)
  // val takens        = Bool()
  val scMeta        = if (EnableSC) Some(new SCMeta(SCNTables)) else None
  val pred_cycle    = if (!env.FPGAPlatform) Some(UInt(64.W)) else None
  val use_alt_on_na = if (!env.FPGAPlatform) Some(Bool()) else None
  val wayIdx        = UInt(2.W)

  def altPreds = basecnts(1)
  def allocateValid = allocates.orR
  def altDiffers = basecnts(1) =/= providerResps.ctr(TageCtrBits - 1)
  def takens = Mux(altUsed, basecnts(1), providerResps.ctr(TageCtrBits-1))
}

trait TBTParams extends HasXSParameter with TageParams {
  val BtSize = 2048
  val bypassEntries = 8
}


class TageBTable(parentName:String = "Unknown")(implicit p: Parameters) extends XSModule with TBTParams{
  val io = IO(new Bundle {
    val req          = Flipped(DecoupledIO(UInt(VAddrBits.W)))
    val cnt          = Output(UInt(2.W))
    val updateMask   = Input(Bool())
    val updatePC     = Input(UInt(VAddrBits.W))
    val updateCnt    = Input(UInt(2.W))
    val updateTakens = Input(Bool())
  })

  def satUpdate(old: UInt, len: Int, taken: Bool): UInt = {
    val oldSatTaken = old === ((1 << len)-1).U
    val oldSatNotTaken = old === 0.U
    Mux(oldSatTaken && taken, ((1 << len)-1).U,
      Mux(oldSatNotTaken && !taken, 0.U,
        Mux(taken, old + 1.U, old - 1.U)))
  }

  val bt = Module(new SRAMTemplate(UInt(2.W), set = BtSize, way=numBr, shouldReset = true,
    holdRead = true, bypassWrite = true,
    hasMbist = coreParams.hasMbist,
    hasShareBus = coreParams.hasShareBus,
    parentName = parentName
  ))
  val mbistPipeline = if(coreParams.hasMbist && coreParams.hasShareBus) {
    MBISTPipeline.PlaceMbistPipeline(1, s"${parentName}_mbistPipe", true)
  } else {
    None
  }

  // reset
  val btReset = RegInit(true.B)
  val resetRow = RegInit(0.U(log2Ceil(BtSize).W))
  resetRow := resetRow + btReset
  when (resetRow === (BtSize-1).U) { btReset := false.B }

  // read bt
  val addr = new TableAddr(log2Up(BtSize), instOffsetBits)
  io.req.ready := !btReset
  val s0PC   = io.req.bits
  val s0Fire = io.req.valid
  val s0Idx  = addr.getIdx(s0PC)
  bt.io.r.req.valid       := s0Fire
  bt.io.r.req.bits.setIdx := s0Idx
  val s1ReadData = bt.io.r.resp.data
  io.cnt := s1ReadData.head

  // Update logic
  val updtIdx = addr.getIdx(io.updatePC)
  val newCtrs = Wire(Vec(numBr, UInt(2.W)))

  val wrbypass = Module(new WrBypass(UInt(2.W), bypassEntries, log2Up(BtSize), numWays = numBr))
  wrbypass.io.wen        := io.updateMask
  wrbypass.io.writeIdx  := updtIdx
  wrbypass.io.writeWayMask.map(_ := io.updateMask)
  wrbypass.io.writeData := newCtrs
  val oldCtrs = Mux(wrbypass.io.hit && wrbypass.io.hitData(0).valid, wrbypass.io.hitData(0).bits,
                                                                      io.updateCnt)
  newCtrs(0) := satUpdate(oldCtrs, 2, io.updateTakens)

  bt.io.w.apply(
    valid   = io.updateMask || btReset,
    data    = Mux(btReset, VecInit(Seq.fill(numBr)(2.U(2.W))), newCtrs),
    setIdx  = Mux(btReset, resetRow, updtIdx),
    waymask = Mux(btReset, Fill(numBr, 1.U(1.W)).asUInt, io.updateMask.asUInt)
  )
}



class TageTable
(
  val nRows: Int, val histLen: Int, val tagLen: Int, val tableIdx: Int, parentName:String = "Unknown"
)(implicit p: Parameters)
  extends TageModule with HasFoldedHistory with HasPerfLogging {
  val io = IO(new Bundle() {
    val req = Flipped(DecoupledIO(new TageReq))
    val resp = Output(Valid(new TageResp))
    val update = Input(new TageUpdate)
  })

  class TageEntry() extends TageBundle {
    val valid = Bool()
    val tag = UInt(tagLen.W)
    val ctr = UInt(TageCtrBits.W)
  }

  require(nRows % SRAM_SIZE == 0)
  require(isPow2(numBr))
  val nRowsPerBr = nRows / numBr
  val bankSize = nRowsPerBr / nBanks
  val uFoldedWidth = nRowsPerBr / SRAM_SIZE
  val bankFoldWidth = if (bankSize >= SRAM_SIZE) bankSize / SRAM_SIZE else 1
  val perBankWrbypassEntries = 8
  if (bankSize < SRAM_SIZE) {
    println(f"warning: tage table $tableIdx has small sram depth of $bankSize")
  }
  
  def getBankMask(idx: UInt) = VecInit((0 until nBanks).map(idx(bankIdxWidth-1, 0) === _.U))
  def getBankIdx(idx: UInt) = idx >> bankIdxWidth

  val idxFhInfo = (histLen, min(log2Ceil(nRowsPerBr), histLen))
  val tagFhInfo = (histLen, min(histLen, tagLen))
  val altTagFhInfo = (histLen, min(histLen, tagLen-1))
  val allFhInfos = Seq(idxFhInfo, tagFhInfo, altTagFhInfo)

  def getFoldedHistoryInfo = allFhInfos.filter(_._1 >0).toSet
  def getHashedIdxTag(unhashed_idx: UInt, allFh: AllFoldedHistories) = {
    val idx_fh = allFh.getHistWithInfo(idxFhInfo).foldedHist
    val tag_fh = allFh.getHistWithInfo(tagFhInfo).foldedHist
    val alt_tag_fh = allFh.getHistWithInfo(altTagFhInfo).foldedHist
    val idx = (unhashed_idx ^ idx_fh)(log2Ceil(nRowsPerBr)-1, 0)
    val tag = (unhashed_idx ^ tag_fh ^ (alt_tag_fh << 1)) (tagLen - 1, 0)
    (idx, tag)
  }
  def incCtr(ctr: UInt, taken: Bool): UInt = satUpdate(ctr, TageCtrBits, taken)
  def getUnhashedIdx(pc: UInt): UInt = pc >> instOffsetBits
  def silentUpdate(ctr: UInt, taken: Bool) = {
    ctr.andR && taken || !ctr.orR && !taken
  }
  
  if (EnableGHistDiff) {
    val idx_history = compute_folded_ghist(io.req.bits.ghist, log2Ceil(nRowsPerBr))
    val idx_fh = io.req.bits.foldedHist.getHistWithInfo(idxFhInfo)
    XSError(idx_history =/= idx_fh.foldedHist, p"tage table $tableIdx has different fh," +
      p" ghist: ${Binary(idx_history)}, fh: ${Binary(idx_fh.foldedHist)}\n")
  }
 
  // val associating = 4
  // val associSetNum = nRowsPerBr / associating
  // val associSetNum = bankSize / associating
  // val us = Module(new SRAMTemplate(Bool(),
  //   set=associSetNum, way=associating,
  //   shouldReset=true, extraReset=true, holdRead=true, singlePort=true,
  //   hasMbist = coreParams.hasMbist,
  //   hasShareBus = coreParams.hasShareBus,
  //   parentName = parentName + "us_"
  // ))
  val us = Module(new FoldedSRAMTemplate(Bool(), set=nRowsPerBr, width=uFoldedWidth, way=numBr,
    shouldReset=true, extraReset=true, holdRead=true, singlePort=true,
    hasMbist = coreParams.hasMbist,
    hasShareBus = coreParams.hasShareBus,
    parentName = parentName + "us_"
  ))
  us.extra_reset.get := io.update.reset_u

  // val tableBanks = Seq.tabulate(nBanks)(idx =>
  //   Module(new SRAMTemplate(new TageEntry,
  //     set=associSetNum, way=associating,
  //     shouldReset=true, holdRead=true, singlePort=true,
  //     hasMbist = coreParams.hasMbist,
  //     hasShareBus = coreParams.hasShareBus,
  //     parentName = parentName + s"table${idx}_"
  //   )))
   val table_banks = Seq.tabulate(nBanks)(idx =>
    Module(new FoldedSRAMTemplate(new TageEntry, set=bankSize, width=bankFoldWidth, way=numBr,
      shouldReset=true, holdRead=true, singlePort=true,
      hasMbist = coreParams.hasMbist,
      hasShareBus = coreParams.hasShareBus,
      parentName = parentName + s"table${idx}_"
    )))

  val mbistTablePipeline = if(coreParams.hasMbist && coreParams.hasShareBus) {
    MBISTPipeline.PlaceMbistPipeline(1, s"${parentName}_mbistTablePipe")
  } else {
    None
  }

  // val replacer = ReplacementPolicy.fromString(Some("setplru"), 4, associSetNum)

  // read
  val reqUnhashedIdx       = getUnhashedIdx(io.req.bits.pc)
  val (reqS0Idx, reqS0Tag) = getHashedIdxTag(reqUnhashedIdx, io.req.bits.foldedHist)
  val reqS0Bank1h          = getBankMask(reqS0Idx)
  for (b <- 0 until nBanks) {
    table_banks(b).io.r.req.valid := (io.req.fire && reqS0Bank1h(b))
    table_banks(b).io.r.req.bits.setIdx := getBankIdx(reqS0Idx)
  }
  us.io.r.req.valid := io.req.fire
  us.io.r.req.bits.setIdx := reqS0Idx

  val s1Idx       = RegEnable(reqS0Idx, io.req.fire)
  val s1Tag       = RegEnable(reqS0Tag, io.req.fire)
  val s1Bank1h    = RegEnable(reqS0Bank1h, io.req.fire)
  val s1ReadWrite = RegEnable(VecInit(table_banks.map(_.io.w.req.valid)), io.req.valid)
  val respInvalidByWrite = Wire(Bool())
  respInvalidByWrite := Mux1H(s1Bank1h, s1ReadWrite)
  
  val reqTageData      = table_banks.map(_.io.r.resp.data)
  val tablesBankValids = reqTageData.map(bank => VecInit(bank.map(_.valid)))
  val unconfs          = reqTageData.map(r => VecInit(r.map(e => WireInit(unconf(e.ctr)))))
  val hits             = reqTageData.map(r =>
    VecInit(r.map(e => e.tag === s1Tag && e.valid && !respInvalidByWrite)))
  
  val setRespVec   = Mux1H(s1Bank1h, reqTageData)
  val setUnconfVec = Mux1H(s1Bank1h, unconfs)
  val setHitsVec   = Mux1H(s1Bank1h, hits)
  val isS1Hit      = setHitsVec.reduce(_||_)
  val s1HitWayIdx  = PriorityEncoder(setHitsVec)

  io.resp.valid       := isS1Hit
  io.resp.bits.ctr    := setRespVec(0).ctr
  io.resp.bits.u      := us.io.r.resp.data(0)
  io.resp.bits.unconf := setUnconfVec(0)
  io.resp.bits.wayIdx := 0.U

  // if (EnableGHistDiff) {
  //   val update_idx_history = compute_folded_ghist(io.update.ghist, log2Ceil(nRowsPerBr))
  //   val update_idx_fh = io.update.foldedHist.getHistWithInfo(idxFhInfo)
  //   XSError(update_idx_history =/= update_idx_fh.foldedHist && io.update.mask.reduce(_||_),
  //     p"tage table $tableIdx has different fh when update," +
  //     p" ghist: ${Binary(update_idx_history)}, fh: ${Binary(update_idx_fh.foldedHist)}\n")
  // }

  // update
  val updateFoldedHist = WireInit(0.U.asTypeOf(new AllFoldedHistories(foldedGHistInfos)))
  updateFoldedHist.getHistWithInfo(idxFhInfo).foldedHist    :=
    compute_folded_ghist(io.update.ghist, log2Ceil(nRowsPerBr))
  updateFoldedHist.getHistWithInfo(tagFhInfo).foldedHist    :=
    compute_folded_ghist(io.update.ghist, tagLen)
  updateFoldedHist.getHistWithInfo(altTagFhInfo).foldedHist :=
    compute_folded_ghist(io.update.ghist, tagLen - 1)

  val updtBanksWdata     = Wire(Vec(nBanks,  new TageEntry))
  val updtUnhashedIdx    = getUnhashedIdx(io.update.pc)
  val (updtIdx, updtTag) = getHashedIdxTag(updtUnhashedIdx, updateFoldedHist)
  val updtBank1h         = getBankMask(updtIdx)
  val updtBankIdx        = getBankIdx(updtIdx)
  val writeWayIdx  = 0.U //Mux(io.update.alloc(0), replacer.way(updtIdx), io.update.wayIdx)
  // val writeWayMask = UIntToOH(writeWayIdx, associating)
  // val touchSetIdx  = Seq.fill(1)(Wire(UInt(log2Ceil(associSetNum).W)))
  // val touchWayIdx  = Seq.fill(1)(Wire(Valid(UInt(2.W))))
  // touchSetIdx(0)       := Mux(io.update.mask(0), updtIdx, s1Idx)
  // touchWayIdx(0).valid := io.update.mask(0) || (RegNext(io.req.fire) && isS1Hit) || io.update.alloc(0)
  // touchWayIdx(0).bits  := Mux(io.update.alloc(0) || io.update.mask(0), writeWayIdx, s1HitWayIdx)
  // replacer.access(touchSetIdx, touchWayIdx)
  
  val notSilentUpdate = Wire(Vec(nBanks, Bool()))
  val updteWayMask = VecInit((0 until nBanks).map(a => io.update.mask && notSilentUpdate(a)))
  val wrBypasses = Seq.fill(nBanks)(
    Module(new WrBypass(UInt(TageCtrBits.W), perBankWrbypassEntries, 1, tagWidth=tagLen)))

  for(a <- 0 until nBanks) {
    val wrBypassCtr       = wrBypasses(a).io.hitData(0).bits
    val wrBypassDataValid = wrBypasses(a).io.hit && wrBypasses(a).io.hitData(0).valid        
    updtBanksWdata(a).valid := true.B
    updtBanksWdata(a).tag   := updtTag
    updtBanksWdata(a).ctr   := Mux(io.update.alloc, Mux(io.update.takens, 4.U, 3.U),
      Mux(wrBypassDataValid, incCtr(wrBypassCtr,          io.update.takens),
                             incCtr(io.update.oldCtrs,    io.update.takens)))
    notSilentUpdate(a) := Mux(wrBypassDataValid,
      !silentUpdate(wrBypassCtr,          io.update.takens),
      !silentUpdate(io.update.oldCtrs, io.update.takens)) || io.update.alloc
    wrBypasses(a).io.wen       := io.update.mask && updtBank1h(a)
    wrBypasses(a).io.writeIdx := getBankIdx(updtIdx)
    wrBypasses(a).io.writeTag.map(_ := updtTag)
    wrBypasses(a).io.writeData(0) := updtBanksWdata(a).ctr
  }

  // write
  for (b <- 0 until nBanks) {
    table_banks(b).io.w.apply(
      valid   = updteWayMask(b) && updtBank1h(b),
      data    = updtBanksWdata(b),
      setIdx  = updtBankIdx,
      waymask = 1.U
    )
  }
  us.io.w.apply(io.update.uMask, io.update.us, updtIdx, 1.U)

  val powerOnResetState = RegInit(true.B)
  when(us.io.r.req.ready && table_banks.map(_.io.r.req.ready).reduce(_ && _)) {
    powerOnResetState := false.B
  }
 
  io.req.ready := !powerOnResetState
  val bank_conflict = (0 until nBanks).map(b => table_banks(b).io.w.req.valid && reqS0Bank1h(b)).reduce(_||_)
  XSPerfAccumulate(f"tage_table_bank_conflict", bank_conflict)

  for (i <- 0 until numBr) {
    for (b <- 0 until nBanks) {
      val wrbypass = wrBypasses(b)
      XSPerfAccumulate(f"tage_table_bank_${b}_wrbypass_enq_$i", io.update.mask(i) && updtBank1h(b) && !wrbypass.io.hit)
      XSPerfAccumulate(f"tage_table_bank_${b}_wrbypass_hit_$i", io.update.mask(i) && updtBank1h(b) &&  wrbypass.io.hit)
    }
  }

  for (b <- 0 until nBanks) {
    val not_silent_update = notSilentUpdate(b)
    XSPerfAccumulate(f"tage_table_bank_${b}_real_updates",
      io.update.mask && updtBank1h(b) && not_silent_update(0))
    XSPerfAccumulate(f"tage_table_bank_${b}_silent_updates_eliminated",
      io.update.mask && updtBank1h(b) && !not_silent_update(0))
  }

  XSPerfAccumulate("tage_table_hits", PopCount(io.resp.valid))
  
  for (b <- 0 until nBanks) {
    XSPerfAccumulate(f"tage_table_bank_${b}_update_req", io.update.mask && updtBank1h(b))
    for (i <- 0 until numBr) {
      val li = i
      val pidx = 0.U
      val pi = i
    }
  }

  val u = io.update
  val b = PriorityEncoder(u.mask)
  val ub = PriorityEncoder(u.uMask)
  XSDebug(io.req.fire,
    p"tableReq: pc=0x${Hexadecimal(io.req.bits.pc)}, " +
    p"idx=$reqS0Idx, tag=$reqS0Tag\n")
  for (i <- 0 until numBr) {
    XSDebug(RegNext(io.req.fire) && isS1Hit,
      p"TageTableResp_br_$i: idx=$s1Idx, hit:${isS1Hit}, " +
      p"ctr:${io.resp.bits.ctr}, u:${io.resp.bits.u}\n")
    XSDebug(io.update.mask(i),
      p"update Table_br_$i: pc:${Hexadecimal(u.pc)}}, " +
      p"taken:${u.takens(i)}, alloc:${u.alloc(i)}, oldCtrs:${u.oldCtrs(i)}\n")
    val bank = OHToUInt(updtBank1h.asUInt, nBanks)
    val pi = 0.U
    XSDebug(io.update.mask(i),
      p"update Table_$i: writing tag:$updtTag, " +
      p"ctr: ${updtBanksWdata(bank).ctr} in idx ${updtIdx}\n")
    XSDebug(RegNext(io.req.fire) && !isS1Hit, p"TageTableResp_$i: not hit!\n")
  }

  // ------------------------------Debug-------------------------------------
  val valids = RegInit(VecInit(Seq.fill(nRows)(false.B)))
  when (io.update.mask) { valids(updtIdx) := true.B }
  XSDebug("Table usage:------------------------\n")
  XSDebug("%d out of %d rows are valid\n", PopCount(valids), nRows.U)

}

abstract class BaseTage(implicit p: Parameters) extends BasePredictor with TageParams with BPUUtils {
}

class FakeTage(implicit p: Parameters) extends BaseTage {
  io.out <> 0.U.asTypeOf(DecoupledIO(new BasePredictorOutput))

  io.s1_ready := true.B
  io.s2_ready := true.B
}


class Tage(val parentName:String = "Unknown")(implicit p: Parameters) extends BaseTage {
  val TickMax = ((1 << TickWidth) - 1).U

  val tageMeta = WireDefault(0.U.asTypeOf(new TageMeta))
  override val meta_size = tageMeta.getWidth

 val tables = TageTableInfos.zipWithIndex.map {
    case ((nRows, histLen, tagLen), i) => {
      val t = Module(new TageTable(nRows, histLen, tagLen, i, parentName = parentName + s"tagtable${i}_"))
      t.io.req.valid           := io.s0_fire(1)
      t.io.req.bits.pc         := s0_pc_dup(1)
      t.io.req.bits.foldedHist := io.in.bits.foldedHist(1)
      t.io.req.bits.ghist      := io.in.bits.ghist
      t
    }
  }

  val bt = Module (new TageBTable(parentName = parentName + "bttable_"))
  bt.io.req.valid := io.s0_fire(1)
  bt.io.req.bits  := s0_pc_dup(1)

  val altCounters = RegInit(VecInit(
    Seq.fill(altCtrsNum)((1 << (alterCtrBits-1)).U(alterCtrBits.W))))

  val s1DebugPC = RegEnable(s0_pc_dup(1), io.s0_fire(1))
  val s2DebugPC = RegEnable(s1DebugPC, io.s1_fire(1))
  val tage_fh_info = tables.map(_.getFoldedHistoryInfo).reduce(_++_).toSet
  override def getFoldedHistoryInfo = Some(tage_fh_info)

  // predict
  val s1RespVec      = VecInit(tables.map(_.io.resp))
  val s1RespBitsVec  = s1RespVec.map(_.bits)
  val s1RespValidVec = s1RespVec.map(_.valid)
  val s1Provide      = s1RespValidVec.reduce(_||_)
  val s1ProIdxVec    = VecInit((0 until TageNTables).map(i => i.U))
  val s1ProvideIdx   = ParallelPriorityMux(s1RespValidVec.reverse, s1ProIdxVec.reverse)
  val s1Resp         = ParallelPriorityMux(s1RespValidVec.reverse, s1RespBitsVec.reverse)
  val predAltCtrIdx  = useAltIdx(s1_pc_dup(1))
  val predAltCtr     = Mux1H( UIntToOH(predAltCtrIdx, altCtrsNum), altCounters )
  val isUseAltCtr    = (predAltCtr(alterCtrBits - 1) && s1Resp.unconf) || !s1Provide
  val s1BaseCtr      = bt.io.cnt
  val s1PredTaken    = Mux(isUseAltCtr, s1BaseCtr(1), s1Resp.ctr(TageCtrBits - 1))
  val s2PredTaken    = RegEnable(s1PredTaken, false.B, io.s1_fire(1))
  val s2PredTakenDup = dup(s2PredTaken)
  val s3PredTaken    = RegEnable(s2PredTaken, false.B, io.s2_fire(1))

  val s2TageEna  = RegEnable(RegEnable(io.ctrl.tage_enable, io.s0_fire(1)), io.s1_fire(1))
  val s3TageEna  = RegEnable(s2TageEna, io.s2_fire(1))
 
  when(s2TageEna) {
    io.out.s2.fullPred.map(_.br_taken := s2PredTaken)
  }
  when(s3TageEna) {
    io.out.s3.fullPred.map(_.br_taken := s3PredTaken)
  }

  // meta
  val s1AllocMask  = VecInit(s1RespVec.map(resp => !resp.valid && !resp.bits.u)).asUInt &
    ~(LowerMask(UIntToOH(s1ProvideIdx, TageNTables)) & Fill(TageNTables, s1Provide.asUInt))
  val s1altDiffer  = s1BaseCtr(1) =/= s1Resp.ctr(TageCtrBits - 1)
  val s1UseAltOnNa = predAltCtr(alterCtrBits - 1) && s1Resp.unconf
  val s1HitWayIdx  = s1Resp.wayIdx

  val s2altUsed     = RegEnable(isUseAltCtr, false.B, io.s1_fire(1))
  val s2Provide     = RegEnable(s1Provide, false.B, io.s1_fire(1))
  val s2ProvideIdx  = RegEnable(s1ProvideIdx, 0.U.asTypeOf(s1ProvideIdx), io.s1_fire(1))
  val s2Resp        = RegEnable(s1Resp, 0.U.asTypeOf(s1Resp), io.s1_fire(1))
  val s2altDiffer   = RegEnable(s1altDiffer, false.B, io.s1_fire(1))
  val s2AllocMask   = RegEnable(s1AllocMask, 0.U.asTypeOf(s1AllocMask), io.s1_fire(1))
  val s2PredCycle   = GTimer()
  val s2UseAltOnNa  = RegEnable(s1UseAltOnNa, 0.U.asTypeOf(s1UseAltOnNa), io.s1_fire(1))
  val s2BaseCtr     = RegEnable(s1BaseCtr, 0.U, io.s1_fire(1))
  val s2HitWayIdx   = RegEnable(s1HitWayIdx, io.s1_fire(1))

  val s3Provide    = RegEnable(s2Provide, false.B, io.s2_fire(1))
  val s3ProvideIdx = RegEnable(s2ProvideIdx, 0.U.asTypeOf(s1ProvideIdx), io.s2_fire(1))
  val s3Resp       = RegEnable(s2Resp, 0.U.asTypeOf(s1Resp), io.s2_fire(1))
  val s3altUsed    = RegEnable(s2altUsed, false.B, io.s2_fire(1))
  val s3altDiffer  = RegEnable(s2altDiffer, false.B, io.s2_fire(1))
  val s3baseCtr    = RegEnable(s2BaseCtr, 0.U, io.s2_fire(1))
  val s3AllocMask  = RegEnable(s2AllocMask, 0.U.asTypeOf(s2AllocMask), io.s2_fire(1))
  val s3PredCycle  = RegEnable(s2PredCycle, 0.U.asTypeOf(s2PredCycle), io.s2_fire(1))
  val s3UseAltOnNa = RegEnable(s2UseAltOnNa, 0.U.asTypeOf(s2UseAltOnNa), io.s2_fire(1))
  val s3HitWayIdx  = RegEnable(s2HitWayIdx, io.s2_fire(1))

  tageMeta.providers.valid   := s3Provide
  tageMeta.providers.bits    := s3ProvideIdx
  tageMeta.providerResps.ctr := s3Resp.ctr
  tageMeta.providerResps.u   := s3Resp.u
  tageMeta.altUsed           := s3altUsed
  // tageMeta.altDiffers       := s3altDiffer
  tageMeta.basecnts          := s3baseCtr
  tageMeta.allocates         := s3AllocMask
  // tageMeta.takens           := s3PredTaken
  tageMeta.pred_cycle.map(_  := s3PredCycle)
  tageMeta.use_alt_on_na.map(_ := s3UseAltOnNa)
  tageMeta.wayIdx             := s3HitWayIdx

  io.out.lastStageMeta := tageMeta.asUInt

  // update
  val updateValid   = io.update(0).valid
  val updateIn      = io.update(0).bits
  val updateMeta    = (io.update(0).bits.meta).asTypeOf(new TageMeta)
  val updateMispred = updateIn.mispred_mask(0)
  val updateBrJmpValid = updateValid && updateIn.ftbEntry.brValid(0) &&
                         !updateIn.ftbEntry.alwaysTaken(0)
  val updateTaken      = updateBrJmpValid && updateIn.br_taken(0)
  val updateProvide    = updateMeta.providers.valid
  val updateProvideIdx = updateMeta.providers.bits
  val updateAltUsed    = updateMeta.altUsed
  val updateAltDiffer  = updateMeta.altDiffers
  val updateTagePredCorrect = updateMeta.providerResps.ctr(TageCtrBits - 1) === updateTaken
  val updateOldCtrIn   = WireDefault(VecInit(Seq.fill(TageNTables)(0.U(TageCtrBits.W))))
  val updateIsUsIn     = WireDefault(VecInit(Seq.fill(TageNTables)(false.B)))
  val updateTageTaken  = WireDefault(VecInit(Seq.fill(TageNTables)(false.B)))
  val updateMask       = WireDefault(VecInit(Seq.fill(TageNTables)(false.B)))
  val updateUMask      = WireDefault(VecInit(Seq.fill(TageNTables)(false.B)))
  // allocate
  val isTageAllocate = updateBrJmpValid && updateMispred &&
                       !(updateProvide && updateTagePredCorrect && updateAltUsed)
  val allocRandomMask = LFSR64()(TageNTables - 1, 0)
  val longerHistoryTableMask = ~(LowerMask(UIntToOH(updateProvideIdx), TageNTables) &
                                 Fill(TageNTables, updateProvide.asUInt))
  val canAllocate = updateMeta.allocates.orR
  val canAllocMask = updateMeta.allocates & longerHistoryTableMask
  val allocTableMask = canAllocMask & allocRandomMask
  val allocateIdx = Mux(canAllocMask(PriorityEncoder(allocTableMask)),
                    PriorityEncoder(allocTableMask), PriorityEncoder(canAllocMask))
  val updateAllocMaskIn = WireDefault(VecInit(Seq.fill(TageNTables)(false.B)))
  // tick
  val tickCnt = RegInit(0.U(TickWidth.W))
  val tickTopDistance = RegInit(TickMax)
  val notAllocate = ~canAllocate & longerHistoryTableMask
  val tickIsInc = PopCount(canAllocate) < PopCount(notAllocate)
  val tickIsDec = PopCount(canAllocate) > PopCount(notAllocate)
  val tickIncVal = PopCount(notAllocate) - PopCount(canAllocate)
  val tickDecVal = PopCount(canAllocate) - PopCount(notAllocate)
  val tickSetToMax = tickIsInc && (tickIncVal >= tickTopDistance)
  val tickSetToMin = tickIsDec && (tickDecVal >= tickCnt)
  val tickCntReset = WireDefault(false.B)

  // update
  when(updateBrJmpValid && updateProvide) {
    updateMask(updateProvideIdx)        := true.B
    updateOldCtrIn(updateProvideIdx)    := updateMeta.providerResps.ctr
    updateIsUsIn(updateProvideIdx)      := updateTagePredCorrect
    updateTageTaken(updateProvideIdx)   := updateTaken
    updateUMask(updateProvideIdx)       := updateAltDiffer
    updateAllocMaskIn(updateProvideIdx) := false.B
  }
  
  // allocate
  when(isTageAllocate && canAllocate) {
    updateMask(allocateIdx)        := true.B
    updateAllocMaskIn(allocateIdx) := true.B
    updateTageTaken(allocateIdx)   := updateTaken
    updateUMask(allocateIdx)       := true.B
    updateIsUsIn(allocateIdx)      := false.B
  }

  // tick
  when(isTageAllocate) {
    when(tickCnt === TickMax) {
      tickCnt := 0.U
      tickTopDistance := TickMax
      tickCntReset := true.B
    }.elsewhen(tickIsInc) {
      when(tickSetToMax) {
        tickCnt := TickMax
        tickTopDistance := 0.U
      }.otherwise {
        tickCnt := tickCnt + tickIncVal
        tickTopDistance := tickTopDistance - tickIncVal
      }
    }.elsewhen(tickIsDec) {
      when(tickSetToMin) {
        tickCnt := 0.U
        tickTopDistance := TickMax
      }.otherwise {
        tickCnt := tickCnt - tickDecVal
        tickTopDistance := tickTopDistance + tickDecVal
      }
    }
  }

  for(a <- 0 until TageNTables) {
    tables(a).io.update.pc       := RegNext(updateIn.pc)
    tables(a).io.update.ghist    := RegNext(updateIn.ghist)
    tables(a).io.update.mask     := RegNext(updateMask(a))
    tables(a).io.update.takens   := RegNext(updateTageTaken(a)) 
    tables(a).io.update.alloc    := RegNext(updateAllocMaskIn(a)) 
    tables(a).io.update.oldCtrs  := RegNext(updateOldCtrIn(a)) 
    tables(a).io.update.uMask    := RegNext(updateUMask(a))
    tables(a).io.update.us       := RegNext(updateIsUsIn(a))
    tables(a).io.update.reset_u  := RegNext(tickCntReset)
    tables(a).io.update.wayIdx   := RegNext(updateMeta.wayIdx)
  }
  bt.io.updateMask   := RegNext(updateBrJmpValid && updateAltUsed)
  bt.io.updateCnt    := RegNext(updateMeta.basecnts)
  bt.io.updatePC     := RegNext(updateIn.pc)
  bt.io.updateTakens := RegNext(updateTaken)

  io.s1_ready := tables.map(_.io.req.ready).reduce(_&&_) && bt.io.req.ready 

  // alt counter update
  val updateProvideWeak = unconf(updateMeta.providerResps.ctr)
  // val updateAltDiff = updateAltDiffer
  val updateAltIdx = useAltIdx(updateIn.pc)
  val updateOldAltCtr = Mux1H( UIntToOH(updateAltIdx, altCtrsNum), altCounters )
  val updateAltPred = updateMeta.altPreds
  val updateAltCorrect = (updateAltPred === updateTaken)
  when(updateBrJmpValid && updateProvide && updateProvideWeak && updateAltDiffer) {
    val newCnt = updateCtr(updateOldAltCtr, alterCtrBits, updateAltCorrect)
    altCounters(updateAltIdx) := newCnt
  }

  def pred_perf(name: String, cnt: UInt)   = XSPerfAccumulate(s"${name}_at_pred", cnt)
  def commit_perf(name: String, cnt: UInt) = XSPerfAccumulate(s"${name}_at_commit", cnt)
  def tage_perf(name: String, pred_cnt: UInt, commit_cnt: UInt) = {
    pred_perf(name, pred_cnt)
    commit_perf(name, commit_cnt)
  }

  // perf counters
  XSPerfAccumulate(f"tage_bank_use_alt_pred", updateBrJmpValid && updateAltUsed)
  XSPerfAccumulate(f"tage_bank_alt_correct", updateBrJmpValid && updateAltUsed && updateAltCorrect)
  XSPerfAccumulate(f"tage_bank_alt_wrong", updateBrJmpValid && updateAltUsed && !updateAltCorrect)
  XSPerfAccumulate(f"tage_bank_alt_differs", updateBrJmpValid && updateAltDiffer)
  XSPerfAccumulate(f"tage_bank_use_alt_on_na_ctr_updated",
    updateBrJmpValid && updateAltDiffer && updateProvide && updateProvideWeak)
  XSPerfAccumulate(f"tage_bank_use_alt_on_na_ctr_inc",
    updateBrJmpValid && updateAltDiffer && updateProvide && updateProvideWeak &&  updateAltCorrect)
  XSPerfAccumulate(f"tage_bank_use_alt_on_na_ctr_dec",
    updateBrJmpValid && updateAltDiffer && updateProvide && updateProvideWeak && !updateAltCorrect)

  XSPerfAccumulate(f"tage_bank_na", updateBrJmpValid && updateProvide && updateProvideWeak)
  XSPerfAccumulate(f"tage_bank_use_na_correct",
    updateBrJmpValid && updateProvide && updateProvideWeak && !updateAltUsed && !updateMispred)
  XSPerfAccumulate(f"tage_bank_use_na_wrong",
    updateBrJmpValid && updateProvide && updateProvideWeak && !updateAltUsed &&  updateMispred)
  updateMeta.use_alt_on_na.foreach(uaon =>
    XSPerfAccumulate(f"tage_bank_use_alt_on_na", updateBrJmpValid && uaon))
  
  XSPerfAccumulate(f"tage_bank_update_allocate_failure", isTageAllocate && !canAllocate)
  XSPerfAccumulate(f"tage_bank_update_allocate_success", isTageAllocate &&  canAllocate)
  XSPerfAccumulate(s"tage_bank_mispred", updateBrJmpValid && updateMispred)
  XSPerfAccumulate(s"tage_bank_reset_u", tickCntReset)
  for (t <- 0 to TageNTables) {
    XSPerfAccumulate(f"tage_bank_tick_inc_${t}", isTageAllocate && tickIsInc && tickIncVal === t.U)
    XSPerfAccumulate(f"tage_bank_tick_dec_${t}", isTageAllocate && tickIsDec && tickDecVal === t.U)
  }

  XSPerfAccumulate(f"tage_write_blocks_read", !io.s1_ready)

  for (i <- 0 until TageNTables) {
    val pred_i_provided = s2Provide && s2ProvideIdx === i.U
    val commit_i_provided = updateProvide && updateProvideIdx === i.U && updateBrJmpValid
    tage_perf(
      s"bank_tage_table_${i}_provided",
      PopCount(pred_i_provided),
      PopCount(commit_i_provided)
    )
  }
  tage_perf(
    s"bank_tage_use_bim",
    PopCount(!s2Provide),
    PopCount(!updateProvide && updateBrJmpValid)
  )
  tage_perf(
    s"bank_tage_use_altpred",
    PopCount(s2Provide && unconf(s2Resp.ctr)),
    PopCount(updateProvide &&
      unconf(updateMeta.providerResps.ctr) && updateBrJmpValid)
  )
  tage_perf(
    s"bank_tage_provided",
    PopCount(s2Provide),
    PopCount(updateProvide && updateBrJmpValid)
  )

  XSDebug(updateBrJmpValid, "update: pc=%x, cycle=%d, taken:%b, misPred:%d, bimctr:%d, " +
    "pvdr(%d):%d, altDiff:%d, pvdrU:%d, pvdrCtr:%d, alloc:%b\n",
    updateIn.pc, 0.U, updateIn.br_taken, updateIn.mispred_mask.head,
    0.U, updateMeta.providers.valid, updateMeta.providers.bits, updateMeta.altDiffers,
    updateMeta.providerResps.u, updateMeta.providerResps.ctr, updateMeta.allocates
  )

  val s2_resps = RegEnable(s1RespVec, io.s1_fire(1))
  XSDebug("req: v=%d, pc=0x%x\n", io.s0_fire(1), s0_pc_dup(1))
  XSDebug("s1_fire:%d, resp: pc=%x\n", io.s1_fire(1), s1DebugPC)
  XSDebug("s2_fireOnLastCycle: resp: pc=%x, target=%x, hits=%b, takens=%b\n",
    s2DebugPC, io.out.s2.target(1), s2Provide.asUInt, s2PredTaken.asUInt)


  for (i <- 0 until TageNTables) {
    XSDebug("bank_tage_table(%d): valid:%b, resp_ctr:%d, resp_us:%d\n",
      i.U, s2_resps(i).valid, s2_resps(i).bits.ctr, s2_resps(i).bits.u)
  }

 
}


class Tage_SC(parentName:String = "Unknown")(implicit p: Parameters)
extends Tage(parentName) with HasSC {}