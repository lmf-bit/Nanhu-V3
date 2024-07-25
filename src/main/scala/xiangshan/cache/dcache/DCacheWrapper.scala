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
import chisel3.experimental.ExtModule
import chisel3.util._
import xiangshan._
import utils._
import freechips.rocketchip.diplomacy.{IdRange, LazyModule, LazyModuleImp, TransferSizes}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.{BundleFieldBase, UIntToOH1}
import difftest.common.DifftestMem
import xs.utils._
import coupledL2.{AliasField, AliasKey, DirtyField, PrefetchField}
import xs.utils.FastArbiter
import mem.AddPipelineReg
import xiangshan.backend.rob.RobPtr
import xs.utils.perf.HasPerfLogging
import xiangshan.mem.HasL1PrefetchSourceParameter
import xiangshan.mem.L1PrefetchReq

import scala.math.max

// DCache specific parameters
case class DCacheParameters
(
  nSets: Int = 64,
  nWays: Int = 8,
  rowBits: Int = 64,
  tagECC: Option[String] = None,
  dataECC: Option[String] = None,
  replacer: Option[String] = Some("setplru"),
  nMissEntries: Int = 1,
  nProbeEntries: Int = 1,
  nReleaseEntries: Int = 1,
  nMMIOEntries: Int = 1,
  nMMIOs: Int = 1,
  blockBytes: Int = 64,
  alwaysReleaseData: Boolean = true
) extends L1CacheParameters {
  // if sets * blockBytes > 4KB(page size),
  // cache alias will happen,
  // we need to avoid this by recoding additional bits in L2 cache
  val setBytes = nSets * blockBytes
  val aliasBitsOpt = if(setBytes > pageSize) Some(log2Ceil(setBytes / pageSize)) else None
  val reqFields: Seq[BundleFieldBase] = Nil
  val echoFields: Seq[BundleFieldBase] = Nil

  def tagCode: Code = Code.fromString(tagECC)

  def dataCode: Code = Code.fromString(dataECC)
}

//           Physical Address
// --------------------------------------
// |   Physical Tag |  PIndex  | Offset |
// --------------------------------------
//                  |
//                  DCacheTagOffset
//
//           Virtual Address
// --------------------------------------
// | Above index  | Set | Bank | Offset |
// --------------------------------------
//                |     |      |        |
//                |     |      |        0
//                |     |      DCacheBankOffset
//                |     DCacheSetOffset
//                DCacheAboveIndexOffset

// Default DCache size = 64 sets * 8 ways * 8 banks * 8 Byte = 32K Byte

trait HasDCacheParameters extends HasL1CacheParameters {
  val cacheParams = dcacheParameters
  val cfg = cacheParams

  def encWordBits = cacheParams.dataCode.width(wordBits)

  def encRowBits = encWordBits * rowWords // for DuplicatedDataArray only
  def eccBits = encWordBits - wordBits

  def encTagBits = cacheParams.tagCode.width(tagBits + ClientStates.width)
  def eccTagBits = encTagBits - tagBits - ClientStates.width

  def blockProbeAfterGrantCycles = 8 // give the processor some time to issue a request after a grant

  def nSourceType = 5
  def sourceTypeWidth = log2Up(nSourceType)
  def LOAD_SOURCE = 0
  def STORE_SOURCE = 1
  def AMO_SOURCE = 2
  def SOFT_PREFETCH = 3
  def DCACHE_PREFETCH_SOURCE = 4

  // each source use a id to distinguish its multiple reqs
  def reqIdWidth = log2Up(nEntries) max log2Up(StoreBufferSize)

  require(isPow2(cfg.nMissEntries)) // TODO
  // require(isPow2(cfg.nReleaseEntries))
  require(cfg.nMissEntries < cfg.nReleaseEntries)
  val nEntries = cfg.nMissEntries + cfg.nReleaseEntries
  val releaseIdBase = cfg.nMissEntries

  // banked dcache support
  val DCacheSets = cacheParams.nSets
  val DCacheWays = cacheParams.nWays
  val DCacheBanks = 8 // hardcoded
  val DCacheSRAMRowBits = cacheParams.rowBits // hardcoded
  val DCacheWordBits = 64 // hardcoded
  val DCacheWordBytes = DCacheWordBits / 8
  require(DCacheSRAMRowBits == 64)

  val DCacheSizeBits = DCacheSRAMRowBits * DCacheBanks * DCacheWays * DCacheSets
  val DCacheSizeBytes = DCacheSizeBits / 8
  val DCacheSizeWords = DCacheSizeBits / 64 // TODO

  val DCacheSameVPAddrLength = 12

  val DCacheSRAMRowBytes = DCacheSRAMRowBits / 8
  val DCacheWordOffset = log2Up(DCacheWordBytes)

  val DCacheBankOffset = log2Up(DCacheSRAMRowBytes)
  val DCacheSetOffset = DCacheBankOffset + log2Up(DCacheBanks)
  val DCacheAboveIndexOffset = DCacheSetOffset + log2Up(DCacheSets)
  val DCacheTagOffset = DCacheAboveIndexOffset min DCacheSameVPAddrLength
  val DCacheLineOffset = DCacheSetOffset

  // parameters about duplicating regs to solve fanout
  // In Main Pipe:
  // tag_write.ready -> data_write.valid * 8 banks
  // tag_write.ready -> meta_write.valid
  // tag_write.ready -> tag_write.valid
  // tag_write.ready -> err_write.valid
  // tag_write.ready -> wb.valid
  val nDupTagWriteReady = DCacheBanks + 4
  // In Main Pipe:
  // data_write.ready -> data_write.valid * 8 banks
  // data_write.ready -> meta_write.valid
  // data_write.ready -> tag_write.valid
  // data_write.ready -> err_write.valid
  // data_write.ready -> wb.valid
  val nDupDataWriteReady = DCacheBanks + 4
  val nDupWbReady = DCacheBanks + 4
  val nDupStatus = nDupTagWriteReady + nDupDataWriteReady
  val dataWritePort = 0
  val metaWritePort = DCacheBanks
  val tagWritePort = metaWritePort + 1
  val errWritePort = tagWritePort + 1
  val wbPort = errWritePort + 1

  val wbqWays = 2

  def addr_to_dcache_bank(addr: UInt) = {
    require(addr.getWidth >= DCacheSetOffset)
    addr(DCacheSetOffset-1, DCacheBankOffset)
  }

  def addr_to_dcache_set(addr: UInt) = {
    require(addr.getWidth >= DCacheAboveIndexOffset)
    addr(DCacheAboveIndexOffset-1, DCacheSetOffset)
  }

  def get_data_of_bank(bank: Int, data: UInt) = {
    require(data.getWidth >= (bank+1)*DCacheSRAMRowBits)
    data(DCacheSRAMRowBits * (bank + 1) - 1, DCacheSRAMRowBits * bank)
  }

  def get_mask_of_bank(bank: Int, data: UInt) = {
    require(data.getWidth >= (bank+1)*DCacheSRAMRowBytes)
    data(DCacheSRAMRowBytes * (bank + 1) - 1, DCacheSRAMRowBytes * bank)
  }

  def arbiter[T <: Bundle](
    in: Seq[DecoupledIO[T]],
    out: DecoupledIO[T],
    name: Option[String] = None): Unit = {
    val arb = Module(new Arbiter[T](chiselTypeOf(out.bits), in.size))
    if (name.nonEmpty) { arb.suggestName(s"${name.get}_arb") }
    for ((a, req) <- arb.io.in.zip(in)) {
      a <> req
    }
    out <> arb.io.out
  }

  def arbiter_with_pipereg[T <: Bundle](
    in: Seq[DecoupledIO[T]],
    out: DecoupledIO[T],
    name: Option[String] = None): Unit = {
    val arb = Module(new Arbiter[T](chiselTypeOf(out.bits), in.size))
    if (name.nonEmpty) { arb.suggestName(s"${name.get}_arb") }
    for ((a, req) <- arb.io.in.zip(in)) {
      a <> req
    }
    AddPipelineReg(arb.io.out, out, false.B)
  }

  def arbiter_with_pipereg_N_dup[T <: Bundle](
    in: Seq[DecoupledIO[T]],
    out: DecoupledIO[T],
    dups: Seq[DecoupledIO[T]],
    name: Option[String] = None): Unit = {
    val arb = Module(new Arbiter[T](chiselTypeOf(out.bits), in.size))
    if (name.nonEmpty) { arb.suggestName(s"${name.get}_arb") }
    for ((a, req) <- arb.io.in.zip(in)) {
      a <> req
    }
    for (dup <- dups) {
      AddPipelineReg(arb.io.out, dup, false.B)
    }
    AddPipelineReg(arb.io.out, out, false.B)
  }

  def rrArbiter[T <: Bundle](
    in: Seq[DecoupledIO[T]],
    out: DecoupledIO[T],
    name: Option[String] = None): Unit = {
    val arb = Module(new RRArbiterInit[T](chiselTypeOf(out.bits), in.size))
    if (name.nonEmpty) { arb.suggestName(s"${name.get}_arb") }
    for ((a, req) <- arb.io.in.zip(in)) {
      a <> req
    }
    out <> arb.io.out
  }

  def fastArbiter[T <: Bundle](
    in: Seq[DecoupledIO[T]],
    out: DecoupledIO[T],
    name: Option[String] = None): Unit = {
    val arb = Module(new FastArbiter[T](chiselTypeOf(out.bits), in.size))
    if (name.nonEmpty) { arb.suggestName(s"${name.get}_arb") }
    for ((a, req) <- arb.io.in.zip(in)) {
      a <> req
    }
    out <> arb.io.out
  }

  val numReplaceRespPorts = 2

  require(isPow2(nSets), s"nSets($nSets) must be pow2")
  require(isPow2(nWays), s"nWays($nWays) must be pow2")
  require(full_divide(rowBits, wordBits), s"rowBits($rowBits) must be multiple of wordBits($wordBits)")
  require(full_divide(beatBits, rowBits), s"beatBits($beatBits) must be multiple of rowBits($rowBits)")
}

abstract class DCacheModule(implicit p: Parameters) extends L1CacheModule
  with HasDCacheParameters

abstract class DCacheBundle(implicit p: Parameters) extends L1CacheBundle
  with HasDCacheParameters with HasL1PrefetchSourceParameter

class ReplacementAccessBundle(implicit p: Parameters) extends DCacheBundle {
  val set = UInt(log2Up(nSets).W)
  val way = UInt(log2Up(nWays).W)
}

class ReplacementWayReqIO(implicit p: Parameters) extends DCacheBundle {
  val set = ValidIO(UInt(log2Up(nSets).W))
  val way = Input(UInt(log2Up(nWays).W))
}

// memory request in word granularity(load, mmio, lr/sc, atomics)
class DCacheWordReq(implicit p: Parameters)  extends DCacheBundle
{
  val cmd    = UInt(M_SZ.W)
  val addr   = UInt(PAddrBits.W)
  val data   = UInt(DataBits.W)
  val mask   = UInt((DataBits/8).W)
  val id     = UInt(reqIdWidth.W)
  val instrtype   = UInt(sourceTypeWidth.W)
  val robIdx = new RobPtr
}

// memory request in word granularity(store)
class DCacheLineReq(implicit p: Parameters)  extends DCacheBundle
{
  val cmd    = UInt(M_SZ.W)
  val vaddr  = UInt(VAddrBits.W)
  val addr   = UInt(PAddrBits.W)
  val data   = UInt((cfg.blockBytes * 8).W)
  val mask   = UInt(cfg.blockBytes.W)
  val id     = UInt(reqIdWidth.W)
  def idx: UInt = get_idx(vaddr)
}

class RefillToSbuffer(implicit p: Parameters) extends  DCacheBundle{
  val data = UInt(l1BusDataWidth.W)
  val id = UInt(reqIdWidth.W)
  val refill_count = UInt((blockBytes/beatBytes).W)
}

class DCacheWordReqWithVaddr(implicit p: Parameters) extends DCacheWordReq {
  val vaddr = UInt(VAddrBits.W)
  val wline = Bool()
}

class BaseDCacheWordResp(implicit p: Parameters) extends DCacheBundle
{
  val data   = UInt(DataBits.W)
  val id     = UInt(reqIdWidth.W)

  // cache req missed, send it to miss queue
  val miss   = Bool()
  // cache miss, and failed to enter the missqueue, replay from RS is needed
  val replay = Bool()
  // data has been corrupted
  val tag_error = Bool() // tag error
}

class DCacheWordResp(implicit p: Parameters) extends BaseDCacheWordResp
{
  // 1 cycle after data resp
  val error_delayed = Bool() // all kinds of errors, include tag error
}

class BankedDCacheWordResp(implicit p: Parameters) extends DCacheWordResp with HasL1PrefetchSourceParameter
{
  //  val bank_data = Vec(DCacheBanks, Bits(DCacheSRAMRowBits.W))
  //  val bank_oh = UInt(DCacheBanks.W)
  val load_data = UInt(DCacheSRAMRowBits.W)
  val meta_prefetch = UInt(L1PfSourceBits.W)
  val wayIdx = UInt(log2Up(DCacheWays).W)
}

class DCacheWordRespWithError(implicit p: Parameters) extends BaseDCacheWordResp
{
  val error = Bool() // all kinds of errors, include tag error
}

class DCacheLineResp(implicit p: Parameters) extends DCacheBundle
{
  val data   = UInt((cfg.blockBytes * 8).W)
  // cache req missed, send it to miss queue
  val miss   = Bool()
  // cache req nacked, replay it later
  val replay = Bool()
  val id     = UInt(reqIdWidth.W)
}

class Refill(implicit p: Parameters) extends DCacheBundle
{
  val addr   = UInt(PAddrBits.W)
  val data   = UInt(l1BusDataWidth.W)
  val error  = Bool() // refilled data has been corrupted
  // for debug usage
  val data_raw = UInt((cfg.blockBytes * 8).W)
  val hasdata = Bool()
  val refill_done = Bool()
}

class Release(implicit p: Parameters) extends DCacheBundle
{
  val paddr  = UInt(PAddrBits.W)
  val wayIdx = UInt(log2Up(DCacheWays).W)
}

class DCacheWordIO(implicit p: Parameters) extends DCacheBundle
{
  val req  = DecoupledIO(new DCacheWordReq)
  val resp = Flipped(DecoupledIO(new BankedDCacheWordResp))
}

class UncacheWordIO(implicit p: Parameters) extends DCacheBundle
{
  val req  = DecoupledIO(new DCacheWordReq)
  val resp = Flipped(DecoupledIO(new DCacheWordRespWithError))
}

class AtomicWordIO(implicit p: Parameters) extends DCacheBundle
{
  val req  = DecoupledIO(new DCacheWordReqWithVaddr)
  val resp = Flipped(DecoupledIO(new DCacheWordRespWithError))
}

// used by load unit
class DCacheLoadIO(implicit p: Parameters) extends DCacheWordIO
{
  // kill previous cycle's req
  val s1_kill  = Output(Bool())
  val s2_kill  = Output(Bool())
  // cycle 0: virtual address: req.addr
  // cycle 1: physical address: s1_paddr
  val s1_paddr_dup_lsu = Output(UInt(PAddrBits.W)) // lsu side paddr
  val s1_paddr_dup_dcache = Output(UInt(PAddrBits.W)) // dcache side paddr
  val s1_disable_fast_wakeup = Input(Bool())
  val s1_bank_conflict = Input(Bool())
  // cycle 2: hit signal
  val s2_hit = Input(Bool()) // hit signal for lsu,

  // debug
  val debug_s1_hit_way = Input(UInt(nWays.W))
}

class DCacheLineIO(implicit p: Parameters) extends DCacheBundle
{
  val req  = DecoupledIO(new DCacheLineReq)
  val resp = Flipped(DecoupledIO(new DCacheLineResp))
}

class DCacheToSbufferIO(implicit p: Parameters) extends DCacheBundle {
  // sbuffer will directly send request to dcache main pipe
  val req = Flipped(Decoupled(new DCacheLineReq))

  val main_pipe_hit_resp = ValidIO(new DCacheLineResp)
  val refill_hit_resp = ValidIO(new DCacheLineResp)

  val replay_resp = ValidIO(new DCacheLineResp)

  val refill_row_data = ValidIO(new RefillToSbuffer)
  val refill_to_mp_req = ValidIO(Output(UInt(reqIdWidth.W)))
  val refill_to_mp_resp = Flipped(ValidIO(new DCacheLineReq))

  def hit_resps: Seq[ValidIO[DCacheLineResp]] = Seq(main_pipe_hit_resp, refill_hit_resp)
}

class DCacheTLDBypassLduIO(implicit p: Parameters) extends DCacheBundle {
  val valid = Bool()
  val mshrid = UInt(log2Up(cfg.nMissEntries).W)
}

class DCacheToLsuIO(implicit p: Parameters) extends DCacheBundle {
  val load  = Vec(LoadPipelineWidth, Flipped(new DCacheLoadIO)) // for speculative load
  val loadReqHandledResp = ValidIO(UInt(log2Up(cfg.nMissEntries).W)) // mshrID which handled load miss req
  val tl_d_channel = Output(new DCacheTLDBypassLduIO)
  //todo: remove lsq
  val lsq = ValidIO(new Refill)  // refill to load queue, wake up load misses
  val store = new DCacheToSbufferIO // for sbuffer
  val atomics  = Flipped(new AtomicWordIO)  // atomics reqs
  val release = ValidIO(new Release) // cacheline release hint for ld-ld violation check
}

class DCacheIO(implicit p: Parameters) extends DCacheBundle {
  val hartId = Input(UInt(8.W))
  val l2_pf_store_only = Input(Bool())
  val lsu = new DCacheToLsuIO
  val csr = new L1CacheToCsrIO
  val error = new L1CacheErrorInfo
  val mshrFull = Output(Bool())
  val pf_req = Flipped(DecoupledIO(new L1PrefetchReq()))
}


class DCache(val parentName:String = "Unknown")(implicit p: Parameters) extends LazyModule with HasDCacheParameters {

  val clientParameters = TLMasterPortParameters.v1(
    Seq(TLMasterParameters.v1(
      name = "dcache",
      sourceId = IdRange(0, nEntries + 1),
      supportsProbe = TransferSizes(cfg.blockBytes)
    )),
    requestFields = cacheParams.reqFields,
    echoFields = cacheParams.echoFields
  )

  val clientNode = TLClientNode(Seq(clientParameters))

  lazy val module = new DCacheImp(this)
}


class DCacheImp(outer: DCache) extends LazyModuleImp(outer) with HasDCacheParameters with HasPerfEvents with HasPerfLogging {

  val io = IO(new DCacheIO)

  val (bus, edge) = outer.clientNode.out.head
  require(bus.d.bits.data.getWidth == l1BusDataWidth, "DCache: tilelink width does not match")

  println("DCache:")
  println("  DCacheSets: " + DCacheSets)
  println("  DCacheWays: " + DCacheWays)
  println("  DCacheBanks: " + DCacheBanks)
  println("  DCacheSRAMRowBits: " + DCacheSRAMRowBits)
  println("  DCacheWordOffset: " + DCacheWordOffset)
  println("  DCacheBankOffset: " + DCacheBankOffset)
  println("  DCacheSetOffset: " + DCacheSetOffset)
  println("  DCacheTagOffset: " + DCacheTagOffset)
  println("  DCacheAboveIndexOffset: " + DCacheAboveIndexOffset)

  val LoadPrefetchL1Enabled = true
  val PrefetchArrayReadPort = if(LoadPrefetchL1Enabled) LoadPipelineWidth  else 0

  //----------------------------------------
  // core data structures
  val bankedDataArray = Module(new BankedDataArray(parentName = outer.parentName + "bankedDataArray_"))
  //  val metaArray = Module(new AsynchronousMetaArray(readPorts = 3, writePorts = 1))
  val errorArray = Module(new ErrorArray(readPorts = 3, writePorts = 1)) // TODO: add it to meta array
  val tagArray = Module(new DuplicatedTagArray(readPorts = LoadPipelineWidth + 1, parentName = outer.parentName + "tagArray_"))
  val prefetchArray = Module(new L1PrefetchSourceArray(readPorts = PrefetchArrayReadPort, writePorts = 1 + LoadPipelineWidth)) // prefetch flag array
  bankedDataArray.dump()

  //----------------------------------------
  // core modules
  val ldu = Seq.tabulate(LoadPipelineWidth)({ i => Module(new LoadPipe(i))})
  val atomicsReplayUnit = Module(new AtomicsReplayEntry)
  val mainPipe   = Module(new MainPipe)
  // val refillPipe = Module(new RefillPipe)
  val missQueue  = Module(new MissQueue(edge))
  val probeQueue = Module(new ProbeQueue(edge))
  val wb         = Module(new WritebackQueue(edge))

  //load req s0
//  require(io.lsu.load.length == 2)
  val ldAllValid = io.lsu.load(0).req.valid && io.lsu.load(1).req.valid
  val ldRob = io.lsu.load.map(_.req.bits.robIdx)
  val ldSelRead = Mux(ldAllValid,Mux(ldRob(0) < ldRob(1),0.U,1.U),0.U)

  //pf req read tag
  val pf_tag_read = Wire(new TagReadReq())
  val pf_tag_resp = Wire(Vec(nWays, UInt(encTagBits.W)))

  pf_tag_read.idx := get_idx(io.pf_req.bits.getVaddr())
  pf_tag_read.way_en := ~0.U(nWays.W)
  pf_tag_resp := DontCare

  missQueue.io.hartId := io.hartId
  missQueue.io.l2_pf_store_only := RegNext(io.l2_pf_store_only, false.B)

  val errors = ldu.map(_.io.error) ++ // load error
    Seq(mainPipe.io.error) // store / misc error
  io.error <> RegNext(Mux1H(errors.map(e => RegNext(e.valid) -> RegNext(e))))

  //----------------------------------------
  // error array
  val tag_read_ports =ldu.map(_.io.tag_read) ++ Seq(mainPipe.io.tag_read)
  val error_flag_resp_ports = ldu.map(_.io.error_flag_resp) ++
    Seq(mainPipe.io.error_flag_resp)
  val error_flag_write_ports = Seq(
    mainPipe.io.error_flag_write,
  )
  tag_read_ports.zip(errorArray.io.read).foreach { case (p, r) => r <> p }
  error_flag_resp_ports.zip(errorArray.io.resp).foreach { case (p, r) => p := r }
  error_flag_write_ports.zip(errorArray.io.write).foreach { case (p, w) => w <> p }

  //tag array
  require(tagArray.io.read.size == (ldu.size + 1))
  // val tag_write_intend = missQueue.io.refill_pipe_req.valid || mainPipe.io.tag_write_intend
  val tag_write_intend = mainPipe.io.tag_write_intend
  assert(!RegNext(!tag_write_intend && tagArray.io.write.valid))
  ldu.zipWithIndex.foreach {
    case (ld, i) =>
      tagArray.io.read(i) <> ld.io.tag_read
      ld.io.tag_resp := tagArray.io.resp(i)
      ld.io.tag_read.ready := !tag_write_intend
  }
  tagArray.io.read.last <> mainPipe.io.tag_read
  mainPipe.io.tag_resp := tagArray.io.resp.last



  val pipe_invalid = Cat((ldu.map(!_.io.tag_read.valid) ++ Seq(!mainPipe.io.tag_read.valid)).reverse)
  val pf_req_can_go = pipe_invalid.orR && !tag_write_intend
  val idx = PriorityEncoder(pipe_invalid)
  when(io.pf_req.fire){
    tagArray.io.read(idx).valid := true.B
    tagArray.io.read(idx).bits := pf_tag_read
  }
  io.pf_req.ready := pf_req_can_go


  val pf_req_s1 = RegEnable(io.pf_req.bits, io.pf_req.fire)
  val pf_req_s1_valid = RegNext(io.pf_req.fire)
  // val pipe_invalid_s1 = RegEnable(pipe_invalid, io.pf_req.fire)
  val s1_idx = RegEnable(idx, io.pf_req.fire)
  when(pf_req_s1_valid){
    pf_tag_resp := tagArray.io.resp(s1_idx)
  }
  val meta_resp = pf_tag_resp.map {r =>
    ClientMetadata(r(tagBits + ClientStates.width - 1, tagBits))
  }
  val s1_tag_match = VecInit((0 until nWays).map((w: Int) => pf_tag_resp(w)(tagBits - 1, 0) === get_tag(pf_req_s1.paddr)))
  val s1_hit  = VecInit((0 until nWays).map((w: Int) => s1_tag_match(w) && meta_resp(w).isValid())).asUInt.orR

  val pf_req_s2 = RegEnable(pf_req_s1, pf_req_s1_valid)
  val pf_req_s2_valid = RegNext(pf_req_s1_valid)
  val s2_hit = RegEnable(s1_hit, pf_req_s1_valid)
  // val pipe_invalid_s2 = RegEnable(pipe_invalid_s1, pf_req_s1_valid)

  val pf_miss_req = Wire(new MissReq())
  pf_miss_req := DontCare
  pf_miss_req.source := DCACHE_PREFETCH_SOURCE.U
  pf_miss_req.cmd := M_PFR
  pf_miss_req.addr := pf_req_s2.paddr
  pf_miss_req.vaddr := pf_req_s2.getVaddr()
  pf_miss_req.pf_source := pf_req_s2.pf_source.value

  val miss_invalid = Cat(Seq(!mainPipe.io.miss_req.valid) ++ ldu.map(!_.io.miss_req.valid))
  val pfReqCanSend = miss_invalid.andR
  // val s2_idx = PriorityEncoder(Reverse(miss_valid))

  //----------------------------------------
  // prefetch array
  ldu.zipWithIndex.foreach {
    case (ld, i) =>
      prefetchArray.io.read(i).valid := ld.io.tag_read.valid
      prefetchArray.io.read(i).bits := ld.io.tag_read.bits
      ld.io.extra_meta_resp := prefetchArray.io.resp(i)
  }
  val prefetch_flag_write_ports = ldu.map(_.io.prefetch_flag_write) ++ Seq(
    mainPipe.io.prefetch_flag_write // set prefetch_flag to false if coh is set to Nothing
  )
  prefetch_flag_write_ports.zip(prefetchArray.io.write).foreach { case (p, w) => w <> p }


  val fake_tag_read_conflict_this_cycle = PopCount(ldu.map(ld=> ld.io.tag_read.valid))
  XSPerfAccumulate("fake_tag_read_conflict", fake_tag_read_conflict_this_cycle)

  // val tag_write_arb = Module(new Arbiter(new TagWriteReq, 2))
  // tag_write_arb.io.in(0) <> refillPipe.io.tag_write
  // tag_write_arb.io.in(1) <> mainPipe.io.tag_write
  tagArray.io.write <> mainPipe.io.tag_write

  //----------------------------------------
  // data array

  // val dataWriteArb = Module(new Arbiter(new L1BankedDataWriteReq, 2))
  // dataWriteArb.io.in(0) <> refillPipe.io.data_write
  // dataWriteArb.io.in(1) <> mainPipe.io.data_write

  bankedDataArray.io.write <> mainPipe.io.data_write

  for (bank <- 0 until DCacheBanks) {
    val dataWriteArb_dup = Module(new Arbiter(new L1BankedDataWriteReqCtrl, 1))
    dataWriteArb_dup.io.in(0).valid := mainPipe.io.data_write_dup(bank).valid
    dataWriteArb_dup.io.in(0).bits := mainPipe.io.data_write_dup(bank).bits

    bankedDataArray.io.write_dup(bank) <> dataWriteArb_dup.io.out
  }

  bankedDataArray.io.readline <> mainPipe.io.data_read
  bankedDataArray.io.readline_intend := mainPipe.io.data_read_intend
  mainPipe.io.readline_error_delayed := bankedDataArray.io.readline_error_delayed
  mainPipe.io.data_resp := bankedDataArray.io.resp

  //loadPipe read bankedDataArray in s1
  bankedDataArray.io.readSel := RegNext(ldSelRead)
  (0 until LoadPipelineWidth).map(i => {
    bankedDataArray.io.read(i) <> ldu(i).io.banked_data_read
    bankedDataArray.io.read_error_delayed(i) <> ldu(i).io.read_error_delayed

    ldu(i).io.bank_conflict_fast := bankedDataArray.io.bank_conflict_fast(i)
    ldu(i).io.bank_conflict_slow := bankedDataArray.io.bank_conflict_slow(i)
  })

  (0 until LoadPipelineWidth).foreach({ case i => {
    ldu(i).io.banked_data_resp := bankedDataArray.io.resp
  }})

  //----------------------------------------
  // load pipe
  // the s1 kill signal
  // only lsu uses this, replay never kills
  for (w <- 0 until LoadPipelineWidth) {
//    ldu(w).io.lsu <> io.lsu.load(w)

    // replay and nack not needed anymore
    // TODO: remove replay and nack
    ldu(w).io.nack := false.B

    ldu(w).io.disable_ld_fast_wakeup :=
      bankedDataArray.io.disable_ld_fast_wakeup(w) // load pipe fast wake up should be disabled when bank conflict
  }

  //todo
  ldu.head.io.lsu <> io.lsu.load.head
  ldu(1).io.lsu <> io.lsu.load(1)

  //----------------------------------------
  // atomics
  // atomics not finished yet
  io.lsu.atomics <> atomicsReplayUnit.io.lsu
  atomicsReplayUnit.io.pipe_resp := RegNext(mainPipe.io.atomic_resp)
  atomicsReplayUnit.io.block_lr <> mainPipe.io.block_lr

  //----------------------------------------
  // miss queue
  val MissReqPortCount = LoadPipelineWidth + 1
  val MainPipeMissReqPort = 0

  // Request
  val missReqArb = Module(new Arbiter(new MissReq, MissReqPortCount))

  missReqArb.io.in(MainPipeMissReqPort) <> mainPipe.io.miss_req
  for (w <- 0 until LoadPipelineWidth) { missReqArb.io.in(w + 1) <> ldu(w).io.miss_req }

  //prefetch
  when(pf_req_s2_valid && !s2_hit && pfReqCanSend){
    missReqArb.io.in(0).valid := true.B
    missReqArb.io.in(0).bits := pf_miss_req
    // assert( !miss_valid(s2_idx) && RegNext(RegNext(pipe_invalid(idx))))
  }

  wb.io.miss_req.valid := missReqArb.io.out.valid
  wb.io.miss_req.bits  := missReqArb.io.out.bits.addr

  // block_decoupled(missReqArb.io.out, missQueue.io.req, wb.io.block_miss_req)
  missReqArb.io.out <> missQueue.io.req
  when(wb.io.block_miss_req) {
    missQueue.io.req.bits.cancel := true.B
    missReqArb.io.out.ready := false.B
  }

  // refill to load queue
  io.lsu.lsq <> missQueue.io.refill_to_ldq
  io.lsu.loadReqHandledResp <> missQueue.io.loadReqHandledResp
//  val (_, _, done, _) = edge.count(bus.d)
//  when (bus.d.bits.opcode === TLMessages.GrantData || bus.d.bits.opcode === TLMessages.Grant) {
//    io.lsu.tl_d_channel.valid := bus.d.valid && done
//    io.lsu.tl_d_channel.mshrid := bus.d.bits.source
//  } .otherwise {
//    io.lsu.tl_d_channel := DontCare
//  }
//  io.lsu.tl_d_channel.valid := bus.d.valid && (bus.d.bits.opcode === TLMessages.GrantData || bus.d.bits.opcode === TLMessages.Grant)
//  io.lsu.tl_d_channel.mshrid := bus.d.bits.source

  io.lsu.tl_d_channel.valid := RegNext(mainPipe.io.replace_req.fire)
  io.lsu.tl_d_channel.mshrid := RegEnable(mainPipe.io.replace_req.bits.miss_id, mainPipe.io.replace_req.fire)



  // tilelink stuff
  bus.a <> missQueue.io.mem_acquire
  bus.e <> missQueue.io.mem_finish
  missQueue.io.probe_addr := bus.b.bits.address

  missQueue.io.main_pipe_resp := RegNext(mainPipe.io.atomic_resp)

  //----------------------------------------
  // probe
  // probeQueue.io.mem_probe <> bus.b
  block_decoupled(bus.b, probeQueue.io.mem_probe, missQueue.io.probe_block)
  probeQueue.io.lrsc_locked_block <> mainPipe.io.lrsc_locked_block
  probeQueue.io.update_resv_set <> mainPipe.io.update_resv_set

  //----------------------------------------
  // mainPipe
  // when a req enters main pipe, if it is set-conflict with replace pipe or refill pipe,
  // block the req in main pipe
  block_decoupled(probeQueue.io.pipe_req, mainPipe.io.probe_req, false.B)
  block_decoupled(io.lsu.store.req, mainPipe.io.store_req, false.B)

  io.lsu.store.replay_resp := RegNext(mainPipe.io.store_replay_resp)
  io.lsu.store.main_pipe_hit_resp := mainPipe.io.store_hit_resp

  arbiter(
    in = Seq(missQueue.io.main_pipe_req, atomicsReplayUnit.io.pipe_req),
    out = mainPipe.io.atomic_req,
    name = Some("main_pipe_atomic_req")
  )

  mainPipe.io.invalid_resv_set := RegNext(wb.io.req.fire && wb.io.req.bits.addr === mainPipe.io.lrsc_locked_block.bits && mainPipe.io.lrsc_locked_block.valid)

  //----------------------------------------
  // replace (main pipe)
//  val mpStatus = mainPipe.io.status
  mainPipe.io.replace_req <> missQueue.io.replace_pipe_req
  missQueue.io.replace_pipe_resp := mainPipe.io.replace_resp
  io.lsu.store.refill_hit_resp := DontCare

   //----------------------------------------
  //sbuffer
  io.lsu.store.refill_row_data <> missQueue.io.refill_to_sbuffer
  io.lsu.store.refill_to_mp_req.valid := (missQueue.io.replace_pipe_req.fire && missQueue.io.replace_pipe_req.bits.source === STORE_SOURCE.U) || missQueue.io.main_pipe_req.fire
  io.lsu.store.refill_to_mp_req.bits := Mux(missQueue.io.replace_pipe_req.fire, missQueue.io.replace_pipe_req.bits.id, 0.U)
  mainPipe.io.refill_data <> io.lsu.store.refill_to_mp_resp

  //----------------------------------------
  // refill pipe
  // val refillShouldBeBlocked = (mpStatus.s1.valid && mpStatus.s1.bits.set === missQueue.io.refill_pipe_req.bits.idx) ||
  //   Cat(Seq(mpStatus.s2, mpStatus.s3).map(s =>
  //     s.valid &&
  //       s.bits.set === missQueue.io.refill_pipe_req.bits.idx &&
  //       s.bits.way_en === missQueue.io.refill_pipe_req.bits.way_en
  //   )).orR
  // block_decoupled(missQueue.io.refill_pipe_req, refillPipe.io.req, refillShouldBeBlocked)

  // val mpStatus_dup = mainPipe.io.status_dup
  // val mq_refill_dup = missQueue.io.refill_pipe_req_dup
  // val refillShouldBeBlocked_dup = VecInit((0 until nDupStatus).map { case i =>
  //   mpStatus_dup(i).s1.valid && mpStatus_dup(i).s1.bits.set === mq_refill_dup(i).bits.idx ||
  //   Cat(Seq(mpStatus_dup(i).s2, mpStatus_dup(i).s3).map(s =>
  //     s.valid &&
  //       s.bits.set === mq_refill_dup(i).bits.idx &&
  //       s.bits.way_en === mq_refill_dup(i).bits.way_en
  //   )).orR
  // })
  // dontTouch(refillShouldBeBlocked_dup)

  // refillPipe.io.req_dup_for_data_w.zipWithIndex.foreach { case (r, i) =>
  //   r.bits := (mq_refill_dup.drop(dataWritePort).take(DCacheBanks))(i).bits
  // }
  // refillPipe.io.req_dup_for_meta_w.bits := mq_refill_dup(metaWritePort).bits
  // refillPipe.io.req_dup_for_tag_w.bits := mq_refill_dup(tagWritePort).bits
  // refillPipe.io.req_dup_for_err_w.bits := mq_refill_dup(errWritePort).bits
  // refillPipe.io.req_dup_for_data_w.zipWithIndex.foreach { case (r, i) =>
  //   r.valid := (mq_refill_dup.drop(dataWritePort).take(DCacheBanks))(i).valid &&
  //     !(refillShouldBeBlocked_dup.drop(dataWritePort).take(DCacheBanks))(i)
  // }
  // refillPipe.io.req_dup_for_meta_w.valid := mq_refill_dup(metaWritePort).valid && !refillShouldBeBlocked_dup(metaWritePort)
  // refillPipe.io.req_dup_for_tag_w.valid := mq_refill_dup(tagWritePort).valid && !refillShouldBeBlocked_dup(tagWritePort)
  // refillPipe.io.req_dup_for_err_w.valid := mq_refill_dup(errWritePort).valid && !refillShouldBeBlocked_dup(errWritePort)

  // val refillPipe_io_req_valid_dup = VecInit(mq_refill_dup.zip(refillShouldBeBlocked_dup).map(
  //   x => x._1.valid && !x._2
  // ))
  // val refillPipe_io_data_write_valid_dup = VecInit(refillPipe_io_req_valid_dup.slice(0, nDupDataWriteReady))
  // val refillPipe_io_tag_write_valid_dup = VecInit(refillPipe_io_req_valid_dup.slice(nDupDataWriteReady, nDupStatus))
  // dontTouch(refillPipe_io_req_valid_dup)
  // dontTouch(refillPipe_io_data_write_valid_dup)
  // dontTouch(refillPipe_io_tag_write_valid_dup)

//  mainPipe.io.data_write_ready_dup := VecInit(Seq.fill(nDupDataWriteReady)(true.B))
//  mainPipe.io.tag_write_ready_dup := VecInit(Seq.fill(nDupDataWriteReady)(true.B))

  mainPipe.io.wb_ready_dup := wb.io.req_ready_dup

  // mq_refill_dup.zip(refillShouldBeBlocked_dup).foreach { case (r, block) =>
  //   r.ready := refillPipe.io.req.ready && !block
  // }

  // missQueue.io.refill_pipe_resp := refillPipe.io.resp
  // io.lsu.store.refill_hit_resp := RegNext(refillPipe.io.store_resp)  //need to do

  //----------------------------------------
  // wb
  // add a queue between MainPipe and WritebackUnit to reduce MainPipe stalls due to WritebackUnit busy

  wb.io.req <> mainPipe.io.wb
  bus.c     <> wb.io.mem_release
  // wb.io.release_wakeup := refillPipe.io.release_wakeup
  // wb.io.release_wakeup := mainPipe.io.replace_resp
  // wb.io.release_update := mainPipe.io.release_update
  wb.io.probe_ttob_check_req <> mainPipe.io.probe_ttob_check_req
  wb.io.probe_ttob_check_resp <> mainPipe.io.probe_ttob_check_resp

  io.lsu.release.valid := RegNext(wb.io.req.fire)
  io.lsu.release.bits.paddr := RegEnable(wb.io.req.bits.addr, wb.io.req.fire)
  io.lsu.release.bits.wayIdx := RegEnable(wb.io.req.bits.wayIdx, wb.io.req.fire)

  // Note: RegNext() is required by:
  // * load queue released flag update logic
  // * load / load violation check logic
  // * and timing requirements
  // CHANGE IT WITH CARE

  // connect bus d
  missQueue.io.mem_grant.valid := false.B
  missQueue.io.mem_grant.bits  := DontCare

  wb.io.mem_grant.valid := false.B
  wb.io.mem_grant.bits  := DontCare

  // in L1DCache, we ony expect Grant[Data] and ReleaseAck
  bus.d.ready := false.B
  when (bus.d.bits.opcode === TLMessages.Grant || bus.d.bits.opcode === TLMessages.GrantData) {
    missQueue.io.mem_grant <> bus.d
  } .elsewhen (bus.d.bits.opcode === TLMessages.ReleaseAck) {
    wb.io.mem_grant <> bus.d
  } .otherwise {
    assert (!bus.d.fire)
  }
  class DCachePLRUWrapper(reqWidth: Int = 1, accessWidth: Int = 3, nSets: Int, nWays: Int) extends Module {
    val io = IO(new Bundle() {
      val req_set = Input(Vec(reqWidth, Valid(UInt(log2Up(nSets).W))))
      val req_way = Output(Vec(reqWidth, UInt(log2Up(nWays).W)))

      val touch_sets = Input(Vec(accessWidth, UInt(log2Up(nSets).W)))
      val touch_ways = Input(Vec(accessWidth, Valid(UInt(log2Up(nWays).W))))
    })
    val replacer = ReplacementPolicy.fromString(cacheParams.replacer, nWays, nSets)

    println("DCcachePLRUWrapper:")
    println("nWays = " + nWays)
    println("nSets = " + nSets)

    (0 until reqWidth).foreach {
      case i => {
        io.req_way(i) := DontCare
        when(io.req_set(i).valid) {
          io.req_way(i) := replacer.way(io.req_set(i).bits)
        }
      }
    }
    replacer.access(io.touch_sets, io.touch_ways)
  }


  //----------------------------------------
  // replacement algorithm
  val replacer = Module(new DCachePLRUWrapper(reqWidth = 1,accessWidth = 3, nSets, nWays))
  replacer.suggestName("DCcachePLURWrapper_0")

  mainPipe.io.replace_way.way := DontCare
  replacer.io.req_set(0) := mainPipe.io.replace_way.set
  when(mainPipe.io.replace_way.set.valid) {
    mainPipe.io.replace_way.way := replacer.io.req_way(0)
  }

  val replAccessReqs = ldu.map(_.io.replace_access) ++ Seq(
    mainPipe.io.replace_access
  )
  val touchWays = Seq.fill(replAccessReqs.size)(Wire(ValidIO(UInt(log2Up(nWays).W))))
  touchWays.zip(replAccessReqs).foreach {
    case (w, req) =>
      w.valid := req.valid
      w.bits := req.bits.way
  }
  val touchSets = replAccessReqs.map(_.bits.set)
  replacer.io.touch_sets := touchSets
  replacer.io.touch_ways := touchWays
  assert(replacer.io.touch_sets.length == touchSets.length)
  assert(replacer.io.touch_ways.length == touchWays.length)

  //----------------------------------------
  // assertions
  // dcache should only deal with DRAM addresses
  when (bus.a.fire) {
    assert(bus.a.bits.address >= 0x80000000L.U)
  }
  when (bus.b.fire) {
    assert(bus.b.bits.address >= 0x80000000L.U)
  }
  when (bus.c.fire) {
    assert(bus.c.bits.address >= 0x80000000L.U)
  }

  //----------------------------------------
  // utility functions
  def block_decoupled[T <: Data](source: DecoupledIO[T], sink: DecoupledIO[T], block_signal: Bool) = {
    sink.valid   := source.valid && !block_signal
    source.ready := sink.ready   && !block_signal
    sink.bits    := source.bits
  }

  //----------------------------------------
  // Customized csr cache op support
  val cacheOpDecoder = Module(new CSRCacheOpDecoder("dcache", CacheInstrucion.COP_ID_DCACHE))
  cacheOpDecoder.io.csr <> io.csr
  bankedDataArray.io.cacheOp.req := cacheOpDecoder.io.cache.req
  // dup cacheOp_req_valid
  bankedDataArray.io.cacheOp_req_dup.zipWithIndex.map{ case(dup, i) => dup := cacheOpDecoder.io.cache_req_dup(i) }
  // dup cacheOp_req_bits_opCode
  bankedDataArray.io.cacheOp_req_bits_opCode_dup.zipWithIndex.map{ case (dup, i) => dup := cacheOpDecoder.io.cacheOp_req_bits_opCode_dup(i) }

  tagArray.io.cacheOp.req := cacheOpDecoder.io.cache.req
  // dup cacheOp_req_valid
  tagArray.io.cacheOp_req_dup.zipWithIndex.map{ case(dup, i) => dup := cacheOpDecoder.io.cache_req_dup(i) }
  // dup cacheOp_req_bits_opCode
  tagArray.io.cacheOp_req_bits_opCode_dup.zipWithIndex.map{ case (dup, i) => dup := cacheOpDecoder.io.cacheOp_req_bits_opCode_dup(i) }

  cacheOpDecoder.io.cache.resp.valid := bankedDataArray.io.cacheOp.resp.valid ||
    tagArray.io.cacheOp.resp.valid
  cacheOpDecoder.io.cache.resp.bits := Mux1H(List(
    bankedDataArray.io.cacheOp.resp.valid -> bankedDataArray.io.cacheOp.resp.bits,
    tagArray.io.cacheOp.resp.valid -> tagArray.io.cacheOp.resp.bits,
  ))
  cacheOpDecoder.io.error := io.error
  assert(!((bankedDataArray.io.cacheOp.resp.valid +& tagArray.io.cacheOp.resp.valid) > 1.U))

  //----------------------------------------
  // performance counters
  val num_loads = PopCount(ldu.map(e => e.io.lsu.req.fire))
  XSPerfAccumulate("num_loads", num_loads)

  io.mshrFull := missQueue.io.full

  // performance counter
  val ld_access = Wire(Vec(LoadPipelineWidth, missQueue.io.debug_early_replace.last.cloneType))
  val st_access = Wire(ld_access.last.cloneType)
  ld_access.zip(ldu).foreach {
    case (a, u) =>
      a.valid := RegNext(u.io.lsu.req.fire) && !u.io.lsu.s1_kill
      a.bits.idx := RegNext(get_idx(u.io.lsu.req.bits.addr))
      a.bits.tag := get_tag(u.io.lsu.s1_paddr_dup_dcache)
  }
  st_access.valid := RegNext(mainPipe.io.store_req.fire)
  st_access.bits.idx := RegNext(get_idx(mainPipe.io.store_req.bits.vaddr))
  st_access.bits.tag := RegNext(get_tag(mainPipe.io.store_req.bits.addr))
  val access_info = ld_access.toSeq ++ Seq(st_access)
  val early_replace = RegNext(missQueue.io.debug_early_replace)
  val access_early_replace = access_info.map {
    case acc =>
      Cat(early_replace.map {
        case r =>
          acc.valid && r.valid &&
            acc.bits.tag === r.bits.tag &&
            acc.bits.idx === r.bits.idx
      })
  }
  XSPerfAccumulate("access_early_replace", PopCount(Cat(access_early_replace)))

  val perfEvents = (Seq(wb, mainPipe, missQueue, probeQueue) ++ ldu).flatMap(_.getPerfEvents)
  generatePerfEvent()
}

class AMOHelper() extends ExtModule {
  val clock  = IO(Input(Clock()))
  val enable = IO(Input(Bool()))
  val cmd    = IO(Input(UInt(5.W)))
  val addr   = IO(Input(UInt(64.W)))
  val wdata  = IO(Input(UInt(64.W)))
  val mask   = IO(Input(UInt(8.W)))
  val rdata  = IO(Output(UInt(64.W)))
}

class DCacheWrapper(parentName:String = "Unknown")(implicit p: Parameters) extends LazyModule with HasXSParameter {

  val useDcache = coreParams.dcacheParametersOpt.nonEmpty
  val clientNode = if (useDcache) TLIdentityNode() else null
  val dcache = if (useDcache) LazyModule(new DCache(parentName = parentName)) else null
  if (useDcache) {
    clientNode := dcache.clientNode
  }
  lazy val module = new DCacheWrapperImp(this)
}
class DCacheWrapperImp(outer:DCacheWrapper) extends LazyModuleImp(outer) with HasPerfEvents {
  val io = IO(new DCacheIO)
  val perfEvents = if (!outer.useDcache) {
    // a fake dcache which uses dpi-c to access memory, only for debug usage!
    val fake_dcache = Module(new FakeDCache())
    io <> fake_dcache.io
    Seq()
  } else {
    io <> outer.dcache.module.io
    outer.dcache.module.getPerfEvents
  }
  generatePerfEvent()
}
