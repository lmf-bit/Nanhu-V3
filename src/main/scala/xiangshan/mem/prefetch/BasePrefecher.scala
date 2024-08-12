package xiangshan.mem.prefetch

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xiangshan._
import xiangshan.cache.mmu.TlbRequestIO
import xiangshan.mem.{LsPipelineBundle, L1PrefetchReq}

class L2PrefetchReq(implicit p: Parameters) extends XSBundle {
  val addr = UInt(PAddrBits.W)
}

class PrefetcherIO()(implicit p: Parameters) extends XSBundle {
  val ld_in = Flipped(Vec(exuParameters.LduCnt, ValidIO(new LsPipelineBundle())))
  val tlb_req = new TlbRequestIO(nRespDups = 2)
  val l1_req = DecoupledIO(new L1PrefetchReq())  
  val l2_req = ValidIO(new L2PrefetchReq())
  val enable = Input(Bool())
}

class PrefetchReqBundle()(implicit p: Parameters) extends XSBundle {
  val vaddr = UInt(VAddrBits.W)
  val paddr = UInt(PAddrBits.W)
  val pc    = UInt(VAddrBits.W)
  val miss  = Bool()
}

trait PrefetcherParams

abstract class BasePrefecher()(implicit p: Parameters) extends XSModule {
  val io = IO(new PrefetcherIO())
}