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
import xiangshan.ExceptionNO._
import xiangshan._
import xiangshan.cache._
import xiangshan.cache.mmu.{TlbCmd, TlbReq, TlbRequestIO, TlbResp}

trait HasL1PrefetchSourceParameter {
  // l1 prefetch source related
  def L1PfSourceBits = 3
  def L1_HW_PREFETCH_NULL = 0.U
  def L1_HW_PREFETCH_STRIDE = 1.U
  def L1_HW_PREFETCH_STREAM = 2.U
  def L1_HW_PREFETCH_STORE  = 3.U

  def isFromL1Prefetch(value: UInt) = value =/= L1_HW_PREFETCH_NULL
  def isFromStride(value: UInt)     = value === L1_HW_PREFETCH_STRIDE
  def isFromStream(value: UInt)     = value === L1_HW_PREFETCH_STREAM
}

class L1PrefetchSource(implicit p: Parameters) extends XSBundle with HasL1PrefetchSourceParameter {
  val value = UInt(L1PfSourceBits.W)
}

class L1PrefetchReq(implicit p: Parameters) extends XSBundle with HasDCacheParameters {
  val paddr = UInt(PAddrBits.W)
  val alias = UInt(2.W)
  val confidence = UInt(1.W)
  val is_store = Bool()
  val pf_source = new L1PrefetchSource

  // only index bit is used, do not use tag
  def getVaddr(): UInt = {
    Cat(alias, paddr(DCacheSameVPAddrLength-1, 0))
  }

  // when l1 cache prefetch req arrives at load unit:
  // if (confidence == 1)
  //   override load unit 2 load req
  // else if (load unit 1/2 is available)
  //   send prefetch req
  // else
  //   report prefetch !ready
}

class L1PrefetchHint(implicit p: Parameters) extends XSBundle with HasDCacheParameters {
  val loadbusy = Bool()
  val missqbusy = Bool()
}
