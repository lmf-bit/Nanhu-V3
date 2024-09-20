/***************************************************************************************
 * Copyright (c) 2020-2023 Institute of Computing Technology, Chinese Academy of Sciences
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
/***************************************************************************************
 * Author: Liang Sen
 * E-mail: liangsen20z@ict.ac.cn
 * Date: 2023-06-19
 ****************************************************************************************/
package xiangshan.backend.execute.exu

import xiangshan.backend.execute.fu.FuConfig
import chisel3._
object ExuType{
  // Exu Pipeline: 00->int, 01->fp, 10->mem, 11->vector
  // Int: 'b00000 -> 'b00110
  def jmp   = 0
  def alu   = 1
  def mul   = 2
  def div   = 3
  def bru   = 4
  def misc  = 5
  def stdi  = 6

  // Fp: 'b01000 -> 'b01011
  def fmisc = 8
  def fmac  = 9
  def fdiv  = 10
  def stdf  = 11

  // Mem: 'b10000 -> 'b10001
  def ldu   = 16
  def sta   = 17
  def stdv  = 18
  
  // Vector: 'b11000->'b11111
  def valu  = 24
  def vmac  = 25
  def vfp   = 26
  def vdiv  = 27
  def vperm = 28
  def s2v   = 29
  def sldu  = 30
  def vmask = 31

  private val mapping = Map(
    jmp   -> "jmp",
    alu   -> "alu",
    mul   -> "mul",
    div   -> "div",
    bru   -> "bru",
    misc  -> "misc",
    stdi  -> "stdi",
    fmisc -> "fmisc",
    fmac  -> "fmac",
    fdiv  -> "fdiv",
    stdf  -> "stdf",
    ldu   -> "ldu",
    sta   -> "sta",
    valu  -> "valu",
    vfp   -> "vfp",
    vdiv  -> "vdiv",
    vmac  -> "vmac",
    vperm -> "vperm",
    s2v   -> "s2v",
    sldu  -> "sldu",
    vmask -> "vmask",
    stdv  -> "stdv"
  )

  def intTypes: Seq[Int] = Seq(alu, mul, div, jmp, misc, bru, stdi)
  def memTypes: Seq[Int] = Seq(ldu, sta, sldu)
  def fpTypes: Seq[Int] = Seq(fmisc, fmac, fdiv, stdf)
  def vecTypes: Seq[Int] = Seq(vfp, valu, vperm, vmac, vdiv, s2v)
  def typeToString(in:Int):String = mapping(in)
  def bypassIntList: Seq[Int] = Seq(alu, mul, ldu, jmp, bru)
  def bypassFpList: Seq[Int] = Seq(fmac)
}

case class ExuConfig
(
  name: String,
  id:Int,
  complexName: String,
  fuConfigs: Seq[FuConfig],
  exuType:Int,
  writebackToRob:Boolean,
  writebackToVms:Boolean,
  needToken:Boolean = false,
  speculativeWakeup:Boolean = false,
  throughVectorRf:Boolean = false
){
  private val intFastWkpSeq = Seq(ExuType.alu, ExuType.jmp, ExuType.mul)
  private val fpFastWkpSeq = Seq(ExuType.fmac, ExuType.mul)
  val intSrcNum:Int = fuConfigs.map(_.numIntSrc).max
  val fpSrcNum:Int = fuConfigs.map(_.numFpSrc).max
  val isIntFastWakeup: Boolean = intFastWkpSeq.contains(exuType)
  val isFpFastWakeup: Boolean = fpFastWkpSeq.contains(exuType)
  val latency: Int = fuConfigs.map(_.latency).max
  val exceptionOut: Seq[Int] = fuConfigs.map(_.exceptionOut).reduce(_ ++ _).distinct.sorted
  val writeIntRf = fuConfigs.map(_.writeIntRf).reduce(_||_)
  val writeFpRf = fuConfigs.map(_.writeFpRf).reduce(_||_)
  val writeVecRf = fuConfigs.map(_.writeVecRf).reduce(_||_)
  val writeFFlags: Boolean = fuConfigs.map(_.writeFflags).reduce(_ || _)

  private val isVector = throughVectorRf
  private val isLs = exuType == ExuType.ldu || exuType == ExuType.sta
  val writebackToRegfile = if(isLs) !isVector && (writeIntRf || writeFpRf) else (writeIntRf || writeFpRf)
  val writebackToIntRs = if(isLs) (!isVector && writeIntRf) else (writeIntRf && !isIntFastWakeup)
  val writebackToFpRs = if(isLs) (!isVector && writeFpRf) else (writeFpRf && !isFpFastWakeup)
  val writebackToReorderQueue = writebackToRob
  val writebackToVecRs = writeVecRf || writeIntRf || writeFpRf
  val writebackToMergeStation = writebackToVms
  val writebackToMemRs = ((writeIntRf || writeFpRf) && !isIntFastWakeup && !isFpFastWakeup) || writeVecRf
  val isVldu = isVector && exuType == ExuType.ldu

  val hasRedirectOut = fuConfigs.map(_.hasRedirect).reduce(_||_)
  val isIntType = ExuType.intTypes.contains(exuType)
  val isMemType = ExuType.memTypes.contains(exuType)
  val isFpType = ExuType.fpTypes.contains(exuType)
  val isVecType = ExuType.vecTypes.contains(exuType)
  val willTriggerVrfWkp = fuConfigs.map(_.triggerVrfWakeup).reduce(_||_)
  val bypassIntRegfile = ExuType.bypassIntList.contains(exuType)
  val bypassFpRegfile = ExuType.bypassFpList.contains(exuType)
  val trigger: Boolean = fuConfigs.map(_.trigger).reduce(_ || _)
  val hasException: Boolean = exceptionOut.nonEmpty || trigger

  override def toString = s"\n\t${name}: intSrcNum: ${intSrcNum} fpSrcNum: ${fpSrcNum} Type: ${ExuType.typeToString(exuType)} " +
    "\n\t\t Functions Units: " + fuConfigs.map(_.toString + " ").reduce(_++_)
}
