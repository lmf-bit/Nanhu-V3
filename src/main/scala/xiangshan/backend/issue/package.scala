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
package xiangshan.backend

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan.backend.rob.RobPtr
import xiangshan.frontend.FtqPtr
import xiangshan.{ExuInput, FuType, SrcState, SrcType, XSBundle}

package object issue {
  abstract class BasicStatusArrayEntry(val srcNum: Int)(implicit p: Parameters) extends XSBundle {
    val psrc: Vec[UInt] = Vec(srcNum, UInt(PhyRegIdxWidth.W))
    val pdest: UInt = UInt(PhyRegIdxWidth.W)
    val srcType: Vec[UInt] = Vec(srcNum, SrcType())
    val srcState: Vec[UInt] = Vec(srcNum, SrcState())
    val lpv: Vec[Vec[UInt]] = Vec(srcNum, Vec(loadUnitNum, UInt(LpvLength.W)))
    val fuType: UInt = FuType()
    val rfWen: Bool = Bool()
    val fpWen: Bool = Bool()
    val robIdx: RobPtr = new RobPtr
    val ftqPtr: FtqPtr = new FtqPtr
    val ftqOffset: UInt = UInt(log2Up(PredictWidth).W)
  }

  class BasicWakeupInfo(implicit p: Parameters) extends XSBundle {
    val pdest: UInt = UInt(PhyRegIdxWidth.W)
    val destType: UInt = SrcType()
    val robPtr = new RobPtr
  }

  class WakeUpInfo(implicit p: Parameters) extends BasicWakeupInfo {
    val lpv: Vec[UInt] = Vec(loadUnitNum, UInt(LpvLength.W))
  }

  class EarlyWakeUpInfo(implicit p: Parameters) extends BasicWakeupInfo {
    val lpv: UInt = UInt(LpvLength.W)
  }

  object RsType {
    def int: Int = 0

    def mem: Int = 1

    def fp: Int = 2

    def vec: Int = 3

    def vperm: Int = 4
  }

  case class RsParam
  (
    name: String,
    rsType: Int,
    entriesNum: Int = 48,
    bankNum: Int = 4
  ) {
    //Unchangeable parameters
    require(entriesNum % bankNum == 0)
    val entryNumPerBank: Int = entriesNum / bankNum
    val isIntRs: Boolean = rsType == RsType.int
    val isMemRs: Boolean = rsType == RsType.mem
    val isFpRs: Boolean = rsType == RsType.fp
    val isVecRs: Boolean = rsType == RsType.vec
    val isVpRs: Boolean = rsType == RsType.vperm
    val isLegal: Boolean = isIntRs || isMemRs || isFpRs || isVecRs || isVpRs

    def TypeName: String = {
      require(isLegal)
      if (isIntRs) {
        "Integer RS "
      } else if (isFpRs) {
        "Floating RS "
      } else if (isVecRs) {
        "Vector RS "
      } else if(isVpRs) {
        "VectorPermutation RS"
      } else {
        "Memory RS"
      }
    }
  }

  object RSFeedbackType {
    val width = 3
    val tlbMiss: UInt = 7.U(width.W)
    val mshrFull: UInt = 4.U(width.W)
    val dataInvalid: UInt = 3.U(width.W)
    val bankConflict: UInt = 0.U(width.W)
    val ldVioCheckRedo: UInt = 0.U(width.W)
    val success: UInt = 6.U(width.W)  ///tmp
    val replayQFull: UInt = 7.U(width.W)

    def apply(): UInt = UInt(width.W)
  }

  class RSFeedback(implicit p: Parameters) extends XSBundle {
    val rsIdx = new RsIdx
    val sourceType: UInt = RSFeedbackType()
  }

  class RSFeedbackIO(implicit p: Parameters) extends XSBundle {
    // Note: you need to update in implicit Parameters p before imp MemRSFeedbackIO
    // for instance: MemRSFeedbackIO()(updateP)
    val feedbackSlowLoad = ValidIO(new RSFeedback) // dcache miss queue full, dtlb miss
    val feedbackFastLoad = ValidIO(new RSFeedback) // bank conflict
    val feedbackSlowStore = ValidIO(new RSFeedback)
  }

  class IssueBundle(implicit p: Parameters) extends XSBundle {
    val issue = DecoupledIO(new ExuInput)
    val rsIdx: RsIdx = Output(new RsIdx)
    val rsFeedback: RSFeedbackIO = Flipped(new RSFeedbackIO)
    val auxValid = Output(Bool())
    val isFirstIssue = Output(Bool())
    val hold = Output(Bool())
    val specialPsrc = Output(UInt(PhyRegIdxWidth.W))
    val specialPsrcType = Output(SrcType())
    val specialPsrcRen = Output(Bool())
  }

  class RsIdx(implicit p: Parameters) extends XSBundle {
    val bankIdxOH: UInt = UInt(coreParams.rsBankNum.W)
    val entryIdxOH: UInt = UInt((coreParams.maxRsEntryNum / coreParams.rsBankNum).W)
  }
}
