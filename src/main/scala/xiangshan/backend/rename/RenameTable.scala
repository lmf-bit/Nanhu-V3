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

package xiangshan.backend.rename

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan._
import xs.utils.ParallelPriorityMux
import xs.utils.perf.HasPerfLogging

class RatReadPort(implicit p: Parameters) extends XSBundle {
  val hold = Input(Bool())
  val addr = Input(UInt(5.W))
  val data = Output(UInt(PhyRegIdxWidth.W))
}

class RatWritePort(implicit p: Parameters) extends XSBundle {
  val wen = Bool()
  val addr = UInt(5.W)
  val data = UInt(PhyRegIdxWidth.W)
}


class RenameTable(float: Boolean)(implicit p: Parameters) extends XSModule{
  val io = IO(new Bundle {
    val redirect = Input(Bool())
    val readPorts = Vec({if(float) 4 else 3} * RenameWidth, new RatReadPort)
    val specWritePorts = Vec(CommitWidth, Input(new RatWritePort))
    val archWritePorts = Vec(CommitWidth, Input(new RatWritePort))
    val old_pdest = Vec(RabCommitWidth, Output(UInt(PhyRegIdxWidth.W)))
    val need_free = Vec(RabCommitWidth, Output(Bool()))
    val snpt = Input(new SnapshotPort)
    val debug_rdata = Vec(32, Output(UInt(PhyRegIdxWidth.W)))
    val diffWritePorts = if (env.EnableDifftest || env.AlwaysBasicDiff) Some(Vec(RabCommitWidth * RenameWidth, Input(new RatWritePort))) else None
  })

  // speculative rename table
  val rename_table_init = VecInit.tabulate(32)(i => (if (float) i else 0).U(PhyRegIdxWidth.W))
  val spec_table = RegInit(rename_table_init)
  val spec_table_next = WireInit(spec_table)
  // arch state rename table
  val arch_table = RegInit(rename_table_init)
  val arch_table_next = WireDefault(arch_table)
  // old_pdest
  val old_pdest = RegInit(VecInit.fill(RabCommitWidth)(0.U(PhyRegIdxWidth.W)))
  val need_free = RegInit(VecInit.fill(RabCommitWidth)(false.B))

  // For better timing, we optimize reading and writing to RenameTable as follows:
  // (1) Writing at T0 will be actually processed at T1.
  // (2) Reading is synchronous now.
  // (3) RAddr at T0 will be used to access the table and get data at T0.
  // (4) WData at T0 is bypassed to RData at T1.
  val t1_rdata = io.readPorts.map(p => Mux(p.hold, p.data, spec_table_next(p.addr)))
  val t1_raddr = io.readPorts.map(p => p.addr)
  val t1_wSpec = WireInit(VecInit(Seq.fill(CommitWidth)(0.U.asTypeOf(new RatWritePort))))
  t1_wSpec.zip(io.specWritePorts).foreach({case(a,b) =>
  //  a.wen := RegNext(b.wen, false.B)
  //  a.addr := RegEnable(b.addr, b.wen)
  //  a.data := RegEnable(b.data, b.wen)
    when(b.wen) {
      a.wen := b.wen
      a.addr := b.addr
      a.data := b.data
    }
  })

  val snapshots = SnapshotGenerator(spec_table, io.snpt.snptEnq, io.snpt.snptDeq, io.redirect, io.snpt.flushVec)

  // WRITE: when instruction commits or walking
  val t1_wSpec_addr = t1_wSpec.map(w => Mux(w.wen, UIntToOH(w.addr), 0.U))
  val hasSpecWrite = t1_wSpec.map(_.wen).reduce(_ || _)
  for ((next, i) <- spec_table_next.zipWithIndex) {
    val matchVec = t1_wSpec_addr.map(w => w(i))
    val wMatch = ParallelPriorityMux(matchVec.reverse, t1_wSpec.map(_.data).reverse)
    // When there's a flush, we use arch_table to update spec_table.
    next := Mux(io.redirect,
      Mux(io.snpt.useSnpt, snapshots(io.snpt.snptSelect)(i), arch_table_next(i)),
      Mux(VecInit(matchVec).asUInt.orR, wMatch, spec_table(i)))
  }
  spec_table := spec_table_next

  // READ: decode-rename stage
  for ((r, i) <- io.readPorts.zipWithIndex) {
    // We use two comparisons here because r.hold has bad timing but addrs have better timing.
//    val t0_bypass = io.specWritePorts.map(w => w.wen && Mux(r.hold, w.addr === t1_raddr(i), w.addr === r.addr))
//    val t1_bypass = RegNext(VecInit(t0_bypass))
    val t1_bypass = io.specWritePorts.map(w => w.wen && Mux(r.hold, w.addr === t1_raddr(i), w.addr === r.addr))
    val t1_bypassVec = VecInit(t1_bypass)
    val bypass_data = ParallelPriorityMux(t1_bypass.reverse, t1_wSpec.map(_.data).reverse)
    r.data := spec_table(r.addr)
  }

for ((w, i) <- io.archWritePorts.zipWithIndex) {
    when (w.wen) {
      arch_table_next(w.addr) := w.data
    }
    val arch_mask = VecInit.fill(PhyRegIdxWidth)(w.wen).asUInt
    old_pdest(i) :=
      MuxCase(arch_table(w.addr) & arch_mask,
              io.archWritePorts.take(i).reverse.map(x => (x.wen && x.addr === w.addr, x.data & arch_mask)))
  }
  arch_table := arch_table_next

  for (((old, free), i) <- (old_pdest zip need_free).zipWithIndex) {
    val hasDuplicate = old_pdest.take(i).map(_ === old)
    val blockedByDup = if (i == 0) false.B else VecInit(hasDuplicate).asUInt.orR
    free := VecInit(arch_table.map(_ =/= old)).asUInt.andR && !blockedByDup
  }

  io.old_pdest := old_pdest
  io.need_free := need_free

  io.debug_rdata := arch_table

  if (env.EnableDifftest || env.AlwaysBasicDiff) {
    val difftest_table = RegInit(rename_table_init)
    val difftest_table_next = WireDefault(difftest_table)

    for (w <- io.diffWritePorts.get) {
      when(w.wen) {
        difftest_table_next(w.addr) := w.data
      }
    }
    difftest_table := difftest_table_next

    io.debug_rdata := difftest_table
  }
  else {
    io.debug_rdata.foreach(_ := 0.U.asTypeOf(io.debug_rdata))
  }

}

class RenameTableWrapper(implicit p: Parameters) extends XSModule with HasPerfLogging{
  val io = IO(new Bundle() {
    val redirect = Input(Bool())
    val robCommits = Flipped(new RobCommitIO)
    val rabCommits = Input(new RabCommitIO)
    val intReadPorts = Vec(RenameWidth, Vec(3, new RatReadPort))
    val intRenamePorts = Vec(RenameWidth, Input(new RatWritePort))
    val fpReadPorts = Vec(RenameWidth, Vec(4, new RatReadPort))
    val fpRenamePorts = Vec(RenameWidth, Input(new RatWritePort))
    val int_old_pdest = Vec(RabCommitWidth, Output(UInt(PhyRegIdxWidth.W)))
    val int_need_free = Vec(RabCommitWidth, Output(Bool()))
    val fp_old_pdest = Vec(RabCommitWidth, Output(UInt(PhyRegIdxWidth.W)))
    val snpt = Input(new SnapshotPort)
    // for debug printing
    val debug_int_rat = Vec(32, Output(UInt(PhyRegIdxWidth.W)))
    val debug_fp_rat = Vec(32, Output(UInt(PhyRegIdxWidth.W)))
    val diffCommits = if (env.EnableDifftest || env.AlwaysBasicDiff) Some(Input(new DiffCommitIO)) else None
  })

  val intRat = Module(new RenameTable(float = false))
  val fpRat = Module(new RenameTable(float = true))

  io.int_old_pdest := intRat.io.old_pdest
  io.int_need_free := intRat.io.need_free

  intRat.io.debug_rdata <> io.debug_int_rat
  intRat.io.readPorts <> io.intReadPorts.flatten
  intRat.io.redirect := io.redirect
  intRat.io.snpt := io.snpt
  val intDestValid = io.rabCommits.info.map(_.rfWen)
  for ((arch, i) <- intRat.io.archWritePorts.zipWithIndex) {
    arch.wen  := io.rabCommits.isCommit && io.rabCommits.commitValid(i) && intDestValid(i)
    arch.addr := io.rabCommits.info(i).ldest
    arch.data := io.rabCommits.info(i).pdest
    XSError(arch.wen && arch.addr === 0.U && arch.data =/= 0.U, "pdest for $0 should be 0\n")
  }
  for ((spec, i) <- intRat.io.specWritePorts.zipWithIndex) {
    spec.wen  := io.rabCommits.isWalk && io.rabCommits.walkValid(i) && intDestValid(i)
    spec.addr := io.rabCommits.info(i).ldest
    spec.data := io.rabCommits.info(i).pdest
    XSError(spec.wen && spec.addr === 0.U && spec.data =/= 0.U, "pdest for $0 should be 0\n")
  }
  for ((spec, rename) <- intRat.io.specWritePorts.zip(io.intRenamePorts)) {
    when (rename.wen) {
      spec.wen  := true.B
      spec.addr := rename.addr
      spec.data := rename.data
    }
  }
  if (env.EnableDifftest || env.AlwaysBasicDiff) {
    for ((diff, i) <- intRat.io.diffWritePorts.get.zipWithIndex) {
      diff.wen := io.diffCommits.get.isCommit && io.diffCommits.get.commitValid(i) && io.diffCommits.get.info(i).rfWen
      diff.addr := io.diffCommits.get.info(i).ldest
      diff.data := io.diffCommits.get.info(i).pdest
    }
  }

  // debug read ports for difftest
  fpRat.io.debug_rdata <> io.debug_fp_rat
  fpRat.io.readPorts <> io.fpReadPorts.flatten
  fpRat.io.redirect := io.redirect
  fpRat.io.snpt := io.snpt
  io.fp_old_pdest := fpRat.io.old_pdest
  for ((arch, i) <- fpRat.io.archWritePorts.zipWithIndex) {
    arch.wen  := io.rabCommits.isCommit && io.rabCommits.commitValid(i) && io.rabCommits.info(i).fpWen
    arch.addr := io.rabCommits.info(i).ldest
    arch.data := io.rabCommits.info(i).pdest
  }
  for ((spec, i) <- fpRat.io.specWritePorts.zipWithIndex) {
    spec.wen  := io.rabCommits.isWalk && io.rabCommits.walkValid(i) && io.rabCommits.info(i).fpWen
    spec.addr := io.rabCommits.info(i).ldest
    spec.data := io.rabCommits.info(i).pdest
  }
  for ((spec, rename) <- fpRat.io.specWritePorts.zip(io.fpRenamePorts)) {
    when (rename.wen) {
      spec.wen  := true.B
      spec.addr := rename.addr
      spec.data := rename.data
    }
  }
  if (env.EnableDifftest || env.AlwaysBasicDiff) {
    for ((diff, i) <- fpRat.io.diffWritePorts.get.zipWithIndex) {
      diff.wen := io.diffCommits.get.isCommit && io.diffCommits.get.commitValid(i) && io.diffCommits.get.info(i).fpWen
      diff.addr := io.diffCommits.get.info(i).ldest
      diff.data := io.diffCommits.get.info(i).pdest
    }
  }

}
