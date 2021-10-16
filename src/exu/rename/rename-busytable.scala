//******************************************************************************
// Copyright (c) 2015 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Rename BusyTable
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.exu

import chisel3._
import chisel3.util._
import boom.common._
import boom.util._
import freechips.rocketchip.config.Parameters

class BusyResp extends Bundle
{
  val prs1_busy = Bool()
  val prs2_busy = Bool()
  val prs3_busy = Bool()
}

class RenameBusyTable(
  val plWidth: Int,
  val numPregs: Int,
  val numWbPorts: Int,
  val bypass: Boolean,
  val float: Boolean)
  (implicit p: Parameters) extends BoomModule
{
  val pregSz = log2Ceil(numPregs)

  val io = IO(new BoomBundle()(p) {
    val ren_uops = Input(Vec(plWidth, new MicroOp))
    val busy_resps = Output(Vec(plWidth, new BusyResp))
    val rebusy_reqs = Input(Vec(plWidth, Bool()))

    val wb_pdsts = Input(Vec(numWbPorts, UInt(pregSz.W)))
    val wb_valids = Input(Vec(numWbPorts, Bool()))

    //chw: rename busytable, io接口中增加is_unicore的信号
    val is_unicore = Input(Bool())

    val debug = new Bundle { val busytable = Output(Bits(numPregs.W)) }
  })

  val busy_table = RegInit(0.U(numPregs.W))
  // Unbusy written back registers.
  val busy_table_wb = busy_table & ~(io.wb_pdsts zip io.wb_valids)
    .map {case (pdst, valid) => UIntToOH(pdst) & Fill(numPregs, valid.asUInt)}.reduce(_|_)
  // Rebusy newly allocated registers.
  val busy_table_next = busy_table_wb | (io.ren_uops zip io.rebusy_reqs)
    .map {case (uop, req) => UIntToOH(uop.pdst) & Fill(numPregs, req.asUInt)}.reduce(_|_)

  busy_table := busy_table_next

  // Read the busy table.
  for (i <- 0 until plWidth) {
    val prs1_was_bypassed = (0 until i).map(j =>
      io.ren_uops(i).lrs1 === io.ren_uops(j).ldst && io.rebusy_reqs(j)).foldLeft(false.B)(_||_)
    val prs2_was_bypassed = (0 until i).map(j =>
      io.ren_uops(i).lrs2 === io.ren_uops(j).ldst && io.rebusy_reqs(j)).foldLeft(false.B)(_||_)
    val prs3_was_bypassed = (0 until i).map(j =>
      io.ren_uops(i).lrs3 === io.ren_uops(j).ldst && io.rebusy_reqs(j)).foldLeft(false.B)(_||_)

    io.busy_resps(i).prs1_busy := busy_table(io.ren_uops(i).prs1) || prs1_was_bypassed && bypass.B
    io.busy_resps(i).prs2_busy := busy_table(io.ren_uops(i).prs2) || prs2_was_bypassed && bypass.B
    val prs3_busy = busy_table(io.ren_uops(i).prs3) || prs3_was_bypassed && bypass.B

    //chw: busytable中更改prs3_busy信号的设置
    io.busy_resps(i).prs3_busy := Mux(!float.B && !io.is_unicore, false.B, prs3_busy)
  }

  io.debug.busytable := busy_table
}



//chw: flag重命名所使用的busytable
class BusyResp_Flag extends Bundle
{
  val pflag_busy = Bool()
}

class RenameBusyTable_Flag(
  val plWidth: Int,
  val numPregs: Int,
  val numWbPorts: Int,
  val bypass: Boolean)
  (implicit p: Parameters) extends BoomModule
{
  val pregSz = log2Ceil(numPregs)

  val io = IO(new BoomBundle()(p) {
    val ren_uops = Input(Vec(plWidth, new MicroOp))
    val busy_resps = Output(Vec(plWidth, new BusyResp_Flag))
    val rebusy_reqs = Input(Vec(plWidth, Bool()))

    val wb_pdsts = Input(Vec(numWbPorts, UInt(pregSz.W)))
    val wb_valids = Input(Vec(numWbPorts, Bool()))

    val debug = new Bundle { val busytable = Output(Bits(numPregs.W)) }
  })

  val busy_table = RegInit(0.U(numPregs.W))//初始均为空闲的
  // Unbusy written back registers.
  val busy_table_wb = busy_table & ~(io.wb_pdsts zip io.wb_valids)
    .map {case (pdst, valid) => UIntToOH(pdst) & Fill(numPregs, valid.asUInt)}.reduce(_|_)
  
  // Rebusy newly allocated registers.

  //0号物理寄存器会不会出现问题，因为valid=0时的特殊情况
  //rebusy_reqs = ren2_alloc_reqs
  val busy_table_next = busy_table_wb | (io.ren_uops zip io.rebusy_reqs)
    .map {case (uop, req) => UIntToOH(uop.pwflag) & Fill(numPregs, req.asUInt)}.reduce(_|_)

  busy_table := busy_table_next

  // Read the busy table.
  for (i <- 0 until plWidth) {
    //uopi需要读，uopj需要写，并且rebusy_reqsj有效，则应该标记为busy
    val pflag_was_bypassed = (0 until i).map(j =>
      io.ren_uops(i).rflag && io.ren_uops(j).wflag && io.rebusy_reqs(j)).foldLeft(false.B)(_||_)
  
    io.busy_resps(i).pflag_busy := busy_table(io.ren_uops(i).prflag) || pflag_was_bypassed && bypass.B
  }

  io.debug.busytable := busy_table
}

