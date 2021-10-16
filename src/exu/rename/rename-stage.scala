//******************************************************************************
// Copyright (c) 2012 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISCV Processor Datapath: Rename Logic
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Supports 1-cycle and 2-cycle latencies. (aka, passthrough versus registers between ren1 and ren2).
//    - ren1: read the map tables and allocate a new physical register from the freelist.
//    - ren2: read the busy table for the physical operands.
//
// Ren1 data is provided as an output to be fed directly into the ROB.

package boom.exu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters

import boom.common._
import boom.util._

/**
 * IO bundle to interface with the Register Rename logic
 *
 * @param plWidth pipeline width
 * @param numIntPregs number of int physical registers
 * @param numFpPregs number of FP physical registers
 * @param numWbPorts number of int writeback ports
 * @param numWbPorts number of FP writeback ports
 */
class RenameStageIO(
  val plWidth: Int,
  val numPhysRegs: Int,
  val numWbPorts: Int)
  (implicit p: Parameters) extends BoomBundle


/**
 * IO bundle to debug the rename stage
 */
class DebugRenameStageIO(val numPhysRegs: Int)(implicit p: Parameters) extends BoomBundle
{
  val freelist  = Bits(numPhysRegs.W)
  val isprlist  = Bits(numPhysRegs.W)
  val busytable = UInt(numPhysRegs.W)
}

abstract class AbstractRenameStage(
  plWidth: Int,
  numPhysRegs: Int,
  numWbPorts: Int)
  (implicit p: Parameters) extends BoomModule
{
  val io = IO(new Bundle {
    val ren_stalls = Output(Vec(plWidth, Bool()))

    val kill = Input(Bool())

    val dec_fire  = Input(Vec(plWidth, Bool())) // will commit state updates
    val dec_uops  = Input(Vec(plWidth, new MicroOp()))

    // physical specifiers available AND busy/ready status available.
    val ren2_mask = Vec(plWidth, Output(Bool())) // mask of valid instructions
    val ren2_uops = Vec(plWidth, Output(new MicroOp()))

    // branch resolution (execute)
    val brupdate = Input(new BrUpdateInfo())

    val dis_fire  = Input(Vec(coreWidth, Bool()))
    val dis_ready = Input(Bool())

    // wakeup ports
    val wakeups = Flipped(Vec(numWbPorts, Valid(new ExeUnitResp(xLen))))

    // commit stage
    val com_valids = Input(Vec(plWidth, Bool()))
    val com_uops = Input(Vec(plWidth, new MicroOp()))
    val rbk_valids = Input(Vec(plWidth, Bool()))
    val rollback = Input(Bool())

    val debug_rob_empty = Input(Bool())
    val debug = Output(new DebugRenameStageIO(numPhysRegs))

    //chw：寄存器重命名阶段的io信号中增加is_unicore信号，用于对第三个源寄存器进行重命名
    val is_unicore = Input(Bool())
  })

  def BypassAllocations(uop: MicroOp, older_uops: Seq[MicroOp], alloc_reqs: Seq[Bool]): MicroOp

  //-------------------------------------------------------------
  // Pipeline State & Wires

  // Stage 1
  val ren1_fire       = Wire(Vec(plWidth, Bool()))
  val ren1_uops       = Wire(Vec(plWidth, new MicroOp))


  // Stage 2
  val ren2_fire       = io.dis_fire
  val ren2_ready      = io.dis_ready
  val ren2_valids     = Wire(Vec(plWidth, Bool()))
  val ren2_uops       = Wire(Vec(plWidth, new MicroOp))
  val ren2_alloc_reqs = Wire(Vec(plWidth, Bool()))


  //-------------------------------------------------------------
  // pipeline registers

  for (w <- 0 until plWidth) {
    ren1_fire(w)          := io.dec_fire(w)
    ren1_uops(w)          := io.dec_uops(w)
  }

  for (w <- 0 until plWidth) {
    val r_valid  = RegInit(false.B)
    val r_uop    = Reg(new MicroOp)
    val next_uop = Wire(new MicroOp)

    next_uop := r_uop

    when (io.kill) {
      r_valid := false.B
    } .elsewhen (ren2_ready) {
      r_valid := ren1_fire(w)
      next_uop := ren1_uops(w)
    } .otherwise {
      r_valid := r_valid && !ren2_fire(w) // clear bit if uop gets dispatched
      next_uop := r_uop
    }

    r_uop := GetNewUopAndBrMask(BypassAllocations(next_uop, ren2_uops, ren2_alloc_reqs), io.brupdate)

    ren2_valids(w) := r_valid
    ren2_uops(w)   := r_uop
  }

  //-------------------------------------------------------------
  // Outputs

  io.ren2_mask := ren2_valids


}


/**
 * Rename stage that connets the map table, free list, and busy table.
 * Can be used in both the FP pipeline and the normal execute pipeline.
 *
 * @param plWidth pipeline width
 * @param numWbPorts number of int writeback ports
 * @param numWbPorts number of FP writeback ports
 */
class RenameStage(
  plWidth: Int,
  numPhysRegs: Int,
  numWbPorts: Int,
  float: Boolean)
(implicit p: Parameters) extends AbstractRenameStage(plWidth, numPhysRegs, numWbPorts)(p)
{
  val pregSz = log2Ceil(numPhysRegs)
  val rtype = if (float) RT_FLT else RT_FIX

  //-------------------------------------------------------------
  // Helper Functions

  def BypassAllocations(uop: MicroOp, older_uops: Seq[MicroOp], alloc_reqs: Seq[Bool]): MicroOp = {
    val bypassed_uop = Wire(new MicroOp)
    bypassed_uop := uop

    // val bypass_hits_rs1 = (older_uops zip alloc_reqs) map { case (r,a) => a && r.ldst === uop.lrs1 }
    // val bypass_hits_rs2 = (older_uops zip alloc_reqs) map { case (r,a) => a && r.ldst === uop.lrs2 }
    // val bypass_hits_rs3 = (older_uops zip alloc_reqs) map { case (r,a) => a && r.ldst === uop.lrs3 }
    // val bypass_hits_dst = (older_uops zip alloc_reqs) map { case (r,a) => a && r.ldst === uop.ldst }
    //增加dst_rtype的判断
    val bypass_hits_rs1 = (older_uops zip alloc_reqs) map { case (r,a) => a && r.ldst === uop.lrs1 && r.dst_rtype === uop.lrs1_rtype}
    val bypass_hits_rs2 = (older_uops zip alloc_reqs) map { case (r,a) => a && r.ldst === uop.lrs2 && r.dst_rtype === uop.lrs2_rtype}
    val bypass_hits_rs3 = (older_uops zip alloc_reqs) map { case (r,a) => a && r.ldst === uop.lrs3 && r.dst_rtype === uop.lrs3_rtype}
    val bypass_hits_dst = (older_uops zip alloc_reqs) map { case (r,a) => a && r.ldst === uop.ldst && r.dst_rtype === uop.dst_rtype}

    val bypass_sel_rs1 = PriorityEncoderOH(bypass_hits_rs1.reverse).reverse
    val bypass_sel_rs2 = PriorityEncoderOH(bypass_hits_rs2.reverse).reverse
    val bypass_sel_rs3 = PriorityEncoderOH(bypass_hits_rs3.reverse).reverse
    val bypass_sel_dst = PriorityEncoderOH(bypass_hits_dst.reverse).reverse

    val do_bypass_rs1 = bypass_hits_rs1.reduce(_||_)
    val do_bypass_rs2 = bypass_hits_rs2.reduce(_||_)
    val do_bypass_rs3 = bypass_hits_rs3.reduce(_||_)
    val do_bypass_dst = bypass_hits_dst.reduce(_||_)

    val bypass_pdsts = older_uops.map(_.pdst)

    when (do_bypass_rs1) { bypassed_uop.prs1       := Mux1H(bypass_sel_rs1, bypass_pdsts) }
    when (do_bypass_rs2) { bypassed_uop.prs2       := Mux1H(bypass_sel_rs2, bypass_pdsts) }
    //chw： 对定点的第三个源寄存器进行重命名
    when (do_bypass_rs3) { bypassed_uop.prs3  := Mux(!float.B && !io.is_unicore, DontCare, Mux1H(bypass_sel_rs3, bypass_pdsts)) }
    when (do_bypass_dst) { bypassed_uop.stale_pdst := Mux1H(bypass_sel_dst, bypass_pdsts) }

    bypassed_uop.prs1_busy := uop.prs1_busy || do_bypass_rs1
    bypassed_uop.prs2_busy := uop.prs2_busy || do_bypass_rs2
    bypassed_uop.prs3_busy := Mux(!float.B && !io.is_unicore, false.B, uop.prs3_busy || do_bypass_rs3)

    bypassed_uop
  }

  //-------------------------------------------------------------
  // Rename Structures
  //chw: 逻辑寄存器从32->36，四个用于临时寄存器使用
  // val maptable = Module(new RenameMapTable(plWidth, 32, numPhysRegs, false, float))
  val maptable = Module(new RenameMapTable(plWidth, 36, numPhysRegs, false, float))
  // val freelist = Module(new RenameFreeList(plWidth, numPhysRegs, if (float) 32 else 31))
  val freelist = Module(new RenameFreeList(plWidth, numPhysRegs, 36))
  val busytable = Module(new RenameBusyTable(plWidth, numPhysRegs, numWbPorts, false,float))

  //chw: debug info
  val debug_cycles = freechips.rocketchip.util.WideCounter(32)

  //chw：增加unicore的接口，调试输出使用
  maptable.io.is_unicore := io.is_unicore
  busytable.io.is_unicore := io.is_unicore
  freelist.io.cycles := debug_cycles.value
  freelist.io.is_print:= io.is_unicore


  val ren2_br_tags    = Wire(Vec(plWidth, Valid(UInt(brTagSz.W))))

  // Commit/Rollback
  val com_valids      = Wire(Vec(plWidth, Bool()))
  val rbk_valids      = Wire(Vec(plWidth, Bool()))

  for (w <- 0 until plWidth) {
    ren2_alloc_reqs(w)    := ren2_uops(w).ldst_val && ren2_uops(w).dst_rtype === rtype && ren2_fire(w)
    ren2_br_tags(w).valid := ren2_fire(w) && ren2_uops(w).allocate_brtag

    com_valids(w)         := io.com_uops(w).ldst_val && io.com_uops(w).dst_rtype === rtype && io.com_valids(w)
    rbk_valids(w)         := io.com_uops(w).ldst_val && io.com_uops(w).dst_rtype === rtype && io.rbk_valids(w)
    ren2_br_tags(w).bits  := ren2_uops(w).br_tag
  }

  //-------------------------------------------------------------
  // Rename Table

  // Maptable inputs.
  val map_reqs   = Wire(Vec(plWidth, new MapReq(lregSz)))
  val remap_reqs = Wire(Vec(plWidth, new RemapReq(lregSz, pregSz)))

  // Generate maptable requests.
  for ((((ren1,ren2),com),w) <- ren1_uops zip ren2_uops zip io.com_uops.reverse zipWithIndex) {
    map_reqs(w).lrs1 := ren1.lrs1
    map_reqs(w).lrs2 := ren1.lrs2
    map_reqs(w).lrs3 := ren1.lrs3
    map_reqs(w).ldst := ren1.ldst

    remap_reqs(w).ldst := Mux(io.rollback, com.ldst      , ren2.ldst)
    remap_reqs(w).pdst := Mux(io.rollback, com.stale_pdst, ren2.pdst)
  }
  ren2_alloc_reqs zip rbk_valids.reverse zip remap_reqs map {
    case ((a,r),rr) => rr.valid := a || r}

  // Hook up inputs.
  maptable.io.map_reqs    := map_reqs
  maptable.io.remap_reqs  := remap_reqs
  maptable.io.ren_br_tags := ren2_br_tags
  maptable.io.brupdate      := io.brupdate
  maptable.io.rollback    := io.rollback

  // Maptable outputs.
  for ((uop, w) <- ren1_uops.zipWithIndex) {
    val mappings = maptable.io.map_resps(w)

    uop.prs1       := mappings.prs1
    uop.prs2       := mappings.prs2
    uop.prs3       := mappings.prs3 // only FP has 3rd operand
    uop.stale_pdst := mappings.stale_pdst
  }



  //-------------------------------------------------------------
  // Free List

  // Freelist inputs.
  freelist.io.reqs := ren2_alloc_reqs
  freelist.io.dealloc_pregs zip com_valids zip rbk_valids map
    {case ((d,c),r) => d.valid := c || r}
  freelist.io.dealloc_pregs zip io.com_uops map
    {case (d,c) => d.bits := Mux(io.rollback, c.pdst, c.stale_pdst)}
  freelist.io.ren_br_tags := ren2_br_tags
  freelist.io.brupdate := io.brupdate
  freelist.io.debug.pipeline_empty := io.debug_rob_empty

  assert (ren2_alloc_reqs zip freelist.io.alloc_pregs map {case (r,p) => !r || p.bits =/= 0.U || (p.bits === 0.U && io.is_unicore)} reduce (_&&_),
           "[rename-stage] A uop is trying to allocate the zero physical register.")

  // Freelist outputs.
  for ((uop, w) <- ren2_uops.zipWithIndex) {
    val preg = freelist.io.alloc_pregs(w).bits
    //chw: 寄存器重命名， 在unicore模式下，不阻止zero reg的重命名
    uop.pdst := Mux(uop.ldst =/= 0.U || float.B || io.is_unicore, preg, 0.U)
  }

  //-------------------------------------------------------------
  // Busy Table

  busytable.io.ren_uops := ren2_uops  // expects pdst to be set up.
  busytable.io.rebusy_reqs := ren2_alloc_reqs
  busytable.io.wb_valids := io.wakeups.map(_.valid)
  busytable.io.wb_pdsts := io.wakeups.map(_.bits.uop.pdst)

  assert (!(io.wakeups.map(x => x.valid && x.bits.uop.dst_rtype =/= rtype).reduce(_||_)),
   "[rename] Wakeup has wrong rtype.")

  for ((uop, w) <- ren2_uops.zipWithIndex) {
    val busy = busytable.io.busy_resps(w)

    uop.prs1_busy := uop.lrs1_rtype === rtype && busy.prs1_busy
    uop.prs2_busy := uop.lrs2_rtype === rtype && busy.prs2_busy
    //chw: 重命名阶段, 修改对第三个源寄存器的busy位的更新设置
    uop.prs3_busy := Mux(io.is_unicore, (uop.lrs3_rtype === rtype) && busy.prs3_busy, uop.frs3_en && busy.prs3_busy)

    val valid = ren2_valids(w)
    //chw： rename stage, 修改断言
    assert (!(valid && busy.prs1_busy && rtype === RT_FIX && uop.lrs1 === 0.U && !io.is_unicore), "[rename] x0 is busy??")
    assert (!(valid && busy.prs2_busy && rtype === RT_FIX && uop.lrs2 === 0.U && !io.is_unicore), "[rename] x0 is busy??")
  }

  //-------------------------------------------------------------
  // Outputs

  for (w <- 0 until plWidth) {
    val can_allocate = freelist.io.alloc_pregs(w).valid

    // Push back against Decode stage if Rename1 can't proceed.
    io.ren_stalls(w) := (ren2_uops(w).dst_rtype === rtype) && !can_allocate

    val bypassed_uop = Wire(new MicroOp)
    if (w > 0) bypassed_uop := BypassAllocations(ren2_uops(w), ren2_uops.slice(0,w), ren2_alloc_reqs.slice(0,w))
    else       bypassed_uop := ren2_uops(w)

    io.ren2_uops(w) := GetNewUopAndBrMask(bypassed_uop, io.brupdate)
  }

  //-------------------------------------------------------------
  // Debug signals

  io.debug.freelist  := freelist.io.debug.freelist
  io.debug.isprlist  := freelist.io.debug.isprlist
  io.debug.busytable := busytable.io.debug.busytable
}

class PredRenameStage(
  plWidth: Int,
  numPhysRegs: Int,
  numWbPorts: Int)
  (implicit p: Parameters) extends AbstractRenameStage(plWidth, numPhysRegs, numWbPorts)(p)
{

  def BypassAllocations(uop: MicroOp, older_uops: Seq[MicroOp], alloc_reqs: Seq[Bool]): MicroOp = {
    uop
  }

  ren2_alloc_reqs := DontCare

  val busy_table = RegInit(VecInit(0.U(ftqSz.W).asBools))
  val to_busy = WireInit(VecInit(0.U(ftqSz.W).asBools))
  val unbusy = WireInit(VecInit(0.U(ftqSz.W).asBools))

  val current_ftq_idx = Reg(UInt(log2Ceil(ftqSz).W))
  var next_ftq_idx = current_ftq_idx

  for (w <- 0 until plWidth) {
    io.ren2_uops(w) := ren2_uops(w)

    val is_sfb_br = ren2_uops(w).is_sfb_br && ren2_fire(w)
    val is_sfb_shadow = ren2_uops(w).is_sfb_shadow && ren2_fire(w)

    val ftq_idx = ren2_uops(w).ftq_idx
    when (is_sfb_br) {
      io.ren2_uops(w).pdst := ftq_idx
      to_busy(ftq_idx) := true.B
    }
    next_ftq_idx = Mux(is_sfb_br, ftq_idx, next_ftq_idx)

    when (is_sfb_shadow) {
      io.ren2_uops(w).ppred := next_ftq_idx
      io.ren2_uops(w).ppred_busy := (busy_table(next_ftq_idx) || to_busy(next_ftq_idx)) && !unbusy(next_ftq_idx)
    }
  }

  for (w <- 0 until numWbPorts) {
    when (io.wakeups(w).valid) {
      unbusy(io.wakeups(w).bits.uop.pdst) := true.B
    }
  }

  current_ftq_idx := next_ftq_idx

  busy_table := ((busy_table.asUInt | to_busy.asUInt) & ~unbusy.asUInt).asBools
}


//标志寄存器的重命名逻辑
class FlagRenameStage(
  plWidth: Int,
  numPhysRegs: Int,
  numWbPorts: Int)
(implicit p: Parameters) extends AbstractRenameStage(plWidth, numPhysRegs, numWbPorts)(p)
{
  val pregSz = log2Ceil(numPhysRegs)
  val rtype = RT_FLT
  
  //-------------------------------------------------------------
  // Helper Functions
  //用于判断同一个周期内重命名的指令之间是否存在冲突
  def BypassAllocations(uop: MicroOp, older_uops: Seq[MicroOp], alloc_reqs: Seq[Bool]): MicroOp = {
    val bypassed_uop = Wire(new MicroOp)
    bypassed_uop := uop

    //前者写，当前读
    val bypass_hits_rflag = (older_uops zip alloc_reqs) map { case (r,a) => a && r.wflag && uop.rflag }
    //前者写，当前也是写，则需要更新stale
    val bypass_hits_wflag = (older_uops zip alloc_reqs) map { case (r,a) => a && r.wflag && uop.wflag }

    val bypass_sel_rflag = PriorityEncoderOH(bypass_hits_rflag.reverse).reverse
    val bypass_sel_wflag = PriorityEncoderOH(bypass_hits_wflag.reverse).reverse

    val do_bypass_rflag = bypass_hits_rflag.reduce(_||_)
    val do_bypass_wflag = bypass_hits_wflag.reduce(_||_)

    val bypass_pwflags = older_uops.map(_.pwflag)

    when (do_bypass_rflag) { bypassed_uop.prflag       := Mux1H(bypass_sel_rflag, bypass_pwflags) }
    when (do_bypass_wflag) { bypassed_uop.stale_pflag  := Mux1H(bypass_sel_wflag, bypass_pwflags) }

    ////更新busy状态，如果存在bypass，则意味着需要等待之前的指令执行完成，即寄存器处于busy状态
    bypassed_uop.pflag_busy := uop.pflag_busy || do_bypass_rflag

    bypassed_uop
  }

  //chw: debugcycle
  val debug_cycles = freechips.rocketchip.util.WideCounter(32)

  //-------------------------------------------------------------
  // Rename Structures
  //false, false: bypass&float = false
  //plWidth, numLregs, numPregs
  val maptable = Module(new RenameMapTable_Flag(plWidth, numPhysRegs, false))
  //int plWidth, int numPregs, int numLRegs
  val freelist = Module(new FlagRenameFreeList(plWidth, numPhysRegs, 1))
  // bool bypass
  val busytable = Module(new RenameBusyTable_Flag(plWidth, numPhysRegs, numWbPorts, false))


  val ren2_br_tags    = Wire(Vec(plWidth, Valid(UInt(brTagSz.W))))

  // Commit/Rollback
  val com_valids      = Wire(Vec(plWidth, Bool()))
  val rbk_valids      = Wire(Vec(plWidth, Bool()))

  val is_unicore_mode = ren2_uops.map(_.is_unicore).reduce(_||_) 


  for (w <- 0 until plWidth) {
    //需要写flag，因此需要分配
    ren2_alloc_reqs(w)    := ren2_uops(w).wflag && ren2_fire(w)
    ren2_br_tags(w).valid := ren2_fire(w) && ren2_uops(w).allocate_brtag

    com_valids(w)         := io.com_uops(w).wflag && io.com_valids(w)
    rbk_valids(w)         := io.com_uops(w).wflag && io.rbk_valids(w)
    ren2_br_tags(w).bits  := ren2_uops(w).br_tag

    //chw: debug printf
    /*when(io.com_valids(w) && io.com_uops(w).is_unicore){
      printf("cycles: %d, renamestage commit inst: pc 0x%x, 0x%x, rflag: %d, stale_flag: %d, wflag: %d, pwflag: %d\n", io.com_uops(w).cycles, Sext(io.com_uops(w).debug_pc(vaddrBits-1,0), xLen), io.com_uops(w).inst, io.com_uops(w).rflag, io.com_uops(w).stale_pflag, io.com_uops(w).wflag, io.com_uops(w).pwflag)
    }*/
    /*
    when(io.rbk_valids(w) && io.com_uops(w).is_unicore){
      printf("cycles: %d, renamestage rbk inst: pc 0x%x, 0x%x, rflag: %d, prflag: %d, wflag: %d, pwflag: %d\n", io.com_uops(w).cycles, Sext(io.com_uops(w).debug_pc(vaddrBits-1,0), xLen), io.com_uops(w).inst, io.com_uops(w).rflag, io.com_uops(w).prflag, io.com_uops(w).wflag, io.com_uops(w).pwflag)
    }*/
  }


  //------------------------------------------------------------------------------------------------------------------
  // Rename Table

  // Maptable inputs.
  val map_reqs   = Wire(Vec(plWidth, new MapReq_Flag()))
  val remap_reqs = Wire(Vec(plWidth, new RemapReq_Flag(pregSz)))

  // Generate maptable requests.
  for ((((ren1,ren2),com),w) <- ren1_uops zip ren2_uops zip io.com_uops.reverse zipWithIndex) {
    map_reqs(w).rflag := Mux(ren1.rflag, 0.U, 1.U)  //如果需要读/写，则逻辑号指定为0， 否则指定为1
    map_reqs(w).wflag := Mux(ren1.wflag, 0.U, 1.U)
    //map_reqs(w).ldst := ren1.ldst

    remap_reqs(w).wflag := Mux(Mux(io.rollback, com.wflag , ren2.wflag), 0.U, 1.U)
    remap_reqs(w).pwflag := Mux(io.rollback, com.stale_pflag, ren2.pwflag)
    //remap_reqs(w).ldst := Mux(io.rollback, com.ldst      , ren2.ldst)
    //remap_reqs(w).pdst := Mux(io.rollback, com.stale_pdst, ren2.pdst)
  }
  ren2_alloc_reqs zip rbk_valids.reverse zip remap_reqs map {
    case ((a,r),rr) => rr.valid := a || r}

  // Hook up inputs.
  maptable.io.map_reqs    := map_reqs
  maptable.io.remap_reqs  := remap_reqs
  maptable.io.ren_br_tags := ren2_br_tags
  maptable.io.brupdate      := io.brupdate
  maptable.io.rollback    := io.rollback

  // Maptable outputs.
  for ((uop, w) <- ren1_uops.zipWithIndex) {
    val mappings = maptable.io.map_resps(w)
    uop.prflag       := mappings.prflag
    uop.stale_pflag := mappings.stale_pflag
  }



  //--------------------------------------------------------------------------------------
  // Free List

  // Freelist inputs.
  freelist.io.reqs := ren2_alloc_reqs
  freelist.io.dealloc_pregs zip com_valids zip rbk_valids map
    {case ((d,c),r) => d.valid := c || r}
  
  //回滚/提交回收寄存器
  freelist.io.dealloc_pregs zip io.com_uops map
    {case (d,c) => d.bits := Mux(io.rollback, c.pwflag, c.stale_pflag)}
    //{case (d,c) => d.bits := Mux(io.rollback, c.pdst, c.stale_pdst)}
  
  freelist.io.ren_br_tags := ren2_br_tags
  freelist.io.brupdate := io.brupdate
  freelist.io.debug.pipeline_empty := io.debug_rob_empty
  //chw: debug
  freelist.io.cycles := debug_cycles.value
  freelist.io.is_print := is_unicore_mode

  // Freelist outputs.
  for ((uop, w) <- ren2_uops.zipWithIndex) {
    val preg = freelist.io.alloc_pregs(w).bits
    uop.pwflag := Mux(uop.wflag, preg, 0.U)
  }

  //---------------------------------------------------------------------------------------------------
  // Busy Table

  busytable.io.ren_uops := ren2_uops  // expects pdst to be set up.
  busytable.io.rebusy_reqs := ren2_alloc_reqs
  busytable.io.wb_valids := io.wakeups.map(_.valid)
  busytable.io.wb_pdsts := io.wakeups.map(_.bits.uop.pwflag)

  assert (!(io.wakeups.map(x => x.valid && !x.bits.uop.wflag).reduce(_||_)),
   "[rename] Wakeup flag: without wflag but wakeup.")

  for ((uop, w) <- ren2_uops.zipWithIndex) {
    val busy = busytable.io.busy_resps(w)
    uop.pflag_busy := uop.rflag && busy.pflag_busy
  }

  //-------------------------------------------------------------------------------------------------------
  // Outputs

  for (w <- 0 until plWidth) {
    val can_allocate = freelist.io.alloc_pregs(w).valid

    // Push back against Decode stage if Rename1 can't proceed.
    io.ren_stalls(w) := (ren2_uops(w).wflag) && !can_allocate
    //上个周期ren2给出的分配有效性

    val bypassed_uop = Wire(new MicroOp)
    if (w > 0) bypassed_uop := BypassAllocations(ren2_uops(w), ren2_uops.slice(0,w), ren2_alloc_reqs.slice(0,w))
    else       bypassed_uop := ren2_uops(w)
    io.ren2_uops(w) := GetNewUopAndBrMask(bypassed_uop, io.brupdate)
  }

  //-------------------------------------------------------------------------------------------------------
  // Debug signals

  io.debug.freelist  := freelist.io.debug.freelist
  io.debug.isprlist  := freelist.io.debug.isprlist
  io.debug.busytable := busytable.io.debug.busytable
}

