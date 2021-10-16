//******************************************************************************
// Copyright (c) 2015 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Rename FreeList
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.exu

import chisel3._
import chisel3.util._
import boom.common._
import boom.util._
import freechips.rocketchip.config.Parameters

class RenameFreeList(
  val plWidth: Int,
  val numPregs: Int,
  val numLregs: Int)
  (implicit p: Parameters) extends BoomModule
{
  private val pregSz = log2Ceil(numPregs)
  private val n = numPregs

  val io = IO(new BoomBundle()(p) {
    // Physical register requests.
    val reqs          = Input(Vec(plWidth, Bool()))
    val alloc_pregs   = Output(Vec(plWidth, Valid(UInt(pregSz.W))))

    // Pregs returned by the ROB.
    val dealloc_pregs = Input(Vec(plWidth, Valid(UInt(pregSz.W))))

    // Branch info for starting new allocation lists.
    val ren_br_tags   = Input(Vec(plWidth, Valid(UInt(brTagSz.W))))

    // Mispredict info for recovering speculatively allocated registers.
    val brupdate        = Input(new BrUpdateInfo)

    //chw: freelist的io接口中的调试信号
    val cycles = Input(UInt(32.W))
    val is_print = Input(Bool())

    val debug = new Bundle {
      val pipeline_empty = Input(Bool())
      val freelist = Output(Bits(numPregs.W))
      val isprlist = Output(Bits(numPregs.W))
    }
  })
  // The free list register array and its branch allocation lists.
  val free_list = RegInit(UInt(numPregs.W), ~(1.U(numPregs.W)))
  val br_alloc_lists = Reg(Vec(maxBrCount, UInt(numPregs.W)))

  // Select pregs from the free list.
  val sels = SelectFirstN(free_list, plWidth)
  val sel_fire  = Wire(Vec(plWidth, Bool()))

  // Allocations seen by branches in each pipeline slot.
  val allocs = io.alloc_pregs map (a => UIntToOH(a.bits))
  val alloc_masks = (allocs zip io.reqs).scanRight(0.U(n.W)) { case ((a,r),m) => m | a & Fill(n,r) }

  // Masks that modify the freelist array.
  val sel_mask = (sels zip sel_fire) map { case (s,f) => s & Fill(n,f) } reduce(_|_)
  val br_deallocs = br_alloc_lists(io.brupdate.b2.uop.br_tag) & Fill(n, io.brupdate.b2.mispredict)
  val dealloc_mask = io.dealloc_pregs.map(d => UIntToOH(d.bits)(numPregs-1,0) & Fill(n,d.valid)).reduce(_|_) | br_deallocs

  val br_slots = VecInit(io.ren_br_tags.map(tag => tag.valid)).asUInt
  // Create branch allocation lists.
  for (i <- 0 until maxBrCount) {
    val list_req = VecInit(io.ren_br_tags.map(tag => UIntToOH(tag.bits)(i))).asUInt & br_slots
    val new_list = list_req.orR
    br_alloc_lists(i) := Mux(new_list, Mux1H(list_req, alloc_masks.slice(1, plWidth+1)),
                                       br_alloc_lists(i) & ~br_deallocs | alloc_masks(0))
  }

  // Update the free list.
  free_list := (free_list & ~sel_mask | dealloc_mask) & ~(1.U(numPregs.W))

  // Pipeline logic | hookup outputs.
  for (w <- 0 until plWidth) {
    val can_sel = sels(w).orR
    val r_valid = RegInit(false.B)
    val r_sel   = RegEnable(OHToUInt(sels(w)), sel_fire(w))

    r_valid := r_valid && !io.reqs(w) || can_sel
    sel_fire(w) := (!r_valid || io.reqs(w)) && can_sel

    io.alloc_pregs(w).bits  := r_sel
    io.alloc_pregs(w).valid := r_valid
  }

  io.debug.freelist := free_list | io.alloc_pregs.map(p => UIntToOH(p.bits) & Fill(n,p.valid)).reduce(_|_)
  io.debug.isprlist := 0.U  // TODO track commit free list.

  assert (!(io.debug.freelist & dealloc_mask).orR, "[freelist] Returning a free physical register.")
  assert (!io.debug.pipeline_empty || PopCount(io.debug.freelist) >= (numPregs - numLregs - 1).U,
    "[freelist] Leaking physical registers.")
}


//chw： flag寄存器的freelist
class FlagRenameFreeList(
  val plWidth: Int,
  val numPregs: Int,
  val numLregs: Int)
  (implicit p: Parameters) extends BoomModule
{
  private val pregSz = log2Ceil(numPregs)
  private val n = numPregs

  val io = IO(new BoomBundle()(p) {
    // Physical register requests.
    val reqs          = Input(Vec(plWidth, Bool()))
    val alloc_pregs   = Output(Vec(plWidth, Valid(UInt(pregSz.W))))

    // Pregs returned by the ROB.
    val dealloc_pregs = Input(Vec(plWidth, Valid(UInt(pregSz.W))))

    // Branch info for starting new allocation lists.
    val ren_br_tags   = Input(Vec(plWidth, Valid(UInt(brTagSz.W))))

    // Mispredict info for recovering speculatively allocated registers.
    val brupdate        = Input(new BrUpdateInfo)

    val cycles = Input(UInt(32.W))
    val is_print = Input(Bool())

    val debug = new Bundle {
      val pipeline_empty = Input(Bool())
      val freelist = Output(Bits(numPregs.W))
      val isprlist = Output(Bits(numPregs.W))
    }
  })
  // The free list register array and its branch allocation lists.
  // 初始化时，处理器会为每条plWidth指令分配一个目的寄存器，暂时用着，保证正确性？，但是后来也没办法释放掉
  // cycles:          0, sels0: 02, sels1: 04, sel_fire0: 1, sel_fire1: 1
  // cycles:          0, free_list: 1e, sel_mask: 06, dealloc_mask: 00, new_free: 18
  val free_list = RegInit(UInt(numPregs.W), ~(1.U(numPregs.W)))
  val br_alloc_lists = Reg(Vec(maxBrCount, UInt(numPregs.W)))

  // Select pregs from the free list.
  val sels = SelectFirstN(free_list, plWidth)
  val sel_fire  = Wire(Vec(plWidth, Bool()))

  // Allocations seen by branches in each pipeline slot.
  val allocs = io.alloc_pregs map (a => UIntToOH(a.bits))
  val alloc_masks = (allocs zip io.reqs).scanRight(0.U(n.W)) { case ((a,r),m) => m | a & Fill(n,r) }
  //io.reqs = ren2_alloc_reqs := ren2_uops(w).wflag && ren2_fire(w)

  // Masks that modify the freelist array.
  val sel_mask = (sels zip sel_fire) map { case (s,f) => s & Fill(n,f) } reduce(_|_)
  val br_deallocs = br_alloc_lists(io.brupdate.b2.uop.br_tag) & Fill(n, io.brupdate.b2.mispredict)
  val dealloc_mask = io.dealloc_pregs.map(d => UIntToOH(d.bits)(numPregs-1,0) & Fill(n,d.valid)).reduce(_|_) | br_deallocs

  val br_slots = VecInit(io.ren_br_tags.map(tag => tag.valid)).asUInt
  // Create branch allocation lists.
  for (i <- 0 until maxBrCount) {
    val list_req = VecInit(io.ren_br_tags.map(tag => UIntToOH(tag.bits)(i))).asUInt & br_slots
    val new_list = list_req.orR
    br_alloc_lists(i) := Mux(new_list, Mux1H(list_req, alloc_masks.slice(1, plWidth+1)),
                                       br_alloc_lists(i) & ~br_deallocs | alloc_masks(0))
  }
  // Update the free list.
  //sel_fire
  free_list := (free_list & ~sel_mask | dealloc_mask) & ~(1.U(numPregs.W))

  // Pipeline logic | hookup outputs.
  for (w <- 0 until plWidth) {
    val can_sel = sels(w).orR   //如果存在空闲的，就会成功true
    val r_valid = RegInit(false.B)
    val r_sel   = RegEnable(OHToUInt(sels(w)), sel_fire(w)) 
    //下个周期拿到具体的分配结果
    //sel_fire(W)则是用于判断能不能分配，并且是否需要分配，本周期得到结果

    //可以分配？
    //true：cansel=true || (上个周期的rvalid=true，并且当前没有请求)
    //下个周期是否的输出是否有效：分配了寄存器/不需要分配 = true
    r_valid := r_valid && !io.reqs(w) || can_sel
    //sel_fire决定了是否更新freelist，即是否真的要分配了
    //true：可以分配，并且有分配的需求或者是上周期没有分配到
    sel_fire(w) := (!r_valid || io.reqs(w)) && can_sel

    //使用的是当前周期的rsel，rvalid，其实就是上个周期rvalid和选出来的rsel
    io.alloc_pregs(w).bits  := r_sel
    io.alloc_pregs(w).valid := r_valid
    
  }

  io.debug.freelist := free_list | io.alloc_pregs.map(p => UIntToOH(p.bits) & Fill(n,p.valid)).reduce(_|_)
  io.debug.isprlist := 0.U  // TODO track commit free list.

  assert (!(io.debug.freelist & dealloc_mask).orR, "[freelist] Returning a free physical register.")
  assert (!io.debug.pipeline_empty || PopCount(io.debug.freelist) >= (numPregs - numLregs - 1).U,
    "[freelist] Leaking physical registers.")
}

/*

第一个ren周期ren1(decuop) => ren2， 此时的ren2会通过组合逻辑传递给freelist，产生分配 
第二个周期根据ren2获取r_sel，r_valid

dis_fire = dis_valid && !dis_stall
dis_valid = ren2_valid = last_cycle(dec_fire)	(dis_ready==true)

dis_ready = false
ren2_valid = last_cycle(ren2_valids) && !ren2_fire(w) = 1 1 && !( 0 1 ) = 1 0 = 2
dis_valid = ren2_valid

dis_fir = dis_valid && !dis_stall
		  (1 0) && !(1 0) = 0
		  
freelist中的rvalid记录了上个周期有没有分配成功

//0x008000273c 指令作为ren2(0)申请一个寄存器，分配了08（3）号寄存器
cycles:      69882, alloc_reqs: 1, sel_fire: 1, freelist: 18, sel_mask: 08  //分配了8
cycles:      69882, index: 0, alloc_reqs: 1, sel_fire: 1, cansel: 1, rsel: 1, rvalid: 1
cycles:      69882, index: 1, alloc_reqs: 0, sel_fire: 0, cansel: 1, rsel: 2, rvalid: 1
cycles:      69882, index: 0, pc: 0x008000273c, inst: 0x0931c246, wflag: 1,  preg: 1
cycles:      69882, index: 1, pc: 0x008000275e, inst: 0x00c90913, wflag: 0,  preg: 2

//第二个周期，0x008000273c获取到自己申请的寄存器
cycles:      69883, index: 0, alloc_reqs: 0, sel_fire: 0, cansel: 1, rsel: 3, rvalid: 1 //上个周期分配了8
cycles:      69883, index: 1, alloc_reqs: 1, sel_fire: 0, cansel: 0, rsel: 2, rvalid: 1
cycles:      69883, index: 0, pc: 0x008000273c, inst: 0x0931c246, wflag: 1,  preg: 3
cycles:      69883, index: 1, pc: 0x0080002740, inst: 0x0931c246, wflag: 1,  preg: 2


第一个周期：
如果dispatch没有被阻止，即ren2_ready = io.dis_ready = true，则从dec获取uop，并且获取fire信号，即哪些decuop是有效的
  - 设置ren2_valid=dec_fire, ren2_uops=dec_uops
  - 而dis_valids 等同于 ren2_valid， 连接着dec_fire，表示下个周期重命名的结果中哪些是有效的
如果dispatch没有完全发出去，即ren2_ready = io.dis_ready = false， 则不更新ren2，继续对ren2剩余的指令进行重命名
  - 此时ren2_valid=!dis_fire, 即当前ren2中没有被成功重命名被dispatch的指令

当ren2被更新之后，freelist的组合逻辑会判断出当前是否能够分配有效的重命名（sel_fire）

第二个周期：
  根据ren2的信息和组合逻辑产生的sel_fire，freelist给出重命名的结果，并且更新freelist

  alloc_reqs=dis_fire，但是此时的dis_fire，由于是组合逻辑，已经变成了new_ren2_valid && !old_stall, 即很可能是全零
  但是此时全零没有影响，因为上个周期需要分配的指令，他们在freelist中的rvalid信号已经变成的false，
  此时可以设置sel_fire=true（前提是can_sel）, 而valid由于是can_sel，因此也会正常变为valid

  在本周期最后，将能够获取到最后分配的物理寄存器和stall信号，然后再次更新dis_fire
  即此时dis_fire = new_ren2_valid & !new_stall, 然后指导取指和下一个周期的重命名


多个不同的renamestage停顿位置不一样，dis_fire会被设置在最早停顿的那个位置，这样是否会存在影响？
不会有影响
1. 由于dis_fire导致的dis_ready=false会使得dis_valid = ren2_valids = ~dis_fire, 
   从而导致组合逻辑中 dis_fire = dis_valid & !stall 变成 dis_fire = ~dis_fire & !stall， 即全为零的状态 （因此此时stall还未更新）
   所以最终的结果就是alloc_req = 0.U，不会有申请

2. 在freelist中，由于sel_fire(w) := (!r_valid || io.reqs(w)) && can_sel，并且如果该重命名逻辑在上个周期成功分配了，则r_valid=true，而且目前req=0， 
   所以sel_fire = 0, 所以不会更改freelist，即不会出现新的问题


*/