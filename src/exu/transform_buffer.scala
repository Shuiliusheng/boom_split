///// chw  ////


package boom.exu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.{Parameters}
import freechips.rocketchip.rocket.{MStatus, BP, BreakpointUnit}

import boom.common._
import boom.util.{BoolToChar, MaskUpper}
import boom.util._


class DecodeUops(val width_enq: Int)(implicit p: Parameters) extends BoomBundle
{
    val dec_uops = Vec(width_enq, new MicroOp())
    val val_mask = Vec(width_enq, Bool())
}

class TransResp(implicit p: Parameters) extends BoomBundle
{
    val tran_uops = Vec(coreWidth, new MicroOp())
    val tran_valids = Vec(coreWidth, Bool())
}


class TransBuffer(implicit p: Parameters) extends BoomModule
  with HasBoomCoreParameters
{
  val numEntries = 48
  val enqWidth = tranBuff_enq_width
  val io = IO(new BoomBundle {
    val enq = Flipped(Decoupled(new DecodeUops(tranBuff_enq_width)))
    val deq = new DecoupledIO(new TransResp())

    // Was the pipeline redirected? Clear/reset this buffer
    val clear = Input(Bool())
    val isUnicoreMode = Input(Bool())
  })

  require (numEntries > coreWidth)
  val numRows = numEntries / coreWidth
  val ram = Reg(Vec(numEntries, new MicroOp))
  ram.suggestName("tb_uop_ram")
  val deq_vec = Wire(Vec(numRows, Vec(coreWidth, new MicroOp)))
  val head = RegInit(1.U(numRows.W))
  val tail = RegInit(1.U(numEntries.W))

  val maybe_full = RegInit(false.B)

  //for debug
  val debug_cycles = freechips.rocketchip.util.WideCounter(32)

  //-------------------------------------------------------------
  // **** Enqueue Uops ****
  //-------------------------------------------------------------
  //判断每一条有效的dec指令是否能够转化，并且插入到buffer中，目前考虑的是判断转换后的数量是否能够一次性放到buffer中
  //转换后的指令插入到buffer中
  //buffer每个周期向rename发出coreWidth条指令
  //需要注意的：enq和deq的ready信号的设置条件
  //内部需要保存dec_uops, 并且再向前发出停止继续取指的请求

  //io.enq.ready: 用于告知外界当前是否能够插入新的指令
  //io.deq.ready: 用于外界告知buffer能够向外发出指令


  def rotateLeft(in: UInt, k: Int) = {
    val n = in.getWidth
    Cat(in(n-k-1,0), in(n-1, n-k))
  }

  val might_hit_head = (1 until enqWidth).map(k => VecInit(rotateLeft(tail, k).asBools.zipWithIndex.filter
    {case (e,i) => i % coreWidth == 0}.map {case (e,i) => e}).asUInt).map(tail => head & tail).reduce(_|_).orR
  val at_head = (VecInit(tail.asBools.zipWithIndex.filter {case (e,i) => i % coreWidth == 0}
    .map {case (e,i) => e}).asUInt & head).orR
  val do_enq = !(at_head && maybe_full || might_hit_head)

  io.enq.ready := do_enq

  // Input microops.
  val in_mask = io.enq.bits.val_mask
  val in_uops = io.enq.bits.dec_uops

  //chw: debug1
  when(io.isUnicoreMode && do_enq){
    for (i <- 0 until enqWidth) {
      printf("cycles: %d, transBuffer enq ready, w: %d, valid: %d, inst: 0x%x 0x%x\n", debug_cycles.value, i.U, in_mask(i), in_uops(i).debug_pc, in_uops(i).inst)
    }
  }


  // Step 2. Generate one-hot write indices.
  val enq_idxs = Wire(Vec(enqWidth, UInt(numEntries.W)))

  def inc(ptr: UInt) = {
    val n = ptr.getWidth
    Cat(ptr(n-2,0), ptr(n-1))
  }

  var enq_idx = tail
  for (i <- 0 until enqWidth) {
    enq_idxs(i) := enq_idx
    enq_idx = Mux(in_mask(i), inc(enq_idx), enq_idx)
  }
  for (j <- 0 until numEntries) {
    for (i <- 0 until enqWidth) {
      when (do_enq && in_mask(i) && enq_idxs(i)(j)) {
        ram(j) := in_uops(i)
      }
    }
  }

  //-------------------------------------------------------------
  // **** Dequeue Uops ****
  //-------------------------------------------------------------
  //冲突
  val tail_collisions = VecInit((0 until numEntries).map(i =>
                          head(i/coreWidth) && (!maybe_full || (i % coreWidth != 0).B))).asUInt & tail

  val slot_will_hit_tail = (0 until numRows).map(i => tail_collisions((i+1)*coreWidth-1, i*coreWidth)).reduce(_|_)
  val will_hit_tail = slot_will_hit_tail.orR

  val do_deq = io.deq.ready && !will_hit_tail

  val deq_valids = (~MaskUpper(slot_will_hit_tail)).asBools

  // Generate vec for dequeue read port.
  for (i <- 0 until numEntries) {
    deq_vec(i/coreWidth)(i%coreWidth) := ram(i)
  }

  io.deq.bits.tran_valids zip deq_valids           map {case (d,v) => d  := v}
  io.deq.bits.tran_uops zip Mux1H(head, deq_vec) map {case (d,q) => d  := q}
  io.deq.valid := deq_valids.reduce(_||_)

  //-------------------------------------------------------------
  // **** Update State ****
  //-------------------------------------------------------------

  when (do_enq) {
    tail := enq_idx
    when (in_mask.reduce(_||_)) {
      maybe_full := true.B
    }
  }

  when (do_deq) {
    head := inc(head)
    maybe_full := false.B
  }

  when (io.clear) {
    head := 1.U
    tail := 1.U
    maybe_full := false.B
    io.deq.bits.tran_valids map { f => f := false.B }
  }

  // TODO Is this necessary?
  when (reset.toBool) {
    io.deq.bits.tran_valids map { f => f := false.B }
  }
}
