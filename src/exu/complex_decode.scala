//******************************************************************************
// Copyright (c) 2015 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

package boom.exu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket.Instructions._
import freechips.rocketchip.rocket.RVCExpander
import freechips.rocketchip.rocket.{CSR,Causes}
import freechips.rocketchip.util.{uintToBitPat,UIntIsOneOf}

import FUConstants._
import boom.common._
import boom.util._
import boom.common.UnicoreInsts._
// scalastyle:off

/**
 * IO bundle for the Decode unit
 */
class CompDecodeUnitIo(implicit p: Parameters) extends BoomBundle
{
  //chw: for tranform
  val enq = new Bundle { val uop = Input(new MicroOp()) }
  val deq = new Bundle {  val uops = Output(Vec(8, new MicroOp()))
                          val uop_valids = Output(Vec(8, Bool()))
                          //chw: for new transform
                          val num = Output(UInt(5.W))
                          val is_decoded = Output(Bool()) //用于表示指令是否能够被该单元译码
                          }

  // from CSRFile
  val status = Input(new freechips.rocketchip.rocket.MStatus())
  val csr_decode = Flipped(new freechips.rocketchip.rocket.CSRDecodeIO)
  val interrupt = Input(Bool())
  val interrupt_cause = Input(UInt(xLen.W))

  //chw: decode io, is_unicore
  val is_unicore = Input(Bool())
}



//chw unicore decode table
object CompUDecode extends DecodeConstants_Unicore
{
 //single-prec:单精
  val table: Array[(BitPat, List[BitPat])] = Array(
                //     is val inst?                        uses_ldq                      rd regtype                                       wakeup_delay
                //     |  is fp inst?                      |  uses_stq                   |       rs1 regtype                              |    bypassable
                //     |  |  is single-prec?               |  |  is_sys_pc2epc           |       |       rs2 regtype                      |    |    mem_cmd
                //     |  |  |  micro-code                 |  |  |  flush_on_commit      |       |       |       rs3 regtype              |    |    |    csr_cmd
                //     |  |  |  |        iq-type  func unit|  |  |  |  inst_unique       |       |       |       |        frs3_en         |    |    |    |      mem size
                //     |  |  |  |        |        |        |  |  |  |  |  is_br          |       |       |       |        |  wflag        |    |    |    |      |   shift way
                //     |  |  |  |        |        |        |  |  |  |  |  |  is_fence    |       |       |       |        |  |  rflag     |    |    |    |      |   |         Rd2 src
                //     |  |  |  |        |        |        |  |  |  |  |  |  |  is_fencei|       |       |       |        |  |  |  imm_sel|    |    |    |      |   |         |
                //     |  |  |  |        |        |        |  |  |  |  |  |  |  |  is_amo|       |       |       |        |  |  |  |      |    |    |    |      |   |         |
                //     |  |  |  |        |        |        |  |  |  |  |  |  |  |  |     |       |       |       |        |  |  |  |      |    |    |    |      |   |         |       
  ADD_L_IMM    -> List(Y, N, X, uopADD  ,IQT_INT, FU_ALU , N, N, N, N, N, N, N, N, N,    RT_FIX, RT_FIX ,RT_FIX ,RT_IMM5, N, N, N, IS_I,  1.U, Y,   M_X, CSR.N, MX, LG_LEFT , RD2_X  ),  
  MULSL        -> List(Y, N, X, uopMUL  ,IQT_INT, FU_MUL , N, N, N, N, N, N, N, N, N,    RT_FIX, RT_FIX ,RT_FIX ,RT_X   , N, N, N, IS_I,  0.U, N,   M_X, CSR.N, MX, NO_SHIFT, RD2_RS3) 
  )
  //add.a: 修改标志位， wflag
  //addc.a: 读标志位，并且修改标志位
  //有读标志寄存器，必须要有读寄存器，因为要将不修改得结果回写？
}



/**
 * Decode unit that takes in a single instruction and generates a MicroOp.
 */

//chw：复杂译码器
//基本框架和普通译码器一致，内部信号也没有更改，但是输出端口已经定好，只需要更改内部逻辑
class CompDecodeUnit(implicit p: Parameters) extends BoomModule
  with freechips.rocketchip.rocket.constants.MemoryOpConstants
{
  val io = IO(new CompDecodeUnitIo)

  val uop = Wire(new MicroOp())
  val uop1 = Wire(new MicroOp())
  val uop2 = Wire(new MicroOp())
  val uop3 = Wire(new MicroOp())
  uop := io.enq.uop

  val inst = uop.inst
  //chw: 创建unicore译码表，确定最终的译码结果
  var decode_table_unicore = CompUDecode.table
  val cs_u = Wire(new CtrlSigs_Unicore()).decode(inst, decode_table_unicore)

  val unicoreMode = io.is_unicore
  val cs_fp_val = cs_u.fp_val
  val cs_fp_single = cs_u.fp_single

  val cs_uopc = cs_u.uopc
  val cs_iq_type = cs_u.iq_type
  val cs_fu_code = cs_u.fu_code
  val cs_uses_ldq = cs_u.uses_ldq
  val cs_uses_stq = cs_u.uses_stq

  val cs_is_br = cs_u.is_br
  val cs_is_amo = cs_u.is_amo
  val cs_is_fence = cs_u.is_fence
  val cs_is_fencei = cs_u.is_fencei
  val cs_is_sys_pc2epc = cs_u.is_sys_pc2epc
  val cs_inst_unique = cs_u.inst_unique
  val cs_flush_on_commit = cs_u.flush_on_commit

  val cs_dst_type = cs_u.dst_type
  val cs_rs1_type = cs_u.rs1_type
  val cs_rs2_type = cs_u.rs2_type
  val cs_rs3_type = cs_u.rs3_type

  val cs_frs3_en = cs_u.frs3_en
  val cs_wflag = cs_u.wflag
  val cs_rflag = cs_u.rflag
  val cs_imm_sel = cs_u.imm_sel
  val cs_bypassable = cs_u.bypassable
  val cs_csr_cmd = cs_u.csr_cmd
  val cs_mem_cmd = cs_u.mem_cmd
  val cs_rocc = false.B


  // Exception Handling
  io.csr_decode.csr := inst(31,20)

  val id_illegal_insn = !cs_u.legal
  //check ok

  def checkExceptions(x: Seq[(Bool, UInt)]) =
    (x.map(_._1).reduce(_||_), PriorityMux(x))

  val (xcpt_valid, xcpt_cause) = checkExceptions(List(
    (io.interrupt,                        io.interrupt_cause),  // Disallow interrupts while we are handling a SFB
    (uop.bp_debug_if,                    (CSR.debugTriggerCause).U),
    (uop.bp_xcpt_if,                     (Causes.breakpoint).U),
    (uop.xcpt_pf_if,                     (Causes.fetch_page_fault).U),
    (uop.xcpt_ae_if,                     (Causes.fetch_access).U),
    (id_illegal_insn,                    (Causes.illegal_instruction).U)))

  uop.exception := xcpt_valid
  uop.exc_cause := xcpt_cause

  //-------------------------------------------------------------
  uop.is_unique := cs_inst_unique

  val debug_cycles = freechips.rocketchip.util.WideCounter(32)
  when(io.is_unicore){
    printf("cycles: %d, complex decode inst: 0x%x, 0x%x, lrs1: %d, lrs2: %d, valid: %d, expt: %d, r: %d, %d, %d, %d, %d, %d\n", debug_cycles.value, uop.debug_pc, inst, uop.lrs1, uop.lrs2, xcpt_valid, xcpt_cause, io.interrupt, uop.bp_debug_if, uop.bp_xcpt_if, uop.xcpt_pf_if, uop.xcpt_ae_if, id_illegal_insn)
  }

  //-------------------------指令属性------------------------------------
  uop.is_unicore      := io.is_unicore
  uop.is_br           := cs_is_br
  uop.is_jal          := (uop.uopc === uopJAL)
  uop.is_jalr         := (uop.uopc === uopJALR)
  uop.is_amo          := cs_is_amo
  uop.is_fence        := cs_is_fence
  uop.is_fencei       := cs_is_fencei
  uop.is_sys_pc2epc   := cs_is_sys_pc2epc
  uop.flush_on_commit := cs_flush_on_commit

  //-------------------------指令信息------------------------------------
  uop.uopc       := cs_uopc
  uop.iq_type    := cs_iq_type
  uop.fu_code    := cs_fu_code
  uop.uses_ldq   := cs_uses_ldq
  uop.uses_stq   := cs_uses_stq
  uop.bypassable   := cs_bypassable
  //shift
  uop.shift := Mux(unicoreMode, cs_u.shift, NO_SHIFT)

  //--------------------------------------------------------------------------------

  uop.ldst       := Mux(unicoreMode, inst(RD_MSB_UNICORE,RD_LSB_UNICORE),   inst(RD_MSB,RD_LSB))
  uop.lrs1       := Mux(unicoreMode, inst(RS1_MSB_UNICORE,RS1_LSB_UNICORE), inst(RS1_MSB,RS1_LSB))
  uop.lrs2       := Mux(unicoreMode, inst(RS2_MSB_UNICORE,RS2_LSB_UNICORE), inst(RS2_MSB,RS2_LSB))
  uop.lrs3       := Mux(unicoreMode, inst(RS3_MSB_UNICORE,RS3_LSB_UNICORE), inst(RS3_MSB,RS3_LSB))
  uop.wflag      := cs_wflag
  uop.rflag      := cs_rflag

  //--------------------------------------------------------------------------------

  val ldst_val_riscv   = cs_dst_type =/= RT_X && !(inst(RD_MSB,RD_LSB) === 0.U && uop.dst_rtype === RT_FIX)
  val ldst_val_unicore = cs_dst_type =/= RT_X && !(inst(RD_MSB_UNICORE,RD_LSB_UNICORE) === 31.U && uop.dst_rtype === RT_FIX)  //r31: pc
  uop.ldst_val   := Mux(unicoreMode, ldst_val_unicore, ldst_val_riscv)
  uop.dst_rtype  := cs_dst_type
  uop.lrs1_rtype := cs_rs1_type
  uop.lrs2_rtype := cs_rs2_type
  uop.lrs3_rtype := cs_rs3_type
  uop.frs3_en    := cs_frs3_en
  uop.fp_val     := cs_fp_val
  uop.fp_single  := cs_fp_single // TODO use this signal instead of the FPU decode's table signal?

  //---------------------------------------------------------------------------------

  uop.mem_cmd    := cs_mem_cmd
  uop.mem_size   := cs_u.mem_size
  uop.mem_signed := Mux(unicoreMode, false.B, !inst(14))

  //---------------------------------------------------------------------------------------
  val di24_20 = Mux(cs_imm_sel === IS_B || cs_imm_sel === IS_S, inst(11,7), inst(24,20))
  val imm_packed_riscv = Cat(inst(31,25), di24_20, inst(19,12))
  
  val imm_packed_unicore = inst(19,0)
  // uop.imm_8bits  := Cat(0.U(4.W), inst(23,20))
  uop.imm_packed := Mux(unicoreMode, imm_packed_unicore, imm_packed_riscv)


  //---------------------------------------------------------------------------------
  uop.ldst_is_rs1 := !io.is_unicore && uop.is_sfb_shadow

  //-------------------------------------------------------------
  //chw: for transform
  var decode_table_subinst1 = SubDecode1.table
  var decode_table_subinst2 = SubDecode2.table
  var decode_table_subinst3 = SubDecode3.table
  var decode_table_subinst4 = SubDecode4.table

  val cs_sub0 = Wire(new CtrlSigs_SubInst()).decode(inst, decode_table_subinst1)
  val cs_sub1 = Wire(new CtrlSigs_SubInst()).decode(inst, decode_table_subinst2)
  val cs_sub2 = Wire(new CtrlSigs_SubInst()).decode(inst, decode_table_subinst3)
  val cs_sub3 = Wire(new CtrlSigs_SubInst()).decode(inst, decode_table_subinst4)
  
  val subDecUnit0 = Module(new SubDecodeUnit)
  val subDecUnit1 = Module(new SubDecodeUnit)
  val subDecUnit2 = Module(new SubDecodeUnit)
  val subDecUnit3 = Module(new SubDecodeUnit)

  subDecUnit0.io.rawuop := uop
  subDecUnit1.io.rawuop := uop1
  subDecUnit2.io.rawuop := uop2
  subDecUnit3.io.rawuop := uop3

  subDecUnit0.io.cs_sub := cs_sub0
  subDecUnit1.io.cs_sub := cs_sub1
  subDecUnit2.io.cs_sub := cs_sub2
  subDecUnit3.io.cs_sub := cs_sub3

  uop1 := uop
  uop2 := uop
  uop3 := uop

  io.deq.is_decoded := !id_illegal_insn

  when(unicoreMode){
    //目前测试是将一条mulsl指令拆分了8条加法，即将mulsl作为复杂指令作为测试
    uop.split_num  := cs_sub0.split_num
    uop1.split_num := cs_sub0.split_num
    uop2.split_num := cs_sub0.split_num
    uop3.split_num := cs_sub0.split_num

    uop.is_unicore   := true.B
    uop1.is_unicore  := true.B
    uop2.is_unicore  := true.B
    uop3.is_unicore  := true.B

    uop.self_index  := 0.U
    uop1.self_index := 1.U
    uop2.self_index := 2.U
    uop3.self_index := 3.U

    io.deq.uop_valids(0) := cs_sub0.valid && io.deq.is_decoded
    io.deq.uop_valids(1) := cs_sub1.valid && io.deq.is_decoded
    io.deq.uop_valids(2) := cs_sub2.valid && io.deq.is_decoded
    io.deq.uop_valids(3) := cs_sub3.valid && io.deq.is_decoded

    io.deq.uops(0) := subDecUnit0.io.subuop
    io.deq.uops(1) := subDecUnit1.io.subuop
    io.deq.uops(2) := subDecUnit2.io.subuop
    io.deq.uops(3) := subDecUnit3.io.subuop

    //4-7复制0-3的输出
    io.deq.uop_valids(4) := cs_sub0.valid && io.deq.is_decoded
    io.deq.uop_valids(5) := cs_sub1.valid && io.deq.is_decoded
    io.deq.uop_valids(6) := cs_sub2.valid && io.deq.is_decoded
    io.deq.uop_valids(7) := cs_sub3.valid && io.deq.is_decoded

    val uop4 = Wire(new MicroOp())
    val uop5 = Wire(new MicroOp())
    val uop6 = Wire(new MicroOp())
    val uop7 = Wire(new MicroOp())

    uop4 := subDecUnit0.io.subuop
    uop5 := subDecUnit1.io.subuop
    uop6 := subDecUnit2.io.subuop
    uop7 := subDecUnit3.io.subuop

    uop4.self_index := 4.U
    uop5.self_index := 5.U
    uop6.self_index := 6.U
    uop7.self_index := 7.U


    io.deq.uops(4) := uop4
    io.deq.uops(5) := uop5
    io.deq.uops(6) := uop6
    io.deq.uops(7) := uop7

    //chw: debug1
    when(io.is_unicore){
      printf("cycles: %d, complex decode inst: 0x%x, 0x%x, num: %d, valid: %d, %d, %d, %d\n", debug_cycles.value, uop.debug_pc, inst, uop.split_num, cs_sub0.valid, cs_sub1.valid, cs_sub2.valid, cs_sub3.valid)
    }

    // io.deq.num := Mux(io.deq.is_decoded, uop.split_num, 0.U)
    io.deq.num := Mux(io.deq.is_decoded, 8.U, 0.U)
  }
  .otherwise{
    io.deq.uop_valids(0) := false.B
    io.deq.uop_valids(1) := false.B
    io.deq.uop_valids(2) := false.B
    io.deq.uop_valids(3) := false.B

    io.deq.uops(0) := uop
    io.deq.uops(1) := uop
    io.deq.uops(2) := uop
    io.deq.uops(3) := uop

    io.deq.uop_valids(4) := false.B
    io.deq.uop_valids(5) := false.B
    io.deq.uop_valids(6) := false.B
    io.deq.uop_valids(7) := false.B

    io.deq.uops(4) := uop
    io.deq.uops(5) := uop
    io.deq.uops(6) := uop
    io.deq.uops(7) := uop

    io.deq.num := 1.U
  }

}