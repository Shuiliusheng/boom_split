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
 * Abstract trait giving defaults and other relevant values to different Decode constants/
 */
abstract trait DecodeConstants
  extends freechips.rocketchip.rocket.constants.ScalarOpConstants
  with freechips.rocketchip.rocket.constants.MemoryOpConstants
{
  val xpr64 = Y // TODO inform this from xLen
  val DC2 = BitPat.dontCare(2) // Makes the listing below more readable
  def decode_default: List[BitPat] =
            //                                                                  frs3_en                        wakeup_delay
            //     is val inst?                                                 |  imm sel                     |    bypassable (aka, known/fixed latency)
            //     |  is fp inst?                                               |  |     uses_ldq              |    |  is_br
            //     |  |  is single-prec?                        rs1 regtype     |  |     |  uses_stq           |    |  |
            //     |  |  |  micro-code                          |       rs2 type|  |     |  |  is_amo          |    |  |
            //     |  |  |  |         iq-type  func unit        |       |       |  |     |  |  |  is_fence     |    |  |
            //     |  |  |  |         |        |                |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall?
            //     |  |  |  |         |        |        dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
            //     |  |  |  |         |        |        regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit
            //     |  |  |  |         |        |        |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd
            //     |  |  |  |         |        |        |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |
              List(N, N, X, uopX    , IQT_INT, FU_X   , RT_X  , DC2    ,DC2    ,X, IS_X, X, X, X, X, N, M_X,   DC2, X, X, N, N, X, CSR.X)

  val table: Array[(BitPat, List[BitPat])]
}
// scalastyle:on

/**
 * Decoded control signals
 */
class CtrlSigs extends Bundle
{
  val legal           = Bool()
  val fp_val          = Bool()
  val fp_single       = Bool()
  val uopc            = UInt(UOPC_SZ.W)
  val iq_type         = UInt(IQT_SZ.W)
  val fu_code         = UInt(FUC_SZ.W)
  val dst_type        = UInt(2.W)
  val rs1_type        = UInt(2.W)
  val rs2_type        = UInt(2.W)
  val frs3_en         = Bool()
  val imm_sel         = UInt(IS_X.getWidth.W)
  val uses_ldq        = Bool()
  val uses_stq        = Bool()
  val is_amo          = Bool()
  val is_fence        = Bool()
  val is_fencei       = Bool()
  val mem_cmd         = UInt(freechips.rocketchip.rocket.M_SZ.W)
  val wakeup_delay    = UInt(2.W)
  val bypassable      = Bool()
  val is_br           = Bool()
  val is_sys_pc2epc   = Bool()
  val inst_unique     = Bool()
  val flush_on_commit = Bool()
  val csr_cmd         = UInt(freechips.rocketchip.rocket.CSR.SZ.W)
  val rocc            = Bool()

  def decode(inst: UInt, table: Iterable[(BitPat, List[BitPat])]) = {
    val decoder = freechips.rocketchip.rocket.DecodeLogic(inst, XDecode.decode_default, table)
    val sigs =
      Seq(legal, fp_val, fp_single, uopc, iq_type, fu_code, dst_type, rs1_type,
          rs2_type, frs3_en, imm_sel, uses_ldq, uses_stq, is_amo,
          is_fence, is_fencei, mem_cmd, wakeup_delay, bypassable,
          is_br, is_sys_pc2epc, inst_unique, flush_on_commit, csr_cmd)
      sigs zip decoder map {case(s,d) => s := d}
      rocc := false.B
      this
  }
}

// scalastyle:off
/**
 * Decode constants for RV32
 */
object X32Decode extends DecodeConstants
{
            //                                                                  frs3_en                        wakeup_delay
            //     is val inst?                                                 |  imm sel                     |    bypassable (aka, known/fixed latency)
            //     |  is fp inst?                                               |  |     uses_ldq              |    |  is_br
            //     |  |  is single-prec?                        rs1 regtype     |  |     |  uses_stq           |    |  |
            //     |  |  |  micro-code                          |       rs2 type|  |     |  |  is_amo          |    |  |
            //     |  |  |  |         iq-type  func unit        |       |       |  |     |  |  |  is_fence     |    |  |
            //     |  |  |  |         |        |                |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall?
            //     |  |  |  |         |        |        dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
            //     |  |  |  |         |        |        regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit
            //     |  |  |  |         |        |        |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd
  val table: Array[(BitPat, List[BitPat])] = Array(//   |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |
  SLLI_RV32-> List(Y, N, X, uopSLLI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SRLI_RV32-> List(Y, N, X, uopSRLI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SRAI_RV32-> List(Y, N, X, uopSRAI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N)
  )
}

/**
 * Decode constants for RV64
 */
object X64Decode extends DecodeConstants
{
           //                                                                  frs3_en                        wakeup_delay
           //     is val inst?                                                 |  imm sel                     |    bypassable (aka, known/fixed latency)
           //     |  is fp inst?                                               |  |     uses_ldq              |    |  is_br
           //     |  |  is single-prec?                        rs1 regtype     |  |     |  uses_stq           |    |  |
           //     |  |  |  micro-code                          |       rs2 type|  |     |  |  is_amo          |    |  |
           //     |  |  |  |         iq-type  func unit        |       |       |  |     |  |  |  is_fence     |    |  |
           //     |  |  |  |         |        |                |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall?
           //     |  |  |  |         |        |        dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
           //     |  |  |  |         |        |        regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit
           //     |  |  |  |         |        |        |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd
  val table: Array[(BitPat, List[BitPat])] = Array(//  |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |
  LD      -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 3.U, N, N, N, N, N, CSR.N),
  LWU     -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 3.U, N, N, N, N, N, CSR.N),
  SD      -> List(Y, N, X, uopSTA  , IQT_MEM, FU_MEM , RT_X  , RT_FIX, RT_FIX, N, IS_S, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N),

  SLLI    -> List(Y, N, X, uopSLLI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SRLI    -> List(Y, N, X, uopSRLI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SRAI    -> List(Y, N, X, uopSRAI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),

  ADDIW   -> List(Y, N, X, uopADDIW, IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SLLIW   -> List(Y, N, X, uopSLLIW, IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SRAIW   -> List(Y, N, X, uopSRAIW, IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SRLIW   -> List(Y, N, X, uopSRLIW, IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),

  ADDW    -> List(Y, N, X, uopADDW , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SUBW    -> List(Y, N, X, uopSUBW , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SLLW    -> List(Y, N, X, uopSLLW , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SRAW    -> List(Y, N, X, uopSRAW , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SRLW    -> List(Y, N, X, uopSRLW , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N)
  )
}

/**
 * Overall Decode constants
 */
object XDecode extends DecodeConstants
{
           //                                                                  frs3_en                        wakeup_delay
           //     is val inst?                                                 |  imm sel                     |    bypassable (aka, known/fixed latency)
           //     |  is fp inst?                                               |  |     uses_ldq              |    |  is_br
           //     |  |  is single-prec?                        rs1 regtype     |  |     |  uses_stq           |    |  |
           //     |  |  |  micro-code                          |       rs2 type|  |     |  |  is_amo          |    |  |
           //     |  |  |  |         iq-type  func unit        |       |       |  |     |  |  |  is_fence     |    |  |
           //     |  |  |  |         |        |                |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall?
           //     |  |  |  |         |        |        dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
           //     |  |  |  |         |        |        regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit
           //     |  |  |  |         |        |        |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd
  val table: Array[(BitPat, List[BitPat])] = Array(//  |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |
  LW      -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 3.U, N, N, N, N, N, CSR.N),
  LH      -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 3.U, N, N, N, N, N, CSR.N),
  LHU     -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 3.U, N, N, N, N, N, CSR.N),
  LB      -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 3.U, N, N, N, N, N, CSR.N),
  LBU     -> List(Y, N, X, uopLD   , IQT_MEM, FU_MEM , RT_FIX, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 3.U, N, N, N, N, N, CSR.N),

  SW      -> List(Y, N, X, uopSTA  , IQT_MEM, FU_MEM , RT_X  , RT_FIX, RT_FIX, N, IS_S, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N),
  SH      -> List(Y, N, X, uopSTA  , IQT_MEM, FU_MEM , RT_X  , RT_FIX, RT_FIX, N, IS_S, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N),
  SB      -> List(Y, N, X, uopSTA  , IQT_MEM, FU_MEM , RT_X  , RT_FIX, RT_FIX, N, IS_S, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N),

  LUI     -> List(Y, N, X, uopLUI  , IQT_INT, FU_ALU , RT_FIX, RT_X  , RT_X  , N, IS_U, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),

  ADDI    -> List(Y, N, X, uopADDI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  ANDI    -> List(Y, N, X, uopANDI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  ORI     -> List(Y, N, X, uopORI  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  XORI    -> List(Y, N, X, uopXORI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SLTI    -> List(Y, N, X, uopSLTI , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SLTIU   -> List(Y, N, X, uopSLTIU, IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),

  SLL     -> List(Y, N, X, uopSLL  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  ADD     -> List(Y, N, X, uopADD  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SUB     -> List(Y, N, X, uopSUB  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SLT     -> List(Y, N, X, uopSLT  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SLTU    -> List(Y, N, X, uopSLTU , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  AND     -> List(Y, N, X, uopAND  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  OR      -> List(Y, N, X, uopOR   , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  XOR     -> List(Y, N, X, uopXOR  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SRA     -> List(Y, N, X, uopSRA  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_I, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),
  SRL     -> List(Y, N, X, uopSRL  , IQT_INT, FU_ALU , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 1.U, Y, N, N, N, N, CSR.N),

  MUL     -> List(Y, N, X, uopMUL  , IQT_INT, FU_MUL , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  MULH    -> List(Y, N, X, uopMULH , IQT_INT, FU_MUL , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  MULHU   -> List(Y, N, X, uopMULHU, IQT_INT, FU_MUL , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  MULHSU  -> List(Y, N, X, uopMULHSU,IQT_INT, FU_MUL , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  MULW    -> List(Y, N, X, uopMULW , IQT_INT, FU_MUL , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  DIV     -> List(Y, N, X, uopDIV  , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  DIVU    -> List(Y, N, X, uopDIVU , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  REM     -> List(Y, N, X, uopREM  , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  REMU    -> List(Y, N, X, uopREMU , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  DIVW    -> List(Y, N, X, uopDIVW , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  DIVUW   -> List(Y, N, X, uopDIVUW, IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  REMW    -> List(Y, N, X, uopREMW , IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  REMUW   -> List(Y, N, X, uopREMUW, IQT_INT, FU_DIV , RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  AUIPC   -> List(Y, N, X, uopAUIPC, IQT_INT, FU_JMP , RT_FIX, RT_X  , RT_X  , N, IS_U, N, N, N, N, N, M_X  , 1.U, N, N, N, N, N, CSR.N), // use BRU for the PC read
  JAL     -> List(Y, N, X, uopJAL  , IQT_INT, FU_JMP , RT_FIX, RT_X  , RT_X  , N, IS_J, N, N, N, N, N, M_X  , 1.U, N, N, N, N, N, CSR.N),
  JALR    -> List(Y, N, X, uopJALR , IQT_INT, FU_JMP , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 1.U, N, N, N, N, N, CSR.N),
  BEQ     -> List(Y, N, X, uopBEQ  , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , 0.U, N, Y, N, N, N, CSR.N),
  BNE     -> List(Y, N, X, uopBNE  , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , 0.U, N, Y, N, N, N, CSR.N),
  BGE     -> List(Y, N, X, uopBGE  , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , 0.U, N, Y, N, N, N, CSR.N),
  BGEU    -> List(Y, N, X, uopBGEU , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , 0.U, N, Y, N, N, N, CSR.N),
  BLT     -> List(Y, N, X, uopBLT  , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , 0.U, N, Y, N, N, N, CSR.N),
  BLTU    -> List(Y, N, X, uopBLTU , IQT_INT, FU_ALU , RT_X  , RT_FIX, RT_FIX, N, IS_B, N, N, N, N, N, M_X  , 0.U, N, Y, N, N, N, CSR.N),

  // I-type, the immediate12 holds the CSR register.
  CSRRW   -> List(Y, N, X, uopCSRRW, IQT_INT, FU_CSR , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.W),
  CSRRS   -> List(Y, N, X, uopCSRRS, IQT_INT, FU_CSR , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.S),
  CSRRC   -> List(Y, N, X, uopCSRRC, IQT_INT, FU_CSR , RT_FIX, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.C),

  CSRRWI  -> List(Y, N, X, uopCSRRWI,IQT_INT, FU_CSR , RT_FIX, RT_PAS, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.W),
  CSRRSI  -> List(Y, N, X, uopCSRRSI,IQT_INT, FU_CSR , RT_FIX, RT_PAS, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.S),
  CSRRCI  -> List(Y, N, X, uopCSRRCI,IQT_INT, FU_CSR , RT_FIX, RT_PAS, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.C),

  SFENCE_VMA->List(Y,N, X, uopSFENCE,IQT_MEM, FU_MEM , RT_X  , RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N,M_SFENCE,0.U,N, N, N, Y, Y, CSR.N),
  SCALL   -> List(Y, N, X, uopERET  ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, Y, Y, Y, CSR.I),
  SBREAK  -> List(Y, N, X, uopERET  ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, Y, Y, Y, CSR.I),
  SRET    -> List(Y, N, X, uopERET  ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.I),
  MRET    -> List(Y, N, X, uopERET  ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.I),
  DRET    -> List(Y, N, X, uopERET  ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.I),

  WFI     -> List(Y, N, X, uopWFI   ,IQT_INT, FU_CSR , RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, Y, Y, CSR.I),

  FENCE_I -> List(Y, N, X, uopNOP  , IQT_INT, FU_X   , RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, Y, M_X  , 0.U, N, N, N, Y, Y, CSR.N),
  FENCE   -> List(Y, N, X, uopFENCE, IQT_INT, FU_MEM , RT_X  , RT_X  , RT_X  , N, IS_X, N, Y, N, Y, N, M_X  , 0.U, N, N, N, Y, Y, CSR.N), // TODO PERF make fence higher performance
                                                                                                                                                       // currently serializes pipeline

           //                                                                  frs3_en                           wakeup_delay
           //     is val inst?                                                 |  imm sel                        |   bypassable (aka, known/fixed latency)
           //     |  is fp inst?                                               |  |     uses_ldq                 |   |  is_br
           //     |  |  is single-prec?                        rs1 regtype     |  |     |  uses_stq              |   |  |
           //     |  |  |  micro-code                          |       rs2 type|  |     |  |  is_amo             |   |  |
           //     |  |  |  |          iq-type  func unit       |       |       |  |     |  |  |  is_fence        |   |  |
           //     |  |  |  |          |        |               |       |       |  |     |  |  |  |  is_fencei    |   |  |  is breakpoint or ecall?
           //     |  |  |  |          |        |       dst     |       |       |  |     |  |  |  |  |  mem       |   |  |  |  is unique? (clear pipeline for it)
           //     |  |  |  |          |        |       regtype |       |       |  |     |  |  |  |  |  cmd       |   |  |  |  |  flush on commit
           //     |  |  |  |          |        |       |       |       |       |  |     |  |  |  |  |  |         |   |  |  |  |  |  csr cmd
  // A-type       |  |  |  |          |        |       |       |       |       |  |     |  |  |  |  |  |         |   |  |  |  |  |  |
  AMOADD_W-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_ADD, 0.U,N, N, N, Y, Y, CSR.N), // TODO make AMOs higherperformance
  AMOXOR_W-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_XOR, 0.U,N, N, N, Y, Y, CSR.N),
  AMOSWAP_W->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_SWAP,0.U,N, N, N, Y, Y, CSR.N),
  AMOAND_W-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_AND, 0.U,N, N, N, Y, Y, CSR.N),
  AMOOR_W -> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_OR,  0.U,N, N, N, Y, Y, CSR.N),
  AMOMIN_W-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MIN, 0.U,N, N, N, Y, Y, CSR.N),
  AMOMINU_W->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MINU,0.U,N, N, N, Y, Y, CSR.N),
  AMOMAX_W-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MAX, 0.U,N, N, N, Y, Y, CSR.N),
  AMOMAXU_W->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MAXU,0.U,N, N, N, Y, Y, CSR.N),

  AMOADD_D-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_ADD, 0.U,N, N, N, Y, Y, CSR.N),
  AMOXOR_D-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_XOR, 0.U,N, N, N, Y, Y, CSR.N),
  AMOSWAP_D->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_SWAP,0.U,N, N, N, Y, Y, CSR.N),
  AMOAND_D-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_AND, 0.U,N, N, N, Y, Y, CSR.N),
  AMOOR_D -> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_OR,  0.U,N, N, N, Y, Y, CSR.N),
  AMOMIN_D-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MIN, 0.U,N, N, N, Y, Y, CSR.N),
  AMOMINU_D->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MINU,0.U,N, N, N, Y, Y, CSR.N),
  AMOMAX_D-> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MAX, 0.U,N, N, N, Y, Y, CSR.N),
  AMOMAXU_D->List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XA_MAXU,0.U,N, N, N, Y, Y, CSR.N),

  LR_W    -> List(Y, N, X, uopLD    , IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_X  , N, IS_X, Y, N, N, N, N, M_XLR   , 0.U,N, N, N, Y, Y, CSR.N),
  LR_D    -> List(Y, N, X, uopLD    , IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_X  , N, IS_X, Y, N, N, N, N, M_XLR   , 0.U,N, N, N, Y, Y, CSR.N),
  SC_W    -> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XSC   , 0.U,N, N, N, Y, Y, CSR.N),
  SC_D    -> List(Y, N, X, uopAMO_AG, IQT_MEM, FU_MEM, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, Y, Y, N, N, M_XSC   , 0.U,N, N, N, Y, Y, CSR.N)
  )
}

/**
 * FP Decode constants
 */
object FDecode extends DecodeConstants
{
  val table: Array[(BitPat, List[BitPat])] = Array(
            //                                                                  frs3_en                        wakeup_delay
            //                                                                  |  imm sel                     |    bypassable (aka, known/fixed latency)
            //                                                                  |  |     uses_ldq              |    |  is_br
            //    is val inst?                                  rs1 regtype     |  |     |  uses_stq           |    |  |
            //    |  is fp inst?                                |       rs2 type|  |     |  |  is_amo          |    |  |
            //    |  |  is dst single-prec?                     |       |       |  |     |  |  |  is_fence     |    |  |
            //    |  |  |  micro-opcode                         |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall
            //    |  |  |  |           iq_type  func    dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
            //    |  |  |  |           |        unit    regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit
            //    |  |  |  |           |        |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd
  FLW     -> List(Y, Y, Y, uopLD     , IQT_MEM, FU_MEM, RT_FLT, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N),
  FLD     -> List(Y, Y, N, uopLD     , IQT_MEM, FU_MEM, RT_FLT, RT_FIX, RT_X  , N, IS_I, Y, N, N, N, N, M_XRD, 0.U, N, N, N, N, N, CSR.N),
  FSW     -> List(Y, Y, Y, uopSTA    , IQT_MFP,FU_F2IMEM,RT_X , RT_FIX, RT_FLT, N, IS_S, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N), // sort of a lie; broken into two micro-ops
  FSD     -> List(Y, Y, N, uopSTA    , IQT_MFP,FU_F2IMEM,RT_X , RT_FIX, RT_FLT, N, IS_S, N, Y, N, N, N, M_XWR, 0.U, N, N, N, N, N, CSR.N),

  FCLASS_S-> List(Y, Y, Y, uopFCLASS_S,IQT_FP , FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCLASS_D-> List(Y, Y, N, uopFCLASS_D,IQT_FP , FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  FMV_S_X -> List(Y, Y, Y, uopFMV_S_X, IQT_INT, FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMV_D_X -> List(Y, Y, N, uopFMV_D_X, IQT_INT, FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMV_X_S -> List(Y, Y, Y, uopFMV_X_S, IQT_FP , FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMV_X_D -> List(Y, Y, N, uopFMV_X_D, IQT_FP , FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  FSGNJ_S -> List(Y, Y, Y, uopFSGNJ_S, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FSGNJ_D -> List(Y, Y, N, uopFSGNJ_D, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FSGNJX_S-> List(Y, Y, Y, uopFSGNJ_S, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FSGNJX_D-> List(Y, Y, N, uopFSGNJ_D, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FSGNJN_S-> List(Y, Y, Y, uopFSGNJ_S, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FSGNJN_D-> List(Y, Y, N, uopFSGNJ_D, IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  // FP to FP
  FCVT_S_D-> List(Y, Y, Y, uopFCVT_S_D,IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_D_S-> List(Y, Y, N, uopFCVT_D_S,IQT_FP , FU_FPU, RT_FLT, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  // Int to FP
  FCVT_S_W-> List(Y, Y, Y, uopFCVT_S_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_S_WU->List(Y, Y, Y, uopFCVT_S_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_S_L-> List(Y, Y, Y, uopFCVT_S_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_S_LU->List(Y, Y, Y, uopFCVT_S_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  FCVT_D_W-> List(Y, Y, N, uopFCVT_D_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_D_WU->List(Y, Y, N, uopFCVT_D_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_D_L-> List(Y, Y, N, uopFCVT_D_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_D_LU->List(Y, Y, N, uopFCVT_D_X, IQT_INT,FU_I2F, RT_FLT, RT_FIX, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  // FP to Int
  FCVT_W_S-> List(Y, Y, Y, uopFCVT_X_S, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_WU_S->List(Y, Y, Y, uopFCVT_X_S, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_L_S-> List(Y, Y, Y, uopFCVT_X_S, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_LU_S->List(Y, Y, Y, uopFCVT_X_S, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  FCVT_W_D-> List(Y, Y, N, uopFCVT_X_D, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_WU_D->List(Y, Y, N, uopFCVT_X_D, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_L_D-> List(Y, Y, N, uopFCVT_X_D, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FCVT_LU_D->List(Y, Y, N, uopFCVT_X_D, IQT_FP, FU_F2I, RT_FIX, RT_FLT, RT_X  , N, IS_I, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  // "fp_single" is used for wb_data formatting (and debugging)
  FEQ_S    ->List(Y, Y, Y, uopCMPR_S , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FLT_S    ->List(Y, Y, Y, uopCMPR_S , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FLE_S    ->List(Y, Y, Y, uopCMPR_S , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  FEQ_D    ->List(Y, Y, N, uopCMPR_D , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FLT_D    ->List(Y, Y, N, uopCMPR_D , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FLE_D    ->List(Y, Y, N, uopCMPR_D , IQT_FP,  FU_F2I, RT_FIX, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  FMIN_S   ->List(Y, Y, Y,uopFMINMAX_S,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMAX_S   ->List(Y, Y, Y,uopFMINMAX_S,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMIN_D   ->List(Y, Y, N,uopFMINMAX_D,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMAX_D   ->List(Y, Y, N,uopFMINMAX_D,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  FADD_S   ->List(Y, Y, Y, uopFADD_S , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FSUB_S   ->List(Y, Y, Y, uopFSUB_S , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMUL_S   ->List(Y, Y, Y, uopFMUL_S , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FADD_D   ->List(Y, Y, N, uopFADD_D , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FSUB_D   ->List(Y, Y, N, uopFSUB_D , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMUL_D   ->List(Y, Y, N, uopFMUL_D , IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),

  FMADD_S  ->List(Y, Y, Y, uopFMADD_S, IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMSUB_S  ->List(Y, Y, Y, uopFMSUB_S, IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FNMADD_S ->List(Y, Y, Y, uopFNMADD_S,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FNMSUB_S ->List(Y, Y, Y, uopFNMSUB_S,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMADD_D  ->List(Y, Y, N, uopFMADD_D, IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FMSUB_D  ->List(Y, Y, N, uopFMSUB_D, IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FNMADD_D ->List(Y, Y, N, uopFNMADD_D,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FNMSUB_D ->List(Y, Y, N, uopFNMSUB_D,IQT_FP,  FU_FPU, RT_FLT, RT_FLT, RT_FLT, Y, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N)
  )
}

/**
 * FP Divide SquareRoot Constants
 */
object FDivSqrtDecode extends DecodeConstants
{
  val table: Array[(BitPat, List[BitPat])] = Array(
            //                                                                  frs3_en                        wakeup_delay
            //                                                                  |  imm sel                     |    bypassable (aka, known/fixed latency)
            //                                                                  |  |     uses_ldq              |    |  is_br
            //     is val inst?                                 rs1 regtype     |  |     |  uses_stq           |    |  |
            //     |  is fp inst?                               |       rs2 type|  |     |  |  is_amo          |    |  |
            //     |  |  is dst single-prec?                    |       |       |  |     |  |  |  is_fence     |    |  |
            //     |  |  |  micro-opcode                        |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall
            //     |  |  |  |           iq-type func    dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
            //     |  |  |  |           |       unit    regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit
            //     |  |  |  |           |       |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd
  FDIV_S    ->List(Y, Y, Y, uopFDIV_S , IQT_FP, FU_FDV, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FDIV_D    ->List(Y, Y, N, uopFDIV_D , IQT_FP, FU_FDV, RT_FLT, RT_FLT, RT_FLT, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FSQRT_S   ->List(Y, Y, Y, uopFSQRT_S, IQT_FP, FU_FDV, RT_FLT, RT_FLT, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
  FSQRT_D   ->List(Y, Y, N, uopFSQRT_D, IQT_FP, FU_FDV, RT_FLT, RT_FLT, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N)
  )
}
//scalastyle:on

/**
 * RoCC initial decode
 */
object RoCCDecode extends DecodeConstants
{
  // Note: We use FU_CSR since CSR instructions cannot co-execute with RoCC instructions
                       //                                                                   frs3_en                        wakeup_delay
                       //     is val inst?                                                  |  imm sel                     |    bypassable (aka, known/fixed latency)
                       //     |  is fp inst?                                                |  |     uses_ldq              |    |  is_br
                       //     |  |  is single-prec                          rs1 regtype     |  |     |  uses_stq           |    |  |
                       //     |  |  |                                       |       rs2 type|  |     |  |  is_amo          |    |  |
                       //     |  |  |  micro-code           func unit       |       |       |  |     |  |  |  is_fence     |    |  |
                       //     |  |  |  |           iq-type  |               |       |       |  |     |  |  |  |  is_fencei |    |  |  is breakpoint or ecall?
                       //     |  |  |  |           |        |       dst     |       |       |  |     |  |  |  |  |  mem    |    |  |  |  is unique? (clear pipeline for it)
                       //     |  |  |  |           |        |       regtype |       |       |  |     |  |  |  |  |  cmd    |    |  |  |  |  flush on commit
                       //     |  |  |  |           |        |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  csr cmd
                       //     |  |  |  |           |        |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |
  val table: Array[(BitPat, List[BitPat])] = Array(//       |       |       |       |       |  |     |  |  |  |  |  |      |    |  |  |  |  |  |
    CUSTOM0            ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM0_RS1        ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM0_RS1_RS2    ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM0_RD         ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM0_RD_RS1     ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM0_RD_RS1_RS2 ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM1            ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM1_RS1        ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM1_RS1_RS2    ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM1_RD         ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM1_RD_RS1     ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM1_RD_RS1_RS2 ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM2            ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM2_RS1        ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM2_RS1_RS2    ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM2_RD         ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM2_RD_RS1     ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM2_RD_RS1_RS2 ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM3            ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM3_RS1        ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM3_RS1_RS2    ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_X  , RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM3_RD         ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_X  , RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM3_RD_RS1     ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_X  , N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N),
    CUSTOM3_RD_RS1_RS2 ->List(Y, N, X, uopROCC   , IQT_INT, FU_CSR, RT_FIX, RT_FIX, RT_FIX, N, IS_X, N, N, N, N, N, M_X  , 0.U, N, N, N, N, N, CSR.N)
  )
}





/**
 * IO bundle for the Decode unit
 */
class DecodeUnitIo(implicit p: Parameters) extends BoomBundle
{
  val enq = new Bundle { val uop = Input(new MicroOp()) }
  //chw: ???????????????io????????????????????????????????????????????????4????????????
  val deq = new Bundle {  val uops = Output(Vec(4, new MicroOp()))
                          val uop_valids = Output(Vec(4, Bool()))
                          //num???????????????????????????
                          val num = Output(UInt(5.W))
                          //????????????????????????????????????????????????
                          val is_decoded = Output(Bool()) 
                          }

  // from CSRFile
  val status = Input(new freechips.rocketchip.rocket.MStatus())
  val csr_decode = Flipped(new freechips.rocketchip.rocket.CSRDecodeIO)
  val interrupt = Input(Bool())
  val interrupt_cause = Input(UInt(xLen.W))

  //chw: ???????????????io??????????????????????????????????????????????????????
  val is_unicore = Input(Bool())
}

//chw?????????Unicore??????????????????
abstract trait DecodeConstants_Unicore
  extends freechips.rocketchip.rocket.constants.ScalarOpConstants
  with freechips.rocketchip.rocket.constants.MemoryOpConstants
{
  val xpr64 = Y // TODO inform this from xLen
  val DC2 = BitPat.dontCare(2) // Makes the listing below more readable
  def decode_default: List[BitPat] =                                                       
          //     is val inst?                          uses_ldq                         rd regtype                                          wakeup_delay
          //     |  is fp inst?                        |  uses_stq                      |       rs1 regtype                                 |    bypassable
          //     |  |  is single-prec?                 |  |    is_sys_pc2epc            |       |       rs2 regtype                         |    |    mem_cmd
          //     |  |  |  micro-code                   |  |    |  flush_on_commit       |       |       |       rs3 regtype                 |    |    |    csr_cmd
          //     |  |  |  |         iq-type  func unit |  |    |  |  inst_unique        |       |       |       |        frs3_en            |    |    |    |      mem size
          //     |  |  |  |         |        |         |  |    |  |  |  is_br           |       |       |       |        |  wflag           |    |    |    |      |   shift way
          //     |  |  |  |         |        |         |  |    |  |  |  |  is_fence     |       |       |       |        |  |  rflag        |    |    |    |      |   |    
          //     |  |  |  |         |        |         |  |    |  |  |  |  |  is_fencei |       |       |       |        |  |  |  imm_sel   |    |    |    |      |   |    
          //     |  |  |  |         |        |         |  |    |  |  |  |  |  |  is_amo |       |       |       |        |  |  |  |         |    |    |    |      |   |    
          //     |  |  |  |         |        |         |  |    |  |  |  |  |  |  |      |       |       |       |        |  |  |  |         |    |    |    |      |   |
            List(N, N, X, uopX    , IQT_INT, FU_X   ,  X, X,   X, X, X, X, X, X, X,     RT_X  , DC2    ,DC2    ,DC2    , X, X, X, IS_X,     DC2, X,   M_X, CSR.X, MX, NO_SHIFT, RD2_X)
  val table: Array[(BitPat, List[BitPat])]
}

//chw?????????unicore???????????????????????????
class CtrlSigs_Unicore extends Bundle
{
  val legal           = Bool()
  val fp_val          = Bool()
  val fp_single       = Bool()

  val uopc            = UInt(UOPC_SZ.W)
  val iq_type         = UInt(IQT_SZ.W)
  val fu_code         = UInt(FUC_SZ.W)
  val uses_ldq        = Bool()
  val uses_stq        = Bool()

  val is_br           = Bool()
  val is_amo          = Bool()
  val is_fence        = Bool()
  val is_fencei       = Bool()
  val is_sys_pc2epc   = Bool()
  val inst_unique     = Bool()
  val flush_on_commit = Bool()

  val dst_type        = UInt(2.W)
  val rs1_type        = UInt(2.W)
  val rs2_type        = UInt(2.W)
  val rs3_type        = UInt(2.W) //imm5/fixed reg/float reg

  val frs3_en         = Bool()
  val wflag           = Bool()
  val rflag           = Bool()
  val imm_sel         = UInt(IS_X.getWidth.W)

  val wakeup_delay    = UInt(2.W)
  val bypassable      = Bool()
  
  val mem_cmd         = UInt(freechips.rocketchip.rocket.M_SZ.W)
  val csr_cmd         = UInt(freechips.rocketchip.rocket.CSR.SZ.W)

  val mem_size        = UInt(2.W)
  val shift           = UInt(3.W)
  val rd2_src         = UInt(2.W)

  def decode(inst: UInt, table: Iterable[(BitPat, List[BitPat])]) = {
    val decoder = freechips.rocketchip.rocket.DecodeLogic(inst, UDecode.decode_default, table)
    val sigs =
      Seq(legal, fp_val, fp_single, uopc, iq_type, fu_code, uses_ldq, uses_stq, 
          is_sys_pc2epc, flush_on_commit, inst_unique, is_br, is_fence, is_fencei, is_amo, 
          dst_type, rs1_type, rs2_type, rs3_type, frs3_en, wflag, rflag, imm_sel,
          wakeup_delay, bypassable,
          mem_cmd, csr_cmd, mem_size, shift, rd2_src)
      sigs zip decoder map {case(s,d) => s := d}
      this
  }
}



//chw?????????Unicore??????????????????
object UDecode extends DecodeConstants_Unicore
{
 //single-prec:??????
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
  ADDA_R_IMM   -> List(Y, N, X, uopADD  ,IQT_INT, FU_ALU , N, N, N, N, N, N, N, N, N,    RT_FIX, RT_FIX ,RT_FIX ,RT_IMM5, N, Y, N, IS_I,  1.U, Y,   M_X, CSR.N, MX, LG_RIGHT, RD2_X  ),  
  ADD_R_REG    -> List(Y, N, X, uopADD  ,IQT_INT, FU_ALU , N, N, N, N, N, N, N, N, N,    RT_FIX, RT_FIX ,RT_FIX ,RT_FIX , N, N, N, IS_I,  1.U, Y,   M_X, CSR.N, MX, LG_RIGHT, RD2_X  ),  
  ADDA_LR_REG  -> List(Y, N, X, uopADD  ,IQT_INT, FU_ALU , N, N, N, N, N, N, N, N, N,    RT_FIX, RT_FIX ,RT_FIX ,RT_FIX , N, Y, N, IS_I,  1.U, Y,   M_X, CSR.N, MX, LP_RIGHT, RD2_X  ),  
  ADDCA_L_IMM  -> List(Y, N, X, uopADD  ,IQT_INT, FU_ALU , N, N, N, N, N, N, N, N, N,    RT_FIX, RT_FIX ,RT_FIX ,RT_IMM5, N, Y, Y, IS_I,  1.U, Y,   M_X, CSR.N, MX, LG_LEFT , RD2_X  ),  
  ADDCA_R_REG  -> List(Y, N, X, uopADD  ,IQT_INT, FU_ALU , N, N, N, N, N, N, N, N, N,    RT_FIX, RT_FIX ,RT_FIX ,RT_FIX , N, Y, Y, IS_I,  1.U, Y,   M_X, CSR.N, MX, LG_RIGHT, RD2_X  ),  
  ADDIA_LR_IMM -> List(Y, N, X, uopADDI ,IQT_INT, FU_ALU , N, N, N, N, N, N, N, N, N,    RT_FIX, RT_FIX ,RT_X   ,RT_IMM5, N, Y, N, IS_I,  1.U, Y,   M_X, CSR.N, MX, LP_RIGHT, RD2_X  ), 
  ADDICA_LR_IMM-> List(Y, N, X, uopADDI ,IQT_INT, FU_ALU , N, N, N, N, N, N, N, N, N,    RT_FIX, RT_FIX ,RT_X   ,RT_IMM5, N, Y, Y, IS_I,  1.U, Y,   M_X, CSR.N, MX, LP_RIGHT, RD2_X  ),
  MULSL        -> List(Y, N, X, uopMUL  ,IQT_INT, FU_MUL , N, N, N, N, N, N, N, N, N,    RT_FIX, RT_FIX ,RT_FIX ,RT_X   , N, N, N, IS_I,  0.U, N,   M_X, CSR.N, MX, NO_SHIFT, RD2_RS3) 
  )
  //add.a: ?????????????????? wflag
  //addc.a: ????????????????????????????????????
  //??????????????????????????????????????????????????????????????????????????????????????????
}



/**
 * Decode unit that takes in a single instruction and generates a MicroOp.
 */
class DecodeUnit(implicit p: Parameters) extends BoomModule
  with freechips.rocketchip.rocket.constants.MemoryOpConstants
{
  val io = IO(new DecodeUnitIo)

  val uop = Wire(new MicroOp())
  val uop1 = Wire(new MicroOp())
  val uop2 = Wire(new MicroOp())
  val uop3 = Wire(new MicroOp())
  uop := io.enq.uop

  var decode_table = XDecode.table
  if (usingFPU) decode_table ++= FDecode.table
  if (usingFPU && usingFDivSqrt) decode_table ++= FDivSqrtDecode.table
  if (usingRoCC) decode_table ++= RoCCDecode.table
  decode_table ++= (if (xLen == 64) X64Decode.table else X32Decode.table)

  val inst = uop.inst
  val cs = Wire(new CtrlSigs()).decode(inst, decode_table)

  //chw: ??????unicore???????????????????????????????????????
  var decode_table_unicore = UDecode.table
  val cs_u = Wire(new CtrlSigs_Unicore()).decode(inst, decode_table_unicore)

  val unicoreMode = io.is_unicore && (!uop.switch_off)

  val cs_fp_val = Mux(unicoreMode, cs_u.fp_val, cs.fp_val)
  val cs_fp_single = Mux(unicoreMode, cs_u.fp_single, cs.fp_single)

  val cs_uopc = Mux(unicoreMode, cs_u.uopc, cs.uopc)
  val cs_iq_type = Mux(unicoreMode, cs_u.iq_type, cs.iq_type)
  val cs_fu_code = Mux(unicoreMode, cs_u.fu_code, cs.fu_code)
  val cs_uses_ldq = Mux(unicoreMode, cs_u.uses_ldq, cs.uses_ldq)
  val cs_uses_stq = Mux(unicoreMode, cs_u.uses_stq, cs.uses_stq)

  val cs_is_br = Mux(unicoreMode, cs_u.is_br, cs.is_br)
  val cs_is_amo = Mux(unicoreMode, cs_u.is_amo, cs.is_amo)
  val cs_is_fence = Mux(unicoreMode, cs_u.is_fence, cs.is_fence)
  val cs_is_fencei = Mux(unicoreMode, cs_u.is_fencei, cs.is_fencei)
  val cs_is_sys_pc2epc = Mux(unicoreMode, cs_u.is_sys_pc2epc, cs.is_sys_pc2epc)
  val cs_inst_unique = Mux(unicoreMode, cs_u.inst_unique, cs.inst_unique)
  val cs_flush_on_commit = Mux(unicoreMode, cs_u.flush_on_commit, cs.flush_on_commit)

  val cs_dst_type = Mux(unicoreMode, cs_u.dst_type, cs.dst_type)
  val cs_rs1_type = Mux(unicoreMode, cs_u.rs1_type, cs.rs1_type)
  val cs_rs2_type = Mux(unicoreMode, cs_u.rs2_type, cs.rs2_type)
  val cs_rs3_type = Mux(unicoreMode, cs_u.rs3_type, RT_X)

  val cs_frs3_en = Mux(unicoreMode, cs_u.frs3_en, cs.frs3_en)
  val cs_wflag = Mux(unicoreMode, cs_u.wflag, false.B)
  val cs_rflag = Mux(unicoreMode, cs_u.rflag, false.B)
  val cs_imm_sel = Mux(unicoreMode, cs_u.imm_sel, cs.imm_sel)
  val cs_bypassable = Mux(unicoreMode, cs_u.bypassable, cs.bypassable)
  val cs_csr_cmd = Mux(unicoreMode, cs_u.csr_cmd, cs.csr_cmd)
  val cs_mem_cmd = Mux(unicoreMode, cs_u.mem_cmd, cs.mem_cmd)
  val cs_rocc = Mux(unicoreMode, false.B, cs.rocc)


  // Exception Handling
  io.csr_decode.csr := inst(31,20)

  //chw???????????????????????????????????????????????????
  //???????????????riscv????????????????????????riscv???????????????????????????
  val csr_en = cs_csr_cmd.isOneOf(CSR.S, CSR.C, CSR.W)
  val csr_ren = cs_csr_cmd.isOneOf(CSR.S, CSR.C) && uop.lrs1 === 0.U
  val system_insn = cs_csr_cmd === CSR.I
  val sfence = cs_uopc === uopSFENCE
  val cs_legal = Mux(unicoreMode, cs_u.legal, cs.legal)

  val id_illegal_insn_riscv = !cs_legal ||
    cs_fp_val && io.csr_decode.fp_illegal || // TODO check for illegal rm mode: (io.fpu.illegal_rm)
    cs_rocc && io.csr_decode.rocc_illegal ||
    cs_is_amo && !io.status.isa('a'-'a')  ||
    (cs_fp_val && !cs_fp_single) && !io.status.isa('d'-'a') ||
    csr_en && (io.csr_decode.read_illegal || !csr_ren && io.csr_decode.write_illegal) ||
    ((sfence || system_insn) && io.csr_decode.system_illegal)
  val id_illegal_insn_unicore = !cs_legal
  val id_illegal_insn = Mux(unicoreMode, id_illegal_insn_unicore, id_illegal_insn_riscv)
  //check ok

  def checkExceptions(x: Seq[(Bool, UInt)]) =
    (x.map(_._1).reduce(_||_), PriorityMux(x))

  val (xcpt_valid, xcpt_cause) = checkExceptions(List(
    (io.interrupt && !io.enq.uop.is_sfb, io.interrupt_cause),  // Disallow interrupts while we are handling a SFB
    (uop.bp_debug_if,                    (CSR.debugTriggerCause).U),
    (uop.bp_xcpt_if,                     (Causes.breakpoint).U),
    (uop.xcpt_pf_if,                     (Causes.fetch_page_fault).U),
    (uop.xcpt_ae_if,                     (Causes.fetch_access).U),
    (id_illegal_insn,                    (Causes.illegal_instruction).U)))

  uop.exception := xcpt_valid
  uop.exc_cause := xcpt_cause

  //-------------------------------------------------------------
  //chw???????????????????????????????????? switch the cpu to unicore/riscv mode
  uop.switch := (cs.uopc === uopADD) && (inst(RD_MSB,RD_LSB) === 0.U && inst(RS1_MSB,RS1_LSB) === 0.U && inst(RS2_MSB,RS2_LSB) === 1.U)
  uop.switch_off := (cs.uopc === uopADD) && (inst(RD_MSB,RD_LSB) === 0.U && inst(RS1_MSB,RS1_LSB) === 0.U && inst(RS2_MSB,RS2_LSB) === 2.U)
  uop.is_unique  := Mux(uop.switch||uop.switch_off, true.B, cs.inst_unique)
  //chw: debug printf
  when(uop.switch){
    printf("decode uop switch is true:%x\n", inst)
  }

  when(uop.switch_off){
    printf("decode uop switch off is true: %x\n", inst)
  }
  val debug_cycles = freechips.rocketchip.util.WideCounter(32)
  when(io.is_unicore){
    //chw: debug printf
    printf("cycles: %d, decode inst: 0x%x, 0x%x, lrs1: %d, lrs2: %d\n", debug_cycles.value, uop.debug_pc, inst, uop.lrs1, uop.lrs2)
  }

  //-------------------------????????????------------------------------------
  uop.is_unicore      := io.is_unicore
  uop.is_br           := cs_is_br
  uop.is_jal          := (uop.uopc === uopJAL)
  uop.is_jalr         := (uop.uopc === uopJALR)
  uop.is_amo          := cs_is_amo
  uop.is_fence        := cs_is_fence
  uop.is_fencei       := cs_is_fencei
  uop.is_sys_pc2epc   := cs_is_sys_pc2epc
  uop.flush_on_commit := cs_flush_on_commit || (csr_en && !csr_ren && io.csr_decode.write_flush)

  //-------------------------????????????------------------------------------
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
  val mem_size_riscv = Mux(cs_mem_cmd.isOneOf(M_SFENCE, M_FLUSH_ALL), Cat(uop.lrs2 =/= 0.U, uop.lrs1 =/= 0.U), inst(13,12))
  uop.mem_size   := Mux(unicoreMode, cs_u.mem_size, mem_size_riscv)
  uop.mem_signed := Mux(unicoreMode, false.B, !inst(14))

  //---------------------------------------------------------------------------------------
  val di24_20 = Mux(cs_imm_sel === IS_B || cs_imm_sel === IS_S, inst(11,7), inst(24,20))
  val imm_packed_riscv = Cat(inst(31,25), di24_20, inst(19,12))
  
  val imm_packed_unicore = inst(19,0)
  // uop.imm_8bits  := Cat(0.U(4.W), inst(23,20))
  uop.imm_packed := Mux(unicoreMode, imm_packed_unicore, imm_packed_riscv)


  //---------------------------------------------------------------------------------
  uop.ldst_is_rs1 := !io.is_unicore && uop.is_sfb_shadow
  // SFB optimization
  when (!io.is_unicore && uop.is_sfb_shadow && cs_rs2_type === RT_X) {
    uop.lrs2_rtype  := RT_FIX
    uop.lrs2        := inst(RD_MSB,RD_LSB)
    uop.ldst_is_rs1 := false.B
  } .elsewhen (!io.is_unicore && uop.is_sfb_shadow && cs_uopc === uopADD && inst(RS1_MSB,RS1_LSB) === 0.U) {
    uop.uopc        := uopMOV
    uop.lrs1        := inst(RD_MSB, RD_LSB)
    uop.ldst_is_rs1 := true.B
  }
  when (uop.is_sfb_br && !io.is_unicore) {
    uop.fu_code := FU_JMP
  }  

  //-------------------------------------------------------------
  //chw: ?????????????????????????????????????????????????????????
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

  when(unicoreMode && (uop.uopc === uopMUL)){
    //chw: for new transform
    io.deq.is_decoded := false.B
  }
  .otherwise{
    io.deq.is_decoded := true.B
  }

  //when(unicoreMode){
  when(unicoreMode){
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

    //chw: debug printf
    when(io.is_unicore){
      printf("cycles: %d, decode inst: 0x%x, 0x%x, num: %d, valid: %d, %d, %d, %d\n", debug_cycles.value, uop.debug_pc, inst, uop.split_num, cs_sub0.valid, cs_sub1.valid, cs_sub2.valid, cs_sub3.valid)
    }

    io.deq.num := Mux(io.deq.is_decoded, uop.split_num, 0.U)
  }
  .otherwise{
    uop.split_num  := 1.U
    uop.self_index := 0.U

    io.deq.uop_valids(0) := true.B
    io.deq.uop_valids(1) := false.B
    io.deq.uop_valids(2) := false.B
    io.deq.uop_valids(3) := false.B

    io.deq.uops(0) := uop
    io.deq.uops(1) := uop
    io.deq.uops(2) := uop
    io.deq.uops(3) := uop

    io.deq.num := 1.U
  }

}

/**
 * Smaller Decode unit for the Frontend to decode different
 * branches.
 * Accepts EXPANDED RVC instructions
  */

class BranchDecodeSignals(implicit p: Parameters) extends BoomBundle
{
  val is_ret   = Bool()
  val is_call  = Bool()
  val target   = UInt(vaddrBitsExtended.W)
  val cfi_type = UInt(CFI_SZ.W)


  // Is this branch a short forwards jump?
  val sfb_offset = Valid(UInt(log2Ceil(icBlockBytes).W))
  // Is this instruction allowed to be inside a sfb?
  val shadowable = Bool()
}

class BranchDecode(implicit p: Parameters) extends BoomModule
{
  val io = IO(new Bundle {
    val inst    = Input(UInt(32.W))
    val pc      = Input(UInt(vaddrBitsExtended.W))
    val is_unicore = Input(Bool())
    //chw: BranchDecode io add is_unicore signal

    val out = Output(new BranchDecodeSignals)
  })

  val bpd_csignals =
    freechips.rocketchip.rocket.DecodeLogic(io.inst,
                  List[BitPat](N, N, N, N, X),
////                               is br?
////                               |  is jal?
////                               |  |  is jalr?
////                               |  |  |
////                               |  |  |  shadowable
////                               |  |  |  |  has_rs2
////                               |  |  |  |  |
            Array[(BitPat, List[BitPat])](
               JAL         -> List(N, Y, N, N, X),
               JALR        -> List(N, N, Y, N, X),
               BEQ         -> List(Y, N, N, N, X),
               BNE         -> List(Y, N, N, N, X),
               BGE         -> List(Y, N, N, N, X),
               BGEU        -> List(Y, N, N, N, X),
               BLT         -> List(Y, N, N, N, X),
               BLTU        -> List(Y, N, N, N, X),

               SLLI        -> List(N, N, N, Y, N),
               SRLI        -> List(N, N, N, Y, N),
               SRAI        -> List(N, N, N, Y, N),

               ADDIW       -> List(N, N, N, Y, N),
               SLLIW       -> List(N, N, N, Y, N),
               SRAIW       -> List(N, N, N, Y, N),
               SRLIW       -> List(N, N, N, Y, N),

               ADDW        -> List(N, N, N, Y, Y),
               SUBW        -> List(N, N, N, Y, Y),
               SLLW        -> List(N, N, N, Y, Y),
               SRAW        -> List(N, N, N, Y, Y),
               SRLW        -> List(N, N, N, Y, Y),

               LUI         -> List(N, N, N, Y, N),

               ADDI        -> List(N, N, N, Y, N),
               ANDI        -> List(N, N, N, Y, N),
               ORI         -> List(N, N, N, Y, N),
               XORI        -> List(N, N, N, Y, N),
               SLTI        -> List(N, N, N, Y, N),
               SLTIU       -> List(N, N, N, Y, N),

               SLL         -> List(N, N, N, Y, Y),
               ADD         -> List(N, N, N, Y, Y),
               SUB         -> List(N, N, N, Y, Y),
               SLT         -> List(N, N, N, Y, Y),
               SLTU        -> List(N, N, N, Y, Y),
               AND         -> List(N, N, N, Y, Y),
               OR          -> List(N, N, N, Y, Y),
               XOR         -> List(N, N, N, Y, Y),
               SRA         -> List(N, N, N, Y, Y),
               SRL         -> List(N, N, N, Y, Y)
            ))

  //chw: unicore???????????????????????????????????????????????????????????????????????????????????????????????????????????????
  val bpd_csignals_unicore =
    freechips.rocketchip.rocket.DecodeLogic(io.inst,
                  List[BitPat](N, N, N, N, N),
                      ////         is br?        
                      ////         |  is jal?   
                      ////         |  |   is jalr?
                      ////         |  |  |  is call?
                      ////         |  |  |  |  is ret?
                      ////         |  |  |  |  |
            Array[(BitPat, List[BitPat])](
               ADD_L_IMM   -> List(N, N, N, N, N),
               ADDA_R_IMM  -> List(N, N, N, N, N) //??????????????????
    ))
  

  val (cs_is_br_riscv: Bool) :: (cs_is_jal_riscv: Bool) :: (cs_is_jalr_riscv:Bool) :: (cs_is_shadowable:Bool) :: (cs_has_rs2) :: Nil = bpd_csignals
  val (cs_is_br_unicore: Bool) :: (cs_is_jal_unicore: Bool) :: (cs_is_jalr_unicore:Bool) :: (cs_is_call:Bool) :: (cs_is_ret:Bool) :: Nil = bpd_csignals_unicore
  val cs_is_br    = Mux(io.is_unicore, cs_is_br_unicore, cs_is_br_riscv)
  val cs_is_jal   = Mux(io.is_unicore, cs_is_jal_unicore, cs_is_jal_riscv)
  val cs_is_jalr  = Mux(io.is_unicore, cs_is_jalr_unicore, cs_is_jalr_riscv)
  val is_call_riscv = (cs_is_jal || cs_is_jalr) && GetRd(io.inst) === RA
  val is_ret_riscv = cs_is_jalr && GetRs1(io.inst) === BitPat("b00?01") && GetRd(io.inst) === X0
  val target_riscv = Mux(cs_is_br, ComputeBranchTarget(io.pc, io.inst, xLen), ComputeJALTarget(io.pc, io.inst, xLen))
  val target_unicore = ComputeBranchTarget_Unicore(io.pc, io.inst, xLen)


  io.out.is_call := Mux(io.is_unicore, cs_is_call, is_call_riscv) //cs_is_call/ret???unicore??????????????????
  io.out.is_ret  := Mux(io.is_unicore, cs_is_ret, is_ret_riscv)
  io.out.target := Mux(io.is_unicore, target_unicore, target_riscv)
  io.out.cfi_type := Mux(cs_is_jalr, CFI_JALR, Mux(cs_is_jal, CFI_JAL, Mux(cs_is_br, CFI_BR, CFI_X)))

  val br_offset = Cat(io.inst(7), io.inst(30,25), io.inst(11,8), 0.U(1.W))
  val sfb_offset_valid_riscv = cs_is_br && !io.inst(31) && br_offset =/= 0.U && (br_offset >> log2Ceil(icBlockBytes)) === 0.U
  val shadowable_riscv = cs_is_shadowable && ( 
      !cs_has_rs2 || (GetRs1(io.inst) === GetRd(io.inst)) ||
      (io.inst === ADD && GetRs1(io.inst) === X0))
  
  io.out.sfb_offset.valid := Mux(io.is_unicore, false.B, sfb_offset_valid_riscv)
  io.out.sfb_offset.bits  := br_offset
  io.out.shadowable := Mux(io.is_unicore, false.B, shadowable_riscv)
  //check 2 Chipyard
}

/**
 * Track the current "branch mask", and give out the branch mask to each micro-op in Decode
 * (each micro-op in the machine has a branch mask which says which branches it
 * is being speculated under).
 *
 * @param pl_width pipeline width for the processor
 */
class BranchMaskGenerationLogic(val pl_width: Int)(implicit p: Parameters) extends BoomModule
{
  val io = IO(new Bundle {
    // guess if the uop is a branch (we'll catch this later)
    val is_branch = Input(Vec(pl_width, Bool()))
    // lock in that it's actually a branch and will fire, so we update
    // the branch_masks.
    val will_fire = Input(Vec(pl_width, Bool()))

    // give out tag immediately (needed in rename)
    // mask can come later in the cycle
    val br_tag    = Output(Vec(pl_width, UInt(brTagSz.W)))
    val br_mask   = Output(Vec(pl_width, UInt(maxBrCount.W)))

     // tell decoders the branch mask has filled up, but on the granularity
     // of an individual micro-op (so some micro-ops can go through)
    val is_full   = Output(Vec(pl_width, Bool()))

    val brupdate         = Input(new BrUpdateInfo())
    val flush_pipeline = Input(Bool())

    val debug_branch_mask = Output(UInt(maxBrCount.W))
  })

  val branch_mask = RegInit(0.U(maxBrCount.W))

  //-------------------------------------------------------------
  // Give out the branch tag to each branch micro-op

  var allocate_mask = branch_mask
  val tag_masks = Wire(Vec(pl_width, UInt(maxBrCount.W)))

  for (w <- 0 until pl_width) {
    // TODO this is a loss of performance as we're blocking branches based on potentially fake branches
    io.is_full(w) := (allocate_mask === ~(0.U(maxBrCount.W))) && io.is_branch(w)

    // find br_tag and compute next br_mask
    val new_br_tag = Wire(UInt(brTagSz.W))
    new_br_tag := 0.U
    tag_masks(w) := 0.U

    for (i <- maxBrCount-1 to 0 by -1) {
      when (~allocate_mask(i)) {
        new_br_tag := i.U
        tag_masks(w) := (1.U << i.U)
      }
    }

    io.br_tag(w) := new_br_tag
    allocate_mask = Mux(io.is_branch(w), tag_masks(w) | allocate_mask, allocate_mask)
  }

  //-------------------------------------------------------------
  // Give out the branch mask to each micro-op
  // (kill off the bits that corresponded to branches that aren't going to fire)

  var curr_mask = branch_mask
  for (w <- 0 until pl_width) {
    io.br_mask(w) := GetNewBrMask(io.brupdate, curr_mask)
    curr_mask = Mux(io.will_fire(w), tag_masks(w) | curr_mask, curr_mask)
  }

  //-------------------------------------------------------------
  // Update the current branch_mask

  when (io.flush_pipeline) {
    branch_mask := 0.U
  } .otherwise {
    val mask = Mux(io.brupdate.b2.mispredict,
      io.brupdate.b2.uop.br_mask,
      ~(0.U(maxBrCount.W)))
    branch_mask := GetNewBrMask(io.brupdate, curr_mask) & mask
  }

  io.debug_branch_mask := branch_mask
}
