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

abstract trait DecodeConstants_SubInst
  extends freechips.rocketchip.rocket.constants.ScalarOpConstants
  with freechips.rocketchip.rocket.constants.MemoryOpConstants
{
  val xpr64 = Y // TODO inform this from xLen
  val DC2 = BitPat.dontCare(2) // Makes the listing below more readable
  def decode_default: List[BitPat] =                                                       
    //     is val inst?                          uses_ldq    rd type                                                wakeup_delay  
    //     |  is fp inst?                        |  uses_stq |       rs1 type                                       |    bypassable  
    //     |  |  is single-prec?                 |  |  is_br |       |       rs2 type rtemp1                        |    |  mem_cmd     
    //     |  |  |  micro-code                   |  |  |     |       |       |        |    rtemp2                   |    |  |    memsize      splitnum    
    //     |  |  |  |         iq-type  func unit |  |  |     |       |       |        |    |    wtemp wflag         |    |  |    |   shift way|    
    //     |  |  |  |         |        |         |  |  |     |       |       |        |    |    |     |  rflag      |    |  |    |   |        |    op1sel    
    //     |  |  |  |         |        |         |  |  |     |       |       |        |    |    |     |  |  imm_sel |    |  |    |   |        |    |     op2sel
    //     |  |  |  |         |        |         |  |  |     |       |       |        |    |    |     |  |  |       |    |  |    |   |        |    |     |     
    //     |  |  |  |         |        |         |  |  |     |       |       |        |    |    |     |  |  |       |    |  |    |   |        |    |     |   
    //     |  |  |  |         |        |         |  |  |     |       |       |        |    |    |     |  |  |       |    |  |    |   |        |    |     |  
      List(N, N, N, uopX    , IQT_INT, FU_X   ,  X, X, X,    RT_NO , RT_NO  ,RT_NO   ,0.U, 0.U, 0.U  ,X ,X, IS_X ,  0.U, N, M_X, MX, NO_SHIFT,1.U, OP_X, OP_X)              
  val table: Array[(BitPat, List[BitPat])]
  //Seq(valid, fp_val, fp_single, uopc, iq_type, fu_code, uses_ldq, uses_stq, is_br, 
  //        dst_type, rs1_type, rs2_type, ldst, lrs1, lrs2, wflag, rflag, imm_sel,
  //        wakeup_delay, bypassable, mem_cmd, mem_size, shift)
  //RT_X  RT_DST RT_TMP RT_RS1 RT_RS2 RT_RS3 RT_IMM
}

//opsel: OP_RS1, OP_RS2, OP_R0, OP_I5, OP_I9, OP_NP, OP_PC, OP_X 

/**
 * Decoded control signals
 */
class CtrlSigs_SubInst extends Bundle
{
  val valid           = Bool()
  val fp_val          = Bool()
  val fp_single       = Bool()

  val uopc            = UInt(UOPC_SZ.W)
  val iq_type         = UInt(IQT_SZ.W)
  val fu_code         = UInt(FUC_SZ.W)
  val uses_ldq        = Bool()
  val uses_stq        = Bool()

  val is_br           = Bool()
  
  val dst_type        = UInt(3.W)
  val rs1_type        = UInt(3.W)
  val rs2_type        = UInt(3.W)

  val rtemp1            = UInt(5.W)
  val rtemp2            = UInt(5.W)
  val wtemp             = UInt(5.W)

  val wflag           = Bool()
  val rflag           = Bool()
  val imm_sel         = UInt(IS_X.getWidth.W)

  val wakeup_delay    = UInt(2.W)
  val bypassable      = Bool()
  
  val mem_cmd         = UInt(freechips.rocketchip.rocket.M_SZ.W)

  val mem_size        = UInt(2.W)
  val shift           = UInt(3.W)

  val split_num       = UInt(microIdxSz.W)

  val op1_sel         = UInt(3.W)
  val op2_sel         = UInt(3.W)

  def decode(inst: UInt, table: Iterable[(BitPat, List[BitPat])]) = {
    val decoder = freechips.rocketchip.rocket.DecodeLogic(inst, SubDecode1.decode_default, table)
    val sigs =
      Seq(valid, fp_val, fp_single, uopc, iq_type, fu_code, uses_ldq, uses_stq, is_br, 
          dst_type, rs1_type, rs2_type, rtemp1, rtemp2, wtemp, wflag, rflag, imm_sel,
          wakeup_delay, bypassable, mem_cmd, mem_size, shift, split_num, bop1_sel, op2_sel)
      sigs zip decoder map {case(s,d) => s := d}
      this
  }
}


//opsel: OP_RS1, OP_RS2, OP_R0, OP_I5, OP_I9, OP_NP, OP_PC, OP_X 
//opsel用于在执行阶段，选择执行得两个操作数, 并且两个源操作数都是可以是这些类型中的值
//  - rs1和rs2为选择req中的io.req.bits.rs1_data和io.req.bits.rs2_data
//  - r0表示源操作数为0
//  - i5和i9表示采用的是五位还是九位立即数作为源操作数
//  - NP：next pc, 
//  - PC: pc

//rd/rs type: RT_X  RT_DST RT_TMP RT_RS1 RT_RS2 RT_RS3 RT_IMM
//type：用于决定当前使用的寄存器的具体值是多少， rs1, rs2, rd均可以是这些类型（rd不能是IMM）
//  - RT_DST, RS1, RS2, RS3：表示当前寄存器使用的是指令中哪一个位域的值
//  - RT_TMP: 表示当前寄存器采用的是硬件内部的Temp寄存器，具体寄存器的值由其它rtemp1/2, wtemp译码信号给出
//  - RT_IMM: 表示该位域不是一个寄存器，而是一个五位立即数
//chw unicore decode table
object SubDecode1 extends DecodeConstants_SubInst
{
 //single-prec:单精
 ////RT_X  RT_DST RT_TMP RT_RS1 RT_RS2 RT_RS3 RT_IMM
  val table: Array[(BitPat, List[BitPat])] = Array(
                  //     is val inst?                         uses_ldq                                                     wakeup_delay  
                  //     |  is fp inst?                       |  uses_stq      rs1 type        rtemp1                      |    bypassable  
                  //     |  |  is single-prec?                |  |  is_br      |       rs2 type|                           |    |  mem_cmd     
                  //     |  |  |  micro-code                  |  |  |  rd type |       |       |    rtemp2                 |    |  |    memsize      splitnum    
                  //     |  |  |  |         iq-type  func unit|  |  |  |       |       |       |    |    wtemp wflag       |    |  |    |   shift way|    
                  //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  rflag    |    |  |    |   |        |    op1sel 
                  //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  | imm_sel|    |  |    |   |        |    |       op2sel 
                  //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  |  |     |    |  |    |   |        |    |       |    
                  //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  |  |     |    |  |    |   |        |    |       |        
                  //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  |  |     |    |  |    |   |        |    |       |      
    ADD_L_IMM    -> List(Y, N, N, uopSHIFT, IQT_INT, FU_ALU , N, N, N, RT_TMP, RT_RS2 ,RT_IMM, 0.U, 0.U, 1.U , N, N, IS_I, 1.U, Y, M_X, MX, LG_LEFT, 2.U, OP_RS1, OP_I5), 
    ADD_R_REG    -> List(Y, N, N, uopSHIFT, IQT_INT, FU_ALU , N, N, N, RT_TMP, RT_RS2 ,RT_RS3, 0.U, 0.U, 1.U , N, N, IS_I, 1.U, Y, M_X, MX, LG_RIGHT,2.U, OP_RS1, OP_RS2), 
    ADDA_R_IMM   -> List(Y, N, N, uopSHIFT, IQT_INT, FU_ALU , N, N, N, RT_TMP, RT_RS2 ,RT_IMM, 0.U, 0.U, 1.U , N, N, IS_I, 1.U, Y, M_X, MX, LG_RIGHT,2.U, OP_RS1, OP_I5), 
    ADDA_LR_REG  -> List(Y, N, N, uopSHIFT, IQT_INT, FU_ALU , N, N, N, RT_TMP, RT_RS2 ,RT_RS3, 0.U, 0.U, 1.U , N, N, IS_I, 1.U, Y, M_X, MX, LP_RIGHT,2.U, OP_RS1, OP_RS2), 
    ADDCA_L_IMM  -> List(Y, N, N, uopSHIFT, IQT_INT, FU_ALU , N, N, N, RT_TMP, RT_RS2 ,RT_IMM, 0.U, 0.U, 1.U , N, N, IS_I, 1.U, Y, M_X, MX, LG_LEFT, 2.U, OP_RS1, OP_I5),
    ADDIA_LR_IMM -> List(Y, N, N, uopSHIFT, IQT_INT, FU_ALU , N, N, N, RT_TMP, RT_IMM ,RT_IMM, 0.U, 0.U, 1.U , N, N, IS_I, 1.U, Y, M_X, MX, LP_RIGHT,2.U, OP_I9 , OP_I5),
    ADDICA_LR_IMM-> List(Y, N, N, uopSHIFT, IQT_INT, FU_ALU , N, N, N, RT_TMP, RT_IMM ,RT_IMM, 0.U, 0.U, 1.U , N, N, IS_I, 1.U, Y, M_X, MX, LP_RIGHT,2.U, OP_I9 , OP_I5),
    MULSL        -> List(Y, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_TMP, RT_RS1 ,RT_RS2, 0.U, 0.U, 1.U , N, N, IS_I, 1.U, Y, M_X, MX, NO_SHIFT,2.U, OP_RS1, OP_RS2)
  )  
}

object SubDecode2 extends DecodeConstants_SubInst
{
 //single-prec:单精
 ////RT_X  RT_DST RT_TMP RT_RS1 RT_RS2 RT_RS3 RT_IMM
  val table: Array[(BitPat, List[BitPat])] = Array(
                //     is val inst?                         uses_ldq                                                     wakeup_delay  
                //     |  is fp inst?                       |  uses_stq      rs1 type        rtemp1                      |    bypassable  
                //     |  |  is single-prec?                |  |  is_br      |       rs2 type|                           |    |  mem_cmd     
                //     |  |  |  micro-code                  |  |  |  rd type |       |       |    rtemp2                 |    |  |    memsize          
                //     |  |  |  |         iq-type  func unit|  |  |  |       |       |       |    |    wtemp wflag       |    |  |    |   shift way    
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  rflag    |    |  |    |   |         splitnum        
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  | imm_sel|    |  |    |   |         |        
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  |  |     |    |  |    |   |         |    op1sel          
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  |  |     |    |  |    |   |         |    |       op2sel         
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  |  |     |    |  |    |   |         |    |       |      
  ADD_L_IMM    -> List(Y, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_DST, RT_RS1 ,RT_TMP, 0.U, 1.U, 0.U , N, N, IS_I, 1.U, Y, M_X, MX, NO_SHIFT, 0.U, OP_RS1, OP_RS2), 
  ADD_R_REG    -> List(Y, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_DST, RT_RS1 ,RT_TMP, 0.U, 1.U, 0.U , N, N, IS_I, 1.U, Y, M_X, MX, NO_SHIFT, 0.U, OP_RS1, OP_RS2), 
  ADDA_R_IMM   -> List(Y, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_DST, RT_RS1 ,RT_TMP, 0.U, 1.U, 0.U , Y, N, IS_I, 1.U, Y, M_X, MX, NO_SHIFT, 0.U, OP_RS1, OP_RS2), 
  ADDA_LR_REG  -> List(Y, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_DST, RT_RS1 ,RT_TMP, 0.U, 1.U, 0.U , Y, N, IS_I, 1.U, Y, M_X, MX, NO_SHIFT, 0.U, OP_RS1, OP_RS2), 
  ADDIA_LR_IMM -> List(Y, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_DST, RT_RS1 ,RT_TMP, 0.U, 1.U, 0.U , Y, N, IS_I, 1.U, Y, M_X, MX, NO_SHIFT, 0.U, OP_RS1, OP_RS2),
  ADDCA_L_IMM  -> List(Y, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_DST, RT_RS1 ,RT_TMP, 0.U, 1.U, 0.U , Y, Y, IS_I, 1.U, Y, M_X, MX, NO_SHIFT, 0.U, OP_RS1, OP_RS2),
  ADDICA_LR_IMM-> List(Y, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_DST, RT_RS1 ,RT_TMP, 0.U, 1.U, 0.U , Y, Y, IS_I, 1.U, Y, M_X, MX, NO_SHIFT, 0.U, OP_RS1, OP_RS2),
  MULSL        -> List(Y, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_TMP, RT_RS1 ,RT_TMP, 0.U, 1.U, 1.U , N, N, IS_I, 1.U, Y, M_X, MX, NO_SHIFT, 0.U, OP_RS1, OP_RS2)
  // MULSL        -> List(Y, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_DST, RT_RS1 ,RT_TMP, 0.U, 1.U, 0.U , N, N, IS_I, 1.U, Y, M_X, MX, NO_SHIFT, 0.U, 0.U, OP_RS1, OP_RS2)
  )  
}


object SubDecode3 extends DecodeConstants_SubInst
{
 //single-prec:单精
 ////RT_X  RT_DST RT_TMP RT_RS1 RT_RS2 RT_RS3 RT_IMM
  val table: Array[(BitPat, List[BitPat])] = Array(
                //     is val inst?                         uses_ldq                                                     wakeup_delay  
                //     |  is fp inst?                       |  uses_stq      rs1 type        rtemp1                      |    bypassable  
                //     |  |  is single-prec?                |  |  is_br      |       rs2 type|                           |    |  mem_cmd     
                //     |  |  |  micro-code                  |  |  |  rd type |       |       |    rtemp2                 |    |  |    memsize          
                //     |  |  |  |         iq-type  func unit|  |  |  |       |       |       |    |    wtemp wflag       |    |  |    |   shift way    
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  rflag    |    |  |    |   |         splitnum        
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  | imm_sel|    |  |    |   |         |       
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  |  |     |    |  |    |   |         |    op1sel          
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  |  |     |    |  |    |   |         |    |       op2sel         
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  |  |     |    |  |    |   |         |    |       |      
  ADD_L_IMM    -> List(N, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_DST, RT_RS1 ,RT_TMP, 0.U, 1.U, 0.U , N, N, IS_I, 1.U, Y, M_X, MX, NO_SHIFT, 0.U, OP_RS1, OP_RS2), 
  ADDCA_L_IMM  -> List(N, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_DST, RT_RS1 ,RT_TMP, 0.U, 1.U, 0.U , Y, Y, IS_I, 1.U, Y, M_X, MX, NO_SHIFT, 0.U, OP_RS1, OP_RS2),
  MULSL        -> List(Y, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_TMP, RT_RS1 ,RT_TMP, 0.U, 1.U, 1.U , N, N, IS_I, 1.U, Y, M_X, MX, NO_SHIFT, 0.U, OP_RS1, OP_RS2)
  // MULSL        -> List(N, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_TMP, RT_RS1 ,RT_TMP, 0.U, 1.U, 1.U , N, N, IS_I, 1.U, Y, M_X, MX, NO_SHIFT, 0.U, 0.U, OP_RS1, OP_RS2)

  )   
}

object SubDecode4 extends DecodeConstants_SubInst
{
 //single-prec:单精
 ////RT_X  RT_DST RT_TMP RT_RS1 RT_RS2 RT_RS3 RT_IMM
  val table: Array[(BitPat, List[BitPat])] = Array(
                //     is val inst?                         uses_ldq                                                     wakeup_delay  
                //     |  is fp inst?                       |  uses_stq      rs1 type        rtemp1                      |    bypassable  
                //     |  |  is single-prec?                |  |  is_br      |       rs2 type|                           |    |  mem_cmd     
                //     |  |  |  micro-code                  |  |  |  rd type |       |       |    rtemp2                 |    |  |    memsize          
                //     |  |  |  |         iq-type  func unit|  |  |  |       |       |       |    |    wtemp wflag       |    |  |    |   shift way    
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  rflag    |    |  |    |   |         splitnum        
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  | imm_sel|    |  |    |   |         |     
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  |  |     |    |  |    |   |         |    op1sel          
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  |  |     |    |  |    |   |         |    |       op2sel         
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  |  |     |    |  |    |   |         |    |       |      
  ADD_L_IMM    -> List(N, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_DST, RT_RS1 ,RT_TMP, 0.U, 1.U, 0.U , N, N, IS_I, 1.U, Y, M_X, MX, NO_SHIFT, 0.U, OP_RS1, OP_RS2), 
  ADDCA_L_IMM  -> List(N, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_DST, RT_RS1 ,RT_TMP, 0.U, 1.U, 0.U , Y, Y, IS_I, 1.U, Y, M_X, MX, NO_SHIFT, 0.U, OP_RS1, OP_RS2),
  MULSL        -> List(Y, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_DST, RT_RS1 ,RT_TMP, 0.U, 1.U, 0.U , N, N, IS_I, 1.U, Y, M_X, MX, NO_SHIFT, 0.U, OP_RS1, OP_RS2)
  // MULSL        -> List(N, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_DST, RT_RS1 ,RT_TMP, 0.U, 1.U, 0.U , N, N, IS_I, 1.U, Y, M_X, MX, NO_SHIFT, 0.U, 0.U, OP_RS1, OP_RS2)

  ) 
}

object SubDecode5 extends DecodeConstants_SubInst
{
 //single-prec:单精
 ////RT_X  RT_DST RT_TMP RT_RS1 RT_RS2 RT_RS3 RT_IMM
  val table: Array[(BitPat, List[BitPat])] = Array(
                //     is val inst?                         uses_ldq                                                     wakeup_delay  
                //     |  is fp inst?                       |  uses_stq      rs1 type        rtemp1                      |    bypassable  
                //     |  |  is single-prec?                |  |  is_br      |       rs2 type|                           |    |  mem_cmd     
                //     |  |  |  micro-code                  |  |  |  rd type |       |       |    rtemp2                 |    |  |    memsize          
                //     |  |  |  |         iq-type  func unit|  |  |  |       |       |       |    |    wtemp wflag       |    |  |    |   shift way    
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  rflag    |    |  |    |   |         splitnum        
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  | imm_sel|    |  |    |   |         |    
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  |  |     |    |  |    |   |         |    op1sel          
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  |  |     |    |  |    |   |         |    |       op2sel         
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  |  |     |    |  |    |   |         |    |       |      
  MULSL        -> List(Y, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_TMP, RT_RS1 ,RT_TMP, 0.U, 1.U, 1.U , N, N, IS_I, 1.U, Y, M_X, MX, NO_SHIFT, 0.U, OP_RS1, OP_RS2)
  // MULSL        -> List(N, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_DST, RT_RS1 ,RT_TMP, 0.U, 1.U, 0.U , N, N, IS_I, 1.U, Y, M_X, MX, NO_SHIFT, 0.U, 0.U, OP_RS1, OP_RS2)

  ) 
}

object SubDecode6 extends DecodeConstants_SubInst
{
 //single-prec:单精
 ////RT_X  RT_DST RT_TMP RT_RS1 RT_RS2 RT_RS3 RT_IMM
  val table: Array[(BitPat, List[BitPat])] = Array(
                //     is val inst?                         uses_ldq                                                     wakeup_delay  
                //     |  is fp inst?                       |  uses_stq      rs1 type        rtemp1                      |    bypassable  
                //     |  |  is single-prec?                |  |  is_br      |       rs2 type|                           |    |  mem_cmd     
                //     |  |  |  micro-code                  |  |  |  rd type |       |       |    rtemp2                 |    |  |    memsize          
                //     |  |  |  |         iq-type  func unit|  |  |  |       |       |       |    |    wtemp wflag       |    |  |    |   shift way    
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  rflag    |    |  |    |   |         splitnum        
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  | imm_sel|    |  |    |   |         |      
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  |  |     |    |  |    |   |         |    op1sel          
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  |  |     |    |  |    |   |         |    |       op2sel         
                //     |  |  |  |         |        |        |  |  |  |       |       |       |    |    |     |  |  |     |    |  |    |   |         |    |       |      
  MULSL        -> List(Y, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_DST, RT_RS1 ,RT_TMP, 0.U, 1.U, 0.U , N, N, IS_I, 1.U, Y, M_X, MX, NO_SHIFT, 0.U, OP_RS1, OP_RS2)
  // MULSL        -> List(N, N, N, uopADD  , IQT_INT, FU_ALU , N, N, N, RT_DST, RT_RS1 ,RT_TMP, 0.U, 1.U, 0.U , N, N, IS_I, 1.U, Y, M_X, MX, NO_SHIFT, 0.U, 0.U, OP_RS1, OP_RS2)

  ) 
}

/*

add r7,r7,r6<<#1
7+6 = 13
add r7,r7,r6<<#1
13+6 = 19
add r7,r7,r6<<#1
19+6 = 25

mulsl r7,r7,r7,r6
25+3 = 28
28+25 = 53
53+25 = 78
78+25 = 103
mulsl r7,r7,r7,r6
103*4+3 = 415
mulsl r7,r7,r7,r6
415*4+3 = 1663
mulsl r7,r7,r7,r6
1663*4+3 = 6655 
*/


class SubDecodeUnit(implicit p: Parameters) extends BoomModule
  with freechips.rocketchip.rocket.constants.MemoryOpConstants
{
    val io = IO(new BoomBundle()(p) {
        val rawuop  = Input(new MicroOp())
        val cs_sub  = Input(new CtrlSigs_SubInst())

        val subuop  = Output(new MicroOp())
    })

    val uop = Wire(new MicroOp())
    uop := io.rawuop

    uop.fp_val      := io.cs_sub.fp_val
    uop.fp_single   := io.cs_sub.fp_single
    uop.uopc        := io.cs_sub.uopc
    uop.iq_type     := io.cs_sub.iq_type
    uop.fu_code     := io.cs_sub.fu_code
    uop.uses_ldq    := io.cs_sub.uses_ldq
    uop.uses_stq    := io.cs_sub.uses_stq
    uop.is_br       := io.cs_sub.is_br
    uop.wflag       := io.cs_sub.wflag
    uop.rflag       := io.cs_sub.rflag
    uop.bypassable  := io.cs_sub.bypassable
    uop.mem_cmd     := io.cs_sub.mem_cmd
    uop.mem_size    := io.cs_sub.mem_size
    uop.shift       := io.cs_sub.shift
    uop.is_jal      := (io.cs_sub.uopc === uopJAL)
    uop.is_jalr     := (io.cs_sub.uopc === uopJALR)
    uop.lrs3_rtype  := RT_X //not need for rs3
    uop.op1_sel     := io.cs_sub.op1_sel 
    uop.op2_sel     := io.cs_sub.op2_sel 

    //dst type
    when(io.cs_sub.dst_type === RT_DST){
      uop.ldst_val   := !(uop.inst(RD_MSB_UNICORE,RD_LSB_UNICORE) === 31.U)
      uop.dst_rtype  := RT_FIX
      uop.ldst       := uop.inst(RD_MSB_UNICORE,RD_LSB_UNICORE)
    }
    .elsewhen(io.cs_sub.dst_type === RT_RS1){
      uop.ldst_val   := !(uop.inst(RS1_MSB_UNICORE,RS1_LSB_UNICORE) === 31.U)
      uop.dst_rtype  := RT_FIX
      uop.ldst       := uop.inst(RS1_MSB_UNICORE,RS1_LSB_UNICORE)
    }
    .elsewhen(io.cs_sub.dst_type === RT_RS2){
      uop.ldst_val   := !(uop.inst(RS2_MSB_UNICORE,RS2_LSB_UNICORE) === 31.U)
      uop.dst_rtype  := RT_FIX
      uop.ldst       := uop.inst(RS2_MSB_UNICORE,RS2_LSB_UNICORE)
    }
    .elsewhen(io.cs_sub.dst_type === RT_RS3){
      uop.ldst_val   := !(uop.inst(RS3_MSB_UNICORE,RS3_LSB_UNICORE) === 31.U)
      uop.dst_rtype  := RT_FIX
      uop.ldst       := uop.inst(RS3_MSB_UNICORE,RS3_LSB_UNICORE)
    }
    .elsewhen(io.cs_sub.dst_type === RT_TMP){
      uop.ldst_val   := true.B
      uop.dst_rtype  := RT_FIX
      uop.ldst       := Cat(1.U(1.W), io.cs_sub.wtemp)
      // printf("test dst: %d %d %d %d\n", io.cs_sub.wtemp, uop.ldst, Cat(1.U(1.W), io.cs_sub.wtemp), lregSz.U)
    }
    .otherwise{
      uop.ldst_val   := false.B
      uop.dst_rtype  := RT_X
    }


    //rs1 type
    when(io.cs_sub.rs1_type === RT_DST){
      uop.lrs1_rtype    := RT_FIX
      uop.lrs1          := uop.inst(RD_MSB_UNICORE,RD_LSB_UNICORE)
    }
    .elsewhen(io.cs_sub.rs1_type === RT_RS1){
      uop.lrs1_rtype    := RT_FIX
      uop.lrs1          := uop.inst(RS1_MSB_UNICORE,RS1_LSB_UNICORE)
    }
    .elsewhen(io.cs_sub.rs1_type === RT_RS2){
      uop.lrs1_rtype    := RT_FIX
      uop.lrs1          := uop.inst(RS2_MSB_UNICORE,RS2_LSB_UNICORE)
    }
    .elsewhen(io.cs_sub.rs1_type === RT_RS3){
      uop.lrs1_rtype    := RT_FIX
      uop.lrs1          := uop.inst(RS3_MSB_UNICORE,RS3_LSB_UNICORE)
    }
    .elsewhen(io.cs_sub.rs1_type === RT_TMP){
      uop.lrs1_rtype    := RT_FIX
      uop.lrs1          := Cat(1.U(1.W), io.cs_sub.rtemp1)
      // printf("test rs1: %d %d %d\n", io.cs_sub.rtemp1, uop.lrs1, Cat(1.U(1.W), io.cs_sub.rtemp1))
    }
    .elsewhen(io.cs_sub.rs1_type === RT_IMM){
      uop.lrs1_rtype    := RT_IMM5      //和RT_PAS重叠，需要特殊注意
      uop.lrs1          := uop.inst(RS3_MSB_UNICORE,RS3_LSB_UNICORE)
    }
    .otherwise{
      uop.lrs1_rtype   := RT_X
    }

    //rs2 type
    when(io.cs_sub.rs2_type === RT_DST){
      uop.lrs2_rtype    := RT_FIX
      uop.lrs2          := uop.inst(RD_MSB_UNICORE,RD_LSB_UNICORE)
    }
    .elsewhen(io.cs_sub.rs2_type === RT_RS1){
      uop.lrs2_rtype    := RT_FIX
      uop.lrs2          := uop.inst(RS1_MSB_UNICORE,RS1_LSB_UNICORE)
    }
    .elsewhen(io.cs_sub.rs2_type === RT_RS2){
      uop.lrs2_rtype    := RT_FIX
      uop.lrs2          := uop.inst(RS2_MSB_UNICORE,RS2_LSB_UNICORE)
    }
    .elsewhen(io.cs_sub.rs2_type === RT_RS3){
      uop.lrs2_rtype    := RT_FIX
      uop.lrs2          := uop.inst(RS3_MSB_UNICORE,RS3_LSB_UNICORE)
    }
    .elsewhen(io.cs_sub.rs2_type === RT_TMP){
      uop.lrs2_rtype    := RT_FIX
      uop.lrs2          := Cat(1.U(1.W), io.cs_sub.rtemp2)
      // printf("test rs2: %d %d %d\n", io.cs_sub.rtemp2, uop.lrs2, Cat(1.U(1.W), io.cs_sub.rtemp1))
    }
    .elsewhen(io.cs_sub.rs2_type === RT_IMM){
      uop.lrs2_rtype    := RT_IMM5      //和RT_PAS重叠，需要特殊注意
      uop.lrs2          := uop.inst(RS3_MSB_UNICORE,RS3_LSB_UNICORE)
    }
    .otherwise{
      uop.lrs2_rtype   := RT_X
    }

    uop.lrs3_rtype   := RT_X
    io.subuop := uop
}