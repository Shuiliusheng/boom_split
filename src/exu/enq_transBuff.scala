///// chw  ////


package boom.exu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.{Parameters}
import freechips.rocketchip.rocket.{MStatus, BP, BreakpointUnit}

import boom.common._
import boom.util.{BoolToChar, MaskUpper}
import boom.util._


class EnqTranBuff(implicit p: Parameters) extends BoomModule
  with HasBoomCoreParameters
{
    val io = IO(new BoomBundle {
        val enq = Input(Vec(enqTranBuff_entries/tranBuff_enq_width, new DecodeUops(tranBuff_enq_width)))
        val set_num = Input(UInt(3.W))
        val enq_valid = Input(Bool())
        val enq_ready = Output(Bool())


        val deq = Output(new TransResp())
        val deq_valid = Output(Bool())
        val deq_ready = Input(Bool())
        
        val clear = Input(Bool())

        val isUnicoreMode = Input(Bool())
    })

    val trans_buffer  = Module(new TransBuffer)

    //连接enqTranBuffer的输入和transbuffer的输入
    trans_buffer.io.clear := io.clear
    trans_buffer.io.isUnicoreMode := io.isUnicoreMode
    trans_buffer.io.deq.ready := io.deq_ready
    
    //连接deq的输出
    io.deq_valid := trans_buffer.io.deq.valid
    io.deq := trans_buffer.io.deq.bits


    /////////// input for transform buffer //////////////////////////////
    val enq_buffer = Reg(Vec(enqTranBuff_entries/tranBuff_enq_width, new DecodeUops(tranBuff_enq_width)))
    val enq_valid = Reg(Bool())

    val set_idx = RegInit(0.U(3.W))
    val set_num = RegInit(0.U(3.W))

    val isBound = Wire(Bool())
    //case1: 如果当前io.set_num和set_idx都为0，则已经到了边界，不论io.enq_valid是否有效
    //case2: 如果当前enq_valid有效，并且set_idx已经和set_num相同，则已经到了边界
    // isBound := (io.enq_valid && io.set_num === 0.U && set_idx === 0.U) || (enq_valid && set_idx === set_num)
    // isBound := (io.set_num === 0.U && set_idx === 0.U) || (enq_valid && set_idx === set_num)
    isBound := Mux(io.enq_valid && set_idx === 0.U && trans_buffer.io.enq.ready, (io.set_num === 0.U && set_idx === 0.U), (enq_valid && set_idx === set_num && trans_buffer.io.enq.ready))
    

    //如果上周期到了边界，并且上周期还可以进入trans_buffer，则意味着这周期之前的所有微指令都已经进入transBuffer
    //同时，如果本周期trans_buffer的ready信号仍旧有效，则tran_enq_buffer可以进入就绪状态，可以进入新的指令
    //问题：如果set_num为0，并且输入无效，此时本周期只要transbuffer.enq就绪即可，但是此表达式仍旧需要要求上周期trans_buffer.io.enq.ready有效
    // io.enq_ready := Mux(io.clear, true.B, RegNext(trans_buffer.io.enq.ready && isBound) && trans_buffer.io.enq.ready)

    //记录本周期是否已经完成所有指令进入transBuffer中
    //case1: 当set_idx和io.set_num均为0，即仅输入了一组，如果此时io.enq_valid无效或者是tranbuffer.enq就绪，则本周期完成工作
    //case2: 当不合符case1，如果此时enq_valid有效，则仅有当set_idx到达set_num，并且tranbuffer.enq就绪，则本周期完成哦工作；
    //case3: 当不符合case1，并且enq_valid无效，则意味本周期没有任务，即也完成了工作
    val last_cycle_over = Wire(Bool())
    // last_cycle_over := Mux((io.set_num === 0.U && set_idx === 0.U), !io.enq_valid || trans_buffer.io.enq.ready, 
                        // Mux(io.enq_valid && set_idx === 0.U && trans_buffer.io.enq.ready, false.B, 
                        // Mux(enq_valid, (set_idx === set_num) && trans_buffer.io.enq.ready, true.B)))

    last_cycle_over := Mux(io.enq_valid && set_idx === 0.U && trans_buffer.io.enq.ready, io.set_num === 0.U,  
                        Mux(enq_valid, (set_idx === set_num) && trans_buffer.io.enq.ready, true.B))   

    io.enq_ready := io.clear || (RegNext(last_cycle_over) && trans_buffer.io.enq.ready)

    //如果trans_buffer处于enq的就绪状态，则更新set_idx
    // when(trans_buffer.io.enq.ready){
    //     when(isBound){//如果当前处于边界，则以为着下周期
    //         set_idx := 0.U
    //     }
    //     .otherwise{
    //         //如果第一个周期有指令有效或者后面的周期也有效，此时才能够将idx+1
    //         when((set_idx === 0.U && io.enq_valid) || (set_idx =/= 0.U && enq_valid)){
    //             set_idx := set_idx.asUInt + 1.U
    //         }
    //     }
    // }

    when(!io.enq_valid && !enq_valid){
        set_num := 0.U
        set_idx := 0.U
    }
    .elsewhen(trans_buffer.io.enq.ready){
        when(isBound){//如果当前处于边界，则以为着下周期
            set_idx := 0.U
        }
        .otherwise{
            //如果第一个周期有指令有效或者后面的周期也有效，此时才能够将idx+1
            set_idx := set_idx.asUInt + 1.U
        }
    }


    //当enq有效，并且set_idx为零，并且当前trans_buffer的enq已经就绪，则更新内部的一些寄存器，记录当前输入的情况
    when(io.enq_valid && set_idx === 0.U && trans_buffer.io.enq.ready){
        enq_buffer := io.enq
        enq_valid := io.enq_valid
        set_num := io.set_num
    }

    //当set_idx为0时，直接从输入传递到transbuffer中，节省一个周期
    when(set_idx === 0.U){  
        trans_buffer.io.enq.bits := io.enq(0)
        trans_buffer.io.enq.valid := io.enq_valid
    }
    .otherwise{
        trans_buffer.io.enq.bits := enq_buffer(set_idx)
        trans_buffer.io.enq.valid := enq_valid
    }

    //当出现clear信号时，应将所有信息清除掉
    when(io.clear){
        set_idx := 0.U
        set_num := 0.U
        enq_valid := false.B
        for( w <- 0 until enqTranBuff_entries/tranBuff_enq_width){
            for( i <- 0 until tranBuff_enq_width){
                enq_buffer(w).val_mask(i) := false.B
            }
        }

    }

    val debug_cycles = freechips.rocketchip.util.WideCounter(32)
    when(io.isUnicoreMode){
        printf("cycles: %d, enq_buffer info, io.enq_valid: %d, io.set_num: %d, set_idx: %d, set_num: %d, enq_valid: %d, isbound: %d, tranbuffer_ready: %d, enq_ready: %d, last_cycle_over: %d, last last_cycle_over: %d\n", debug_cycles.value, io.enq_valid, io.set_num, set_idx, set_num, enq_valid, isBound, trans_buffer.io.enq.ready, io.enq_ready, last_cycle_over, RegNext(last_cycle_over))
    }

}
