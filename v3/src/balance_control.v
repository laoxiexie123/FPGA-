`timescale 1ns / 1ps
// 立摆串级控制：位置PD产生角度参考偏置，角度PD产生电机命令。
// 外部ADC提供Q8与四倍标度；本模块不采样、不滤波ADC、不授予运行许可。
// 50MHz单域，tick_5ms为事务使能；rst_n异步低有效复位。
// 输入在wait_control_request接受请求时锁存；仅result_valid发布历史，新位置偏置下一事务生效。
// clear仅清控制/故障；capture_seed保留外部ADC与锁存故障。两者均中止在途计算。
// 舍入：角度D乘积>>>6向下；Q8滤波增量、Q16量化、权重乘积向零截断。
// 优先级：clear > capture > 故障 > stop > origin > 位置关闭沿 > 状态推进。
/* verilator lint_off DECLFILENAME */
module balance_control (
/* verilator lint_on DECLFILENAME */
input wire clk_50m, rst_n, tick_5ms, enable, clear, capture_seed,
input wire entry_boost, position_loop_enable, position_origin_seed,
input wire [11:0] filtered_adc_x4,
input wire signed [31:0] filtered_adc_q8,
input wire adc_ready,
input wire [11:0] zero_adc_x4,
input wire signed [31:0] position_counts,
input wire [15:0] position_weight_q15,
input wire signed [31:0] position_reference_offset,
output wire signed [31:0] position_origin_counts,
output reg signed [15:0] motor_command,
output reg result_valid, calculating, control_fault,
output reg [7:0] fault_code
);
/*
localparam signed [31:0] angle_kp_q16=32'sd80000, angle_kd_q16=32'sd350000;
localparam signed [31:0] position_kp_q16=32'sd10000, position_kd_q16=32'sd500000;
*/
localparam signed [31:0] angle_kp_q16=32'sd70000, angle_kd_q16=32'sd350000;
localparam signed [31:0] position_kp_q16=32'sd10000, position_kd_q16=32'sd600000;
// angle_offset_limit = 150：位置环参考偏置限幅，单位为四倍ADC码，不是150度。
localparam signed [31:0] position_filter_alpha=32'sd64, angle_offset_limit=32'sd150;
localparam signed [15:0] friction_command=16'sd1;
localparam [15:0] motor_command_limit=16'd150;
// 外部ADC两个标度来自同一个滤波历史；range检查不替代上层新鲜度保护。
wire signed [31:0] external_adc_x4 = $signed({20'd0, filtered_adc_x4});
wire signed [31:0] zero_adc_extended = $signed({20'd0, zero_adc_x4});
function signed [63:0] trunc_q8;
  input signed [63:0] value;
  begin
    if (value < 0) begin
      trunc_q8 = -((-value) >>> 8);
    end
    else begin
      trunc_q8 = value >>> 8;
    end
  end
endfunction
function signed [63:0] trunc_q16;
  input signed [63:0] value;
  begin
    if (value < 0) begin
      trunc_q16 = -((-value) >>> 16);
    end
    else begin
      trunc_q16 = value >>> 16;
    end
  end
endfunction
function fits_s32;
  input signed [63:0] value;
  begin
    fits_s32 = (value >= -64'sd2147483648 && value <= 64'sd2147483647);
  end
endfunction
wire position_enabled_next = (position_loop_enable !== 1'b0);
wire origin_seed_next = position_enabled_next && (position_origin_seed === 1'b1);
// position_counts合法范围为±1000000编码器计数；对应机械行程或圈数待确认。
wire position_bad_next = position_counts > 32'sd1000000 || position_counts < -32'sd1000000;
// filtered_adc_x4上限4092 = 1023 * 4，为10位ADC满量程的四倍标度。
// filtered_adc_q8上限261888 = 1023 * 256，为同一ADC的Q8标度，下限0。
wire adc_bad_next = adc_ready && (filtered_adc_x4 > 12'd4092 ||
filtered_adc_q8 < 0 || filtered_adc_q8 > 32'sd261888);
// 8'd1：position_counts越界；8'd3：adc_ready时filtered_adc_x4或filtered_adc_q8越界。
wire [7:0] input_fault_reason_next = position_bad_next ? 8'd1 : adc_bad_next ? 8'd3 :
// 8'd4：enable时zero_adc_x4越过四倍ADC满量程4092。
(enable && zero_adc_x4 > 12'd4092) ? 8'd4 :
// 8'd5：enable且calculating时又收到tick_5ms，计算请求重叠。
(enable && tick_5ms && calculating) ? 8'd5 : 8'd0;
wire input_fault_next = input_fault_reason_next != 0;
localparam [5:0]
wait_control_request=0,
calculate_angle_difference=1,
start_angle_proportional=2,
angle_proportional_ready=3,
combine_angle_proportional_derivative=4,
quantize_angle_command=5,
start_position_filter=6,
position_filter_ready=7,
calculate_position_error=8,
calculate_position_difference=9,
start_position_proportional=10,
position_proportional_ready=11,
combine_position_proportional_derivative=12,
quantize_position_offset=13,
apply_motor_friction=14,
limit_motor_command=15,
publish_control_result=16,
wait_angle_proportional=17,
start_position_weight=18,
position_weight_ready=19,
wait_angle_derivative=32,
wait_position_filter=33,
wait_position_proportional=34,
wait_position_derivative=35,
wait_position_weight=36;
reg [5:0] state_q;
reg position_loop_active_q;
reg position_seeded_q;
reg angle_history_valid_q;
reg position_history_valid_q;
reg [2:0] position_holds_q;
reg position_due_q;
reg boost_snapshot;
reg signed [31:0] previous_angle_error;
reg signed [31:0] previous_position_error;
reg signed [31:0] position_filter_history;
reg signed [31:0] position_target;
reg signed [31:0] reference_snapshot;
assign position_origin_counts = position_target;
reg signed [31:0] position_offset;
reg signed [31:0] position_sample;
reg signed [31:0] position_filter_candidate;
reg signed [31:0] angle_error;
reg signed [31:0] precise_angle_error;
reg signed [31:0] angle_difference;
reg signed [31:0] position_error;
reg signed [31:0] position_difference;
reg signed [31:0] angle_command_candidate;
reg signed [31:0] position_offset_candidate;
reg [15:0] position_weight;
reg signed [15:0] motor_command_candidate;
reg signed [63:0] proportional_derivative_sum;
reg signed [63:0] compensated_motor_command;
wire signed [31:0] selected_angle_kp = boost_snapshot ? angle_kp_q16+(angle_kp_q16>>>1) : angle_kp_q16;
wire signed [31:0] selected_angle_kd = boost_snapshot ? angle_kd_q16+(angle_kd_q16>>>1) : angle_kd_q16;
// 各语义乘积直接表达；等待状态仅保留原提交节拍，不复用公共操作数。
wire signed [63:0] angle_proportional=angle_error*selected_angle_kp;
wire signed [63:0] angle_derivative=angle_difference*selected_angle_kd;
wire signed [63:0] position_proportional=position_error*position_kp_q16;
wire signed [63:0] position_derivative=position_difference*position_kd_q16;
wire signed [31:0] position_filter_difference=(position_sample<<<8)-position_filter_history;
wire signed [63:0] position_filter_increment=position_filter_difference*position_filter_alpha;
wire signed [31:0] position_weight_extended=$signed({16'd0,position_weight});
wire signed [63:0] position_weight_product=position_offset_candidate*position_weight_extended;
// 保留64位滤波、量化及权重结果，避免提前截断；候选寄存器只消费[31:0]。
// 低32位消费者：angle_command_candidate、position_offset_candidate、position_filter_candidate、
// position_error；quantized_pd_next还参与溢出检查。
/* verilator lint_off UNUSEDSIGNAL */
wire signed [63:0] weighted_position_next = position_weight_product < 0 ?
-((-position_weight_product)>>>15) : (position_weight_product>>>15);
wire signed [63:0] quantized_pd_next=trunc_q16(proportional_derivative_sum);
wire signed [63:0] position_filter_result_next = {{32{position_filter_history[31]}},position_filter_history}+trunc_q8(position_filter_increment);
wire signed [63:0] position_integer_next=trunc_q8({{32{position_filter_candidate[31]}},position_filter_candidate});
wire signed [63:0] position_error_wide_next = $signed({{32{position_integer_next[31]}},position_integer_next[31:0]})-
$signed({{32{position_target[31]}},position_target}) -
$signed({{32{reference_snapshot[31]}},reference_snapshot});
/* verilator lint_on UNUSEDSIGNAL */
wire signed [63:0] friction_extended_next={{48{friction_command[15]}},friction_command};
wire signed [63:0] mix_raw_next={{32{angle_command_candidate[31]}},angle_command_candidate};
wire signed [63:0] position_bound_wide_next={{32{angle_offset_limit[31]}},angle_offset_limit};
wire angle_overflow=state_q==quantize_angle_command && !fits_s32(quantized_pd_next);
wire motor_overflow=state_q==limit_motor_command && !fits_s32(compensated_motor_command);
// 状态 20-31 与 37+ 非法；0-19及32-36为合法计算或等待状态。
wire illegal_state=state_q>position_weight_ready &&
!(state_q>=wait_angle_derivative && state_q<=wait_position_weight);
wire position_transition_next=origin_seed_next || (!position_enabled_next && position_loop_active_q);
// 8'd6：quantize_angle_command的量化结果超出signed32范围。
// 8'd7：limit_motor_command的补偿后命令超出signed32范围。
// 8'd8：illegal_state检测到非法编码；X/Z另由legal_state识别。
wire [7:0] arithmetic_reason=angle_overflow ? 8'd6 : motor_overflow ? 8'd7 : illegal_state ? 8'd8 : 8'd0;
wire transition_fault_next=position_transition_next && arithmetic_reason!=0;
localparam [2:0] event_compute=0,event_clear=1,event_capture=2,event_fault=3,
event_stop=4,event_origin=5,event_position_disable=6;
// 优先级：clear > capture > 故障 > stop > origin > 位置关闭沿 > 状态推进；与文件头优先级列表一致，改动需同步。
function [2:0] select_control_event;
  input clear_request,capture_request,fault_request,stop_request,origin_request,disable_request;
  begin
    if(clear_request)select_control_event=event_clear;
    else if(capture_request)select_control_event=event_capture;
    else if(fault_request)select_control_event=event_fault;
    else if(stop_request)select_control_event=event_stop;
    else if(origin_request)select_control_event=event_origin;
    else if(disable_request)select_control_event=event_position_disable;
    else select_control_event=event_compute;
  end
endfunction
wire [2:0] control_event=select_control_event(clear,capture_seed,
input_fault_next || control_fault || transition_fault_next,!enable,origin_seed_next,
!position_enabled_next && position_loop_active_q);
reg legal_state;
// 功能：组合译码合法状态；clk_50m域使用，无寄存器与复位。
always @* begin
  case(state_q)
    0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,32,33,34,35,36:legal_state=1'b1;
    default:legal_state=1'b0;
  endcase
end
wire numerical_fault=control_event==event_compute && (arithmetic_reason!=0 || !legal_state);
wire commit_now=control_event==event_compute && state_q==publish_control_result;
wire position_commit=commit_now && position_enabled_next && position_due_q;
// 功能：计算PD候选、命令限幅与故障仲裁；clk_50m上升沿，rst_n异步低有效复位。
// 独占写state_q、计算候选、motor_command、result_valid、calculating及故障输出。
// 只读第二段历史/原点/position_offset及提交条件，不写历史；诊断端口已移除。
always @(posedge clk_50m or negedge rst_n) begin
  if(!rst_n) begin
    state_q<=wait_control_request;
    result_valid<=0;
    calculating<=0;
    control_fault<=0;
    fault_code<=0;
    motor_command<=0;
    position_due_q<=0;
    boost_snapshot<=0;
    position_sample<=0;
    reference_snapshot<=0;
    position_filter_candidate<=0;
    angle_error<=0;
    precise_angle_error<=0;
    angle_difference<=0;
    position_error<=0;
    position_difference<=0;
    angle_command_candidate<=0;
    position_offset_candidate<=0;
    position_weight<=0;
    motor_command_candidate<=0;
    proportional_derivative_sum<=0;
    compensated_motor_command<=0;
  end
  else begin
    result_valid<=0;
    if(!position_enabled_next)begin
      position_filter_candidate<=0;
      position_error<=0;
      position_difference<=0;
      position_due_q<=0;
      position_offset_candidate<=0;
    end
    if(control_event!=event_compute || numerical_fault)begin
      state_q<=wait_control_request;
      calculating<=0;
      case(control_event)
        event_clear:begin
          control_fault<=0;
          fault_code<=0;
          motor_command<=0;
        end
        event_capture:begin
          motor_command<=0;
          position_due_q<=0;
          control_fault<=control_fault||input_fault_next;
          if(!control_fault&&input_fault_next)fault_code<=input_fault_reason_next;
        end
        event_fault:begin
          control_fault<=1;
          motor_command<=0;
          if(!control_fault)fault_code<=input_fault_next?input_fault_reason_next:arithmetic_reason;
        end
        event_stop:motor_command<=0;
        event_origin:begin
          position_filter_candidate<=position_counts<<<8;
          position_sample<=position_counts;
          position_error<=0;
          position_difference<=0;
          position_due_q<=0;
          position_offset_candidate<=0;
        end
        event_position_disable:begin
          // 本块不处理，由第二段 always 清历史；统一取消逻辑已中止在途计算。
        end
        default:begin
          control_fault<=1;
          fault_code<=!legal_state?8'd8:arithmetic_reason;
          motor_command<=0;
        end
      endcase
    end
    else begin
      case(state_q)
        // wait_control_request: 隔离事务并等5ms请求，锁存输入 → 有请求到calculate_angle_difference，否则保持
        wait_control_request: begin
          if (tick_5ms && adc_ready) begin
            calculating <= 1'd1;
            boost_snapshot <= entry_boost;
            position_sample <= position_counts;
            reference_snapshot <= position_reference_offset;
            position_weight <= position_weight_q15;
            position_due_q <= position_enabled_next && (position_holds_q == 0);
            angle_error <= external_adc_x4 - $signed(
            zero_adc_extended
            ) + (position_enabled_next ? $signed(
            position_offset
            ) : 32'sd0);
            precise_angle_error <= filtered_adc_q8 - ($signed(
            zero_adc_extended
            ) <<< 6) + (position_enabled_next ? ($signed(
            position_offset
            ) <<< 6) : 32'sd0);
            position_offset_candidate <= position_enabled_next ? position_offset : 32'sd0;
            state_q <= calculate_angle_difference;
          end
        end
        // calculate_angle_difference: 求精细误差差值，首次置零 → start_angle_proportional
        calculate_angle_difference: begin
          angle_difference <= angle_history_valid_q ? precise_angle_error - previous_angle_error : 32'sd0;
          state_q   <= start_angle_proportional;
        end
        // start_angle_proportional: 进入角度P等待槽 → wait_angle_proportional
        start_angle_proportional: begin
          state_q <= wait_angle_proportional;
        end
        // angle_proportional_ready: 保留旧P完成槽，转等D → wait_angle_derivative
        angle_proportional_ready: begin
          state_q <= wait_angle_derivative;
        end
        // combine_angle_proportional_derivative: 合并P与恢复标度的D → quantize_angle_command
        combine_angle_proportional_derivative: begin
          proportional_derivative_sum <= angle_proportional + (angle_derivative >>> 6);
          state_q  <= quantize_angle_command;
        end
        // quantize_angle_command: 保存量化命令 → 到期start_position_filter，否则apply_motor_friction
        quantize_angle_command: begin
          angle_command_candidate<=quantized_pd_next[31:0];
          state_q<=position_due_q ? start_position_filter : apply_motor_friction;
        end
        // start_position_filter: 首样本直建Q8 → calculate_position_error；否则wait_position_filter
        start_position_filter: begin
          if (!position_seeded_q) begin
            position_filter_candidate <= position_sample <<< 8;
            state_q <= calculate_position_error;
          end
          else begin
            state_q <= wait_position_filter;
          end
        end
        // position_filter_ready: 保存IIR候选，不覆盖历史 → calculate_position_error
        position_filter_ready: begin
          position_filter_candidate <= position_filter_result_next[31:0];
          state_q <= calculate_position_error;
        end
        // calculate_position_error: 由滤波候选与原点求误差 → calculate_position_difference
        calculate_position_error: begin
          position_error <= position_error_wide_next[31:0];
          state_q <= calculate_position_difference;
        end
        // calculate_position_difference: 求误差差值，首次置零 → start_position_proportional
        calculate_position_difference: begin
          position_difference <= position_history_valid_q ? position_error-previous_position_error : 32'sd0;
          state_q <= start_position_proportional;
        end
        // start_position_proportional: 进入位置P等待槽 → wait_position_proportional
        start_position_proportional: begin
          state_q <= wait_position_proportional;
        end
        // position_proportional_ready: 保留旧P完成槽，转等D → wait_position_derivative
        position_proportional_ready: begin
          state_q <= wait_position_derivative;
        end
        // combine_position_proportional_derivative: 合并位置P与D → quantize_position_offset
        combine_position_proportional_derivative: begin
          proportional_derivative_sum <= position_proportional + position_derivative;
          state_q  <= quantize_position_offset;
        end
        // quantize_position_offset: 量化限幅 → 满权重apply_motor_friction，否则start_position_weight
        quantize_position_offset: begin
          if (quantized_pd_next > position_bound_wide_next) begin
            position_offset_candidate <= angle_offset_limit;
          end
          else if (quantized_pd_next < -position_bound_wide_next) begin
            position_offset_candidate <= -angle_offset_limit;
          end
          else begin
            position_offset_candidate <= quantized_pd_next[31:0];
          end
          // position_weight == 16'd32768：Q15的1.0满权重，旁路缩放以保持原偏置。
          state_q <= (position_weight == 16'd32768) ? apply_motor_friction : start_position_weight;
        end
        // apply_motor_friction: 补同向摩擦，零不补偿 → limit_motor_command
        apply_motor_friction: begin
          if (mix_raw_next > 0) begin
            compensated_motor_command <= mix_raw_next + friction_extended_next;
          end
          else if (mix_raw_next < 0) begin
            compensated_motor_command <= mix_raw_next - friction_extended_next;
          end
          else begin
            compensated_motor_command <= 64'sd0;
          end
          state_q <= limit_motor_command;
        end
        // limit_motor_command: 将补偿后命令限幅为候选 → publish_control_result
        limit_motor_command: begin
          if(compensated_motor_command>$signed({48'd0,motor_command_limit}))motor_command_candidate<=$signed(motor_command_limit);
          else if(compensated_motor_command<-$signed({48'd0,motor_command_limit}))motor_command_candidate<=-$signed(motor_command_limit);
          else motor_command_candidate<=compensated_motor_command[15:0];
          state_q<=publish_control_result;
        end
        // publish_control_result: 唯一commit_now状态，同沿发布命令/历史及result_valid → wait_control_request
        publish_control_result: begin
          result_valid <= 1'b1;
          motor_command <= motor_command_candidate;
          calculating <= 1'b0;
          state_q <= wait_control_request;
        end
        // wait_angle_proportional: 保留旧提交节拍，等P乘积稳定 → angle_proportional_ready
        wait_angle_proportional: state_q<=angle_proportional_ready;
        // wait_angle_derivative: 保留旧提交节拍，等D乘积稳定 → combine_angle_proportional_derivative
        wait_angle_derivative: state_q<=combine_angle_proportional_derivative;
        // wait_position_filter: 保留旧提交节拍，等滤波乘积稳定 → position_filter_ready
        wait_position_filter: state_q<=position_filter_ready;
        // wait_position_proportional: 保留旧提交节拍，等P乘积稳定 → position_proportional_ready
        wait_position_proportional: state_q<=position_proportional_ready;
        // wait_position_derivative: 保留旧提交节拍，等D乘积稳定 → combine_position_proportional_derivative
        wait_position_derivative: state_q<=combine_position_proportional_derivative;
        // wait_position_weight: 保留旧提交节拍，等权重乘积稳定 → position_weight_ready
        wait_position_weight: state_q<=position_weight_ready;
        // start_position_weight: 进入偏置权重乘积等待槽 → wait_position_weight
        start_position_weight: begin
          state_q <= wait_position_weight;
        end
        // position_weight_ready: 向零截断加权偏置并保存候选 → apply_motor_friction
        position_weight_ready: begin
          position_offset_candidate <= weighted_position_next[31:0];
          state_q <= apply_motor_friction;
        end
        default:begin
          // 非法状态已由 numerical_fault 同沿处理，此处不重复动作。
        end
      endcase
    end
  end
end
// 功能：保存提交历史、位置原点及位置更新节奏；clk_50m上升沿，rst_n异步低有效复位。
// 独占写previous_angle_error、previous_position_error、滤波/有效位/原点/偏置及保持计数。
// 只读计算候选及commit_now/position_commit；不写第一段命令、握手或故障输出。
always @(posedge clk_50m or negedge rst_n) begin
  if(!rst_n)begin
    position_loop_active_q<=1;
    previous_angle_error<=0;
    previous_position_error<=0;
    position_filter_history<=0;
    position_target<=0;
    position_offset<=0;
    position_holds_q<=0;
    position_seeded_q<=0;
    angle_history_valid_q<=0;
    position_history_valid_q<=0;
  end
  else begin
    position_loop_active_q<=position_enabled_next;
    if(!position_enabled_next)begin
      position_target<=0;
      position_filter_history<=0;
      position_seeded_q<=0;
      previous_position_error<=0;
      position_history_valid_q<=0;
      position_holds_q<=0;
      position_offset<=0;
    end
    case(control_event)
      event_clear,event_capture:begin
        position_offset<=0;
        position_holds_q<=0;
        angle_history_valid_q<=0;
        position_history_valid_q<=0;
        position_seeded_q<=position_enabled_next&&!position_bad_next;
        if(!position_bad_next)position_filter_history<=position_enabled_next?(position_counts<<<8):32'sd0;
        position_target<=(control_event==event_capture&&position_enabled_next)?position_counts:32'sd0;
        if(control_event==event_capture)begin
          previous_angle_error<=0;
          previous_position_error<=0;
        end
      end
      event_fault,event_stop:begin
        position_offset<=0;
        position_holds_q<=0;
        angle_history_valid_q<=0;
        position_history_valid_q<=0;
        if(control_event==event_stop)begin
          position_filter_history<=position_enabled_next?(position_counts<<<8):32'sd0;
          position_seeded_q<=position_enabled_next;
          position_target<=0;
        end
      end
      event_origin:begin
        position_target<=position_counts;
        position_filter_history<=position_counts<<<8;
        position_seeded_q<=1;
        position_history_valid_q<=0;
        previous_position_error<=0;
        position_holds_q<=0;
        position_offset<=0;
      end
      event_compute:begin
        if(numerical_fault)begin
        end
        else if(commit_now)begin
          previous_angle_error<=precise_angle_error;
          angle_history_valid_q<=1;
          if(position_commit)begin
            position_offset<=position_offset_candidate;
            previous_position_error<=position_error;
            position_history_valid_q<=1;
            position_filter_history<=position_filter_candidate;
            position_seeded_q<=position_enabled_next;
            // position_holds_q装4：后四次5ms提交保持偏置，每五次更新位置（标称25ms）。
            position_holds_q<=4;
          end
          else if(position_enabled_next)position_holds_q<=position_holds_q-1'b1;
        end
      end
      default:begin
      end
    endcase
  end
end
endmodule
