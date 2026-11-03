`timescale 1ns / 1ps
// 固定参数脉冲监督器；12级state/epoch流水保留旧逐拍运算。
// 数据流拍号：以差值/起点/权重进入p0的上升沿N为第1拍；均指该沿NBA更新后。
// p0=N(第1拍)：锁差值、起点、权重；p1=N+1(第2拍)：差值乘权重。
// p2=N+2(第3拍)：混合求和；p3=N+3(第4拍)：按阶段选择目标。
// p4=N+4(第5拍)：起摆限幅；p5=N+5(第6拍)：全局限幅。
// p6=N+6(第7拍)：锁目标及变化范围；p7=N+7(第8拍)：限制变化上界。
// p8=N+8(第9拍)：限制变化下界；p9=N+9(第10拍)：保留对齐延迟。
// p10=N+10(第11拍)：最终限幅；p11=N+11(第12拍)：发布候选及交接比较标志。
// 阶段标签逐拍同行；p3目标选择读取当拍state，候选仍须phase/epoch匹配才可消费。
// candidate在N+11沿后可见，独立时序消费者最早N+12沿读取，且须满足控制请求。
module swing_control (
 input wire clk_50m, rst_n, tick_1ms, tick_5ms,
 input wire start_request, stop_request, safety_fault,
 input wire sample_commit, angle_valid, observer_fault,
 input wire [9:0] average_raw,
 input wire [11:0] filtered_adc_x4, zero_adc_x4,
 input wire signed [31:0] angle_cdeg, speed_cdeg_s, arm_speed,
 input wire signed [15:0] core_duty,
 input wire core_commit, core_fault,
 output wire core_enable,
 output reg core_reseed_q,
 output wire [15:0] position_weight_q15,
 output reg signed [15:0] duty_q,
 output wire entry_boost,
 output wire position_loop_enable,
 output wire position_origin_seed,
 output wire position_ready,
 output wire [1:0] main_state_q,
 output wire [3:0] phase_q,
 output reg [7:0] fault_q,
 output reg [31:0] state_ms_q
);
 localparam stop=0,capture_wait=4,blend=5,balance=6,
            imp_drive=10,imp_brake=11,imp_coast=12,imp_kick=13,imp_wait=14,imp_fire=15;
 // 起摆脉冲：驱动持续时间，单位ms；用于首次imp_kick和后续imp_fire。
 localparam [31:0] pulse_ms = 180;
 // 起摆脉冲：每次驱动后的反刹时间，单位ms。
 localparam [31:0] brake_ms = 100;
 // 起摆脉冲：驱动总次数，包含首次驱动；末次反刹后进入滑行。
 localparam [31:0] pulse_count = 2;
 // 起摆脉冲：驱动命令幅值，单位为电机命令刻度，不是占空比百分数。
 localparam signed [15:0] drive_level = 180;
 // 起摆脉冲：反刹命令幅值；现有逻辑还用它确定阶段命令变化上限。
 localparam signed [15:0] brake_level = 180;
 // 输出保护：监督器最终命令的绝对值上限，包含起摆、交接和正常立摆。
 localparam signed [15:0] command_limit = 220;

 // 再次触发：|摆角|<=85度；还须新观测有效、朝直立运动且已武装。
 localparam [31:0] trigger_angle = 8500;       // 单位0.01度。
 // 重新武装：|摆角|>trigger_angle+该值，即当前超过90度；单位0.01度。
 localparam [31:0] trigger_hysteresis = 500;
 // 等待保护：imp_wait阶段等待再次触发的超时时间，单位ms。
 localparam [31:0] wait_timeout = 2000;
 // 总时限：从启动至进入正常balance前的最长时间，包含捕获与混合；单位ms。
 localparam [31:0] total_timeout = 10000;

 // 捕获条件：|摆角|<=25度；同时要求有效观测和其余捕获条件。
 localparam [31:0] capture_angle = 2500;       // 单位0.01度。
 // 捕获条件：|摆角速度|<=1000度/秒，单位0.01度/秒。
 localparam [31:0] capture_speed = 100000;
 // 捕获条件：|摆臂编码器速度|<=800 counts/秒。
 localparam [31:0] capture_arm_speed = 4400;
 // 捕获确认：每个合格观测累计5ms，不合格即清零；达到该值后进入捕获等待。
 localparam [31:0] capture_dwell = 5;          // 单位ms，按观测周期量化。
 // 捕获丢失：捕获等待、混合或正常立摆时，|摆角|>32度即停机。
 localparam [31:0] loss_angle = 3200;          // 单位0.01度。
 // 捕获丢失：上述阶段|摆角速度|>2000度/秒即停机；观测无效也会判丢失。
 localparam [31:0] loss_speed = 200000;        // 单位0.01度/秒。
 // 捕获交接：混合权重从0推进至1的标称时间；输出仍须满足变化率及交接判据。
 localparam [31:0] blend_duration = 5;         // 单位ms。
 localparam integer pipeline_depth = 12;
 localparam integer stage_target = 3, stage_slew_upper = 6, stage_slew_lower = 7;
 localparam integer stage_final_limit = pipeline_depth - 2;
 localparam integer stage_candidate = pipeline_depth - 1;
 localparam integer blend_fraction_bits = 15;
 localparam [15:0] full_blend_weight = 16'd32768;
 localparam [31:0] full_blend_accum = 32'd32768;
 localparam [15:0] handover_slew = 16'd20;
 localparam [31:0] core_watchdog_ms = 32'd20;
 localparam [31:0] sample_period_ms = 32'd5;
 localparam [31:0] minimum_blend_ms = 32'd5;
 localparam [10:0] boost_duration_ms = 11'd2000;
 localparam integer position_origin_delay_cycles = 25000000;
 // 16位有符号命令扩展到18位；函数返回signed，保留原比较/取负语义。
 function signed [17:0] sext18;
   input signed [15:0] value;
   begin
     sext18 = {{2{value[15]}}, value};
   end
 endfunction
 // 捕获条件：原始平均ADC和滤波ADC相对零点的误差都须在±380内，单位ADC×4。
 localparam signed [12:0] capture_adc_bound = 13'sd380;
 reg core_capture_ready_q;
 wire core_capture_ready = core_capture_ready_q;
 wire signed [12:0] filtered_error_next =
     $signed({1'b0, filtered_adc_x4}) - $signed({1'b0, zero_adc_x4});
 wire signed [12:0] raw_error_next =
     $signed({1'b0, average_raw, 2'b00}) - $signed({1'b0, zero_adc_x4});

 // 功能：原始与滤波ADC捕获准入；clk_50m域，rst_n异步置不就绪，保留一拍寄存。
 always @(posedge clk_50m or negedge rst_n) begin
   if (!rst_n) begin
     core_capture_ready_q <= 1'b0;
   end else begin
     core_capture_ready_q <= filtered_error_next <= capture_adc_bound &&
         filtered_error_next >= -capture_adc_bound && raw_error_next <= capture_adc_bound &&
         raw_error_next >= -capture_adc_bound;
   end
 end
 reg [3:0] state_q;
 reg start_previous_q;
 wire active_next = state_q != stop;
 wire start_edge_next = start_request && !start_previous_q;
 reg [24:0] origin_delay_count_q;
 reg position_origin_locked_q, position_origin_pulse_q;
 wire position_permitted = active_next && state_q == balance && fault_q == 0 &&
     !stop_request && !safety_fault && !observer_fault && !core_fault && !core_reseed_q;
 // 使能保留旧top的locked直连；停止当沿前由许可门控seed/ready，下个沿撤销locked。
 assign position_loop_enable = position_origin_locked_q;
 assign position_origin_seed = position_origin_pulse_q && position_permitted;
 assign position_ready = position_permitted && position_origin_locked_q &&
     !position_origin_pulse_q;

 // 功能：正常立摆连续0.5秒后锁定位置原点并发单拍种子；clk_50m域，rst_n异步清会话。
 always @(posedge clk_50m or negedge rst_n) begin
   if (!rst_n) begin
     origin_delay_count_q <= 25'd0;
     position_origin_locked_q <= 1'b0;
     position_origin_pulse_q <= 1'b0;
   end else begin
     position_origin_pulse_q <= 1'b0;
     if (!position_permitted) begin
       origin_delay_count_q <= 25'd0;
       position_origin_locked_q <= 1'b0;
     end else if (!position_origin_locked_q) begin
       if ({7'd0, origin_delay_count_q} == position_origin_delay_cycles - 1) begin
         position_origin_locked_q <= 1'b1;
         position_origin_pulse_q <= 1'b1;
       end else begin
         origin_delay_count_q <= origin_delay_count_q + 25'd1;
       end
     end
   end
 end
 assign phase_q=state_q;
 assign main_state_q=(state_q==stop)?2'd0:
     ((state_q==capture_wait || state_q==blend)?2'd2:
     ((state_q==balance)?2'd3:2'd1));
  // 功能：记录启动电平以检测上升沿；clk_50m域，rst_n异步清零。
 always @(posedge clk_50m or negedge rst_n) begin
   if(!rst_n) start_previous_q<=1'b0;
   else start_previous_q<=start_request;
 end
  reg [3:0] impulse_count_q;
  reg brake_last_q;  // 1表示反刹位于末次脉冲之后，随后进入滑行
  reg [31:0] wait_ms_q;
  reg trigger_armed_q;
  wire [31:0] impulse_limit_next=pulse_count, trigger_angle_next=trigger_angle;
  wire [31:0] pulse_ms_next=pulse_ms, brake_ms_next=brake_ms;
  wire [31:0] dwell_ms_next=capture_dwell, blend_ms_next=blend_duration;
  function magnitude_gt;
    input [31:0] value, bound;
    begin
      magnitude_gt = value[31] ? ((~value) >= bound) : (value > bound);
    end
  endfunction
  wire trigger_inside_next = !magnitude_gt(angle_cdeg, trigger_angle_next);
  wire trigger_toward_next=angle_cdeg!=32'sd0 && speed_cdeg_s!=32'sd0 &&
  (angle_cdeg[31]^speed_cdeg_s[31]);
  wire impulse_trigger_next=sample_commit && angle_valid && trigger_inside_next &&
  trigger_toward_next && trigger_armed_q;
  wire capture_good_next = angle_valid && core_capture_ready && !magnitude_gt(
      angle_cdeg, capture_angle
  ) && !magnitude_gt(
      speed_cdeg_s, capture_speed
  ) && !magnitude_gt(
      arm_speed, capture_arm_speed
  );
  wire capture_lost_next = !angle_valid || magnitude_gt(
      angle_cdeg, loss_angle
  ) || magnitude_gt(
      speed_cdeg_s, loss_speed
  );
  reg [31:0] elapsed_ms_q, capture_ms_q, core_age_ms_q;
  reg [15:0] weight_q;
  reg [31:0] weight_remainder_q;
  reg blend_step_q;
  reg signed [15:0] blend_origin_q;
  reg [10:0] boost_ms_q;  // 最多容纳2047ms
  assign entry_boost = boost_ms_q != 0;
  wire handover_stage_next = (state_q == capture_wait || state_q == blend || state_q == balance);
  wire [15:0] stage_slew_next=(handover_stage_next && brake_level>handover_slew)?
   handover_slew:brake_level;
  wire signed [15:0] slew_next = $signed(stage_slew_next);
  wire signed [16:0] ulimit_next = $signed({1'b0, command_limit});
  wire signed [15:0] swing_limit_next = $signed(command_limit);
  wire signed [15:0] signed_kick_next = $signed(drive_level);
  wire signed [15:0] signed_brake_next = $signed(brake_level);
  wire reverse_polarity_next = 1'b0;
  assign core_enable=active_next && fault_q==0 &&
      (state_q==capture_wait || state_q==blend || state_q==balance);
  assign position_weight_q15=(state_q==balance)?full_blend_weight:
      ((state_q==blend)?weight_q:16'd0);
  reg [15:0] epoch_q;
  reg [pipeline_depth-1:0] pv_q;
  reg [3:0] ps_q[0:pipeline_depth-1];
  reg [15:0] pe_q[0:pipeline_depth-1];
  reg signed [15:0] difference_p0_q, origin_p0_q, origin_p1_q;
  reg signed [16:0] weight_p0_q;
  reg signed [32:0] product_p1_q;
  reg signed [17:0] blend_p2_q, target_p3_q, state_p4_q;
  reg signed [15:0] u_p5_q, target_p6_q, upper_p7_q, lower_p8_q, pass_p9_q, u_p10_q, candidate_q;
  reg signed [16:0] upper_p6_q, lower_p6_q, lower_p7_q;
  reg candidate_equal_q;
  reg signed [15:0] candidate_core_q;
  reg candidate_full_weight_q;
  reg [15:0] pw_q[0:pipeline_depth-1];
  reg signed [15:0] pc_q[0:pipeline_depth-1];
  reg tick_pending_q;
  reg [3:0] pending_state_q;
  reg [15:0] pending_epoch_q;
  wire candidate_valid_next = pv_q[stage_candidate] && ps_q[stage_candidate] == state_q && pe_q[stage_candidate] == epoch_q;
  wire consume_next=(tick_5ms || (tick_pending_q && pending_state_q==state_q && pending_epoch_q
      ==epoch_q)) && candidate_valid_next;
  // Legacy explicitly discards these high result bits; retain original arithmetic width.
  /* verilator lint_off UNUSEDSIGNAL */
  wire signed [32:0] blend_sum_next =
      $signed({{17{origin_p1_q[15]}}, origin_p1_q}) + (product_p1_q >>> blend_fraction_bits);
  wire signed [16:0] negative_ulimit_next = -ulimit_next;
  /* verilator lint_on UNUSEDSIGNAL */
  integer p;
  // 功能：命令候选运算和状态批次流水；clk_50m域，rst_n异步清流水。
  always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
      pv_q <= {pipeline_depth{1'b0}};
      for (p = 0; p < pipeline_depth; p = p + 1) begin
        ps_q[p] <= stop[3:0];
        pe_q[p] <= 16'd0;
        pw_q[p] <= 16'd0;
        pc_q[p] <= 16'sd0;
      end
      difference_p0_q <= 16'sd0;
      origin_p0_q <= 16'sd0;
      origin_p1_q <= 16'sd0;
      weight_p0_q <= 17'sd0;
      product_p1_q <= 33'sd0;
      blend_p2_q <= 18'sd0;
      target_p3_q <= 18'sd0;
      state_p4_q <= 18'sd0;
      u_p5_q <= 16'sd0;
      target_p6_q <= 16'sd0;
      upper_p6_q <= 17'sd0;
      lower_p6_q <= 17'sd0;
      upper_p7_q <= 16'sd0;
      lower_p7_q <= 17'sd0;
      lower_p8_q <= 16'sd0;
      pass_p9_q <= 16'sd0;
      u_p10_q <= 16'sd0;
      candidate_q <= 16'sd0;
      candidate_equal_q <= 1'd0;
      candidate_core_q <= 16'sd0;
      candidate_full_weight_q <= 1'd0;
    end else begin
      if(!active_next || stop_request || safety_fault || fault_q!=0 || observer_fault || core_fault)begin
        pv_q <= {pipeline_depth{1'b0}};
      end else begin
        pv_q <= {pv_q[pipeline_depth-2:0], 1'b1};
      end
      ps_q[0] <= state_q;
      pe_q[0] <= epoch_q;
      pw_q[0] <= weight_q;
      pc_q[0] <= core_duty;
      for (p = 1; p < pipeline_depth; p = p + 1) begin
        ps_q[p] <= ps_q[p-1];
        pe_q[p] <= pe_q[p-1];
        pw_q[p] <= pw_q[p-1];
        pc_q[p] <= pc_q[p-1];
      end
      difference_p0_q <= $signed(core_duty) - $signed(blend_origin_q);
      origin_p0_q <= blend_origin_q;
      weight_p0_q <= $signed({1'b0, weight_q});
      product_p1_q <= difference_p0_q * weight_p0_q;
      origin_p1_q <= origin_p0_q;
      blend_p2_q <= blend_sum_next[17:0];
      case (state_q)
        imp_drive: begin
          target_p3_q <= reverse_polarity_next ?
              -sext18(signed_kick_next) :
              sext18(signed_kick_next);
        end
        imp_brake: begin
          target_p3_q <= reverse_polarity_next ?
              sext18(signed_brake_next) :
              -sext18(signed_brake_next);
        end
        imp_kick: begin
          target_p3_q <= reverse_polarity_next ?
              -sext18(signed_kick_next) :
              sext18(signed_kick_next);
        end
        imp_fire: begin
          target_p3_q <= reverse_polarity_next ?
              -sext18(signed_kick_next) :
              sext18(signed_kick_next);
        end
        capture_wait: begin
          target_p3_q <= sext18(duty_q);
        end
        blend: begin
          target_p3_q <= blend_p2_q;
        end
        balance: begin
          target_p3_q <= sext18(core_duty);
        end
        default: begin
          target_p3_q <= 18'sd0;
        end
      endcase
      state_p4_q <= target_p3_q;
      if(ps_q[stage_target]==imp_drive || ps_q[stage_target]==imp_brake || ps_q[stage_target]==imp_kick || ps_q[stage_target]==imp_fire)begin
        if (target_p3_q > sext18(swing_limit_next)) begin
          state_p4_q <= sext18(swing_limit_next);
        end else if (target_p3_q < -sext18(swing_limit_next)) begin
          state_p4_q <= -sext18(swing_limit_next);
        end
      end
      if (state_p4_q > $signed({ulimit_next[16], ulimit_next})) begin
        u_p5_q <= ulimit_next[15:0];
      end else if (state_p4_q < -$signed({ulimit_next[16], ulimit_next})) begin
        u_p5_q <= negative_ulimit_next[15:0];
      end else begin
        u_p5_q <= state_p4_q[15:0];
      end
      target_p6_q <= u_p5_q;
      upper_p6_q  <= $signed({duty_q[15], duty_q}) + $signed({slew_next[15], slew_next});
      lower_p6_q  <= $signed({duty_q[15], duty_q}) - $signed({slew_next[15], slew_next});
      lower_p7_q  <= lower_p6_q;
      if (ps_q[stage_slew_upper] != balance && $signed({target_p6_q[15], target_p6_q}) > upper_p6_q) begin
        upper_p7_q <= upper_p6_q[15:0];
      end else begin
        upper_p7_q <= target_p6_q;
      end
      if (ps_q[stage_slew_lower] != balance && $signed({upper_p7_q[15], upper_p7_q}) < lower_p7_q) begin
        lower_p8_q <= lower_p7_q[15:0];
      end else begin
        lower_p8_q <= upper_p7_q;
      end
      pass_p9_q <= lower_p8_q;
      if ($signed({pass_p9_q[15], pass_p9_q}) > ulimit_next) begin
        u_p10_q <= ulimit_next[15:0];
      end else if ($signed({pass_p9_q[15], pass_p9_q}) < -ulimit_next) begin
        u_p10_q <= negative_ulimit_next[15:0];
      end else begin
        u_p10_q <= pass_p9_q;
      end
      candidate_q <= u_p10_q;
      candidate_equal_q <= u_p10_q == pc_q[stage_final_limit];
      candidate_core_q <= pc_q[stage_final_limit];
      candidate_full_weight_q <= pw_q[stage_final_limit] == full_blend_weight;
    end
  end
  task change_state;
    input [3:0] value;
    begin
      state_q <= value;
      epoch_q <= epoch_q + 1'b1;
      tick_pending_q <= 1'd0;
    end
  endtask

  task trip;
    input [7:0] code;
    begin
      fault_q<=code; change_state(stop[3:0]); duty_q<=16'sd0;
      state_ms_q<=0; boost_ms_q<=0; weight_q<=0;
      weight_remainder_q<=0; core_age_ms_q<=0;
    end
  endtask
  // 功能：起摆、捕获交接、超时和停止仲裁；clk_50m域，rst_n异步清控制历史。
  always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
      state_q<=stop[3:0]; fault_q<=0; duty_q<=0; core_reseed_q<=0;
      state_ms_q<=0; elapsed_ms_q<=0; capture_ms_q<=0; core_age_ms_q<=0;
      weight_q<=0; weight_remainder_q<=0; blend_origin_q<=0; epoch_q<=0;
      tick_pending_q<=0; pending_state_q<=0; pending_epoch_q<=0;
      impulse_count_q<=0; wait_ms_q<=0; trigger_armed_q<=1;
      boost_ms_q<=0; brake_last_q<=0;
    end else begin
      core_reseed_q<=0;
      if(tick_1ms && boost_ms_q!=0) boost_ms_q<=boost_ms_q-1'b1;
      if(stop_request) begin
        change_state(stop[3:0]); duty_q<=0; state_ms_q<=0;
        boost_ms_q<=0; weight_q<=0; weight_remainder_q<=0;
      end else if(safety_fault || observer_fault || core_fault) begin
        if(active_next || start_edge_next) trip(8'd4);
      end else if(state_q==stop) begin
        duty_q<=0; boost_ms_q<=0;
        if(start_edge_next) begin
          change_state(imp_kick[3:0]); fault_q<=0;
          state_ms_q<=0; elapsed_ms_q<=0; capture_ms_q<=0; core_age_ms_q<=0;
          weight_q<=0; weight_remainder_q<=0; blend_origin_q<=0;
          impulse_count_q<=0; wait_ms_q<=0; trigger_armed_q<=1; brake_last_q<=0;
        end
      end else begin
        if (tick_1ms) begin
          if (state_ms_q != 32'hffffffff) begin
            state_ms_q <= state_ms_q + 1'b1;
          end
          if (elapsed_ms_q != 32'hffffffff) begin
            elapsed_ms_q <= elapsed_ms_q + 1'b1;
          end
          if (core_enable) begin
            core_age_ms_q <= core_age_ms_q + 1'b1;
          end else begin
            core_age_ms_q <= 32'd0;
          end
          if (state_q == blend && weight_q < full_blend_weight) begin
            weight_remainder_q <= weight_remainder_q + full_blend_accum;
          end
          if (state_q == imp_wait && wait_ms_q != 32'hffffffff) begin
            wait_ms_q <= wait_ms_q + 1'b1;
          end
        end
        if (blend_step_q && weight_q < full_blend_weight) begin
          weight_remainder_q <= weight_remainder_q - blend_ms_next;
          weight_q <= weight_q + 1'b1;
        end
        if (core_commit) begin
          core_age_ms_q <= 32'd0;
        end
        if (sample_commit && angle_valid && magnitude_gt(
                angle_cdeg, trigger_angle_next + trigger_hysteresis
            )) begin
          trigger_armed_q <= 1'd1;
        end
        if (tick_5ms) begin
          tick_pending_q  <= 1'd1;
          pending_state_q <= state_q;
          pending_epoch_q <= epoch_q;
        end
        if (consume_next) begin
          duty_q <= candidate_q;
          epoch_q <= epoch_q + 1'b1;
          tick_pending_q <= 1'd0;
        end
        if (state_q != stop && state_q != balance && elapsed_ms_q >= total_timeout) begin
          trip(8'd3);
        end else if (core_enable && core_age_ms_q >= core_watchdog_ms) begin
          trip(8'd6);
        end
    else if((state_q==capture_wait || state_q==blend || state_q==balance) && sample_commit
        && capture_lost_next)begin
          trip(8'd5);
        end else begin
          case (state_q)
            imp_drive, imp_brake, imp_coast, imp_kick, imp_wait, imp_fire: begin
              if((state_q==imp_kick || state_q==imp_fire) && state_ms_q>=
                  pulse_ms_next)begin
                impulse_count_q <= impulse_count_q + 1'b1;
                brake_last_q <= ({28'd0, impulse_count_q} + 32'd1 >= impulse_limit_next);
                change_state(imp_brake[3:0]);
                state_ms_q <= 32'd0;
                wait_ms_q  <= 32'd0;
              end else if(state_q==imp_brake && state_ms_q>=brake_ms_next)begin
                if (brake_last_q) begin
                  change_state(imp_coast[3:0]);
                end else begin
                  change_state(imp_wait[3:0]);
                end
                state_ms_q <= 32'd0;
                wait_ms_q  <= 32'd0;
              end else if(state_q==imp_wait && impulse_trigger_next &&
                  !(capture_good_next))begin
                change_state(imp_fire[3:0]);
                state_ms_q <= 32'd0;
                wait_ms_q <= 32'd0;
                trigger_armed_q <= 1'd0;
              end else if (state_q == imp_wait && wait_ms_q >= wait_timeout) begin
                trip(8'd3);
              end else if (sample_commit) begin
                if (capture_good_next) begin
                  if (capture_ms_q + sample_period_ms >= dwell_ms_next) begin
                    change_state(capture_wait[3:0]);
                    state_ms_q <= 32'd0;
                    core_age_ms_q <= 32'd0;
                    core_reseed_q <= 1'd1;
                    boost_ms_q <= boost_duration_ms;
                    weight_q <= 16'd0;
                    weight_remainder_q <= 32'd0;
                    blend_origin_q <= duty_q;
                    duty_q <= duty_q;
                  end else begin
                    capture_ms_q <= capture_ms_q + sample_period_ms;
                  end
                end else begin
                  capture_ms_q <= 32'd0;
                end
              end
            end
            capture_wait: begin
              if (core_commit) begin
                change_state(blend[3:0]);
                state_ms_q <= 32'd0;
                weight_q <= 16'd0;
                weight_remainder_q <= 32'd0;
                duty_q <= duty_q;
              end
            end
            blend: begin
              if(consume_next && weight_q==full_blend_weight && candidate_full_weight_q
                  && candidate_equal_q && candidate_core_q==core_duty)begin
                change_state(balance[3:0]);
                state_ms_q <= 32'd0;
              end
            end
            balance: begin
            end
            default: begin
              trip(8'd1);
            end
          endcase
        end
      end
    end
  end
  // 功能：寄存混合权重步进使能；clk_50m域，rst_n异步撤销使能。
  always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
      blend_step_q <= 1'd0;
    end else begin
      blend_step_q<=(!tick_1ms) && state_q==blend && weight_q<full_blend_weight && blend_ms_next>=minimum_blend_ms &&
                    weight_remainder_q>=blend_ms_next;
    end
  end
endmodule
