`timescale 1ns / 1ps
// 本机界面装配：旧屏幕菜单和EC11输入；位置移动保留V3的+130计数/4秒逻辑。
// 单一clk_50m域；1/5/10ms均为使能，不作为逻辑时钟。复位异步拉低、同步释放。
// 零点参照for_rip_top/src/pid_ctrl.v：theta_rr=theta_r+135，err=theta_rr-3100。
// 等效原始ADC四倍零点为3100-135=2965；不是未参与此路径的balance_theta宏2950。
module top (
    input wire clk_50m,
    input wire rst_n,
    input wire ec11_a, ec11_b, ec11_button_n,
    output wire lcd_sclk_next, lcd_mosi_next, lcd_dc_next,
    input wire enc1_a,
    input wire enc1_b,
    input wire [9:0] adc_data_in,
    output wire adc_clk_next,
    output wire adc_oe_n_next,
    output wire pwma_next, an1_next, an2_next,
    output wire pwmb_next, bn1_next, bn2_next
);
  wire [11:0] upright_zero;
  // 保护数值沿用旧local_startup_profile：C07/C08/C09/C15/C18。
  localparam signed [31:0] running_speed_limit = 32'sd4400;
  localparam signed [31:0] starting_speed_limit = 32'sd200;
  localparam signed [31:0] position_limit = 32'sd10000;
  localparam signed [12:0] upright_error_limit = 13'sd400;
  localparam [9:0] adc_rail_margin = 10'd10;
  localparam [6:0] ready_window_ms = 7'd100;
  reg reset_meta, reset_sync;
  wire reset_ok = reset_sync;
  // 功能：外部复位同步释放；clk_50m域，rst_n异步低有效。
  // 仅写reset_meta/reset_sync；子模块只读reset_ok，末级不反馈同步器，不增加时钟域。
  always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
      reset_meta <= 1'b0;
      reset_sync <= 1'b0;
    end else begin
      reset_meta <= 1'b1;
      reset_sync <= reset_meta;
    end
  end

  reg [15:0] millisecond_divider;
  reg [2:0] five_ms_phase;
  reg [3:0] ten_ms_phase;
  reg tick_1ms, tick_5ms, tick_10ms;
  // 功能：统一生成单拍使能；clk_50m域，reset_ok异步低有效。
  // 仅写分频计数和tick；其他模块读取使能，不得将tick接入时钟端口。
  always @(posedge clk_50m or negedge reset_ok) begin
    if (!reset_ok) begin
      millisecond_divider <= 0;
      five_ms_phase <= 0;
      ten_ms_phase <= 0;
      tick_1ms <= 0;
      tick_5ms <= 0;
      tick_10ms <= 0;
    end else begin
      tick_1ms <= 0;
      tick_5ms <= 0;
      tick_10ms <= 0;
      // 50MHz / 50000 = 1kHz，与旧top的毫秒时基一致。
      if (millisecond_divider == 16'd49999) begin
        millisecond_divider <= 0;
        tick_1ms <= 1;
        if (five_ms_phase == 3'd4) begin
          five_ms_phase <= 0;
          tick_5ms <= 1;
        end else five_ms_phase <= five_ms_phase + 1'b1;
        if (ten_ms_phase == 4'd9) begin
          ten_ms_phase <= 0;
          tick_10ms <= 1;
        end else ten_ms_phase <= ten_ms_phase + 1'b1;
      end else millisecond_divider <= millisecond_divider + 1'b1;
    end
  end

  // 用户确认13PPR、四倍频、减速比20：1040 counts/圈，45度=130 counts。
  // 顺时针计数增加；旋钮±130或顺时针整圈+1040，每次4秒，忙时忽略。
  wire signed [31:0] requested_step_counts;
  wire position_mode;
  wire ec_cw, ec_ccw, ec_press;
  wire ui_request, ui_owned, ui_stop, ui_busy, ui_done, ui_query_done, ui_armed;
  wire [3:0] ui_kind;
  wire [5:0] ui_index;
  wire signed [31:0] ui_value, ui_target;
  wire [2:0] ui_page;
  wire [1:0] ui_selection;
  wire [31:0] ui_zero, ui_errors;
  wire ui_start, command_stop, encoder_zero, move_press;
  wire [7:0] display_fault;
  wire ui_fault_ack;
  reg display_fault_latched;
  wire reference_moving;
  wire stop_request = command_stop || ui_stop;
  reg control_release;
  wire control_reset_ok = reset_ok && control_release;
  // 功能：停止命令清控制会话；clk_50m域，reset_ok异步低有效。
  // 仅写control_release；不复位UI或ADC，保留菜单、校准值及测量历史。
  always @(posedge clk_50m or negedge reset_ok) begin
    if (!reset_ok) control_release <= 0;
    else control_release <= !stop_request;
  end
  /* verilator lint_off PINCONNECTEMPTY */
  ec11_input knob (
      .clk(clk_50m), .rst_n(reset_ok), .a(ec11_a), .b(ec11_b),
      .button_n(ec11_button_n), .step_cw_q(ec_cw), .step_ccw_q(ec_ccw), .press_q(ec_press)
  );

  wire [9:0] average_raw;
  wire [11:0] filtered_adc_x4;
  wire signed [31:0] filtered_adc_q8, angle_cdeg, speed_cdeg_s;
  wire filter_ready, average_fresh, sample_commit, pendulum_valid, observer_fault;
  wire average_valid;
  wire [31:0] average_age;
  adc_pend pendulum (
      .clk_50m(clk_50m), .rst_n(reset_ok), .filter_clear(1'b0),
      .tick_1ms(tick_1ms), .tick_5ms(tick_5ms), .zero_adc_x4(upright_zero),
      .adc_data_in(adc_data_in), .adc_clk_next(adc_clk_next), .adc_oe_n_next(adc_oe_n_next),
      .average_raw_q(average_raw), .average_valid_q(average_valid), .angle_x4_q(filtered_adc_x4),
      .filtered_adc_q8(filtered_adc_q8), .angle_valid_q(), .filter_ready_q(filter_ready),
      .average_seen_q(), .average_age_ms_q(average_age), .average_fresh_next(average_fresh),
      .sample_commit_q(sample_commit), .observer_busy_q(), .pendulum_valid_q(pendulum_valid),
      .angle_cdeg_q(angle_cdeg), .speed_cdeg_s_q(speed_cdeg_s),
      .observer_seq_q(), .observer_fault_q(observer_fault)
  );
  wire signed [31:0] position_counts, arm_speed;
  wire encoder_fault;
  encoder arm_encoder (
      .clk_50m(clk_50m), .rst_n(reset_ok), .enc_a(enc1_a), .enc_b(enc1_b),
      .zero(encoder_zero), .tick_10ms(tick_10ms), .position_q(position_counts),
      .speed_q(arm_speed), .invalid_count_q(), .overflow_fault_q(encoder_fault)
  );

  wire core_enable, core_reseed, core_commit, core_fault;
  wire entry_boost, position_loop_enable, position_origin_seed, position_ready;
  wire signed [31:0] reference_offset, position_origin;
  wire [15:0] position_weight;
  wire signed [15:0] core_duty, supervised_duty;
  wire [1:0] main_state;
  wire [7:0] swing_fault;
  wire [3:0] swing_phase;
  reg run_requested, fault_latched;
  reg [6:0] ready_ms;
  wire signed [12:0] raw_angle_error =
      $signed({1'b0, average_raw, 2'b00}) - $signed({1'b0, upright_zero});
  wire position_bad = position_counts > position_limit || position_counts < -position_limit;
  wire speed_bad = arm_speed > running_speed_limit || arm_speed < -running_speed_limit;
  wire starting_speed_bad =
      arm_speed > starting_speed_limit || arm_speed < -starting_speed_limit;
  wire balance_stage = main_state == 2'd3;
  // 起摆阶段保留旧模式3的角度/轨端豁免；正常立摆恢复C07和C18检查。
  wire balance_measurement_bad = balance_stage &&
      (average_raw < adc_rail_margin || average_raw > 10'd1023 - adc_rail_margin ||
       raw_angle_error > upright_error_limit || raw_angle_error < -upright_error_limit);
  // 起摆允许观测暂未有效：只禁止用无效观测捕获；真实observer_fault仍属于硬故障。
  wire measurement_ready = average_fresh && filter_ready;
  wire component_fault = observer_fault || encoder_fault || core_fault || swing_fault != 0;
  wire live_fault = !measurement_ready || component_fault || position_bad || speed_bad ||
      balance_measurement_bad;
  wire start_ready = measurement_ready && !component_fault && !position_bad && !starting_speed_bad;
  wire ready_to_start = start_ready && ready_ms == ready_window_ms &&
      !fault_latched && control_reset_ok;
  wire start_request = ui_start && ready_to_start && !run_requested;
  wire motor_permitted = reset_ok && run_requested && !fault_latched && !live_fault && !stop_request && control_reset_ok;
  // 功能：本地启动准入和首故障锁停；clk_50m域，reset_ok异步低有效。
  // 仅写run_requested/fault_latched/ready_ms；只读传感器、按键及控制故障，不重算PD。
  // 待机测量未就绪只阻止启动；运行故障锁停，经菜单停止命令清会话后重新准入。
  always @(posedge clk_50m or negedge reset_ok) begin
    if (!reset_ok) begin
      run_requested <= 0;
      fault_latched <= 0;
      ready_ms <= 0;
    end else begin
      if (!start_ready || fault_latched) ready_ms <= 0;
      else if (tick_1ms && ready_ms < ready_window_ms) ready_ms <= ready_ms + 1'b1;
      if (stop_request) begin
        run_requested <= 0;
        fault_latched <= 0;
        ready_ms <= 0;
      end else if (run_requested && live_fault) begin
        run_requested <= 0;
        fault_latched <= 1;
      end else if (start_request) run_requested <= 1;
    end
  end

  position_reference reference_generator (
      .clk_50m(clk_50m), .rst_n(reset_ok), .tick_1ms(tick_1ms),
      .ready(position_ready && motor_permitted), .move_press(move_press),
      .origin_counts(position_origin), .step_counts(requested_step_counts),
      .position_limit(position_limit), .target_offset_counts(reference_offset),
      .moving(reference_moving), .target_done()
  );

  swing_control supervisor (
      .clk_50m(clk_50m), .rst_n(control_reset_ok), .tick_1ms(tick_1ms), .tick_5ms(tick_5ms),
      .start_request(start_request), .stop_request(fault_latched || stop_request),
      .safety_fault(run_requested && live_fault), .sample_commit(sample_commit),
      .angle_valid(pendulum_valid), .observer_fault(observer_fault), .average_raw(average_raw),
      .filtered_adc_x4(filtered_adc_x4), .zero_adc_x4(upright_zero),
      .angle_cdeg(angle_cdeg), .speed_cdeg_s(speed_cdeg_s), .arm_speed(arm_speed),
      .core_duty(core_duty), .core_commit(core_commit), .core_fault(core_fault),
      .core_enable(core_enable), .core_reseed_q(core_reseed), .position_weight_q15(position_weight),
      .duty_q(supervised_duty), .entry_boost(entry_boost),
      .position_loop_enable(position_loop_enable), .position_origin_seed(position_origin_seed),
      .position_ready(position_ready), .main_state_q(main_state), .phase_q(swing_phase), .fault_q(swing_fault), .state_ms_q()
  );
  balance_control balance (
      .clk_50m(clk_50m), .rst_n(control_reset_ok), .tick_5ms(tick_5ms),
      .enable(core_enable && motor_permitted), .clear(1'b0), .capture_seed(core_reseed),
      .entry_boost(entry_boost), .position_loop_enable(position_loop_enable),
      .position_origin_seed(position_origin_seed), .filtered_adc_x4(filtered_adc_x4),
      .filtered_adc_q8(filtered_adc_q8), .adc_ready(filter_ready && average_fresh),
      .zero_adc_x4(upright_zero), .position_counts(position_counts),
      .position_weight_q15(position_weight), .position_reference_offset(reference_offset),
      .position_origin_counts(position_origin), .motor_command(core_duty),
      .result_valid(core_commit), .calculating(), .control_fault(core_fault), .fault_code()
  );
  // 自动停止可清控制会话，但故障提示须保留到操作者确认或复位。
  always @(posedge clk_50m or negedge reset_ok) begin
    if (!reset_ok) display_fault_latched <= 1'b0;
    else if (run_requested && live_fault) display_fault_latched <= 1'b1;
    else if (ui_fault_ack || start_request) display_fault_latched <= 1'b0;
  end
  assign display_fault = display_fault_latched ? 8'd1 : 8'd0;
  ui_control_bridge command_bridge (
      .clk(clk_50m), .rst_n(reset_ok), .tick_1ms(tick_1ms),
      .command_request(ui_request), .command_kind(ui_kind), .command_index(ui_index),
      .command_value(ui_value), .start_ready(ready_to_start), .run_fault(fault_latched),
      .position_ready(position_ready && motor_permitted), .trajectory_fault(1'b0),
      .running(run_requested), .position_limit(position_limit), .tx_busy(ui_busy),
      .tx_done(ui_done), .query_done(ui_query_done), .armed(ui_armed),
      .start_pulse(ui_start), .stop_pulse(command_stop), .zero_pulse(encoder_zero),
      .heartbeat_pulse(), .motion_enter(), .motion_exit(), .motion_target(move_press),
      .motion_target_offset(requested_step_counts), .zero_adc_x4(upright_zero), .error_count(ui_errors)
  );
  ui_controller ui (
      .clk(clk_50m), .rst_n(reset_ok), .tick_1ms(tick_1ms),
      .step_cw(ec_cw), .step_ccw(ec_ccw), .press(ec_press),
      .running(run_requested), .armed(ui_armed), .position_ready(position_ready && motor_permitted),
      .adjust_active(1'b0), .braking(reference_moving), .swing_state(swing_phase),
      .fault(display_fault), .key_stop(1'b0), .serial_activity(1'b0), .serial_stop(1'b0),
      .query_done(ui_query_done), .parser_errors(ui_errors), .adc_raw({6'd0, average_raw}),
      .adc_valid(average_valid), .adc_age(average_age), .goal_offset(reference_offset),
      .tx_busy(ui_busy), .tx_done(ui_done), .cmd_request_q(ui_request), .cmd_kind_q(ui_kind),
      .cmd_index_q(ui_index), .cmd_value_q(ui_value), .local_owned_q(ui_owned),
      .local_stop_q(ui_stop), .fault_ack_q(ui_fault_ack), .page_q(ui_page), .selection_q(ui_selection),
      .target_offset_q(ui_target), .upright_zero_q(ui_zero), .position_mode_q(position_mode)
  );
  // 页面和字形复用旧实现，数值来自当前控制数据，不回灌旧控制核心。
  ui_lcd display (
      .clk(clk_50m), .rst_n(reset_ok), .page(ui_page), .selection(ui_selection),
      .status(!run_requested ? 2'd0 : (main_state == 2'd3 ? 2'd2 : 2'd1)),
      .position_active(reference_moving), .position_mode(position_mode), .adc_raw({6'd0, average_raw}),
      .position(position_counts), .target_offset(reference_offset),
      .actual_offset(position_counts - position_origin), .pwm(supervised_duty),
      .zero_x4({20'd0, upright_zero}), .fault(display_fault),
      .angle_cdeg(angle_cdeg), .angle_valid(pendulum_valid),
      .angle_kp(32'd80000), .angle_kd(32'd350000),
      .position_kp(32'd10000), .position_kd(32'd500000),
      .lcd_sclk_next(lcd_sclk_next), .lcd_mosi_next(lcd_mosi_next), .lcd_dc_next(lcd_dc_next)
  );
  // UI保留输出用于界面状态，但控制只消费已校验的命令桥结果。
  /* verilator lint_off UNUSEDSIGNAL */
  wire unused_owned = ui_owned;
  wire signed [31:0] unused_target = ui_target;
  wire [31:0] unused_zero = ui_zero;
  /* verilator lint_on UNUSEDSIGNAL */
  /* verilator lint_on PINCONNECTEMPTY */
  motro motor (
      .clk_50m(clk_50m), .rst_n(reset_ok), .enable(motor_permitted),
      .duty(supervised_duty), .in1_q(bn1_next), .in2_q(bn2_next), .pwm_q(pwmb_next)
  );
  // 旧约束确认XH1为B通道；未使用的A通道明确关闭，不悬空驱动电机。
  assign pwma_next = 1'b0;
  assign an1_next = 1'b0;
  assign an2_next = 1'b0;
endmodule
