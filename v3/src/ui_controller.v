`timescale 1ns / 1ps
// 定位：管理本机菜单、加载与校准，经串行器提交命令，不绕过 parser/guard。
// 时序：clk 单域，tick_1ms 仅作使能，rst_n 低有效异步复位。
// 有效性：cmd_request_q 为 1 时 cmd_kind_q/cmd_index_q/cmd_value_q 才可信。
// 握手：tx_done 决定发送后推进；固定启动配置由 local_startup_profile 提供。
// [加载] 顺序为 S→C00..47→U/V/J/B/I→Z→H→查询→稳定等待→A→L。
// [覆盖] C06 发送当前校准值，其他 C 项直接取组合表，不增加等待拍。
// [交互] 运行菜单：步进模式每格±130；整圈请求+1040；4秒轨迹由控制模块执行。
// [舍入] 零点为 round_half_up((cal_sum_q+adc_raw)/32)*4，即加 16 后 >>5 再 <<2。
// [校准] 只提交 C06，恢复默认也不恢复整套启动表；复位默认零点由 profile 提供。
// [安全] 本机占用期间，停止、解析错误与运行故障优先中断当前操作。
module ui_controller (
    input wire clk,
    rst_n,
    tick_1ms,
    step_cw,
    step_ccw,
    press,
    input wire running,
    armed,
    position_ready,
    /* verilator lint_off UNUSEDSIGNAL */
    adjust_active,
    /* verilator lint_on UNUSEDSIGNAL */
    braking,
    /* verilator lint_off UNUSEDSIGNAL */
    input wire [3:0] swing_state,
    /* verilator lint_on UNUSEDSIGNAL */
    input wire [7:0] fault,
    input wire key_stop,
    serial_activity,
    serial_stop,
    query_done,
    input wire [31:0] parser_errors,
    input wire [15:0] adc_raw,
    input wire adc_valid,
    input wire [31:0] adc_age,
    input wire signed [31:0] goal_offset,
    input wire tx_busy,
    tx_done,
    output reg cmd_request_q,
    output reg [3:0] cmd_kind_q,
    output reg [5:0] cmd_index_q,
    output reg signed [31:0] cmd_value_q,
    output reg local_owned_q,
    output reg local_stop_q,
    output reg fault_ack_q,
    output reg position_mode_q,
    output reg [2:0] page_q,
    output reg [1:0] selection_q,
    output reg signed [31:0] target_offset_q,
    output reg [31:0] upright_zero_q
);
  localparam state_menu = 0, state_wait_tx = 1, state_load_cfg = 2, state_load_u = 3,
      state_load_v = 4, state_load_j = 5, state_load_b = 6, state_zero_enc = 7,
      state_start_heart = 8, state_start_query = 9, state_wait_query = 10, state_settle = 11,
      state_arm = 12, state_wait_arm = 13, state_start = 14, state_wait_run = 15, state_active = 16,
      state_send_target = 17, state_stopping = 18, state_release = 19, state_cal_sample = 20,
      state_cal_write = 21, state_cal_restore = 22, state_load_i = 23;
  reg [5:0] state_q, after_tx_q, config_index_q;
  reg [15:0] timer_ms_q, quiet_ms_q;
  reg [5:0] heart_ms_q;
  reg [31:0] error_base_q;
  reg [20:0] cal_sum_q;
  reg [5:0] cal_count_q;
  reg cal_bad_q;
  /* verilator lint_off UNUSEDSIGNAL */
  reg target_pending_q;
  /* verilator lint_on UNUSEDSIGNAL */
  reg last_running_q;
  reg query_seen_q;
  reg press_pending_q;
  reg [15:0] startup_age_q;
  // [位宽] 配置与默认零点为 32 位无符号；五项启动命令为 32 位有符号。
  wire [31:0] cfg_value_next, default_zero_next;
  wire signed [31:0] startup_u_next, startup_v_next, startup_j_next, startup_b_next, startup_i_next;
  // [加载] 查表索引直接接当前配置索引，组合输出同拍送入发送任务，无新增等待。
  local_startup_profile u_startup_profile (
      .config_index(config_index_q), .cfg_value_next(cfg_value_next),
      .startup_u_next(startup_u_next), .startup_v_next(startup_v_next),
      .startup_j_next(startup_j_next),
      .startup_b_next(startup_b_next), .startup_i_next(startup_i_next),
      .default_zero_next(default_zero_next)
  );
  /* verilator lint_off UNUSEDSIGNAL */
  wire active_press_next = press || press_pending_q;
  /* verilator lint_on UNUSEDSIGNAL */
  wire stopped_next = !running && !armed;
  wire local_failure_next = local_owned_q && state_q != state_menu && state_q != state_stopping &&
      state_q != state_release &&
      ((key_stop && !(state_q == state_wait_tx && after_tx_q == state_release)) || serial_stop ||
       parser_errors != error_base_q ||
       ((state_q == state_active || state_q == state_send_target) && (fault != 0 || !running)));
  // [握手] 请求仅拉高 1 个 clk；发送完成后按 after_tx_q 返回，禁止同拍连续提交。
  task send_command;
    input [3:0] kind;
    input [5:0] idx;
    input signed [31:0] value;
    input [5:0] next_state;
    begin
      cmd_kind_q <= kind;
      cmd_index_q <= idx;
      cmd_value_q <= value;
      cmd_request_q <= 1'd1;
      after_tx_q <= next_state;
      state_q <= state_wait_tx[5:0];
    end
  endtask
  // [时序] 职责：仲裁菜单、加载、交互与校准；clk 域，rst_n 异步低复位全部寄存器。
  // [优先级] 停止与解析错误优先于菜单和发送；同拍后写覆盖前面的计时或目标更新。
  // [复位] 回到菜单并释放本机占用；默认零点来自组合 profile，尚未向解析器写入。
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state_q <= state_menu[5:0];
      after_tx_q <= state_menu[5:0];
      config_index_q <= 6'd0;
      timer_ms_q <= 16'd0;
      quiet_ms_q <= 16'd0;
      heart_ms_q <= 6'd0;
      cmd_request_q <= 1'd0;
      cmd_kind_q <= 4'd0;
      cmd_index_q <= 6'd0;
      cmd_value_q <= 32'sd0;
      local_owned_q <= 1'd0;
      local_stop_q <= 1'd0;
      fault_ack_q <= 1'b0;
      position_mode_q <= 1'b0;
      page_q <= 3'd0;
      selection_q <= 2'd0;
      target_offset_q <= 32'sd0;
      upright_zero_q <= default_zero_next;
      error_base_q <= 32'd0;
      cal_sum_q <= 21'd0;
      cal_count_q <= 6'd0;
      cal_bad_q <= 1'd0;
      target_pending_q <= 1'd0;
      last_running_q <= 1'd0;
      query_seen_q <= 1'd0;
      press_pending_q <= 1'd0;
      startup_age_q <= 16'd0;
    end else begin
      cmd_request_q  <= 1'd0;
      local_stop_q   <= 1'd0;
      fault_ack_q <= 1'b0;
      last_running_q <= running;
      // 退出模式不等待命令握手；当前在途轨迹仍由控制模块完成。
      if (!running || fault != 0 || key_stop || serial_stop || local_failure_next)
        position_mode_q <= 1'b0;
      else if (position_mode_q && press && page_q == 3'd2)
        position_mode_q <= 1'b0;
      if (query_done) begin
        query_seen_q <= 1'd1;
      end

      // V3移动请求不排队：握手忙时的按压丢弃，长按仍只有单个事件。
      press_pending_q <= 1'b0;
      // [门限] 启动总龄 16 位饱和，查询等待达到 2500 ms 进入停机。
      if (local_owned_q && tick_1ms && startup_age_q < 65535) begin
        startup_age_q <= startup_age_q + 1'b1;
      end
      if (state_q == state_menu) begin
        startup_age_q   <= 16'd0;
        press_pending_q <= 1'd0;
      end
      // [门限] 串口静默累计到 1000 ms 饱和；取得本机占用要求连续静默至少 200 ms。
      if (serial_activity) begin
        quiet_ms_q <= 16'd0;
      end else if (tick_1ms && quiet_ms_q < 1000) begin
        quiet_ms_q <= quiet_ms_q + 1'b1;
      end
      // [计时] 16 位等待计时在 65535 ms 饱和，6 位心跳计时在 63 ms 饱和。
      if (tick_1ms) begin
        if (timer_ms_q != 65535) begin
          timer_ms_q <= timer_ms_q + 1'b1;
        end
        if (heart_ms_q < 63) begin
          heart_ms_q <= heart_ms_q + 1'b1;
        end
      end
      if (local_failure_next) begin
        local_stop_q <= 1'd1;
        state_q <= state_stopping[5:0];
        page_q <= 3'd4;
        selection_q <= 2'd0;
        target_pending_q <= 1'd0;
      end else if (key_stop && state_q == state_wait_tx && after_tx_q == state_release) begin
        local_stop_q <= 1'd1;
        state_q <= state_release[5:0];
      end else begin
        case (state_q)
          // [流程] 本拍处理菜单与外部运行显示；确认后发停止并由发送等待转入加载或校准。
          state_menu: begin
            local_owned_q <= 1'd0;
            timer_ms_q <= 16'd0;
            heart_ms_q <= 6'd0;
            error_base_q <= parser_errors;
            if (running) begin
              page_q <= 3'd2;
              selection_q <= 2'd0;
            end else if (last_running_q) begin
              page_q <= fault != 0 ? 3'd4 : 3'd0;
              selection_q <= 2'd0;
            end
            if (!running && fault != 0 && page_q != 3) begin
              page_q <= 3'd4;
            end
            if (page_q == 0) begin
              if (step_cw || step_ccw) begin
                selection_q <= selection_q == 0 ? 2'd1 : 2'd0;
              end
              if (press && stopped_next && quiet_ms_q >= 200 && !serial_activity && !key_stop) begin
                if (selection_q == 0) begin
                  page_q <= 3'd1;
                  selection_q <= 2'd0;
                end else begin
                  page_q <= 3'd3;
                  selection_q <= 2'd0;
                end
              end
            end else if (page_q == 1) begin
              if (step_cw || step_ccw) begin
                selection_q <= selection_q == 0 ? 2'd1 : 2'd0;
              end
              if (press && stopped_next && quiet_ms_q >= 200 && !serial_activity && !key_stop) begin
                if (selection_q == 0) begin
                  page_q <= 3'd0;
                  selection_q <= 2'd0;
                end else begin
                  local_owned_q <= 1'd1;
                  error_base_q <= parser_errors;
                  config_index_q <= 6'd0;
                  page_q <= 3'd0;
                  selection_q <= 2'd0;
                  send_command(4'd7, 6'd0, 32'sd0, state_load_cfg[5:0]);
                end
              end
            end else if (page_q == 3) begin
              if (step_cw) begin
                selection_q <= selection_q == 2 ? 2'd0 : selection_q + 2'd1;
              end
              if (step_ccw) begin
                selection_q <= selection_q == 0 ? 2'd2 : selection_q - 2'd1;
              end
              if (press && stopped_next && quiet_ms_q >= 200 && !serial_activity && !key_stop) begin
                if (selection_q == 2) begin
                  page_q <= 3'd0;
                  selection_q <= 2'd0;
                end else begin
                  local_owned_q <= 1'd1;
                  error_base_q <= parser_errors;
                  cal_sum_q <= 21'd0;
                  cal_count_q <= 6'd0;
                  cal_bad_q <= 1'd0;
                  timer_ms_q <= 16'd0;
                  send_command(4'd7, 6'd0, 32'sd0,
                               selection_q == 0 ? state_cal_sample[5:0] : state_cal_restore[5:0]);
                end
              end
            end else if (page_q == 4 && press && stopped_next && quiet_ms_q >= 200 &&
                         !serial_activity && !key_stop) begin
              fault_ack_q <= 1'b1; // 操作者确认故障页，区别于自动停止。
              local_owned_q <= 1'd1;
              error_base_q  <= parser_errors;
              send_command(4'd7, 6'd0, 32'sd0, state_release[5:0]);
              page_q <= 3'd0;
            end
          end
          // [流程] 本拍仅等待 tx_done；完成后转 after_tx_q 并清计时，否则保持等待。
          state_wait_tx: begin
            if (tx_done) begin
              state_q <= after_tx_q;
              timer_ms_q <= 16'd0;
            end
          end
          // [流程] 本拍空闲时发送当前 C 项；C06 用当前校准值覆盖，C47 后转 U，其余继续 C。
          state_load_cfg: begin
            if (!tx_busy) begin
              send_command(4'd0, config_index_q,
                           config_index_q == 6 ? upright_zero_q : cfg_value_next,
                           config_index_q == 47 ? state_load_u : state_load_cfg);
              config_index_q <= config_index_q + 1'b1;
            end
          end
          // [流程] 本拍空闲时加载 profile 的 U；发送完成后转 V。
          state_load_u: begin
            if (!tx_busy) begin
              send_command(4'd1, 6'd0, startup_u_next, state_load_v[5:0]);
            end
          end
          // [流程] 本拍空闲时加载 profile 的 V；发送完成后转 J。
          state_load_v: begin
            if (!tx_busy) begin
              send_command(4'd2, 6'd0, startup_v_next, state_load_j[5:0]);
            end
          end
          // [流程] 本拍空闲时加载 profile 的 J；发送完成后转 B。
          state_load_j: begin
            if (!tx_busy) begin
              send_command(4'd3, 6'd0, startup_j_next, state_load_b[5:0]);
            end
          end
          // [流程] 本拍空闲时加载 profile 的 B；发送完成后转 I。
          state_load_b: begin
            if (!tx_busy) begin
              send_command(4'd4, 6'd0, startup_b_next, state_load_i[5:0]);
            end
          end
          // [流程] 本拍空闲时加载 profile 的 I；发送完成后转编码器清零。
          state_load_i: begin
            if (!tx_busy) begin
              send_command(4'd13, 6'd0, startup_i_next, state_zero_enc[5:0]);
            end
          end
          // [流程] 本拍空闲时发 Z 清零；发送完成后转首个心跳。
          state_zero_enc: begin
            if (!tx_busy) begin
              send_command(4'd8, 6'd0, 32'sd0, state_start_heart[5:0]);
            end
          end
          // [流程] 本拍空闲时发 H 并清心跳计时；发送完成后转查询。
          state_start_heart: begin
            if (!tx_busy) begin
              heart_ms_q <= 6'd0;
              send_command(4'd12, 6'd0, 32'sd0, state_start_query[5:0]);
            end
          end
          // [流程] 本拍空闲时清查询锁存并发查询；发送完成后转查询等待。
          state_start_query: begin
            if (!tx_busy) begin
              query_seen_q <= 1'd0;
              send_command(4'd9, 6'd0, 32'sd0, state_wait_query[5:0]);
            end
          end
          // [流程] 本拍见查询完成转稳定等待；启动超时转停止，否则定期发 H 后返回本态。
          state_wait_query: begin
            if (query_seen_q) begin
              state_q <= state_settle[5:0];
              timer_ms_q <= 16'd0;
            end else if (startup_age_q >= 2500) begin
              local_stop_q <= 1'd1;
              state_q <= state_stopping[5:0];
              page_q <= 3'd4;
            end else if (heart_ms_q >= 20 && !tx_busy) begin
              heart_ms_q <= 6'd0;
              send_command(4'd12, 6'd0, 32'sd0, state_wait_query[5:0]);
            end
          end
          // [流程] 本拍稳定满 150 ms 且请求已接收才转解锁；等待期间每 20 ms 补 H。
          state_settle: begin
            // [握手] 串行器下一沿才接收请求；busy 和 cmd_request 都清空才允许解锁。
            // [优先级] 满 150 ms 时先转解锁，不再补心跳，防止两请求重叠。
            if (timer_ms_q >= 150 && !tx_busy && !cmd_request_q) begin
              state_q <= state_arm[5:0];
            end else if (timer_ms_q < 150 && heart_ms_q >= 20 && !tx_busy && !cmd_request_q) begin
              heart_ms_q <= 6'd0;
              cmd_kind_q <= 4'd12;
              cmd_index_q <= 6'd0;
              cmd_value_q <= 32'sd0;
              cmd_request_q <= 1'd1;
            end
          end
          // [流程] 本拍空闲时发 A=1；发送完成后转解锁等待。
          state_arm: begin
            if (!tx_busy) begin
              send_command(4'd10, 6'd0, 32'sd1, state_wait_arm[5:0]);
            end
          end
          // [流程] 本拍见 armed 转启动；故障或 250 ms 超时转停止，否则保持。
          state_wait_arm: begin
            if (armed) begin
              state_q <= state_start[5:0];
            end else if (fault != 0 || timer_ms_q >= 250) begin
              local_stop_q <= 1'd1;
              state_q <= state_stopping[5:0];
              page_q <= 3'd4;
            end
          end
          // [流程] 本拍空闲时发 L=1；发送完成后转运行等待。
          state_start: begin
            if (!tx_busy) begin
              send_command(4'd11, 6'd0, 32'sd1, state_wait_run[5:0]);
            end
          end
          // [流程] 本拍见 running 转运行页并清目标；故障或 250 ms 超时转停止。
          state_wait_run: begin
            if (running) begin
              state_q <= state_active[5:0];
              page_q <= 3'd2;
              selection_q <= 2'd0;
              target_offset_q <= 32'sd0;
              target_pending_q <= 1'd0;
            end else if (fault != 0 || timer_ms_q >= 250) begin
              local_stop_q <= 1'd1;
              state_q <= state_stopping[5:0];
              page_q <= 3'd4;
            end
          end
          // 两项运行菜单；步进模式按压优先退出，旋转/整圈请求忙时丢弃，不排队。
          state_active: begin
            page_q <= 3'd2;
            target_offset_q <= goal_offset;
            target_pending_q <= 1'b0;
            press_pending_q <= 1'b0;
            if (position_mode_q) begin
              if (press) begin
                position_mode_q <= 1'b0;
              end else if (position_ready && !braking && !tx_busy && (step_cw ^ step_ccw)) begin
                send_command(4'd6, 6'd0, step_cw ? 32'sd130 : -32'sd130,
                             state_active[5:0]);
              end else if (heart_ms_q >= 20 && !tx_busy) begin
                heart_ms_q <= 6'd0;
                send_command(4'd12, 6'd0, 32'sd0, state_active[5:0]);
              end
            end else begin
              if (press) begin
                if (position_ready && !braking && !tx_busy) begin
                  if (selection_q == 0) position_mode_q <= 1'b1;
                  else send_command(4'd6, 6'd0, 32'sd1040, state_active[5:0]);
                end
              end else if ((step_cw ^ step_ccw) && !braking && !tx_busy) begin
                selection_q <= selection_q == 0 ? 2'd1 : 2'd0;
              end else if (heart_ms_q >= 20 && !tx_busy) begin
                heart_ms_q <= 6'd0;
                send_command(4'd12, 6'd0, 32'sd0, state_active[5:0]);
              end
            end
          end
          // [流程] 本拍持续请求本机停机；空闲时发 S，发送完成后转释放。
          state_stopping: begin
            local_stop_q <= 1'd1;
            target_pending_q <= 1'd0;
            if (!tx_busy) begin
              send_command(4'd7, 6'd0, 32'sd0, state_release[5:0]);
            end
          end
          // [流程] 本拍撤销占用与待发目标，后继回菜单。
          state_release: begin
            local_owned_q <= 1'd0;
            state_q <= state_menu[5:0];
            selection_q <= 2'd0;
            target_pending_q <= 1'd0;
          end
          // [流程] 本拍累计有效 ADC；第 32 个样本合格转写入，过期、超时或轨边转释放。
          // [门限] 只在停止时采样；样龄 >=20 ms、历时 >=1000 ms 或 ADC 不在 10..1013 拒绝。
          state_cal_sample: begin
            if (!stopped_next || adc_age >= 20 || timer_ms_q >= 1000) begin
              state_q <= state_release[5:0];
              page_q  <= 3'd4;
            end else if (adc_valid) begin
              cal_sum_q   <= cal_sum_q + {5'd0, adc_raw};
              cal_count_q <= cal_count_q + 1'b1;
              if (adc_raw < 10 || adc_raw > 1013) begin
                cal_bad_q <= 1'd1;
              end
              if (cal_count_q == 31) begin
                if (cal_bad_q || adc_raw < 10 || adc_raw > 1013) begin
                  state_q <= state_release[5:0];
                  page_q  <= 3'd4;
                end else begin
                  // [位宽] 21 位和容纳 32 个 16 位原始值，末样本显式扩到 32 位。
                  // [舍入] 加末样本及 16 后右移 5 位，正数半入取均值，再左移 2 位得 ADC×4。
                  upright_zero_q <= (({11'd0, cal_sum_q} + {16'd0, adc_raw} + 32'd16) >> 5) << 2;
                  state_q <= state_cal_write[5:0];
                end
              end
            end
          end
          // [流程] 本拍空闲时仅发送当前 C06；发送完成后转释放。
          state_cal_write: begin
            if (!tx_busy) begin
              send_command(4'd0, 6'd6, upright_zero_q, state_release[5:0]);
            end
          end
          // [校准] 恢复仅发送 C06，其他配置和 Ki 保持原值。
          // [流程] 本拍空闲时用 profile 默认零点更新并发送 C06；发送完成后转释放。
          state_cal_restore: begin
            if (!tx_busy) begin
              upright_zero_q <= default_zero_next;
              send_command(4'd0, 6'd6, default_zero_next, state_release[5:0]);
            end
          end
          // [异常] 本拍请求停止，后继转停止流程；不直接返回菜单掩盖非法状态。
          default: begin
            local_stop_q <= 1'd1;
            state_q <= state_stopping[5:0];
          end
        endcase
      end
    end
  end
endmodule
