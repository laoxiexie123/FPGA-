`timescale 1ns / 1ps
// 定位：读取并行ADC，完成八点平均、ADC低通、摆角与摆速估计；不判定运行许可。
// 时钟复位：内部仅clk_50m域，rst_n低有效异步复位；ADC时钟为接口输出。
// 有效性：average_valid_q和angle_valid_q分别标记新结果；filter_ready_q表示已有滤波初值。
// 新鲜度：tick_1ms为同域毫秒使能；平均值已见且样龄<20ms时average_fresh_next有效。
// 边界：filter_clear同步清滤波历史，不清采样计数和平均累计；片外时序由板级接口保证。
// [参数] 50MHz系统时钟，每9375拍采样，8点一组，平均结果每1.5ms发布。
// [平均] average=floor(sum/8)，先锁存引脚，下一拍累计；原始码范围0..1023。
// [滤波] 保留Q8历史：y+=trunc_zero((average*256-y)*128/256)，首样本直接建立初值。
// [标度] angle_x4_q=floor(y/64)，范围0..4092；保留四分之一码，不表示硬件精度增加。
// [延迟] 平均发布下一拍接收；首样本即发布，后续差值、乘法、发布共3拍。
// [清除] filter_clear优先于在途计算和新均值，冲掉该沿滤波事件；平均链持续运行。
module adc_pend (
    input wire clk_50m,
    input wire rst_n,
    input wire filter_clear,
    input wire tick_1ms,
    input wire tick_5ms,
    input wire [11:0] zero_adc_x4,
    input wire [9:0] adc_data_in,
    output wire adc_clk_next,
    output wire adc_oe_n_next,
    output reg [9:0] average_raw_q,
    output reg average_valid_q,
    output reg [11:0] angle_x4_q,
    output wire signed [31:0] filtered_adc_q8,
    output reg angle_valid_q,
    output reg filter_ready_q,
    output reg average_seen_q,
    output reg [31:0] average_age_ms_q,
    output wire average_fresh_next,
    output reg sample_commit_q,
    output reg observer_busy_q,
    output reg pendulum_valid_q,
    output reg signed [31:0] angle_cdeg_q,
    output reg signed [31:0] speed_cdeg_s_q,
    output reg [31:0] observer_seq_q,
    output reg observer_fault_q
);
  localparam [13:0] sample_last = 14'd9374;
  localparam [2:0] average_last = 3'd7;
  localparam signed [31:0] filter_alpha = 32'sd128;
  localparam [31:0] maximum_age_ms = 32'd20;

  reg adc_clock_q;
  reg [13:0] sample_counter_q;
  reg [9:0] sampled_adc_q;
  reg sample_valid_q;
  reg [2:0] average_count_q;
  reg [12:0] average_sum_q;
  wire [12:0] average_total_next = average_sum_q + {3'd0, sampled_adc_q};

  reg signed [31:0] filter_q8_q;
  reg signed [31:0] filter_difference_q;
  reg signed [31:0] filter_product_q;
  reg multiply_valid_q;
  reg publish_valid_q;
  wire signed [31:0] average_q8_next = $signed({14'd0, average_raw_q, 8'd0});
  // 有符号>>>对负数向下取整；原算法要求向零截断，故先对幅值右移再恢复符号。
  // 差值幅值<=261888，乘128后幅值<=33521664，signed32足以保存全部合法中间值。
  wire signed [31:0] filter_increment_next = filter_product_q[31] ?
      -((-filter_product_q) >>> 8) : (filter_product_q >>> 8);
  wire signed [31:0] filter_sum_next = filter_q8_q + filter_increment_next;

  // 只读导出原Q8历史供立摆D项使用，不增加寄存级，不重新滤波。
  assign filtered_adc_q8 = filter_q8_q;
  assign adc_clk_next = adc_clock_q;
  assign adc_oe_n_next = 1'b0;
  assign average_fresh_next = average_seen_q && average_age_ms_q < maximum_age_ms;

  // clk_50m域：ADC平均值样龄；rst_n异步置未见，平均发布优先于毫秒老化。
  // 与旧guard相同，在消费average_valid的下一沿清样龄；滤波清历史不清采集历史。
  always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
      average_seen_q <= 1'b0;
      average_age_ms_q <= 32'h0000ffff;
    end else if (average_valid_q) begin
      average_seen_q <= 1'b1;
      average_age_ms_q <= 32'd0;
    end else if (tick_1ms && average_age_ms_q != 32'hffffffff) begin
      average_age_ms_q <= average_age_ms_q + 32'd1;
    end
  end

  // clk_50m域：仅生成连续25MHz接口时钟，rst_n异步置低，不作为内部逻辑时钟。
  always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
      adc_clock_q <= 1'b0;
    end else begin
      adc_clock_q <= ~adc_clock_q;
    end
  end

  // clk_50m域：定间隔锁存外部总线，rst_n异步复位；未到间隔不读取新样本。
  always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
      sample_counter_q <= 14'd0;
      sampled_adc_q <= 10'd0;
      sample_valid_q <= 1'b0;
    end else begin
      sample_valid_q <= 1'b0;
      if (sample_counter_q == sample_last) begin
        sample_counter_q <= 14'd0;
        sampled_adc_q <= adc_data_in;
        sample_valid_q <= 1'b1;
      end else begin
        sample_counter_q <= sample_counter_q + 14'd1;
      end
    end
  end

  // clk_50m域：只累计上一拍锁存样本，rst_n异步清组；组末包含第8点后再量化。
  always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
      average_count_q <= 3'd0;
      average_sum_q <= 13'd0;
      average_raw_q <= 10'd0;
      average_valid_q <= 1'b0;
    end else begin
      average_valid_q <= 1'b0;
      if (sample_valid_q) begin
        if (average_count_q == average_last) begin
          average_raw_q <= average_total_next[12:3];
          average_valid_q <= 1'b1;
          average_count_q <= 3'd0;
          average_sum_q <= 13'd0;
        end else begin
          average_count_q <= average_count_q + 3'd1;
          average_sum_q <= average_total_next;
        end
      end
    end
  end

  // clk_50m域：Q8滤波采用差值/乘法/发布流水，rst_n异步复位；clear同步冲掉所有在途标志。
  always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
      filter_q8_q <= 32'sd0;
      filter_difference_q <= 32'sd0;
      filter_product_q <= 32'sd0;
      multiply_valid_q <= 1'b0;
      publish_valid_q <= 1'b0;
      angle_x4_q <= 12'd0;
      angle_valid_q <= 1'b0;
      filter_ready_q <= 1'b0;
    end else if (filter_clear) begin
      filter_q8_q <= 32'sd0;
      filter_difference_q <= 32'sd0;
      filter_product_q <= 32'sd0;
      multiply_valid_q <= 1'b0;
      publish_valid_q <= 1'b0;
      angle_x4_q <= 12'd0;
      angle_valid_q <= 1'b0;
      filter_ready_q <= 1'b0;
    end else begin
      angle_valid_q <= 1'b0;
      multiply_valid_q <= 1'b0;
      publish_valid_q <= multiply_valid_q;
      if (average_valid_q) begin
        if (!filter_ready_q) begin
          filter_q8_q <= average_q8_next;
          angle_x4_q <= {average_raw_q, 2'd0};
          filter_ready_q <= 1'b1;
          angle_valid_q <= 1'b1;
        end else begin
          filter_difference_q <= average_q8_next - filter_q8_q;
          multiply_valid_q <= 1'b1;
        end
      end
      if (multiply_valid_q) begin
        filter_product_q <= filter_difference_q * filter_alpha;
      end
      if (publish_valid_q) begin
        filter_q8_q <= filter_sum_next[31:0];
        angle_x4_q <= filter_sum_next[17:6];
        angle_valid_q <= 1'b1;
      end
    end
  end

  // 以下观测流水使用八点平均值，独立于上方平衡ADC低通。
  localparam signed [16:0] obs_angle_scale = 17'sd8518;
  localparam [9:0] obs_adc_min = 10'd1, obs_adc_max = 10'd1022;
  localparam signed [31:0] obs_max_speed = 32'sd200000;
  localparam signed [9:0] obs_speed_alpha = 10'sd16;
  localparam [31:0] obs_reacquire_samples = 32'd2;
  localparam signed [31:0] obs_max_angle_step = 32'sd18000;
  function signed [63:0] obs_trunc_shift;
    input signed [63:0] x;
    input [5:0] bits;
    begin
      if (x < 0) begin
        obs_trunc_shift = -((-x) >>> bits);
      end else begin
        obs_trunc_shift = x >>> bits;
      end
    end
  endfunction
  function obs_fits_s32;
    input signed [63:0] x;
    begin
      obs_fits_s32 = (x >= -64'sd2147483648 && x <= 64'sd2147483647);
    end
  endfunction

  localparam obs_stage_idle = 0, obs_stage_map_multiply = 1, obs_stage_map_check = 2,
      obs_stage_difference = 3, obs_stage_raw_speed = 4, obs_stage_speed_check = 5,
      obs_stage_filter_multiply = 6, obs_stage_filter_add = 7, obs_stage_filter_check = 8,
      obs_stage_publish_speed = 9, obs_stage_commit = 10, obs_stage_invalid = 11;
  reg [3:0] obs_state_q;
  reg signed [12:0] obs_map_delta_q;
  reg signed [63:0] obs_product_q, obs_filter_sum_q;
  reg signed [31:0] obs_sample_angle_q, obs_previous_angle_q, obs_delta_angle_q, obs_raw_speed_q;
  reg signed [31:0] obs_velocity_q8_q, obs_pending_velocity_q8_q, obs_sample_speed_q;
  reg [7:0] obs_consecutive_q;
  wire signed [63:0] obs_mapped_next = obs_trunc_shift(obs_product_q, 6'd10);
  wire signed [63:0] obs_filter_increment_next = obs_trunc_shift(obs_product_q, 6'd8);
  wire signed [63:0] obs_published_speed_next = obs_trunc_shift(
      {{32{obs_pending_velocity_q8_q[31]}}, obs_pending_velocity_q8_q}, 6'd8
  );

  // Range checks bound the published results; upper quantization bits are redundant.
  wire unused_quantized_high = ^{obs_mapped_next[63:32], obs_published_speed_next[63:32]};

  // 无效化清速度及连续样本历史；角度值由调用分支决定是否清零。
  task obs_invalidate;
    begin
      pendulum_valid_q <= 1'd0;
      speed_cdeg_s_q <= 32'sd0;
      obs_consecutive_q <= 8'd0;
      obs_velocity_q8_q <= 32'sd0;
      obs_previous_angle_q <= 32'sd0;
    end
  endtask
  task obs_fault;
    begin
      observer_fault_q <= 1'd1;
      observer_busy_q <= 1'd0;
      obs_state_q <= obs_stage_idle[3:0];
      obs_invalidate;
    end
  endtask

  // 功能：摆角映射、角差测速、速度低通及有效性发布；clk_50m域，rst_n异步复位。
  // filter_clear同步清观测历史和故障；数据失鲜立即取消在途事务，原流水拍数保持。
  always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
      sample_commit_q <= 1'd0;
      observer_busy_q <= 1'd0;
      pendulum_valid_q <= 1'd0;
      angle_cdeg_q <= 32'sd0;
      speed_cdeg_s_q <= 32'sd0;
      observer_seq_q <= 32'd0;
      observer_fault_q <= 1'd0;
      obs_state_q <= obs_stage_idle[3:0];
      obs_consecutive_q <= 8'd0;
      obs_previous_angle_q <= 32'sd0;
      obs_velocity_q8_q <= 32'sd0;
      obs_map_delta_q <= 13'sd0;
      obs_product_q <= 64'sd0;
      obs_filter_sum_q <= 64'sd0;
      obs_sample_angle_q <= 32'sd0;
      obs_delta_angle_q <= 32'sd0;
      obs_raw_speed_q <= 32'sd0;
      obs_pending_velocity_q8_q <= 32'sd0;
      obs_sample_speed_q <= 32'sd0;
    end else begin
      sample_commit_q <= 1'd0;
      if (filter_clear) begin
        observer_fault_q <= 1'd0;
        observer_busy_q <= 1'd0;
        obs_state_q <= obs_stage_idle[3:0];
        angle_cdeg_q <= 32'sd0;
        obs_invalidate;
      end else if (observer_fault_q) begin
        observer_busy_q  <= 1'd0;
        obs_state_q <= obs_stage_idle[3:0];
        obs_invalidate;
      end else if (!average_fresh_next) begin
        // 新鲜度立即门控，在途事务不能在测量过期后有效提交。
        observer_busy_q <= 1'd0;
        obs_state_q <= obs_stage_idle[3:0];
        angle_cdeg_q <= 32'sd0;
        obs_invalidate;
        if (tick_5ms) begin
          sample_commit_q <= 1'd1;
          observer_seq_q  <= observer_seq_q + 1'b1;
        end
      end else if (tick_5ms && observer_busy_q) begin
        obs_fault;
      end else begin
        case (obs_state_q)
          obs_stage_idle: begin
            if (tick_5ms) begin
              observer_busy_q <= 1'd1;
              if (zero_adc_x4 > 12'd4092) begin
                obs_fault;
              end else if (average_raw_q < obs_adc_min || average_raw_q > obs_adc_max) begin
                obs_state_q <= obs_stage_invalid[3:0];
              end else begin
                obs_map_delta_q <= $signed({1'b0, average_raw_q, 2'b00}) - $signed({1'b0, zero_adc_x4});
                obs_state_q <= obs_stage_map_multiply[3:0];
              end
            end
          end
          obs_stage_map_multiply: begin
            obs_product_q <= $signed(obs_map_delta_q[12:0]) * obs_angle_scale;
            obs_state_q   <= obs_stage_map_check[3:0];
          end
          // 量化前先检查 +/-180 度，避免截断掩盖越界。
          obs_stage_map_check: begin
            if (obs_product_q > 64'sd18432000 || obs_product_q < -64'sd18432000) begin
              obs_state_q <= obs_stage_invalid[3:0];
            end else begin
              obs_sample_angle_q <= obs_mapped_next[31:0];
              obs_state_q <= obs_stage_difference[3:0];
            end
          end
          obs_stage_difference: begin
            if (obs_consecutive_q == 0) begin
              obs_previous_angle_q <= obs_sample_angle_q;
              obs_velocity_q8_q <= 32'sd0;
              obs_consecutive_q <= 8'd1;
              angle_cdeg_q <= obs_sample_angle_q;
              pendulum_valid_q <= 1'd0;
              speed_cdeg_s_q <= 32'sd0;
              sample_commit_q <= 1'd1;
              observer_seq_q <= observer_seq_q + 1'b1;
              observer_busy_q <= 1'd0;
              obs_state_q <= obs_stage_idle[3:0];
            end else begin
              obs_delta_angle_q <= obs_sample_angle_q - obs_previous_angle_q;
              obs_state_q <= obs_stage_raw_speed[3:0];
            end
          end
          // 先检查原始跳变/速度，再做 Q8 IIR，不以滤波掩盖越界。
          obs_stage_raw_speed: begin
            if (obs_delta_angle_q > obs_max_angle_step || obs_delta_angle_q < -obs_max_angle_step) begin
              obs_state_q <= obs_stage_invalid[3:0];
            end else begin
              obs_raw_speed_q <= $signed(obs_delta_angle_q[16:0]) * 9'sd200;
              obs_state_q <= obs_stage_speed_check[3:0];
            end
          end
          obs_stage_speed_check: begin
            if (obs_raw_speed_q > obs_max_speed || obs_raw_speed_q < -obs_max_speed) begin
              obs_state_q <= obs_stage_invalid[3:0];
            end else begin
              // obs_max_speed<=200000 保证两项及其差值均容纳于有符号32位。
              obs_delta_angle_q <= (obs_raw_speed_q <<< 8) - obs_velocity_q8_q;
              obs_state_q <= obs_stage_filter_multiply[3:0];
            end
          end
          obs_stage_filter_multiply: begin
            obs_product_q <= $signed(obs_delta_angle_q[27:0]) * obs_speed_alpha;
            obs_state_q   <= obs_stage_filter_add[3:0];
          end
          obs_stage_filter_add: begin
            obs_filter_sum_q <= {{32{obs_velocity_q8_q[31]}}, obs_velocity_q8_q} + obs_filter_increment_next;
            obs_state_q <= obs_stage_filter_check[3:0];
          end
          obs_stage_filter_check: begin
            if (!obs_fits_s32(obs_filter_sum_q)) begin
              obs_fault;
            end else begin
              obs_pending_velocity_q8_q <= obs_filter_sum_q[31:0];
              obs_state_q <= obs_stage_publish_speed[3:0];
            end
          end
          // 独立一拍读取刚更新的 obs_pending_velocity_q8_q，避免发布上一样本速度。
          obs_stage_publish_speed: begin
            obs_sample_speed_q <= obs_published_speed_next[31:0];
            obs_state_q <= obs_stage_commit[3:0];
          end
          // 未到连续样本门限时仅积累内部历史，公开速度仍为零。
          obs_stage_commit: begin
            obs_previous_angle_q <= obs_sample_angle_q;
            obs_velocity_q8_q <= obs_pending_velocity_q8_q;
            if (obs_consecutive_q < 100) begin
              obs_consecutive_q <= obs_consecutive_q + 1'b1;
            end
            angle_cdeg_q <= obs_sample_angle_q;
            if (({24'd0, obs_consecutive_q} + 32'd1) >= obs_reacquire_samples) begin
              pendulum_valid_q  <= 1'd1;
              speed_cdeg_s_q <= obs_sample_speed_q;
            end else begin
              pendulum_valid_q  <= 1'd0;
              speed_cdeg_s_q <= 32'sd0;
            end
            observer_busy_q <= 1'd0;
            obs_state_q <= obs_stage_idle[3:0];
            sample_commit_q <= 1'd1;
            observer_seq_q <= observer_seq_q + 1'b1;
          end
          obs_stage_invalid: begin
            obs_invalidate;
            angle_cdeg_q <= 32'sd0;
            observer_busy_q <= 1'd0;
            obs_state_q <= obs_stage_idle[3:0];
            sample_commit_q <= 1'd1;
            observer_seq_q <= observer_seq_q + 1'b1;
          end
          default: begin
            obs_fault;
          end
        endcase
      end
    end
  end
endmodule
