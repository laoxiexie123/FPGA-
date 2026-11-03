`timescale 1ns / 1ps
// 定位：读取AB编码器并输出四倍频位置、10ms窗口速度；不控制电机或判定运行许可。
// 时钟复位：内部仅clk_50m域，rst_n低有效异步复位；AB分别经两级同步。
// 有效性：首次稳定相位仅建基准；speed_q在tick_10ms更新，其余时间保持。
// 接口：tick_10ms须为同域每10ms一个周期的使能；zero同步清位置及测速历史。
// [方向] 00->10->11->01->00每步+1，反向每步-1；不定义机械顺逆时针。
// [判稳] 固定4拍：候选变化置计数1，达到4后的下一判定沿接收，不提前一拍。
// [清零] zero清溢出、重新建相位但保留非法次数；同步链和预热继续推进。
// [异常] 双位跳变不改位置但更新相位基准；异常次数达到32'hffffffff后饱和。
// [位置] signed32端点向外计数保持并锁故障，之后合法反向步仍可计数。
// [速度] signed33位位置差乘40'sd100，饱和至signed32，单位counts/s，无额外低通。
// [同沿] 位置接受与tick同沿时，速度读更新前位置；zero优先于tick，避免清零尖峰。
module encoder (
    input wire clk_50m,
    input wire rst_n,
    input wire enc_a,
    input wire enc_b,
    input wire zero,
    input wire tick_10ms,
    output reg signed [31:0] position_q,
    output reg signed [31:0] speed_q,
    output reg [31:0] invalid_count_q,
    output reg overflow_fault_q
);
  localparam [2:0] stable_cycles = 3'd4;
  localparam signed [31:0] position_max = 32'sh7fffffff;
  localparam signed [31:0] position_min = 32'sh80000000;
  localparam [31:0] invalid_max = 32'hffffffff;
  localparam signed [39:0] speed_scale = 40'sd100;
  localparam signed [39:0] speed_max = 40'sd2147483647;
  localparam signed [39:0] speed_min = -40'sd2147483648;

  reg [1:0] a_sync_q, b_sync_q;
  reg [1:0] candidate_phase_q, accepted_phase_q;
  reg [2:0] stable_count_q;
  reg [3:0] warm_q;
  reg primed_q;
  wire [1:0] sampled_phase_next = {a_sync_q[1], b_sync_q[1]};
  wire step_positive_next = ({accepted_phase_q, candidate_phase_q} == 4'b0010) ||
      ({accepted_phase_q, candidate_phase_q} == 4'b1011) ||
      ({accepted_phase_q, candidate_phase_q} == 4'b1101) ||
      ({accepted_phase_q, candidate_phase_q} == 4'b0100);
  wire step_negative_next = ({accepted_phase_q, candidate_phase_q} == 4'b0001) ||
      ({accepted_phase_q, candidate_phase_q} == 4'b0111) ||
      ({accepted_phase_q, candidate_phase_q} == 4'b1110) ||
      ({accepted_phase_q, candidate_phase_q} == 4'b1000);

  reg signed [31:0] previous_position_q;
  wire signed [32:0] speed_difference_next =
      $signed({position_q[31], position_q}) -
      $signed({previous_position_q[31], previous_position_q});
  wire signed [39:0] speed_product_next = speed_difference_next * speed_scale;

  // clk_50m域：同步AB输入，rst_n异步清零；后级仅读取同步链末级。
  always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
      a_sync_q <= 2'd0;
      b_sync_q <= 2'd0;
    end else begin
      a_sync_q <= {a_sync_q[0], enc_a};
      b_sync_q <= {b_sync_q[0], enc_b};
    end
  end

  // clk_50m域：共同判稳、建立相位和位置计数；rst_n异步清历史及故障。
  always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
      candidate_phase_q <= 2'd0;
      accepted_phase_q <= 2'd0;
      stable_count_q <= 3'd0;
      warm_q <= 4'd0;
      primed_q <= 1'b0;
      position_q <= 32'sd0;
      invalid_count_q <= 32'd0;
      overflow_fault_q <= 1'b0;
    end else begin
      warm_q <= {warm_q[2:0], 1'b1};
      if (zero) begin
        position_q <= 32'sd0;
        overflow_fault_q <= 1'b0;
        candidate_phase_q <= sampled_phase_next;
        accepted_phase_q <= sampled_phase_next;
        stable_count_q <= 3'd0;
        primed_q <= 1'b0;
      end else if (!warm_q[3]) begin
        // 预热期间跟随同步相位，避免上电初值形成虚假运动。
        candidate_phase_q <= sampled_phase_next;
        accepted_phase_q <= sampled_phase_next;
        stable_count_q <= 3'd0;
      end else if (sampled_phase_next != candidate_phase_q) begin
        candidate_phase_q <= sampled_phase_next;
        stable_count_q <= 3'd1;
      end else if (stable_count_q < stable_cycles) begin
        stable_count_q <= stable_count_q + 3'd1;
      end else if (!primed_q) begin
        accepted_phase_q <= candidate_phase_q;
        primed_q <= 1'b1;
      end else if (candidate_phase_q != accepted_phase_q) begin
        // 非法变化也接收为新基准，同一错误只记一次。
        accepted_phase_q <= candidate_phase_q;
        if (step_positive_next) begin
          if (position_q == position_max) begin
            overflow_fault_q <= 1'b1;
          end else begin
            position_q <= position_q + 32'sd1;
          end
        end else if (step_negative_next) begin
          if (position_q == position_min) begin
            overflow_fault_q <= 1'b1;
          end else begin
            position_q <= position_q - 32'sd1;
          end
        end else if (invalid_count_q != invalid_max) begin
          invalid_count_q <= invalid_count_q + 32'd1;
        end
      end
    end
  end

  // clk_50m域：窗口差分测速；rst_n异步复位，zero同步清历史并优先于tick。
  always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
      previous_position_q <= 32'sd0;
      speed_q <= 32'sd0;
    end else if (zero) begin
      previous_position_q <= 32'sd0;
      speed_q <= 32'sd0;
    end else if (tick_10ms) begin
      previous_position_q <= position_q;
      if (speed_product_next > speed_max) begin
        speed_q <= position_max;
      end else if (speed_product_next < speed_min) begin
        speed_q <= position_min;
      end else begin
        speed_q <= speed_product_next[31:0];
      end
    end
  end
endmodule
