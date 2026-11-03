`timescale 1ns / 1ps
// 定位：将带符号命令转换为方向与PWM；不做低通、斜率限制或闭环控制。
// 时钟复位：clk_50m单域，rst_n低有效异步复位；集成时由顶层同步释放复位。
// 接口：enable和duty按同域输入使用；输出在时钟沿更新，不提供请求握手。
// 兼容：保留旧驱动的换向消隐、自由载波和周期起点更新行为。
// [参数] 周期5000拍，50MHz下PWM为10kHz；消隐同为5000拍即100us。
// [幅值] signed16命令取补码幅值后限255，-32768也正确限制到255。
// [换算] width=floor((level*5000+135)/270)，最大4722拍；不是满占空比。
// [换向] 检出符号变化当沿关闭PWM并重装消隐；消隐期间保持旧方向脚。
// [恢复] 消隐结束后仍等待载波零点装载脉宽；载波不随停止或换向重启。
// [停止] disable或零命令在下一沿关PWM、清方向，每个停止沿重新装满消隐。
module motro (
    input wire clk_50m,
    input wire rst_n,
    input wire enable,
    input wire signed [15:0] duty,
    output reg in1_q,
    output reg in2_q,
    output reg pwm_q
);
  localparam [15:0] pwm_period = 16'd5000;
  localparam [15:0] carrier_last = 16'd4999;
  localparam [15:0] maximum_level = 16'd255;
  localparam [31:0] width_rounding = 32'd135;
  localparam [31:0] width_divisor = 32'd270;

  reg [15:0] carrier_count_q;
  reg [15:0] blank_count_q;
  reg active_sign_q;
  reg [31:0] pulse_width_q;
  wire desired_sign_next = duty[15];
  wire stopped_next = !enable || duty == 16'sd0;
  // 补码取负保留16位无符号幅值，避免-32768被误判为负幅值。
  wire [15:0] magnitude_next = duty[15] ? (~$unsigned(duty) + 16'd1) : $unsigned(duty);
  wire [7:0] limited_level_next =
      magnitude_next > maximum_level ? maximum_level[7:0] : magnitude_next[7:0];

  function [31:0] normalized_width;
    input [7:0] level;
    begin
      // 原查表对含X/Z的level返回0；保留该仿真语义，合法幅值统一执行公式。
      if ((^level) === 1'bx) begin
        normalized_width = 32'd0;
      end else begin
        // 商最大4722；32位运算上下文保留常量除法的完整结果。
        normalized_width = ({24'd0, level} * {16'd0, pwm_period} + width_rounding) /
            width_divisor;
      end
    end
  endfunction
  wire [31:0] requested_width_next = normalized_width(limited_level_next);
  wire pwm_allowed_next = !stopped_next && blank_count_q == 16'd0 &&
      desired_sign_next == active_sign_q;

  // clk_50m域：自由运行载波；rst_n异步清零，停止和换向均不改变相位。
  always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
      carrier_count_q <= 16'd0;
    end else begin
      carrier_count_q <= carrier_count_q == carrier_last ? 16'd0 : carrier_count_q + 16'd1;
    end
  end

  // clk_50m域：消隐、有效符号和周期脉宽；rst_n异步置满消隐并清脉宽。
  always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
      blank_count_q <= pwm_period;
      active_sign_q <= 1'b0;
      pulse_width_q <= 32'd0;
    end else if (stopped_next || desired_sign_next != active_sign_q) begin
      active_sign_q <= desired_sign_next;
      blank_count_q <= pwm_period;
      pulse_width_q <= 32'd0;
    end else if (blank_count_q != 16'd0) begin
      blank_count_q <= blank_count_q - 16'd1;
      pulse_width_q <= 32'd0;
    end else if (carrier_count_q == 16'd0) begin
      pulse_width_q <= requested_width_next;
    end
  end

  // clk_50m域：方向脚；rst_n异步清零，换向和消隐期间保持，不提前翻转。
  always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
      in1_q <= 1'b0;
      in2_q <= 1'b0;
    end else if (stopped_next) begin
      in1_q <= 1'b0;
      in2_q <= 1'b0;
    end else if (blank_count_q == 16'd0 && desired_sign_next == active_sign_q) begin
      in1_q <= !active_sign_q;
      in2_q <= active_sign_q;
    end
  end

  // clk_50m域：PWM；rst_n异步关断，禁止条件当沿覆盖输出，不等载波结束。
  always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
      pwm_q <= 1'b0;
    end else if (!pwm_allowed_next) begin
      pwm_q <= 1'b0;
    end else if (carrier_count_q == 16'd0) begin
      // 旧实现周期>=270时，允许输出的周期起点固定为高；保留这一沿的行为。
      pwm_q <= 1'b1;
    end else begin
      pwm_q <= {16'd0, carrier_count_q} < pulse_width_q;
    end
  end
endmodule
