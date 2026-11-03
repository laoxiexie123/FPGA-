`timescale 1ns / 1ps
// 保留旧显示器逐拍行为：以下未用位/占位状态属于既有渲染实现，不影响控制。
/* verilator lint_off UNUSEDSIGNAL */
/* verilator lint_off UNUSEDPARAM */
/* verilator lint_off DECLFILENAME */

// 定位：帧快照、字符与图形渲染到 128×160 LCD；不发控制命令或运行许可。
// 时钟/复位：clk 单域，rst_n 异步低复位；输入按同域可信使用，无快照握手。
// 可信：内部 tx_start_q 请求后等 tx_done_next；无外部 ready，不因显示阻塞控制。
// legacy：保留初始化 ROM、x+2/y+1 窗口偏移及无 TE 的连续刷帧；不保证无撕裂。
// [param]rule：power_wait_cycles=15000000，reset_wait_cycles=sleep_wait_cycles=6000000。
// [legal]rule：等待参数为正 integer 拍数；32 位 delay_count_q 以 >=参数-1 退出等待。
// [frame]rule：128*160=20480 像素，15 位 pixel_index_q 于 20479 请求低字节后结束帧。
// [text]rule：16 行×21 字符，每字符 8 位；光栅格 6×10，字形仅 gx<5、1<=gy<=7 可见。
// [bcd]rule：32 位幅值转 40 位 BCD 共 32 次；每半字节>=5 先加 3，再移一位。
// [gain]rule：合法 BCD 的 digits[39:28]!=0 即值>=10000000，七字符字段返回横线。
// [phase]rule：32 位补码幅值先模 1040；负且余数非零转 1040-r，再 floor(r*32/1040)。
// [state]rule：state_pix_row=20 仅声明、没有 case；落入 default 回 state_wait_pwr，不虚构行状态。
// [pixel]rule：低字节请求沿预推进坐标；其 SPI 完成后才进入下一像素准备或 state_snap。
module ui_lcd #(
    parameter integer power_wait_cycles = 15000000,
    parameter integer reset_wait_cycles = 6000000,
    parameter integer sleep_wait_cycles = 6000000
) (
    input wire clk,
    input wire rst_n,
    input wire [2:0] page,
    input wire [1:0] selection,
    input wire [1:0] status,
    input wire position_active,
    input wire position_mode,
    input wire [15:0] adc_raw,
    input wire signed [31:0] position,
    target_offset,
    actual_offset,
    input wire signed [15:0] pwm,
    input wire [31:0] zero_x4,
    input wire [7:0] fault,
    input wire signed [31:0] angle_cdeg,
    input wire angle_valid,
    input wire [31:0] angle_kp,
    angle_kd,
    position_kp,
    position_kd,
    output wire lcd_sclk_next,
    output wire lcd_mosi_next,
    output wire lcd_dc_next
);
  localparam [15:0] black = 16'h0000, white = 16'hffff, navy = 16'h0010, cyan = 16'h07ff,
      green = 16'h07e0, blue = 16'h001f, red = 16'hf800;
  localparam [4:0] state_wait_pwr = 0, state_swr = 1, state_wait_rst = 2, state_init = 3,
      state_wait_sleep = 4, state_snap = 5, state_num_start = 6, state_num_run = 7, state_window =
      8, state_pixel = 9, state_wait_byte = 10, state_ph_start = 11, state_ph_bits = 12,
      state_ph_sign = 13, state_ph_div = 14, state_ph_store = 15, state_pix_prep = 16,
      state_pix_char = 17, state_pix_glyph = 18, state_pix_color = 19, state_pix_row = 20;
  reg [4:0] state_q, return_state_q;
  reg [31:0] delay_count_q;
  reg [6:0] init_idx_q;
  reg [3:0] win_idx_q;
  reg [14:0] pixel_index_q;
  reg low_byte_q;
  reg [15:0] pixel_latch_q;
  reg [7:0] tx_byte_q;
  reg tx_dc_q, tx_start_q;
  wire tx_busy_next, tx_done_next;
  ui_lcd_spi u_spi (
      .clk(clk),
      .rst_n(rst_n),
      .start(tx_start_q),
      .tx_byte(tx_byte_q),
      .dc_level(tx_dc_q),
      .sclk_q(lcd_sclk_next),
      .mosi_q(lcd_mosi_next),
      .dc_q(lcd_dc_next),
      .busy_q(tx_busy_next),
      .done_q(tx_done_next)
  );
  reg [2:0] snap_page_q;
  reg [1:0] snap_selection_q, snap_status_q;
  reg snap_active_q, snap_angle_valid_q;
  reg snap_position_mode_q;
  reg signed [31:0] snap_angle_q, snap_position_q, snap_target_q, snap_actual_q;
  reg signed [15:0] snap_pwm_q;
  reg [15:0] snap_adc_q;
  reg [31:0] snap_zero_q;
  reg [7:0] snap_fault_q;
  reg [167:0] text_rows_q[0:15];
  reg [31:0] snap_angle_kp_q, snap_angle_kd_q, snap_position_kp_q, snap_position_kd_q;
  reg [3:0] number_index_q;
  reg [5:0] number_bit_q;
  reg [31:0] magnitude_q;
  reg negative_q;
  reg [39:0] bcd_q;
  reg [39:0] bcd_adjust_next;
  integer k_next, j_next;
  `include "ui_font.vh"

  function [9:0] init_rom;
    input [6:0] n;
    begin
      case (n)
        // 索引 0：同拍组合返回命令，字节 0x11；无寄存等待。
        0: begin
          init_rom = {2'b00, 8'h11};
        end
        // 索引 1：同拍组合返回退出休眠等待标记，字节 0x00；无寄存等待。
        1: begin
          init_rom = {2'b10, 8'h00};
        end
        // 索引 2：同拍组合返回命令，字节 0xb1；无寄存等待。
        2: begin
          init_rom = {2'b00, 8'hb1};
        end
        // 索引 3：同拍组合返回数据，字节 0x05；无寄存等待。
        3: begin
          init_rom = {2'b01, 8'h05};
        end
        // 索引 4：同拍组合返回数据，字节 0x3c；无寄存等待。
        4: begin
          init_rom = {2'b01, 8'h3c};
        end
        // 索引 5：同拍组合返回数据，字节 0x3c；无寄存等待。
        5: begin
          init_rom = {2'b01, 8'h3c};
        end
        // 索引 6：同拍组合返回命令，字节 0xb2；无寄存等待。
        6: begin
          init_rom = {2'b00, 8'hb2};
        end
        // 索引 7：同拍组合返回数据，字节 0x05；无寄存等待。
        7: begin
          init_rom = {2'b01, 8'h05};
        end
        // 索引 8：同拍组合返回数据，字节 0x3c；无寄存等待。
        8: begin
          init_rom = {2'b01, 8'h3c};
        end
        // 索引 9：同拍组合返回数据，字节 0x3c；无寄存等待。
        9: begin
          init_rom = {2'b01, 8'h3c};
        end
        // 索引 10：同拍组合返回命令，字节 0xb3；无寄存等待。
        10: begin
          init_rom = {2'b00, 8'hb3};
        end
        // 索引 11：同拍组合返回数据，字节 0x05；无寄存等待。
        11: begin
          init_rom = {2'b01, 8'h05};
        end
        // 索引 12：同拍组合返回数据，字节 0x3c；无寄存等待。
        12: begin
          init_rom = {2'b01, 8'h3c};
        end
        // 索引 13：同拍组合返回数据，字节 0x3c；无寄存等待。
        13: begin
          init_rom = {2'b01, 8'h3c};
        end
        // 索引 14：同拍组合返回数据，字节 0x05；无寄存等待。
        14: begin
          init_rom = {2'b01, 8'h05};
        end
        // 索引 15：同拍组合返回数据，字节 0x3c；无寄存等待。
        15: begin
          init_rom = {2'b01, 8'h3c};
        end
        // 索引 16：同拍组合返回数据，字节 0x3c；无寄存等待。
        16: begin
          init_rom = {2'b01, 8'h3c};
        end
        // 索引 17：同拍组合返回命令，字节 0xb4；无寄存等待。
        17: begin
          init_rom = {2'b00, 8'hb4};
        end
        // 索引 18：同拍组合返回数据，字节 0x03；无寄存等待。
        18: begin
          init_rom = {2'b01, 8'h03};
        end
        // 索引 19：同拍组合返回命令，字节 0xc0；无寄存等待。
        19: begin
          init_rom = {2'b00, 8'hc0};
        end
        // 索引 20：同拍组合返回数据，字节 0x28；无寄存等待。
        20: begin
          init_rom = {2'b01, 8'h28};
        end
        // 索引 21：同拍组合返回数据，字节 0x08；无寄存等待。
        21: begin
          init_rom = {2'b01, 8'h08};
        end
        // 索引 22：同拍组合返回数据，字节 0x04；无寄存等待。
        22: begin
          init_rom = {2'b01, 8'h04};
        end
        // 索引 23：同拍组合返回命令，字节 0xc1；无寄存等待。
        23: begin
          init_rom = {2'b00, 8'hc1};
        end
        // 索引 24：同拍组合返回数据，字节 0xc0；无寄存等待。
        24: begin
          init_rom = {2'b01, 8'hc0};
        end
        // 索引 25：同拍组合返回命令，字节 0xc2；无寄存等待。
        25: begin
          init_rom = {2'b00, 8'hc2};
        end
        // 索引 26：同拍组合返回数据，字节 0x0d；无寄存等待。
        26: begin
          init_rom = {2'b01, 8'h0d};
        end
        // 索引 27：同拍组合返回数据，字节 0x00；无寄存等待。
        27: begin
          init_rom = {2'b01, 8'h00};
        end
        // 索引 28：同拍组合返回命令，字节 0xc3；无寄存等待。
        28: begin
          init_rom = {2'b00, 8'hc3};
        end
        // 索引 29：同拍组合返回数据，字节 0x8d；无寄存等待。
        29: begin
          init_rom = {2'b01, 8'h8d};
        end
        // 索引 30：同拍组合返回数据，字节 0x2a；无寄存等待。
        30: begin
          init_rom = {2'b01, 8'h2a};
        end
        // 索引 31：同拍组合返回命令，字节 0xc4；无寄存等待。
        31: begin
          init_rom = {2'b00, 8'hc4};
        end
        // 索引 32：同拍组合返回数据，字节 0x8d；无寄存等待。
        32: begin
          init_rom = {2'b01, 8'h8d};
        end
        // 索引 33：同拍组合返回数据，字节 0xee；无寄存等待。
        33: begin
          init_rom = {2'b01, 8'hee};
        end
        // 索引 34：同拍组合返回命令，字节 0xc5；无寄存等待。
        34: begin
          init_rom = {2'b00, 8'hc5};
        end
        // 索引 35：同拍组合返回数据，字节 0x1a；无寄存等待。
        35: begin
          init_rom = {2'b01, 8'h1a};
        end
        // 索引 36：同拍组合返回命令，字节 0x36；无寄存等待。
        36: begin
          init_rom = {2'b00, 8'h36};
        end
        // 索引 37：同拍组合返回数据，字节 0x00；无寄存等待。
        37: begin
          init_rom = {2'b01, 8'h00};
        end
        // 索引 38：同拍组合返回命令，字节 0xe0；无寄存等待。
        38: begin
          init_rom = {2'b00, 8'he0};
        end
        // 索引 39：同拍组合返回数据，字节 0x04；无寄存等待。
        39: begin
          init_rom = {2'b01, 8'h04};
        end
        // 索引 40：同拍组合返回数据，字节 0x22；无寄存等待。
        40: begin
          init_rom = {2'b01, 8'h22};
        end
        // 索引 41：同拍组合返回数据，字节 0x07；无寄存等待。
        41: begin
          init_rom = {2'b01, 8'h07};
        end
        // 索引 42：同拍组合返回数据，字节 0x0a；无寄存等待。
        42: begin
          init_rom = {2'b01, 8'h0a};
        end
        // 索引 43：同拍组合返回数据，字节 0x2e；无寄存等待。
        43: begin
          init_rom = {2'b01, 8'h2e};
        end
        // 索引 44：同拍组合返回数据，字节 0x30；无寄存等待。
        44: begin
          init_rom = {2'b01, 8'h30};
        end
        // 索引 45：同拍组合返回数据，字节 0x25；无寄存等待。
        45: begin
          init_rom = {2'b01, 8'h25};
        end
        // 索引 46：同拍组合返回数据，字节 0x2a；无寄存等待。
        46: begin
          init_rom = {2'b01, 8'h2a};
        end
        // 索引 47：同拍组合返回数据，字节 0x28；无寄存等待。
        47: begin
          init_rom = {2'b01, 8'h28};
        end
        // 索引 48：同拍组合返回数据，字节 0x26；无寄存等待。
        48: begin
          init_rom = {2'b01, 8'h26};
        end
        // 索引 49：同拍组合返回数据，字节 0x2e；无寄存等待。
        49: begin
          init_rom = {2'b01, 8'h2e};
        end
        // 索引 50：同拍组合返回数据，字节 0x3a；无寄存等待。
        50: begin
          init_rom = {2'b01, 8'h3a};
        end
        // 索引 51：同拍组合返回数据，字节 0x00；无寄存等待。
        51: begin
          init_rom = {2'b01, 8'h00};
        end
        // 索引 52：同拍组合返回数据，字节 0x01；无寄存等待。
        52: begin
          init_rom = {2'b01, 8'h01};
        end
        // 索引 53：同拍组合返回数据，字节 0x03；无寄存等待。
        53: begin
          init_rom = {2'b01, 8'h03};
        end
        // 索引 54：同拍组合返回数据，字节 0x13；无寄存等待。
        54: begin
          init_rom = {2'b01, 8'h13};
        end
        // 索引 55：同拍组合返回命令，字节 0xe1；无寄存等待。
        55: begin
          init_rom = {2'b00, 8'he1};
        end
        // 索引 56：同拍组合返回数据，字节 0x04；无寄存等待。
        56: begin
          init_rom = {2'b01, 8'h04};
        end
        // 索引 57：同拍组合返回数据，字节 0x16；无寄存等待。
        57: begin
          init_rom = {2'b01, 8'h16};
        end
        // 索引 58：同拍组合返回数据，字节 0x06；无寄存等待。
        58: begin
          init_rom = {2'b01, 8'h06};
        end
        // 索引 59：同拍组合返回数据，字节 0x0d；无寄存等待。
        59: begin
          init_rom = {2'b01, 8'h0d};
        end
        // 索引 60：同拍组合返回数据，字节 0x2d；无寄存等待。
        60: begin
          init_rom = {2'b01, 8'h2d};
        end
        // 索引 61：同拍组合返回数据，字节 0x26；无寄存等待。
        61: begin
          init_rom = {2'b01, 8'h26};
        end
        // 索引 62：同拍组合返回数据，字节 0x23；无寄存等待。
        62: begin
          init_rom = {2'b01, 8'h23};
        end
        // 索引 63：同拍组合返回数据，字节 0x27；无寄存等待。
        63: begin
          init_rom = {2'b01, 8'h27};
        end
        // 索引 64：同拍组合返回数据，字节 0x27；无寄存等待。
        64: begin
          init_rom = {2'b01, 8'h27};
        end
        // 索引 65：同拍组合返回数据，字节 0x25；无寄存等待。
        65: begin
          init_rom = {2'b01, 8'h25};
        end
        // 索引 66：同拍组合返回数据，字节 0x2d；无寄存等待。
        66: begin
          init_rom = {2'b01, 8'h2d};
        end
        // 索引 67：同拍组合返回数据，字节 0x3b；无寄存等待。
        67: begin
          init_rom = {2'b01, 8'h3b};
        end
        // 索引 68：同拍组合返回数据，字节 0x00；无寄存等待。
        68: begin
          init_rom = {2'b01, 8'h00};
        end
        // 索引 69：同拍组合返回数据，字节 0x01；无寄存等待。
        69: begin
          init_rom = {2'b01, 8'h01};
        end
        // 索引 70：同拍组合返回数据，字节 0x04；无寄存等待。
        70: begin
          init_rom = {2'b01, 8'h04};
        end
        // 索引 71：同拍组合返回数据，字节 0x13；无寄存等待。
        71: begin
          init_rom = {2'b01, 8'h13};
        end
        // 索引 72：同拍组合返回命令，字节 0x3a；无寄存等待。
        72: begin
          init_rom = {2'b00, 8'h3a};
        end
        // 索引 73：同拍组合返回数据，字节 0x05；无寄存等待。
        73: begin
          init_rom = {2'b01, 8'h05};
        end
        // 索引 74：同拍组合返回命令，字节 0x29；无寄存等待。
        74: begin
          init_rom = {2'b00, 8'h29};
        end
        // 索引 default：同拍组合返回ROM 结束标记，字节 0x00；无寄存等待。
        default: begin
          init_rom = {2'b11, 8'h00};
        end
      endcase
    end
  endfunction
  wire [9:0] init_word_next = init_rom(init_idx_q);
  function [7:0] window_byte;
    input [3:0] n;
    begin
      case (n)
        // 索引 0：同拍组合返回 8'h2a，供窗口发送选择。
        0: begin
          window_byte = 8'h2a;
        end
        // 索引 2：同拍组合返回 8'd2，供窗口发送选择。
        2: begin
          window_byte = 8'd2;
        end
        // 索引 4：同拍组合返回 8'd129，供窗口发送选择。
        4: begin
          window_byte = 8'd129;
        end
        // 索引 5：同拍组合返回 8'h2b，供窗口发送选择。
        5: begin
          window_byte = 8'h2b;
        end
        // 索引 7：同拍组合返回 8'd1，供窗口发送选择。
        7: begin
          window_byte = 8'd1;
        end
        // 索引 9：同拍组合返回 8'd160，供窗口发送选择。
        9: begin
          window_byte = 8'd160;
        end
        // 索引 10：同拍组合返回 8'h2c，供窗口发送选择。
        10: begin
          window_byte = 8'h2c;
        end
        // 索引 default：同拍组合返回 8'd0，供窗口发送选择。
        default: begin
          window_byte = 8'd0;
        end
      endcase
    end
  endfunction

  // 职责：共用 BCD 加三校正；纯组合，无独立时钟/复位，先默认复制全部位。
  always @* begin
    bcd_adjust_next = bcd_q;
    for (k_next = 0; k_next < 10; k_next = k_next + 1) begin
      if (bcd_q[k_next*4+:4] >= 5) begin
        bcd_adjust_next[k_next*4+:4] = bcd_q[k_next*4+:4] + 4'd3;
      end
    end
  end
  // [format]rule：11 字符右对齐，前导零改空格但个位始终输出；负号紧邻首个数字。
  function [87:0] decimal_text;
    input [39:0] digits;
    input neg;
    integer d_next;
    reg begun_next;
    reg [87:0] s_next;
    begin
      s_next = {11{8'h20}};
      begun_next = 1'd0;
      for (d_next = 9; d_next >= 0; d_next = d_next - 1) begin
        if (digits[d_next*4+:4] != 0 || begun_next || d_next == 0) begin
          if (!begun_next && neg) begin
            s_next[(d_next+1)*8+:8] = "-";
          end
          s_next[d_next*8+:8] = 8'd48 + {4'd0, digits[d_next*4+:4]};
          begun_next = 1'd1;
        end
      end
      decimal_text = s_next;
    end
  endfunction

  // [format]rule：仅保留低七字符；合法 BCD 值>=10000000 时整字段显示 -------。
  function [55:0] gain_text;
    input [39:0] digits;
    reg [87:0] full_text_next;
    begin
      full_text_next = decimal_text(digits, 1'b0);
      gain_text = (digits[39:28] != 0) ? "-------" : full_text_next[55:0];
    end
  endfunction
  reg signed [32:0] number_value_next;

  // 职责：按 number_index_q 选择 33 位数值；纯组合，无独立复位，default 选择故障码。
  always @* begin
    case (number_index_q)
      // 同拍选择有符号角度 snap_angle_q为转换源；不锁存。
      0: begin
        number_value_next = {snap_angle_q[31], snap_angle_q};
      end
      // 同拍选择有符号 16 位 PWM 符号扩展为转换源；不锁存。
      1: begin
        number_value_next = {{17{snap_pwm_q[15]}}, snap_pwm_q};
      end
      // 同拍选择有符号位置 snap_position_q为转换源；不锁存。
      2: begin
        number_value_next = {snap_position_q[31], snap_position_q};
      end
      // 同拍选择有符号目标 snap_target_q为转换源；不锁存。
      3: begin
        number_value_next = {snap_target_q[31], snap_target_q};
      end
      // 同拍选择无符号 ADC 零扩展为转换源；不锁存。
      4: begin
        number_value_next = {17'd0, snap_adc_q};
      end
      // 同拍选择无符号零点 snap_zero_q为转换源；不锁存。
      5: begin
        number_value_next = {1'b0, snap_zero_q};
      end
      // 同拍选择无符号角度 KP为转换源；不锁存。
      7: begin
        number_value_next = {1'b0, snap_angle_kp_q};
      end
      // 同拍选择无符号角度 KD为转换源；不锁存。
      8: begin
        number_value_next = {1'b0, snap_angle_kd_q};
      end
      // 同拍选择无符号位置 KP为转换源；不锁存。
      9: begin
        number_value_next = {1'b0, snap_position_kp_q};
      end
      // 同拍选择无符号位置 KD为转换源；不锁存。
      10: begin
        number_value_next = {1'b0, snap_position_kd_q};
      end
      // 同拍选择无符号故障码并零扩至 33 位；无需下一拍。
      default: begin
        number_value_next = {25'd0, snap_fault_q};
      end
    endcase
  end

  reg phase_actual_q, phase_negative_q;
  reg  [31:0] phase_magnitude_q;
  reg  [ 5:0] phase_bit_q;
  reg  [10:0] phase_remainder_q;
  reg  [15:0] phase_scaled_q;
  reg  [ 4:0] phase_bin_q;
  wire [11:0] phase_trial_next = {phase_remainder_q, phase_magnitude_q[31]};
  wire [31:0] phase_source_next = phase_actual_q ? snap_actual_q : snap_target_q;
  function signed [6:0] sine20;
    input [4:0] p;
    begin
      case (p)
        // 相位 0, 16：同拍组合返回 7'sd0，不额外延迟。
        0, 16: begin
          sine20 = 7'sd0;
        end
        // 相位 1, 15：同拍组合返回 7'sd4，不额外延迟。
        1, 15: begin
          sine20 = 7'sd4;
        end
        // 相位 2, 14：同拍组合返回 7'sd8，不额外延迟。
        2, 14: begin
          sine20 = 7'sd8;
        end
        // 相位 3, 13：同拍组合返回 7'sd11，不额外延迟。
        3, 13: begin
          sine20 = 7'sd11;
        end
        // 相位 4, 12：同拍组合返回 7'sd14，不额外延迟。
        4, 12: begin
          sine20 = 7'sd14;
        end
        // 相位 5, 11：同拍组合返回 7'sd17，不额外延迟。
        5, 11: begin
          sine20 = 7'sd17;
        end
        // 相位 6, 10：同拍组合返回 7'sd18，不额外延迟。
        6, 10: begin
          sine20 = 7'sd18;
        end
        // 相位 7, 9：同拍组合返回 7'sd20，不额外延迟。
        7, 9: begin
          sine20 = 7'sd20;
        end
        // 相位 8：同拍组合返回 7'sd20，不额外延迟。
        8: begin
          sine20 = 7'sd20;
        end
        // 相位 17, 31：同拍组合返回 -7'sd4，不额外延迟。
        17, 31: begin
          sine20 = -7'sd4;
        end
        // 相位 18, 30：同拍组合返回 -7'sd8，不额外延迟。
        18, 30: begin
          sine20 = -7'sd8;
        end
        // 相位 19, 29：同拍组合返回 -7'sd11，不额外延迟。
        19, 29: begin
          sine20 = -7'sd11;
        end
        // 相位 20, 28：同拍组合返回 -7'sd14，不额外延迟。
        20, 28: begin
          sine20 = -7'sd14;
        end
        // 相位 21, 27：同拍组合返回 -7'sd17，不额外延迟。
        21, 27: begin
          sine20 = -7'sd17;
        end
        // 相位 22, 26：同拍组合返回 -7'sd18，不额外延迟。
        22, 26: begin
          sine20 = -7'sd18;
        end
        // 相位 23, 25：同拍组合返回 -7'sd20，不额外延迟。
        23, 25: begin
          sine20 = -7'sd20;
        end
        // 相位 24：同拍组合返回 -7'sd20，不额外延迟。
        24: begin
          sine20 = -7'sd20;
        end
        // 相位 default：同拍组合返回 7'sd0，不额外延迟。
        default: begin
          sine20 = 7'sd0;
        end
      endcase
    end
  endfunction
  reg signed [8:0] target_x_q, target_y_q, actual_x_q, actual_y_q;

  function [9:0] ring_span;
    input [7:0] ay;
    begin
      case (ay)
        // 绝对纵距 0：同拍返回 {5'd19, 5'd21} 的内/外横距闭区间。
        0: begin
          ring_span = {5'd19, 5'd21};
        end
        // 绝对纵距 1, 2, 3, 4, 5, 6：同拍返回 {5'd19, 5'd20} 的内/外横距闭区间。
        1, 2, 3, 4, 5, 6: begin
          ring_span = {5'd19, 5'd20};
        end
        // 绝对纵距 7, 8：同拍返回 {5'd18, 5'd19} 的内/外横距闭区间。
        7, 8: begin
          ring_span = {5'd18, 5'd19};
        end
        // 绝对纵距 9, 10：同拍返回 {5'd17, 5'd18} 的内/外横距闭区间。
        9, 10: begin
          ring_span = {5'd17, 5'd18};
        end
        // 绝对纵距 11：同拍返回 {5'd16, 5'd17} 的内/外横距闭区间。
        11: begin
          ring_span = {5'd16, 5'd17};
        end
        // 绝对纵距 12：同拍返回 {5'd15, 5'd17} 的内/外横距闭区间。
        12: begin
          ring_span = {5'd15, 5'd17};
        end
        // 绝对纵距 13：同拍返回 {5'd14, 5'd16} 的内/外横距闭区间。
        13: begin
          ring_span = {5'd14, 5'd16};
        end
        // 绝对纵距 14：同拍返回 {5'd13, 5'd15} 的内/外横距闭区间。
        14: begin
          ring_span = {5'd13, 5'd15};
        end
        // 绝对纵距 15：同拍返回 {5'd12, 5'd14} 的内/外横距闭区间。
        15: begin
          ring_span = {5'd12, 5'd14};
        end
        // 绝对纵距 16：同拍返回 {5'd11, 5'd13} 的内/外横距闭区间。
        16: begin
          ring_span = {5'd11, 5'd13};
        end
        // 绝对纵距 17：同拍返回 {5'd9, 5'd12} 的内/外横距闭区间。
        17: begin
          ring_span = {5'd9, 5'd12};
        end
        // 绝对纵距 18：同拍返回 {5'd7, 5'd10} 的内/外横距闭区间。
        18: begin
          ring_span = {5'd7, 5'd10};
        end
        // 绝对纵距 19：同拍返回 {5'd0, 5'd8} 的内/外横距闭区间。
        19: begin
          ring_span = {5'd0, 5'd8};
        end
        // 绝对纵距 20：同拍返回 {5'd0, 5'd6} 的内/外横距闭区间。
        20: begin
          ring_span = {5'd0, 5'd6};
        end
        // 绝对纵距 21：同拍返回 {5'd0, 5'd0} 的内/外横距闭区间。
        21: begin
          ring_span = {5'd0, 5'd0};
        end
        // 绝对纵距 default：同拍返回 {5'd31, 5'd0} 的内/外横距闭区间。
        default: begin
          ring_span = {5'd31, 5'd0};
        end
      endcase
    end
  endfunction
  function target_dot;
    input [7:0] ax, ay;
    begin
      case (ay)
        // 绝对纵距 0：同拍返回条件 ax <= 3，不引入状态。
        0: begin
          target_dot = ax <= 3;
        end
        // 绝对纵距 1, 2：同拍返回条件 ax <= 2，不引入状态。
        1, 2: begin
          target_dot = ax <= 2;
        end
        // 绝对纵距 3：同拍返回条件 ax == 0，不引入状态。
        3: begin
          target_dot = ax == 0;
        end
        // 绝对纵距 default：同拍返回条件 1'd0，不引入状态。
        default: begin
          target_dot = 1'd0;
        end
      endcase
    end
  endfunction
  function actual_ring;
    input [7:0] ax, ay;
    begin
      case (ay)
        // 绝对纵距 0：同拍返回条件 ax >= 4 && ax <= 5，不引入状态。
        0: begin
          actual_ring = ax >= 4 && ax <= 5;
        end
        // 绝对纵距 1, 2：同拍返回条件 ax == 4，不引入状态。
        1, 2: begin
          actual_ring = ax == 4;
        end
        // 绝对纵距 3：同拍返回条件 ax >= 3 && ax <= 4，不引入状态。
        3: begin
          actual_ring = ax >= 3 && ax <= 4;
        end
        // 绝对纵距 4：同拍返回条件 ax <= 3，不引入状态。
        4: begin
          actual_ring = ax <= 3;
        end
        // 绝对纵距 5：同拍返回条件 ax == 0，不引入状态。
        5: begin
          actual_ring = ax == 0;
        end
        // 绝对纵距 default：同拍返回条件 1'd0，不引入状态。
        default: begin
          actual_ring = 1'd0;
        end
      endcase
    end
  endfunction
  function [7:0] abs_diff;
    input [7:0] a, b;
    begin
      abs_diff = a >= b ? a - b : b - a;
    end
  endfunction
  reg [  6:0] draw_x_q;
  reg [  7:0] draw_y_q;
  reg [  4:0] text_col_q;
  reg [  3:0] text_row_q;
  reg [  2:0] glyph_x_q;
  reg [  3:0] glyph_y_q;
  reg [167:0] row_latch_q;
  reg [  7:0] char_latch_q;
  reg [ 34:0] glyph_latch_q;
  reg [7:0] dx_abs_q, dy_abs_q, tdx_abs_q, tdy_abs_q, adx_abs_q, ady_abs_q;
  reg [9:0] span_latch_q;
  reg circle_latch_q, selected_latch_q, header_latch_q;
  reg [5:0] glyph_bit_q;
  reg glyph_visible_q;
  reg [15:0] final_color_next;
  reg selected_comb_next;

  // 职责：菜单选择与像素优先级着色；纯组合，无独立复位，默认背景后依次覆盖。
  always @* begin
    selected_comb_next = 1'd0;
    case (snap_page_q)
      // 主菜单：同拍选择行 4/7；其余像素继续按颜色优先级合成。
      0: begin
        selected_comb_next = (text_row_q == 4 && snap_selection_q == 0) ||
            (text_row_q == 7 && snap_selection_q == 1);
      end
      // 确认页：同拍选择行 6/9；其余像素继续按颜色优先级合成。
      1: begin
        selected_comb_next = (text_row_q == 6 && snap_selection_q == 0) ||
            (text_row_q == 9 && snap_selection_q == 1);
      end
      // 运行页：同拍选择行 9；其余像素继续按颜色优先级合成。
      2: begin
        selected_comb_next = snap_position_mode_q ? text_row_q == 9 :
             ((text_row_q == 9 && snap_selection_q == 0) ||
              (text_row_q == 10 && snap_selection_q == 1));
      end
      // 校准页：同拍选择行 6/9/12；其余像素继续按颜色优先级合成。
      3: begin
        selected_comb_next = (text_row_q == 6 && snap_selection_q == 0) ||
            (text_row_q == 9 && snap_selection_q == 1) ||
            (text_row_q == 12 && snap_selection_q == 2);
      end
      // 其余页：同拍不选菜单行，后续仍可按标题/字形合成。
      default: begin
        selected_comb_next = 1'd0;
      end
    endcase
    final_color_next = header_latch_q ? ((snap_page_q == 4) ? red : navy) : black;
    if (selected_latch_q) begin
      final_color_next = cyan;
    end
    if (glyph_visible_q && glyph_latch_q[glyph_bit_q]) begin
      final_color_next = selected_latch_q ? black : white;
    end
    // V3.2按要求移除圆形位置图；像素仅由背景、菜单高亮和文字合成。
  end
  wire [11:0] phase_reduced_next = phase_trial_next >= 12'd1040 ? phase_trial_next - 12'd1040 :
      phase_trial_next;
  wire signed [6:0] phase_sine_next = sine20(phase_bin_q);
  wire signed [6:0] phase_cosine_next = sine20(phase_bin_q + 5'd8);
  wire signed [32:0]
      number_magnitude_next = number_value_next < 0 ? -number_value_next : number_value_next;
  wire [31:0]
      glyph_index_next = 32'd34 - (({28'd0, glyph_y_q} - 32'd1) * 32'd5 + {29'd0, glyph_x_q});

  // 仿真参考保留乘法、除法与平方距离；综合硬件使用逐拍光栅、跨度查表及流水。
  // 参考函数同拍组合返回；坐标非负除法等于 floor，不能把 signed 除法统称 floor。
  // Verilog signed 整数除法为 trunc_zero；本函数 /6、/10 的被除数为非负坐标。
  // synthesis translate_off
  function [15:0] render_pixel;
    input [6:0] x;
    input [7:0] y;
    integer
        row_next,
        col_next,
        gx_next,
        gy_next,
        dx_next,
        dy_next,
        r2_next,
        tdx_next,
        tdy_next,
        adx_next,
        ady_next;
    reg selected_next;
    reg [7:0] ch_next;
    reg [34:0] glyph_next;
    reg [15:0] bg_next, fg_next;
    begin
      row_next = {24'd0, y} / 10;
      col_next = {25'd0, x} / 6;
      gx_next = {25'd0, x} % 6;
      gy_next = {24'd0, y} % 10;
      selected_next = 1'd0;
      case (snap_page_q)
        // 参考主菜单：同拍选第 4/7 行高亮，组合返回参考颜色。
        0: begin
          selected_next = (row_next == 4 && snap_selection_q == 0) ||
              (row_next == 7 && snap_selection_q == 1);
        end
        // 参考确认页：同拍选第 6/9 行高亮，组合返回参考颜色。
        1: begin
          selected_next = (row_next == 6 && snap_selection_q == 0) ||
              (row_next == 9 && snap_selection_q == 1);
        end
        // 参考运行页：同拍选第 9 行高亮，组合返回参考颜色。
        2: begin
          selected_next = snap_position_mode_q ? row_next == 9 :
              ((row_next == 9 && snap_selection_q == 0) ||
               (row_next == 10 && snap_selection_q == 1));
        end
        // 参考校准页：同拍选第 6/9/12 行高亮，组合返回参考颜色。
        3: begin
          selected_next = (row_next == 6 && snap_selection_q == 0) ||
              (row_next == 9 && snap_selection_q == 1) || (row_next == 12 && snap_selection_q == 2);
        end
        // 参考其他页：同拍不选菜单高亮，组合继续着色。
        default: begin
          selected_next = 1'd0;
        end
      endcase
      bg_next = black;
      fg_next = white;
      if (row_next == 0) begin
        bg_next = (snap_page_q == 4) ? red : navy;
      end
      if (selected_next) begin
        bg_next = cyan;
        fg_next = black;
      end
      render_pixel = bg_next;
      if (col_next < 21 && row_next < 16 && gx_next < 5 && gy_next >= 1 && gy_next <= 7) begin
        ch_next = text_rows_q[row_next][167-col_next*8-:8];
        glyph_next = ui_glyph(ch_next);
        if (glyph_next[34-((gy_next-1)*5+gx_next)]) begin
          render_pixel = fg_next;
        end
      end
      // 与实际像素流水线一致：不再叠加圆形位置图。
    end
  endfunction
  // synthesis translate_on

  // 职责：初始化、快照、相位/数值转换与像素 FSM；clk 域，异步低复位回 state_wait_pwr。
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state_q <= state_wait_pwr;
      return_state_q <= state_wait_pwr;
      delay_count_q <= 32'd0;
      init_idx_q <= 7'd0;
      win_idx_q <= 4'd0;
      pixel_index_q <= 15'd0;
      low_byte_q <= 1'd0;
      pixel_latch_q <= 16'd0;
      tx_byte_q <= 8'd0;
      tx_dc_q <= 1'd0;
      tx_start_q <= 1'd0;
      snap_page_q <= 3'd0;
      snap_selection_q <= 2'd0;
      snap_status_q <= 2'd0;
      snap_active_q <= 1'd0;
      snap_position_mode_q <= 1'b0;
      snap_angle_valid_q <= 1'd0;
      snap_angle_q <= 32'sd0;
      snap_position_q <= 32'sd0;
      snap_target_q <= 32'sd0;
      snap_actual_q <= 32'sd0;
      snap_pwm_q <= 16'sd0;
      snap_adc_q <= 16'd0;
      snap_zero_q <= 32'd0;
      snap_fault_q <= 8'd0;
      snap_angle_kp_q <= 32'd0;
      snap_angle_kd_q <= 32'd0;
      snap_position_kp_q <= 32'd0;
      snap_position_kd_q <= 32'd0;
      number_index_q <= 4'd0;
      number_bit_q <= 6'd0;
      magnitude_q <= 32'd0;
      negative_q <= 1'd0;
      bcd_q <= 40'd0;
      target_x_q <= 9'sd64;
      target_y_q <= 9'sd113;
      actual_x_q <= 9'sd64;
      actual_y_q <= 9'sd113;
      phase_actual_q <= 1'd0;
      phase_negative_q <= 1'd0;
      phase_magnitude_q <= 32'd0;
      phase_bit_q <= 6'd0;
      phase_remainder_q <= 11'd0;
      phase_scaled_q <= 16'd0;
      phase_bin_q <= 5'd0;
      draw_x_q <= 7'd0;
      draw_y_q <= 8'd0;
      text_col_q <= 5'd0;
      text_row_q <= 4'd0;
      glyph_x_q <= 3'd0;
      glyph_y_q <= 4'd0;
      row_latch_q <= 168'd0;
      char_latch_q <= 8'd0;
      glyph_latch_q <= 35'd0;
      dx_abs_q <= 8'd0;
      dy_abs_q <= 8'd0;
      tdx_abs_q <= 8'd0;
      tdy_abs_q <= 8'd0;
      adx_abs_q <= 8'd0;
      ady_abs_q <= 8'd0;
      span_latch_q <= 10'd0;
      circle_latch_q <= 1'd0;
      selected_latch_q <= 1'd0;
      header_latch_q <= 1'd0;
      glyph_bit_q <= 6'd0;
      glyph_visible_q <= 1'd0;
      for (j_next = 0; j_next < 16; j_next = j_next + 1) begin
        text_rows_q[j_next] <= {21{8'h20}};
      end
    end else begin
      tx_start_q <= 1'd0;
      case (state_q)
        // 本拍到 power_wait_cycles-1 则清等待计数；下拍 state_swr，否则保持 state_wait_pwr。
        state_wait_pwr: begin
          if (delay_count_q >= power_wait_cycles - 1) begin
            delay_count_q <= 32'd0;
            state_q <= state_swr;
          end else begin
            delay_count_q <= delay_count_q + 1'b1;
          end
        end
        // 本拍请求软件复位命令 01；下拍 state_wait_byte，完成后返回 state_wait_rst。
        state_swr: begin
          tx_byte_q <= 8'h01;
          tx_dc_q <= 1'd0;
          tx_start_q <= 1'd1;
          return_state_q <= state_wait_rst;
          state_q <= state_wait_byte;
        end
        // 本拍到 reset_wait_cycles-1 则清等待计数；下拍 state_init，否则保持 state_wait_rst。
        state_wait_rst: begin
          if (delay_count_q >= reset_wait_cycles - 1) begin
            delay_count_q <= 32'd0;
            state_q <= state_init;
          end else begin
            delay_count_q <= delay_count_q + 1'b1;
          end
        end
        // 本拍解释 ROM：终止则下拍 state_snap，延时标记则 state_wait_sleep，字节则 state_wait_byte 后回 state_init。
        state_init: begin
          if (init_word_next[9:8] == 2'b11) begin
            state_q <= state_snap;
          end else if (init_word_next[9:8] == 2'b10) begin
            delay_count_q <= 32'd0;
            init_idx_q <= init_idx_q + 1'b1;
            state_q <= state_wait_sleep;
          end else begin
            tx_byte_q <= init_word_next[7:0];
            tx_dc_q <= init_word_next[8];
            tx_start_q <= 1'd1;
            init_idx_q <= init_idx_q + 1'b1;
            return_state_q <= state_init;
            state_q <= state_wait_byte;
          end
        end
        // 本拍到 sleep_wait_cycles-1 则清等待计数；下拍 state_init，否则保持 state_wait_sleep。
        state_wait_sleep: begin
          if (delay_count_q >= sleep_wait_cycles - 1) begin
            delay_count_q <= 32'd0;
            state_q <= state_init;
          end else begin
            delay_count_q <= delay_count_q + 1'b1;
          end
        end

        // 本拍锁存整帧输入并写静态文本、清行缓冲；下拍 state_ph_start，先计算目标相位。
        state_snap: begin
          snap_page_q <= page;
          snap_selection_q <= selection;
          snap_status_q <= status;
          snap_active_q <= position_active;
          snap_position_mode_q <= position_mode;
          snap_angle_q <= angle_cdeg;
          snap_angle_valid_q <= angle_valid;
          snap_position_q <= position;
          snap_target_q <= target_offset;
          snap_actual_q <= actual_offset;
          snap_pwm_q <= pwm;
          snap_adc_q <= adc_raw;
          snap_zero_q <= zero_x4;
          snap_fault_q <= fault;
          snap_angle_kp_q <= angle_kp;
          snap_angle_kd_q <= angle_kd;
          snap_position_kp_q <= position_kp;
          snap_position_kd_q <= position_kd;
          phase_actual_q <= 1'd0;
          for (j_next = 0; j_next < 16; j_next = j_next + 1) begin
            text_rows_q[j_next] <= {21{8'h20}};
          end
          case (page)
            // 本拍写主菜单静态文本并嵌套选择状态标题；下拍 state_ph_start。
            0: begin
              case (status)
                // 本拍标题写 STATUS SWING；下拍随外层 state_snap 转 state_ph_start。
                1: begin
                  text_rows_q[0] <= "STATUS SWING         ";
                end
                // 本拍标题写 STATUS BALANCE；下拍随外层 state_snap 转 state_ph_start。
                2: begin
                  text_rows_q[0] <= "STATUS BALANCE       ";
                end
                // 本拍标题写 STATUS IDLE；下拍随外层 state_snap 转 state_ph_start。
                default: begin
                  text_rows_q[0] <= "STATUS IDLE          ";
                end
              endcase
              text_rows_q[4] <= "     START SWING     ";
              text_rows_q[7] <= "     CALIBRATION     ";
            end
            // 本拍写确认页与 CANCEL/START 文本；下拍 state_ph_start。
            1: begin
              text_rows_q[0] <= "CONFIRM              ";
              text_rows_q[3] <= "     AREA CLEAR?     ";
              text_rows_q[6] <= "       CANCEL        ";
              text_rows_q[9] <= "       START         ";
            end
            // 本拍写运行页标签及状态标题；下拍 state_ph_start，再补数值和图形。
            2: begin
              text_rows_q[0] <= (status == 1) ? "SWINGING             " : "BALANCING            ";
              text_rows_q[2] <= "ANG CDEG             ";
              text_rows_q[3] <= "PWM                  ";
              text_rows_q[4] <= "ENC                  ";
              text_rows_q[5] <= "TARGET               ";
              text_rows_q[6] <= "KP        KD         ";
              text_rows_q[7] <= "PP        PD         ";
              text_rows_q[9] <= position_mode ? "   STEP +/-45 DEG    " : "    POSITION STEP    ";
              text_rows_q[10] <= position_mode ? "  PRESS TO EXIT      " : "    CW360 4S         ";
            end
            // 本拍写校准页标签；RESTORE DEFAULTS 仅为文字，不在此恢复参数；下拍 state_ph_start。
            3: begin
              text_rows_q[0]  <= "CALIBRATION          ";
              text_rows_q[2]  <= "ADC                  ";
              text_rows_q[3]  <= "ZERO X4              ";
              text_rows_q[6]  <= "     SET UPRIGHT     ";
              text_rows_q[9]  <= "  RESTORE DEFAULTS   ";
              text_rows_q[12] <= "        BACK         ";
            end
            // 本拍写故障页标签和 STOPPED 文本；不在此执行停机；下拍 state_ph_start。
            default: begin
              text_rows_q[0] <= "FAULT                ";
              text_rows_q[2] <= "CODE                 ";
              text_rows_q[5] <= "       STOPPED       ";
            end
          endcase
          number_index_q <= 0;
          state_q <= state_ph_start;
        end
        // 本拍锁存源符号/补码幅值并清相位工作量；下拍 state_ph_bits。
        state_ph_start: begin
          phase_negative_q <= phase_source_next[31];

          phase_magnitude_q <= phase_source_next[31] ? (~phase_source_next + 32'd1) :
              phase_source_next;
          phase_remainder_q <= 11'd0;
          phase_bit_q <= 6'd0;
          phase_bin_q <= 5'd0;
          state_q <= state_ph_bits;
        end
        // 本拍移入源高位并条件减 1040；第 32 位后下拍 state_ph_sign，否则保持。
        state_ph_bits: begin
          phase_remainder_q <= phase_reduced_next[10:0];
          phase_magnitude_q <= {phase_magnitude_q[30:0], 1'b0};
          if (phase_bit_q == 31) begin
            state_q <= state_ph_sign;
          end else begin
            phase_bit_q <= phase_bit_q + 1'b1;
          end
        end
        // 本拍将负余数归一化后乘 32；下拍 state_ph_div，bin 从 0 开始。
        state_ph_sign: begin
          if (phase_negative_q && phase_remainder_q != 0) begin
            phase_scaled_q <= (16'd1040 - {5'd0, phase_remainder_q}) << 5;
          end else begin
            phase_scaled_q <= {phase_remainder_q, 5'b0};
          end
          phase_bin_q <= 5'd0;
          state_q <= state_ph_div;
        end
        // 本拍若 scaled>=1040 则减并累计 bin、保持；否则下拍 state_ph_store。
        state_ph_div: begin
          if (phase_scaled_q >= 16'd1040) begin
            phase_scaled_q <= phase_scaled_q - 16'd1040;
            phase_bin_q <= phase_bin_q + 1'b1;
          end else begin
            state_q <= state_ph_store;
          end
        end
        // 本拍由 sine20 存坐标；目标完成下拍 state_ph_start 处理实际，实际完成则 state_num_start。
        state_ph_store: begin

          if (!phase_actual_q) begin
            target_x_q <= 9'sd64 + {{2{phase_sine_next[6]}}, phase_sine_next};
            target_y_q <= 9'sd133 - {{2{phase_cosine_next[6]}}, phase_cosine_next};
            phase_actual_q <= 1'd1;
            state_q <= state_ph_start;
          end else begin
            actual_x_q <= 9'sd64 + {{2{phase_sine_next[6]}}, phase_sine_next};
            actual_y_q <= 9'sd133 - {{2{phase_cosine_next[6]}}, phase_cosine_next};
            state_q <= state_num_start;
          end
        end
        // 本拍锁存符号与 32 位幅值并清 BCD；下拍 state_num_run。
        state_num_start: begin
          negative_q <= number_value_next < 0;
          magnitude_q <= number_magnitude_next[31:0];
          bcd_q <= 40'd0;
          number_bit_q <= 6'd0;
          state_q <= state_num_run;
        end
        // 本拍未满 32 次则校正移位并保持；满后写字段，下拍 state_num_start 或末项转 state_window。
        state_num_run: begin
          if (number_bit_q == 32) begin
            case (number_index_q)
              // 本拍按页条件写入运行页角度，angle_valid=0 时写 ---；下拍由外层决定下一数值或 state_window。
              0: begin
                if (snap_page_q == 2) begin
                  text_rows_q[2][87:0] <= snap_angle_valid_q ? decimal_text(bcd_q, negative_q) :
                      "        ---";
                end
              end
              // 本拍按页条件写入运行页 PWM；下拍由外层决定下一数值或 state_window。
              1: begin
                if (snap_page_q == 2) begin
                  text_rows_q[3][87:0] <= decimal_text(bcd_q, negative_q);
                end
              end
              // 本拍按页条件写入运行页 ENC；下拍由外层决定下一数值或 state_window。
              2: begin
                if (snap_page_q == 2) begin
                  text_rows_q[4][87:0] <= decimal_text(bcd_q, negative_q);
                end
              end
              // 本拍按页条件写入运行页 TARGET；下拍由外层决定下一数值或 state_window。
              3: begin
                if (snap_page_q == 2) begin
                  text_rows_q[5][87:0] <= decimal_text(bcd_q, negative_q);
                end
              end
              // 本拍按页条件写入校准页 ADC；下拍由外层决定下一数值或 state_window。
              4: begin
                if (snap_page_q == 3) begin
                  text_rows_q[2][87:0] <= decimal_text(bcd_q, negative_q);
                end
              end
              // 本拍按页条件写入校准页 ZERO X4；下拍由外层决定下一数值或 state_window。
              5: begin
                if (snap_page_q == 3) begin
                  text_rows_q[3][87:0] <= decimal_text(bcd_q, negative_q);
                end
              end
              // 本拍按页条件写入故障页 CODE；下拍由外层决定下一数值或 state_window。
              6: begin
                if (snap_page_q >= 4) begin
                  text_rows_q[2][87:0] <= decimal_text(bcd_q, negative_q);
                end
              end
              // 本拍按页条件写入运行页 KP 七字符字段；下拍由外层决定下一数值或 state_window。
              7: begin
                if (snap_page_q == 2) begin
                  text_rows_q[6][151:96] <= gain_text(bcd_q);
                end
              end
              // 本拍按页条件写入运行页 KD 七字符字段；下拍由外层决定下一数值或 state_window。
              8: begin
                if (snap_page_q == 2) begin
                  text_rows_q[6][71:16] <= gain_text(bcd_q);
                end
              end
              // 本拍按页条件写入运行页 PP 七字符字段；下拍由外层决定下一数值或 state_window。
              9: begin
                if (snap_page_q == 2) begin
                  text_rows_q[7][151:96] <= gain_text(bcd_q);
                end
              end
              // 本拍按页条件写入运行页 PD 七字符字段；下拍由外层决定下一数值或 state_window。
              10: begin
                if (snap_page_q == 2) begin
                  text_rows_q[7][71:16] <= gain_text(bcd_q);
                end
              end
            endcase
            if (number_index_q == 10) begin
              win_idx_q <= 4'd0;
              state_q   <= state_window;
            end else begin
              number_index_q <= number_index_q + 1'b1;
              state_q <= state_num_start;
            end
          end else begin
            bcd_q <= {bcd_adjust_next[38:0], magnitude_q[31]};
            magnitude_q <= {magnitude_q[30:0], 1'b0};
            number_bit_q <= number_bit_q + 1'b1;
          end
        end
        // 本拍请求窗口字节；下拍 state_wait_byte，完成后回 state_window，末字节后转 state_pix_prep。
        state_window: begin
          tx_byte_q <= window_byte(win_idx_q);
          tx_dc_q <= !(win_idx_q == 0 || win_idx_q == 5 || win_idx_q == 10);
          tx_start_q <= 1'd1;
          state_q <= state_wait_byte;
          if (win_idx_q == 10) begin
            pixel_index_q <= 15'd0;
            low_byte_q <= 1'd0;
            draw_x_q <= 7'd0;
            draw_y_q <= 8'd0;
            text_col_q <= 5'd0;
            text_row_q <= 4'd0;
            glyph_x_q <= 3'd0;
            glyph_y_q <= 4'd0;
            return_state_q <= state_pix_prep;
          end else begin
            win_idx_q <= win_idx_q + 1'b1;
            return_state_q <= state_window;
          end
        end
        // 本拍锁存字符行、距离及选择/标题标志；下拍 state_pix_char，不在 SPI 等待中取新帧。
        state_pix_prep: begin

          row_latch_q <= text_rows_q[text_row_q];
          dx_abs_q <= abs_diff({1'b0, draw_x_q}, 8'd64);
          dy_abs_q <= abs_diff(draw_y_q, 8'd133);
          tdx_abs_q <= abs_diff({1'b0, draw_x_q}, target_x_q[7:0]);
          tdy_abs_q <= abs_diff(draw_y_q, target_y_q[7:0]);
          adx_abs_q <= abs_diff({1'b0, draw_x_q}, actual_x_q[7:0]);
          ady_abs_q <= abs_diff(draw_y_q, actual_y_q[7:0]);
          circle_latch_q <= 1'b0; // 圆形显示已移除，运行页文字区域不被图形覆盖。
          selected_latch_q <= selected_comb_next;
          header_latch_q <= text_row_q == 0;
          state_q <= state_pix_char;
        end
        // 本拍提取字符、查圆环跨度并锁存可见范围/字形位索引；下拍 state_pix_glyph。
        state_pix_char: begin
          char_latch_q <= row_latch_q[167-text_col_q*8-:8];
          span_latch_q <= ring_span(dy_abs_q);
          glyph_visible_q <= text_col_q < 21 && glyph_x_q < 5 && glyph_y_q >= 1 && glyph_y_q <= 7;
          glyph_bit_q <= glyph_index_next[5:0];
          state_q <= state_pix_glyph;
        end
        // 本拍组合查字形并锁存；下拍 state_pix_color。
        state_pix_glyph: begin
          glyph_latch_q <= ui_glyph(char_latch_q);
          state_q <= state_pix_color;
        end
        // 本拍锁存已算出的 RGB565；下拍 state_pixel，后续两字节使用同一颜色。
        state_pix_color: begin
          pixel_latch_q <= final_color_next;
          state_q <= state_pixel;
        end

        // 本拍请求 RGB565 高/低字节；下拍 state_wait_byte，低字节请求沿预推进，完成后按 return_state_q 返回。
        state_pixel: begin
          tx_dc_q <= 1'd1;
          tx_start_q <= 1'd1;
          state_q <= state_wait_byte;
          return_state_q <= state_pixel;
          if (!low_byte_q) begin
            tx_byte_q  <= pixel_latch_q[15:8];
            low_byte_q <= 1'd1;
          end else begin
            tx_byte_q  <= pixel_latch_q[7:0];
            low_byte_q <= 1'd0;
            if (pixel_index_q == 20479) begin
              return_state_q <= state_snap;
            end else begin
              pixel_index_q  <= pixel_index_q + 1'b1;
              return_state_q <= state_pix_prep;
              if (draw_x_q == 127) begin
                draw_x_q   <= 7'd0;
                draw_y_q   <= draw_y_q + 1'b1;
                text_col_q <= 5'd0;
                glyph_x_q  <= 3'd0;
                if (glyph_y_q == 9) begin
                  glyph_y_q  <= 4'd0;
                  text_row_q <= text_row_q + 1'b1;
                end else begin
                  glyph_y_q <= glyph_y_q + 1'b1;
                end
              end else begin
                draw_x_q <= draw_x_q + 1'b1;
                if (glyph_x_q == 5) begin
                  glyph_x_q  <= 3'd0;
                  text_col_q <= text_col_q + 1'b1;
                end else begin
                  glyph_x_q <= glyph_x_q + 1'b1;
                end
              end
            end
          end
        end
        // 本拍仅当 tx_done_next 有效才选择返回状态；下拍 return_state_q，否则保持。
        state_wait_byte: begin
          if (tx_done_next) begin
            state_q <= return_state_q;
          end
        end
        // 本拍遇未列状态仅改状态；下拍 state_wait_pwr，不假设其他寄存器被复位。
        default: begin
          state_q <= state_wait_pwr;
        end
      endcase
    end
  end
endmodule

// 定位：单字节 SPI mode 0 发送；不控制 CS、不排队、不解释显示协议。
// 时钟/复位：clk 单域，rst_n 异步低复位；SCLK 非独立逻辑时钟。
// 可信：仅 !busy_q 的沿锁存 start、tx_byte、dc_level；busy 时请求忽略。
// legacy：MSB 先行；done 清 busy 的沿仍走忙分支，不能同沿接新字节。
// [width]rule：8 位 shift_q、3 位 bit_index_q；bit7 先建立，末位下降沿置 done_q。
// [clock]rule：half_count_q 每两 clk 拍翻转 SCLK，SCLK=clk/4；50MHz 时为 12.5MHz。
// [setup]rule：接收 start 后首个 SCLK 上升沿间隔 2 拍；50MHz 时 MOSI 建立 40ns。
// [trust]rule：done_q 为单拍；DC 在空闲 SCLK=0 的接受沿锁存，整字节保持。
module ui_lcd_spi (
    input wire clk,
    rst_n,
    start,
    input wire [7:0] tx_byte,
    input wire dc_level,
    output reg sclk_q,
    mosi_q,
    dc_q,
    output reg busy_q,
    done_q
);
  reg [7:0] shift_q;
  reg [2:0] bit_index_q;
  reg half_count_q;

  // 职责：mode 0 字节移位与完成脉冲；clk 域，异步低复位清 SCLK/MOSI/DC/busy/done。
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sclk_q <= 1'd0;
      mosi_q <= 1'd0;
      dc_q <= 1'd0;
      busy_q <= 1'd0;
      done_q <= 1'd0;
      shift_q <= 8'd0;
      bit_index_q <= 3'd0;
      half_count_q <= 1'd0;
    end else begin
      done_q <= 1'd0;
      if (!busy_q) begin
        sclk_q <= 1'd0;
        if (start) begin
          shift_q <= tx_byte;
          mosi_q <= tx_byte[7];
          dc_q <= dc_level;
          bit_index_q <= 3'd7;
          half_count_q <= 1'd0;
          busy_q <= 1'd1;
        end
      end else if (!half_count_q) begin
        half_count_q <= 1'd1;
      end else begin
        half_count_q <= 1'd0;
        if (!sclk_q) begin
          sclk_q <= 1'd1;
        end else begin
          sclk_q <= 1'd0;
          if (bit_index_q == 0) begin
            busy_q <= 1'd0;
            done_q <= 1'd1;
          end else begin
            bit_index_q <= bit_index_q - 1'b1;
            shift_q <= {shift_q[6:0], 1'b0};
            mosi_q <= shift_q[6];
          end
        end
      end
    end
  end
endmodule

/* verilator lint_on DECLFILENAME */
/* verilator lint_on UNUSEDPARAM */
/* verilator lint_on UNUSEDSIGNAL */
