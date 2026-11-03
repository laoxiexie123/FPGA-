`timescale 1ns / 1ps

// 定位：EC11 A/B 与低有效按键产生单拍事件；不补猜漏步、不控制执行器。
// 时钟/复位：clk 单域，rst_n 低有效异步复位；触点经过两级同步。
// 可信：step_cw_q/step_ccw_q/press_q 为单拍事件，无 ready 或事件队列。
// legacy：旋转先确认 11；按键启动必须先稳定释放，复位时按住不补发事件。
// [param]rule：filter_cycles=25000、press_cycles=1000000，均为 clk 拍数，不含外部频率假设。
// [width]rule：参数>1 时计数宽度为 $clog2(参数)，否则为 1；零扩展表达式要求宽度<=32。
// [threshold]rule：参数<=1 直接采样；参数>1 候选变化先记 1，计至参数-1 后确认。
// [phase]rule：CW 为 11→01→00→10→11；signed[3:0] 累计值在回 11 且为 +3/-3 才出事件。
// [startup]rule：sync_valid_q[1] 后才消抖；双位跳变清累计，按键稳定高才重新 armed。
module ec11_input #(
    parameter integer filter_cycles = 25000,
    parameter integer press_cycles  = 1000000
) (
    input  wire clk,
    input  wire rst_n,
    input  wire a,
    input  wire b,
    input  wire button_n,
    output reg  step_cw_q,
    output reg  step_ccw_q,
    output reg  press_q
);

  localparam integer filter_width = (filter_cycles > 1) ? $clog2(filter_cycles) : 1;
  localparam integer press_width  = (press_cycles > 1) ? $clog2(press_cycles) : 1;

  (* ASYNC_REG = "TRUE" *) reg [1:0] ab_meta_q, ab_sync_q;
  (* ASYNC_REG = "TRUE" *) reg button_meta_q, button_sync_q;
  reg [1:0] sync_valid_q;

  // 职责：同步 A/B、按键与流水有效；clk 域，异步低复位为释放态且 valid=0。
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ab_meta_q <= 2'b11;
      ab_sync_q <= 2'b11;
      button_meta_q <= 1'b1;
      button_sync_q <= 1'b1;
      sync_valid_q <= 2'b00;
    end else begin
      ab_meta_q <= {a, b};
      ab_sync_q <= ab_meta_q;
      button_meta_q <= button_n;
      button_sync_q <= button_meta_q;
      sync_valid_q <= {sync_valid_q[0], 1'b1};
    end
  end

  reg [1:0] ab_candidate_q, ab_filtered_q;
  reg [filter_width-1:0] filter_count_q;
  reg ab_ready_q;

  // 职责：AB 成对候选消抖；clk 域，异步低复位为 11 且 ab_ready_q=0。
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ab_candidate_q <= 2'b11;
      ab_filtered_q <= 2'b11;
      filter_count_q <= {filter_width{1'b0}};
      ab_ready_q <= 1'b0;
    end else if (sync_valid_q[1]) begin
      if (filter_cycles <= 1) begin
        ab_candidate_q <= ab_sync_q;
        ab_filtered_q <= ab_sync_q;
        ab_ready_q <= 1'b1;
      end else if (ab_sync_q != ab_candidate_q) begin
        ab_candidate_q <= ab_sync_q;
        filter_count_q <= {{(filter_width - 1) {1'b0}}, 1'b1};
        ab_ready_q <= 1'b0;
      end else if (!ab_ready_q) begin
        if ({{(32 - filter_width) {1'b0}}, filter_count_q} == filter_cycles - 1) begin
          ab_filtered_q <= ab_candidate_q;
          ab_ready_q <= 1'b1;
        end else begin
          filter_count_q <= filter_count_q + 1'b1;
        end
      end
    end
  end

  reg [1:0] ab_previous_q;
  reg signed [3:0] quarter_steps_q;
  reg encoder_armed_q;

  // 职责：相序累计与整步单拍事件；clk 域，异步低复位清 armed、累计和事件。
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ab_previous_q <= 2'b11;
      quarter_steps_q <= 4'sd0;
      encoder_armed_q <= 1'b0;
      step_cw_q <= 1'b0;
      step_ccw_q <= 1'b0;
    end else begin
      step_cw_q  <= 1'b0;
      step_ccw_q <= 1'b0;
      if (ab_ready_q) begin
        ab_previous_q <= ab_filtered_q;
        if (!encoder_armed_q) begin
          quarter_steps_q <= 4'sd0;

          if (ab_filtered_q == 2'b11) begin
            encoder_armed_q <= 1'b1;
          end
        end else if (ab_filtered_q != ab_previous_q) begin
          if ((ab_filtered_q ^ ab_previous_q) == 2'b11) begin
            quarter_steps_q <= 4'sd0;
            encoder_armed_q <= (ab_filtered_q == 2'b11);
          end else begin
            case ({
              ab_previous_q, ab_filtered_q
            })
              // 合法 CW 单相变化：本拍累计；回 11 且旧累计=3 时单拍报 CW，下拍归零。
              4'b1101, 4'b0100, 4'b0010, 4'b1011: begin
                if (ab_filtered_q == 2'b11) begin
                  if (quarter_steps_q == 4'sd3) begin
                    step_cw_q <= 1'b1;
                  end
                  quarter_steps_q <= 4'sd0;
                end else begin
                  quarter_steps_q <= quarter_steps_q + 4'sd1;
                end
              end
              // 合法 CCW 单相变化：本拍累计；回 11 且旧累计=-3 时单拍报 CCW，下拍归零。
              4'b1110, 4'b1000, 4'b0001, 4'b0111: begin
                if (ab_filtered_q == 2'b11) begin
                  if (quarter_steps_q == -4'sd3) begin
                    step_ccw_q <= 1'b1;
                  end
                  quarter_steps_q <= 4'sd0;
                end else begin
                  quarter_steps_q <= quarter_steps_q - 4'sd1;
                end
              end
              // 未列相序：本拍清累计，不产生整步事件；下拍等待后续稳定变化。
              default: begin
                quarter_steps_q <= 4'sd0;
              end
            endcase
          end
        end
      end
    end
  end

  reg button_candidate_q;
  reg [press_width-1:0] press_count_q;
  reg button_armed_q;

  // 职责：按键稳定确认与单拍按下；clk 域，异步低复位清 armed，必须先确认释放。
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      button_candidate_q <= 1'b1;
      press_count_q <= {press_width{1'b0}};
      button_armed_q <= 1'b0;
      press_q <= 1'b0;
    end else begin
      press_q <= 1'b0;
      if (sync_valid_q[1]) begin
        if (press_cycles <= 1) begin
          button_candidate_q <= button_sync_q;
          if (button_sync_q) begin
            button_armed_q <= 1'b1;
          end else if (button_armed_q) begin
            press_q <= 1'b1;
            button_armed_q <= 1'b0;
          end
        end else if (button_sync_q != button_candidate_q) begin
          button_candidate_q <= button_sync_q;
          press_count_q <= {{(press_width - 1) {1'b0}}, 1'b1};
        end else if ({{(32 - press_width) {1'b0}}, press_count_q} == press_cycles - 1) begin

          if (button_candidate_q) begin
            button_armed_q <= 1'b1;
          end else if (button_armed_q) begin
            press_q <= 1'b1;
            button_armed_q <= 1'b0;
          end
        end else begin
          press_count_q <= press_count_q + 1'b1;
        end
      end
    end
  end
endmodule
