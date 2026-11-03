"""Generate a self-contained fixed 4s quintic reference LUT; run only for initial authoring."""
from pathlib import Path
root = Path(__file__).resolve().parents[1]
header = '''`timescale 1ns / 1ps
// 平滑位置偏移：4秒五次曲线；50MHz单域，tick_1ms只作使能。
// step_counts为每次有符号位移：数值/正负由编码器规格和实际方向确认，不在此猜测。
// ready须同时包含运行许可、正常立摆、位置原点已锁定；失效清零并取消移动。
module position_reference (
    input wire clk_50m, rst_n, tick_1ms,
    input wire ready, move_press,
    input wire signed [31:0] origin_counts, step_counts,
    input wire signed [31:0] position_limit,
    output reg signed [31:0] target_offset_counts,
    output reg moving, target_done
);
    reg [4:0] millisecond_phase;
    reg [7:0] step_index;
    reg signed [31:0] start_offset, goal_offset, move_delta;
    reg signed [48:0] weighted_delta;
    reg publish_pending;
    wire signed [32:0] proposed_goal =
        $signed({target_offset_counts[31], target_offset_counts}) +
        $signed({step_counts[31], step_counts});
    wire signed [33:0] proposed_absolute =
        $signed({{2{origin_counts[31]}}, origin_counts}) +
        $signed({proposed_goal[32], proposed_goal});
    wire signed [33:0] start_absolute =
        $signed({{2{origin_counts[31]}}, origin_counts}) +
        $signed({{2{target_offset_counts[31]}}, target_offset_counts});
    wire signed [33:0] limit_extended = $signed({2'b00, position_limit});
    wire request_valid = step_counts != 0 && position_limit > 0 &&
        proposed_goal >= -33'sd2147483648 && proposed_goal <= 33'sd2147483647 &&
        proposed_absolute >= -limit_extended && proposed_absolute <= limit_extended &&
        start_absolute >= -limit_extended && start_absolute <= limit_extended;
    wire [7:0] next_index = step_index + 1'b1;
    wire signed [17:0] next_weight = $signed({1'b0, curve_weight(next_index)});
    wire signed [49:0] product_next = move_delta * next_weight;
    // 乘积保证落在signed49范围；被舍弃的最高位只是符号扩展。
    /* verilator lint_off UNUSEDSIGNAL */
    wire product_sign_extension = product_next[49];
    /* verilator lint_on UNUSEDSIGNAL */
    wire signed [48:0] rounded_delta = weighted_delta < 0 ?
        -((-weighted_delta) >>> 16) : (weighted_delta >>> 16);
    wire signed [48:0] offset_next =
        $signed({{17{start_offset[31]}}, start_offset}) + rounded_delta;

    // Q16权重表：round(65536*(10s^3-15s^4+6s^5))，s=index/160。
    // 表只描述无量纲曲线，不包含电机线数、减速比或方向假设。
    function [16:0] curve_weight;
        input [7:0] index;
        begin
            case (index)
'''
rows = []
for i in range(161):
    # Exact rational quintic, rounded to nearest integer with ties upward.
    n = (10*i**3*160**2 - 15*i**4*160 + 6*i**5) * 65536
    den = 160**5
    w = (2*n+den)//(2*den)
    rows.append(f"                8'd{i}: curve_weight = 17'd{w};")
footer = '''                default: curve_weight = 17'd65536;
            endcase
        end
    endfunction

    // 功能：请求准入、25ms节拍、乘积和发布；clk_50m上升沿，rst_n异步低有效。
    // 独占全部状态；输入ready撤销优先于发布/按键，忙时不记录或排队按键。
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            millisecond_phase <= 0;
            step_index <= 0;
            start_offset <= 0;
            goal_offset <= 0;
            move_delta <= 0;
            weighted_delta <= 0;
            publish_pending <= 0;
            target_offset_counts <= 0;
            moving <= 0;
            target_done <= 0;
        end else begin
            target_done <= 0;
            if (!ready) begin
                millisecond_phase <= 0;
                step_index <= 0;
                start_offset <= 0;
                goal_offset <= 0;
                move_delta <= 0;
                weighted_delta <= 0;
                publish_pending <= 0;
                target_offset_counts <= 0;
                moving <= 0;
            end else if (moving) begin
                if (publish_pending) begin
                    publish_pending <= 0;
                    if (step_index == 8'd160) begin
                        target_offset_counts <= goal_offset;
                        moving <= 0;
                        target_done <= 1;
                    end else begin
                        target_offset_counts <= offset_next[31:0];
                    end
                end
                if (tick_1ms) begin
                    if (millisecond_phase == 5'd24) begin
                        millisecond_phase <= 0;
                        step_index <= next_index;
                        weighted_delta <= product_next[48:0];
                        publish_pending <= 1;
                    end else millisecond_phase <= millisecond_phase + 1'b1;
                end
            end else if (move_press && request_valid) begin
                start_offset <= target_offset_counts;
                goal_offset <= proposed_goal[31:0];
                move_delta <= step_counts;
                millisecond_phase <= 0;
                step_index <= 0;
                publish_pending <= 0;
                moving <= 1;
            end
        end
    end
endmodule
'''
(root/'src/position_reference.v').write_text(header+'\n'.join(rows)+'\n'+footer,encoding='utf-8')
