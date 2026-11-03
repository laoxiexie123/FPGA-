`timescale 1ns / 1ps
// 50MHz低有效按键：按下和释放均连续稳定stable_cycles拍才接受。
// stable_cycles须>=1；默认1000000拍为20ms，不包含输入同步延迟。
// 复位按住只更新pressed；先稳定释放再稳定按下才允许press_pulse。
module key_filter #(
    parameter integer stable_cycles = 1000000
)(
    input wire clk_50m,
    input wire rst_n,
    input wire key_n,
    output reg pressed,
    output reg press_pulse
);
    localparam integer counter_width = (stable_cycles > 1) ? $clog2(stable_cycles) : 1;
    localparam [counter_width-1:0] last_count = stable_cycles[counter_width-1:0] - 1'b1;
    reg key_first_stage;
    reg key_synchronized;
    reg [1:0] synchronized_valid;
    reg [counter_width-1:0] stable_count;
    reg press_armed;
    reg previous_key_level;
    wire qualification_needed = (pressed == key_synchronized) ||
        (!press_armed && key_synchronized);

    // 功能：两级同步异步按键并标记真实样本到达；clk_50m上升沿，rst_n异步低有效。
    // 独占写key_first_stage、key_synchronized、synchronized_valid；只读key_n。
    // 第二段只读同步结果，不把消抖状态反馈到同步链。
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            key_first_stage <= 1'b1;
            key_synchronized <= 1'b1;
            synchronized_valid <= 2'b00;
        end else begin
            key_first_stage <= key_n;
            key_synchronized <= key_first_stage;
            synchronized_valid <= {synchronized_valid[0], 1'b1};
        end
    end

    // 功能：双沿消抖及释放后武装，按下发单拍事件；clk_50m上升沿，rst_n异步低有效。
    // 独占写stable_count、previous_key_level及按键状态/事件；只读同步链。
    // 同步初值不得武装，避免stable_cycles=1且复位按住时误发press_pulse。
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            stable_count <= {counter_width{1'b0}};
            press_armed <= 1'b0;
            previous_key_level <= 1'b1;
            pressed <= 1'b0;
            press_pulse <= 1'b0;
        end else begin
            press_pulse <= 1'b0;
            previous_key_level <= key_synchronized;
            if (!synchronized_valid[1] || !qualification_needed) begin
                stable_count <= {counter_width{1'b0}};
            end else if (stable_cycles == 1 ||
                (previous_key_level == key_synchronized && stable_count == last_count)) begin
                stable_count <= {counter_width{1'b0}};
                pressed <= !key_synchronized;
                if (key_synchronized) begin
                    press_armed <= 1'b1;
                end else begin
                    press_pulse <= press_armed;
                    press_armed <= 1'b0;
                end
            end else if (previous_key_level != key_synchronized) begin
                stable_count <= {{(counter_width-1){1'b0}}, 1'b1};
            end else begin
                stable_count <= stable_count + 1'b1;
            end
        end
    end
endmodule
