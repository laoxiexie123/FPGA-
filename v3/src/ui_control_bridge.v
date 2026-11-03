`timescale 1ns / 1ps
// 本地UI命令桥：固定profile只校验，不伪装成运行时配置写入；不含UART解析器。
module ui_control_bridge (
    input wire clk, rst_n,
    input wire tick_1ms,
    input wire command_request,
    input wire [3:0] command_kind,
    input wire [5:0] command_index,
    input wire signed [31:0] command_value,
    input wire start_ready, run_fault, position_ready, trajectory_fault, running,
    input wire signed [31:0] position_limit,
    output reg tx_busy, tx_done, query_done, armed,
    output reg start_pulse, stop_pulse, zero_pulse, heartbeat_pulse,
    output reg motion_enter, motion_exit, motion_target,
    output reg signed [31:0] motion_target_offset,
    output reg [11:0] zero_adc_x4,
    output reg [31:0] error_count
);
    reg [3:0] pending_kind;
    reg [5:0] pending_index;
    reg signed [31:0] pending_value;
    wire [31:0] profile_config;
    wire signed [31:0] profile_u, profile_v, profile_j, profile_b;
    // 无心跳超时计数；tick只保留统一接口，profile积分/默认零点不覆盖当前标定。
    /* verilator lint_off UNUSEDSIGNAL */
    wire reserved_tick = tick_1ms;
    wire signed [31:0] profile_i;
    wire [31:0] profile_zero;
    /* verilator lint_on UNUSEDSIGNAL */
    local_startup_profile fixed_profile (
        .config_index(pending_index), .cfg_value_next(profile_config),
        .startup_u_next(profile_u), .startup_v_next(profile_v),
        .startup_j_next(profile_j), .startup_b_next(profile_b),
        .startup_i_next(profile_i), .default_zero_next(profile_zero)
    );
    reg accepted;
    // 功能：校验已锁存请求；组合译码，无寄存器、时钟或复位。
    always @* begin
        accepted = 1'b0;
        case (pending_kind)
            4'd0: accepted = !running && ((pending_index == 6'd6) ?
                (pending_value >= 0 && pending_value <= 32'sd4092) :
                (pending_index < 6'd48 && $unsigned(pending_value) == profile_config));
            4'd1: accepted = !running && pending_value == profile_u;
            4'd2: accepted = !running && pending_value == profile_v;
            4'd3: accepted = !running && pending_value == profile_j;
            4'd4: accepted = !running && pending_value == profile_b;
            4'd5: accepted = (pending_value == 1 && running && position_ready &&
                !run_fault && !trajectory_fault) || (pending_value == 0 && running);
            // 内部相对移动事件：T=+130/-130单步，T=1040顺时针一圈。
            // 4秒曲线、累计位置包络和忙时忽略由V3 position_reference执行。
            4'd6: accepted = running && position_ready && !run_fault && !trajectory_fault &&
                (pending_value == 32'sd130 || pending_value == -32'sd130 ||
                  pending_value == 32'sd1040) && position_limit > 0;
            4'd7: accepted = 1'b1;
            4'd8: accepted = !running;
            4'd9: accepted = 1'b1;
            4'd10: accepted = pending_value == 1 && start_ready && !run_fault &&
                !trajectory_fault && !running;
            4'd11: accepted = pending_value == 1 && armed && start_ready && !run_fault &&
                !trajectory_fault && !running;
            4'd12: accepted = 1'b1;
            4'd13: accepted = !running && pending_value == 0;
            default: accepted = 1'b0;
        endcase
    end
    wire collision = tx_busy && command_request;
    wire rejected = tx_busy && !accepted;
    // 功能：锁请求、下一拍完成并发单拍控制事件；clk域，rst_n异步低有效。
    // 独占全部寄存器；busy碰撞拒收并计错，完成沿同样不接受第二条请求。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_busy <= 0; tx_done <= 0; query_done <= 0; armed <= 0;
            start_pulse <= 0; stop_pulse <= 0; zero_pulse <= 0; heartbeat_pulse <= 0;
            motion_enter <= 0; motion_exit <= 0; motion_target <= 0;
            motion_target_offset <= 0; zero_adc_x4 <= 12'd2965; error_count <= 0;
            pending_kind <= 0; pending_index <= 0; pending_value <= 0;
        end else begin
            tx_done <= 0; query_done <= 0;
            start_pulse <= 0; stop_pulse <= 0; zero_pulse <= 0; heartbeat_pulse <= 0;
            motion_enter <= 0; motion_exit <= 0; motion_target <= 0;
            if (run_fault || trajectory_fault || running) armed <= 0;
            if (collision || rejected) begin
                // 饱和错误计数，碰撞及原请求拒绝可在同一拍各记一次。
                if (error_count == 32'hffffffff ||
                    (error_count == 32'hfffffffe && collision && rejected))
                    error_count <= 32'hffffffff;
                else error_count <= error_count + {31'd0,collision} + {31'd0,rejected};
            end
            if (tx_busy) begin
                tx_busy <= 0;
                tx_done <= 1;
                if (accepted) begin
                    case (pending_kind)
                        4'd0: if (pending_index == 6'd6) zero_adc_x4 <= pending_value[11:0];
                        4'd5: if (pending_value == 1) motion_enter <= 1;
                              else motion_exit <= 1;
                        4'd6: begin motion_target <= 1; motion_target_offset <= pending_value; end
                        4'd7: begin stop_pulse <= 1; armed <= 0; end
                        4'd8: zero_pulse <= 1;
                        4'd9: query_done <= 1;
                        4'd10: armed <= 1;
                        4'd11: begin start_pulse <= 1; armed <= 0; end
                        4'd12: heartbeat_pulse <= 1;
                        default: begin end
                    endcase
                end
            end else if (command_request) begin
                pending_kind <= command_kind;
                pending_index <= command_index;
                pending_value <= command_value;
                tx_busy <= 1;
            end
        end
    end
endmodule
