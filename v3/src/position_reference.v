`timescale 1ns / 1ps
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
    // 起终点准入保证插值落在signed32内；高位仅保留宽加法的符号扩展。
    /* verilator lint_off UNUSEDSIGNAL */
    wire signed [48:0] offset_next =
        $signed({{17{start_offset[31]}}, start_offset}) + rounded_delta;
    /* verilator lint_on UNUSEDSIGNAL */

    // Q16权重表：round(65536*(10s^3-15s^4+6s^5))，s=index/160。
    // 表只描述无量纲曲线，不包含电机线数、减速比或方向假设。
    function [16:0] curve_weight;
        input [7:0] index;
        begin
            case (index)
                8'd0: curve_weight = 17'd0;
                8'd1: curve_weight = 17'd0;
                8'd2: curve_weight = 17'd1;
                8'd3: curve_weight = 17'd4;
                8'd4: curve_weight = 17'd10;
                8'd5: curve_weight = 17'd19;
                8'd6: curve_weight = 17'd33;
                8'd7: curve_weight = 17'd51;
                8'd8: curve_weight = 17'd76;
                8'd9: curve_weight = 17'd107;
                8'd10: curve_weight = 17'd145;
                8'd11: curve_weight = 17'd192;
                8'd12: curve_weight = 17'd246;
                8'd13: curve_weight = 17'd310;
                8'd14: curve_weight = 17'd383;
                8'd15: curve_weight = 17'd467;
                8'd16: curve_weight = 17'd561;
                8'd17: curve_weight = 17'd666;
                8'd18: curve_weight = 17'd783;
                8'd19: curve_weight = 17'd911;
                8'd20: curve_weight = 17'd1052;
                8'd21: curve_weight = 17'd1205;
                8'd22: curve_weight = 17'd1372;
                8'd23: curve_weight = 17'd1551;
                8'd24: curve_weight = 17'd1744;
                8'd25: curve_weight = 17'd1951;
                8'd26: curve_weight = 17'd2171;
                8'd27: curve_weight = 17'd2406;
                8'd28: curve_weight = 17'd2655;
                8'd29: curve_weight = 17'd2918;
                8'd30: curve_weight = 17'd3196;
                8'd31: curve_weight = 17'd3489;
                8'd32: curve_weight = 17'd3796;
                8'd33: curve_weight = 17'd4118;
                8'd34: curve_weight = 17'd4455;
                8'd35: curve_weight = 17'd4806;
                8'd36: curve_weight = 17'd5172;
                8'd37: curve_weight = 17'd5553;
                8'd38: curve_weight = 17'd5949;
                8'd39: curve_weight = 17'd6359;
                8'd40: curve_weight = 17'd6784;
                8'd41: curve_weight = 17'd7223;
                8'd42: curve_weight = 17'd7677;
                8'd43: curve_weight = 17'd8144;
                8'd44: curve_weight = 17'd8626;
                8'd45: curve_weight = 17'd9121;
                8'd46: curve_weight = 17'd9630;
                8'd47: curve_weight = 17'd10152;
                8'd48: curve_weight = 17'd10688;
                8'd49: curve_weight = 17'd11236;
                8'd50: curve_weight = 17'd11797;
                8'd51: curve_weight = 17'd12370;
                8'd52: curve_weight = 17'd12956;
                8'd53: curve_weight = 17'd13553;
                8'd54: curve_weight = 17'd14162;
                8'd55: curve_weight = 17'd14781;
                8'd56: curve_weight = 17'd15412;
                8'd57: curve_weight = 17'd16053;
                8'd58: curve_weight = 17'd16705;
                8'd59: curve_weight = 17'd17366;
                8'd60: curve_weight = 17'd18036;
                8'd61: curve_weight = 17'd18715;
                8'd62: curve_weight = 17'd19403;
                8'd63: curve_weight = 17'd20100;
                8'd64: curve_weight = 17'd20804;
                8'd65: curve_weight = 17'd21515;
                8'd66: curve_weight = 17'd22234;
                8'd67: curve_weight = 17'd22958;
                8'd68: curve_weight = 17'd23689;
                8'd69: curve_weight = 17'd24426;
                8'd70: curve_weight = 17'd25168;
                8'd71: curve_weight = 17'd25914;
                8'd72: curve_weight = 17'd26665;
                8'd73: curve_weight = 17'd27419;
                8'd74: curve_weight = 17'd28177;
                8'd75: curve_weight = 17'd28938;
                8'd76: curve_weight = 17'd29701;
                8'd77: curve_weight = 17'd30466;
                8'd78: curve_weight = 17'd31233;
                8'd79: curve_weight = 17'd32000;
                8'd80: curve_weight = 17'd32768;
                8'd81: curve_weight = 17'd33536;
                8'd82: curve_weight = 17'd34303;
                8'd83: curve_weight = 17'd35070;
                8'd84: curve_weight = 17'd35835;
                8'd85: curve_weight = 17'd36598;
                8'd86: curve_weight = 17'd37359;
                8'd87: curve_weight = 17'd38117;
                8'd88: curve_weight = 17'd38871;
                8'd89: curve_weight = 17'd39622;
                8'd90: curve_weight = 17'd40368;
                8'd91: curve_weight = 17'd41110;
                8'd92: curve_weight = 17'd41847;
                8'd93: curve_weight = 17'd42578;
                8'd94: curve_weight = 17'd43302;
                8'd95: curve_weight = 17'd44021;
                8'd96: curve_weight = 17'd44732;
                8'd97: curve_weight = 17'd45436;
                8'd98: curve_weight = 17'd46133;
                8'd99: curve_weight = 17'd46821;
                8'd100: curve_weight = 17'd47500;
                8'd101: curve_weight = 17'd48170;
                8'd102: curve_weight = 17'd48831;
                8'd103: curve_weight = 17'd49483;
                8'd104: curve_weight = 17'd50124;
                8'd105: curve_weight = 17'd50755;
                8'd106: curve_weight = 17'd51374;
                8'd107: curve_weight = 17'd51983;
                8'd108: curve_weight = 17'd52580;
                8'd109: curve_weight = 17'd53166;
                8'd110: curve_weight = 17'd53739;
                8'd111: curve_weight = 17'd54300;
                8'd112: curve_weight = 17'd54848;
                8'd113: curve_weight = 17'd55384;
                8'd114: curve_weight = 17'd55906;
                8'd115: curve_weight = 17'd56415;
                8'd116: curve_weight = 17'd56910;
                8'd117: curve_weight = 17'd57392;
                8'd118: curve_weight = 17'd57859;
                8'd119: curve_weight = 17'd58313;
                8'd120: curve_weight = 17'd58752;
                8'd121: curve_weight = 17'd59177;
                8'd122: curve_weight = 17'd59587;
                8'd123: curve_weight = 17'd59983;
                8'd124: curve_weight = 17'd60364;
                8'd125: curve_weight = 17'd60730;
                8'd126: curve_weight = 17'd61081;
                8'd127: curve_weight = 17'd61418;
                8'd128: curve_weight = 17'd61740;
                8'd129: curve_weight = 17'd62047;
                8'd130: curve_weight = 17'd62340;
                8'd131: curve_weight = 17'd62618;
                8'd132: curve_weight = 17'd62881;
                8'd133: curve_weight = 17'd63130;
                8'd134: curve_weight = 17'd63365;
                8'd135: curve_weight = 17'd63585;
                8'd136: curve_weight = 17'd63792;
                8'd137: curve_weight = 17'd63985;
                8'd138: curve_weight = 17'd64164;
                8'd139: curve_weight = 17'd64331;
                8'd140: curve_weight = 17'd64484;
                8'd141: curve_weight = 17'd64625;
                8'd142: curve_weight = 17'd64753;
                8'd143: curve_weight = 17'd64870;
                8'd144: curve_weight = 17'd64975;
                8'd145: curve_weight = 17'd65069;
                8'd146: curve_weight = 17'd65153;
                8'd147: curve_weight = 17'd65226;
                8'd148: curve_weight = 17'd65290;
                8'd149: curve_weight = 17'd65344;
                8'd150: curve_weight = 17'd65391;
                8'd151: curve_weight = 17'd65429;
                8'd152: curve_weight = 17'd65460;
                8'd153: curve_weight = 17'd65485;
                8'd154: curve_weight = 17'd65503;
                8'd155: curve_weight = 17'd65517;
                8'd156: curve_weight = 17'd65526;
                8'd157: curve_weight = 17'd65532;
                8'd158: curve_weight = 17'd65535;
                8'd159: curve_weight = 17'd65536;
                8'd160: curve_weight = 17'd65536;
                default: curve_weight = 17'd65536;
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
