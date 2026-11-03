`timescale 1ns / 1ps
// 用途：旧UI启动握手表；有效项对齐V3.1固定控制值，不作为动态参数写入总线。
// 接口：config_index 组合选择 32 位 cfg_value_next，无 valid；其余输出为固定常量。
// 时序：纯组合逻辑，无时钟、复位或寄存器，不增加加载周期。
// 边界：此表不保存校准结果；C06 的当前值由 UI 在发送时覆盖。
// [位宽] 配置和默认零点为 32 位无符号；U/V/J/B/I 为 32 位有符号命令值。
// [配置] Cnn 对应 config_index=6'dN；UI 发往配置字 cfg[32*N+:32]，C06 用当前零点覆盖。
// [启动] U=220 为驱动幅值上限。
// [启动] V=52 counts/s，保持现有位置轨迹速度限制。
// [启动] J=52 counts/s²，保持现有位置轨迹加速度限制。
// [启动] B=520 counts；UI 旋钮步长与交互钳位仍由 UI 管理。
// [启动] I=0：V3.1保持V3无积分控制，启动表不得引入旧位置积分。
// [校准] 默认零点 2965 为 ADC×4，复位、恢复和 C06 共用唯一常量。
module local_startup_profile (
    input wire [5:0] config_index,
    output reg [31:0] cfg_value_next,
    output wire signed [31:0] startup_u_next,
    output wire signed [31:0] startup_v_next,
    output wire signed [31:0] startup_j_next,
    output wire signed [31:0] startup_b_next,
    output wire signed [31:0] startup_i_next,
    output wire [31:0] default_zero_next
);
  assign startup_u_next = 32'sd220;
  assign startup_v_next = 32'sd52;
  assign startup_j_next = 32'sd52;
  assign startup_b_next = 32'sd520;
  assign startup_i_next = 32'sd0;
  assign default_zero_next = 32'd2965;

  // [组合] 职责：6 位索引即时查表；无时钟域和复位，全部 64 种索引均赋值。
  always @* begin
    case (config_index)
      // [配置] C00：角度比例增益 Q16。
      6'd0: begin
        cfg_value_next = 32'd80000;
      end
      // [配置] C01：每控制拍角误差增量增益 Q16。
      6'd1: begin
        cfg_value_next = 32'd350000;
      end
      // [配置] C02：位置比例增益 Q16。
      6'd2: begin
        cfg_value_next = 32'd10000;
      end
      // [配置] C03：每控制拍位置误差增量增益 Q16。
      6'd3: begin
        cfg_value_next = 32'd500000;
      end
      // [配置] C04：ADC 低通权重 /256。
      6'd4: begin
        cfg_value_next = 32'd128;
      end
      // [配置] C05：位置低通权重 /256。
      6'd5: begin
        cfg_value_next = 32'd64;
      end
      // [配置] C06：直立零点 ADC×4。
      6'd6: begin
        cfg_value_next = default_zero_next;
      end
      // [配置] C07：运行角误差上限 ADC×4。
      6'd7: begin
        cfg_value_next = 32'd400;
      end
      // [配置] C08：摆臂速度保护 counts/s。
      6'd8: begin
        cfg_value_next = 32'd4400;
      end
      // [配置] C09：位置绝对值保护 counts。
      6'd9: begin
        cfg_value_next = 32'd10000;
      end
      // [配置] C10：总运行限时 ms，0 关闭该限时。
      6'd10: begin
        cfg_value_next = 32'd0;
      end
      // [配置] C11：心跳超时 ms。
      6'd11: begin
        cfg_value_next = 32'd100;
      end
      // [配置] C12：ADC 最大样龄 ms。
      6'd12: begin
        cfg_value_next = 32'd20;
      end
      // [配置] C13：编码器稳定采样周期数。
      6'd13: begin
        cfg_value_next = 32'd4;
      end
      // [配置] C14：启动角误差 ADC×4。
      6'd14: begin
        cfg_value_next = 32'd380;
      end
      // [配置] C15：启动摆臂速度 counts/s。
      6'd15: begin
        cfg_value_next = 32'd200;
      end
      // [配置] C16：启动位置绝对值 counts。
      6'd16: begin
        cfg_value_next = 32'd10000;
      end
      // [配置] C17：遥测周期 ms。
      6'd17: begin
        cfg_value_next = 32'd50;
      end
      // [配置] C18：ADC 轨边裕量 counts。
      6'd18: begin
        cfg_value_next = 32'd10;
      end
      // [配置] C19：平衡驱动幅值上限。
      6'd19: begin
        cfg_value_next = 32'd150;
      end
      // [配置] C20：摩擦补偿幅值。
      6'd20: begin
        cfg_value_next = 32'd1;
      end
      // [配置] C21：起摆模式，3 为脉冲模式。
      6'd21: begin
        cfg_value_next = 32'd3;
      end
      // [配置] C22：脉冲驱动幅值。
      6'd22: begin
        cfg_value_next = 32'd180;
      end
      // [配置] C23：脉冲持续时间 ms。
      6'd23: begin
        cfg_value_next = 32'd180;
      end
      // [配置] C24：兼容占位能量增益 Q8。
      6'd24: begin
        cfg_value_next = 32'd27648;
      end
      // [配置] C25：起摆驱动幅值上限。
      6'd25: begin
        cfg_value_next = 32'd220;
      end
      // [配置] C26：兼容占位助推极性。
      6'd26: begin
        cfg_value_next = 32'd1;
      end
      // [配置] C27：反向制动持续时间 ms。
      6'd27: begin
        cfg_value_next = 32'd100;
      end
      // [配置] C28：捕获角度 cdeg。
      6'd28: begin
        cfg_value_next = 32'd2500;
      end
      // [配置] C29：捕获角速度 cdeg/s。
      6'd29: begin
        cfg_value_next = 32'd100000;
      end
      // [配置] C30：捕获摆臂速度 counts/s。
      6'd30: begin
        cfg_value_next = 32'd4400;
      end
      // [配置] C31：捕获驻留时间 ms。
      6'd31: begin
        cfg_value_next = 32'd5;
      end
      // [配置] C32：接管混合时间 ms。
      6'd32: begin
        cfg_value_next = 32'd5;
      end
      // [配置] C33：反向制动幅值与每控制使能驱动变化上限。
      6'd33: begin
        cfg_value_next = 32'd180;
      end
      // [配置] C34：角度换算比例 Q8。
      6'd34: begin
        cfg_value_next = 32'd8518;
      end
      // [配置] C35：兼容占位固有角频率 mrad/s。
      6'd35: begin
        cfg_value_next = 32'd9710;
      end
      // [配置] C36：有效 ADC 下限。
      6'd36: begin
        cfg_value_next = 32'd1;
      end
      // [配置] C37：有效 ADC 上限。
      6'd37: begin
        cfg_value_next = 32'd1022;
      end
      // [配置] C38：摆杆速度有效上限 cdeg/s。
      6'd38: begin
        cfg_value_next = 32'd200000;
      end
      // [配置] C39：速度低通权重 /256。
      6'd39: begin
        cfg_value_next = 32'd16;
      end
      // [配置] C40：含首脉冲的脉冲总数。
      6'd40: begin
        cfg_value_next = 32'd2;
      end
      // [配置] C41：后续脉冲触发角度窗 cdeg。
      6'd41: begin
        cfg_value_next = 32'd8500;
      end
      // [配置] C42：退出捕获角 cdeg。
      6'd42: begin
        cfg_value_next = 32'd3200;
      end
      // [配置] C43：起摆超时 ms。
      6'd43: begin
        cfg_value_next = 32'd10000;
      end
      // [配置] C44：盲区驱动幅值上限。
      6'd44: begin
        cfg_value_next = 32'd0;
      end
      // [配置] C45：脉冲方向选择。
      6'd45: begin
        cfg_value_next = 32'd1;
      end
      // [配置] C46：起摆校准确认。
      6'd46: begin
        cfg_value_next = 32'd1;
      end
      // [配置] C47：重捕获单步角差 cdeg。
      6'd47: begin
        cfg_value_next = 32'd18000;
      end
      // [边界] 索引 48..63 本次输出 32 位零；无状态，也不回绕加载有效项。
      default: begin
        cfg_value_next = 32'd0;
      end
    endcase
  end
endmodule
