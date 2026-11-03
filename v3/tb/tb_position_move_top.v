`timescale 1ns/1ps
// II级集成：ready/测量/运行许可为仿真force，不代表真实起摆或完整0.5s锁原点。
// SW3经过真实同步链，仅20ms消抖末值和1ms使能加速；不替换生产模块。
module tb_position_move_top;
  reg clk=0;
  always #10 clk=~clk;
  reg rst_n=0,sw3_n=1;
  wire ac,oe,pa,a1,a2,pb,b1,b2;
  top dut(.clk_50m(clk),.rst_n(rst_n),.key_sw1_n(1'b1),.key_sw3_n(sw3_n),
    .key_sw4_n(1'b1),.enc1_a(1'b0),.enc1_b(1'b0),.adc_data_in(10'd741),
    .adc_clk_next(ac),.adc_oe_n_next(oe),.pwma_next(pa),.an1_next(a1),
    .an2_next(a2),.pwmb_next(pb),.bn1_next(b1),.bn2_next(b2));
  integer checks=0,presses=0,done_count=0,i;
  reg tick_b=0,seed_b=0,enable_b=0;
  reg signed [31:0] offset_b=0;
  wire signed [15:0] cmd_b;
  wire valid_b,busy_b,fault_b;
  wire [7:0] code_b;
  wire signed [31:0] origin_b;
  balance_control probe(.clk_50m(clk),.rst_n(rst_n),.tick_5ms(tick_b),.enable(enable_b),
    .clear(1'b0),.capture_seed(seed_b),.entry_boost(1'b0),.position_loop_enable(1'b1),
    .position_origin_seed(1'b0),.filtered_adc_x4(12'd2964),.filtered_adc_q8(32'sd189696),
    .adc_ready(1'b1),.zero_adc_x4(12'd2965),.position_counts(32'sd0),
    .position_weight_q15(16'd32768),.position_reference_offset(offset_b),
    .position_origin_counts(origin_b),.motor_command(cmd_b),.result_valid(valid_b),
    .calculating(busy_b),.control_fault(fault_b),.fault_code(code_b));
  task check;
    input condition;
    input [767:0] text;
    begin checks=checks+1;if(condition!==1'b1)$fatal(1,"CHECK %0s",text);end
  endtask
  task clocks;
    input integer n;integer k;
    begin for(k=0;k<n;k=k+1)begin @(posedge clk);#2;end end
  endtask
  always @(posedge clk) begin
    #1;
    if(dut.move_press)presses=presses+1;
    if(dut.reference_generator.target_done)done_count=done_count+1;
  end
  task key_release;
    begin
      @(negedge clk);sw3_n=1;clocks(6);
      @(negedge clk);dut.move_key.stable_count=20'd999999;
      clocks(4);
    end
  endtask
  task key_press;
    begin
      @(negedge clk);sw3_n=0;clocks(6);
      @(negedge clk);dut.move_key.stable_count=20'd999999;
      clocks(4);
    end
  endtask
  task milliseconds;
    input integer n;integer k;
    begin
      for(k=0;k<n;k=k+1)begin
        @(negedge clk);force dut.tick_1ms=1'b1;
        clocks(1);
        @(negedge clk);force dut.tick_1ms=1'b0;
        clocks(2);
      end
    end
  endtask
  task balance_request;
    begin
      @(negedge clk);tick_b=1;
      clocks(1);
      @(negedge clk);tick_b=0;
    end
  endtask
  initial begin
    #1;rst_n=0;clocks(3);@(negedge clk);rst_n=1;clocks(8);
    // 只测试位置移动连接；禁用实际起摆与平衡运行，避免force许可引起既有保护。
    force dut.position_ready=1'b0;
    force dut.motor_permitted=1'b1;
    force dut.core_enable=1'b0;
    force dut.average_fresh=1'b1;
    force dut.filter_ready=1'b1;
    force dut.observer_fault=1'b0;
    force dut.tick_1ms=1'b0;
    key_release;key_press;
    check(presses==1,"physical SW3 produces one pulse while not ready");
    check(!dut.reference_generator.moving&&dut.reference_offset==0,"not ready ignores press");
    force dut.position_ready=1'b1;
    key_release;key_press;
    check(presses==2&&dut.reference_generator.moving,"ready physical SW3 starts move");
    check(dut.reference_offset==0,"start is continuous at old target");
    milliseconds(2000);
    check(dut.reference_offset==65,"two seconds reaches half of 130 count step");
    key_release;key_press;
    check(presses==3&&dut.reference_generator.goal_offset==130,"busy physical press ignored");
    milliseconds(2000);
    check(!dut.reference_generator.moving&&dut.reference_offset==130,"4000 one-ms pulses reach 130");
    check(done_count==1,"one completion pulse for first move");
    milliseconds(10);
    check(dut.reference_offset==130&&!dut.reference_generator.moving,"ignored busy request not queued");
    key_release;key_press;milliseconds(4000);
    check(dut.reference_offset==260&&!dut.reference_generator.moving,"second press accumulates to 260");
    check(done_count==2,"second move has one completion pulse");
    key_release;key_press;milliseconds(1000);
    check(dut.reference_generator.moving&&dut.reference_offset>260,"third move active before interruption");
    @(negedge clk);force dut.position_ready=1'b0;clocks(2);
    check(dut.reference_offset==0&&!dut.reference_generator.moving,"ready revoked clears and cancels trajectory");
    force dut.position_ready=1'b1;
    key_release;key_press;milliseconds(100);
    @(negedge clk);force dut.motor_permitted=1'b0;clocks(2);
    check(dut.reference_offset==0&&!dut.reference_generator.moving,"fault permission revoked clears trajectory");

    // 独立真实balance实例检查接入误差和事务快照，不force其算法内部状态。
    @(negedge clk);seed_b=1;enable_b=1;clocks(1);
    @(negedge clk);seed_b=0;offset_b=130;
    balance_request;
    check(probe.reference_snapshot==130,"accepted transaction snapshots reference input");
    @(negedge clk);offset_b=260;
    clocks(12);
    check(probe.reference_snapshot==130,"mid-transaction input change cannot alter snapshot");
    check(probe.position_error==-130,"reference offset affects actual position error with correct sign");
    clocks(20);
    check(!fault_b&&!busy_b,"first reference transaction commits without fault");
    for(i=0;i<5;i=i+1)begin balance_request;clocks(35);end
    check(probe.position_error==-260,"later position transaction consumes new target offset");
    check(!fault_b,"updated reference remains legal");
    $display("PASS POSITION_MOVE_TOP checks=%0d physical_presses=%0d done_pulses=%0d accelerated_ms=9100 forced_ready_not_real_0p5s=1",checks,presses,done_count);
    $finish;
  end
  initial begin #10000000;$fatal(1,"watchdog");end
endmodule
