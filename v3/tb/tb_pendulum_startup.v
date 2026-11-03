`timescale 1ns/1ps
module tb_pendulum_startup;
  reg clk=0;always #10 clk=~clk;
  reg rst_n=0,start=0,clear=0,stop=0;
  wire commit,core_fault,adc_ready,position_enabled,origin_seed,position_ready;
  wire signed [15:0] core_command;
  wire signed [31:0] adc_q8;
  wire [15:0] position_weight;
  reg signed [31:0] arm_position=0;
  // 仅测试台观察原算法历史，不增设可综合诊断端口或影子状态。
  wire signed [31:0] filtered_position=balance.position_filter_history>>>8;
  wire signed [31:0] position_offset=balance.position_offset;
  integer position_updates=0;
  integer origin_checks=0;
  integer cycles=0;
  wire t1=(cycles%50000==49999),t5=(cycles%250000==249999);
  wire core_enable,reseed,valid,obs_commit,fault,boost;
  wire [1:0] state;
  wire [7:0] reason;
  wire signed [31:0] angle,speed;
  wire signed [15:0] duty;
  integer samples=0;
  wire [9:0] average_raw;
  wire [11:0] filtered_adc;
  // Testbench wiring directly represents the future top; no extra RTL wrapper.
  adc_pend adc(.clk_50m(clk),.rst_n(rst_n),.filter_clear(clear),.tick_1ms(t1),
      .tick_5ms(t5),.zero_adc_x4(12'd2000),.adc_data_in(10'd500),
      .adc_clk_next(),.adc_oe_n_next(),.average_raw_q(average_raw),.average_valid_q(),
      .angle_x4_q(filtered_adc),.filtered_adc_q8(adc_q8),.angle_valid_q(),
      .filter_ready_q(adc_ready),.average_seen_q(),
      .average_age_ms_q(),.average_fresh_next(),.sample_commit_q(obs_commit),
      .observer_busy_q(),.pendulum_valid_q(valid),.angle_cdeg_q(angle),
      .speed_cdeg_s_q(speed),.observer_seq_q(),.observer_fault_q(fault));
  swing_control control(.clk_50m(clk),.rst_n(rst_n),.tick_1ms(t1),.tick_5ms(t5),
      .start_request(start),.stop_request(stop||clear),.safety_fault(1'b0),
      .average_raw(average_raw),.filtered_adc_x4(filtered_adc),.zero_adc_x4(12'd2000),
      .arm_speed(32'sd0),.core_duty(core_command),.core_commit(commit),.core_fault(core_fault),
      .sample_commit(obs_commit),.angle_valid(valid),.observer_fault(fault),
      .angle_cdeg(angle),.speed_cdeg_s(speed),.core_enable(core_enable),
      .core_reseed_q(reseed),.position_weight_q15(position_weight),.entry_boost(boost),
      .position_loop_enable(position_enabled),.position_origin_seed(origin_seed),
      .position_ready(position_ready),
      .duty_q(duty),.main_state_q(state),.phase_q(),.fault_q(reason),.state_ms_q());
  balance_control balance(.clk_50m(clk),.rst_n(rst_n),.tick_5ms(t5),
      .enable(core_enable),.clear(clear),.capture_seed(reseed),.entry_boost(boost),
      .position_loop_enable(position_enabled),.position_origin_seed(origin_seed),
      .filtered_adc_x4(filtered_adc),.filtered_adc_q8(adc_q8),.adc_ready(adc_ready),
      .zero_adc_x4(12'd2000),.position_counts(arm_position),
      .position_reference_offset(32'sd0),.position_origin_counts(),
      .position_weight_q15(position_weight),.motor_command(core_command),.result_valid(commit),
      .calculating(),.control_fault(core_fault),.fault_code());
  // 仅仿真计数：position_commit取NBA前的本沿条件，NBA后等待者才看到递增。
  // 不使用延迟一拍的commit重建位置事件，也不在RTL恢复position_sequence。
  always @(posedge clk) begin
    if(!rst_n)begin cycles<=0;position_updates<=0;end
    else begin
      cycles<=cycles+1;
      if(obs_commit)samples<=samples+1;
      if(balance.position_commit)position_updates<=position_updates+1;
    end
  end
  initial begin
    #1;rst_n=0;#24;rst_n=1;
    @(negedge clk);start=1;
    wait(state==3);@(negedge clk);
    if(!valid||fault||reason!=0||angle!=0||speed!=0||!boost||samples<2)
      $fatal(1,"startup sensor/handshake mismatch");
    if(core_fault||position_enabled||position_ready)
      $fatal(1,"balance/origin state before 0.5 seconds");
    wait(duty==0);@(negedge clk);
    // Full acquisition uses real clock/ticks. Only the long origin wait is preloaded;
    // its terminal two cycles and actual balance-core seed consumption are exercised.
    if(control.origin_delay_count_q<1 || position_enabled)
      $fatal(1,"origin timer did not begin after entering balance");
    arm_position=1234;
    control.origin_delay_count_q=25'd24999998;
    @(posedge clk);#1;
    if(position_enabled||origin_seed||position_ready)$fatal(1,"origin enabled early");
    @(posedge clk);#1;
    if(!position_enabled||!origin_seed||position_ready)$fatal(1,"origin seed edge mismatch");
    origin_checks=origin_checks+1;
    @(posedge clk);#1;
    if(!position_ready||origin_seed||filtered_position!=1234||position_offset!=0||core_fault)
      $fatal(1,"real balance did not consume neutral origin seed");
    wait(position_updates!=0);@(negedge clk);
    if(core_fault||reason!=0||duty!=0)$fatal(1,"origin first position update failed");
    origin_checks=origin_checks+1;
    stop=1;
    @(posedge clk);#1;
    if(state!=0||duty!=0)$fatal(1,"stop did not clear output");
    @(negedge clk);stop=0;
    repeat(100)@(negedge clk);
    if(state!=0)$fatal(1,"held start retried");
    start=0;@(negedge clk);start=1;wait(state==1);
    @(negedge clk);clear=1;@(posedge clk);#1;
    if(state!=0||duty!=0||valid)$fatal(1,"measurement clear did not abort startup");
    $display("PASS STARTUP real_cycles=%0d samples=%0d origin_checks=%0d actual_PD=1 origin_wait_preloaded=1",cycles,samples,origin_checks);$finish;
  end
  initial begin #150000000;$fatal(1,"startup timeout state=%0d reason=%0d",state,reason);end
endmodule
