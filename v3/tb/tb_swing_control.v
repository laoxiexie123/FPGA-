`timescale 1ns/1ps
module tb_swing_control;
 reg clk=0; always #10 clk=~clk;
 reg rst_n=0,tick1=0,tick5=0,start=0,stop_req=0,safety=0;
 reg sample=0,valid=0,obsfault=0,commit=0,corefault=0;
 reg [9:0] average_raw=0;
 reg [11:0] filtered_adc_x4=0,zero_adc_x4=2000;
 reg expected_ready=0;
 integer raw_error,filtered_error;
 // Independent registered old-top capture contract; both supervisors consume this
 // preceding-cycle result. This prevents hiding a DUT register latency error.
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) expected_ready<=0;
  else begin
   raw_error=average_raw*4-zero_adc_x4;
   filtered_error=filtered_adc_x4-zero_adc_x4;
   expected_ready<=(raw_error>=-380 && raw_error<=380 &&
                    filtered_error>=-380 && filtered_error<=380);
  end
 end
 reg signed [31:0] angle=10000,speed=-100,arm=0;
 reg signed [15:0] core_duty=-60;
 wire enable,reseed,boost;wire [15:0] weight;wire signed [15:0] duty;
 wire [1:0] main;wire [3:0] phase;wire [7:0] fault;wire [31:0] ms;
 wire position_loop_enable,position_origin_seed,position_ready;
 swing_control dut(.clk_50m(clk),.rst_n(rst_n),.tick_1ms(tick1),.tick_5ms(tick5),
 .start_request(start),.stop_request(stop_req),.safety_fault(safety),
 .sample_commit(sample),.angle_valid(valid),.observer_fault(obsfault),
 .average_raw(average_raw),.filtered_adc_x4(filtered_adc_x4),.zero_adc_x4(zero_adc_x4),
 .angle_cdeg(angle),.speed_cdeg_s(speed),.arm_speed(arm),
 .core_duty(core_duty),.core_commit(commit),.core_fault(corefault),
 .core_enable(enable),.core_reseed_q(reseed),.position_weight_q15(weight),
 .entry_boost(boost),.duty_q(duty),.main_state_q(main),.phase_q(phase),
 .fault_q(fault),.state_ms_q(ms),.position_loop_enable(position_loop_enable),
 .position_origin_seed(position_origin_seed),.position_ready(position_ready));
 reg oldrun=0,oldseed=0,compare=0;reg [1535:0] cfg=0;
 wire oldenable,oldreseed,oldboost;wire [15:0] oldweight,blendweight;
 wire signed[15:0] oldduty;wire [3:0] oldphase;wire [7:0] oldfault;wire[31:0] oldms;
 swing_supervisor old(.clk(clk),.rst_n(rst_n),.tick_1ms(tick1),.tick_5ms(tick5),
 .run_enable(oldrun),.reseed(oldseed),.cfg(cfg),.pwm_limit(16'd210),
 .sample_commit(sample),.angle_valid(valid),.observer_fault(obsfault),
 .core_capture_ready(expected_ready),.angle_cdeg(angle),.speed_cdeg_s(speed),.arm_speed(arm),
 .core_duty(core_duty),.core_commit(commit),.core_fault(corefault),
 .core_enable_next(oldenable),.core_reseed_q(oldreseed),
 .position_weight_q15_next(oldweight),.entry_boost_next(oldboost),
 .duty_q(oldduty),.state_q(oldphase),.fault_q(oldfault),.state_ms_q(oldms),
 .blend_weight_q15_next(blendweight));
 integer cycles=0,checks=0,comparisons=0;
 always @(posedge clk) begin
  #1;cycles=cycles+1;
  if(dut.core_capture_ready_q !== expected_ready)
   $fatal(1,"capture-ready mismatch cycle%0d raw=%0d filtered=%0d zero=%0d",cycles,
          average_raw,filtered_adc_x4,zero_adc_x4);
  if(compare)begin
   comparisons=comparisons+1;
   if({duty,phase,enable,reseed,weight,boost,ms,fault} !==
      {oldduty,oldphase,oldenable,oldreseed,oldweight,oldboost,oldms,oldfault})
    $fatal(1,"old mismatch cycle%0d phase%0d/%0d duty%0d/%0d ms%0d/%0d",cycles,phase,oldphase,duty,oldduty,ms,oldms);
  end
 end
 task step;input integer n;begin repeat(n)@(negedge clk);end endtask
 task ck;input cond;begin checks=checks+1;if(cond!==1'b1)$fatal(1,"check%0d phase%0d fault%0d",checks,phase,fault);end endtask
 task millisecond;begin tick1=1;step(1);tick1=0;tick5=1;step(1);tick5=0;step(40);end endtask
 task boot;begin
  compare=0;start=0;stop_req=0;oldrun=0;valid=0;
  average_raw=0;filtered_adc_x4=0;zero_adc_x4=2000;sample=0;commit=0;
  safety=0;obsfault=0;corefault=0;tick1=0;tick5=0;rst_n=0;step(3);rst_n=1;step(4);
  start=1;oldrun=1;compare=1;step(1);ck(phase==13);step(30);
 end endtask
 task capture;begin
 average_raw=500;filtered_adc_x4=2000;step(1); // Register ready before new sample.
 angle=100;speed=-100;valid=1;sample=1;step(1);sample=0;
 ck(main==2 && reseed && boost);step(30);end endtask
 // Change arithmetic inputs at a falling edge and require readiness to hold
 // until the next rising edge; then check the inclusive +/-380 comparator.
 task ready_case;
 input [9:0] raw;
 input [11:0] filtered,zero_value;
 input expected;
 reg before_ready;
 begin
  before_ready=dut.core_capture_ready_q;
  average_raw=raw;filtered_adc_x4=filtered;zero_adc_x4=zero_value;
  #1;ck(dut.core_capture_ready_q===before_ready);
  step(1);ck(dut.core_capture_ready_q===expected);
 end endtask
 task ready_boundaries;begin
  compare=0;start=0;oldrun=0;rst_n=0;step(2);
  ck(dut.core_capture_ready_q===0);
  rst_n=1;
  ready_case(500,2380,2000,1);ready_case(500,2381,2000,0);
  ready_case(500,1620,2000,1);ready_case(500,1619,2000,0);
  ready_case(595,2000,2000,1);ready_case(596,2000,2000,0);
  ready_case(405,2000,2000,1);ready_case(404,2000,2000,0);
  ready_case(500,1619,1619,0);ready_case(500,2381,2381,0); // Raw +/-381.
  ready_case(1023,4092,0,0);ready_case(0,4095,0,0);
  ready_case(0,0,4095,0);
  ready_case(1023,4092,4092,1);ready_case(0,0,0,1);
  // Asynchronous reset clears a previously true register before a clock.
  #3;rst_n=0;#1;ck(dut.core_capture_ready_q===0);step(2);rst_n=1;
 end endtask
 // Isolated session cancellation: deposit state only after old-path comparison ends.
 task origin_cancel;input integer cause;begin
  compare=0;stop_req=0;safety=0;obsfault=0;corefault=0;sample=0;
  dut.state_q=6;dut.fault_q=0;dut.core_reseed_q=0;
  dut.origin_delay_count_q=123;dut.position_origin_locked_q=1;
  dut.position_origin_pulse_q=1;
  case(cause)
   0:stop_req=1;
   1:safety=1;
   2:obsfault=1;
   3:corefault=1;
   4:dut.core_reseed_q=1;
   5:dut.state_q=5;
   6:dut.fault_q=4;
  endcase
  #1;ck(!position_origin_seed && !position_ready && position_loop_enable);
  step(1);ck(!position_loop_enable && !position_origin_seed && !position_ready &&
             dut.origin_delay_count_q==0);
  stop_req=0;safety=0;obsfault=0;corefault=0;
 end endtask
 integer i;
 initial begin
 cfg[21*32+:32]=3;cfg[22*32+:32]=210;cfg[23*32+:32]=200;
 cfg[25*32+:32]=210;cfg[27*32+:32]=100;cfg[28*32+:32]=2500;
 cfg[29*32+:32]=100000;cfg[30*32+:32]=800;cfg[31*32+:32]=5;
 cfg[32*32+:32]=5;cfg[33*32+:32]=200;cfg[38*32+:32]=200000;
 cfg[40*32+:32]=2;cfg[41*32+:32]=8500;cfg[42*32+:32]=3200;
 cfg[43*32+:32]=10000;cfg[45*32+:32]=1;cfg[46*32+:32]=1;
 ready_boundaries;
 boot;
 for(i=0;i<200;i=i+1)millisecond;ck(phase==11);
 for(i=0;i<100;i=i+1)millisecond;ck(phase==14);
 valid=1;angle=8000;speed=-100;sample=1;step(1);sample=0;ck(phase==15);step(30);
 for(i=0;i<200;i=i+1)millisecond;ck(phase==11);
 for(i=0;i<100;i=i+1)millisecond;ck(phase==12);
 capture;commit=1;step(1);commit=0;ck(phase==5);step(30);
 // Allow the original Bresenham 32768 increments to execute; no compressed blend arithmetic.
 for(i=0;i<24;i=i+1)begin
 tick1=1;commit=1;step(1);tick1=0;commit=0;step(8000);
 tick5=1;step(1);tick5=0;step(30);
 end
 ck(main==3 && weight==32768 && boost);
 ck(!position_loop_enable && !position_origin_seed && !position_ready);
 // Inject only near-terminal count; retain two real edges and seed/ready timing.
 dut.origin_delay_count_q=25'd24999998;
 step(1);ck(dut.origin_delay_count_q==24999999 && !position_loop_enable &&
            !position_origin_seed && !position_ready);
 step(1);ck(position_loop_enable && position_origin_seed && !position_ready);
 step(1);ck(position_loop_enable && !position_origin_seed && position_ready);
 step(3);ck(position_ready && !position_origin_seed);core_duty=150;step(30);
 tick5=1;step(1);tick5=0;step(30);ck(duty==150);
 // Boost remains independent of main balance state and expires after original 2000 ticks.
 commit=1;for(i=0;i<2000;i=i+1)millisecond;commit=0;ck(main==3 && !boost);
 compare=0;
 // Direct safety gate suppresses ready/seed before the next clock; loop remains locked until it.
 #1;safety=1;#1;ck(!position_ready && !position_origin_seed && position_loop_enable);
 safety=0;#1;ck(position_ready);
 valid=0;sample=1;step(1);sample=0;ck(main==0 && fault==5 && duty==0);
 ck(!position_ready && !position_origin_seed);
 step(1);ck(!position_loop_enable && dut.origin_delay_count_q==0);
 step(100);ck(main==0 && fault==5); // held start must not retry
 start=0;step(1);start=1;step(1);ck(main==1 && fault==0);
 stop_req=1;tick5=1;step(1);stop_req=0;tick5=0;step(30);ck(main==0 && duty==0);
 boot;capture;compare=0;
 for(i=0;i<20;i=i+1)millisecond;ck(main==0 && fault==6 && duty==0);
 boot;compare=0;dut.state_ms_q=200;step(1);step(30);
 dut.state_ms_q=100;step(1);step(30);ck(phase==14);
 dut.wait_ms_q=2000;step(1);ck(main==0 && fault==3);
 boot;compare=0;dut.elapsed_ms_q=10000;step(1);ck(main==0 && fault==3);
 boot;capture;compare=0;corefault=1;step(1);corefault=0;ck(main==0 && fault==4);
 boot;compare=0;safety=1;step(1);safety=0;ck(main==0 && duty==0 && fault==4);
 boot;compare=0;dut.state_q=4'd9;step(1);ck(main==0 && fault==1 && duty==0);
 for(i=0;i<7;i=i+1)origin_cancel(i);
 dut.state_q=6;dut.fault_q=0;dut.core_reseed_q=0;
 dut.position_origin_locked_q=1;dut.position_origin_pulse_q=1;dut.origin_delay_count_q=123;
 #3;rst_n=0;#1;ck(!position_loop_enable && !position_origin_seed && !position_ready &&
                      dut.origin_delay_count_q==0);
 $display("PASS SWING cycles=%0d checks=%0d old_comparisons=%0d",cycles,checks,comparisons);$finish;
 end
 initial begin #100000000;$fatal(1,"timeout");end
endmodule
