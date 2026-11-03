`timescale 1ns/1ps
module tb_pendulum_observer;
 reg clk_50m=0; always #10 clk_50m=~clk_50m;
 reg rst_n=0,tick_5ms=0,reseed=0,adc_fresh=1;
 reg [9:0] adc_raw=512; reg [11:0] zero_adc_x4=2048;
 wire commit,busy,valid,fault; wire signed [31:0] angle,speed;wire [31:0] seq;
 wire oc,ob,ov,ofault;wire signed[31:0] oa,os;wire[31:0] oq;
 reg [1535:0] cfg;
 adc_pend dut(.clk_50m(clk_50m),.rst_n(rst_n),.tick_5ms(tick_5ms),
 .filter_clear(reseed),.tick_1ms(1'b0),.adc_data_in(10'd0),.zero_adc_x4(zero_adc_x4),
 .adc_clk_next(),.adc_oe_n_next(),.average_raw_q(),.average_valid_q(),
 .angle_x4_q(),.angle_valid_q(),.filter_ready_q(),.average_seen_q(),
 .average_age_ms_q(),.average_fresh_next(),
 .sample_commit_q(commit),.observer_busy_q(busy),.pendulum_valid_q(valid),.angle_cdeg_q(angle),
 .speed_cdeg_s_q(speed),.observer_seq_q(seq),.observer_fault_q(fault));
 // Isolate merged observation logic only, not the full acquisition chain.
 // Continuous force follows input changes; ADC regression covers physical sampling.
 initial begin
  force dut.average_raw_q=adc_raw;
  force dut.average_fresh_next=adc_fresh;
 end
 always @* begin
  cfg=0;cfg[6*32+:32]={20'd0,zero_adc_x4};
  cfg[34*32+:32]=8518;cfg[36*32+:32]=1;cfg[37*32+:32]=1022;
  cfg[38*32+:32]=200000;cfg[39*32+:32]=16;cfg[40*32+:32]=2;cfg[47*32+:32]=18000;
 end
 swing_observer old(.clk(clk_50m),.rst_n(rst_n),.tick_5ms(tick_5ms),.reseed(reseed),
 .adc_raw({6'd0,adc_raw}),.adc_fresh(adc_fresh),.cfg(cfg),.sample_commit_q(oc),
 .busy_q(ob),.angle_valid_q(ov),.angle_cdeg_q(oa),.speed_cdeg_s_q(os),
 .energy_q16_next(),.energy_duty_next(),.observer_seq_q(oq),.arithmetic_fault_q(ofault));
 integer cycles=0,checks=0,commits=0,models=0,stage_mask=0,i,j,n;
 integer history=0,previous=0,y=0,ma,rawv,my,ms,expected_delay;
 reg[31:0] saved_seq,rng=32'h2178beef;
 always @(posedge clk_50m) begin
  #1;cycles=cycles+1;
  if(rst_n) begin
   checks=checks+1;
   if({commit,busy,valid,fault,angle,speed,seq} !== {oc,ob,ov,ofault,oa,os,oq})
    $fatal(1,"old/new mismatch cycle=%0d stage=%0d new=%b old=%b",cycles,dut.obs_state_q,
     {commit,busy,valid,fault,angle,speed,seq},{oc,ob,ov,ofault,oa,os,oq});
   if(commit)commits=commits+1;
  end
 end
 task step;input integer count;begin repeat(count)@(negedge clk_50m);end endtask
 task ck;input condition;begin if(condition!==1'b1)$fatal(1,"assert cycle=%0d",cycles);end endtask
 task clear_history;begin reseed=1;step(1);reseed=0;history=0;previous=0;y=0;end endtask
 // Independent integer arithmetic uses division, no Q8 multiplication pipeline copy.
 task sample;
 input integer code;
 integer invalid_sample,rawangle,inc,age;
 begin
  adc_raw=code; rawangle=(code*4-zero_adc_x4)*8518;
  invalid_sample=(code<1 || code>1022 || rawangle>18432000 || rawangle< -18432000);
  ma=rawangle/1024;expected_delay=4;
  if(invalid_sample)expected_delay=(code<1 || code>1022)?2:4;
  else if(history!=0)begin
   rawv=(ma-previous)*200;
   if(ma-previous>18000 || ma-previous< -18000)begin invalid_sample=1;expected_delay=6;end
   else if(rawv>200000 || rawv< -200000)begin invalid_sample=1;expected_delay=7;end
   else begin inc=(rawv*256-y)/16;my=y+inc;ms=my/256;expected_delay=11;end
  end
  tick_5ms=1;step(1);tick_5ms=0;
  age=1;
  while(!commit && age<20)begin step(1);age=age+1;end
  ck(commit && age==expected_delay);
  if(invalid_sample)begin ck(!valid && angle==0 && speed==0);history=0;previous=0;y=0;end
  else if(history==0)begin ck(!valid && angle==ma && speed==0);history=1;previous=ma;y=0;end
  else begin ck(valid && angle==ma && speed==ms);previous=ma;y=my;history=history+1;end
  models=models+1;step(1);ck(!commit);
 end endtask
 initial begin
  #1 rst_n=0;zero_adc_x4=0;#1 zero_adc_x4=2048;#23 rst_n=1;step(2);
  sample(512);sample(513);sample(512);sample(511);sample(512);
  for(i=0;i<100;i=i+1)begin rng={rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};sample(500+rng%25);end
  sample(0);sample(1023);sample(512);sample(1022);sample(1);
  zero_adc_x4=0;sample(1022);zero_adc_x4=4092;sample(1);
  zero_adc_x4=2048;sample(512);sample(543);sample(512);sample(900);
  for(j=4093;j<4096;j=j+1)begin
   clear_history;zero_adc_x4=j;tick_5ms=1;step(1);tick_5ms=0;ck(fault && !busy && !commit);
   saved_seq=seq;step(2);ck(fault && seq==saved_seq);zero_adc_x4=2048;clear_history;
  end
  // All in-flight normal stages: reseed, freshness cancellation, then busy tick.
  for(i=1;i<=10;i=i+1)begin
   clear_history;sample(512);adc_raw=513;tick_5ms=1;step(1);tick_5ms=0;
   while(dut.obs_state_q!=i)step(1);
   stage_mask=stage_mask | (1<<i);saved_seq=seq;reseed=1;step(1);reseed=0;
   ck(!busy && !valid && !commit && !fault && seq==saved_seq);
   history=0;previous=0;y=0;
   sample(512);adc_raw=513;tick_5ms=1;step(1);tick_5ms=0;
   while(dut.obs_state_q!=i)step(1);
   adc_fresh=0;step(1);ck(!busy && !valid && !commit && angle==0);adc_fresh=1;clear_history;
   sample(512);adc_raw=513;tick_5ms=1;step(1);tick_5ms=0;
   while(dut.obs_state_q!=i)step(1);
   tick_5ms=1;step(1);tick_5ms=0;ck(fault && !busy && !commit);clear_history;
  end
  adc_fresh=0;tick_5ms=1;saved_seq=seq;step(1);ck(commit && seq==saved_seq+1 && !fault);
  step(1);ck(commit && seq==saved_seq+2);tick_5ms=0;adc_fresh=1;clear_history;
  // Fresh cancellation outranks a busy tick; fault does not arise.
  sample(512);adc_raw=513;tick_5ms=1;step(1);tick_5ms=0;step(2);
  adc_fresh=0;tick_5ms=1;step(1);ck(commit && !fault && !busy);adc_fresh=1;tick_5ms=0;clear_history;
  // Zero is snapshotted at acceptance, not read again during mapping.
  adc_raw=512;tick_5ms=1;step(1);tick_5ms=0;zero_adc_x4=4095;
  step(3);ck(commit && angle==0 && !fault);zero_adc_x4=2048;clear_history;
  sample(512);sample(513);#3 rst_n=0;#2;ck(!busy && !valid && !fault && seq==0);
  #22 rst_n=1;step(3);history=0;sample(512);sample(513);
  // Arithmetic fault path and illegal stage are unreachable with valid fixed inputs;
  // paired state injection exercises the retained defensive checks explicitly.
  clear_history;dut.obs_state_q=8;old.state_q=8;dut.observer_busy_q=1;old.busy_q=1;
  dut.obs_filter_sum_q=64'sd2147483648;old.filter_sum_q=64'sd2147483648;
  step(1);ck(fault && !busy && !valid);clear_history;
  dut.obs_state_q=15;old.state_q=15;step(1);ck(fault);clear_history;
  // Sequence rollover and fresh/reseed priorities retain legacy semantics.
  dut.observer_seq_q=32'hffffffff;old.observer_seq_q=32'hffffffff;
  adc_fresh=0;tick_5ms=1;step(1);ck(commit && seq==0 && !fault);
  reseed=1;step(1);ck(!commit && seq==0 && !fault);reseed=0;
  tick_5ms=0;adc_fresh=1;clear_history;
  // Map boundary: delta 2163 remains below +/-180deg, delta2164 exceeds it.
  zero_adc_x4=1;sample(541);zero_adc_x4=0;sample(541);
  zero_adc_x4=2167;sample(1);zero_adc_x4=2168;sample(1);
  zero_adc_x4=2048;clear_history;
  sample(1);sample(1022); // mapped endpoints are valid, their jump exceeds18000.
  clear_history;sample(512);sample(542); // 998cdeg step, raw speed199600 is allowed.
  sample(573); // 1032cdeg step, raw speed206400 is rejected.
  ck(stage_mask==2046);
  $display("PASS OBSERVER cycles=%0d checks=%0d commits=%0d models=%0d stage_mask=%0d",cycles,checks,commits,models,stage_mask);
  $finish;
 end
 initial begin #20000000;$fatal(1,"timeout");end
endmodule
