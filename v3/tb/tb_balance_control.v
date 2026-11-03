`timescale 1ns/1ps
// II级加速使能仿真：旧完整核心提供ADC低通，两个控制器同沿读取相同Q8/x4。
// 保留的五个控制输出逐拍对照；测试专用层次探针不增加RTL诊断硬件。
// 外部ADC接口非法值另行测，不声称保留旧ADC故障耦合。
module tb_balance_control;
 reg clk=0; always #10 clk=~clk;
 reg rst_n=0,tick=0,enable=0,clear=0,capture=0,boost=0,pos_enable=1,origin=0;
 reg [15:0] raw=500; reg adc_valid=0;
 reg [11:0] zero=2000;
 reg signed [31:0] position=0;
 reg ref_valid=0; reg signed [63:0] reference=0;
 reg [15:0] weight=32768; reg [31:0] ki=0;
 reg [671:0] cfg;
 reg override_adc=0; reg [11:0] forced_x4=0;reg signed [31:0] forced_q8=0;
 wire signed [31:0] old_adc;
 wire signed [15:0] old_duty;wire old_commit,old_busy,old_fault;
 wire [7:0] old_code;
 wire signed [15:0] duty;wire valid,busy,fault;
 wire [7:0] code;
 // 仅测试台计数，不综合为诊断硬件；提交条件在NBA前采样。
 integer tb_control_count=0,tb_position_count=0;
 reg position_commit_this_edge=0;
 wire [11:0] adc_x4=override_adc?forced_x4:old_adc[11:0];
 wire signed [31:0] adc_q8=override_adc?forced_q8:legacy.adc_q8_q;
 dual_pd_core legacy(.clk(clk),.rst_n(rst_n),.tick_5ms(tick),.run_enable(enable),.reseed(clear),
   .adc_raw(raw),.adc_valid(adc_valid),.position(position),.cfg(cfg),.pwm_limit(16'd150),
   .position_reference_valid(ref_valid),.position_reference_q16(reference),.position_tick_next(),
   .position_weight_q15(weight),.capture_seed(capture),.entry_boost(boost),.position_loop_enable(pos_enable),
   .position_origin_seed(origin),.commit_q(old_commit),.busy_q(old_busy),.adc_filtered_x4_q(old_adc),
   .position_filtered_q(),.error_angle_q(),.delta_angle_q(),
   .error_position_q(),.delta_position_q(),.pwm1_q(),.pwm2_q(),.mix_q(),
   .duty_q(old_duty),.fault_reason_q(old_code),.control_seq_q(),.position_seq_q(),
   .arithmetic_fault_q(old_fault),.position_ki_q16(ki),.position_integral_next());
 balance_control dut(.clk_50m(clk),.rst_n(rst_n),.tick_5ms(tick),.enable(enable),.clear(clear),
   .capture_seed(capture),.entry_boost(boost),.position_loop_enable(pos_enable),.position_origin_seed(origin),
   .filtered_adc_x4(adc_x4),.filtered_adc_q8(adc_q8),.adc_ready(legacy.adc_seeded_q),
   .zero_adc_x4(zero),.position_counts(position),.position_weight_q15(weight),
    .position_reference_offset(32'sd0),.position_origin_counts(),
   .motor_command(duty),.result_valid(valid),.calculating(busy),.control_fault(fault),.fault_code(code));
 integer cycles=0,comparisons=0,checks=0,commits=0,models=0,i,j;
 integer anti_outer=0,anti_motor=0,freeze_cases=0;
 reg compare=1,model_enable=1,model_pending=0,model_history=0;
 reg signed [63:0] model_previous=0,model_error,model_fine,model_delta,model_sum,model_angle,model_duty;
 reg signed [63:0] model_kp,model_kd;
 reg [27:0] stage_mask=0;
 reg [4:0] direct_wait_mask=0;
 integer accepted_cycle=0,latency_angle=0,latency_position=0,latency_weighted=0;
 reg accepted_position=0,accepted_weighted=0;
 reg [5:0] injected_state;
 integer deleted_checks=0;
 function signed [63:0] trunc16;
  input signed [63:0] x;
  begin trunc16=(x<0)?-((-x)>>>16):(x>>>16);end
 endfunction
 task check;
  input condition;input [511:0] message;
  begin checks=checks+1;if(condition!==1'b1)$fatal(1,"CHECK %0s cycle=%0d",message,cycles);end
 endtask
 // 功能：接受沿独立角度数值模型及覆盖计数；clk域，无硬件实现含义。
 always @(posedge clk) begin
  cycles=cycles+1;
  // 仅仿真观察本沿位置提交；NBA后再计数，不重新引入RTL诊断寄存器。
  position_commit_this_edge=dut.position_commit;
  if(rst_n && dut.state_q>=32 && dut.state_q<=36)direct_wait_mask[dut.state_q-32]=1;
  if(rst_n && enable && !clear && !capture && !origin && !fault && tick && dut.state_q==0 && legacy.adc_seeded_q)begin
    accepted_cycle=cycles;accepted_position=pos_enable && dut.position_holds_q==0;
    accepted_weighted=weight!=32768;
  end
  if(rst_n && dut.state_q<=19)stage_mask[dut.state_q]=1;
  if(!rst_n || clear || capture || !enable || fault)begin model_history=0;model_pending=0;end
  else if(origin || (!pos_enable && dut.position_loop_active_q))model_pending=0;
  else if(model_enable && tick && !busy && legacy.adc_seeded_q)begin
   model_error=$signed({1'b0,adc_x4})-$signed({1'b0,zero})+(pos_enable?$signed(dut.position_offset):0);
   model_fine=$signed(adc_q8)-($signed({1'b0,zero})*64)+(pos_enable?$signed(dut.position_offset)*64:0);
   model_delta=model_history?(model_fine-model_previous):0;
   model_kp=boost?120000:80000;model_kd=boost?525000:350000;
   model_sum=model_error*model_kp+((model_delta*model_kd)>>>6);
   model_angle=trunc16(model_sum);
   model_duty=model_angle>0?model_angle+1:model_angle<0?model_angle-1:0;
   if(model_duty>150)model_duty=150;if(model_duty< -150)model_duty=-150;
   model_pending=1;
  end

  #1;
  if(!rst_n)begin tb_control_count=0;tb_position_count=0;end
  else if(valid)begin
   tb_control_count=tb_control_count+1;
   if(position_commit_this_edge)tb_position_count=tb_position_count+1;
  end
  if(valid)begin
    if(!accepted_position)latency_angle=cycles-accepted_cycle;
    else if(accepted_weighted)latency_weighted=cycles-accepted_cycle;
    else latency_position=cycles-accepted_cycle;
  end
  if(rst_n && compare)begin
   comparisons=comparisons+1;
   if({valid,busy,fault,code,duty} !== {old_commit,old_busy,old_fault,old_code,old_duty})
    $fatal(1,"legacy retained-output mismatch cycle=%0d state=%0d/%0d fault=%0d/%0d duty=%0d/%0d",cycles,dut.state_q,legacy.state_q,code,old_code,duty,old_duty);
  end
  if(valid)begin
   commits=commits+1;
   if(model_enable && model_pending)begin
    // 仅测试读取原算法候选，不要求RTL保留诊断端口或影子寄存器。
    if(dut.angle_error!==model_error[31:0] || dut.angle_difference!==model_delta[31:0] ||
       dut.angle_command_candidate!==model_angle[31:0] || duty!==model_duty[15:0])
     $fatal(1,"independent angle model cycle=%0d error=%0d/%0d delta=%0d/%0d command=%0d/%0d duty=%0d/%0d",cycles,dut.angle_error,model_error,dut.angle_difference,model_delta,dut.angle_command_candidate,model_angle,duty,model_duty);
    models=models+1;model_previous=model_fine;model_history=1;model_pending=0;
   end
  end
 end
 task clocks;input integer count;begin repeat(count)@(negedge clk);end endtask
 task send_adc;input [15:0] value;begin
  @(negedge clk);raw=value;adc_valid=1;@(negedge clk);adc_valid=0;clocks(5);
 end endtask
 task pulse_clear;begin
  @(negedge clk);clear=1;tick=0;adc_valid=0;@(negedge clk);clear=0;
  clocks(2);send_adc(500);clocks(2);
 end endtask
 task transaction;
  integer timeout;
  begin
   @(negedge clk);tick=1;@(negedge clk);tick=0;timeout=0;
   while(!valid && !fault && timeout<100)begin @(negedge clk);timeout=timeout+1;end
   check(valid&&!fault,"control transaction must commit without fault");
   @(negedge clk);
  end
 endtask
 // 真实计算到commit前沿，不force状态；五类抢占不得发布旧事务。
 task preempt_commit;
  input integer event_kind;
  integer saved_control_count,saved_position_count;
  reg signed [15:0] saved_duty;
  reg signed [31:0] saved_angle_history,saved_position_history;
  reg saved_angle_history_valid;
  integer timeout;
  begin
   model_enable=0;model_pending=0;
   enable=1;pos_enable=1;origin=0;capture=0;clear=0;
   transaction; // 建立已发布输出，再启动一份会被取消的新事务。
   @(negedge clk);tick=1;@(negedge clk);tick=0;timeout=0;
   while(dut.state_q!=16 && !fault && timeout<100)begin clocks(1);timeout=timeout+1;end
   check(dut.state_q==16&&legacy.state_q==16&&!fault,"real calculation reached commit boundary");
   saved_control_count=tb_control_count;saved_position_count=tb_position_count;saved_duty=duty;
   saved_angle_history=dut.previous_angle_error;saved_position_history=dut.previous_position_error;
   saved_angle_history_valid=dut.angle_history_valid_q;
   case(event_kind)
    0:clear=1;
    1:capture=1;
    2:origin=1;
    3:enable=0;
    4:pos_enable=0;
   endcase
   @(posedge clk);#2;
   check(!valid&&!busy&&tb_control_count==saved_control_count&&tb_position_count==saved_position_count,
         "commit preemption prevents publication and test-only event count advance");
   if(event_kind==0 || event_kind==1 || event_kind==3)
    check(duty==0,"clear capture stop clear output instead of publishing pending command");
   else check(duty==saved_duty,"origin and position disable preserve published motor output");
   // 只读算法历史，验证取消未提交新候选；不恢复被删除的诊断寄存器。
   if(event_kind==1)check(dut.previous_angle_error==0&&dut.previous_position_error==0,
                         "capture clears history without committing pending values");
   else begin
    check(dut.previous_angle_error==saved_angle_history,"cancel preserves previous angle history");
    if(event_kind==2 || event_kind==4)
     check(dut.previous_position_error==0&&dut.position_offset==0&&
           dut.angle_history_valid_q==saved_angle_history_valid,
           "position cancellation clears position history and offset but preserves angle validity");
    else check(dut.previous_position_error==saved_position_history,
               "clear and stop do not publish pending position history");
   end
   @(negedge clk);clear=0;capture=0;origin=0;enable=1;pos_enable=1;
   // clear清旧ADC仅属于legacy；重新采样后再共同接受下一次事务。
   if(event_kind==0)send_adc(500);
   clocks(2);
  end
 endtask
 task expect_fault;input [7:0] expected;begin
  clocks(3);check(fault&&code==expected&&duty==0&&!busy,"fault code and cleared output");
 end endtask
 initial begin
  cfg=0;cfg[0+:32]=80000;cfg[32+:32]=350000;cfg[64+:32]=10000;cfg[96+:32]=500000;
  cfg[128+:32]=128;cfg[160+:32]=64;cfg[192+:32]=2000;cfg[608+:32]=150;cfg[640+:32]=1;
  #1;rst_n=0;#24;rst_n=1;clocks(3);enable=1;
  // 首样本、默认路径、正负小数差分与位置每1/6/11提交。
  send_adc(500);
  for(i=0;i<26;i=i+1)begin
   send_adc(499+(i%3));position=(i%7)-3;transaction;
   check(tb_position_count==((tb_control_count+4)/5),"position cadence first sixth eleventh commits");
  end
  // 输入在事务接受时锁存；增强只提升角度P/D。
  boost=1;for(i=0;i<12;i=i+1)begin send_adc(497+(i%7));position=i-6;transaction;end
  boost=0;pos_enable=0;clocks(2);for(i=0;i<6;i=i+1)transaction;
  check(dut.position_offset==0,"position disabled clears algorithm offset");
  pos_enable=1;position=-37;origin=1;clocks(1);origin=0;transaction;
  check(dut.position_error==0&&dut.position_difference==0,"origin seeds neutral first position error");
  // 接受后改变实时boost，模型仍使用接受沿增益。
  boost=1;@(negedge clk);tick=1;@(negedge clk);tick=0;boost=0;
  while(!valid&&!fault)clocks(1);
  check(valid&&!fault,"accepted boost snapshot survives live input change");clocks(1);
  // PD-only partial weight and saturation coverage.
  position=2;weight=16384;
  for(i=0;i<15;i=i+1)begin send_adc(500);transaction;end
  weight=32768;position=-4;for(i=0;i<20;i=i+1)transaction;
  position=100000;pulse_clear;for(i=0;i<12;i=i+1)transaction;
  position=0;pulse_clear;pos_enable=0;clocks(2);send_adc(600);transaction;
  pos_enable=1;position=1;for(i=0;i<15;i=i+1)transaction;
  // capture保留ADC和锁存故障；停止、原点和关闭沿取消在途事务。
  @(negedge clk);capture=1;clocks(1);capture=0;
  check(legacy.adc_seeded_q,"capture preserves ADC seed");transaction;check(dut.angle_difference==0&&dut.position_difference==0,"capture makes first derivatives zero");
  @(negedge clk);tick=1;clocks(1);tick=0;clocks(2);enable=0;clocks(2);
  check(!busy&&duty==0&&!valid,"stop cancels in-flight calculation");enable=1;transaction;
  @(negedge clk);tick=1;clocks(1);tick=0;clocks(2);origin=1;clocks(1);origin=0;
  check(!busy&&!valid,"origin cancels in-flight calculation");transaction;
  @(negedge clk);tick=1;clocks(1);tick=0;clocks(2);pos_enable=0;clocks(2);
  check(!busy&&!valid,"position disable cancels in-flight calculation");pos_enable=1;transaction;
  // 五个真正commit同沿抢占：clear/capture/origin/stop/位置关闭。
  for(j=0;j<5;j=j+1)preempt_commit(j);
  // 输入故障优先级及故障保留。
  position=1000001;expect_fault(1);capture=1;clocks(1);capture=0;check(fault&&code==1,"capture preserves locked fault");
  position=0;pulse_clear;
  @(negedge clk);tick=1;clocks(2);tick=0;expect_fault(5);pulse_clear;
  // 强制旧/new相同运算阶段，检验算术溢出、非法状态与同沿origin故障优先。
  model_enable=0;
  @(negedge clk);force dut.state_q=5;force legacy.state_q=5;
  force dut.proportional_derivative_sum=64'sh7fffffffffffffff;force legacy.pd_sum_q=64'sh7fffffffffffffff;
  clocks(1);release dut.state_q;release legacy.state_q;release dut.proportional_derivative_sum;release legacy.pd_sum_q;
  expect_fault(6);pulse_clear;
  @(negedge clk);force dut.state_q=15;force legacy.state_q=15;
  force dut.compensated_motor_command=64'sh7fffffffffffffff;force legacy.work_mix_q=64'sh7fffffffffffffff;
  origin=1;clocks(1);origin=0;release dut.state_q;release legacy.state_q;release dut.compensated_motor_command;release legacy.work_mix_q;
  expect_fault(7);pulse_clear;
  @(negedge clk);force dut.state_q=31;force legacy.state_q=31;clocks(1);
  release dut.state_q;release legacy.state_q;expect_fault(8);pulse_clear;
  // 外部ADC接口的新增责任：仅在adc_ready时检查范围，不再检测旧内部ADC busy/raw。
  compare=0;override_adc=1;forced_x4=4093;forced_q8=128000;expect_fault(3);
  forced_x4=2000;pulse_clear;forced_q8=-1;expect_fault(3);
  forced_q8=261889;pulse_clear;expect_fault(3);
  forced_q8=128000;zero=4093;pulse_clear;expect_fault(4);
  zero=2000;override_adc=0;pulse_clear;
  compare=0;model_enable=0;model_pending=0;
  for(j=0;j<3;j=j+1)begin
    for(i=20;i<=31;i=i+1)begin
      if(i<=27 || i==31)begin
        origin=0;pos_enable=1;position=0;pulse_clear;
        @(negedge clk);injected_state=i;force dut.state_q=injected_state;
        if(j==1)origin=1;if(j==2)pos_enable=0;
        @(posedge clk);#2;check(fault&&code==8&&!valid&&duty==0,"removed state faults before position transition");
        deleted_checks=deleted_checks+1;
        @(negedge clk);release dut.state_q;origin=0;pos_enable=1;
      end
    end
  end
  for(j=0;j<2;j=j+1)begin
    origin=0;pos_enable=1;position=0;pulse_clear;
    @(negedge clk);if(j==0)force dut.state_q=5'bxxxxx;else force dut.state_q=5'bzzzzz;
    @(posedge clk);#2;check(fault&&code==8&&!valid&&duty==0,"unknown state locks fault8 normally");
    @(negedge clk);release dut.state_q;
  end
`ifdef DIRECT_BALANCE
  check(direct_wait_mask==5'b11111,"all five added direct wait states covered");
`endif
  $display("LATENCY accept_to_commit angle=%0d position=%0d weighted=%0d direct_wait=%b",latency_angle,latency_position,latency_weighted,direct_wait_mask);
  for(j=0;j<4;j=j+1)begin
    origin=0;pos_enable=1;position=0;pulse_clear;
    @(negedge clk);force dut.state_q=5'bxxxxx;
    case(j)
      0:origin=1;
      1:pos_enable=0;
      2:clear=1;
      3:capture=1;
    endcase
    @(posedge clk);#2;check(!fault&&!valid,"unknown state preserves higher cancellation priority");
    @(negedge clk);release dut.state_q;origin=0;pos_enable=1;clear=0;capture=0;
  end
`ifdef DIRECT_BALANCE
  for(j=0;j<3;j=j+1)begin
    for(i=32;i<=36;i=i+1)begin
      origin=0;pos_enable=1;tick=0;position=0;pulse_clear;
      @(negedge clk);injected_state=i;force dut.state_q=injected_state;
      if(j==0)origin=1;else if(j==1)pos_enable=0;else begin tick=1;force dut.calculating=1;end
      @(posedge clk);#2;
      if(j<2)check(!fault&&!valid&&!busy,"direct wait allows origin or disable cancellation");
      else check(fault&&code==5&&!valid,"direct wait repeated tick faults5");
      @(negedge clk);release dut.state_q;release dut.calculating;tick=0;origin=0;pos_enable=1;
    end
  end
`endif
  check(stage_mask==28'h00fffff,"all 20 PD calculation states covered");
  check(models>90&&comparisons>1800,"substantial numerical and differential coverage");
  $display("PASS BALANCE cycles=%0d comparisons=%0d checks=%0d commits=%0d models=%0d anti_outer=%0d anti_motor=%0d freeze=%0d deleted=%0d stages=%h",cycles,comparisons,checks,commits,models,anti_outer,anti_motor,freeze_cases,deleted_checks,stage_mask);
  $finish;
 end
 initial begin #10000000;$fatal(1,"balance test timeout");end
endmodule
