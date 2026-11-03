`timescale 1ns/1ps
// II级实际模块集成：先跑真实10ms时基，其余长等待用层次计数预置加速。
// 不改RTL参数、不用替身模块；预置/force仅为仿真激励，不代表完整100ms或20ms等待。
module tb_top;
  reg clk=0;
  always #10 clk=~clk;
  reg rst_n=0,sw1_n=1,sw4_n=1,enc_a=0,enc_b=0;
  reg [9:0] adc=500;
  wire adc_clk,adc_oe,pa,a1,a2,pb,b1,b2;
  top dut(.clk_50m(clk),.rst_n(rst_n),.key_sw1_n(sw1_n),.key_sw4_n(sw4_n),.key_sw3_n(1'b1),
      .enc1_a(enc_a),.enc1_b(enc_b),.adc_data_in(adc),.adc_clk_next(adc_clk),
      .adc_oe_n_next(adc_oe),.pwma_next(pa),.an1_next(a1),.an2_next(a2),
      .pwmb_next(pb),.bn1_next(b1),.bn2_next(b2));
  integer cycles=0,checks=0,ms_ticks=0,five_ticks=0,ten_ticks=0;
  integer last_ms=0,last_five=0,last_ten=0,real_begin=0;
  integer key_start_events=0,transmit_events=0,core_results=0,pwm_high_cycles=0;
  integer guard_cases=0,i,j;
  reg real_tick_check=0;
  task check;
    input condition;
    input [767:0] message;
    begin
      checks=checks+1;
      if(condition!==1'b1)$fatal(1,"CHECK %0s cycle=%0d",message,cycles);
    end
  endtask
  task clocks;
    input integer n;
    integer k;
    begin for(k=0;k<n;k=k+1)begin @(posedge clk);#2;end end
  endtask
  // 仿真专用观察；实际端口保持没有诊断输出的设计，不增加硬件影子状态。
  always @(posedge clk) begin
    cycles=cycles+1;
    #1;
    if(dut.reset_ok)begin
      if((^{adc_clk,adc_oe,pa,a1,a2,pb,b1,b2})===1'bx)
        $fatal(1,"Unknown physical output cycle=%0d",cycles);
      if({pa,a1,a2}!==3'b000)$fatal(1,"Unused motor A must remain off");
      if(dut.start_press)key_start_events=key_start_events+1;
      if(dut.transmit_request)transmit_events=transmit_events+1;
      if(dut.core_commit)core_results=core_results+1;
      if(pb)pwm_high_cycles=pwm_high_cycles+1;
      if(real_tick_check)begin
        if(dut.tick_1ms)begin
          if(last_ms!=0 && cycles-last_ms!=50000)$fatal(1,"1ms real divider");
          last_ms=cycles;ms_ticks=ms_ticks+1;
        end
        if(dut.tick_5ms)begin
          if(last_five!=0 && cycles-last_five!=250000)$fatal(1,"5ms real divider");
          last_five=cycles;five_ticks=five_ticks+1;
        end
        if(dut.tick_10ms)begin
          if(last_ten!=0 && cycles-last_ten!=500000)$fatal(1,"10ms real divider");
          last_ten=cycles;ten_ticks=ten_ticks+1;
        end
      end
    end
  end
  task reset_system;
    begin
      @(negedge clk);rst_n=0;sw1_n=1;sw4_n=1;
      #2;check({pa,a1,a2,pb,b1,b2}==0,"asynchronous reset stops both motor channels");
      clocks(3);
      @(negedge clk);rst_n=1;
      @(posedge clk);#2;check(!dut.reset_ok,"reset release first synchronizer cycle");
      @(posedge clk);#2;check(dut.reset_ok,"reset release second synchronizer cycle");
      clocks(5);check(!dut.run_requested&&!dut.fault_latched,"reset clears run and fault latch");
    end
  endtask
  task observer_tick;
    begin
      @(negedge clk);force dut.tick_5ms=1'b1;
      @(posedge clk);#2;
      @(negedge clk);release dut.tick_5ms;
      // Remove the forced tick without leaving an unintended second request.
      dut.tick_5ms=0;
      clocks(24);
    end
  endtask
  task seed_actual_measurements;
    integer n;
    begin
      // 只预置采样间隔计数：8次锁存、分组平均、IIR与两次观测仍由真实模块执行。
      for(n=0;n<8;n=n+1)begin
        @(negedge clk);dut.pendulum.sample_counter_q=14'd9374;
        clocks(3);
      end
      clocks(20);observer_tick;observer_tick;
      check(dut.average_raw==adc&&dut.filtered_adc_x4==adc*4,"actual ADC mean and first IIR seed");
      check(dut.measurement_ready&&!dut.observer_fault,"actual observer establishes valid data");
    end
  endtask
  task ready_boundary;
    begin
      check(dut.start_ready,"ready fixture starts with healthy measurements");
      @(negedge clk);dut.ready_ms=7'd99;dut.millisecond_divider=16'd49999;
      clocks(3);check(dut.ready_ms==100,"ready 99 to 100ms terminal edge");
      check(!dut.run_requested,"ready becoming true never automatically starts");
    end
  endtask
  task arm_physical_keys;
    begin
      sw1_n=1;sw4_n=1;clocks(5);
      @(negedge clk);
      dut.start_key.stable_count=20'd999999;
      dut.transmit_key.stable_count=20'd999999;
      clocks(3);
      check(dut.start_key.press_armed&&dut.transmit_key.press_armed,"stable release arms real keys");
    end
  endtask
  task press_start;
    begin
      @(negedge clk);sw1_n=0;clocks(5);
      @(negedge clk);dut.start_key.stable_count=20'd999999;
      clocks(4);
    end
  endtask
  task press_transmit;
    integer saved_events;
    begin
      saved_events=transmit_events;
      @(negedge clk);sw4_n=0;clocks(5);
      @(negedge clk);dut.transmit_key.stable_count=20'd999999;
      clocks(4);
      check(transmit_events==saved_events+1,"SW4 real key emits exactly one future transmit request");
    end
  endtask
  task boot_ready;
    begin reset_system;seed_actual_measurements;ready_boundary;arm_physical_keys;end
  endtask
  task start_running;
    begin
      press_start;
      check(dut.run_requested&&dut.motor_permitted&&!dut.fault_latched,"SW1 starts only ready healthy system");
      clocks(20);observer_tick;clocks(30);
      check(dut.supervised_duty!=0,"actual swing supervisor produces nonzero impulse command");
    end
  endtask
  task wait_pwm_high;
    integer n;
    begin
      n=0;
      while(!pb&&n<16000)begin clocks(1);n=n+1;end
      check(pb,"actual motor blanking and carrier eventually produce PWM");
    end
  endtask
  task assert_stopped_latched;
    begin
      clocks(2);
      check(dut.fault_latched&&!dut.run_requested&&!dut.motor_permitted,"running fault locks top until reset");
      check({pb,b1,b2}==0,"running fault shuts PWM and directions without waiting carrier period");
      guard_cases=guard_cases+1;
    end
  endtask
  initial begin
    #1;rst_n=0;
    reset_system;
    real_tick_check=1;real_begin=cycles;
    clocks(500100);
    real_tick_check=0;
    check(ms_ticks==10&&five_ticks==2&&ten_ticks==1,"real first 10ms has correct 1/5/10ms tick counts");
    check(last_ms-real_begin>=49990&&last_ms-real_begin<=500010,"real tick schedule spans 10ms");
    check(dut.upright_zero==2965,"reference effective zero is 3100 minus 135 = 2965");
    check(dut.measurement_ready&&dut.average_raw==500,"real ADC and observer initialized without substitutes");
    check(!dut.run_requested&&{pb,b1,b2}==0,"no automatic start after reset and measurement establishment");
    arm_physical_keys;
    press_start;check(!dut.run_requested&&!dut.fault_latched,"SW1 before ready window is ignored, not queued");
    ready_boundary;clocks(10);
    check(!dut.run_requested,"held early SW1 does not start when ready becomes true");
    press_transmit;
    check(!dut.run_requested&&{pb,b1,b2}==0,"SW4 standby request never starts motor");
    arm_physical_keys;start_running;wait_pwm_high;
    arm_physical_keys;press_transmit;
    check(dut.run_requested&&dut.motor_permitted,"SW4 request does not stop active control");
    // 异步外部复位必须在PWM高电平期间直接关断，不等待下一载波。
    wait_pwm_high;
    @(negedge clk);rst_n=0;#2;
    check({pb,b1,b2}==0,"reset during PWM high disables motor asynchronously");
    boot_ready;
    force dut.arm_speed=32'sd201;press_start;
    check(!dut.run_requested&&!dut.fault_latched,"starting speed 201 exceeds old limit 200");
    release dut.arm_speed;
    // 每个保护场景独立复位。传感器故障仅force真实模块信号，不替换任何模块。
    for(i=0;i<8;i=i+1)begin
      boot_ready;start_running;
      if(i==0)wait_pwm_high;
      @(negedge clk);
      case(i)
        0:force dut.pendulum.average_age_ms_q=32'd20;
        1:force dut.encoder_fault=1'b1;
        2:force dut.core_fault=1'b1;
        3:force dut.swing_fault=8'd3;
        4:force dut.position_counts=32'sd10001;
        5:force dut.arm_speed=32'sd4401;
        6:begin force dut.main_state=2'd3;force dut.average_raw=10'd0;end
        7:begin force dut.main_state=2'd3;force dut.average_raw=10'd500;end
      endcase
      #2;check(!dut.motor_permitted,"live guard suppresses permission before latch clock");
      assert_stopped_latched;
      case(i)
        0:release dut.pendulum.average_age_ms_q;
        1:release dut.encoder_fault;
        2:release dut.core_fault;
        3:release dut.swing_fault;
        4:release dut.position_counts;
        5:release dut.arm_speed;
        6,7:begin release dut.main_state;release dut.average_raw;end
      endcase
      @(negedge clk);force dut.start_press=1'b1;clocks(1);
      @(negedge clk);release dut.start_press;clocks(3);
      check(dut.fault_latched&&!dut.run_requested,"new press cannot clear latched run fault");
    end
    // 起摆阶段轨端豁免：ADC0/1023真实采样可启动，但无有效角度不得进入捕获。
    for(j=0;j<2;j=j+1)begin
      if(j==0)adc=0;else adc=1023;
      boot_ready;start_running;
      check(!dut.pendulum_valid&&!dut.core_enable,"rail ADC remains outside valid capture window");
      check(dut.run_requested&&!dut.fault_latched,"mode3 permits rail ADC while impulse startup is active");
    end
    // 零点2965对应741.25码，整数ADC741的误差为-1（四倍ADC单位），不是精确零。
    adc=741;boot_ready;press_start;
    for(j=0;j<6;j=j+1)begin
      observer_tick;
      @(negedge clk);force dut.tick_1ms=1'b1;
      clocks(1);
      @(negedge clk);release dut.tick_1ms;dut.tick_1ms=0;
      // 每次只发一个毫秒使能；留真实时钟让混合权重逐拍推进，避免伪造ADC过龄。
      clocks(7000);
    end
    check(dut.run_requested&&!dut.fault_latched&&dut.core_enable,"actual capture enables real balance core");
    check(core_results>0,"actual balance core produces commits through top wiring");
    check(dut.core_duty==-2,"ADC741 zero2965 yields error -1 and boosted PD -1 plus friction -1");
    check(pwm_high_cycles>0,"nonzero motor path was actually exercised");
    $display("PASS TOP cycles=%0d checks=%0d real_10ms_cycles=500100 ticks=%0d/%0d/%0d guard_cases=%0d start_events=%0d transmit_events=%0d core_results=%0d pwm_high=%0d ready_wait_preloaded=1 key_wait_preloaded=1",cycles,checks,ms_ticks,five_ticks,ten_ticks,guard_cases,key_start_events,transmit_events,core_results,pwm_high_cycles);
    $finish;
  end
  initial begin #300000000;$fatal(1,"top simulation watchdog");end
endmodule
