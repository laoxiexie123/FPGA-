`timescale 1ns/1ps
module tb_adc_freshness;
  reg clk=0; always #10 clk=~clk;
  reg rst_n=0,tick=0,clear=0;
  wire seen,fresh,valid;
  wire [31:0] age;
  integer checks=0;
  adc_pend dut(.clk_50m(clk),.rst_n(rst_n),.filter_clear(clear),.tick_1ms(tick),
      .adc_data_in(10'd500),.adc_clk_next(),.adc_oe_n_next(),.average_raw_q(),
      .average_valid_q(valid),.angle_x4_q(),.angle_valid_q(),.filter_ready_q(),
      .average_seen_q(seen),.average_age_ms_q(age),.average_fresh_next(fresh),
      .tick_5ms(1'b0),.zero_adc_x4(12'd2000),.sample_commit_q(),.observer_busy_q(),
      .pendulum_valid_q(),.angle_cdeg_q(),.speed_cdeg_s_q(),.observer_seq_q(),
      .observer_fault_q());
  task step; begin @(posedge clk);#1;checks=checks+1;@(negedge clk);end endtask
  task ck; input ok; begin if(ok!==1'b1)$fatal(1,"freshness check failed age=%0d",age);end endtask
  integer i;
  initial begin
    #1;rst_n=0;#24;rst_n=1;step;ck(!seen&&!fresh&&age==65535);
    while(!valid)step;
    tick=1;step;ck(seen&&fresh&&age==0); // Valid beats simultaneous tick.
    for(i=1;i<=19;i=i+1)begin step;ck(age==i&&fresh);end
    step;ck(age==20&&!fresh);
    tick=0;clear=1;step;ck(age==20&&seen&&!fresh);clear=0;
    while(!valid)step;
    step;ck(age==0&&fresh);
    // Diagnostic injection covers saturation without waiting billions of milliseconds.
    dut.average_age_ms_q=32'hffffffff;tick=1;step;ck(age==32'hffffffff&&!fresh);
    #3;rst_n=0;#1;ck(!seen&&!fresh&&age==65535);
    $display("PASS ADC_FRESHNESS checks=%0d",checks);$finish;
  end
  initial begin #4000000;$fatal(1,"freshness timeout");end
endmodule
