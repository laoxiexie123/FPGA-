`timescale 1ns/1ps
// II级：真实UI/命令桥/ADC/核心/旋钮/LCD；长计时仅加速tick和消抖/原点/LCD等待末值。
module tb_ui_top;
 reg clk=0; always #10 clk=~clk;
 reg rst_n=0, a=1,b=1,button_n=1;
 reg [9:0] adc=741;
 wire sclk,mosi,dc,ac,oe,pa,a1,a2,pb,b1,b2;
 top dut(.clk_50m(clk),.rst_n(rst_n),.ec11_a(a),.ec11_b(b),.ec11_button_n(button_n),
 .lcd_sclk_next(sclk),.lcd_mosi_next(mosi),.lcd_dc_next(dc),.enc1_a(1'b0),.enc1_b(1'b0),
 .adc_data_in(adc),.adc_clk_next(ac),.adc_oe_n_next(oe),.pwma_next(pa),.an1_next(a1),
 .an2_next(a2),.pwmb_next(pb),.bn1_next(b1),.bn2_next(b2));
 integer checks=0,cycles=0,ms=0,k,j,n,commands=0,cfg_count=0,presses=0,moves=0,lcd_bits=0;
 reg [13:0] kinds=0;
 task check; input condition; input [767:0] message; begin
 checks=checks+1;if(condition!==1'b1)$fatal(1,"CHECK %0s cycles=%0d ui=%0d phase=%0d errors=%0d",message,cycles,dut.ui.state_q,dut.swing_phase,dut.ui_errors);
 end endtask
 task clocks; input integer count; integer q; begin
 for(q=0;q<count;q=q+1)begin @(posedge clk);#2;end end endtask
 always @(posedge clk)begin
 cycles=cycles+1;
 if(dut.reset_ok)begin
 if(dut.ec_press)presses=presses+1;
 if(dut.move_press)moves=moves+1;
 if(dut.ui_request)begin
 commands=commands+1;kinds[dut.ui_kind]=1;
 if(dut.ui_kind==0 && dut.ui.state_q!=21)begin
 if(cfg_count<48)begin
 if(dut.ui_index!==cfg_count[5:0])$fatal(1,"configuration order");
 cfg_count=cfg_count+1;end
 end
 end
 end
 end
 always @(posedge sclk)lcd_bits=lcd_bits+1;
 task samples; integer q; begin
 for(q=0;q<8;q=q+1)begin @(negedge clk);dut.pendulum.sample_counter_q=14'd9374;clocks(3);end
 clocks(8);end endtask
 task millisecond; begin
 // 避免真实分频器在加速毫秒之间插入额外使能；不改变控制状态或UI状态。
 @(negedge clk);dut.millisecond_divider=0;
 samples;
 ms=ms+1;
 @(negedge clk);force dut.tick_1ms=1;
 if(ms%5==0)force dut.tick_5ms=1;
 if(ms%10==0)force dut.tick_10ms=1;
 clocks(1);
 @(negedge clk);release dut.tick_1ms;dut.tick_1ms=0;
 release dut.tick_5ms;dut.tick_5ms=0;release dut.tick_10ms;dut.tick_10ms=0;
 clocks(35);
 end endtask
 task event_press; begin
 @(negedge clk);force dut.ec_press=1;clocks(1);
 @(negedge clk);release dut.ec_press;clocks(12);
 end endtask
 task event_cw; begin
 @(negedge clk);force dut.ec_cw=1;clocks(1);
 @(negedge clk);release dut.ec_cw;clocks(4);
 end endtask
 task event_ccw; begin
 @(negedge clk);force dut.ec_ccw=1;clocks(1);
 @(negedge clk);release dut.ec_ccw;clocks(12);
 end endtask
 task physical_press; begin
 @(negedge clk);button_n=1;clocks(6);
 @(negedge clk);dut.knob.press_count_q=20'd999999;clocks(2);
 @(negedge clk);button_n=0;clocks(6);
 @(negedge clk);dut.knob.press_count_q=20'd999999;clocks(3);
 @(negedge clk);button_n=1;clocks(6);
 @(negedge clk);dut.knob.press_count_q=20'd999999;clocks(3);
 end endtask
 task lcd_waits; integer q;begin
 for(q=0;q<1200;q=q+1)begin
 @(negedge clk);
 if(dut.display.state_q==0)dut.display.delay_count_q=32'd14999999;
 if(dut.display.state_q==2 || dut.display.state_q==4)dut.display.delay_count_q=32'd5999999;
 clocks(1);
 end end endtask
 initial begin
 #1;rst_n=0;clocks(4);@(negedge clk);rst_n=1;clocks(12);
 check(!dut.run_requested&&dut.upright_zero==2965,"boot menu stopped and default zero2965");
 lcd_waits;check(lcd_bits>=8,"real LCD SPI emits initialization bits");
 for(k=0;k<220;k=k+1)millisecond;
 check(dut.ui_page==0&&dut.measurement_ready,"menu healthy actual ADC initialized");
 physical_press;check(dut.ui_page==1,"physical EC button enters confirmation page");
 check(presses>=1,"real EC synchronized press observed");
 event_cw;check(dut.ui_selection==1,"CW selects start confirmation");
 event_press;
 for(k=0;k<450 && !dut.run_requested;k=k+1)millisecond;
 check(dut.run_requested&&dut.ui_errors==0,"whole UI startup actually runs without rejected commands");
 check(cfg_count==48,"all48 startup configuration words loaded in order");
 check(kinds[0]&&kinds[1]&&kinds[2]&&kinds[3]&&kinds[4]&&kinds[7]&&kinds[8]&&kinds[9]&&kinds[10]&&kinds[11]&&kinds[12]&&kinds[13],"startup S C U V J B I Z H Q A L commands covered");
 // 混合权重之外还需输出限速追随；保留100ms节拍让驱动从190回到PD命令。
 for(k=0;k<100;k=k+1)begin millisecond;clocks(7000);end
 check(dut.main_state==3&&dut.core_commit!==1'bx,"actual ADC captures into real balance");
 @(negedge clk);dut.supervisor.origin_delay_count_q=25'd24999999;clocks(5);
 check(dut.position_ready,"accelerated real origin lock enables position");
 for(k=0;k<4;k=k+1)millisecond;
 event_press;clocks(10);
 check(dut.position_mode&&!dut.reference_moving,"press enters step mode without motion");
 event_cw;clocks(12);check(dut.reference_moving,"CW step starts +130 reference");
 for(k=0;k<200;k=k+1)millisecond;
 n=moves;event_ccw;event_cw;check(moves==n,"busy direction events ignored without queue");
 event_press;check(!dut.position_mode&&dut.reference_moving,"press exits mode during motion without cancel");
 for(k=200;k<4000;k=k+1)millisecond;
 clocks(5);check(!dut.reference_moving&&dut.reference_offset==130,"4000ms positive step reaches130 after mode exit");
 check(moves==n,"ignored busy rotations never replay");
 event_press;check(dut.position_mode&&!dut.reference_moving,"press reenters step mode");
 event_ccw;clocks(12);check(dut.reference_moving,"CCW starts negative130 reference");
 for(k=0;k<4000;k=k+1)millisecond;
 check(dut.reference_offset==0&&!dut.reference_moving,"negative130 returns reference to0");
 event_press;check(!dut.position_mode,"press exits position mode again");
 event_cw;check(dut.ui_selection==1,"CW outside mode selects CW3604S menu");
 event_press;clocks(12);check(dut.reference_moving&&!dut.position_mode,"circle menu starts clockwise1040 without mode");
 for(k=0;k<200;k=k+1)millisecond;
 check(dut.reference_offset>0,"full circle begins positive direction");
 n=moves;event_press;event_ccw;check(moves==n&&dut.ui_selection==1,"busy circle press and rotation ignored");
 for(k=200;k<4000;k=k+1)millisecond;
 check(dut.reference_offset==1040&&!dut.reference_moving,"CW full turn ends at1040 without reverse return");
 for(k=0;k<30;k=k+1)millisecond;
 check(moves==n&&dut.reference_offset==1040&&!dut.reference_moving,"circle holds endpoint without automatic repeat");
 event_ccw;check(dut.ui_selection==0,"CCW selects step menu after circle");
 event_press;check(dut.position_mode,"step mode can enter after full circle");
 event_cw;clocks(12);check(dut.reference_moving,"start partial step before injected fault");
 for(k=0;k<200;k=k+1)millisecond;
 // 本地故障进入停止流程；控制会话复位，ADC/UI校准历史保留。
 @(negedge clk);force dut.encoder_fault=1;clocks(8);release dut.encoder_fault;
 for(k=0;k<50;k=k+1)millisecond;
 check(!dut.run_requested&&!dut.reference_moving&&dut.reference_offset==0&&!dut.position_mode,"fault stops cancels mode and reference");
 check(dut.display_fault!=0 && dut.ui_page==4,"fault page retains fault code until acknowledgement");
 // 故障页确认返回菜单，再选校准；使用真实ADC分组平均32次。
 event_press;clocks(30);
 for(k=0;k<210;k=k+1)millisecond;
 check(dut.ui_page==0&&dut.display_fault==0,"fault acknowledgement clears code and returns menu");
 event_cw;event_press;check(dut.ui_page==3,"calibration page accessible");
 event_press;clocks(30);
 for(k=0;k<40;k=k+1)begin samples;clocks(8);end
 clocks(30);check(dut.upright_zero==2964,"32 actual ADC741 calibration yields2964");
 // 校准仍停留校准页，转一格选择恢复。
 for(k=0;k<210;k=k+1)millisecond;
 event_cw;event_press;clocks(50);
 check(dut.upright_zero==2965,"restore default zero2965 through real UI command");
 @(negedge clk);rst_n=0;#2;check({pa,a1,a2,pb,b1,b2}==0,"reset asynchronously stops all motor outputs");
 $display("PASS UI_TOP cycles=%0d checks=%0d commands=%0d cfg=%0d presses=%0d moves=%0d lcd_bits=%0d accelerated_ms=%0d real_adc=1 ui_states_forced=0",cycles,checks,commands,cfg_count,presses,moves,lcd_bits,ms);
 $finish;
 end
 initial begin #1000000000;$fatal(1,"watchdog");end
endmodule
