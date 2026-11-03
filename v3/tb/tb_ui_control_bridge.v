`timescale 1ns/1ps
module tb_ui_control_bridge;
 reg clk=0; always #10 clk=~clk;
 reg rst_n=1,tick_1ms=0,command_request=0;
 reg [3:0] command_kind=0;
 reg [5:0] command_index=0;
 reg signed [31:0] command_value=0;
 reg start_ready=0,run_fault=0,position_ready=0,trajectory_fault=0,running=0;
 reg signed [31:0] position_limit=10000;
 wire tx_busy,tx_done,query_done,armed,start_pulse,stop_pulse,zero_pulse,heartbeat_pulse;
 wire motion_enter,motion_exit,motion_target;
 wire signed [31:0] motion_target_offset;
 wire [11:0] zero_adc_x4;
 wire [31:0] error_count;
 ui_control_bridge dut(.*);
 wire [31:0] cfg,default_zero;
 wire signed [31:0] u,v,j,b,ki;
 local_startup_profile oracle(.config_index(command_index),.cfg_value_next(cfg),
 .startup_u_next(u),.startup_v_next(v),.startup_j_next(j),.startup_b_next(b),
 .startup_i_next(ki),.default_zero_next(default_zero));
 integer checks=0,k;
 reg [31:0] old_errors;
 task check;input condition;input [511:0] msg;begin
 checks=checks+1;if(condition!==1'b1)$fatal(1,"%0s",msg);end endtask
 task step;begin @(posedge clk);#2;end endtask
 task send;input [3:0] kind;input [5:0] index;input signed [31:0] value;
 begin
 @(negedge clk);command_kind=kind;command_index=index;command_value=value;command_request=1;
 step;check(tx_busy&&!tx_done&&!query_done,"request accepted busy without same-cycle done");
 @(negedge clk);command_request=0;command_kind=15;command_value=99999;
 step;check(!tx_busy&&tx_done,"latched command completes next cycle");
 end endtask
 task quiet;begin step;check(!tx_done&&!query_done&&!start_pulse&&!stop_pulse&&!zero_pulse&&
 !heartbeat_pulse&&!motion_enter&&!motion_exit&&!motion_target,"events and done are single cycle");end endtask
 initial begin
 #1;rst_n=0;step;@(negedge clk);rst_n=1;step;
 check(zero_adc_x4==2965&&!armed&&error_count==0,"reset state");
 send(0,6,3001);check(zero_adc_x4==3001&&error_count==0,"C06 calibrated zero write");quiet;
 send(0,6,-1);check(zero_adc_x4==3001&&error_count==1,"reject negative zero");quiet;
 send(0,6,4093);check(zero_adc_x4==3001&&error_count==2,"reject overrange zero");quiet;
 send(0,6,4092);check(zero_adc_x4==4092,"zero upper boundary");quiet;
 send(0,6,0);check(zero_adc_x4==0,"zero lower boundary");quiet;
 for(k=0;k<48;k=k+1)if(k!=6)begin
 @(negedge clk);command_index=k[5:0];#1;old_errors=error_count;
 send(0,k[5:0],$signed(cfg));check(error_count==old_errors,"fixed profile exact acknowledge");quiet;
 end
 send(0,0,123);check(error_count==3,"reject fake config write");quiet;
 send(0,48,0);check(error_count==4,"reject unsupported config index");quiet;
 send(1,0,u);quiet;send(2,0,v);quiet;send(3,0,j);quiet;send(4,0,b);quiet;
 check(error_count==4,"fixed U V J B accepted idle");
 send(13,0,0);quiet;send(13,0,16);check(error_count==5,"integral nonzero rejected");quiet;
 send(11,0,1);check(!start_pulse&&error_count==6,"unarmed launch rejected");quiet;
 send(10,0,1);check(!armed&&error_count==7,"not ready arm rejected");quiet;
 start_ready=1;send(10,0,1);check(armed,"ready arm");quiet;
 send(11,0,1);check(start_pulse&&!armed,"armed launch and disarm");quiet;
 running=1;send(0,6,2965);check(zero_adc_x4==0&&error_count==8,"running config rejected");quiet;
 send(8,0,0);check(!zero_pulse&&error_count==9,"running zero rejected");quiet;
 send(5,0,1);check(!motion_enter&&error_count==10,"position not ready motion enter rejected");quiet;
 position_ready=1;send(5,0,1);check(motion_enter,"motion enter");quiet;
 send(6,0,130);check(motion_target&&motion_target_offset==130,"fixed relative move130 accepted");quiet;
 send(6,0,-130);check(motion_target&&motion_target_offset==-130&&error_count==10,"negative130 move accepted");quiet;
 send(6,0,1040);check(motion_target&&motion_target_offset==1040&&error_count==10,"clockwise full turn1040 accepted");quiet;
 send(6,0,-1040);check(!motion_target&&error_count==11,"reverse full turn not supported");quiet;
 send(6,0,520);check(!motion_target&&error_count==12,"legacy absolute target rejected");quiet;
 trajectory_fault=1;send(6,0,130);check(!motion_target&&error_count==13,"trajectory fault reject target");quiet;
 send(5,0,0);check(motion_exit,"motion exit allowed despite fault");quiet;
 send(7,0,0);check(stop_pulse&&!armed&&error_count==13,"stop always and retains error count");quiet;
 running=0;trajectory_fault=0;send(8,0,0);check(zero_pulse,"idle zero pulse");quiet;
 send(12,0,0);check(heartbeat_pulse,"heartbeat completion event");quiet;
 send(9,0,0);check(query_done,"query done on completion cycle");quiet;
 send(10,0,1);check(armed,"rearm");quiet;
 run_fault=1;step;check(!armed,"run fault revokes arm");
 send(11,0,1);check(!start_pulse&&error_count==14,"fault launch rejected");quiet;run_fault=0;
 // 完成时才读取实时准入，不沿用请求拍的就绪状态。
 @(negedge clk);command_kind=10;command_value=1;command_request=1;step;
 @(negedge clk);command_request=0;start_ready=0;step;
 check(!armed&&error_count==15,"readiness withdrawn before completion");quiet;
 // busy期间输入不覆盖原命令，但额外请求必须记错。
 @(negedge clk);command_kind=9;command_value=0;command_request=1;step;
 @(negedge clk);command_kind=7;command_request=1;step;
 check(query_done&&!stop_pulse&&error_count==16,"busy collision rejected original query completes");
 @(negedge clk);command_request=0;quiet;
 send(15,0,0);check(error_count==17,"unknown kind rejected");quiet;
 // 错误计数饱和，不回绕。
 @(negedge clk);dut.error_count=32'hfffffffe;
 command_request=1;command_kind=15;step;
 step;check(error_count==32'hffffffff,"collision and reject saturate");
 @(negedge clk);command_request=0;quiet;
 $display("PASS UI_BRIDGE checks=%0d config_entries=47 delayed_handshake=1",checks);$finish;
 end
 initial begin #1000000;$fatal(1,"watchdog");end
endmodule
