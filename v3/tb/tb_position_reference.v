`timescale 1ns / 1ps
// 只测公开接口；1ms使能压缩为每3个50MHz时钟一次，不冒充真实4秒墙钟。
// 用实数五次多项式独立检查160个目标样本，不读取RTL权重表或内部状态。
module tb_position_reference;
    reg clk = 0;
    always #10 clk = ~clk;
    reg rst_n = 1;
    reg tick_1ms = 0, ready = 0, move_press = 0;
    reg signed [31:0] origin_counts = 0, step_counts = 0;
    reg signed [31:0] position_limit = 10000;
    wire signed [31:0] target_offset_counts;
    wire moving, target_done;
    position_reference dut(
        .clk_50m(clk), .rst_n(rst_n), .tick_1ms(tick_1ms),
        .ready(ready), .move_press(move_press), .origin_counts(origin_counts),
        .step_counts(step_counts), .position_limit(position_limit),
        .target_offset_counts(target_offset_counts), .moving(moving), .target_done(target_done)
    );
    integer checks = 0, samples = 0, cycles = 0, done_count = 0;
    integer total_ticks = 0, trajectories = 0, rejected = 0;
    reg last_done = 0;
    real maximum_error = 0.0;
    always @(posedge clk) begin
        cycles = cycles + 1;
        #1;
        if (rst_n) begin
            if ((^{target_offset_counts, moving, target_done}) === 1'bx)
                $fatal(1, "unknown output cycle=%0d", cycles);
            if (target_done && last_done) $fatal(1, "done wider than one clock");
            if (target_done) done_count = done_count + 1;
            if (target_done && moving) $fatal(1, "done while still moving");
        end
        last_done = rst_n && target_done;
    end
    task check;
        input condition;
        input [767:0] message;
        begin
            checks = checks + 1;
            if (condition !== 1'b1) $fatal(1, "%0s cycle=%0d", message, cycles);
        end
    endtask
    task clocks;
        input integer n;
        integer k;
        begin for (k=0;k<n;k=k+1) begin @(posedge clk); #2; end end
    endtask
    task reset_case;
        begin
            @(negedge clk); rst_n=0; tick_1ms=0; move_press=0; ready=0;
            #2; check(target_offset_counts==0 && !moving && !target_done,
                      "asynchronous reset clears all outputs");
            clocks(2);
            @(negedge clk); rst_n=1; ready=1;
            origin_counts=0; position_limit=10000;
            clocks(2);
        end
    endtask
    task request;
        input signed [31:0] delta;
        begin
            @(negedge clk); step_counts=delta; move_press=1;
            clocks(1);
            @(negedge clk); move_press=0;
        end
    endtask
    task millisecond;
        begin
            @(negedge clk); tick_1ms=1;
            clocks(1);
            @(negedge clk); tick_1ms=0;
            // 第二拍发布乘积，第三拍清done；每个tick始终至少3clk。
            clocks(1);
            @(negedge clk); move_press=0;
            clocks(1);
            total_ticks=total_ticks+1;
        end
    endtask
    task run_curve;
        input signed [31:0] start_value;
        input signed [31:0] delta;
        input integer inject_busy;
        integer ms, index, previous_target, done_before;
        real s, weight, expected, error;
        begin
            check(moving, "accepted request enters moving");
            check(target_offset_counts==start_value, "request acceptance holds start target");
            previous_target=start_value;
            done_before=done_count;
            for(ms=1;ms<=4000;ms=ms+1) begin
                // 新请求和step输入变化不得改动已锁存轨迹，包括完成发布所在拍。
                if(inject_busy && (ms==1337 || ms==4000)) begin
                    @(negedge clk); move_press=1; step_counts=-777;
                end
                millisecond;
                if(inject_busy && (ms==1337 || ms==4000)) begin
                    @(negedge clk); move_press=0;
                end
                if(ms<4000)check(moving && done_count==done_before,
                                "cannot finish before 4000 ticks");
                if(ms%25==0) begin
                    index=ms/25;
                    s=index/160.0;
                    weight=10.0*s*s*s-15.0*s*s*s*s+6.0*s*s*s*s*s;
                    expected=start_value+delta*weight;
                    error=$itor($signed(target_offset_counts))-expected;
                    if(error<0.0)error=-error;
                    if(error>maximum_error)maximum_error=error;
                    if(delta<=1000 && delta>=-1000)
                        check(error<=1.000001, "quintic target sample error exceeds one count");
                    else check(error<=16385.0, "extreme s32 curve exceeds Q16 quantization bound");
                    if(delta>0)check($signed(target_offset_counts)>=previous_target,
                                     "positive curve must be monotonic");
                    else check($signed(target_offset_counts)<=previous_target,
                               "negative curve must be monotonic");
                    previous_target=$signed(target_offset_counts);
                    samples=samples+1;
                end else check($signed(target_offset_counts)==previous_target,
                               "target must hold between 25ms publications");
            end
            check(!moving && !target_done && done_count==done_before+1,
                  "4000th tick publishes endpoint and exactly one done pulse");
            check($signed(target_offset_counts)==start_value+delta, "exact terminal target");
            clocks(5);
            check(!moving && done_count==done_before+1, "busy press never queues another move");
            trajectories=trajectories+1;
        end
    endtask
    task reject_request;
        input signed [31:0] delta;
        integer held, old_done;
        begin
            held=$signed(target_offset_counts); old_done=done_count;
            request(delta); clocks(4);
            check(!moving && $signed(target_offset_counts)==held && done_count==old_done,
                  "invalid request holds target and emits no done");
            rejected=rejected+1;
        end
    endtask
    integer i, saved_done;
    initial begin
        #1;
        reset_case;
        request(1000); run_curve(0,1000,1);
        request(1000); run_curve(1000,1000,0);
        request(-1000); run_curve(2000,-1000,1);
        reset_case;
        request(-1000); run_curve(0,-1000,0);
        // 绝对位置边界包含origin，端点恰好在边界允许，超过一计数拒绝。
        reset_case;
        origin_counts=900; position_limit=1000;
        reject_request(101);
        request(100); run_curve(0,100,0);
        reject_request(1);
        reset_case;
        origin_counts=-900; position_limit=1000;
        reject_request(-101);
        request(-100); run_curve(0,-100,0);
        reject_request(-1);
        reject_request(0);
        position_limit=0; reject_request(1);
        position_limit=-1; reject_request(-1);
        // 起点已经越界，即使新终点回到界内也拒绝。
        reset_case;
        origin_counts=1001; position_limit=1000; reject_request(-10);
        // offset自身溢出而origin补偿使绝对终点仍在限内：必须明确拒绝。
        reset_case;
        position_limit=32'sh7fffffff; origin_counts=-1000;
        request(32'sh7fffffff); run_curve(0,32'sh7fffffff,0);
        reject_request(1);
        reset_case;
        position_limit=32'sh7fffffff; origin_counts=1000;
        request(-32'sd2147483647); run_curve(0,-32'sd2147483647,0);
        request(-1); run_curve(-32'sd2147483647,-1,0);
        reject_request(-1);
        // 绝对位置加法不能32位回绕后错误通过范围判断。
        reset_case;
        position_limit=32'sh7fffffff; origin_counts=32'sh7fffffff;
        reject_request(1);
        origin_counts=-32'sd2147483647; reject_request(-2);
        // ready撤销优先于下一拍的乘积发布和按键请求。
        reset_case;
        request(1000);
        for(i=0;i<24;i=i+1)millisecond;
        @(negedge clk);tick_1ms=1;clocks(1);
        saved_done=done_count;
        @(negedge clk);tick_1ms=0;ready=0;move_press=1;
        clocks(2);
        check(!moving && target_offset_counts==0 && done_count==saved_done,
              "ready cancellation discards pending publication and request");
        @(negedge clk);move_press=0;ready=1;clocks(4);
        check(!moving && target_offset_counts==0, "ready restore does not resume old move");
        request(-1000);run_curve(0,-1000,0);
        @(negedge clk);ready=0;clocks(1);
        check(target_offset_counts==0 && !moving, "ready loss clears completed held target");
        request(1000);clocks(2);
        check(!moving && target_offset_counts==0, "press while not ready is ignored");
        // 移动中异步复位，并验证释放后无自动恢复。
        @(negedge clk);ready=1;
        request(1000);
        for(i=0;i<200;i=i+1)millisecond;
        check(moving && target_offset_counts>0, "reset scenario exercises nonzero moving target");
        reset_case;
        check(!moving && target_offset_counts==0, "reset release does not resume trajectory");
        $display("PASS POSITION_REFERENCE cycles=%0d checks=%0d samples=%0d trajectories=%0d ticks=%0d rejected=%0d done=%0d max_error_small_step_checked=1",cycles,checks,samples,trajectories,total_ticks,rejected,done_count);
        $finish;
    end
    initial begin #100000000; $fatal(1,"watchdog"); end
endmodule
