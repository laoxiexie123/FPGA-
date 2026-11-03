`timescale 1ns / 1ps
// 仅测试台：独立连续电平计数模型逐拍核对，不读取DUT内部寄存器。
module key_filter_case #(
    parameter integer stable_cycles = 4,
    parameter integer default_case = 0
)(
    output reg finished
);
    reg clk_50m = 1'b0;
    always #10 clk_50m = !clk_50m;
    reg rst_n = 1'b0;
    reg key_n = 1'b1;
    wire pressed, press_pulse;
    key_filter #(.stable_cycles(stable_cycles)) dut(
        .clk_50m(clk_50m), .rst_n(rst_n), .key_n(key_n),
        .pressed(pressed), .press_pulse(press_pulse)
    );
    integer checks = 0;
    integer pulse_count = 0;
    integer low_run = 0;
    integer high_run = 0;
    integer valid_samples = 0;
    reg first_sample = 1'b1;
    reg second_sample = 1'b1;
    reg expected_pressed = 1'b0;
    reg expected_pulse = 1'b0;
    reg released_since_reset = 1'b0;

    // 功能：按真实50MHz沿独立建模同步与连续稳定长度；rst_n异步低有效。
    // 模型只观察公开输入/输出；不复制DUT计数器、资格判断或内部有效移位寄存器。
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            first_sample = 1'b1;
            second_sample = 1'b1;
            valid_samples = 0;
            low_run = 0;
            high_run = 0;
            expected_pressed = 1'b0;
            expected_pulse = 1'b0;
            released_since_reset = 1'b0;
        end else begin
            expected_pulse = 1'b0;
            if (valid_samples >= 2) begin
                if (second_sample) begin
                    low_run = 0;
                    if (high_run < stable_cycles) high_run = high_run + 1;
                    if (high_run == stable_cycles) begin
                        expected_pressed = 1'b0;
                        released_since_reset = 1'b1;
                    end
                end else begin
                    high_run = 0;
                    if (low_run < stable_cycles) low_run = low_run + 1;
                    if (low_run == stable_cycles) begin
                        expected_pressed = 1'b1;
                        expected_pulse = released_since_reset;
                        released_since_reset = 1'b0;
                    end
                end
            end
            second_sample = first_sample;
            first_sample = key_n;
            if (valid_samples < 2) valid_samples = valid_samples + 1;
        end
        #1;
        checks = checks + 1;
        if ({pressed, press_pulse} !== {expected_pressed, expected_pulse})
            $fatal(1, "model mismatch stable_cycles=%0d check=%0d actual=%b%b expected=%b%b",
                stable_cycles, checks, pressed, press_pulse, expected_pressed, expected_pulse);
        if (rst_n && press_pulse) pulse_count = pulse_count + 1;
    end

    task clocks;
        input integer count;
        begin repeat(count) @(negedge clk_50m); end
    endtask
    task check;
        input condition;
        input [511:0] message;
        begin
            if (condition !== 1'b1)
                $fatal(1, "stable_cycles=%0d: %0s", stable_cycles, message);
        end
    endtask
    task drive;
        input level;
        begin @(negedge clk_50m); key_n = level; end
    endtask
    task reset_with;
        input level;
        begin
            @(negedge clk_50m); key_n = level; rst_n = 1'b0;
            #2; check(!pressed && !press_pulse, "asynchronous reset clears outputs");
            clocks(2); rst_n = 1'b1;
        end
    endtask
    integer pulses_before;
    integer bounce;
    initial begin
        finished = 1'b0;
        reset_with(default_case ? 1'b1 : 1'b0);
        if (!default_case) begin
            clocks(stable_cycles + 5);
            check(pressed && pulse_count == 0, "boot-held must report pressed without pulse");
            clocks(stable_cycles + 3);
            check(pulse_count == 0, "long boot-held cannot arm itself");
            if (stable_cycles > 1) begin
                drive(1'b1); clocks(stable_cycles - 2); drive(1'b0);
                clocks(stable_cycles + 4);
                check(pressed && pulse_count == 0, "short release cannot arm boot-held key");
            end
            drive(1'b1); clocks(stable_cycles + 3);
            check(!pressed && !press_pulse, "stable release arms without pulse");
            if (stable_cycles > 1) begin
                for (bounce = 0; bounce < 4; bounce = bounce + 1) begin
                    drive(1'b0); clocks(0); drive(1'b1); clocks(0);
                end
                clocks(stable_cycles + 4);
                check(!pressed && pulse_count == 0, "bounce and short press cannot trigger");
            end
        end else begin
            clocks(stable_cycles + 3);
            check(!pressed && pulse_count == 0, "default stable release produces no event");
        end
        pulses_before = pulse_count;
        drive(1'b0);
        clocks(stable_cycles + 1);
        check(!pressed && pulse_count == pulses_before, "must not accept one sample early");
        clocks(1);
        check(pressed && press_pulse && pulse_count == pulses_before + 1,
            "accept exactly on stable threshold after two synchronizer stages");
        clocks(stable_cycles > 8 ? 8 : stable_cycles + 4);
        check(pressed && !press_pulse && pulse_count == pulses_before + 1,
            "held press has exactly one clock pulse");
        drive(1'b1); clocks(stable_cycles + 1);
        check(pressed, "release must not be accepted early");
        clocks(1); check(!pressed && !press_pulse, "release threshold does not emit pulse");
        if (!default_case) begin
            drive(1'b0); clocks(stable_cycles + 3);
            check(pressed && pulse_count == pulses_before + 2, "second real press produces event");
            pulses_before = pulse_count;
            reset_with(1'b0); clocks(stable_cycles + 5);
            check(pressed && pulse_count == pulses_before, "reset held key must disarm again");
            drive(1'b1); clocks(stable_cycles + 3);
            drive(1'b0); clocks(stable_cycles + 3);
            check(pressed && pulse_count == pulses_before + 1, "release then press rearms after reset");
        end
        $display("PASS KEY stable_cycles=%0d checks=%0d pulses=%0d", stable_cycles, checks, pulse_count);
        finished = 1'b1;
    end
endmodule

module tb_key_filter;
    wire four_finished, one_finished, default_finished;
    key_filter_case #(.stable_cycles(4)) four(.finished(four_finished));
    key_filter_case #(.stable_cycles(1)) one(.finished(one_finished));
    key_filter_case #(.stable_cycles(1000000), .default_case(1)) normal(.finished(default_finished));
    initial begin
        wait(four_finished && one_finished && default_finished);
        $display("PASS KEY_FILTER all cases at actual 50MHz; default debounce=20ms");
        $finish;
    end
    initial begin
        #100000000;
        $fatal(1, "key_filter test timeout");
    end
endmodule
