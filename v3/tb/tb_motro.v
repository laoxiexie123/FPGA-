`timescale 1ns/1ps
// Known-input differential test plus independent arithmetic/cycle/pulse models.
module tb_motro;
  reg clk_50m = 1'b0;
  always #10 clk_50m = ~clk_50m;
  reg rst_n = 1'b0;
  reg enable = 1'b0;
  reg signed [15:0] duty = 16'sd0;
  wire in1_q, in2_q, pwm_q, old_in1, old_in2, old_pwm;
  motro dut (.clk_50m(clk_50m), .rst_n(rst_n), .enable(enable), .duty(duty),
      .in1_q(in1_q), .in2_q(in2_q), .pwm_q(pwm_q));
  motor_drv old (.clk(clk_50m), .rst_n(rst_n), .enable(enable), .duty(duty),
      .ain1_q(old_in1), .ain2_q(old_in2), .pwm_q(old_pwm));
  integer cycles = 0, checks = 0, arithmetic_checks = 0, pulse_periods = 0;
  integer carrier, blank, sign_model, width_model, dir1, dir2, pwm_model;
  integer value, level, width, wanted_sign, allowed;
  integer boundary_mask = 0, reset_events = 0, reblank_events = 0;
  integer i, rng = 32'h4927512;

  function integer math_width;
    input integer command;
    integer magnitude;
    begin
      magnitude = command < 0 ? -command : command;
      if (magnitude > 255) magnitude = 255;
      math_width = (magnitude * 5000 + 135) / 270;
    end
  endfunction

  // Model uses integers and pre-edge state, never DUT internal state for predictions.
  always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
      carrier = 0; blank = 5000; sign_model = 0; width_model = 0;
      dir1 = 0; dir2 = 0; pwm_model = 0;
    end else begin
      cycles = cycles + 1;
      value = $signed(duty);
      wanted_sign = value < 0;
      allowed = enable && value != 0 && blank == 0 && wanted_sign == sign_model;
      if (!enable || value == 0) begin dir1 = 0; dir2 = 0; end
      else if (blank == 0 && wanted_sign == sign_model) begin
        dir1 = !sign_model; dir2 = sign_model;
      end
      pwm_model = allowed && (carrier == 0 || carrier < width_model);
      if (carrier == 0) boundary_mask = boundary_mask | 1;
      if (carrier == 4999) boundary_mask = boundary_mask | 2;
      if (!enable || value == 0) begin
        sign_model = wanted_sign; blank = 5000; width_model = 0;
      end else if (wanted_sign != sign_model) begin
        if (blank > 0) reblank_events = reblank_events + 1;
        sign_model = wanted_sign; blank = 5000; width_model = 0;
      end else if (blank > 0) begin
        blank = blank - 1; width_model = 0;
      end else if (carrier == 0) width_model = math_width(value);
      if (carrier == 4999) carrier = 0; else carrier = carrier + 1;
    end
    #1;
    checks = checks + 1;
    if ({in1_q,in2_q,pwm_q} !== {old_in1,old_in2,old_pwm})
      $fatal(1, "Old/new mismatch cycle=%0d duty=%0d", cycles, duty);
    if (in1_q !== dir1[0] || in2_q !== dir2[0] || pwm_q !== pwm_model[0])
      $fatal(1, "Independent cycle model mismatch cycle=%0d", cycles);
    if (rst_n && (dut.carrier_count_q !== carrier[15:0] ||
        dut.blank_count_q !== blank[15:0] || dut.active_sign_q !== sign_model[0] ||
        dut.pulse_width_q !== width_model[15:0]))
      $fatal(1, "Internal timing invariant mismatch cycle=%0d", cycles);
  end

  task hold_command;
    input integer command;
    input integer count;
    begin
      @(negedge clk_50m); enable = 1'b1; duty = command;
      repeat (count) @(negedge clk_50m);
    end
  endtask
  task pulse_check;
    input integer command;
    integer highs, n;
    begin
      hold_command(command, 16000);
      while (carrier != 0) @(negedge clk_50m);
      highs = 0;
      for (n = 0; n < 5000; n = n + 1) begin
        @(posedge clk_50m); #2;
        if (pwm_q) highs = highs + 1;
      end
      if (highs != math_width(command))
        $fatal(1, "Pulse count mismatch command=%0d highs=%0d expected=%0d",
               command, highs, math_width(command));
      pulse_periods = pulse_periods + 1;
    end
  endtask
  task async_reset;
    begin
      @(negedge clk_50m); #3 rst_n = 1'b0;
      reset_events = reset_events + 1;
      #2;
      if ({in1_q,in2_q,pwm_q} !== 3'b000)
        $fatal(1, "Asynchronous reset failed before clock");
      #22 rst_n = 1'b1;
    end
  endtask

  initial begin
    #1;
    // Exhaust the full signed16 domain; use independent integer absolute value.
    for (i = -32768; i <= 32767; i = i + 1) begin
      duty = i; #1;
      width = math_width(i);
      level = i < 0 ? -i : i;
      if (level > 255) level = 255;
      if (dut.requested_width_next !== width[15:0] ||
          old.normalized_width(level[7:0]) !== width[15:0])
        $fatal(1, "Arithmetic mismatch command=%0d width=%0d", i, width);
      arithmetic_checks = arithmetic_checks + 1;
    end
    // Preserve the old table's documented unknown-level fallback separately.
    if (old.normalized_width(8'bxxxxxxxx) !== 16'd0 ||
        old.normalized_width(8'bzzzzzzzz) !== 16'd0 ||
        dut.normalized_width(8'bxxxxxxxx) !== 16'd0 ||
        dut.normalized_width(8'bzzzzzzzz) !== 16'd0)
      $fatal(1, "Old unknown table fallback changed");
    duty = 0;
    @(negedge clk_50m); rst_n = 1'b1;
    pulse_check(1); pulse_check(-1);
    pulse_check(210); pulse_check(-210);
    pulse_check(255); pulse_check(-255);
    pulse_check(270); pulse_check(-270); pulse_check(-32768); pulse_check(32767);
    hold_command(0, 10001);
    // Change amplitude away from period boundaries; width must hold until carrier0.
    hold_command(1, 17001); hold_command(210, 2317); hold_command(255, 9321);
    // Reverse, then reverse again before blank expires.
    hold_command(-210, 11); hold_command(210, 17); hold_command(-1, 12001);
    @(negedge clk_50m); enable = 0;
    repeat (5123) @(negedge clk_50m);
    hold_command(-255, 15001);
    while (carrier != 4999) @(negedge clk_50m);
    duty = 0;
    @(negedge clk_50m); duty = 210;
    while (carrier != 0) @(negedge clk_50m);
    duty = -210;
    repeat (12000) @(negedge clk_50m);
    async_reset;
    hold_command(210, 15001);
    hold_command(-210, 5); async_reset;
    hold_command(-32768, 15001);
    for (i = 0; i < 600; i = i + 1) begin
      @(negedge clk_50m);
      rng = rng ^ (rng << 13); rng = rng ^ (rng >> 17); rng = rng ^ (rng << 5);
      duty = rng[15:0]; enable = (i % 11 != 0);
      repeat ((rng & 1023) + 1) @(negedge clk_50m);
    end
    hold_command(0, 5001);
    if (arithmetic_checks != 65536 || pulse_periods != 10 || boundary_mask != 3 ||
        reset_events != 2 || reblank_events == 0)
      $fatal(1, "Coverage guard failed");
    $display("PASS MOTOR cycles=%0d checks=%0d signed16=%0d pulse_periods=%0d boundary_mask=%0d resets=%0d reblank=%0d",
             cycles, checks, arithmetic_checks, pulse_periods, boundary_mask,
             reset_events, reblank_events);
    $finish;
  end
  initial begin
    #100000000;
    $fatal(1, "Global timeout");
  end
endmodule
