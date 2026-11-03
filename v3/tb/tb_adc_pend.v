`timescale 1ns/1ps
// Independent cycle/numeric model plus old ADC/full-core normal-filter shadow.
module tb_adc_pend;
  reg clk_50m = 1'b0;
  always #10 clk_50m = ~clk_50m;
  reg rst_n = 1'b0;
  reg filter_clear = 1'b0;
  reg [9:0] adc_data_in = 10'd0;
  wire adc_clk_next, adc_oe_n_next, average_valid_q, angle_valid_q, filter_ready_q;
  wire [9:0] average_raw_q;
  wire [11:0] angle_x4_q;
  wire signed [31:0] adc_q8;
  adc_pend dut (
      .clk_50m(clk_50m), .rst_n(rst_n), .filter_clear(filter_clear), .tick_1ms(1'b0),
      .adc_data_in(adc_data_in), .adc_clk_next(adc_clk_next),
      .adc_oe_n_next(adc_oe_n_next), .average_raw_q(average_raw_q),
      .average_valid_q(average_valid_q), .angle_x4_q(angle_x4_q), .filtered_adc_q8(adc_q8),
      .angle_valid_q(angle_valid_q), .filter_ready_q(filter_ready_q),
      .average_seen_q(), .average_age_ms_q(), .average_fresh_next(),
      .tick_5ms(1'b0), .zero_adc_x4(12'd2000), .sample_commit_q(), .observer_busy_q(),
      .pendulum_valid_q(), .angle_cdeg_q(), .speed_cdeg_s_q(), .observer_seq_q(),
      .observer_fault_q()
  );
  wire old_clk, old_oe, old_valid;
  wire [15:0] old_raw;
  old_adc_pend old_adc (
      .clk_50m(clk_50m), .rst_n(rst_n), .adc_clk_next(old_clk),
      .adc_oe_n_next(old_oe), .adc_data_in(adc_data_in),
      .pend_raw_q(old_raw), .pend_valid_q(old_valid)
  );
  reg [671:0] old_cfg;
  wire signed [31:0] old_filtered;
  wire old_fault;
  dual_pd_core old_core (
      .clk(clk_50m), .rst_n(rst_n), .tick_5ms(1'b0), .run_enable(1'b0),
      .reseed(filter_clear), .adc_raw(old_raw), .adc_valid(old_valid),
      .position(32'sd0), .cfg(old_cfg), .pwm_limit(16'd150),
      .position_reference_valid(1'b0), .position_reference_q16(64'sd0),
      .position_tick_next(), .position_weight_q15(16'd32768),
      .capture_seed(1'b0), .entry_boost(1'b0), .position_loop_enable(1'b0),
      .position_origin_seed(1'b0), .commit_q(), .busy_q(),
      .adc_filtered_x4_q(old_filtered), .position_filtered_q(),
      .error_angle_q(), .delta_angle_q(), .error_position_q(), .delta_position_q(),
      .pwm1_q(), .pwm2_q(), .mix_q(), .duty_q(), .fault_reason_q(),
      .control_seq_q(), .position_seq_q(), .arithmetic_fault_q(old_fault),
      .position_ki_q16(32'd0), .position_integral_next()
  );

  integer sample_phase, group_count, group_sum, latched_sample;
  integer pending_sample, mean_value, mean_event;
  integer filter_stage, saved_difference, filter_q8, filtered_value;
  integer ready_model, angle_event, clk_model;
  integer cycles = 0, means = 0, updates = 0, checks = 0;
  integer fractional_updates = 0, downward_updates = 0, rounded_groups = 0;
  integer clear_mask = 0, reset_events = 0;
  integer new_mean, next_q8;
  // Independent model uses integer sums and /2 for fixed alpha=128, rather
  // than reproducing the implementation's multiplier and truncation helper.
  always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
      sample_phase = 0;
      group_count = 0;
      group_sum = 0;
      latched_sample = 0;
      pending_sample = 0;
      mean_value = 0;
      mean_event = 0;
      filter_stage = 0;
      saved_difference = 0;
      filter_q8 = 0;
      filtered_value = 0;
      ready_model = 0;
      angle_event = 0;
      clk_model = 0;
    end else begin
      cycles = cycles + 1;
      clk_model = 1 - clk_model;
      angle_event = 0;
      // Consume the PREVIOUS cycle's average event, preserving the NBA boundary.
      if (filter_clear) begin
        clear_mask = clear_mask | (1 << filter_stage);
        if (mean_event) clear_mask = clear_mask | 8;
        filter_stage = 0;
        saved_difference = 0;
        filter_q8 = 0;
        filtered_value = 0;
        ready_model = 0;
      end else if (filter_stage == 0) begin
        if (mean_event) begin
          if (!ready_model) begin
            filter_q8 = mean_value * 256;
            filtered_value = mean_value * 4;
            ready_model = 1;
            angle_event = 1;
          end else begin
            saved_difference = mean_value * 256 - filter_q8;
            filter_stage = 1;
          end
        end
      end else if (filter_stage == 1) begin
        filter_stage = 2;
      end else begin
        next_q8 = filter_q8 + saved_difference / 2;
        if (next_q8 < filter_q8) downward_updates = downward_updates + 1;
        filter_q8 = next_q8;
        filtered_value = next_q8 / 64;
        filter_stage = 0;
        angle_event = 1;
      end
      if (angle_event) begin
        updates = updates + 1;
        if (filtered_value % 4 != 0) fractional_updates = fractional_updates + 1;
      end
      mean_event = 0;
      if (pending_sample) begin
        if (group_count == 7) begin
          new_mean = group_sum + latched_sample;
          if (new_mean % 8 != 0) rounded_groups = rounded_groups + 1;
          mean_value = new_mean / 8;
          mean_event = 1;
          means = means + 1;
          group_count = 0;
          group_sum = 0;
        end else begin
          group_sum = group_sum + latched_sample;
          group_count = group_count + 1;
        end
      end
      pending_sample = 0;
      if (sample_phase == 9374) begin
        sample_phase = 0;
        latched_sample = adc_data_in;
        pending_sample = 1;
      end else sample_phase = sample_phase + 1;
    end
    #1;
    checks = checks + 1;
    if (adc_clk_next !== clk_model[0] || adc_oe_n_next !== 1'b0)
      $fatal(1, "ADC clock/OE mismatch cycle=%0d", cycles);
    if (average_valid_q !== mean_event[0] || average_raw_q !== mean_value[9:0])
      $fatal(1, "Independent average mismatch cycle=%0d", cycles);
    if (angle_valid_q !== angle_event[0] || filter_ready_q !== ready_model[0] ||
        angle_x4_q !== filtered_value[11:0])
      $fatal(1, "Independent filter mismatch cycle=%0d got=%0d expected=%0d", cycles,
             angle_x4_q, filtered_value);
    if (old_clk !== adc_clk_next || old_oe !== adc_oe_n_next ||
        old_valid !== average_valid_q || old_raw !== {6'd0, average_raw_q})
      $fatal(1, "Old ADC shadow mismatch cycle=%0d", cycles);
    if (adc_q8 !== old_core.adc_q8_q)
      $fatal(1, "Q8 output differs from old full-core history cycle=%0d", cycles);
    if (old_fault !== 1'b0 || old_filtered !== $signed({20'd0, angle_x4_q}) ||
        old_core.adc_seeded_q !== filter_ready_q)
      $fatal(1, "Old full-core filter shadow mismatch cycle=%0d old=%0d new=%0d fault=%b",
             cycles, old_filtered, angle_x4_q, old_fault);
  end

  integer drive_mode = 0;
  reg [31:0] rng = 32'h1adcb179;
  // Change pin data BETWEEN sample edges to expose accidental live-pin averaging.
  always @(negedge clk_50m) begin
    rng = {rng[30:0], rng[31] ^ rng[21] ^ rng[1] ^ rng[0]};
    case (drive_mode)
      0: adc_data_in = 10'd0;
      1: adc_data_in = 10'd1023;
      2: adc_data_in = 10'd500;
      3: adc_data_in = 10'd501;
      4: adc_data_in = rng[9:0];
      default: adc_data_in = 10'd17;
    endcase
  end
  task wait_means;
    input integer count;
    integer goal;
    begin
      goal = means + count;
      while (means < goal) @(negedge clk_50m);
      repeat (5) @(negedge clk_50m);
    end
  endtask
  task clear_at_stage;
    input integer stage;
    begin
      while (!(filter_stage == stage && ready_model && !mean_event))
        @(negedge clk_50m);
      filter_clear = 1'b1;
      @(negedge clk_50m);
      filter_clear = 1'b0;
      wait_means(2);
    end
  endtask
  task async_reset;
    begin
      #3 rst_n = 1'b0;
      reset_events = reset_events + 1;
      #2;
      if (angle_x4_q !== 0 || average_raw_q !== 0 || filter_ready_q !== 0 ||
          angle_valid_q !== 0 || average_valid_q !== 0 || adc_clk_next !== 0)
        $fatal(1, "Asynchronous reset did not clear outputs before next clock");
      #22 rst_n = 1'b1;
    end
  endtask
  initial begin
    old_cfg = 672'd0;
    old_cfg[4*32+:32] = 32'd128;
    old_cfg[5*32+:32] = 32'd128;
    old_cfg[19*32+:32] = 32'd200;
    #1 rst_n = 1'b0;
    #24 rst_n = 1'b1;
    wait_means(2);                       // zero and first-sample initialization
    drive_mode = 1; wait_means(5);      // upward step, maximum input
    drive_mode = 0; wait_means(5);      // downward step, signed truncation
    drive_mode = 2; wait_means(5);
    drive_mode = 3; wait_means(5);      // fractional ADC codes survive x4 output
    drive_mode = 2;
    filter_clear = 1'b1;
    @(negedge clk_50m);
    filter_clear = 1'b0;
    wait_means(2);                      // exact 500 seed, no prior-history residue
    drive_mode = 3;
    wait_means(1);
    if (angle_x4_q !== 12'd2002)
      $fatal(1, "Half-code precision lost: 500 -> 501 must first publish 500.5*4");
    drive_mode = 4; wait_means(16);     // nondivisible sums and changing live pins
    clear_at_stage(0);
    clear_at_stage(1);
    clear_at_stage(2);
    while (!mean_event) @(negedge clk_50m);
    filter_clear = 1'b1;               // same edge as filter consumes mean-valid
    @(negedge clk_50m);
    filter_clear = 1'b0;
    wait_means(2);
    while (filter_stage != 1) @(negedge clk_50m);
    async_reset;                       // asynchronous reset during filter pipeline
    wait_means(3);
    while (!(group_count == 3 && sample_phase > 10)) @(negedge clk_50m);
    async_reset;                       // reset during partially accumulated group
    drive_mode = 1; wait_means(3);
    if (clear_mask != 15 || reset_events != 2 || fractional_updates == 0 ||
        downward_updates == 0 || rounded_groups == 0)
      $fatal(1, "Coverage guards failed mask=%0d fractional=%0d down=%0d rounded=%0d",
             clear_mask, fractional_updates, downward_updates, rounded_groups);
    $display("PASS ADC cycles=%0d checks=%0d means=%0d updates=%0d clear_mask=%0d fractional=%0d down=%0d rounded=%0d resets=%0d",
             cycles, checks, means, updates, clear_mask, fractional_updates,
             downward_updates, rounded_groups, reset_events);
    $finish;
  end
  initial begin
    #200000000;
    $fatal(1, "Global timeout");
  end
endmodule
