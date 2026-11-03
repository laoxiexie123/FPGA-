`timescale 1ns/1ps
// Fixed-four-cycle receiver: independent cycle model, old decoder and top speed shadow.
module tb_encoder;
  reg clk_50m = 1'b0;
  always #10 clk_50m = ~clk_50m;
  reg rst_n = 1'b0, enc_a = 1'b0, enc_b = 1'b0, zero = 1'b0;
  reg tick_10ms = 1'b0;
  wire signed [31:0] position_q, speed_q, old_position;
  wire [31:0] invalid_count_q, old_invalid;
  wire overflow_fault_q, old_fault;
  encoder dut(.clk_50m(clk_50m), .rst_n(rst_n), .enc_a(enc_a), .enc_b(enc_b),
      .zero(zero), .tick_10ms(tick_10ms), .position_q(position_q), .speed_q(speed_q),
      .invalid_count_q(invalid_count_q), .overflow_fault_q(overflow_fault_q));
  encoder_guard old_encoder(.clk(clk_50m), .rst_n(rst_n), .enc_a(enc_a), .enc_b(enc_b),
      .zero(zero), .debounce_cycles(32'd4), .position_q(old_position),
      .invalid_count_q(old_invalid), .overflow_fault_q(old_fault));

  // Literal old top speed arithmetic/priority, with names changed only for shadow wiring.
  reg signed [31:0] old_previous, old_speed;
  wire signed [32:0] old_difference = $signed({old_position[31], old_position}) -
      $signed({old_previous[31], old_previous});
  wire signed [39:0] old_product = old_difference * 40'sd100;
  always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin old_previous <= 0; old_speed <= 0; end
    else if (zero) begin old_previous <= 0; old_speed <= 0; end
    else if (tick_10ms) begin
      old_previous <= old_position;
      if (old_product > 40'sd2147483647) old_speed <= 32'sh7fffffff;
      else if (old_product < -40'sd2147483648) old_speed <= 32'sh80000000;
      else old_speed <= old_product[31:0];
    end
  end

  integer sync_a0, sync_a1, sync_b0, sync_b1, warm;
  integer candidate, accepted, stable, primed, sample, delta;
  reg signed [31:0] model_position, model_previous, model_speed;
  reg [31:0] model_invalid;
  reg model_fault;
  reg signed [63:0] wide_speed;
  integer cycles = 0, checks = 0, resets = 0, natural_ticks = 0;
  integer transition_mask = 0, directed_checks = 0, random_cycles = 0;
  integer boundary_checks = 0;
  function integer direction;
    input integer before_phase, after_phase;
    begin
      // Cyclic phase ordinal is independent of the decoder's eight bit-pattern predicates.
      case (before_phase)
        0: direction = 0;
        2: direction = 1;
        3: direction = 2;
        default: direction = 3;
      endcase
      case (after_phase)
        0: direction = (0 - direction + 4) % 4;
        2: direction = (1 - direction + 4) % 4;
        3: direction = (2 - direction + 4) % 4;
        default: direction = (3 - direction + 4) % 4;
      endcase
      if (direction == 3) direction = -1;
    end
  endfunction
  always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
      sync_a0 = 0; sync_a1 = 0; sync_b0 = 0; sync_b1 = 0; warm = 0;
      candidate = 0; accepted = 0; stable = 0; primed = 0;
      model_position = 0; model_previous = 0; model_speed = 0;
      model_invalid = 0; model_fault = 0;
    end else begin
      cycles = cycles + 1;
      // Speed observes position BEFORE this edge's phase acceptance (NBA contract).
      if (zero) begin model_previous = 0; model_speed = 0; end
      else if (tick_10ms) begin
        wide_speed = ($signed(model_position) * 64'sd1 -
            $signed(model_previous) * 64'sd1) * 64'sd100;
        model_previous = model_position;
        if (wide_speed > 2147483647) model_speed = 32'sh7fffffff;
        else if (wide_speed < -64'sd2147483648) model_speed = 32'sh80000000;
        else model_speed = wide_speed[31:0];
      end
      sample = sync_a1 * 2 + sync_b1;
      if (zero) begin
        model_position = 0; model_fault = 0;
        candidate = sample; accepted = sample; stable = 0; primed = 0;
      end else if (warm < 4) begin
        candidate = sample; accepted = sample; stable = 0;
      end else if (sample != candidate) begin candidate = sample; stable = 1; end
      else if (stable < 4) stable = stable + 1;
      else if (!primed) begin accepted = candidate; primed = 1; end
      else if (candidate != accepted) begin
        delta = direction(accepted, candidate);
        if (delta == 1) begin
          if (model_position == 32'sh7fffffff) model_fault = 1;
          else model_position = model_position + 1;
        end else if (delta == -1) begin
          if (model_position == 32'sh80000000) model_fault = 1;
          else model_position = model_position - 1;
        end else if (model_invalid != 32'hffffffff) model_invalid = model_invalid + 1;
        accepted = candidate;
      end
      if (warm < 4) warm = warm + 1;
      sync_a1 = sync_a0; sync_a0 = enc_a;
      sync_b1 = sync_b0; sync_b0 = enc_b;
    end
    #1;
    checks = checks + 1;
    if (position_q !== model_position || speed_q !== model_speed ||
        invalid_count_q !== model_invalid || overflow_fault_q !== model_fault)
      $fatal(1,"model mismatch cycle=%0d pos=%0d/%0d speed=%0d/%0d invalid=%0d/%0d",
          cycles,position_q,model_position,speed_q,model_speed,invalid_count_q,model_invalid);
    if (position_q !== old_position || invalid_count_q !== old_invalid ||
        overflow_fault_q !== old_fault || speed_q !== old_speed)
      $fatal(1,"old chain mismatch cycle=%0d",cycles);
  end
  task step;
    input integer n;
    begin repeat(n) @(negedge clk_50m); end
  endtask
  task phase;
    input [1:0] value;
    begin {enc_a,enc_b}=value; step(12); end
  endtask
  task ck;
    input condition;
    begin directed_checks=directed_checks+1;
      if(condition!==1'b1)$fatal(1,"directed check %0d failed",directed_checks);
    end
  endtask
  task clear;
    begin zero=1;step(1);zero=0;step(12);end
  endtask
  task reset_at_phase;
    input [1:0] value;
    begin
      #3 rst_n=0; {enc_a,enc_b}=value; resets=resets+1;
      #2; ck(position_q===0 && speed_q===0 && invalid_count_q===0 && !overflow_fault_q);
      #22 rst_n=1; step(20); ck(position_q===0 && invalid_count_q===0);
    end
  endtask
  task deposit;
    input signed [31:0] pos, prev;
    input [31:0] invalid;
    begin
      dut.position_q=pos;old_encoder.position_q=pos;model_position=pos;
      dut.previous_position_q=prev;old_previous=prev;model_previous=prev;
      dut.invalid_count_q=invalid;old_encoder.invalid_count_q=invalid;model_invalid=invalid;
    end
  endtask
  task speed_boundary;
    input signed [31:0] pos, prev, expected;
    begin
      deposit(pos,prev,0);tick_10ms=1;step(1);tick_10ms=0;
      ck(speed_q===expected);boundary_checks=boundary_checks+1;
    end
  endtask
  integer a,b,i,k,hold_cycles;
  reg [31:0] rng=32'h2178bafe;
  initial begin
    #1 rst_n=0;#24 rst_n=1;step(20);
    // All sixteen settled AB transitions, including no change and diagonals.
    for(a=0;a<4;a=a+1) begin
      for(b=0;b<4;b=b+1) begin
        phase(a[1:0]);clear;phase(b[1:0]);
        transition_mask=transition_mask | (1 << (a*4+b));
      end
    end
    for(a=0;a<4;a=a+1)reset_at_phase(a[1:0]);
    // Warm-up activity must not manufacture initial motion.
    rst_n=0;step(2);rst_n=1;
    phase(2'b10);phase(2'b11);clear;
    // Stable candidates for one through six clocks exercise threshold boundaries.
    for(hold_cycles=1;hold_cycles<=6;hold_cycles=hold_cycles+1) begin
      phase(0);clear;{enc_a,enc_b}=2'b10;step(hold_cycles);
      {enc_a,enc_b}=0;step(14);
      if(hold_cycles<4)ck(position_q==0);
    end
    phase(0);clear;{enc_a,enc_b}=2'b10;
    // Two synchronizer cycles plus candidate capture, then threshold increment.
    step(6);ck(position_q==0);step(1);ck(position_q==1);
    // Accepted phase and speed tick share an edge: speed must see the old position.
    phase(0);clear;{enc_a,enc_b}=2'b10;step(6);
    tick_10ms=1;step(1);tick_10ms=0;ck(position_q==1 && speed_q==0);
    tick_10ms=1;step(1);tick_10ms=0;ck(speed_q==100);
    // Same acceptance edge zero wins; held zero repeats and tick history stays zero.
    phase(0);clear;{enc_a,enc_b}=2'b10;step(6);
    zero=1;tick_10ms=1;step(4);ck(position_q==0 && speed_q==0);
    zero=0;tick_10ms=0;step(12);ck(position_q==0);
    phase(0);clear;deposit(32'sh7fffffff,0,0);phase(2);
    ck(position_q==32'sh7fffffff && overflow_fault_q);phase(0);
    ck(position_q==32'sh7ffffffe && overflow_fault_q);
    clear;phase(0);deposit(32'sh80000000,0,0);phase(1);
    ck(position_q==32'sh80000000 && overflow_fault_q);phase(0);
    ck(position_q==32'sh80000001 && overflow_fault_q);
    clear;deposit(0,0,32'hffffffff);phase(3);
    ck(invalid_count_q==32'hffffffff);phase(1);ck(position_q==1);
    clear;phase(0);
    speed_boundary(21474836,0,2147483600);
    speed_boundary(21474837,0,32'sh7fffffff);
    speed_boundary(-21474836,0,-2147483600);
    speed_boundary(-21474837,0,32'sh80000000);
    speed_boundary(32'sh7fffffff,32'sh80000000,32'sh7fffffff);
    speed_boundary(32'sh80000000,32'sh7fffffff,32'sh80000000);
    clear;
    // Three actual 500000-clock (10ms) windows; input edges happen away from tick.
    for(k=0;k<3;k=k+1) begin
      phase(2);phase(3);phase(1);phase(0);
      step(500000-49);tick_10ms=1;step(1);tick_10ms=0;
      natural_ticks=natural_ticks+1;
    end
    for(i=0;i<12000;i=i+1) begin
      rng={rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};
      {enc_a,enc_b}=rng[1:0];zero=(rng[9:2]==8'h21);
      tick_10ms=(rng[13:10]==4'd3);step(1);random_cycles=random_cycles+1;
    end
    zero=0;tick_10ms=0;step(20);reset_at_phase(2'b10);
    ck(transition_mask==65535 && natural_ticks==3 && boundary_checks==6);
    $display("PASS ENCODER cycles=%0d checks=%0d directed=%0d transitions=%0d windows=%0d boundaries=%0d random=%0d resets=%0d",
        cycles,checks,directed_checks,transition_mask,natural_ticks,boundary_checks,random_cycles,resets);
    $finish;
  end
  initial begin #100000000;$fatal(1,"Global timeout");end
endmodule
