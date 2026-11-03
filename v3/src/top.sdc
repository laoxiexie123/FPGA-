# 沿用v2.2/src/dual_pd.sdc的唯一50MHz系统时钟。
create_clock -name clk_50m -period 20 -waveform {0 10} [get_ports {clk_50m}]
# 异步按键/编码器只进入各自两级同步器；复位异步拉低、两级同步释放。
set_false_path -from [get_ports {enc1_a}]
set_false_path -from [get_ports {enc1_b}]
set_false_path -from [get_ports {rst_n}]
# ADC接口沿用旧采样关系；片外建立保持/板延迟尚无确认值，不能编造input_delay。
# 本文件不代表ADC外部接口时序已闭合，仍需手册/原理图和布局布线报告确认。
set_false_path -from [get_ports {ec11_a}]
set_false_path -from [get_ports {ec11_b}]
set_false_path -from [get_ports {ec11_button_n}]
