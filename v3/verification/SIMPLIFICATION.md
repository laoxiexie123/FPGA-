# 立摆简化：分阶段迁移记录

## 后续更新：诊断端口暂不保留

用户后续明确移除诊断端口，当前src仅保留motor_command、result_valid、calculating、control_fault、fault_code。删除原10个数值/序号端口及纯观测position_update，删除对应10个输出寄存器和2个仅供遥测的候选寄存器。result_valid并入主计算块，同沿提交规则不变。当前为31个时序变量、1个组合reg，较原带诊断直接版43个减少12个。

本文件其余内容为上一轮带诊断迁移历史，其快照、综合报告和哈希不覆盖、不伪装成当前版本。当前改动验证保留的控制输出逐拍一致；本轮未重跑综合，不引用旧FF/LUT/Fmax作为当前资源结果。

## 上轮明确范围

用户选择先做阶段1/2/3，删除位置积分和外部Q16参考；保留默认角度PD、位置PD、位置低通、固定原点、接管权重、故障与同沿提交合同。额外尝试取消共享乘法器，以实际Gowin模块综合比较资源，不仅凭仿真判断可行性。诊断输出不延迟一拍，不裁剪现有诊断端口（已删除功能的积分输出除外）。

## 常见信号说明

- **计算请求**：idle且数据已建立时接受tick_5ms。5ms是控制间隔，不是计算必须耗时5ms。
- **提交拍数**：接受请求的时钟沿到result_valid发布新结果的沿之间相隔多少个50MHz周期。1拍为20ns；这与5ms=250000拍不同。位置环到期、滤波初值与权重分支可能有不同计算拍数，必须按路径分别比较。
- **诊断端口**：用于解释一次输出的误差、滤波值、分项及更新次数。不是额外控制器，也不直接给电机输出。发布时刻保持与控制结果同沿。
- **控制接口**：motor_command/result_valid/control_fault/calculating等属于运行握手，不是可随意取消的诊断。

## 回退基线

[migration_baseline](migration_baseline/source_hashes.json)登记原始控制源码、测试台和运行脚本SHA256；字节副本保存在同目录。该目录不加入Gowin工程，不用于最终硬件。回退应连同匹配的测试和接口恢复，不能只换一份src而仍使用新接口测试。

## 阶段与验证门槛

| 阶段 | 改动 | 验证 | 回退点 |
|---|---|---|---|
| 1 | 六个固定参数快照改常量，两个角度增益快照改增强选择位 | 全旧功能双模式逐拍、严格lint，状态和提交沿不变 | baseline |
| 2 | 历史、诊断成为单一写入者的独立块；共享组合提交条件 | 全输出同沿对照，真实commit同沿五类抢占，原点/关闭位置环历史；正式回退用stage2_fixed修订，普通if仲裁还原X条件处理 | stage1快照 |
| 3 | 删除积分、外部Q16参考及相应接口和状态；仅保留位置PD固定原点目标 | 基线Ki0/ref0默认路径对照、负数舍入、位置周期、非法状态及真实交接 | stage2快照及旧接口测试 |
| direct候选 | 语义乘法替代公共操作数/返回寄存器，保留原请求/提交时刻 | 默认PD逐拍、实际模块综合与stage3同设置比较 | stage3共享乘法器快照 |

阶段快照及日志在migration_stages。阶段结果中src路径表示当时参与运行的原路径，核验历史版本应对照同stage快照字节哈希，不应与已演进的当前src直接比较。baseline及各阶段是回退/验证资料，不是并行实例化的控制模块。

## 已实施结果

最终采用[直接公式版](../src/balance_control.v)，与[migration_stages/direct_balance.v](migration_stages/direct_balance.v)字节一致，SHA256为`61ffa6effb2268af2de4c5887fe3c37b76d687a5b23c2d479a22160a550e3891`。保留[migration_stages/stage3_balance.v](migration_stages/stage3_balance.v)作为同功能共享乘法器回退点。

| 版本 | 时序寄存器变量 | 时序声明位数 | 源码行数 |
|---|---:|---:|---:|
| 基线 | 71 | 2192 | 807 |
| 阶段1 | 64 | 1969 | 789 |
| 阶段2修订回退 | 64 | 1969 | 972 |
| 阶段3共享版 | 48 | 1262 | 403 |
| 最终直接公式版 | 43 | 1066 | 537 |

阶段3和直接版另有1个`reg legal_state`，在完整组合always中赋值，不是时序存储。统计器输出所有reg语法数分别49/44，须与48/43个时序变量区分。声明位数不等于综合FF。直接版行数比阶段3长是因为声明与赋值逐行展开，不为追求行数把多条语句挤在一起。原阶段2是只分块的中间兼容版本，不作为最终阅读目标。

[可复现统计脚本](audit_balance_complexity.py) / [逐变量账本](balance_complexity.json)。最终43个时序变量按主要职责互斥归类：

- **控制与算法状态12个**：motor_command、result_valid、control_fault、fault_code、position_loop_active_q、position_holds_q、position_seeded_q、angle_history_valid_q、position_history_valid_q、position_filter_history、position_target、position_offset。
- **计算及调度中间16个**：calculating、state_q、position_filter_candidate、angle_error、precise_angle_error、angle_difference、position_error、position_difference、position_filtered_candidate、angle_command_candidate、position_offset_candidate、motor_command_candidate、proportional_derivative_sum、compensated_motor_command、raw_motor_command_candidate、position_due_q。
- **历史与输入快照5个**：previous_angle_error、previous_position_error、boost_snapshot、position_sample、position_weight。
- **诊断10个**：position_filtered_counts、angle_error_adc_x4、angle_delta_adc_x4、position_error_counts、position_delta_counts、angle_command、angle_offset_adc_x4、raw_motor_command、control_sequence、position_sequence。
- **积分/外部参考0个**：原16个扩展寄存器全部删除，不再提供相应4个接口。

公共操作数/返回/乘积寄存器和临时比例项已删除：multiply_return_q、multiply_a_q、multiply_b_q、multiply_result_q、proportional_term。angle_proportional、angle_derivative、position_proportional、position_derivative现在是语义组合表达式。固定等待状态只为保留旧提交沿，不再调度一个公共乘法器。

### 实测提交时刻与测试

最终核心双模式各3259周期、2474次旧默认PD逐拍对照、229项检查、118次提交、113次独立角度模型检查。旧核心Ki=0、外部参考无效；不声称被删除功能等价。五类真实提交同沿取消、已删除状态编码、X/Z正常状态恢复、未知状态与origin/关闭位置环/clear/capture同沿，以及新增等待状态32..36的忙时tick和取消均有定向检查。新增等待状态覆盖11111。

测试测得从接受请求沿到提交沿：普通角度10拍=200ns；已建立滤波历史的位置更新21拍=420ns；部分权重位置更新24拍=480ns。它们是所测路径，不代表所有首次初始化分支都相同；逐拍对照覆盖了首位置事务。5ms控制间隔仍为250000个50MHz周期。

最终真实ADC+起摆+直接公式PD集成双模式各3750125周期、15次观测、2组原点检查通过，运行源码哈希稳定。0.5秒等待仅注入到末两沿，不宣称完整长等待仿真或板级闭环。

stage1保留完整旧功能的对照为4945周期/4880比较/291检查；阶段2正式回退使用stage2_fixed，修复四态事件仲裁并独立复验完整积分/参考路径。原stage2快照保留历史，不覆盖证据。各阶段严格lint及Verilog-2005/2012结果、精确源码/测试哈希在migration_stages。

### 实际综合比较

工具Gowin V1.9.12.03，器件与同设置相同，全部剩余诊断端口导出，关闭IO插入，只运行syn。最终报告见[综合说明](synthesis_compare/README.md) / [资源摘要](synthesis_compare/report_summary.json)。

| 指标 | 阶段3共享版 | 最终直接公式版 |
|---|---:|---:|
| FF | 1064 | 939 |
| LUT，不含另列INV | 2340 | 2016 |
| ALU | 443 | 380 |
| MULT36X36 | 1 | 4 |
| MULTALU36X18 | 0 | 1 |
| Latch / BSRAM | 0 / 0 | 0 / 0 |
| 综合估算Fmax | 76.846MHz | 62.149MHz |

取消共享减少125 FF、324 LUT、63 ALU，但增加乘法原语、估算时序余量减小。不能把原语数直接称为物理DSP块数，也不能把综合估算当布局布线收敛。声明寄存器减少不一定物理资源减少：阶段1相对基线FF/LUT还略增，详见报告；最终下降包含删除功能，只有shared/direct同功能比较可隔离乘法组织变化。

## 时钟、电气和验证边界

器件来源为v3.gprj中的GW2A-LV55PG484C8/I7。所有新增/调整寄存器仍clk_50m单域，rst_n低有效异步复位，系统集成负责同步释放；无新增CDC。tick_5ms为同域使能。物理引脚、IO标准、时钟源及板级约束不修改；不设置Bank供电、不推测任何物理引脚。综合采用无IO插入的模块评估，不运行布局布线或生成/烧写位流。

仿真通过为II级；实际综合报告通过可称对应模块III级，但不能等同完整top可实现、50MHz时序收敛或硬件稳定。最后需要依据本机工具结果如实记录，不将计划当作完成。

## 解释诊断值

| 输出 | 用途 |
|---|---|
| angle_error_adc_x4 / angle_delta_adc_x4 | 观察角度误差及其变化 |
| position_error_counts / position_delta_counts | 判断位置偏移及变化 |
| position_filtered_counts | 检查位置滤波及原点建立 |
| angle_command | 角度PD量化结果 |
| angle_offset_adc_x4 | 位置环生成的角度参考偏置 |
| raw_motor_command | 摩擦补偿前命令 |
| control_sequence / position_sequence | 确认新结果及25ms位置更新是否持续发生 |

这些输出若先经result_valid寄存一拍再更新，会比motor_command晚一拍，本轮不采用；统一由本沿组合提交条件更新。
