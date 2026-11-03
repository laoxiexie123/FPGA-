# 脉冲起摆与摆杆观测

## 本次边界

按最新要求：ADC集中采样、平均、原ADC低通、样龄、摆角映射、角差测速、速度低通及有效性检查；独立观测源码已删除。swing_control管理脉冲、捕获准入和交接。冗余起摆连接层已删除，由测试台直接连接ADC和控制，后续由系统top接入同一套PD核心。

已迁移[立摆核心](../src/balance_control.v)，以motor_command/result_valid/control_fault接入起摆core_duty/core_commit/core_fault；本轮明确删除位置积分及外部Q16轨迹参考接口；外层run_guard及板级top尚未迁移。真实节拍集成使用真实PD，不再使用握手替身；不宣称已完成板级闭环。新增位置会话输出及立摆接口见[立摆说明](BALANCE.md)。

## 文件

- [adc_pend.v](../src/adc_pend.v)：包含采样、原ADC低通、样龄和完整摆杆观测流水。新增tick_5ms/zero_adc_x4输入及sample_commit_q、observer_busy_q、pendulum_valid_q、angle_cdeg_q、speed_cdeg_s_q、observer_seq_q、observer_fault_q输出。每个always前说明职责、时钟及复位，平均、滤波、观测算法和原更新时序保持。
- [swing_control.v](../src/swing_control.v)：固定原模式3参数，删除模式0直接平衡和模式2诊断路径。
- 捕获准入已移入起摆控制：新增average_raw、filtered_adc_x4、zero_adc_x4输入，原始/滤波误差同时不超过380，保持原一拍寄存。独立core_capture_ready输入已移除。
- 系统top应把测量清历史信号同时接到ADC的filter_clear和控制的stop_request；当前直接连接测试台已验证该合同，不新增替代连接源码。

## 时钟、引脚与标定

目标沿用GW2A-LV55PG484C8/I7，纯Verilog2005；50MHz单域，tick_1ms/tick_5ms为外部统一单拍使能，不产生新时钟域。ADC二分频仅接口输出；本轮无新增物理引脚绑定/IO标准/约束，也不下载位流。

zero_adc_x4是外部提供的已校准零点，范围0..4092，变更后需measurement_clear；不硬猜某块板的物理零位。measurement_clear同时清测量历史并停止当前起摆，需要新启动沿再次开始。

## 测量数据与标志

```text
ADC引脚 → 八点平均 ┬→ 原Q8低通 → 四倍标度ADC → 外部平衡PD
                  └→ 摆角映射 → 5ms角差×200 → Q8速度低通 → 起摆判断
```

ADC的angle_valid_q是四倍标度滤波结果更新事件，不是物理角度有效性；起摆angle_valid应连接ADC的pendulum_valid_q。立摆adc_ready应连接filter_ready_q，不接单拍滤波更新事件。

- sample_commit：单拍，表示发布新观测；不得每系统拍重复累计捕获驻留。
- angle_valid：观测可用于触发/捕获；首次只建历史，连续两个合法样本后有效。
- observer_fault：配置/算术/忙tick等锁存错误；普通窗口/映射/跳变/速度越界仅使结果无效并重建历史。
- average_fresh_next：至少见过平均结果且样龄严格小于20ms。消费average_valid的下一沿清龄，优先于同沿tick；样龄饱和，滤波清历史不清采集样龄。

观测固定映射系数8518，除1024向零截断；ADC合法1..1022；原始速度上限200000 cdeg/s，速度低通16/256；重捕获2样本，相邻角度变化上限18000cdeg。角速度单位cdeg/s，电机编码器臂速单位counts/s，两者不能混用。

## 主状态与内部阶段

| main_state_q | 职责 | 内部phase_q |
|---|---|---|
| 0 | 停止/等待新的启动沿 | 0 |
| 1 | 脉冲起摆 | 首脉冲13、反刹11、等触发14、后续脉冲15、滑行12 |
| 2 | 尝试接管 | 等核心首提交4、混合5 |
| 3 | 正常立摆 | 6 |

为保留逐拍行为，实际使用一个具名内部阶段FSM，并映射为四个主状态；不是另写两套并行PD或两个独立状态机。输出经过原12级流水，携带phase/epoch校验，状态切换的旧候选不得提交。

固定首/后脉冲210、200ms；反刹200、100ms；总脉冲2；触发窗8500cdeg、重新武装需超过9000cdeg且新有效提交。方向固定原C45=1。起摆变化率200/控制更新，交接20/更新，balance跳过此斜率级。最大命令210。

捕获条件：新有效观测，角度<=2500cdeg、角速度<=100000cdeg/s、臂速<=800counts/s、原始与滤波ADC对零点误差均<=380（原C14）。ADC误差准入保留一拍寄存。驻留按每个合格提交累计5ms，当前门限5ms故一个合格提交足够；脉冲/反刹结束分支优先，同沿样本不再捕获。

捕获后初始化外部PD、保持命令等首次commit，再执行5ms混合。权重32768且流水候选与当前core_duty一致才进入3，不能只看计时。entry_boost从捕获起持续2000个1ms使能，可跨入状态3；外部同一PD应将角度P/D增益增至1.5倍、位置增益保持原值。entry_boost在起摆模块只是选择信号，立摆核心事务锁存1.5倍角度增益。

状态3连续0.5秒后，position_loop_enable开放并发position_origin_seed单拍，下一拍position_ready有效。停止、故障、重新捕获或退出状态3取消计时及会话。位置环使能保留旧top的锁定寄存器时序，原点/ready直接许可门控；不新增第二套PD。

## 失败与停止

等待触发2000ms、起摆总10000ms、核心无提交20ms超时；接管/平衡阶段新提交角度无效、超过3200cdeg或摆速超过200000cdeg/s则退出。

失败下一系统沿清输出，进入0保留fault_q；必须新的start_request上升沿才重试。start始终高不会自动重试，stop/safety优先于启动。外层仍须提供限位、编码器等safety_fault和板级驱动许可，本层不能替代完整run_guard。

故障码沿旧监督器：1非法状态，3等待/总超时，4外层或观测/核心故障，5捕获丢失，6核心提交超时。

明确差异：失败由旧锁故障状态改为0保原因；start沿启动；删除非脉冲模式；补齐旧boost/brake_last异步复位，消除上电未知值。正常运行算法和命令流水保留。未声称所有故障路径逐拍等价。

## 验证

- [ADC结果](adc_test_results.json)：两语言模式各4453144运行拍旧链对照及独立数值模型；额外新鲜度20ms边界、同沿优先级及饱和检查。
- [观测结果](observer_test_results.json)：旧观测器固定cfg对照和独立数值检查。
- [起摆结果](swing_test_results.json)：加速tick进行完整双脉冲、交接、失败和boost检查；不能将加速tick称为真实时间板级验证。
- [集成结果](startup_test_results.json)：真实50MHz/1ms/5ms节拍，ADC到观测到早捕获交接，使用真实立摆PD，检查停止/heldstart/measurement_clear及实际核心对原点seed的消费。长原点等待预置至末两沿，未完整运行0.5秒。

Verilog2005/2012对照：观测1829拍、160次数学事务；起摆303527拍、302410次旧正常路径对照、86项定向断言。真实PD连接的最终周期及状态见集成结果JSON，须status=passed及当前源码哈希匹配才表示该版本通过。

严格lint保留旧混合运算有意截断高位UNUSEDSIGNAL的局部豁免；冗余连接源码已删除，不再有该源码的空诊断连接豁免。验证仅I/II级，不等于综合通过、时序收敛或上板稳定。
