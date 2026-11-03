# 立摆控制：角度PD与位置PD

## 当前接口更新：移除诊断

按后续要求，当前源码移除10个数值/序号诊断端口及纯观测position_update，共11个输出。保留motor_command、result_valid、calculating、control_fault、fault_code五个控制输出。删除12个仅供诊断的时序寄存器（含raw_motor_command_candidate与position_filtered_candidate），result_valid并入主计算块，仍由同沿commit_now驱动；当前位置偏置、历史和25ms节奏不变。源码现在为两段时序always，状态编码未改，原25个状态说明保留。

下文涉及诊断和Gowin资源的部分记录上一轮带诊断版本，不代表当前接口或当前资源。当前版本重新执行严格lint与保留控制接口逐拍/真实交接仿真；不沿用旧综合结果声称本次资源或时序已确认。

## 上轮迁移口径

[balance_control.v](../src/balance_control.v)实现位置外环生成角度偏置、角度内环产生电机命令。用户本轮明确删除位置积分及外部Q16参考：不保留Ki、轨迹参考输入或积分诊断输出；固定原点目标、位置滤波、位置PD及接管权重保留。旧完整实现只作为[回退基线](migration_baseline/balance_control.v)，不加入当前工程。

分阶段改动、回退点及综合比较入口见[简化记录](SIMPLIFICATION.md)。阶段1去常量快照；阶段2分离计算、历史和诊断；阶段3裁掉明确不使用的扩展。最终已采用直接公式乘法版：默认PD逐拍仿真与实际Gowin同设置模块综合均通过；共享乘法器版保留为独立回退快照。各阶段记录是历史验证，最终src以当前测试哈希为准。

## 主要数据流

```text
ADC四倍标度 / 原Q8滤波历史
    → 减直立零点、加已发布位置偏置
    → 角度P + 角度差分D
    → 同向摩擦补偿
    → ±150命令限幅
    → motor_command

编码器位置
    → 每25ms位置低通
    → 减固定原点目标
    → 位置P + 位置差分D
    → ±150偏置限幅及接管权重
    → 提交位置偏置，下个角度事务生效
```

角度每接受一次5ms请求计算；位置第1、6、11…次成功提交更新。P使用四倍ADC整数误差，D使用Q8精细误差的相邻已提交差值；首次D为零，不用观测摆速替代，不乘200。

角度量化保持`trunc_zero((Kp*error_x4 + floor(Kd*delta_q8/64))/65536)`。位置滤波增量、位置PD量化和权重计算保留向零截断。分块不改变非阻塞赋值关系：只有成功提交才保存正常运行历史，新的位置偏置不会提前参与本次角度计算。

## 固定参数

| 参数 | 数值 |
|---|---:|
| 正常角度P / D，Q16 | 80000 / 350000 |
| 增强角度P / D，Q16 | 120000 / 525000 |
| 位置P / D，Q16 | 10000 / 500000 |
| 位置低通系数 | 64/256 |
| 位置产生的角度偏置上限 | ±150，四倍ADC单位 |
| 同向摩擦补偿 | 1，零命令不补偿 |
| 电机命令上限 | ±150 |

固定参数来自[v2.2本地配置](../../../v2.2/src/local_startup_profile.v)及[原top](../../../v2.2/src/top.v)。角度增强状态在接受计算请求时锁存，事务中改变entry_boost不应改变本次增益。位置原点由现有起摆控制在进入正常立摆持续0.5秒后建立。

## 当前控制接口

- enable允许计算；撤销中止事务并清输出，不清锁存故障。
- clear清控制历史及故障，不清外部ADC；capture_seed清控制历史并保留已有故障与ADC。
- filtered_adc_x4 / filtered_adc_q8来自同一ADC滤波历史；合法范围0..4092、0..261888。
- adc_ready是滤波已有初值，不是数据新鲜度或单拍更新事件；上层仍须提供安全许可。
- zero_adc_x4是外部已校准零点，不猜板上零位；position_counts范围±1000000。
- position_loop_enable / position_origin_seed由起摆控制提供；position_weight_q15保留0..32768权重。
- motor_command、result_valid、calculating、control_fault/fault_code属于控制及握手接口。
- 诊断端口angle_error_adc_x4、angle_delta_adc_x4、position_error_counts、position_delta_counts、position_filtered_counts、angle_command、angle_offset_adc_x4、raw_motor_command及两个sequence均已删除，纯观测position_update也删除。

result_valid在主计算块每拍默认清零，只在publish_control_result分支置1，避免未知状态下把X传播为提交事件。内部position_offset及历史仍按原同沿组合提交条件保存。测试台仅通过现有算法状态和测试专用计数观测位置节奏，不增加RTL诊断影子寄存器。

先前已删除扩展接口：position_reference_valid、position_reference_q16、position_ki_q16、position_integral_adc_x4。

## 优先级与会话

clear > capture_seed > 故障 > 停止 > 原点建立 > 位置关闭沿 > 正常推进。不同事件对历史、误差诊断和输出的清除/保持不同，不能统一清零。原点建立保留角度历史及已发布命令，取消在途提交并清位置历史与偏置。

[swing_control.v](../src/swing_control.v)保持正常立摆连续25000000个50MHz周期后锁定原点并发单拍seed，下一拍position_ready有效。停止、故障、重新捕获或退出正常立摆取消会话。[真实PD集成测试](../tb/tb_pendulum_startup.v)直接连接ADC、起摆和立摆，不新增RTL连接层。

## 验证与硬件边界

- [核心结果](balance_test_results.json)：当前无诊断版双模式各3259周期、2474次保留控制输出逐拍对照、236项检查、113次独立角度模型检查。旧核心以Ki=0、外部参考无效对照；不能称完整积分/参考功能等价。
- [ADC结果](adc_test_results.json)：验证Q8历史正常路径；不声称旧控制故障冻结ADC的耦合保持。
- [起摆结果](swing_test_results.json)：起摆/原点合同未改。
- [真实节拍结果](startup_test_results.json)：最终直接公式版两模式各3750125周期、15次观测、2组原点交接检查通过，源码哈希稳定；实际ADC和真实PD交接，长0.5秒原点等待仅预置末两沿，未完整仿真。
- [Gowin正式综合](synthesis_compare/README.md)：最终模块FF939、LUT2016、ALU380、Latch0；MULT36X36为4、MULTALU36X18为1，估算Fmax62.149MHz。属于模块III级，不是完整系统或布局布线时序证明。该旧报告对应43个时序变量版本；当前无诊断版31个时序变量、另1组合译码变量，本轮仅I/II级验证，未重跑综合。逐变量账本及比较见[简化记录](SIMPLIFICATION.md)。

沿用v3.gprj器件GW2A-LV55PG484C8/I7和50MHz单逻辑域；无新增CDC或引脚约束。只评估模块综合，不跑布局布线、不生成或下载位流。完整top、run_guard、物理IO时序、电气规格、受控上板验证仍待后续完成。
