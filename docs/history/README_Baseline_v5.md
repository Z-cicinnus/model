# 四轮足半人马：后续算法对比基准 v5.0

这是基于三项基础任务的固定平地仿真基准及现有控制器成绩。它不是新的WBC或强化学习算法，也不是实机安全认证。默认物理频率100 Hz（0.01 s），控制指令更新100 Hz。模型、初态、动作、扰动、评分规则固定后，比较后续控制器的表现。

## 安装与运行

本包是已有 Wheel_Centaur 项目的增量更新。先备份同名文件，再将 centaur_control 合并到项目根目录；沿用原有网格。推荐使用本轮验证的 MuJoCo 3.3.7。

```powershell
python -m pip install mujoco==3.3.7 numpy scipy matplotlib
python -m centaur_control.run_comparison_suite --output comparison_baseline_v5
python -m centaur_control.evaluate_comparison_suite comparison_baseline_v5
```

第一条仿真命令默认无窗口，完整运行21个工况。输出 trace.csv、commands.jsonl、config.json 与 protocol.json；评价生成 scores.csv、evaluation.json、overview.png。仿真失败也写出状态并继续下一工况，不能删除失败样本再算通过率。文件夹非空时拒绝覆盖；中断后可追加 --resume，保留已有已记录工况（包括失败），只运行尚未记录的工况。若进程强制关闭留下无config的半成品，保留该目录并改用新输出目录重跑。

查看单个工况：

```powershell
python -m centaur_control.run_comparison_suite --case move_120N_yp --view --speed 3 --output preview_v5
```

move/posture仍为54仿真秒，upper为72秒；三倍速在性能允许时观看约18/24秒。100 Hz减少物理步数，不截断动作。

## 冻结的21个工况

| 类别 | 数量 | 内容 |
|---|---:|---|
| 无扰动基础任务 | 3 | move、posture、upper各一组 |
| 移动中的推力 | 8 | 40/120 N × 世界坐标±X/±Y |
| 姿态调节中的推力 | 8 | 40/120 N × 世界坐标±X/±Y |
| 上部非对称运动 | 2 | 左臂单独、右臂单独动作，另一臂保持初始姿态 |

移动仍执行前进、后退、左右转弯；姿态仍执行5°四方向倾斜及回正；上部仍执行±2 cm升降、双肩前伸/外展/回收。前4秒初始化不计入常规误差统计，初始化中出现异常仍记录失败。推力在第10秒开始，0.5秒后撤除，作用在 base_link 刚体质心，冲量分别20/60 N·s。方向固定于世界系，不随车体转向。上部两组没有额外外力或手持负载。

protocol.json保存完整规则。本版阈值为工程比较约定，不是来自安全标准；在本轮运行前固定，不按结果放宽。需要更改阈值、动作、摩擦、质量、控制频率或外力位置时，建立新协议版本并让所有算法重跑。

## 评分规则

全部适用门槛通过，才记为该工况通过。所有21个工况均在分母；未完成、数值异常、非车轮触地都不能记为通过。

| 指标 | 本版门槛 |
|---|---:|
| 高度RMSE | ≤10 mm |
| roll/pitch各自RMSE | ≤1° |
| 前向速度RMSE | ≤0.05 m/s |
| 偏航角速度RMSE | ≤0.03 rad/s |
| 被指令肩关节综合RMSE | ≤0.1 rad |
| roll/pitch最大绝对跟踪误差 | ≤5° |
| 任一轮载低于1 N的最长连续时段 | ≤0.1 s |
| 任一受限执行器达到99%力限幅的最长连续时段 | ≤0.2 s |
| 非车轮触地、MuJoCo警告 | 均为0 |
| 腿关节越限容差 | 最小限位余量≥−0.01 rad |
| 推力撤除后的配对恢复时间 | ≤3 s，连续满足恢复范围1 s |

恢复范围是相对于同一算法自身无扰动轨迹的差：roll/pitch各≤0.5°、高度≤5 mm、前向速度≤0.02 m/s、偏航角速度≤0.02 rad/s。另列加上XY位置差≤2 cm的恢复时间，暂不纳入通过门槛。若扰动期间始终没离开恢复范围，恢复时间可为0，须结合 left_recovery_band 解读，不能解释为瞬时完全恢复。

主要排名先比较通过工况数，再在双方都完成的同一工况上看绝对参考跟踪误差、轮载、恢复时间、滑移和机械功。失败行没有完整误差值，不能用0替代，也不能只汇总成功样本掩盖失败率。不要把21个确定性工况的通过比例解释为随机环境中的可靠性概率。

absolute_actuator_work_J = ∫Σ|执行器力×执行器速度|dt，是绝对机械功指标，不是电池耗电量；没有电机效率与回馈模型。滑移只对有效接触样本统计；脱离接触本身由轮载指标体现。wall_seconds受机器负载影响，不能用本次并行运行耗时证明实时控制性能。

## 如何接入后续WBC与强化学习算法

保持相同模型及执行器限制、初始状态、任务参考、外力、观察频率、控制频率、时长和评分器。只替换控制律；现有的姿态/位置辅助反馈属于基线算法，若新算法不使用，需在方法说明中明确。禁止将未来参考或预告外力作为新算法独享输入。

当前运行器的基线控制律位于 run_dynamic_task.py，包含IK、竖直载荷分配、轮速反馈和上部补偿；它不是完整动力学WBC。未来替换时保留任务与记录接口，记录真实算法名称、代码哈希及训练检查点。上部控制也要替换时，避免让 FoundationTask.apply 的肩关节覆盖逻辑覆盖新算法的输出；相应适配需验证后另行实现，本包不声称已有RL/WBC控制器接口。

两套完整成绩可用统一脚本并排比较：

```powershell
python -m centaur_control.compare_algorithms comparison_baseline_v5 comparison_candidate --output algorithm_comparison.csv
```

脚本检查协议、模型/网格、任务生成器、仿真版本及频率一致，允许控制律代码不同，并重新从原始记录评价。当前工具面向确定性固定工况；未来随机策略评估需固定并公布评估种子集合，各种子都跑同一21工况，汇总均值、离散度与失败数。训练种子和评估种子分别记录。

## 数值与复现检查

默认仍是100 Hz。--physics-hz 200或500只用于显式数值敏感性检查，控制指令仍每0.01秒更新并在子步间保持，结果另存目录，不能混入100 Hz排行榜。本轮复查的实际工况及差异见验证报告；单个工况的频率复查不证明所有工况数值收敛。

模型XML和全部引用网格、任务与控制代码均保存SHA-256；评价拒绝同一套成绩混用不同代码/物理配置。复跑检查采用相同确定性初态，没有故意加入随机噪声；因此复跑一致不等于参数鲁棒性已验证。

```powershell
python -m unittest centaur_control.tests.test_foundation_tasks centaur_control.tests.test_dynamic_benchmark centaur_control.tests.test_foundation_benchmark centaur_control.tests.test_comparison_suite
```

尚未覆盖斜坡、门槛、人员碰撞、手持负载、传感噪声、摩擦/质量随机化和真实硬件。本版可作为后续算法的第一层平地基础对照，不应宣称是家庭服务全场景或倾覆极限基准。

## 本轮结果与使用边界

本次默认100 Hz成绩为19/21。move_120N_xm与posture_120N_xm未通过，保留在完整排行榜中。两项均有俯仰跟踪误差超标、连续低轮载和恢复超时。程序完成运行不等于工况通过；本轮没有把失败工况删掉，也没有为了通过而调整门槛。

姿态−X强推力工况在更小物理步长下仍出现轮载归零，但100/200/500 Hz瞬态曲线并不一致。因而本版100 Hz成绩可用于相同离散仿真条件下的开发对比，不能把失稳附近的峰值或恢复时刻当成真实物理极限。后续算法若主要改善这两个挑战工况，必须同时做更小步长复查后再下物理性能结论。详细数值见验证报告。

附带verification/baseline_v5为完整成绩，可直接重新评价，无需重跑物理仿真：

```powershell
python -m centaur_control.evaluate_comparison_suite verification/baseline_v5
python -m centaur_control.plot_benchmark_diagnostics verification/baseline_v5 --fine verification/v5_sensitivity_200 verification/v5_sensitivity_500
```

报告内包含一次强推力重复运行核对、三项旧版无扰动轨迹回归核对及数值敏感性检查。两次姿态+X记录完整性问题和一次内存中断的原始文件保存在audit目录，最终排行榜只使用同参数完整重跑数据。没有将这些计算/文件问题解释为控制器物理失败。
