# 四轮足半人马：WBC 接入与固定基准对比

本包在 foundation-v5.0 的相同模型、初态、21 项任务、扰动与评分规则下，增加独立的全身逆动力学 QP 控制器。默认物理与控制频率均为 100 Hz；移动/姿态任务仍为 54 仿真秒，上部任务 72 秒。结果和不足见 `WBC_验证报告.md`，不能把程序运行完成等同于通过评分。

## 安装和运行

这是对已有 Wheel_Centaur + v5 基准的增量包。备份同名文件，将本包合并至 Wheel_Centaur 根目录，沿用原有 as2w_model 和 r1_a7_o6_model 网格目录。无需改变机器人 XML 或替换网格。

```powershell
python -m pip install -r requirements_wbc.txt
python -m centaur_control.run_wbc_suite --output comparison_wbc_v1
python -m centaur_control.evaluate_comparison_suite comparison_wbc_v1
python -m centaur_control.compare_algorithms verification/baseline_v5 comparison_wbc_v1 --output comparison_wbc.csv
```

上面的对照路径 `verification/baseline_v5` 来自原 v5 包；也可改为自己运行的完整基础控制器输出目录。本包不重复附带原 v5 的全部基础控制器轨迹。已有基础控制器命令继续有效：

```powershell
python -m centaur_control.run_comparison_suite --output comparison_baseline
```

查看单个 WBC 工况：

```powershell
python -m centaur_control.run_wbc_suite --case move_nominal --view --speed 3 --output preview_wbc_move
python -m centaur_control.run_wbc_suite --case posture_120N_xm --view --speed 3 --output preview_wbc_push
```

`--speed` 只改变观看倍速，不改变物理步长、任务持续时间或评分。首次选择全新输出目录；已有完整记录时用 `--resume` 保留已记录工况，包括失败工况。强制关闭留下不完整记录时保留旧目录，并在新目录运行，不手动删除失败样本来提高成绩。`--task move/posture/upper` 可用于开发子集，正式对比必须尝试全部 21 项。

数值复查显式使用 `--physics-hz 500`，控制频率仍为 100 Hz；须使用不同输出目录，不能与默认成绩混用。

## 实现范围

`controllers/wbc.py` 使用统一整机的 61 维广义速度模型，联合求解 61 维加速度、四轮三维接触力及 10 项手指联动反力，共 83 个优化变量。腰部、双臂、手指、腿部和轮子均在同一动力学模型中。45 个执行器的最终指令均由 QP 输出；没有基线控制输出混合，也没有求解失败后偷偷回退到基础控制器。

设优化变量为 x=(a,f,λ)，采用加权最小二乘目标，包含基座高度/姿态/速度、腿部支撑构型、上部关节参考、轮速、软滚动接触和关节限位恢复。采用加权任务，不是严格分层 HQP。上部任务跟踪的是现有基准的肩关节参考，尚未实现新的双手笛卡尔空间任务。

动力学采用一个控制周期的阻尼近似：

`M_eff a + h - q_passive = Sᵀ τ + Jᵀ f + Eᵀ λ`

其中 `M_eff = M + 0.01 × (D_joint + D_servo)`，用于考虑原隐式积分器及伺服速度阻尼；这属于离散近似，不是连续动力学的精确等式。浮动基及被动关节动力学、线性手指联动关系作为等式约束。关节力矩由动力学方程消去后恢复。求解器为 Clarabel；每步记录状态、迭代次数、约束残差与计算耗时。

原位置伺服器的增益、阻尼、控制范围和物理力矩上限保持不变。WBC 将目标力矩反解为原伺服输入；四轮仍沿用 v5 的力矩驱动模式。有效力矩范围同时考虑启用的 actuator_forcerange 和 joint actuatorfrcrange，未启用的默认零范围不能当成零力矩限制。关节限位通过预测越界的恢复目标软处理，不声称硬保证；实际腿部越限由原评分器检查。

平地接触采用轮心正下方近似接触点及完整关节雅可比，包含轮子自转。滚动关系是接触点速度约束，允许轮心移动；侧向约束使用较低权重，允许滑移转向。法向力非负，切向力位于保守摩擦棱锥内；估计离地轮的接触力置零。滚动/法向加速度是软目标，接触点几何及其变化也作了近似，不能把这个平地实现直接推广到轮胎变形、大侧倾或复杂地形。

控制器使用当前状态和当前参考，不读取将来的指令或测试外力。扰动仍由原任务对象按世界系在同一时刻施加。姿态/上部任务沿用原任务生成器提供的位置和偏航辅助反馈。接入标记 `owns_control` 使共享运行器跳过基础控制律；基础模式的执行路径与 v5 保持一致。

## 公平对比与记录

原 `run_foundation_tasks.py`、`run_comparison_suite.py`、`benchmark_foundation.py`、模型、网格、协议和评价器未修改。控制入口、WBC 控制代码及相关模型初始化源文件保存 SHA-256，记录 MuJoCo、Clarabel、NumPy、SciPy 版本。开发只使用无扰动任务调整接口和参数；完整扰动评测期间参数固定。此固定基准已经用于开发，因此成绩属于开发基准成绩，不是独立泛化测试。

每个工况保留 `trace.csv`、`commands.jsonl`、`config.json` 和 `wbc.jsonl`。QP 失败会记录异常并终止该工况，继续其余工况，失败仍计入 21 项分母。QP 残差包含不同物理量，仅用于程序接受检查，不是具有统一物理单位的误差指标。

原评分器的饱和指标只覆盖 actuator_forcelimited 的执行器，不能完整反映上部关节级力矩限制；为保持规则一致，本轮不改评分器。WBC 另在 `wbc.jsonl` 的 `peak_force_util` 记录包含关节级限制的利用率。该附加量是控制步目标力矩利用率，不能替代原评分。

`seconds` 包括单次 WBC 构建与求解开销，不能单独证明实时性；整轮 wall_seconds 还包含物理仿真、记录和其他控制接口开销。本轮有并发仿真，耗时仅作开发诊断。100 Hz 指更新周期，不等于已证明每次计算都能在 10 ms 内完成。

## 检查

```powershell
python -m unittest centaur_control.tests.test_foundation_tasks centaur_control.tests.test_dynamic_benchmark centaur_control.tests.test_foundation_benchmark centaur_control.tests.test_comparison_suite centaur_control.tests.test_wbc
```

本包还补回 v5 测试所依赖、但 v5 增量包漏带的 `benchmark_dynamic.py`，原样取自 v2。它不参与本轮冻结评分。程序测试中的人工失败样本输出不是仿真成绩。

本轮未验证 Windows 图形窗口、实机、抓取负载、门槛/楼梯、地形变化或传感噪声。最终方法是独立 WBC；早期调试中尝试过力矩正则化和直接力矩模式，均未用于最终成绩。
