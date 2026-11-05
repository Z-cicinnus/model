# AS2-W 与 R1-A7 统一全身控制

## 项目结构

```text
centaur_control/
├─ models/          MuJoCo统一模型构建、名称定义、初始状态和碰撞规则
├─ kinematics/      正运动学、逆运动学、雅可比与速度QP
├─ dynamics/        刚体动力学、质量属性和静态支撑分析
├─ controllers/     可复用PD目标与支撑姿态控制
├─ workspace/       四轮固定支撑下的几何与静力工作空间
├─ tasks/           双臂、整机分析、倾角遍历和冲击实验
├─ visualization/   MuJoCo结果回放
├─ tests/           模型与数值检查
├─ analysis_results/报告、机器可读数据、图像、日志和冲击轨迹
├─ centaur_unified.xml
└─ scene.xml
```

算法放在所属模块中，独立实验放在 `tasks/`。`centaur_control/` 根目录的旧 `.py` 脚本保留为兼容入口；新增功能请直接放入模块包。新增一次性任务时，在 `tasks/` 新建模块并提供 `main()`，随后用 `python -m centaur_control.tasks.<模块名>` 从项目根目录运行。

源资产保持在 `as2w_model/` 和 `r1_a7_o6_model/`，构建器依赖这些相对路径。统一模型采用 R1 pelvis 相对 AS2-W `base_link` 平移 `[-0.015, 0, 0.1] m`、四元数 `[1, 0, 0, 0]`（wxyz）。模型质量约 `43.266282 kg`，有 `nv=61` 个广义速度自由度和 `nu=45` 个位置伺服执行器。

## 环境与数值验收

以下命令在 Windows PowerShell 中运行。把 `$python` 改为已安装 MuJoCo Python 包的解释器路径；所有命令都从项目根目录执行。

```powershell
Set-Location -LiteralPath 'C:\Users\16389\Desktop\Wheel_Centaur'
$env:PYTHONIOENCODING = 'utf-8'
$python = 'D:\Users\16389\anaconda3\envs\mujoco\python.exe'

& $python -m centaur_control.models.build
if ($LASTEXITCODE -ne 0) { throw '模型构建失败' }
& $python -m centaur_control.tests.validate_model
if ($LASTEXITCODE -ne 0) { throw '模型检查失败' }
& $python -m centaur_control.tests.test_analysis
if ($LASTEXITCODE -ne 0) { throw '数值回归失败' }
```

完整数值回归覆盖末端及轮地雅可比有限差分、质量矩阵正定性、动力学方程残差、逆运动学、固定轮接触几何和冲击基线。当前最近一次检查通过：末端雅可比误差 `3.22e-7`，轮地雅可比误差 `3.30e-7`，动力学残差 `1.07e-7`，逆解位置误差 `0.16 mm`。

## MuJoCo 可视化验收

先检查统一模型和场景能正常加载：

```powershell
& $python -m mujoco.viewer --mjcf centaur_control/scene.xml
```

模型静态检查通过后，运行双臂PD与IK-QP闭环测试；此命令会在 MuJoCo 窗口中显示机器人静止保持和双臂运动过程：

```powershell
& $python -m centaur_control.tasks.bilateral --viewer
```

检查双手是否伸展/回收、四轮是否接地以及机身姿态是否变化。关闭窗口后，终端会输出本次测试结果；提前关闭会中止测试。

已生成的工作空间和倾角姿态可分别回放（每次关闭一个窗口后再运行下一条）：

```powershell
& $python -m centaur_control.visualization.viewer --mode ik
& $python -m centaur_control.visualization.viewer --mode posture
& $python -m centaur_control.visualization.viewer --mode workspace
& $python -m centaur_control.visualization.viewer --mode height_sweep
& $python -m centaur_control.visualization.viewer --mode height_optimum
```

冲击轨迹回放示例：

```powershell
& $python -m centaur_control.visualization.viewer --mode impact --case active_chest_forward_100ms_upper --speed 0.25
```

回放命令读取 `analysis_results/` 中已有结果，不会重跑扫描。空格暂停/继续，`Home` 重播（英文输入法下也可按 `R`），鼠标旋转视角、滚轮缩放。画面中的 `PLAYING / PAUSED` 表示播放/暂停状态。除冲击轨迹外，回放中的构型切换是几何展示，不代表控制器已验证可动态执行。

2026-11-05 可视化检查：六种回放模式均完成短时启动、绘制循环和定时关闭，退出码均为0；最优倾角窗口已实际查看并验证暂停及手动关闭。中文输入法会拦截字母R，因此新增Home重播键；暂停/恢复、Home和R回调已通过程序检查。Home的桌面按键效果尚待用户验收。此前退出码1未复现，不能认定已查明原因；隔离执行环境中窗口可能不会出现在用户桌面，请在本机PowerShell中运行上述命令。

## 任务运行

```powershell
# 重算正逆运动学、动力学、工作空间和五个高度的姿态边界
& $python -m centaur_control.tasks.analysis --samples 20000 --seed 20261104

# 遍历腿高，搜索各方向最大静态倾角
& $python -m centaur_control.tasks.height_scan --height-step-mm 1

# 重算自由滚动与主动抗扰冲击边界，耗时较长
& $python -m centaur_control.tasks.impacts
```

倾角遍历当前搜索高度约 `0.2031–0.4733 m`；全局最大样本为后倾约 `44.507°`，发生在 `base_link` 高度约 `0.30513 m`。这是当前站姿分支、固定轮接触位置及模型静力约束下的搜索结果，不是动态防倾覆保证，也不证明任意关节构型的全局最优。

## 结果索引与文件分类

| 类别 | 路径 | 内容 |
|---|---|---|
| 源输入 | 项目根目录 `as2w_model/`、`r1_a7_o6_model/` | 原始URDF、MJCF、网格和手部资产 |
| 可复用实现 | `models/`、`kinematics/`、`dynamics/`、`controllers/`、`workspace/` | 算法与模型组件 |
| 可运行实验 | `tasks/` | 双臂控制、整机分析、高度扫描、冲击扫描 |
| 可视化和测试 | `visualization/`、`tests/` | 结果回放、数值和模型验收 |
| 统一模型 | `centaur_unified.xml`、`scene.xml` | 构建输出和MuJoCo场景 |
| 实验产物 | `analysis_results/` | Markdown报告、JSON摘要、NPZ数据、CSV边界、PNG图和过程日志 |

主要报告：

- `analysis_results/analysis_report.md`：正逆运动学、动力学、手部与腿部工作空间、五高度倾斜边界。
- `analysis_results/height_tilt_scan_report.md`：全高度倾角搜索和最优高度。
- `analysis_results/impacts/impact_report.md`：自由滚动与主动抗扰的冲击对照。
- `model_validation.md`：统一模型的阶段验证记录。
- 项目根目录 `项目总览与文件分类建议.md`：任务总结、限制和后续目录讨论。

当前 `analysis_results/` 路径被分析、回放和验收代码共同使用，因此暂时保持原位。后续若确认需要整理产物，可迁移为 `reports/`、`data/`、`figures/`、`logs/` 和 `impacts/`，并同步更新 `paths.py`、读取代码和文档；在迁移前不删除扫描检查点、日志或预览图。`__pycache__/` 是可再生成缓存，不属于源代码。

## 结果边界

固定支撑工作空间是有限采样的近似；静态倾角由重力、关节/执行器限制、当前碰撞规则和轮端法向力约束决定。冲击结果使用当前模型及控制增益，地面接触参数、网格碰撞简化和质量惯量尚未用实物标定。尚未实现动态行驶移动操作基准任务、全身逆动力学平衡控制、MPC、PPO或融合强化学习算法。
