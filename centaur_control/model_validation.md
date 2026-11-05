# 统一模型与全身控制阶段验证

## 复现命令

在仓库根目录的 PowerShell 中运行：

```powershell
Set-Location -LiteralPath 'C:\Users\16389\Desktop\Wheel_Centaur'
$env:PYTHONIOENCODING = 'utf-8'
$python = 'D:\Users\16389\anaconda3\envs\mujoco\python.exe'
& $python centaur_control/build_model.py
if ($LASTEXITCODE -ne 0) { throw '模型生成失败' }
& $python centaur_control/validate_model.py
if ($LASTEXITCODE -ne 0) { throw '模型验证失败' }
& $python centaur_control/run_bilateral_test.py
if ($LASTEXITCODE -ne 0) { throw '双臂测试失败' }
```

控制过程的可视化检查：

```powershell
Set-Location -LiteralPath 'C:\Users\16389\Desktop\Wheel_Centaur'
$env:PYTHONIOENCODING = 'utf-8'
& 'D:\Users\16389\anaconda3\envs\mujoco\python.exe' centaur_control/run_bilateral_test.py --viewer
```

窗口显示静止 PD 基线与双臂运动过程；两个测试使用各自的相同初始状态。完成后保留最终画面，关闭窗口退出。单独加载 XML 的通用查看器不会运行此控制回路。

## 模型与 PD 检查

| 检查项 | 结果 |
|---|---:|
| MuJoCo 版本 | 3.7.0 |
| 刚体数 / 关节数 | 61 / 56 |
| nq / nv / nu | 62 / 61 / 45 |
| 总质量 | 43.2663 kg |
| 验证姿态质心 | (0.0387513, -0.0001660, 0.5244617) m |
| 质量矩阵 | 61 x 61，有限且正定 |
| 左右手雅可比 | 各 6 x 61 |
| 四轮接触雅可比 | 12 x 61 |
| PD 控制目标 | 覆盖全部 45 个位置伺服执行器 |
| 碰撞排除 | 底盘与前大腿及左右肩部装配网格 |

轮轴与腿部采用仿真调试增益，以确保静止支撑；这不是硬件标定。底盘/肢体碰撞排除针对源网格在装配姿态下的重叠，后续仍需结合 CAD 与真实碰撞体复核。

## 双臂测试结果

结果文件：`bilateral_test_results.json`。当前 MuJoCo 3.7.0 的结果如下：

| 指标 | 结果 |
|---|---:|
| 静止 PD 基线时长 | 2 s |
| 静止最大机身倾角 | 0.01448 rad |
| 静止最大水平漂移 | 0.03749 m |
| 双臂任务时长 / 步长 | 4 s / 2 ms |
| 双臂运动最大机身倾角 | 0.02304 rad |
| 双臂运动期间保持接触的轮数 | 4 / 4 |
| 左右手位置 RMSE | 0.03647 m |
| 最终机身位置 | (-0.03385, -0.00001, 0.43926) m |

双臂任务使用对称往返目标，位置幅值为前向 45 mm、左右各 20 mm、竖直 20 mm。任务期间只有双臂关节参与 IK-QP，腿、轮轴、躯干、头部和手指由 PD 保持各自目标；腿部 PD 目标根据机身 roll/pitch 误差进行补偿。机身是浮动基，纯运动学 QP 不能直接指令其姿态，本阶段也未实现全身逆动力学平衡控制。

本阶段未验证移动中的双手跟踪、静态力/接触平衡、物体抓取、碰撞安全或硬件部署。全身动态行驶与移动操作基准属于后续工作。
