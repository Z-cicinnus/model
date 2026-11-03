# 模型文件核查与已完成验证（2026-11-03）

本文依据项目已有文件包中的 README、模型及验证结果整理。**来源项目的历史验证不等于当前四轮足半人马整机的验证**。

## 已核对的来源文件

| 文件包 | 实际内容 | 验证边界 |
| --- | --- | --- |
| `AS2W_control_project.zip` | 原始 XML、URDF、36 个编号 STL、简化碰撞模型与控制原型 | AS2-W 单独底盘代理模型 |
| `AS2W_kinematics_checked.zip` | 18 个具名 STL、运动学检查代码及验证报告 | 几何适配/运动学，不是稳定运动 |
| `R1_A7_MuJoCo.zip` | 19 个 STL、URDF、17 轴 MJCF、校验报告 | 独立固定基上半身 |
| `models/centaur/centaur.xml` | 整机 XML，1 个自由基座、33 个执行器和 55 个网格引用 | 整机尚待实际加载与接触校核 |

**不能将 AS2-W 的 36 个编号网格与另一版 18 个具名网格直接混用。** 二者源模型的网格坐标/缩放处理方式不同。

## 文件记录中的主要参数

**AS2-W：** 原始模型总质量 26 kg，16 个执行器；另一版运动学核对记录 wheel mesh 半径约 0.086 m、轮胎中面偏置 0.05461429 m。100 组随机配置、400 次轮心位置对照中的最大正运动学位置差约 2.83×10⁻¹⁶ m。其参考姿态存在 4 个前腿与机身的穿透接触，碰撞问题未关闭。

**R1-A7：** 19 个刚体、17 个运动关节（腰 1、左臂 7、右臂 7、头部 2），URDF 源总质量 16.19940901 kg。归档报告中固定基 MuJoCo 3.17.0 验证通过，101 个测试姿态的最大位置误差约 3.89×10⁻¹⁶ m，完整惯量对照差最大约 5.55×10⁻¹⁷ kg·m²。原模型**没有独立夹爪驱动关节**，机器人内部自碰撞被关闭，实机阻尼/驱动及相机惯量仍需复核。

**AS2-W 简化碰撞控制原型：** 旧 ZIP 的 validation.json 记录 11 项预设场景通过；物理周期 0.001 s、控制周期 0.002 s，使用理想 MuJoCo 状态与接触力。源记录中，直行（摩擦系数 0.8）线速度 RMSE 为 0.00348 m/s；原地转向（摩擦系数 0.8）角速度 RMSE 为 0.03635 rad/s；转弯（摩擦系数 0.8）角速度 RMSE 为 0.02989 rad/s。5 kg 已知居中载荷直行的线速度 RMSE 为 0.00400 m/s。这些**不是整机控制实验结果**，也不是硬件控制验证。

## 本轮新增工具

可对本地原始文件包作结构与 SHA256 核对（严格哈希对应**原压缩包字节**，重新打包可能改变哈希）：

```bash
python scripts/check_source_packages.py --as2w-zip /path/to/AS2W_control_project.zip --r1a7-zip /path/to/R1_A7_MuJoCo.zip --require-exact-hash
```

恢复 STL 资源，完成模型编译检查后，在同一个整机模型上计算质量矩阵及各关节链的整体雅可比：

```bash
python scripts/import_assets.py --as2w-zip /path/to/AS2W_control_project.zip --r1a7-zip /path/to/R1_A7_MuJoCo.zip
python scripts/check_repo.py --strict --compile
python scripts/whole_body_diagnostics.py --model models/centaur/centaur.xml --full-matrices --output experiments/results/whole_body_diagnostics.json
```

输出包含整机广义质量矩阵、左右腕部及四个轮体坐标原点雅可比、**实际检测到的轮地接触点**雅可比。腕部刚体原点不是夹爪 TCP；没有接触时不应虚构轮地接触雅可比。该工具仅用于诊断，**不等于完成 QP/WBC/MPC/RL 控制算法**。

## 仍需验证

必须继续检查真实模型加载、安装位姿、质量惯量、执行器限幅、轮地接触、自碰撞与末端 TCP；再逐步验证 PD、全身逆运动学 QP、WBC/MPC、统一 PPO，以及固定场景的直行、转向、双臂协同与搬运任务。

由于当前仓库公开，缺少明确的再分发许可的第三方 STL 与源 URDF 不直接公开上传，使用者应从自己已有的来源包在本地恢复。
