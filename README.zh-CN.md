# 四轮足半人马机器人 | Centaur Robot

> AS2-W 四轮足底盘 + R1-A7 人形上部机构的统一 MuJoCo 建模与全身控制研究项目。

[English README](README.md) · [整机模型](models/centaur/centaur.xml) · [验证计划](docs/validation_plan.md)

## 目录导航

- [项目简介](#项目简介)
- [当前进度](#当前进度)
- [仓库结构](#仓库结构)
- [环境安装与运行](#环境安装与运行)
- [整机控制研究路线](#整机控制研究路线)
- [跳跃控制研究（规划）](#跳跃控制研究规划)
- [测试与验证](#测试与验证)
- [模型来源与注意事项](#模型来源与注意事项)

## 项目简介

本项目以 **AS2-W 四轮足底盘**与 **R1-A7 人形上部机构**为基础，建立具有浮动基座、轮腿、躯干和双臂的统一多体动力学模型，用于研究家庭环境中的稳定移动、转向、姿态调节与双臂移动操作。

**建模原则：** 把四轮足与双臂视为一个整体；控制器可按任务层级组织，但不把底盘和双臂视为两个互不关联的动力学系统。

## 当前进度

| 模块 | 状态 | 说明 |
| --- | --- | --- |
| AS2-W MuJoCo XML | ✅ 已入库 | `models/as2_w/as2_w.xml` |
| R1-A7 MuJoCo XML | ✅ 已入库 | `models/r1_a7/r1_a7.xml` |
| 半人马整机 XML | ✅ 已入库 | `models/centaur/centaur.xml` |
| 模型网格与原始 URDF | ⚠️ 尚未公开上传 | 需从原始 ZIP 本地导入 |
| PD 控制基础代码 | 🧪 初始实现 | 尚未完成整机动态验证 |
| 全身逆运动学 QP、WBC、MPC、PPO | 📋 规划中 | 不代表已实现 |
| 移动、转向、跳跃与移动操作实测 | ⏳ 待验证 | 当前没有可复现的整机性能结论 |

整机 XML 含 **1 个浮动基座、33 个执行器**，引用 **55 个网格资源**（AS2-W：36 个；R1-A7：19 个）。模型描述已归档，**缺少原始 STL 时无法完成整机 MJCF 编译**；XML 文件存在并不等于仿真已经通过。

## 仓库结构

```text
model/
├── models/
│   ├── as2_w/                  # 底盘模型及本地网格
│   ├── r1_a7/                  # 上部机构模型及本地网格
│   └── centaur/                # 整机模型 centaur.xml
├── src/centaur_sim/
│   ├── controllers/            # PD 基础控制代码
│   ├── interfaces.py           # 全身控制接口
│   └── model.py                # MuJoCo 模型加载
├── scripts/                    # 资源导入、检查与仿真
├── configs/                    # 控制及实验参数
├── scenes/                     # 后续实验场景
├── tests/                      # XML 结构和运行时测试
├── experiments/results/        # 本地实验输出（默认忽略）
├── docs/                       # 模型来源和验证路线
└── .github/workflows/          # GitHub Actions 检查
```

## 环境安装与运行

建议使用 Python 3.10+。

```bash
git clone https://github.com/Z-cicinnus/model.git
cd model
python -m pip install -e ".[dev]"
```

将自己保存的 `AS2W_control_project.zip` 和 `R1_A7_MuJoCo.zip` 作为输入，本地恢复模型资源：

```bash
python scripts/import_assets.py \
  --as2w-zip /path/to/AS2W_control_project.zip \
  --r1a7-zip /path/to/R1_A7_MuJoCo.zip
```

验证模型资源并查看关节、执行器信息：

```bash
python scripts/check_repo.py --strict
python scripts/check_repo.py --strict --compile
python scripts/inspect_model.py --model models/centaur/centaur.xml
python -m pytest -q
```

运行零控制输入的短时仿真（**仅用于数值运行检查，不代表站立或运动控制成功**）：

```bash
python scripts/run_simulation.py --model models/centaur/centaur.xml --steps 500
```

若未导入网格，部分不依赖真实网格的结构测试仍可能通过，不能据此判断整机模型可正常运行。

## 整机控制研究路线

1. **阶段 0：整机模型校核** —— 关节链、质量惯量、安装位置、执行器映射、轮地接触。
2. **阶段 1：经典控制** —— 关节 PD、全身正逆运动学、全身 IK-QP。
3. **阶段 2：优化控制** —— 全身逆动力学 QP（WBC）、模型预测控制（MPC）。
4. **阶段 3：统一强化学习** —— PPO 全身策略，同时协调轮腿与双臂。
5. **阶段 4：融合控制与任务验证** —— 历史观测、负载自适应、RL + WBC/MPC 约束控制。

重点任务是稳定直行、原地转向、双臂动作时的姿态维持、搬运过程中运动与操作的统一协调。各阶段使用一致的模型、任务、随机种子和评价指标。

## 跳跃控制研究（规划）

**跳跃控制是后续探索方向，不是仓库目前已实现的功能。** 四轮足平台是否有足够的关节功率、行程和有效接触结构实现离地跳跃，需要先基于真实电机参数及模型校核确定。

在可行性满足的前提下，拟按以下顺序开展：

1. **动力学可行性：** 校核整机总质量、关节峰值力矩/功率、腿部工作空间、起跳冲量与落地冲击。
2. **接触状态：** 分析预压缩、起跳、腾空、落地阶段的轮地接触、轮端滚动约束和支撑切换。
3. **基线方法：** 在 MuJoCo 中构建受限高度的起跳—落地任务，先验证轨迹优化、QP/WBC 或 MPC 基线。
4. **学习控制：** 仅在基线和安全约束可靠后训练全身强化学习策略，分析双臂姿态对整机起跳、空中姿态和落地稳定性的影响。
5. **评价指标：** 起跳高度、落地姿态偏差、接触冲击、滑移、关节峰值力矩、成功率及能耗。

跳跃相关实验应和普通移动控制区分，**不能仅凭仿真接触求解成功就认定实体机器人具备跳跃能力**。

## 测试与验证

当前 CI 主要做 Python/模型 XML 结构检查；运行时模型测试在网格缺失时会跳过。完整整机验证仍需要恢复全部模型资源，检查碰撞体与真实执行器设置后开展。

后续固定测试包含：双臂静态动作、匀速直行、原地转向、转向时双臂协同、双臂搬运及障碍/门槛通行。跳跃测试需独立评估风险及可行性。

详细计划见 [docs/validation_plan.md](docs/validation_plan.md)。

## 模型来源与注意事项

- AS2-W 来源于此前项目的 XML、URDF 和 STL 文件；R1-A7 来源于此前整理的 MuJoCo 资源包。
- `centaur.xml` 是已有整机模型的英文文件名版本；必须在恢复网格后重新验证质量惯量、安装姿态与轮地接触。
- 因仓库为公开仓库，STL 和源 URDF 暂未公开分发；上传前须确认相应许可。
- 研究阶段的控制增益与算法结构不等于经过实体机器人验证的安全参数。

[返回顶部](#四轮足半人马机器人--centaur-robot)
