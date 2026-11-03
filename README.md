# 四轮足半人马机器人 | Centaur Robot

**AS2-W 四轮足底盘 + R1-A7 人形上部机构的统一 MuJoCo 建模与全身控制研究**

[English README](README.en.md) · [整机模型](models/centaur/centaur.xml) · [模型来源](docs/asset_provenance.md) · [验证计划](docs/validation_plan.md)

## 1. 项目简介

本项目面向家庭服务场景，将 AS2-W 四轮足底盘与 R1-A7 人形上部机构集成为**同一个浮动基多体动力学系统**，研究整机的稳定移动、转向、姿态控制及双臂协同移动操作。

**核心原则：** 底盘、躯干和双臂统一建模、统一评价。可按照规划、感知和控制频率分层组织算法，但不将底盘控制和双臂控制视为互不关联的两个动力学系统。后续不同控制算法应尽量共享同一整机模型、实验任务及评价指标。

## 2. 当前工程状态

| 模块 | 状态 | 说明 |
| --- | --- | --- |
| AS2-W 模型 | 已入库 | `models/as2_w/as2_w.xml` |
| R1-A7 模型 | 已入库 | `models/r1_a7/r1_a7.xml` |
| 四轮足半人马整机模型 | 已入库、待复核 | `models/centaur/centaur.xml` |
| STL 网格及原始 URDF | 尚未公开入库 | 通过原始工程 ZIP 在本地恢复 |
| 模型检查与仿真入口 | 初步提供 | `scripts/` |
| 关节 PD 基线及统一控制接口 | 初步实现 | `src/centaur_sim/` |
| 全身逆运动学 QP、WBC、MPC、PPO | 计划实现 | 尚无可复现的整机控制结论 |
| 稳定直行、原地转向、双臂操作实验 | 待验证 | 需先完成网格恢复、模型编译和参数校核 |

当前整机 XML 描述包括 **1 个浮动基座、33 个执行器**，并引用 **55 个 STL 网格**（AS2-W：36 个；R1-A7：19 个）。这些是模型描述中的数量，不等于已完成动力学或硬件验证。

> **注意：** 目前公开仓库没有原始 STL/URDF 文件。仅克隆仓库无法直接运行完整的 MuJoCo 整机模型。模型结构测试通过，也不意味着机器人已实现稳定移动、转向或抓取。

## 3. 仓库目录

```text
model/
├── README.md                     # 中文项目说明（默认首页）
├── README.zh-CN.md               # 中文说明副本
├── README.en.md                  # English README
├── pyproject.toml                # Python 工程依赖
├── models/
│   ├── as2_w/
│   │   ├── as2_w.xml             # AS2-W MuJoCo 模型
│   │   └── meshes/               # 本地恢复 36 个 STL，不上传
│   ├── r1_a7/
│   │   ├── r1_a7.xml             # R1-A7 MuJoCo 模型
│   │   └── meshes/               # 本地恢复 19 个 STL，不上传
│   └── centaur/
│       └── centaur.xml           # 统一整机模型入口
├── src/centaur_sim/
│   ├── interfaces.py             # 统一整机任务命令接口
│   ├── model.py                  # MuJoCo 模型加载
│   └── controllers/
│       └── pd.py                 # 直接力矩电机关节 PD 基线
├── scripts/
│   ├── import_assets.py          # 导入本地模型资源
│   ├── check_repo.py             # 检查 XML 和网格引用
│   ├── inspect_model.py          # 查看自由度和执行器
│   └── run_simulation.py         # 零控制输入仿真检查
├── configs/                      # 机器人及控制器参数
├── scenes/                       # 后续实验场景
├── tests/                        # 结构及可选运行时测试
├── experiments/results/          # 实验输出（默认忽略提交）
├── docs/                         # 模型来源、命名和验证方案
└── .github/workflows/            # GitHub Actions 自动检查
```

新文件统一采用小写英文、下划线命名；原模型关节名与 STL 文件名在引用关系完成核对前不随意更改。

## 4. 安装与运行

建议使用 Python 3.10 或更高版本。首先克隆仓库并安装开发依赖：

```bash
git clone https://github.com/Z-cicinnus/model.git
cd model
python -m pip install -e ".[dev]"
```

将之前保存的 `AS2W_control_project.zip` 与 `R1_A7_MuJoCo.zip` 放到本地可访问的位置，运行资源导入脚本（将示例路径替换为实际路径）：

```bash
python scripts/import_assets.py --as2w-zip /path/to/AS2W_control_project.zip --r1a7-zip /path/to/R1_A7_MuJoCo.zip
```

脚本保留 STL 原始文件名与文件内容，并导入对应的 URDF。随后依次执行：

```bash
# 检查必需资源；有缺失时返回非零退出码
python scripts/check_repo.py --strict

# 尝试编译三个 MuJoCo 模型
python scripts/check_repo.py --strict --compile

# 查看整机自由度、执行器与驱动信息
python scripts/inspect_model.py --model models/centaur/centaur.xml

# 运行项目基础测试
python -m pytest -q
```

如模型能够正常编译，可进行短时零控制输入仿真，用于观察是否出现数值异常：

```bash
python scripts/run_simulation.py --model models/centaur/centaur.xml --steps 500
```

**零控制仿真不是站立控制，也不是稳定移动实验。** 若尚未恢复 STL，运行时模型测试可能跳过，请阅读实际测试输出。

## 5. 统一全身控制研究路线

| 阶段 | 研究内容 | 目标成果 |
| --- | --- | --- |
| **0：整机建模与校核** | 关节链、安装坐标、质量与惯量、执行器、轮地接触 | 可验证的完整 MuJoCo 模型 |
| **1：经典全身控制** | PD、整机正逆运动学、全身逆运动学 QP | 全关节控制基线及双臂—车身协调 |
| **2：优化控制** | 全身逆动力学 QP（WBC）、模型预测控制（MPC） | 满足动力学和接触约束的整机控制 |
| **3：统一强化学习** | PPO 全身策略及统一观测、动作、奖励 | 轮腿和双臂联合控制 |
| **4：增强型强化学习** | 历史编码、状态预测、多任务与负载自适应 | 鲁棒性和泛化能力提升 |
| **5：模型—学习融合** | RL + WBC / MPC、安全约束和动力学投影 | 兼顾精度、适应性与安全的全身控制 |
| **6：任务驱动控制** | 视觉感知、目标生成、室内导航与移动操作衔接 | 家庭场景自主移动操作 |

以上是**后续研究计划**，并非仓库现有算法的完成清单。各阶段应保持统一的任务接口、基准场景和评价指标，保证对比公平。

## 6. 固定实验任务与评价

初期重点设置四项整机任务：

1. **双臂运动时的机身稳定：** 固定位置进行双臂伸展、回收和末端轨迹跟踪。
2. **稳定移动与原地转向：** 直行、曲线移动、不同角速度转向，评价姿态、速度跟踪和轮地滑移。
3. **移动过程中的双臂协同：** 边移动或转向，边完成双臂目标跟踪。
4. **双臂抓取与负载搬运：** 改变物体质量、位置与质心，分析整机运动稳定性和操作精度。

在基础任务可靠后，再扩展门槛、斜坡、狭窄通道及动态环境中的移动操作。

建议统一记录速度和角速度 RMSE、横滚与俯仰 RMS、左右末端误差、接触力与轮地滑移、关节力矩与能耗、成功率、控制时延及随机种子。详细内容参见 [验证计划](docs/validation_plan.md)。

## 7. 模型来源及使用说明

- AS2-W 与 R1-A7 的 XML 来自之前整理的项目模型；`centaur.xml` 是已有整机描述的英文文件名版本。
- 模型合并后仍需核查坐标系、整机质心、质量惯量、执行器映射、关节极限、自碰撞与轮地接触。
- 原始 STL/URDF 暂不在公开仓库分发，需确认原始资源的许可证与再发布权限。
- 当前控制增益仅用于研究起点，未经实机标定，不可直接作为硬件安全参数。

本仓库的当前重点是：**优先完成整机模型验证和 PD / 全身逆运动学基线，再逐步研究 WBC、MPC、PPO 及模型—学习融合控制。**

## 8. 文件来源校核与整机诊断

根据已有 AS2-W 与 R1-A7 工程压缩包，新增[来源文件及历史验证核查说明](docs/source_verification_zh.md)，并提供本地文件包校验和整机动力学诊断脚本。

```bash
python scripts/check_source_packages.py --as2w-zip /path/to/AS2W_control_project.zip --r1a7-zip /path/to/R1_A7_MuJoCo.zip
python scripts/whole_body_diagnostics.py --model models/centaur/centaur.xml --full-matrices --output experiments/results/whole_body_diagnostics.json
```

第二条命令需要先恢复 STL 并成功编译整机 XML。历史 AS2-W 代理模型的移动结果与 R1-A7 固定基运动学检查，不能当作半人马整机稳定移动验证。

## 9. 最新补充：完整整机控制研究工程及 O6 双手

根据新上传的两个完整压缩包，本仓库新增以下可追溯项目目录：

- [`research/centaur_complete/`](research/centaur_complete/)：保留原包的整机模型、33 执行器映射、站立基线、姿态反馈、离线全身 IK-QP、六项任务基准、自碰撞版本、动力学分析代码及小型结果 JSON/CSV。
- [`models/r1_a7_o6/`](models/r1_a7_o6/)：R1-A7 搭载双 O6 灵巧手的**独立上部模型**（39 个物理关节，29 个执行器）；**尚未验证其与 AS2-W 合并的整机版本**。
- [`docs/uploaded_archives_zh.md`](docs/uploaded_archives_zh.md)：归档来源、研究成果边界及完整复现说明。

原始 ZIP 中的网格、NPZ 轨迹与视频需在本地导入，仓库不公开分发来源不明确的二进制资源：

```bash
python scripts/import_uploaded_archives.py --centaur-zip "/path/to/Centaur_MuJoCo_Complete (2).zip" --o6-zip "/path/to/r1_a7_o6_model(1).zip"
python scripts/check_uploaded_models.py --strict
python scripts/check_uploaded_models.py --strict --compile
```

如需复现归档中的整机任务实验，安装 `configs/requirements_complete.txt` 中依赖，再进入 `research/centaur_complete`，使用：

```bash
python benchmark/run_tasks.py --task all
python whole_body_ik/run.py --case all
```

**完成状态说明：** 归档包含已执行过的站立、动力学分析、离线 IK-QP 和六任务基准记录，但 GitHub 仓库中的二进制网格尚未分发；这些是原压缩包的历史实验输出，**不是本次 GitHub CI 重新运行的控制实验**。当前尚未实现或验证在线全身 WBC、MPC、PPO，也未将 O6 双手与四轮足底盘合成为已验证的整机模型。
