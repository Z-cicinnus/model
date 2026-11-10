# 四轮足半人马机器人｜Wheel Centaur

> 统一模型下的底层控制、100 Hz 固定协议基准及全身逆动力学 QP-WBC 仿真研究。

**当前可复核的资料来源**：用户项目的 `Wheel_Centaur.zip`、`Centaur_Dynamic_Benchmark_v2.zip`、`Centaur_Algorithm_Baseline_v5_100Hz.zip`、`Centaur_WBC_v6_100Hz.zip`。仓库根目录以 **v6 源码**为当前主工作版本，v2/v5/v6 的完整 ZIP 原始字节另存至 [`archives/`](archives/) 并提供完整性校验及恢复脚本。原始模型网格仍需从用户自有的 Wheel_Centaur 基础包恢复。

## 工程进度

| 阶段 | 内容 | 文件及状态 |
|---|---|---|
| 基础建模 | AS2-W＋R1-A7/O6 统一整机 MJCF、运动学和动力学分析 | 当前 `centaur_control/` 与基础包；需加载模型资源 |
| 动态实验 v2 | 移动、四方向姿态与倾角边界扫描 | 历史归档，保留旧实验数据 |
| 对比基准 v5 | **固定 21 工况，物理/控制 100 Hz**，基础控制器 | 归档报告为 **19/21** |
| 全身优化 v6 | 同模型和评分协议下的独立 QP-WBC | 归档报告为 **21/21** |
| 强化学习 | PPO 与混合控制 | 待实现，非当前已验证功能 |

**注意**：上述数字来自已有 ZIP 的历史确定性仿真实验报告，不是此次 GitHub 更新重新运行出的成绩，也不代表实物或真实家庭环境的通过率。WBC 在部分无扰动任务中的跟踪误差高于基础控制器，且当前求解开销尚不能保证实机 100 Hz 截止时间。

## 目录

- [`centaur_control/`](centaur_control/) — v6 的基础控制、冻结 21 工况、WBC 求解器、评价和测试脚本。
- [`as2w_model/`](as2w_model/) 与 [`r1_a7_o6_model/`](r1_a7_o6_model/) — 初始模型与配套资源路径。
- [`benchmarks/`](benchmarks/) — v5/v6 的协议和成绩摘要（不是完整原始轨迹）。
- [`docs/history/`](docs/history/) — 各阶段原始中文版说明和验证报告。
- [`archives/`](archives/) — 三个**完整 ZIP 的二进制分块**及 SHA-256 校验清单。
- [`scripts/reconstruct_update_packages.py`](scripts/reconstruct_update_packages.py) — 无依赖恢复原始 ZIP。

## 安装与运行

建议从项目根目录执行，使用 MuJoCo 3.3.7 与 Python 3.10+；安装 WBC 依赖：

```powershell
python -m pip install -r requirements_wbc.txt
```

确认已有基础模型 STL/OBJ 文件，再执行：

```powershell
# 重新生成基础控制器对照
python -m centaur_control.run_comparison_suite --output comparison_baseline_v5
python -m centaur_control.evaluate_comparison_suite comparison_baseline_v5

# 运行全身 WBC 的同一套 21 工况
python -m centaur_control.run_wbc_suite --output comparison_wbc_v6
python -m centaur_control.evaluate_comparison_suite comparison_wbc_v6

# 统一协议的公平对比
python -m centaur_control.compare_algorithms comparison_baseline_v5 comparison_wbc_v6 --output comparison_algorithms.csv

# 程序检查
python -m unittest centaur_control.tests.test_foundation_tasks centaur_control.tests.test_dynamic_benchmark centaur_control.tests.test_foundation_benchmark centaur_control.tests.test_comparison_suite centaur_control.tests.test_wbc
```

正式评分须使用同一模型、相同初态、**21 个完整工况**、相同扰动和评分器；不得通过删除失败项或改变门槛提高成绩。单工况图形预览可用 `--view`，其 `--speed` 仅改变观看倍速，不改变物理时间轴。

## 完整更新包恢复

```powershell
python scripts/reconstruct_update_packages.py --all --output-dir update_packages
```

严格校验整体 SHA-256 后获得三份原始 ZIP；见 [`archives/README.md`](archives/README.md)。若分块尚未全部提交，请勿将其视为可恢复的完整更新包。恢复出的 v2/v5 是**历史归档**，不要覆盖当前工作版本的 v6 文件。

## 研究约束

100 Hz 为固定仿真/控制更新频率，不等于 WBC 求解总能在 10 ms 内完成。参考：[v5 基准说明](docs/history/README_Baseline_v5.md)、[v6 WBC 实现](docs/history/README_WBC_v6.md)、[v6 验证报告](docs/history/Report_WBC_v6.md)。尚未覆盖实际抓取、坡道、门槛、地形变化、外界感知噪声及实体机器人。
