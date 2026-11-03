# 模型归档与完整控制基线（从上传压缩包恢复）

这一目录保存用户上传的 `Centaur_MuJoCo_Complete (2).zip` 中 `centaur_quadruped/` 的**原始文本源码与测试摘要**，保留了控制/碰撞/分析脚本的相对路径。原始二进制 STL、OBJ、NPZ 和视频不上传公开 GitHub；使用 `scripts/import_uploaded_archives.py` 在本地从 ZIP 恢复网格。

## 推荐运行入口

在 `research/centaur_complete` 目录中（先安装 Python 3.10+ / MuJoCo / NumPy / SciPy）：

- `python scripts/validate_model.py`：模型结构校验。
- `python scripts/validate_self_collision.py`：自碰撞模型校验。
- `python dynamics_analysis/run.py`：整机质量矩阵、雅可比和耦合分析。
- `python standing/run_standing.py --case hold`：站立基线。
- `python pose_feedback/run.py --case hold`：位置/偏航反馈基线。
- `python whole_body_ik/run.py --case all`：离线全身速度级 QP + 在线关节 PD。
- `python benchmark/run_tasks.py --task all`：固定六项任务。结果会覆盖/产生本地输出，请先查看原始结果摘要。

详见 `README_zh.md`、`whole_body_ik/README_zh.md`、`benchmark/README_zh.md` 及根目录 `docs/uploaded_archives_zh.md`。

## 重要限制

- 源报告记载了模型/任务的历史测试，并不表示此仓库在当前环境已重新完成运行。
- 基准控制器采用离线 QP 参考、PD 与轮式位姿反馈；**不是在线全身 WBC/MPC 或 PPO**。
- 33 轴整机模型仍没有 O6 双手。新增 O6 模型在 `models/r1_a7_o6` 中作为独立实验变体，不要将其误认为已与底盘整合。
- `centaur_self_collision.xml` 使用近似碰撞几何，当前近似不代表所有姿态的碰撞安全。
