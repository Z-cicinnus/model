# 2026-11-03：来自用户两个 ZIP 的研究代码同步

## 来源与保留策略

上传文件：

1. `Centaur_MuJoCo_Complete (2).zip`，含统一 AS2-W+R1-A7 整机、站立、机身位姿反馈、全身速度级 IK-QP、六项移动操作基准、自碰撞模型、动力学分析及对应结果。
2. `r1_a7_o6_model(1).zip`，含 R1-A7 + 双 O6 灵巧手 MJCF、示例脚本、18 个身体 STL、48 个手部 OBJ。

新研究源码按历史目录原样放在 `research/centaur_complete/`。O6 上部模型放在 `models/r1_a7_o6/`。**不覆盖先前的 `models/centaur/centaur.xml` 研究基线**，也不把 O6 独立模型宣称为已集成的 AS2-W 半人马。

## 新文件本地导入

```bash
python scripts/import_uploaded_archives.py --centaur-zip '/path/to/Centaur_MuJoCo_Complete (2).zip' --o6-zip '/path/to/r1_a7_o6_model(1).zip'
python scripts/check_uploaded_models.py --strict
python scripts/check_uploaded_models.py --strict --compile
```

完整整机包中的 AS2-W 36 个 STL 和 R1-A7 19 个 STL 同时复制到原仓库的 `models/as2_w/meshes`、`models/r1_a7/meshes`，因此原整机 XML 的网格引用也可以得到补齐。自碰撞实验另需要 `research/centaur_complete/meshes/collision_v1`，导入脚本会恢复。O6 的 18 个 STL、48 个 OBJ 保存在 `models/r1_a7_o6/meshes` 和 `models/r1_a7_o6/hand/meshes`。

## 源项目已报告的结果（尚未在 GitHub CI 中重新运行）

- 原整机自由基座模型：33 个执行器，`nq=40`、`nv=39`，源报告的总质量 `42.19940901 kg`；整机质量矩阵为 `39×39`。
- 来源的动力学分析包括双腕雅可比、四轮接触位置雅可比、质心雅可比及质量矩阵正定性检查；数值与适用边界详见 `research/centaur_complete/dynamics_analysis/README_zh.md`。
- 已有站立、外力扰动、双臂小幅动作的仿真基线；已做离线速度级 QP 的原地全身协调仿真，以及六项任务跟踪基准。
- 源项目明确说明：已有任务控制使用**离线全身速度 QP 参考 + 关节 PD + 轮式位姿反馈**；尚非在线优化全身 WBC、MPC 或 PPO，也未覆盖实物抓取、复杂地形及实机。
- O6 上部模型的源码可做独立结构检查，但未见该 O6 模型与四轮足整机完成闭环控制验证的记录。

## 仍待完成

1. 恢复全部真实网格、检查自碰撞模型与当前 MuJoCo 编译。
2. 比较新增整机模型与 `models/centaur/centaur.xml` 的哈希、关节、执行器和质量参数。
3. 对六项任务重新运行并存档种子、版本、输入/输出及通过率。
4. 按统一动作接口扩展为在线 WBC/QP、MPC、PPO，再考虑 O6 双手与 AS2-W 整机联合模型。

## 公开仓库的资产限制

上传的 STL、OBJ、视频、NPZ 与部分原始 URDF **不自动具有公开再分发许可**。由于仓库为 public，本次保留源码、模型文本和小型可复核 JSON/CSV 结果；二进制网格由用户本地 ZIP 导入、不纳入 Git 管理。归档名及源码引用仅用于溯源，不构成第三方商标、硬件型号或实验真实性保证。
