# 自碰撞增量更新

将本更新包内容解压到原有 centaur_quadruped 文件夹，允许覆盖 scripts/view_model.py。无需重新下载原 STL。原始两份 XML 和安装位置 [-0.015, 0, 0.1] 保持不变。

运行修订版：

```bash
python scripts/view_model.py --self-collision
```

后续控制使用 centaur_self_collision.xml，初始化时加载 home 关键帧。固定基座查看版本为 fixed_base_self_collision.xml。默认无 --self-collision 时仍加载旧版。--floating --self-collision 仅打开浮动版本，不代表已有站立控制器。

## 本次变更

针对底盘、左右前大腿和左右上臂的碰撞网格，以 99 个凸块替换 5 个单一凸包。所有原始视觉网格、质量、质心、惯量、关节链、限位、执行器和安装位姿保持一致。车轮碰撞网格保持原样。

自碰撞已启用。15 个显式排除对仅覆盖底盘固定刚体组与直接连接的关节刚体，使浮动和固定基座版本的安装关节过滤一致；没有排除底盘与前大腿、底盘与上臂等非相邻运动部件。配置列出每个排除对及理由。

原始双臂姿态仍有约 0.6–1.0 mm 的近似碰撞体重叠，因此修订版 home 将左肩 roll 设为 +3°、右肩 roll 设为 -3°，向外展开。原始姿态保存在 assembly_reference，仅用于装配对照，启用自碰撞时不要把它当作无接触初始状态。两份原始模型不受影响。

## 验证结果及边界

实际环境为 MuJoCo 3.3.7。初始姿态无内部接触；整机下移 0.6 mm 的诊断中四轮均接地。固定基座使用受力矩上限约束的重力补偿保持 0.5 s，最大关节漂移 0 rad。浮动模型运行 0.1 s，无非有限数或引擎警告。这些检查不等于浮动站立稳定性验证。

29 个有限位关节各采样 3 个位置，共 87 个姿态；24 个姿态出现自碰撞，相关关节和接触在 collision_validation.json 中。控制器仍需避免碰撞，不能只依靠关节限位。此抽查不覆盖多关节组合或连续路径。

凸块为 CoACD 近似分解；底盘和上臂达到 24 块上限时仍未达到设置的凹度阈值 0.025，不能视为 CAD 精确干涉认证。其余部件仍使用原凸包；当前结果支持进一步静力与控制开发，尚未证明所有运动范围的几何精度。

复核：python scripts/validate_self_collision.py

根据已保存凸块重新生成两份修订 XML：python scripts/build_self_collision.py

只有重新计算凸块才需要 generate_collision_meshes.py 及 trimesh/coacd；普通加载模型不需要这些库。requirements_collision.txt 记录本次软件版本。build_model.py 仍生成原始基线，不会覆盖带“_自碰撞”后缀的模型；若修改源模型，应同步更新生成器使用的 source/user_confirmed 输入后再复核。
