# Wheel Centaur

本仓库已按用户提供的 `Wheel_Centaur.zip` **重新建立**。仓库顶层内容直接对应压缩包内 `Wheel_Centaur/` 目录；之前的 `models/`、`research/`、`src/centaur_sim/` 等旧版工程不在本次新版提交中。

## 源文件

- `centaur_control/`：半人马整机 MuJoCo XML、模型构建、运动学/动力学、关节控制、工作空间、外力冲击及可视化分析。详细入口见 [原始控制工程说明](centaur_control/README.md)。
- `as2w_model/as2w.urdf`：AS2-W 底盘 URDF。
- `r1_a7_o6_model/r1_a7_o6_model/`：R1-A7 + O6 灵巧手 XML、演示和部件模型说明。
- `centaur_control/analysis_results/`：ZIP 附带的 JSON、CSV 和报告，均为原始归档结果，不代表本次重新验证。

## 二进制资源

由于当前上传接口不支持直接将 ZIP 内的大型 STL、OBJ、NPZ、PNG 等二进制文件整体导入 GitHub，**这些资源尚未提交到仓库**。请将原始 `Wheel_Centaur.zip` 解压，并以相同相对路径放置；或使用新增的 `restore_local_assets.py` 在本地自动恢复。恢复后才能编译与测试依赖网格的 MuJoCo 模型。

```bash
python restore_local_assets.py --zip /path/to/Wheel_Centaur.zip
python -m centaur_control.validate_model
```

本仓库不声明已通过物理仿真或真实机器人验证；控制效果需在完整模型资源和所需 Python 依赖就绪后重新测试。
