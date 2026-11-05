# R1_A7 + O6 灵巧手 MuJoCo 模型

Unitree R1_A7 人形机器人 + linkerhand_o6 六指灵巧手（双手），纯 MuJoCo MJCF 格式。

## 目录结构

```
r1_a7_o6_model/
├── README.md
├── r1_a7_O6.xml          ← 主模型入口（机器人 + O6 双手）
├── scene_O6.xml           ← 场景文件（地面 + 光照 + 机器人）
├── meshes/                ← 人形身体网格 (18 STL)
├── hand/                  ← O6 灵巧手
│   ├── linkerhand_o6_left.xml
│   ├── linkerhand_o6_right.xml
│   └── meshes/
│       ├── collision/     ← 碰撞网格 (24 OBJ)
│       └── visual/        ← 视觉网格 (24 OBJ)
└── demo.py                ← 独立加载示例
```

## 快速启动

```bash
pip install mujoco>=3.1 numpy

# 方式 1：用场景文件加载（推荐）
python -c "
import mujoco
m = mujoco.MjModel.from_xml_path('scene_O6.xml')
d = mujoco.MjData(m)
print(f'{m.nbody} bodies, {m.njnt} joints, {m.nu} actuators')
"

# 方式 2：只加载机器人
python -c "
import mujoco
m = mujoco.MjModel.from_xml_path('r1_a7_O6.xml')
d = mujoco.MjData(m)
print(f'{m.nbody} bodies, {m.njnt} joints, {m.nu} actuators')
"

# 方式 3：演示脚本（只加载左手，独立控制手指）
python demo.py
```

## 关节概览

| 部位 | 关节数 | 说明 |
|---|---|---|
| 躯干 | 1 | waist_yaw |
| 左臂 | 7 | shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw |
| 右臂 | 7 | 同上 |
| 头部 | 2 | pitch, yaw |
| **左手 O6** | **11** | thumb_cmc_yaw/pitch/ip, index/ring/middle/pinky mcp/dip |
| **右手 O6** | **11** | 同上 |
| **合计** | **39** | |

## 依赖

- mujoco >= 3.1
- numpy
- 无需 ROS、无需 linker-sim、无需 conda 环境