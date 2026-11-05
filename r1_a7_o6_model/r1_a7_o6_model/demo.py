"""R1_A7 + O6 灵巧手 — 纯 MuJoCo 加载演示。

用法:
  python demo.py
"""

import mujoco
import numpy as np

print("=== 加载场景 scene_O6.xml ===")
model = mujoco.MjModel.from_xml_path("C:\\Users\\16389\\Desktop\\r1_a7_o6_model\\r1_a7_o6_model\\scene_O6.xml")
data = mujoco.MjData(model)
print(f"  bodies={model.nbody}, joints={model.njnt}, actuators={model.nu}, geoms={model.ngeom}")

# 列出所有关节
print("\n=== 关节列表（限幅信息）===")
for j in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
    qa = model.jnt_qposadr[j]
    rg = model.jnt_range[j] if model.jnt_limited[j] else None
    lim = f"range={rg}" if rg is not None else "unlimited"
    print(f"  {name:40s} qpos={data.qpos[qa]:7.4f}  {lim}")

# 仿真 200 步，观察自由落体/初始姿态
print("\n=== 仿真 200 步 ===")
mujoco.mj_forward(model, data)
for step in range(200):
    mujoco.mj_step(model, data)
    if step % 50 == 0:
        pelvis_z = data.qpos[2]  # pelvis z
        print(f"  step {step:3d}: pelvis_z={pelvis_z:.4f} m")

print("\n完成！模型加载正常。")