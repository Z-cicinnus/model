"""浮动基准站立测试。外力仅在命名的推力测试中出现。"""
from pathlib import Path
import argparse, json, hashlib, time
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from controller import ROOT, StandController

def run(case, duration, viewer=False, record=False):
    """运行单个站立测试用例。
    
    参数:
        case: 测试用例名称 ('hold', 'push_x', 'push_y', 'arms')
        duration: 仿真持续时间（秒）
        viewer: 是否显示交互式查看器
        record: 是否录制视频
    返回:
        metric: 包含测试结果指标的字典
    """
    # 加载模型文件，校验模型是否与静态分析一致
    file = ROOT / 'centaur_self_collision.xml'
    reference = json.loads((ROOT / 'static_support/results.json').read_text())
    if hashlib.sha256(file.read_bytes()).hexdigest() != reference['model_sha256']:
        raise RuntimeError('模型与静态分析不一致；请先运行 scripts/analyze_static_support.py。')

    # 初始化 MuJoCo 模型、数据和站立控制器
    m = mujoco.MjModel.from_xml_path(str(file))
    d = mujoco.MjData(m)
    c = StandController(m, d)
    c.reset(d)

    # 断言：接触检测未被禁用，且无重力补偿
    assert not m.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    assert np.all(m.body_gravcomp == 0)

    # 获取基座 body id 和四条腿的标识
    base = m.body('as2w_base_link').id
    legs = ['FL', 'FR', 'RL', 'RR']  # 前左、前右、后左、后右
    wheels = {m.geom(f'as2w_{s}_foot_collision_1').id: i for i, s in enumerate(legs)}

    # 创建输出目录
    out = ROOT / 'standing/results'
    out.mkdir(exist_ok=True)
    view = None
    renderer = None
    writer = None

    # 如果启用查看器，启动被动查看器并设置相机视角
    if viewer:
        import mujoco.viewer as mv
        view = mv.launch_passive(m, d)
        view.cam.lookat[:] = [0, 0, .55]
        view.cam.distance = 1.9
        view.cam.azimuth = 145
        view.cam.elevation = -12
        view.opt.geomgroup[3] = 0
        view.opt.sitegroup[:] = 0

    # 如果启用录制，初始化渲染器和视频写入器
    if record:
        import imageio.v2 as imageio
        renderer = mujoco.Renderer(m, height=480, width=640)
        writer = imageio.get_writer(str(out / (case + '.mp4')), fps=25, codec='libx264', quality=7)
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [0, 0, .55]
        camera.distance = 1.9
        camera.azimuth = 145
        camera.elevation = -12
        opt = mujoco.MjvOption()
        opt.geomgroup[3] = 0
        opt.sitegroup[:] = 0

    # 初始化统计变量
    rows = []                # 采样数据行
    maxrp = 0                # 最大滚转/俯仰角（弧度）
    minheight = 1            # 最小高度
    maxheight = 0            # 最大高度
    maxxy = 0                # 最大水平位移
    saturations = 0          # 执行器饱和步数
    internal_steps = 0       # 内部接触步数
    maxutil = 0              # 最大力矩利用率
    allfour = 0              # 四轮均着地的采样次数
    lastqvel = 0

    # 计算总步数和采样步长
    total = int(round(duration / m.opt.timestep))
    stride = max(1, round(.01 / m.opt.timestep))       # 数据记录步长（每0.01秒）
    vidstride = max(1, round(.04 / m.opt.timestep))    # 视频帧步长（每0.04秒）

    try:
        for k in range(total):
            wall = time.perf_counter()
            d.xfrc_applied[:] = 0
            external = np.zeros(3)

            # 对于推力测试，在 3.0-3.2 秒之间施加 30N 的外力
            if case in ['push_x', 'push_y'] and 3 <= d.time < 3.2:
                external[0 if case == 'push_x' else 1] = 30
                d.xfrc_applied[base, :3] = external

            # 断言：无关节外力
            assert not np.any(d.qfrc_applied)

            # 计算目标姿态和控制指令
            target = c.target(d.time, case == 'arms')
            d.ctrl[:] = c.command(d, case == 'arms')

            # 检测执行器饱和
            saturations += int(
                np.any(c.last_raw < m.actuator_ctrlrange[:, 0]) or
                np.any(c.last_raw > m.actuator_ctrlrange[:, 1])
            )
            # 更新最大力矩利用率
            maxutil = max(maxutil, float(np.max(
                abs(d.ctrl) / np.maximum(abs(m.actuator_ctrlrange[:, 0]),
                                         abs(m.actuator_ctrlrange[:, 1]))
            )))

            # 执行一步 MuJoCo 仿真
            mujoco.mj_step(m, d)

            # 检查状态是否有效
            if not np.isfinite(d.qpos).all():
                raise RuntimeError('状态量出现非有限值')

            # 计算滚转/俯仰角、水平位移和高度范围
            angles = Rotation.from_quat(d.qpos[[4, 5, 6, 3]]).as_euler('xyz')
            maxrp = max(maxrp, float(max(abs(angles[:2]))))
            maxxy = max(maxxy, float(np.linalg.norm(d.qpos[:2])))
            minheight = min(minheight, float(d.qpos[2]))
            maxheight = max(maxheight, float(d.qpos[2]))

            # 检测内部接触（非足端与地面的接触穿透）
            internal_steps += int(
                any(m.geom_bodyid[x.geom1] > 0 and m.geom_bodyid[x.geom2] > 0 and x.dist < -1e-6
                    for x in d.contact)
            )

            # 按步长记录采样数据
            if k % stride == 0:
                mujoco.mj_forward(m, d)
                fz = np.zeros(4)

                # 遍历所有接触，计算每个轮子的垂直接触力
                for ci, con in enumerate(d.contact):
                    wheel = (con.geom1 if con.geom1 in wheels
                             else con.geom2 if con.geom2 in wheels
                             else None)
                    if wheel is None or (m.geom_bodyid[con.geom1] != 0 and
                                         m.geom_bodyid[con.geom2] != 0):
                        continue
                    force = np.zeros(6)
                    mujoco.mj_contactForce(m, d, ci, force)
                    world = con.frame.reshape(3, 3).T @ force[:3]
                    fz[wheels[wheel]] += world[2] * (1 if con.geom2 == wheel else -1)

                # 拼接一行数据：时间、位置、姿态、速度、轮力、控制量、关节误差、外力
                rows.append(np.r_[d.time, d.qpos[:3], angles, d.qvel[:6],
                                  fz, d.ctrl, target - d.qpos[c.qadr], external])
                allfour += int(np.all(fz > 1))

            #按视频步长渲染并写入视频帧
            if renderer and k % vidstride == 0:
                renderer.update_scene(d, camera, scene_option=opt)
                writer.append_data(renderer.render())

            # 同步查看器显示
            if view and k % stride == 0:
                if not view.is_running():
                    raise RuntimeError('查看器在测试完成前关闭')
                view.sync()

            # 实时同步，控制仿真速度与真实时间一致
            if view:
                time.sleep(max(0, m.opt.timestep - (time.perf_counter() - wall)))
    finally:
        # 清理资源
        if view:
            view.close()
        if writer:
            writer.close()
        if renderer:
            renderer.close()

    # 后处理：构建数据矩阵和尾部统计
    a = np.array(rows)
    tail = a[a[:, 0] >= duration - 1]  # 最后1秒的数据
    warn = [int(x.number) for x in d.warning]

    # 数据列：时间、xyz位置、rpy姿态、基座6维速度、四轮垂向力、33维控制量、33维关节误差、3维外力
    metric = {
        'case': case,
        'duration_s': duration,
        'model_sha256': reference['model_sha256'],
        'mujoco_version': mujoco.__version__,
        'max_roll_pitch_deg': float(np.rad2deg(maxrp)),
        'height_range_m': [minheight, maxheight],
        'max_xy_displacement_m': maxxy,
        'final_position_m': d.qpos[:3].tolist(),
        'final_roll_pitch_yaw_deg': np.rad2deg(angles).tolist(),
        'final_second_base_velocity_rms': np.sqrt(np.mean(tail[:, 7:13] ** 2, axis=0)).tolist(),
        'final_second_wheel_load_mean_N': np.mean(tail[:, 13:17], axis=0).tolist(),
        'final_second_joint_error_max_rad': float(np.max(abs(tail[:, 50:83]))),
        'all_four_loaded_sample_fraction': allfour / len(rows),
        'max_torque_limit_fraction': maxutil,
        'saturated_steps': saturations,
        'internal_contact_steps': internal_steps,
        'engine_warnings': warn,
        'push_N': 30 if case.startswith('push') else 0,
        'push_duration_s': .2 if case.startswith('push') else 0,
        'controller': 'constant whole-model static torque feedforward plus joint PD; no base pose feedback',
        'no_virtual_support': True
    }

    # 验收标准
    metric['acceptance_limits'] = {
        'roll_pitch_deg': 2,
        'height_error_m': .01,
        'xy_displacement_m': .03,
        'internal_contact_steps': 0,
        'saturated_steps': 0
    }

    # 判断是否通过：滚转/俯仰<2°、高度误差<1cm、水平位移<3cm、无内部接触、无饱和、无引擎警告
    metric['passed'] = bool(
        maxrp < np.deg2rad(2) and
        max(abs(minheight - c.qref[2]), abs(maxheight - c.qref[2])) < .01 and
        maxxy < .03 and
        not internal_steps and
        not saturations and
        not any(warn)
    )

    # 保存采样数据和测试指标
    np.savez_compressed(
        out / (case + '.npz'),
        samples=a,
        columns=np.array(
            ['time'] +
            ['x', 'y', 'z'] +
            ['roll', 'pitch', 'yaw'] +
            ['vx', 'vy', 'vz', 'wx', 'wy', 'wz'] +
            ['fz_' + s for s in legs] +
            ['ctrl_' + m.actuator(i).name for i in range(m.nu)] +
            ['error_' + m.actuator(i).name for i in range(m.nu)] +
            ['push_x', 'push_y', 'push_z']
        )
    )
    (out / (case + '.json')).write_text(
        json.dumps(metric, ensure_ascii=False, indent=2)
    )
    print(json.dumps(metric, ensure_ascii=False), flush=True)
    return metric


if __name__ == '__main__':
    # 命令行参数解析
    p = argparse.ArgumentParser()
    p.add_argument('--case', choices=['hold', 'push_x', 'push_y', 'arms', 'all'],
                   default='hold',
                   help='测试用例：hold=悬停保持, push_x=X方向推力, push_y=Y方向推力, arms=摆臂, all=全部')
    p.add_argument('--duration', type=float,
                   help='仿真持续时间（秒），默认hold为30秒，其余为15秒')
    p.add_argument('--viewer', action='store_true',
                   help='显示交互式 MuJoCo 查看器')
    p.add_argument('--record', action='store_true',
                   help='录制MP4视频')
    args = p.parse_args()

    # 运行测试用例
    results = [
        run(case,
            args.duration or (30 if case == 'hold' else 15),
            args.viewer,
            args.record)
        for case in (['hold', 'push_x', 'push_y', 'arms'] if args.case == 'all' else [args.case])
    ]

    # 如有任何用例未通过验收标准，则以非零退出码退出
    if not all(x['passed'] for x in results):
        raise SystemExit('一个或多个用例未通过验收标准')