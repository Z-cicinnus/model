"""MuJoCo交互验收：实际冲击轨迹、逆解过程、接地姿态和协同工作空间。"""

import argparse
import hashlib
import json
from pathlib import Path
import time
import mujoco
import mujoco.viewer
import numpy as np
from centaur_control.paths import MODEL_PATH
from centaur_control.models.orientation import roll_pitch

from centaur_control.paths import FORMAL_DIR, RESULTS_DIR


def sphere(scene, point, radius, rgba):
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE, np.full(3,radius),
                       np.asarray(point,dtype=float), np.eye(3).ravel(), np.array(rgba,dtype=np.float32))
    scene.ngeom += 1


def line(scene, start, end, rgba, arrow=False):
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    kind = mujoco.mjtGeom.mjGEOM_ARROW if arrow else mujoco.mjtGeom.mjGEOM_CAPSULE
    mujoco.mjv_initGeom(geom, kind, np.zeros(3), np.zeros(3), np.eye(3).ravel(), np.array(rgba,dtype=np.float32))
    mujoco.mjv_connector(geom, kind, .008 if arrow else .002, np.asarray(start), np.asarray(end))
    scene.ngeom += 1


def view(mode, case, speed=1., seconds=0., results_dir=None):
    ROOT = Path(results_dir) if results_dir is not None else RESULTS_DIR if mode == "impact" else FORMAL_DIR
    path = ROOT/"impacts"/(case+".npz") if mode == "impact" else ROOT/"height_tilt_scan.npz" if mode == "height_optimum" else ROOT/"analysis_data.npz"
    if not path.is_file():
        raise FileNotFoundError(f"缺少回放文件 {path}，请先运行对应分析脚本。")
    arrays = np.load(path,allow_pickle=False)
    metadata_path = path.with_suffix(".json") if mode == "impact" else ROOT/"height_tilt_scan.json" if mode == "height_optimum" else ROOT/"analysis_summary.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    provenance = ROOT/"impacts"/"summary.json" if mode == "impact" else metadata_path
    summary = json.loads(provenance.read_text(encoding="utf-8"))
    if summary["model_sha256"] != hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest():
        raise RuntimeError("模型已变化，须重新分析后才能回放，防止用新模型显示旧轨迹。")
    key = {"impact":"qpos","ik":"ik_trajectory","posture":"posture_qpos","workspace":"workspace_qpos",
           "height_sweep":"height_tilt_qpos","height_optimum":"best_qpos"}[mode]
    trajectory = arrays[key].reshape(-1,arrays[key].shape[-1]) if mode == "height_sweep" else arrays[key]
    frame_dt = .01 if mode == "impact" else .08 if mode in ("ik","posture") else 2. if mode == "height_sweep" else 1.
    timestamps = arrays["t"] if mode == "impact" else np.arange(len(trajectory))*frame_dt
    duration = float(timestamps[-1])+1.5
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    state = {"paused":False,"reset":False}
    def key_callback(keycode):
        if keycode == 32:
            state["paused"] = not state["paused"]
        # Home 不受中文输入法字母组合影响；保留 R 兼容原操作。
        elif keycode in (82,114,268):
            state["reset"] = True
    print("MuJoCo验收：空格暂停/继续；Home或英文输入法下R从头开始；鼠标旋转、滚轮缩放；关闭窗口结束。",flush=True)
    print("黄色球=整机质心；红色箭头=外力方向（1000N对应1m）；蓝/绿色点=左右手采样。",flush=True)
    print("冲击模式显示实际动力学积分；其他模式为几何状态展示，姿态切换不代表可执行动态轨迹。",flush=True)
    if mode == "impact":
        print(json.dumps({k:metadata[k] for k in ("mode","point","direction","duration_s","peak_force_N","status","first_lift","fall_event")},ensure_ascii=False,indent=2),flush=True)
    if mode == "height_sweep":
        print("五高度序列：每个样本依次为水平、pitch正、pitch负、roll正、roll负极限。",flush=True)
        for item in summary["height_tilt_analysis"]:
            print(json.dumps(item,ensure_ascii=False),flush=True)
    if mode == "height_optimum":
        print(f"高度粗扫描步长 {metadata['search']['height_step_mm']:g} mm，显示四方向峰值姿态。",flush=True)
        for key,item in metadata["directional_maxima"].items():
            print(f"{item['label']}：高度{item['height_m']:.5f} m，角度{item['signed_angle_deg']:.3f}°",flush=True)
    started = last = time.perf_counter()
    playback = 0.
    with mujoco.viewer.launch_passive(model,data,key_callback=key_callback,show_left_ui=False,show_right_ui=False) as viewer:
        viewer.cam.distance = 2.4
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        viewer.cam.lookat[:] = [.0,.0,.55]
        viewer.opt.geomgroup[3] = 0
        print("MuJoCo可视化窗口已打开。",flush=True)
        while viewer.is_running():
            now = time.perf_counter()
            if seconds and now-started >= seconds:
                break
            if not state["paused"]:
                playback += (now-last)*speed
            last = now
            if state["reset"]:
                playback,state["reset"] = 0.,False
            t = playback % duration
            index = min(len(trajectory)-1,max(0,int(np.searchsorted(timestamps,t,side="right")-1)))
            with viewer.lock():
                data.qpos[:] = trajectory[index]
                data.qvel[:] = arrays["qvel"][index] if mode == "impact" else 0.
                mujoco.mj_forward(model,data)
                scene = viewer.user_scn
                scene.ngeom = 0
                com = data.subtree_com[model.body("base_link").id]
                sphere(scene,com,.018,[1,.8,0,1])
                line(scene,com,np.r_[com[:2],.005],[1,.8,0,.7])
                if mode == "impact":
                    force = arrays["force"][index]
                    point = arrays["point"][index]
                    if np.linalg.norm(force) > 1e-6:
                        line(scene,point,point+force/1000,[1,.1,.05,1],True)
                    viewer.cam.lookat[:] = data.qpos[:3]+[0,0,.2]
                    detail = f"F={np.linalg.norm(force):.1f} N  wheels={int(arrays['wheels'][index])}\n{case}"
                else:
                    support = arrays["support_points"]
                    for a,b in zip((0,1,3,2),(1,3,2,0)):
                        line(scene,support[a]+[0,0,.005],support[b]+[0,0,.005],[1,.5,.1,1])
                    if mode == "workspace":
                        cloud = arrays["workspace_tips_world"]
                        stride = max(1,int(np.ceil(len(cloud)/650)))
                        for pair in cloud[::stride]:
                            sphere(scene,pair[0],.006,[.1,.5,1,.5])
                            sphere(scene,pair[1],.006,[.1,1,.4,.5])
                    if mode == "ik":
                        for goal in arrays["ik_goals"]:
                            sphere(scene,goal[:3,3],.015,[1,.1,.7,.8])
                    if mode == "height_sweep":
                        sample = index//5
                        pose = index%5
                        names = ("level", "pitch+", "pitch-", "roll+", "roll-")
                        detail = f"Height {sample+1}/5  {names[pose]}"
                    elif mode == "height_optimum":
                        names=("pitch+", "pitch-", "roll+", "roll-")
                        item=list(metadata["directional_maxima"].values())[index]
                        detail=f"{names[index]}  H={item['height_m']:.3f}m  limit={item['signed_angle_deg']:.2f}deg"
                    else:
                        detail = "Geometric states only"
                angles = np.degrees(roll_pitch(data))
            viewer.set_texts((None,None,f"{mode}  frame {index+1}/{len(trajectory)}\nt={timestamps[index]:.2f}s  roll={angles[0]:.2f}  pitch={angles[1]:.2f}\n{detail}\n{'PAUSED' if state['paused'] else 'PLAYING'} | SPACE pause | HOME / R restart", ""))
            viewer.sync()
            time.sleep(.015)
    print("可视化已结束。",flush=True)


def main():
    parser=argparse.ArgumentParser(description="整机分析的MuJoCo可视化验收")
    parser.add_argument("--mode",choices=("impact","ik","posture","workspace","height_sweep","height_optimum"),default="impact",help="回放内容")
    parser.add_argument("--case",default="free_chest_backward_100ms_upper",help="冲击轨迹文件名，不含扩展名")
    parser.add_argument("--speed",type=float,default=1.,help="播放倍率，冲击建议0.25")
    parser.add_argument("--seconds",type=float,default=0.,help="窗口自动关闭秒数，0为手动关闭")
    parser.add_argument("--results-dir",type=Path,help="分析结果目录；默认读取 formal，冲击仍读历史目录")
    args=parser.parse_args()
    if args.speed <= 0 or args.seconds < 0:
        parser.error("播放倍率须为正，自动关闭时长不能为负")
    view(args.mode,args.case,args.speed,args.seconds,args.results_dir)


if __name__ == "__main__":
    main()
