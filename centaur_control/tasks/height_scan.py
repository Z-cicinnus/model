"""逐毫米遍历固定四轮支撑下的腿高，搜索各方向最大静态倾角。"""

import argparse
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from centaur_control.workspace.grounded import GroundedWorkspace
from centaur_control.paths import MODEL_PATH

from centaur_control.paths import analysis_output
from centaur_control.tasks.provenance import manifest
LABELS = {"pitch_positive": "前倾", "pitch_negative": "后倾",
          "roll_positive": "右倾", "roll_negative": "左倾"}


def scan(step_mm=1., output_dir=None):
    if not 0.1 <= step_mm <= 5.:
        raise ValueError("高度遍历步长应在0.1到5毫米之间")
    OUT = analysis_output(output_dir, formal=step_mm <= 1.)
    parameters = dict(height_step_mm=step_mm, peak_refinement_step_mm=.1)
    manifest(OUT, "height_scan", parameters, "running")
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    workspace = GroundedWorkspace(model)
    start = workspace.nominal.copy()
    bounds = [workspace.scan_axis("z", sign) for sign in (-1, 1)]
    min_height = start[2]+bounds[0]["feasible"]
    max_height = start[2]+bounds[1]["feasible"]
    heights = np.arange(min_height, max_height, step_mm/1000.)
    if not len(heights) or max_height-heights[-1] > 1e-9:
        heights = np.r_[heights, max_height]
    keys = ("pitch_positive", "pitch_negative", "roll_positive", "roll_negative")
    axes = {key: ("pitch" if key.startswith("pitch") else "roll",
                  1 if key.endswith("positive") else -1) for key in keys}
    values = np.full((len(heights),len(keys)),np.nan)
    feasible = np.zeros(len(heights),dtype=bool)
    winners = {key:None for key in keys}
    started = time.perf_counter()
    checkpoint = OUT/"height_tilt_scan_progress.json"
    for i,height in enumerate(heights):
        position=start[:3].copy();position[2]=height
        q=workspace.geometry(position,np.eye(3))
        base=workspace.evaluate(q) if q is not None else None
        if base is None:
            continue
        feasible[i]=True
        for j,key in enumerate(keys):
            axis,sign=axes[key]
            result=workspace.scan_axis(axis,sign,base["qpos"])
            angle=abs(float(result["feasible"]))
            values[i,j]=angle
            if winners[key] is None or angle>winners[key]["angle_deg"]:
                winners[key]={"direction":key,"label":LABELS[key],"height_m":float(height),
                              "angle_deg":angle,"signed_angle_deg":float(result["feasible"]),
                              "qpos":result["frames"][-1].tolist(),
                              "normal_forces_N":workspace.evaluate(result["frames"][-1])["normal_forces"].tolist()}
        if (i+1)%10==0 or i+1==len(heights):
            progress={"completed":i+1,"total":len(heights),"height_m":float(height),
                      "elapsed_s":time.perf_counter()-started,
                      "current_maxima_deg":{key:None if winners[key] is None else winners[key]["angle_deg"] for key in keys}}
            checkpoint.write_text(json.dumps(progress,ensure_ascii=False,indent=2),encoding="utf-8")
            print(json.dumps(progress,ensure_ascii=False),flush=True)
    if not feasible.any():
        raise RuntimeError("没有高度通过固定四轮接地的静力筛选")
    # 每个方向的离散峰值附近再以0.1 mm分辨率搜索，降低高度离散误差。
    coarse_winners={key:winners[key]["height_m"] for key in keys}
    for key in keys:
        best=winners[key]
        left=max(min_height,best["height_m"]-step_mm/1000.)
        right=min(max_height,best["height_m"]+step_mm/1000.)
        refine=np.unique(np.r_[np.arange(left,right,.0001),right])
        for height in refine:
            position=start[:3].copy();position[2]=height
            q=workspace.geometry(position,np.eye(3))
            base=workspace.evaluate(q) if q is not None else None
            if base is None:
                continue
            axis,sign=axes[key]
            result=workspace.scan_axis(axis,sign,base["qpos"])
            angle=abs(float(result["feasible"]))
            if angle>best["angle_deg"]:
                best.update(height_m=float(height),angle_deg=angle,
                            signed_angle_deg=float(result["feasible"]),qpos=result["frames"][-1].tolist(),
                            normal_forces_N=workspace.evaluate(result["frames"][-1])["normal_forces"].tolist())
    overall=max(winners.values(),key=lambda item:item["angle_deg"])
    summary={"model_sha256":__import__("hashlib").sha256(MODEL_PATH.read_bytes()).hexdigest(),
             "mujoco_version":mujoco.__version__,"fixed_wheel_contact_points_world_m":workspace.points.tolist(),
             "search":{"min_base_link_height_m":min_height,"max_base_link_height_m":max_height,
                       "height_step_mm":step_mm,"coarse_height_count":int(len(heights)),
                       "feasible_height_count":int(feasible.sum()),"peak_refinement_step_mm":.1,
                       "fixed_base_xy_m":start[:2].tolist(),"upper_body":"源文件零位",
                       "constraints":"四轮固定接触点；逐轴倾斜；非负轮法向力平衡重力；当前模型碰撞、关节范围及执行器/关节力矩限幅"},
             "directional_maxima":winners,"overall_maximum":overall,
             "resolution_note":f"高度粗扫描步长{step_mm:g}毫米，峰值附近0.1毫米细化；角度按0.5度扫描并局部二分。属于模型当前构型分支的数值搜索，不是任意腿部构型的全局证明。"}
    summary_path=OUT/"height_tilt_scan.json"
    summary_path.write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    pose_order=[winners[k]["qpos"] for k in keys]
    np.savez_compressed(OUT/"height_tilt_scan.npz",heights_m=heights,angles_deg=values,feasible=feasible,
                        best_qpos=np.asarray(pose_order),support_points=workspace.points)
    lines=["# 固定四轮支撑下的最佳倾斜腿高","",
           "机身x/y固定在名义位置，双臂、腰、头和手指保持源文件零位；四轮接触点固定。遍历底盘高度，再对前、后、左、右分别做单轴倾斜搜索。计入重力静力平衡、关节限位、模型当前启用的碰撞和力矩约束。","",
           f"高度区间 {min_height:.6f}–{max_height:.6f} m，粗遍历步长 {step_mm:.3f} mm，共 {len(heights)} 点；峰值附近另以0.1 mm间隔细化。角度搜索先以0.5°步进，再局部二分。","",
           "| 方向 | 最大倾角 | 对应base_link高度 | 重力下轮端法向力 N（FL, FR, RL, RR） |","|---|---:|---:|---|"]
    for key in keys:
        item=winners[key]
        lines.append(f"| {item['label']} | {item['angle_deg']:.3f}° | {item['height_m']:.5f} m | {np.round(item['normal_forces_N'],2).tolist()} |")
    lines += ["",f"总体最大值：{overall['label']} {overall['angle_deg']:.3f}°，发生在base_link高度 {overall['height_m']:.5f} m。","",
              "这是静态准静态构型可行边界，不代表动态运动中可稳定跟踪；碰撞排除、接触摩擦和电机模型仍未实物校准。高度路径只改变机身z，不能推广到通过改变x/y或独立调整四条腿找到的其他构型。",
              "可视化回放：`python centaur_control/view_robot_analysis.py --mode height_optimum`。逐方向显示四个最大倾角姿态；四轮接触位置以橙色线标出。","",
              f"原始逐高度结果在 `{OUT/'height_tilt_scan.npz'}`；MuJoCo {mujoco.__version__}。"]
    (OUT/"height_tilt_scan_report.md").write_text("\n".join(lines)+"\n",encoding="utf-8")
    manifest(OUT, "height_scan", parameters, "complete")
    print("方向峰值：",json.dumps({k:{x:v for x,v in item.items() if x not in ("qpos","normal_forces_N")} for k,item in winners.items()},ensure_ascii=False,indent=2),flush=True)
    print("总体最大：",json.dumps({k:v for k,v in overall.items() if k not in ("qpos","normal_forces_N")},ensure_ascii=False),flush=True)
    print(f"报告：{OUT/'height_tilt_scan_report.md'}",flush=True)
    return summary


def main():
    parser=argparse.ArgumentParser(description="逐高度搜索重力下四方向最大倾角")
    parser.add_argument("--height-step-mm",type=float,default=1.,help="腿高粗遍历步长，默认1毫米")
    parser.add_argument("--output-dir", type=Path, help="自定义结果目录；默认正式扫描写 formal，粗扫描写 checks")
    args=parser.parse_args()
    scan(args.height_step_mm,args.output_dir)


if __name__ == "__main__":
    main()
