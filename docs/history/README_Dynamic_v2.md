# 半人马动态控制批量验证 v2

将压缩包中的 centaur_control 文件夹合并到原 Wheel_Centaur 根目录。更新包包含 run_dynamic_task.py 和新增 benchmark_dynamic.py。测试数据放在 packaged_results 中。

在 Wheel_Centaur 根目录执行：

```powershell
python -m pip install numpy scipy matplotlib
python -m centaur_control.benchmark_dynamic --suite basic
python -m centaur_control.benchmark_dynamic --suite sweep
```

basic 共 30 组，前后移动速度为 ±0.20 m/s，分别叠加 roll/pitch 的 ±5°、±10°、±15°，另有左右转向叠加 -15°、0°、15°横滚。转弯段前向速度 0.05 m/s，角速度 ±0.05 rad/s。正 roll 为右倾，正 pitch 为前倾，正 yaw 为左转。

sweep 共 18 组，前向速度 0.05、0.25、0.50 m/s，左转角速度 0.05、0.10、0.20 rad/s，分别采用水平和左倾 15°。这是离散参数扫描。两组均采用原始空载上身姿态、水平地面、源模型摩擦参数。每次独立运行 12 秒，0–4 秒站立稳定，4–5.5 秒平滑加载指令，随后保持。起步沿用原控制器的 9 cm 降低底盘目标。

默认顺序运行。可以设置 --workers 2 并行，但每个进程都要加载完整网格模型，内存占用会增加。本次高并发曾导致进程退出，正式基础结果采用单进程重跑。

结果保存到 dynamic_benchmark/basic 和 dynamic_benchmark/sweep，包含逐物理步 CSV、summary.csv、results.json、manifest.json、overview.png 和每组曲线 PNG。再次运行请给出新目录，例如：

```powershell
python -m centaur_control.benchmark_dynamic --suite basic --output dynamic_benchmark/basic_second
```

查看单组后退右倾 15°：

```powershell
python -m centaur_control.benchmark_dynamic --suite basic --case "v-0.20_roll+15" --view --output dynamic_benchmark/view_backward
```

原连续演示命令仍可使用：

```powershell
python -m centaur_control.run_dynamic_task --angle 15 --turn-speed 0.05 --turn-rate 0.05 --view
```

窗口显示增加墙钟同步，模拟计算速度不足时播放会变慢。关闭单组测试窗口会留下部分日志，缺少最后 2 秒时会提示测试未完整执行。

## 数据解释

- 速度和姿态 RMSE 在 10–12 秒统计，用于衡量稳定跟踪。
- 最小轮载、低载荷时长、滑移和力矩利用率在 4–12 秒统计，包含指令过渡。
- 单轮法向力低于 1 N 记为低载荷。low_load_seconds 是任一轮低载荷的累计时间，longest_low_load_seconds 是最长连续时间，单位秒。
- 滑移为实际轮地接触点的轮体材料点切向速度，包含轮子自转和全身运动。多接触点按法向力加权。完全卸载时该轮记 NaN，滑移平均排除该样本，因此必须结合轮载一起阅读。差速转弯中的横向擦滑也包含在内。
- peak_util 为腿部和车轮中最大的实际执行器力矩/模型力矩限值，达到 0.99 记为近饱和。该统计不包含上身执行器。
- screen_pass 是本轮自行设定的初筛标准：线速度 RMSE ≤0.03 m/s，角速度 RMSE ≤0.03 rad/s，横滚和俯仰 RMSE 均≤1°，4–12 秒四轮均≥1 N，且腿轮无近饱和。滑移单独报告，未设统一通过阈值。
- 单步低载荷会使初筛失败，需进一步查看连续时长。短暂卸载与倾覆应分别分析。
- 超过 60°横滚/俯仰或出现非有限状态时提前终止并记录错误。

本次验证环境为 Python 3.12、MuJoCo 3.3.7，仿真步长 2 ms。前版报告曾使用不同 MuJoCo 版本，数值应结合 manifest 中的版本和模型哈希比较。代码语法及仿真在当前环境验证，Windows 图形窗口和 Python 3.14 运行需由本机确认。

## 本轮范围

控制算法沿用轮速闭环、姿态逆解和四轮法向力 QP 分配。新增部分用于测量与对比。完整动力学 WBC、上身姿态改变、外力推扰、摩擦变化、步长敏感性及实物实验需要后续验证。离散测试结果可以作为后续算法对照数据。

## 自适应倾角上限搜索

角度范围已扩展。原连续演示的 --angle 接受 0–60°。大角度建议首先使用下列专用扫描，它对倾斜做渐变加载：

```powershell
python -m centaur_control.scan_tilt_limits
```

扫描静止和 0.20 m/s 前进时的前、后、左、右四方向，共 8 条工况线。0°起步，5°粗步进，遇到首次失败后进行二分，最终成功—失败区间宽度≤0.5°。默认搜索上限60°，达到上限仍通过时标注 cap_reached。每个角度独立重置，不把前一次的残余状态带入下一次。该方法搜索从零角度起的首个可用区间，未排除远端孤立可行区间。

静止表示前向速度指令为零，车轮允许滚动，未约束机身固定在某个世界坐标位置。上身保持源模型初始姿态，起步降低底盘目标9 cm。大角度过渡时间为 max(1.5 s, 目标角度/5°每秒)，采用三次平滑曲线，峰值倾角指令速率约7.5°/s。到达目标后保持4秒，末2秒评价跟踪误差。

倾角搜索的 limit_pass 采用速度RMSE≤0.03 m/s、转向RMSE≤0.03 rad/s、两轴姿态RMSE≤1°、单次连续低载荷≤20 ms、力矩近饱和累计≤20 ms、腿关节越界≤0.02 rad的数值容差，且无非车轮部件接触地面。这比基础测试的逐步严格四轮载荷条件宽松，trials.json 同时保留 screen_pass 以供对照。初筛阈值由本轮设定，允许按实验目标调整。

超过60°实际横滚/俯仰会中止该次仿真并判失败。搜索边界是当前控制器、加载速率、初始站姿及摩擦模型下的跟踪/接触可用边界，包含关节限位影响。它与结构绝对倾角极限、任意速度条件下的稳定边界需要分别研究。尤其应对边界附近做步长敏感性和重复扰动测试。

每个方向文件夹保存全部原始曲线、trials.json、limit.json；总表在 limits.json。first_fail_deg 为空表示只获得下界或基线失败，请结合 status 阅读。


可单独查看某个极限附近姿态，加载速率与搜索一致，例如静止左倾45°：

```powershell
python -m centaur_control.scan_tilt_limits --view-case stationary_left --angle 45 --output dynamic_benchmark/view_left45
```

--view-case 支持 stationary_left、stationary_right、stationary_front、stationary_back、forward_left、forward_right、forward_front、forward_back。--angle 使用正的角度大小，方向由 --view-case 指定。
