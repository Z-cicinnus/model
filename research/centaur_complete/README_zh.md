# 四轮足半人马 MuJoCo 完整项目

AS2-W 与 R1-A7 为统一浮动基座模型，安装位置 [-0.015, 0, 0.1] m，安装四元数 [1,0,0,0]，33个执行器。所有文件名称已改为英文，中文说明保留。模型参数未因重命名而改变。

## 安装与当前入口

在本目录运行：

```bash
python -m pip install -r requirements.txt
python benchmark/run_tasks.py --task all
```

上述命令运行任务跟踪控制器并更新六项任务结果。可单独选择 `--task straight_reach` 或 `--task turn_reach`。参考规划在动力学仿真前完成，运行并非实时窗口演示。

查看已记录的仿真运动：

```bash
python benchmark/replay_tasks.py --task straight_reach
python benchmark/replay_tasks.py --task turn_reach
```

回放只是显示已记录状态，不是新的控制实验。算法、任务限制与结果见 benchmark/Task_Controller_Notes.md；统一环境协议见 benchmark/README_zh.md。

## 内容索引

- centaur.xml / fixed_base.xml：用户确认原模型及固定基座检查版本，原始基线在source/user_confirmed。
- centaur_self_collision.xml / fixed_base_self_collision.xml：后续控制使用的自碰撞修订版，模型必须与meshes一起保留。
- assembly_config.json / collision_config.json / scripts：装配、碰撞与验证脚本。
- dynamics_analysis：整机质量矩阵、质心、末端与接触雅可比、局部耦合分析及数值校验。
- whole_body_ik：第3项原地小幅全身协调基线，独立保留其原有往返测试。
- benchmark：固定六任务环境、保持控制负对照、任务跟踪控制器及最终结果；results/task_qp_v1是当前任务控制器结果。
- pose_feedback / standing：前期站立与位置/偏航反馈控制和结果，用于历史对照。
- static_support / checks_step2：静力分析及模型检查。
- history / benchmark/development：历史与调试记录，不是当前运行入口。

当前控制器采用离线全身速度QP参考、1kHz电机PD和轮式位姿反馈，并用50Hz腿部雅可比修正机身位姿误差。尚未实现在线全身优化、MPC或强化学习。统一环境当前测试平地空载、低速移动和腕部目标，没有物体抓取/释放、载荷搬运、复杂地形或实机验证。自碰撞使用近似凸块，无碰撞结论仅限所测轨迹。驱动力矩限值沿用源模型，未核实连续额定值；安装件质量尚未加入。
