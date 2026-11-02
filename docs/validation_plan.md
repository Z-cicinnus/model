# Reproducible validation plan

## Phase 0 — model integrity
1. Restore the original meshes/URDFs using `scripts/import_assets.py`.
2. Check every MJCF mesh reference and compile all three models.
3. Check `nq`, `nv`, `nu`, joint names, actuator gears, control limits, joint limits and exactly one free joint in the combined model.
4. Compare body masses, local center of mass and inertia matrices with the source URDFs.
5. Validate attachment location/orientation and R1-A7 self-collision limitations.
6. Check flat-ground contacts and controller safety before applying velocity commands.

## Phase 1 — baseline control
- Use one whole-body state and one actuator command vector.
- Verify joint motor mapping with zero torque and bounded PD tests first.
- Introduce separate robot task *commands* (base velocity, body pose, left/right end-effector targets) while solving for all actuators in a unified controller.
- Execute fixed scenarios: static arm movement, straight driving, in-place turns, arm movement during turns, carried object motion.

## Phase 2 — algorithms
- Whole-body IK/QP -> inverse-dynamics WBC -> MPC -> unified PPO -> learning/model fusion.
- Compare on identical models, initial states, scenario parameters and seeds.
- Metrics: velocity/heading RMSE, body tilt RMS, end-effector tracking error, contact stability, torque/energy and timing.

**No Phase 0 or Phase 1 experimental success is asserted by the repository's initial CI tests.**
