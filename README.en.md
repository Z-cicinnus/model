# Centaur Robot — Unified MuJoCo Whole-Body Simulation

[中文 README](README.md) · [Combined MJCF](models/centaur/centaur.xml) · [Validation plan](docs/validation_plan.md)

This repository studies a **single floating-base multibody robot** combining an AS2-W four wheel-legged base and an R1-A7 humanoid upper body. The control research targets stable locomotion, turning, posture regulation, and coordinated dual-arm mobile manipulation.

## Current status

- Source model files: [AS2-W MJCF](models/as2_w/as2_w.xml), [R1-A7 MJCF](models/r1_a7/r1_a7.xml), and [centaur MJCF](models/centaur/centaur.xml).
- The integrated XML describes one floating base, 33 actuators and 55 STL references (36 AS2-W + 19 R1-A7).
- **Original binary meshes and URDFs are not publicly distributed.** Restore these locally from your previously saved source archives before attempting MuJoCo model compilation.
- Basic Python project infrastructure, model checks, a direct-motor joint PD baseline and a shared command interface are present.
- Whole-body IK/QP, WBC, MPC, PPO, advanced RL and hybrid controllers are **planned, not implemented or validated**.
- No claim of stable whole-body standing, turning, mobile manipulation, or hardware safety is implied by these source files.

## Install and restore geometry

```bash
git clone https://github.com/Z-cicinnus/model.git
cd model
python -m pip install -e ".[dev]"
python scripts/import_assets.py --as2w-zip /path/to/AS2W_control_project.zip --r1a7-zip /path/to/R1_A7_MuJoCo.zip
python scripts/check_repo.py --strict
python scripts/check_repo.py --strict --compile
python scripts/inspect_model.py --model models/centaur/centaur.xml
python -m pytest -q
```

A zero-control physics smoke test (not a standing/locomotion controller) can be attempted after the model compiles:

```bash
python scripts/run_simulation.py --model models/centaur/centaur.xml --steps 500
```

## Research roadmap

0. Validate unified geometry, body masses, inertias, joints, actuators, contact and mounting.
1. Establish PD and whole-body kinematics / IK-QP baselines.
2. Develop QP-based inverse dynamics whole-body control (WBC) and MPC.
3. Train one whole-body PPO policy coordinating wheel-legs and both arms.
4. Improve RL with observation history, state prediction, multi-task learning and payload adaptation.
5. Combine learned policies with WBC/MPC and explicit physical constraints.
6. Extend to vision-driven indoor navigation and autonomous mobile manipulation.

**Shared baseline tasks:** stationary arm motions, straight driving, in-place turning, coordinated arm motion while driving, and dual-arm payload transport. Use consistent seeds, models, scenarios and evaluation metrics across algorithm comparisons.

## Repository map

- `models/`: component MJCF models and unified `centaur.xml`; original meshes/URDFs restored locally.
- `src/centaur_sim/`: simulation helpers, unified controller interface and a direct-motor PD baseline.
- `scripts/`: import original assets, check references, inspect models and run a smoke test.
- `configs/`, `scenes/`, `tests/`, `experiments/`, `docs/`: configuration, future scenes, tests, results and documentation.

Read [asset provenance and redistribution constraints](docs/asset_provenance.md) and the [validation plan](docs/validation_plan.md) before publishing source geometry or interpreting numerical results.

**Note:** Passing static XML checks or a mesh-free unit test does not establish that the integrated robot is able to stand, drive, turn, or manipulate objects safely.

## Source archive audits and whole-body diagnostics

See [the source-package and validation audit (Chinese)](docs/source_verification_zh.md). Source ZIP files can be inspected before restoring geometry:

```bash
python scripts/check_source_packages.py --as2w-zip /path/to/AS2W_control_project.zip --r1a7-zip /path/to/R1_A7_MuJoCo.zip
```

After restoring the 55 meshes and successfully compiling the whole-body model, generate a mass-matrix and Jacobian report:

```bash
python scripts/whole_body_diagnostics.py --model models/centaur/centaur.xml --full-matrices --output experiments/results/whole_body_diagnostics.json
```

These diagnostics do not establish a functioning IK/QP or stable whole-body locomotion controller.
