# Centaur Robot — MuJoCo Whole-Body Simulation

[中文说明 · Chinese README](README.zh-CN.md)  |  [Jumping-control research roadmap / 跳跃研究规划](README.zh-CN.md#跳跃控制研究规划)

This is the research repository for **AS2-W four wheel-legged base + R1-A7 humanoid upper body**, modeled as one floating-base multibody robot. File and directory names use English snake_case.

**Current status (2026-11-02):** Original XML descriptions have been imported from previously shared project assets, including the combined centaur XML (1 floating base, 55 referenced meshes). Mesh binaries and source URDFs are **not present in this public repository**; they must be restored locally from the supplied source archives. The assembled robot **cannot compile until those files are installed**. No whole-body locomotion/control results are claimed.

## Quick start

1. Install Python 3.10+ and run `python -m pip install -e ".[dev]"`.
2. Restore geometry from your original `AS2W_control_project.zip` and `R1_A7_MuJoCo.zip`:
   ```bash
   python scripts/import_assets.py --as2w-zip /path/to/AS2W_control_project.zip --r1a7-zip /path/to/R1_A7_MuJoCo.zip
   ```
   The asset importer preserves source STL bytes and names, and copies the original URDF files.
3. Check asset completeness: `python scripts/check_repo.py --strict`.
4. Inspect the integrated model: `python scripts/inspect_model.py --model models/centaur/centaur.xml`.
5. Run a short **uncontrolled** physics smoke test: `python scripts/run_simulation.py --model models/centaur/centaur.xml --steps 500`.
6. Run `python -m pytest -q`. A pass without model assets means only repository-level tests were run.

## Repository layout

- `models/as2_w/`: AS2-W MJCF + local URDF/STL (after import).
- `models/r1_a7/`: R1-A7 MJCF + local URDF/STL (after import).
- `models/centaur/centaur.xml`: unified whole-body model, referencing the two source mesh directories.
- `src/centaur_sim/`: shared model/controller interfaces, direct-motor PD baseline.
- `scripts/`: asset import, validation, inspection and run entrypoints.
- `configs/`: controller and experiment configuration placeholders.
- `tests/`: repo/XML structure tests, optional runtime model tests.
- `docs/`: source provenance, assumptions, limits and next milestones.

## Scientific scope and limitations

The integrated MJCF is a **pre-existing model artifact**, not a newly validated physical robot. Previously generated AS2-W driving/turning results were for a *collision-simplified AS2-W-only proxy* and do **not** establish centaur locomotion stability. R1-A7 upper-body conversion was verified in its prior standalone project but hardware parameter accuracy and collision safety remain open. This repository starts with descriptive source files, safety-conscious checks, and baseline interfaces; WBIK, QP/WBC, MPC and PPO are planned, not implemented.

For model data origin and publication concerns see [docs/asset_provenance.md](docs/asset_provenance.md) and [docs/validation_plan.md](docs/validation_plan.md).

**重要说明：** 模型 XML 已上传，但 STL/URDF 资产需要从用户已有压缩包导入后才能正常加载整机模型。不要把仓库结构测试通过误解为整机运动控制通过。
