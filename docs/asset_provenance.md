# Model file provenance and geometry

Source artifacts recovered from previous project files (not from an external download):

- AS2-W: `AS2W_control_project.zip` -> `as2w_control/original/robot.xml`, `as2w.urdf`, 36 numbered STL meshes.
- R1-A7: `R1_A7_MuJoCo.zip` -> `r1_a7.xml`, `source/r1_a7.urdf`, 19 STL meshes.
- Combined XML: `四轮足半人马.xml` (formerly generated project artifact). Uploaded under `models/centaur/centaur.xml`; MJCF model label translated to `centaur`, and mesh paths redirected to shared source directories.

`models/as2_w/as2_w.xml` is copied from the standalone `AS2W.xml` library artifact. `models/r1_a7/r1_a7.xml` is copied from the R1-A7 model library artifact. Both retain original joint, inertial, actuator data; no vendor parameters are invented.

**Mesh and source URDF rights:** The mesh binaries and source URDF files are not redistributed here. They may carry third-party ownership or redistribution restrictions. The importer copies from archives the user already possesses into ignored local paths, and does not modify original geometry bytes. Confirm asset distribution rights before making those binaries public or choosing a repository license.

Expected after local restoration:
- `models/as2_w/meshes/part_1.stl` ... `part_36.stl` (36)
- `models/r1_a7/meshes/*.STL` (19)
- `models/as2_w/as2_w.urdf` and `models/r1_a7/r1_a7.urdf`

The combined XML refers to these local directories directly; avoid copying the meshes a second time into the centaur model folder.
