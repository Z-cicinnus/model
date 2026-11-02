# Naming conventions

- All new folders, Python modules, configuration files and XML filenames: lowercase snake_case in English.
- Preserve original manufacturer joint names and mesh basenames unless every dependent reference is updated and verified.
- AS2-W base model: `models/as2_w/as2_w.xml`.
- Upper body: `models/r1_a7/r1_a7.xml`.
- Unified model: `models/centaur/centaur.xml`.
- Model names inside XML use stable English labels; robot body and actuator prefixes are `as2w_` and `r1a7_`.
- Use explicit `_rad`, `_radps`, `_m`, `_mps` and `_nm` suffixes for measured scalar variables when practical.
- Do not silently rename physical joint names that are used as controller lookups.
