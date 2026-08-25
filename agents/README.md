# Historical engineering evidence

The reports and checklists in this directory document work performed before the platform cleanup, primarily against baseline `aafed44` through 2026-08-21.

They are retained because they contain useful hardware evidence: motor packet ordering, encoder signs, charged-pack floor observations, container incidents, and the provenance of `Rosmaster_Lib` assumptions.

They are not current operating instructions. In particular, references to `/odom_raw`, EKF-owned `/odom`, `ekf_filter_node`, 19 packages, navigation/SLAM packages, old device paths, or the previous autostart graph describe the pre-cleanup tree preserved at tag `pre-platform-contract-cleanup`.

For the current encoder-only `/odom` platform, use the root README, `context.md`, and `docs/`.
