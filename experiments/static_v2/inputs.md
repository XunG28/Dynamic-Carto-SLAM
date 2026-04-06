## static_v2 inputs

This experiment uses the `static_v2` dataset as the reference input.

- **Input bag**: `bags/static_v2`
- **Localization pbstream**: `maps/static_v2.pbstream`

Expected topics in the input bag:

- `/scan`
- `/odom`
- `/odom_noisy` (slip/noise injected)
- `/imu`
- `/tf`, `/tf_static`
- `/clock` (if `use_sim_time:=true`)
- Ground truth: `/eval/ground_truth/pose`

Key outputs:

- Cartographer localization pose: `/tracked_pose`

