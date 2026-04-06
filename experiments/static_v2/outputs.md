## static_v2 outputs

### Output bags (contain `/tracked_pose`)

- `bags/loc_v2_odom`
- `bags/loc_v2_noisy_tf`
- `bags/loc_v2_fused_ekf_v2`

### Export TUM for evaluation

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

mkdir -p eval
ros2 run dynamic_carto_demo bag_pose_to_tum bags/loc_v2_odom /tracked_pose -o eval/loc_v2_odom_tracked.tum
ros2 run dynamic_carto_demo bag_pose_to_tum bags/static_v2 /eval/ground_truth/pose -o eval/gt_pose.tum
```

Recommended practice:

- Keep KPI tables in markdown (small text) tracked by git.
- Keep large artifacts (`*.tum`, evo zips, plots) in `eval/` (ignored).

