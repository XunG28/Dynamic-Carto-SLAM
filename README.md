## Dynamic-Carto-SLAM (ROS 2 Humble)

TurtleBot3 + Gazebo (Industrial Warehouse) + Cartographer mapping/localization, with offline rosbag2 replay and evaluation helpers.

This repo is organized for two goals:

- **Research-style evaluation**: reproducible KPI comparison (e.g., APE/RPE, RMSE) under degraded wheel odometry (slip/noise).
- **Engineering-style pipeline**: a practical way to improve localization by feeding Cartographer with an EKF-fused odometry (KISS-ICP + wheel odom + IMU).

### Repository layout

- `ros2_ws/src/dynamic_carto_demo/`: launch files, configs, and helper nodes (ROS 2).
- `bags/` (ignored): local rosbag2 datasets and generated output bags.
- `maps/` (ignored): local Cartographer `*.pbstream` assets.
- `eval/` (ignored): local exported `*.tum`, plots, and evo outputs.
- `experiments/`: experiment index and reproducible command templates (no large data committed).

### Key launches (high level)

- `tb3_gazebo_industrial_warehouse.launch.py`: Gazebo Classic + TB3 in the industrial warehouse world.
- `offline_mapping.launch.py`: offline mapping from bag (optional).
- `offline_localization.launch.py`: single-shot bag replay + Cartographer pure localization (records an output bag with `/tracked_pose`).
- `offline_localization_fused.launch.py`: single-shot bag replay + KISS-ICP + EKF; feeds `/odometry/filtered` to Cartographer as odom and records results.

### Build

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

### Experiments

See `experiments/static_v2/` for the current baseline/noisy/fused localization comparison template.

### Results (static_v2)

The current KPI snapshot is tracked in:

- `experiments/static_v2/results.md`

