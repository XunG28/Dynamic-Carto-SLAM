## Dynamic-Carto-SLAM (ROS 2 Humble)

TurtleBot3 + Gazebo (Industrial Warehouse) + Cartographer mapping/localization, with offline rosbag2 replay and evaluation helpers.

You can use this repo to:
- reproduce a localization accuracy comparison under degraded wheel odometry (slip/noise)
- run an odometry fusion pipeline (KISS-ICP + EKF) and feed the result into Cartographer

### Key result (static_v2)

Absolute Pose Error (APE) of Cartographer localization (`/tracked_pose`) vs ground truth:

| run bag | RMSE (m) | mean (m) | median (m) | max (m) | std (m) |
|---|---:|---:|---:|---:|---:|
| `loc_odom` | 0.03295 | 0.03051 | 0.03093 | 0.06127 | 0.01245 |
| `loc_noisy` | **0.33258** | 0.26311 | 0.18674 | 1.00114 | 0.20342 |
| `loc_fused` | **0.08115** | 0.06654 | 0.05416 | 0.23572 | 0.04646 |

**Key improvement:** `loc_fused` reduces RMSE from **0.33258 m** to **0.08115 m** (a **75.6%** reduction) under slip/degraded wheel odometry.

Planned: add Relative Pose Error (RPE) metrics.

### Repository layout

- `ros2_ws/src/dynamic_carto_demo/`: launch files, configs, and helper nodes (ROS 2)
- `experiments/`: experiment notes and command templates

### Data (static_v2)

Expected paths:
- bag: `data/bags/static_v2`
- pbstream: `data/maps/static_v2.pbstream`

Downloads:
- static_v2 bag: <GOOGLE_DRIVE_LINK>
- static_v2 pbstream: <GOOGLE_DRIVE_LINK>

### Build

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

### static_v2: end-to-end recipe

```bash
export DATA_ROOT="$(pwd)/data"

source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

# 1) Baseline: Cartographer + clean wheel odom
ros2 launch dynamic_carto_demo offline_localization.launch.py \
  bag:="$DATA_ROOT/bags/static_v2" \
  pbstream_in:="$DATA_ROOT/maps/static_v2.pbstream" \
  bag_out:="$DATA_ROOT/bags/loc_odom" \
  odom_topic:=/odom \
  use_sim_time:=true

# 2) Degraded: Cartographer + odom_noisy
ros2 launch dynamic_carto_demo offline_localization.launch.py \
  bag:="$DATA_ROOT/bags/static_v2" \
  pbstream_in:="$DATA_ROOT/maps/static_v2.pbstream" \
  bag_out:="$DATA_ROOT/bags/loc_noisy" \
  odom_topic:=/odom_noisy \
  use_sim_time:=true

# 3) Fused: KISS-ICP + EKF -> feed Cartographer with /odometry/filtered
ros2 launch dynamic_carto_demo offline_localization_fused.launch.py \
  bag:="$DATA_ROOT/bags/static_v2" \
  pbstream_in:="$DATA_ROOT/maps/static_v2.pbstream" \
  bag_out:="$DATA_ROOT/bags/loc_fused" \
  use_sim_time:=true
```

### Offline mapping (optional)

```bash
export DATA_ROOT="$(pwd)/data"

source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 launch dynamic_carto_demo offline_mapping.launch.py \
  bag:="$DATA_ROOT/bags/static_v2" \
  pbstream_out:="$DATA_ROOT/maps/static_v2.pbstream"
```

### Evaluation (evo)

Install evo in a dedicated environment:

```bash
python3 -m venv ~/evo_venv
source ~/evo_venv/bin/activate
pip install -U pip
pip install "evo" "matplotlib>=3.7,<3.10" "numpy"
pip install PyQt6
```

Headless plotting:

```bash
export MPLBACKEND=Agg
export QT_QPA_PLATFORM=offscreen
```

Export TUM:

```bash
export DATA_ROOT="$(pwd)/data"

source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

mkdir -p "$DATA_ROOT/eval"
ros2 run dynamic_carto_demo bag_pose_to_tum "$DATA_ROOT/bags/static_v2" /eval/ground_truth/pose -o "$DATA_ROOT/eval/gt_pose.tum"
ros2 run dynamic_carto_demo bag_pose_to_tum "$DATA_ROOT/bags/loc_fused" /tracked_pose -o "$DATA_ROOT/eval/tracked_pose.tum"
```

Compute APE (example):

```bash
source ~/evo_venv/bin/activate

evo_ape tum "$DATA_ROOT/eval/gt_pose.tum" "$DATA_ROOT/eval/tracked_pose.tum" \
  --align --correct_scale \
  -p --plot_mode xy \
  --save_results "$DATA_ROOT/eval/ape_results.zip"
```

