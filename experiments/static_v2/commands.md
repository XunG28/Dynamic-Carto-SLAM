## static_v2 commands (template)

All commands below are intended to be run from the repository root.

### 1) Baseline: Cartographer + clean wheel odom

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 launch dynamic_carto_demo offline_localization.launch.py \
  bag:=bags/static_v2 \
  pbstream_in:=maps/static_v2.pbstream \
  bag_out:=bags/loc_v2_odom \
  odom_topic:=/odom \
  use_sim_time:=true
```

### 2) Degraded: Cartographer + odom_noisy (with TF/odom frame handling)

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 launch dynamic_carto_demo offline_localization.launch.py \
  bag:=bags/static_v2 \
  pbstream_in:=maps/static_v2.pbstream \
  bag_out:=bags/loc_v2_noisy_tf \
  odom_topic:=/odom_noisy \
  use_sim_time:=true
```

### 3) Fused: KISS-ICP + EKF -> feed Cartographer with `/odometry/filtered`

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 launch dynamic_carto_demo offline_localization_fused.launch.py \
  bag:=bags/static_v2 \
  pbstream_in:=maps/static_v2.pbstream \
  bag_out:=bags/loc_v2_fused_ekf_v2 \
  use_sim_time:=true
```

