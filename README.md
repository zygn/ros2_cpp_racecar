# ros2_cpp_racecar
F1TENTH Racecar driving algorithm planner binding for ROS2 

# Installation
```bash
$ cd colcon_ws/src
$ git clone https://github.com/zygn/ros2_cpp_racecar
$ cd .. && colcon build --packages-select ros2_cpp_racecar
$ source ./install/setup.bash
```

# Run
```base 
$ ros2 launch ros2_cpp_racecar racecar.py
```

# Parameters
Check `config/params.yaml` file in folder. 
Rebuild (with `colcon`) is required when parameters are changed.

or you can change parameters directly in `colcon_ws/install/ros2_cpp_racecar/share/ros2_cpp_racecar/config/params.yaml` located file.

```yaml
# Default
racecar:
  ros__parameters:
    bubble_radius: 120
    preprocess_conv_size: 70
    best_point_conv_size: 20
    maximum_lidar_distance: 8.0
    straight_speed: 4.0
    corners_speed: 1.5
    robot_scale: 0.325

    scan_topic: /scan 
    odom_topic: /odom 
    drive_topic: /drive
    tf_topic: /tf_static
    joy_topic: /joy
```

# TODO
- Marker Publisher
- ~~Make Launch file~~
- ~~Declaring Parameters~~
