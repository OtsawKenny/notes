
# Architecture break down of a simple delivery robot

[text](architecutre.drawio)

### 1. Perception

Sensors detecting the world around the robot.

Takes raw sensor topics → turns them into understanding of the environment.

Example:
```
/scan → obstacle positions
/points → 3D point cloud
Publishes costmaps: /nav2_costmap/global_costmap/costmap & /nav2_costmap/local_costmap/costmap
```

| Topic                                  | Type                        | Notes                                |
| -------------------------------------- | --------------------------- | ------------------------------------ |
| `/scan`                                | `sensor_msgs/LaserScan`     | 2D LiDAR scan for obstacle detection |
| `/points`                              | `sensor_msgs/PointCloud2`   | 3D point cloud from LiDAR/stereo     |
| `/camera/image_raw`                    | `sensor_msgs/Image`         | RGB camera feed                      |
| `/camera/depth/image_raw`              | `sensor_msgs/Image`         | Depth image for 3D understanding     |
| `/camera/info`                         | `sensor_msgs/CameraInfo`    | Calibration data                     |
| `/obstacles`                           | `custom_msgs/ObstacleArray` | Processed obstacle list              |
| `/nav2_costmap/global_costmap/costmap` | `nav_msgs/OccupancyGrid`    | Global obstacle map                  |
| `/nav2_costmap/local_costmap/costmap`  | `nav_msgs/OccupancyGrid`    | Local obstacle map                   |

### 2. Localization

Where the robot thinks it is.

| Topic                | Type                                      | Notes                                   |
| -------------------- | ----------------------------------------- | --------------------------------------- |
| `/map`               | `nav_msgs/OccupancyGrid`                  | Static map from SLAM                    |
| `/map_metadata`      | `nav_msgs/MapMetaData`                    | Map resolution & size                   |
| `/odom`              | `nav_msgs/Odometry`                       | Odometry from wheel encoders & IMU      |
| `/imu/data`          | `sensor_msgs/Imu`                         | Orientation, acceleration, gyro         |
| `/amcl_pose`         | `geometry_msgs/PoseWithCovarianceStamped` | Estimated robot pose                    |
| `/tf` / `/tf_static` | `tf2_msgs/TFMessage`                      | Transform tree (e.g., map → base\_link) |

### 3. Motion Planning (Nav2)

How the robot decides to move.

| Topic                | Type                        | Notes                            |
| -------------------- | --------------------------- | -------------------------------- |
| `/goal_pose`         | `geometry_msgs/PoseStamped` | Navigation target                |
| `/plan`              | `nav_msgs/Path`             | Global planner output            |
| `/local_plan`        | `nav_msgs/Path`             | Local planner trajectory         |
| `/cmd_vel`           | `geometry_msgs/Twist`       | Velocity commands to controllers |
| `/behavior_tree_log` | `nav2_msgs/BehaviorTreeLog` | Nav2 decision trace              |

### 4. Controls (4-Wheel Drive Specific)

How to make all four wheels move in sync.

Type: Differential drive 4WD → each side’s two wheels move together

Controller: diff_drive_controller (ros2_control) or custom 4WD controller

| Topic                                      | Type                     | Notes                          |
| ------------------------------------------ | ------------------------ | ------------------------------ |
| `/cmd_vel` (input)                         | `geometry_msgs/Twist`    | From Nav2                      |
| `/wheel_states`                            | `sensor_msgs/JointState` | Individual wheel speed/pos     |
| `/left_wheel_velocity_controller/command`  | `std_msgs/Float64`       | Velocity for both left wheels  |
| `/right_wheel_velocity_controller/command` | `std_msgs/Float64`       | Velocity for both right wheels |
| `/joint_states`                            | `sensor_msgs/JointState` | Feedback for odometry          |
| `/odom` (output)                           | `nav_msgs/Odometry`      | Feeds back to Localization     |

### 5. Hardware/Electronics

Sensors, motors, battery, and MCU.

| Topic            | Type                              | Notes                         |
| ---------------- | --------------------------------- | ----------------------------- |
| `/battery_state` | `sensor_msgs/BatteryState`        | Battery SOC, voltage, current |
| `/motor_status`  | `custom_msgs/MotorStatus`         | Motor temp, faults, current   |
| `/diagnostics`   | `diagnostic_msgs/DiagnosticArray` | System health info            |
