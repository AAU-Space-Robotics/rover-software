# ROS 2 Interface Guide

This page documents the ROS 2 topics, services, and message types for the GORM rover.

NOTE: Please update this documentation if you discover new topics or services, or if existing ones change.

## Core Topics

### Control & Navigation

| Topic | Message Type | Description |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Primary velocity command for rover motion |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Navigation goal for autonomous systems |
| `/motor_commands` | `std_msgs/Float64MultiArray` | Low-level commands for the 6-wheel, 4-steer system |

### Localization & Odometry

| Topic | Message Type | Description |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Odometry data for pose estimation |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Robot pose from the AMCL localization system |
| `/tf` | `tf2_msgs/TFMessage` | Dynamic and static coordinate transforms |
| `/tf_static` | `tf2_msgs/TFMessage` | Static coordinate transforms |

### Teleoperation

| Topic | Message Type | Description |
|---|---|---|
| `/joy` | `sensor_msgs/Joy` | Raw joystick input for manual control |

## Sensor Topics

### ZED Camera (Front)

| Topic | Message Type | Description |
|---|---|---|
| `/zed_front/zed/depth/depth_registered` | `sensor_msgs/Image` | Registered depth image |
| `/zed_front/zed/rgb/image_rect_color` | `sensor_msgs/Image` | Rectified color image |
| `/zed_front/zed/rgb/image_rect_color/compressed` | `sensor_msgs/CompressedImage` | Compressed color image for web streaming |
| `/zed_front/zed/point_cloud/cloud_registered` | `sensor_msgs/PointCloud2` | Registered 3D point cloud |
| `/zed_front/zed/imu/data` | `sensor_msgs/Imu` | IMU data from the camera |
| `/zed_front/zed/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Visual odometry pose from the camera |

### GNSS (if enabled)

| Topic | Message Type | Description |
|---|---|---|
| `/zed_front/zed/fix` | `sensor_msgs/NavSatFix` | GNSS position data from ZED camera fusion |

### U-blox GNSS

| Topic | Message Type | Description |
|---|---|---|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Diagnostic information from the U-blox driver |
| `/fix` | `sensor_msgs/NavSatFix` | GNSS position fix |
| `/fix_velocity` | `geometry_msgs/TwistWithCovarianceStamped` | GNSS velocity information |
| `/navclock` | `ublox_msgs/NavCLOCK` | Navigation clock solution |
| `/navinfo` | `ublox_msgs/NavINFO` | Navigation information |
| `/navposecef` | `ublox_msgs/NavPOSECEF` | Position solution in ECEF coordinates |
| `/navposllh` | `ublox_msgs/NavPOSLLH` | Position solution in geodetic coordinates |
| `/navsol` | `ublox_msgs/NavSOL` | Navigation solution information |
| `/navstatus` | `ublox_msgs/NavSTATUS` | Navigation status |
| `/navvelned` | `ublox_msgs/NavVELNED` | Velocity solution in NED frame |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | U-blox node parameter updates |

## Services

### Motor Control

| Service | Service Type | Description |
|---|---|---|
| `/start_motors` | `std_srvs/srv/Trigger` | Initializes and enables all motors |
| `/shutdown_motors` | `std_srvs/srv/Trigger` | Safely disables all motors |

## Key Message Types

- **`geometry_msgs/Twist`**: Linear and angular velocity
- **`geometry_msgs/PoseStamped`**: Position and orientation with a timestamp
- **`sensor_msgs/Image`**: Raw image data
- **`sensor_msgs/CompressedImage`**: Compressed image for low-bandwidth streaming
- **`sensor_msgs/PointCloud2`**: 3D point cloud
- **`sensor_msgs/Joy`**: Joystick button and axis states
- **`nav_msgs/Odometry`**: Estimated pose and velocity
- **`std_srvs/srv/Trigger`**: Standard service for simple actions

## Topic Conventions

- **Global Topics**: Core topics like `/cmd_vel` and `/odom` are in the global namespace.
- **Namespacing**: Sensors are namespaced by their position (e.g., `/zed_front`).
- **Standardization**: Follows ROS REP-105 for common topic names.