# GORM Rover twist_mux Control System

## Overview

The GORM rover now uses the ROS 2 `twist_mux` multiplexer node to robustly switch between different control sources. This provides a clean, modular approach to handling multiple velocity command inputs with automatic priority-based switching and manual override capabilities.

## Architecture

### Control Flow
```
Input Sources → twist_mux → Base Control → Motor Commands
```

1. **Input Sources** (Publishers):
   - **Local Joystick**: `joy_node` publishes `sensor_msgs/Joy` to `/joy/joy`. A `joy_to_cmd_vel_node` instance processes this into `geometry_msgs/Twist` on `/joystick/cmd_vel` (Priority: 100).
   - **Remote Control**: A remote `joy_node` publishes `sensor_msgs/Joy` to `/remote/joy`. A second `joy_to_cmd_vel_node` instance on the rover processes this into `geometry_msgs/Twist` on `/remote/cmd_vel` (Priority: 90).
   - **Autonomous Navigation**: The RL navigation node publishes `geometry_msgs/Twist` directly to `/autonomous/cmd_vel` (Priority: 10).

2. **Central Multiplexer**:
   - `twist_mux` subscribes to all input topics and selects one based on priority
   - Publishes selected velocity command to `/cmd_vel`

3. **Robot Control**:
   - `ackermann_node` subscribes to `/cmd_vel` and converts to motor commands

### Priority System

- **Joystick**: Priority 100 (Highest) - Direct manual control takes precedence
- **Remote**: Priority 90 - Remote joystick control  
- **Autonomous**: Priority 10 (Lowest) - Autonomous navigation runs when no manual control is active

### Safety Features

- **Timeout Protection**: Each input has a 0.5-second timeout. If no messages are received within the timeout, that source is automatically deactivated
- **Graceful Degradation**: Robot stops safely if all active sources disconnect

## Usage

### Automatic Mode Switching

The system automatically selects the active control source with the highest priority:

1. Move joystick → Joystick control takes over
2. Stop using joystick (timeout after 0.5s) → Remote control activates (if publishing)
3. No manual control → Autonomous navigation runs

### Manual Mode Switching (Service Calls)

You can manually lock to a specific control source using ROS 2 services:

#### Lock to Autonomous Mode
```bash
ros2 service call /twist_mux/change_topic twist_mux_msgs/srv/SetMux "{topic: 'autonomous'}"
```

#### Lock to Joystick Mode
```bash
ros2 service call /twist_mux/change_topic twist_mux_msgs/srv/SetMux "{topic: 'joystick'}"
```

#### Lock to Remote Mode
```bash
ros2 service call /twist_mux/change_topic twist_mux_msgs/srv/SetMux "{topic: 'remote'}"
```

#### Unlock (Return to Automatic Priority-based Switching)
```bash
ros2 service call /twist_mux/change_topic twist_mux_msgs/srv/SetMux "{topic: ''}"
```

### Monitor Current Mode

Check which control source is currently active:

```bash
ros2 topic echo /twist_mux/selected
```

## Launch Files

### Main System Launch
```bash
# Launches complete system with twist_mux
ros2 launch gorm_bringup bringup_teleop.launch.py
```

### Individual Components
```bash
# Just the control switching system
ros2 launch gorm_bringup control_switch.launch.py

# Just teleop (publishes to /joystick/cmd_vel and /remote/cmd_vel)
ros2 launch gorm_teleop teleop.launch.py

# Autonomous navigation (publishes to /autonomous/cmd_vel)
ros2 launch gorm_navigation autonomous_navigation.launch.py
```

## Configuration Files

The twist_mux configuration is in `gorm_bringup/config/twist_mux.yaml`:

```yaml
twist_mux:
  ros__parameters:
    cmd_vel_out: "cmd_vel_out"
    topics:
      joystick:
        topic: "joystick/cmd_vel"
        timeout: 0.5
        priority: 100
      remote:
        topic: "remote/cmd_vel" 
        timeout: 0.5
        priority: 90
      autonomous:
        topic: "autonomous/cmd_vel"
        timeout: 0.5
        priority: 10
```

## Topics

### Input Topics (twist_mux subscribes to these)
- `/joystick/cmd_vel` - Local joystick control commands
- `/remote/cmd_vel` - Remote joystick control commands  
- `/autonomous/cmd_vel` - Autonomous navigation commands

### Intermediate Topics
- `/local/joy` - Raw data from the local joystick
- `/remote/joy` - Raw data from the remote joystick

### Output Topics (twist_mux publishes to these)
- `/cmd_vel` - Selected velocity commands sent to base control

### Status Topics
- `/twist_mux/selected` - Currently selected input topic
- `/twist_mux/twist_stamped` - Selected twist commands with timestamp

## Dependencies

The system requires the `twist_mux` package:

### ROS Package Dependencies
- Added to `gorm_bringup/package.xml`: `<depend>twist_mux</depend>`

### Docker/System Dependencies  
- Added to `Dockerfile`: `ros-humble-twist-mux`

### Manual Installation (if needed)
```bash
sudo apt update
sudo apt install ros-humble-twist-mux
```

## Troubleshooting

### No Movement
1. Check if any control source is publishing:
   ```bash
   ros2 topic echo /joystick/cmd_vel
   ros2 topic echo /autonomous/cmd_vel
   ```

2. Check twist_mux status:
   ```bash
   ros2 topic echo /twist_mux/selected
   ```

3. Verify base control is receiving commands:
   ```bash
   ros2 topic echo /cmd_vel
   ```

### Wrong Control Source Active
- Use manual override service calls to lock to desired source
- Check timeout settings in configuration file
- Verify priority values are correct

### Service Calls Not Working
- Ensure twist_mux node is running:
   ```bash
   ros2 node list | grep twist_mux
   ```
- Check available services:
   ```bash
   ros2 service list | grep twist_mux
   ```