# Launching the GORM Rover

The GORM rover can be launched in two modes: **development mode** (for active development) and **production mode** (for deployment). This guide covers the development mode. For production deployment, see the [Deployment Guide](../../deployment/overview.md).

## Quick Start with Docker (Development Mode)

1. **Navigate to the Rover Software Directory**: Open a terminal and navigate to the directory where the rover software (on the rover) is located.
    ```bash
    cd /workspace/rover-software/docker
    ```
2. **Run the Docker Container and Attach to It**: Start the Docker container that contains the rover software in development mode and attach to it.
    ```bash
    ./run.sh rover --dev

    # Attach to the running Docker container
    docker exec -it rover bash
    ```
3. **Build the ROS2 Packages**: Inside the Docker container, build the ROS2 packages to ensure everything is set up correctly.
    ```bash
    colcon build
    source install/setup.bash
    ```
4. **Launch the Rover Software**: Finally, launch the rover software using the provided launch file.
    ```bash
    ros2 launch gorm_bringup bringup_teleop.launch.py
    ```

5. **(Optional) Developing and changing the code**: You can make changes to the code in the `src` directory on the host machine (outside the container on the Jetson Orin). The changes will be reflected inside the container because the source code is mounted as a volume. After making changes, you can rebuild the packages inside the container:
    ```bash
    colcon build
    source install/setup.bash
    ros2 launch gorm_bringup bringup_teleop.launch.py
    ```

> **💡 Production Alternative**: For production deployment with automatic restart and pre-built packages, start the deploy image (or use docker-compose).

1. **Run the Production Deployment**: Start the production deployment which runs continuously and automatically restarts if it crashes or the system reboots.
    ```bash
    ./run.sh rover --prod
    # or with docker-compose:
    docker-compose up -d rover-deploy
    ```

<!-- ## Launching the GORM Rover
**Full system bringup:**
```bash
ros2 launch gorm_bringup bringup.launch.py
```

**Teleop mode (manual control):**
```bash
ros2 launch gorm_bringup bringup_teleop.launch.py
```

**Individual components:**
```bash
# Motors only
ros2 launch gorm_base_control motor_driver.launch.py

# Cameras only  
ros2 launch gorm_sensors cameras.launch.py

# Joystick control only
ros2 launch gorm_teleop teleop.launch.py
``` -->

## Useful docker inspection commands
    ```bash
    # List running containers
    docker ps
    # List all containers (including stopped)
    docker ps -a
    # View logs of a specific container
    docker logs -f <container_name_or_id>
    # Access a running container's shell
    docker exec -it <container_name_or_id> bash
    ```

## Key Topics

- `/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/odom` - Odometry feedback (nav_msgs/Odometry)  
- `/joy` - Joystick input
