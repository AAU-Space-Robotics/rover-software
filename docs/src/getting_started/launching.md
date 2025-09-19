# Launching the GORM Rover

The GORM rover can be launched in two modes: **development mode** (for active development) and **production mode** (for deployment). This page covers the development mode. For production deployment, see the [Deployment Guide](../../deployment/overview.md).

## Quick Start with Docker (Development Mode)

1. **Navigate to the Rover Software Directory**: Open a terminal and navigate to the directory where the rover software (on the rover) is located.
    ```bash
    cd /workspace/rover-software/docker
    ```
2. **Run the Docker Container and Attach to It**: Start the Docker container that contains the rover software in development mode and attach to it.
    ```bash
    ./run.sh rover --dev

    # Attach to the running Docker container
    docker exec -it rover-software-dev bash
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

5. **BEFORE YOU START DEVELOPMENT**: Make sure to set up your SSH keys and Git configuration as described in the [Development Setup Guide](../contributing/development-setup.md).

<!-- > **💡 Production Alternative**: For production deployment with automatic restart and pre-built packages, start the deploy image (or use docker-compose).

1. **Run the Production Deployment**: Start the production deployment which runs continuously and automatically restarts if it crashes or the system reboots.
    ```bash
    ./run.sh rover --prod
    # or with docker-compose:
    docker-compose up -d rover-deploy
    ``` -->

## Quick Start (Production Mode)
1. **Navigate to the Rover Software Directory**: Open a terminal and navigate to the directory where the rover software (on the rover) is located.
    ```bash
    cd /workspace/rover-software/docker
    ```
2. **Run the Production Deployment**: Start the production deployment which runs continuously and automatically restarts if it crashes or the system reboots.
    ```bash
    ./run.sh rover --prod
    # or with docker-compose:
    docker-compose up -d rover-deploy
    ```

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
