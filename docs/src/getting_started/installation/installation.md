# Installation Guide
Here is a step-by-step guide to install the ROS2 Navigation Stack on the Rover. This is mainly in the context of reinstalling the software on the rover itself after a fresh OS installation, and is not necessary for development on your own machine.

## Hardware Requirements
- **Computer**: A Nvidia Jetson Orin with Jetpack 6.1.
- **Sensors**: 2x Zed 2i Cameras,
## Prerequisites
1. **Docker**
2. **Docker Compose**
3. **Nvidia Container Toolkit**
4. **Resolve Joy Driver Issues**: If you encounter issues with the joystick driver, you can resolve them by opening a terminal and running the following commands: see [Joy Driver Issues](https://forums.developer.nvidia.com/t/logitech-f710-kernel-module-issues-jetpack-6/296904/21).
     ```bash
     sudo git clone https://github.com/paroj/xpad.git /usr/src/xpad-0.4
     sudo dkms install -m xpad -v 0.4
     ```

## Installation Steps
1. **Connect to the Rover**: SSH into the rover's computer using the following command:
   ```bash
   ssh gorm@192.168.0.100 # or ssh gorm@<ROVER_IP_ADDRESS>
   ```
   The default password is `gorm`.

2. **Clone the Repository**: Clone the AAU Rover Software repository to the rover's computer.
   ```bash
    git clone https://github.com/aaU-Space-Robotics/rover-software/
    cd rover-software
    ```
3. **Build & Run the Docker Images**: Navigate to the `docker` directory and start the main rover service in development mode (this will build images and mount the source code).
    ```bash
    cd docker
    ./run.sh rover --dev
    ```

    > **💡 Production Deployment**: For production deployment with auto-restart capabilities, see the [Deployment Guide](../../deployment/overview.md) which provides a pre-built image that doesn't require manual building inside the container.

4. **Launch the Rover Software**: After the Docker images are built and running, you can attach to the running container and launch the rover software.
    ```bash
    # Attach to the running Docker container
    docker exec -it rover-software bash

    # Build the ROS2 packages inside the container
    colcon build
    source install/setup.bash

    # Launch the rover software
    ros2 launch gorm_bringup bringup_teleop.launch.py
    ```