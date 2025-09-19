# Development Setup

### Multi-Developer Setup (ORIN System)

When developing on the shared ORIN system (192.168.50.100) with multiple developers, follow these steps:

1. **Create personal workspace and clone:**
   ```bash
   # Replace 'abmo' with your initials/username
   mkdir -p ws/abmo_dev
   cd ws/abmo_dev
   git clone https://github.com/AAU-Space-Robotics/rover-software.git
   cd rover-software
   ```

2. **Configure Git locally for your identity:**
   ```bash
   git config user.name "Your Name"
   git config user.email "your.email@example.com"
   ```

3. **SSH Agent Setup for ORIN (192.168.50.100):**
   
   Add to your SSH config (`~/.ssh/config`):
   ```
    Host 192.168.50.100
        HostName 192.168.50.100
        User gorm
        ForwardAgent yes
        IdentityFile ~/.ssh/id_ed25519
   ```

4. **Setup SSH key evaluation:**
   
   Add to the bottom of your `~/.profile` and `~/.xprofile`:
   ```bash
   eval $(keychain --eval --quiet id_ed25519)
   ```
   _create the files if they don't exist._
5. **Development with Docker (Recommended):**
   ```bash
   cd docker
   ./run.sh rover --dev
   ```

6. **Native development:**
   ```bash
   # Source ROS 2
   source /opt/ros/humble/setup.bash
   
   # Build packages
   colcon build
   source install/setup.bash
   ```

## Testing

### ROS 2 Packages

```bash
# Build and test a specific package
colcon build --packages-select gorm_control
colcon test --packages-select gorm_control
```

### Docker Testing

```bash
# Test in development container
cd docker
./run.sh rover --dev

# Test production build
./build.sh
./run.sh rover --prod
```
