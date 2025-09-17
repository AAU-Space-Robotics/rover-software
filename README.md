# AAU Space Robotics - System Setup Repository
This repository contains the files and documentation for running the GORM rover software on the physical rover or in simulation.

Documentation site: https://aau-space-robotics.github.io/aau-rover/

## Quick Start (Development)

Use development mode when actively editing source and testing interactively. The `rover` service mounts your workspace for live editing.

```bash
# from the repository root
cd docker/
./run.sh rover --dev          # start the development container (builds and mounts source)

# attach to the running container
docker exec -it rover bash

# inside the container (build and launch)
colcon build
source install/setup.bash
ros2 launch gorm_bringup bringup_teleop.launch.py
```

Notes:
- Container name: `rover` (development). Changes to source files on the host are visible inside the container.
- Use `./stop.sh` or `docker compose -f docker/docker-compose.yaml down` to stop services.

## Quick Start (Production)

For autonomous deployments, use the production image. The `rover-deploy` service is pre-built and runs continuously.

```bash
cd docker/
./build.sh                   # build the production image (if necessary)
./run.sh rover --prod        # start the production deploy image (background)

# view logs
docker compose -f docker/docker-compose.yaml logs -f rover-deploy

# access running container
docker compose -f docker/docker-compose.yaml exec rover-deploy bash
```

Notes:
- Container name: `rover-deploy` (production).
- Production containers are configured to restart automatically unless stopped.

## Useful commands

- Start development: `./run.sh rover --dev`
- Start production: `./run.sh rover --prod`
- Build production image: `./build.sh`
- Attach to container: `docker exec -it rover bash` (dev) or `docker compose -f docker/docker-compose.yaml exec rover-deploy bash` (prod)
- Tail logs: `docker compose -f docker/docker-compose.yaml logs -f <service>`

## SSH access to the rover

To access the rover directly (outside Docker), SSH to the rover's IP. The username and IP may vary; example below is for an onboard user account commonly used in our docs:

```bash
ssh gorm@192.168.0.100 # Default IP for the rover is 192.168.0.100
```

Confirm the correct username and password for your hardware image before connecting.

## Simulation

Simulation assets and detailed instructions are not available yet — coming soon. Check the documentation site for updates and the `docs/` directory later for example scenarios and launch files.

---

For more detailed guides, see the `docs/` directory or visit the documentation site linked above.
