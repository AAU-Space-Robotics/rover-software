# Development vs Production

## Development Setup (`rover` service)

**Use for:** Active development, debugging, testing

- Source code mounted as volumes (`../src:/home/workspace/src`)
- Manual building required (`colcon build`)
- Interactive environment
- Code changes immediately available

**Workflow:**
```bash
./run.sh rover --dev
# or attach to the running container:
docker exec -it rover bash
# inside the container:
colcon build && source install/setup.bash
ros2 launch gorm_bringup bringup_teleop.launch.py
```

## Production Setup (`rover-deploy` service)

**Use for:** Autonomous operation, field deployment, competitions

- Source code copied during image build
- Pre-built workspace (no manual building needed)
- **Runs continuously in background** with `restart: unless-stopped`
- Production-ready and stable

**Workflow:**
```bash
./build.sh
./run.sh rover --prod  # starts the production deploy image in background
# or using docker compose directly:
# docker compose -f docker/docker-compose.yaml up -d rover-deploy
```

## Key Differences

| Aspect | Development | Production |
|--------|-------------|------------|
| **Source Code** | Volume mounted | Copied during build |
| **Building** | Manual | Automatic during image build |
| **Operation** | Manual start/stop | **Continuous background operation** |
| **Updates** | Edit files directly | Rebuild image |
| **Use Case** | Development & testing | Deployment & autonomous operation |

## Deployment Files

| File | Purpose |
|------|---------|
| `run.sh` | The main script for starting services in different modes (e.g., `rover --dev`, `rover --prod`). |
| `docker-compose.yaml` | Defines all services, e.g. `rover`, `rover-deploy`, `cameras`, etc. |
| `Dockerfile` | Defines the multi-stage build environment for both development and production. |
| `build.sh` | Script to build the production Docker image. |
| `stop.sh` | A simple script to stop all running services defined in `docker-compose.yaml`. |
| `attach.sh` | Helper script to quickly attach to the main `rover` container. |
| `entrypoint.sh` | The entrypoint for the containers. |

**Important:** Production deployment runs continuously in the background and will automatically restart if it crashes, unless manually stopped.