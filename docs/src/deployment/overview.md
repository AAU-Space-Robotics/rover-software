# Deployment Overview

The GORM rover software provides two deployment modes: **development** and **production**.

## Development Mode
- Source code mounted as volumes for live editing
- Manual building inside container
- Interactive development and testing

## Production Mode
- ROS2 Source code copied and pre-built during image creation
- **Runs continuously in the background** with automatic restarts

## Deployment Files

| File | Purpose |
|------|---------|
| `run.sh` | The main script for starting services in different modes (e.g., `rover --dev`, `rover --prod`). |
| `docker-compose.yaml` | Defines all services, e.g. `rover`, `rover-deploy`, `cameras`, etc. |
| `Dockerfile` | Defines the multi-stage build environment for both development and production. |
| `build.sh` | Script to build the production Docker image. |
| `stop.sh` | A simple script to stop all running services defined in `docker-compose.yaml`. |
| `attach.sh` | Helper script to quickly attach to the main `rover` container. |
| `entrypoint.sh` | The entrypoint for the development container. |
| `entrypoint.deploy.sh` | The entrypoint for the production container, handling startup logic. |

## Quick Start

```bash
# Build production image
cd docker/
./build.sh

# Deploy (runs in background continuously)
./run.sh rover --prod
# or using docker compose directly:
# docker compose -f docker/docker-compose.yaml up -d rover-deploy
```

The production deployment will run continuously in the background unless manually stopped.