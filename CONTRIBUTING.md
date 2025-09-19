# Contributing to the GORM Rover Project

Welcome to the GORM rover project! This guide will help you understand our development practices and contribution process to ensure smooth collaboration.

## 🚨 Important Rules

**NEVER commit directly to `main` or `dev` branches.** All changes must go through the Pull Request process.

## Branching Strategy

We follow a structured branching approach to maintain code quality and enable parallel development:

### Branch Types

- **`main`** - Production-ready code. Protected branch.
- **`dev`** - Integration branch for features. Protected branch.
- **`feature/`** - New features and enhancements
- **`bugfix/`** - Bug fixes for existing functionality
- **`hotfix/`** - Critical fixes that need immediate deployment
- **`release/`** - Release preparation and versioning

### Branch Naming Convention

Use descriptive names with the appropriate prefix:

```
feature/add-navigation-controller
feature/improve-motor-control
bugfix/fix-odometry-calculation
bugfix/resolve-docker-build-issue
hotfix/critical-localization-failure
release/v1.2.0
```

### Workflow

1. **Create feature branch from `dev`:**
   ```bash
   git checkout dev
   git pull origin dev
   git checkout -b feature/your-feature-name
   ```

2. **Work on your changes and commit regularly**

3. **Push your branch and create a Pull Request to `dev`**

4. **After review and approval, merge to `dev`**

5. **For releases, create a release branch from `dev` and merge to `main`**

## Versioning

This project uses a structured versioning approach.

### Key Points for Contributors

- **System releases** use Git tags with format `vMAJOR.MINOR.PATCH` (e.g., `v1.0.0`)
- **ROS 2 packages** version independently using `MAJOR.MINOR.PATCH` format in their `package.xml`.

### Version Updates

When making changes that affect versioning:

1. **Breaking changes**: Increment MAJOR version
2. **New features**: Increment MINOR version  
3. **Bug fixes**: Increment PATCH version

Always update `package.xml` when changing ROS 2 package versions.

## Commit Style and Format

We use **Conventional Commits** to maintain a clear and searchable commit history.

### Commit Message Format

```
<type>[optional scope]: <description>

[optional body]

[optional footer(s)]
```

### Types

- **`feat:`** - A new feature
- **`fix:`** - A bug fix
- **`docs:`** - Documentation changes
- **`style:`** - Code style changes (formatting, no logic changes)
- **`refactor:`** - Code refactoring without changing functionality
- **`perf:`** - Performance improvements
- **`test:`** - Adding or updating tests
- **`chore:`** - Maintenance tasks, dependency updates
- **`ci:`** - CI/CD configuration changes
- **`build:`** - Build system or external dependency changes

### Scopes (Optional)

Use scopes to indicate the area of change:

- **`ros2:`** - ROS 2 packages and nodes
- **`docker:`** - Docker configuration and tooling
- **`control:`** - Control system components
- **`perception:`** - Perception system components
- **`teleop:`** - Teleoperation functionality
- **`nav:`** - Navigation stack components

### Examples

```bash
feat(control): add PID controller for motor speed regulation

fix(nav): resolve timeout issue in planner server subscription

docs: update README with new installation instructions

style(ros2): format Python code according to PEP 8

refactor(docker): simplify compose service configuration

chore: update dependencies to latest stable versions
```

## Pull Request Process

### Before Creating a PR

1. **Ensure your branch is up to date:**
   ```bash
   git checkout dev
   git pull origin dev
   git checkout your-branch
   git rebase dev  # or merge dev into your branch
   ```

2. **Test your changes:**
   ```bash
   # For ROS 2 packages
   colcon build --packages-select <package_name>
   colcon test --packages-select <package_name>
   ```

### PR Requirements

- **Link the related issue:** Use `Fixes #<issue-number>` or `Closes #<issue-number>`
- **Clear title:** Use conventional commit format for PR title
- **Detailed description:** Explain what, why, and how
- **Test coverage:** Include test results or explain why tests aren't needed
- **Breaking changes:** Clearly document any breaking changes

### PR Template

```markdown
## Description
Brief description of changes made.

## Type of Change
- [ ] Bug fix (non-breaking change which fixes an issue)
- [ ] New feature (non-breaking change which adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to not work as expected)
- [ ] Documentation update

## Related Issues
Fixes #<issue-number>

## Testing
- [ ] I have run the existing tests
- [ ] I have added tests for my changes
- [ ] All tests pass locally

## Checklist
- [ ] My code follows the project's style guidelines
- [ ] I have performed a self-review of my code
- [ ] I have commented my code, particularly in hard-to-understand areas
- [ ] I have updated documentation as needed
- [ ] My changes generate no new warnings
- [ ] I have tested this change in the appropriate environment (Docker/hardware)

## Additional Notes
Any additional information, deployment notes, or concerns.
```

### Review Process

1. **Automated Checks:** All PRs must pass automated linting and build checks
2. **Code Review:** At least one maintainer review required
3. **Testing:** Verify changes work in relevant environments
4. **Documentation:** Ensure documentation is updated if needed

## Development Setup

### Prerequisites

- **Docker & Docker Compose:** For containerized development
- **ROS 2 Humble:** If building natively
- **Python 3.8+:** For ROS 2 packages

### Quick Start

1. **Clone the repository:**
   ```bash
   git clone https://github.com/AAU-Space-Robotics/rover-software.git
   cd rover-software
   ```

2. **Development with Docker (Recommended):**
   ```bash
   cd docker
   ./run.sh rover --dev
   ```

3. **Native development:**
   ```bash
   # Source ROS 2
   source /opt/ros/humble/setup.bash
   
   # Build packages
   colcon build
   source install/setup.bash
   ```

### Code Style Guidelines

#### Python (ROS 2 Packages)

- Follow **PEP 8** style guide
- Use **type hints** where appropriate
- Maximum line length: **120 characters**
- Use **docstrings** for classes and functions
- Import order: standard library, third-party, local imports

Example:
```python
#!/usr/bin/env python3
from typing import Optional

import rclpy
from rclpy.node import Node

class MyNode(Node):
    """Brief description of the node's purpose."""
    
    def __init__(self) -> None:
        super().__init__('my_node')
        self.get_logger().info("Node initialized")
```

#### C++

- Follow **Google C++ Style Guide**
- Use **camelCase** for variables and functions
- Use **PascalCase** for classes and structs
- Maximum line length: **100 characters**
- Use meaningful variable names

### Testing

#### ROS 2 Packages

```bash
# Build and test a specific package
colcon build --packages-select gorm_base_control
colcon test --packages-select gorm_base_control

# Run linting
colcon test --packages-select gorm_base_control --event-handlers console_direct+
```

#### Docker Testing

```bash
# Test in development container
cd docker
./run.sh rover --dev

# Test production build
./build.sh
./run.sh rover --prod
```

### Documentation

- Update **README.md** for significant changes
- Add **docstrings/comments** for complex logic
- Update **package.xml** descriptions when needed
- Include **examples** in documentation

## Hardware and Environment

### Development Environment

- **Primary:** Docker containers for development and deployment.
- **Testing:** Simulation and hardware-in-the-loop.
- **Dependencies:** ROS 2 Humble.

### Hardware Setup

See individual component READMEs:
- `docker/README.md` - Container environment
- `src/rover/*/README.md` - Package-specific setup

## Additional Resources

- [Conventional Commits](https://www.conventionalcommits.org/)
- [ROS 2 Style Guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- [Git Best Practices](https://git-scm.com/book/en/v2/Distributed-Git-Contributing-to-a-Project)
- [Docker Best Practices](https://docs.docker.com/develop/dev-best-practices/)

## Getting Help

- **Issues:** Use GitHub Issues for bugs and feature requests
- **Discussions:** Use GitHub Discussions for questions and ideas
- **Maintainers:** Tag `@AAU-Space-Robotics` maintainers for urgent matters

---

**Remember:** Never commit directly to `main` or `dev`. Always use Pull Requests for code changes.
