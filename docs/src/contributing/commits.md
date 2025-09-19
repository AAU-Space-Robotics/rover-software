# Commit Guidelines

We use **Conventional Commits** to maintain a clear and searchable commit history.

## Commit Message Format

```
<type>[optional scope]: <description>

[optional body]

[optional footer(s)]
```

## Types

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

## Scopes (Optional)

Use scopes to indicate the area of change:

- **`ros2:`** - ROS 2 packages and nodes
- **`docker:`** - Docker configuration and tooling
- **`control:`** - Control system components
- **`perception:`** - Perception system components
- **`teleop:`** - Teleoperation functionality
- **`nav:`** - Navigation stack components

## Examples

```bash
feat(control): add PID controller for motor speed regulation

fix(nav): resolve timeout issue in planner server subscription

docs: update README with new installation instructions

style(ros2): format Python code according to PEP 8

refactor(docker): simplify compose service configuration

chore: update dependencies to latest stable versions
```