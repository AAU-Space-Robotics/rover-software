# Branching Strategy

We follow a structured branching approach to maintain code quality and enable parallel development:

## Branch Types

- **`main`** - Production-ready code. Protected branch.
- **`dev`** - Integration branch for features. Protected branch.
- **`feature/`** - New features and enhancements
- **`bugfix/`** - Bug fixes for existing functionality
- **`hotfix/`** - Critical fixes that need immediate deployment
- **`release/`** - Release preparation and versioning

## Branch Naming Convention

Use descriptive names with the appropriate prefix:

```
feature/add-navigation-controller
feature/improve-motor-control
bugfix/fix-odometry-calculation
bugfix/resolve-docker-build-issue
hotfix/critical-localization-failure
release/v1.2.0
```

## Workflow

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