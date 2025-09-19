# Contributing Overview

Welcome to the GORM rover project! This guide will help you understand our development practices and contribution process to ensure smooth collaboration.

## 🚨 Important Rules

**NEVER commit directly to `main` or `dev` branches.** All changes must go through the Pull Request process.

## Key Points for Contributors

- **System releases** use Git tags with format `vMAJOR.MINOR.PATCH` (e.g., `v1.0.0`)
- **ROS 2 packages** version independently using `MAJOR.MINOR.PATCH` format in their `package.xml`
- All changes must be made through Pull Requests
- Follow our branching strategy and commit conventions
- Test your changes before submitting PRs
- Update documentation as needed

## Version Updates

When making changes that affect versioning:

1. **Breaking changes**: Increment MAJOR version
2. **New features**: Increment MINOR version  
3. **Bug fixes**: Increment PATCH version

Always update `package.xml` when changing ROS 2 package versions.

## Getting Help

- **Issues:** Use GitHub Issues for bugs and feature requests
- **Discussions:** Use GitHub Discussions for questions and ideas
- **Maintainers:** Tag `@AAU-Space-Robotics` maintainers for urgent matters

---

**Remember:** Never commit directly to `main` or `dev`. Always use Pull Requests for code changes.