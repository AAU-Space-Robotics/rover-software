# Pull Request Process

## Before Creating a PR

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

## PR Requirements

- **Link the related issue:** Use `Fixes #<issue-number>` or `Closes #<issue-number>`
- **Clear title:** Use conventional commit format for PR title
- **Detailed description:** Explain what, why, and how
- **Test coverage:** Include test results or explain why tests aren't needed
- **Breaking changes:** Clearly document any breaking changes

## PR Template

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

## Review Process

1. **Automated Checks:** All PRs must pass automated linting and build checks
2. **Code Review:** At least one maintainer review required
3. **Testing:** Verify changes work in relevant environments
4. **Documentation:** Ensure documentation is updated if needed