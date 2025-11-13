# Contributing to SLAM Bot

## Code Style Guide

### C++ Guidelines

- Follow the [ROS2 C++ Style Guide](https://docs.ros.org/en/rolling/Contributing/Code-Style-Language-Versions.html)
- Use modern C++ features (C++17)
- Comment all public APIs using Doxygen format
- Keep functions focused and under 50 lines where possible

### Python Guidelines

- Follow [PEP 8](https://www.python.org/dev/peps/pep-0008/)
- Use type hints for function parameters
- Document all modules and public functions

### ROS2 Best Practices

- Use standard message types when possible
- Follow the [ROS2 Developer Guide](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html)
- Write launch files in Python for flexibility

## Git Workflow

1. Fork the repository
2. Create a feature branch

```bash
git checkout -b feature/your-feature-name
```

3. Make your changes
4. Run tests

```bash
colcon test
```

5. Commit with clear messages

```bash
git commit -m "feat: Add new feature X

Detailed description of changes and why they were made.
Fixes #issue_number"
```

6. Push and create a Pull Request

## Testing

### Required Tests

- Unit tests for all new functionality
- Integration tests for node interactions
- Launch file tests
- Python tests using pytest
- C++ tests using gtest

### Running Tests

```bash
# Run all tests
colcon test

# Run specific package tests
colcon test --packages-select package_name

# Run with coverage
colcon test --coverage
```

## Documentation

### Required Documentation

- API documentation for all public interfaces
- Usage examples in README files
- Launch file documentation
- Parameter documentation
- Topic/service documentation

### Building Documentation

```bash
# Generate Doxygen documentation
cd docs && doxygen

# Build and view locally
cd docs && make html
python -m http.server 8000
```

## Review Process

1. Automated checks must pass:

   - Build success
   - All tests passing
   - Code coverage requirements
   - Style guide compliance

2. Code Review Requirements:

   - Clear commit messages
   - Documentation updated
   - Tests included
   - No breaking changes without discussion

3. Review Checklist:
   - [ ] Code follows style guide
   - [ ] Tests are included and pass
   - [ ] Documentation is updated
   - [ ] Commit messages are clear
   - [ ] Changes are appropriate in scope

## Release Process

1. Version numbers follow [SemVer](https://semver.org/)
2. Update CHANGELOG.md
3. Create tagged releases
4. Include release notes

## Questions?

- Open an issue for discussion
