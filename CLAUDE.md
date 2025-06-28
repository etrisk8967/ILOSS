# CLAUDE.md - AI Assistant Instructions for ILOSS Development

## Overview
This document provides instructions and best practices for AI assistants working on the Integrated Launch and Orbit Simulation System (ILOSS) project. Follow these guidelines to ensure consistent, high-quality development.

## Critical Instructions for Every Task

### 1. Pre-Task Checklist
Before starting ANY task, you MUST:
- [ ] Read `/Documentation/ILOSS_Requirements_v4.md` to understand system requirements
- [ ] Read `/Documentation/ILOSS_Project_Plan.md` to understand the specific task
- [ ] Review any existing code in related directories
- [ ] Check for previous task outputs that this task depends on
- [ ] Understand the task's place in the overall system architecture

### 2. Task Execution Standards

#### Code Quality Requirements
- **C++ Standard**: Use C++20 features appropriately
- **Naming Conventions**: 
  - Classes: `PascalCase`
  - Functions/Methods: `camelCase`
  - Constants: `UPPER_SNAKE_CASE`
  - Member variables: `m_camelCase`
  - Namespaces: `lowercase`
- **Header Guards**: Use `#pragma once`
- **Documentation**: Every public class and method must have Doxygen comments
- **Error Handling**: Use exceptions for errors, return codes for expected failures

#### Implementation Guidelines
- **No Technical Debt**: Every implementation must be production-ready
- **SOLID Principles**: Follow Single Responsibility, Open/Closed, etc.
- **Memory Management**: Use RAII, smart pointers (prefer `std::unique_ptr`)
- **Thread Safety**: Document thread safety guarantees for each class
- **Performance**: Profile before optimizing, document performance characteristics

### 3. Testing Requirements
For EVERY task that creates code:
- [ ] Write comprehensive unit tests using Google Test
- [ ] Achieve minimum 80% code coverage
- [ ] Include edge cases and error conditions
- [ ] Add integration tests where applicable
- [ ] Validate against physics/mathematics where relevant

### 4. Build and Compilation
After implementing any code:
```bash
# Always build from the build directory
cd /home/ethan/projects/SatelliteSimulator/build
cmake ..
make -j$(nproc)
# Run tests
ctest --output-on-failure
```

If compilation fails:
- Fix the root cause, not symptoms
- Ensure fixes work across platforms (Windows/Linux)
- Update CMakeLists.txt if new dependencies are added
- Never commit code that doesn't compile

### 5. Error Resolution Process
When encountering errors:
1. **Understand the Error**: Read the full error message and stack trace
2. **Identify Root Cause**: Don't apply band-aid fixes
3. **Implement Proper Solution**: Follow best practices for the language/framework
4. **Test the Fix**: Ensure it doesn't break existing functionality
5. **Document the Solution**: Add comments explaining non-obvious fixes

### 6. Documentation Updates
After completing each task:
- [ ] Update code documentation (Doxygen comments)
- [ ] Update `/README.md` if user-facing features are added
- [ ] Update `/Documentation/` if architecture changes
- [ ] Create/update design documents for complex components
- [ ] Add usage examples for new APIs

### 7. Task Completion Criteria
A task is ONLY complete when:
- [ ] All deliverables listed in the project plan are implemented
- [ ] Code compiles without warnings on all target platforms
- [ ] All tests pass
- [ ] Code coverage meets requirements (>80%)
- [ ] Documentation is complete
- [ ] Code follows all style guidelines
- [ ] No TODO or FIXME comments remain
- [ ] Performance meets requirements

## Project-Specific Guidelines

### Physics Implementation
- Always validate physics calculations against analytical solutions
- Use SI units internally, provide conversion utilities for display
- Document assumptions and approximations
- Reference academic papers or standards for algorithms

### Visualization with osgEarth
- Follow osgEarth best practices for scene graph management
- Implement LOD (Level of Detail) for performance
- Use osgEarth's built-in features rather than reimplementing
- Profile GPU usage and optimize rendering

### Qt6 UI Development
- Use Qt Designer for complex layouts
- Follow Model-View-Controller pattern
- Ensure UI remains responsive (no blocking operations)
- Support both dark and light themes from the start
- Test on high-DPI displays

### Data Management
- Validate all input data before processing
- Use prepared statements for SQL queries
- Implement proper transaction handling
- Include data versioning from the beginning
- Support backward compatibility

## Common Pitfalls to Avoid

1. **Premature Optimization**: Profile first, optimize only bottlenecks
2. **Global State**: Avoid global variables, use dependency injection
3. **Magic Numbers**: Define all constants with meaningful names
4. **Copy-Paste Code**: Extract common functionality into utilities
5. **Ignoring Edge Cases**: Consider all possible inputs and states
6. **Platform-Specific Code**: Abstract platform differences properly
7. **Resource Leaks**: Always use RAII, test with valgrind/sanitizers
8. **Race Conditions**: Design thread safety from the start

## Development Workflow

### For Each Task:
1. **Read Requirements**: Understand what needs to be built
2. **Design First**: Plan the architecture before coding
3. **Implement Incrementally**: Build and test small pieces
4. **Test Continuously**: Write tests alongside code
5. **Refactor Regularly**: Keep code clean and maintainable
6. **Document Thoroughly**: Future developers (including AI) need context
7. **Review and Validate**: Ensure it meets all requirements

### Git Commit Guidelines
When committing code:
- Use clear, descriptive commit messages
- Reference the task number from the project plan
- Separate logical changes into different commits
- Never commit broken code to main branch
- Include test files in the same commit as implementation

## Performance Requirements
Remember these targets from the SRS:
- Real-time simulation at 0.1s timestep
- 60 FPS for 3D visualization
- Support for 365-day simulations
- Memory usage under 8GB for typical missions

## Quality Metrics
Maintain these standards:
- Code coverage: >80%
- Cyclomatic complexity: <10 per function
- Compilation warnings: 0
- Static analysis issues: 0
- Documentation coverage: 100% of public APIs

## Resources and References

### External Documentation
- [C++20 Standard](https://en.cppreference.com/w/cpp/20)
- [Qt6 Documentation](https://doc.qt.io/qt-6/)
- [osgEarth Documentation](http://docs.osgearth.org/)
- [Google Test Documentation](https://google.github.io/googletest/)
- [CMake Documentation](https://cmake.org/documentation/)

### Physics References
- Vallado, D. A. "Fundamentals of Astrodynamics and Applications"
- Montenbruck, O. & Gill, E. "Satellite Orbits"
- NASA GMAT Documentation
- SPICE Toolkit Documentation

## Final Checklist Before Moving to Next Task

- [ ] Current task fully complete per project plan
- [ ] All code compiles and tests pass
- [ ] Documentation updated
- [ ] No memory leaks or warnings
- [ ] Performance requirements met
- [ ] Code reviewed for best practices
- [ ] Changes committed to git
- [ ] Ready for integration with other modules

---

Remember: Quality over speed. It's better to implement something correctly the first time than to create technical debt that will slow down future development. Each task builds on previous work, so maintaining high standards throughout is critical for project success.