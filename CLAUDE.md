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

## Task 25: Propulsion System Modeling - Implementation Instructions

### Overview
Task 25 implements a comprehensive propulsion system that integrates with the existing 6-DOF dynamics engine. The implementation must support thrust forces, mass depletion, specific impulse handling, thrust vector control, and multi-engine configurations.

### Key Requirements
1. **Integration with Existing Systems**:
   - Must implement `IAttitudeAwareForceModel` for thrust force calculations
   - Must implement `IMassProperties` for time-varying mass properties
   - Must work seamlessly with `ForceAggregator` and `DynamicsEngine`

2. **Physical Accuracy**:
   - Use Tsiolkovsky rocket equation for mass flow: ṁ = F/(g₀·Isp)
   - Model atmospheric back-pressure effects: F = F_vac - p_amb·A_exit
   - Transform thrust vectors correctly from body to inertial frame

3. **Production-Ready Features**:
   - Thread-safe implementation for parallel simulations
   - Comprehensive error handling and validation
   - Support for multiple engines and propellant tanks
   - Configurable via JSON/ForceModelConfig
   - Full Doxygen documentation

### Implementation Order
1. **PropulsionConstants.h**: Define g₀ = 9.80665 m/s² and other constants
2. **PropellantTank**: Simple class to track propellant mass and type
3. **Engine**: Core engine model with thrust curves and Isp
4. **ThrustVectorControl**: Gimbal modeling for thrust direction
5. **PropulsionSystem**: Manages engines/tanks, implements IMassProperties
6. **ThrustForceModel**: Main force model implementing IAttitudeAwareForceModel

### Critical Implementation Details

#### Engine Class Must Support:
```cpp
class Engine {
    // Required parameters
    double m_vacuumThrust;      // N
    double m_vacuumIsp;         // s
    double m_seaLevelThrust;    // N (optional)
    double m_seaLevelIsp;       // s (optional)
    double m_minThrottle;       // 0-1
    double m_maxThrottle;       // 0-1
    double m_nozzleExitArea;    // m²
    
    // Methods
    double getThrust(double pressure, double throttle) const;
    double getIsp(double pressure) const;
    double getMassFlowRate(double pressure, double throttle) const;
};
```

#### PropulsionSystem Must:
- Track total propellant mass across all tanks
- Update center of mass as propellant depletes
- Update inertia tensor for mass changes
- Return time-varying mass properties
- Handle propellant depletion (engine shutdown when empty)

#### ThrustForceModel Must:
- Query current throttle and gimbal settings from config
- Calculate thrust force in body frame
- Transform to inertial frame using spacecraft attitude
- Return acceleration (F/m) not force
- Handle both StateVector and DynamicsState inputs

### Testing Requirements
Create tests in `/tests/physics/propulsion/` covering:
1. **test_Engine.cpp**: Thrust curves, Isp variations, mass flow
2. **test_PropellantTank.cpp**: Mass tracking, depletion
3. **test_ThrustVectorControl.cpp**: Gimbal limits and transformations
4. **test_PropulsionSystem.cpp**: Mass properties, multi-engine coordination
5. **test_ThrustForceModel.cpp**: Force calculations, frame transformations
6. **test_PropulsionIntegration.cpp**: Full system integration test

### Common Pitfalls to Avoid
1. **Frame Confusion**: Always be clear about body vs inertial frames
2. **Mass Updates**: Ensure mass depletion is properly integrated
3. **Unit Errors**: Use SI units consistently (N, kg, m, s)
4. **Throttle Limits**: Respect min/max throttle constraints
5. **Empty Tanks**: Handle propellant depletion gracefully

### Validation Benchmarks
Compare against:
- Falcon 9 first stage: 7,607 kN thrust, 311s Isp (vacuum)
- Delta-V calculations using Tsiolkovsky equation
- Known burn durations for standard missions

### Example Configuration
```json
{
    "engines": [{
        "name": "Merlin-1D",
        "vacuum_thrust": 7607000.0,
        "vacuum_isp": 311.0,
        "sea_level_thrust": 6806000.0,
        "sea_level_isp": 282.0,
        "min_throttle": 0.4,
        "max_throttle": 1.0
    }],
    "tanks": [{
        "name": "Main Tank",
        "propellant_type": "RP1/LOX",
        "propellant_mass": 418000.0
    }],
    "throttle": 1.0,
    "gimbal_pitch": 0.0,
    "gimbal_yaw": 0.0
}
```

## Task 26: Aerodynamic Force Model - Implementation Instructions

### Overview
Task 26 implements comprehensive aerodynamic forces and torques for atmospheric flight simulation. The implementation supports coefficient-based aerodynamics with Mach number and angle of attack dependencies, integrating seamlessly with the 6-DOF dynamics system.

### Key Requirements
1. **Core Components**:
   - **AerodynamicCoefficients**: Data structures for CD, CL, CY, Cl, Cm, Cn
   - **AerodynamicDatabase**: CSV loading and 2D interpolation (Mach, AoA)
   - **AerodynamicCalculator**: Flow properties, Mach number, AoA calculations
   - **AerodynamicForceModel**: IAttitudeAwareForceModel implementation
   - **AerodynamicTorqueModel**: ITorqueModel implementation

2. **Physical Accuracy**:
   - Proper frame transformations (body, wind, stability, inertial)
   - Dynamic pressure calculation: q = 0.5 * ρ * V²
   - Force calculation: F = q * S * C
   - Moment calculation: M = q * S * L * C
   - Center of pressure effects for stability

3. **Production Features**:
   - Thread-safe coefficient database with caching
   - CSV import/export for coefficient data
   - Bilinear interpolation with extrapolation handling
   - Support for multiple vehicle configurations
   - Extended coefficients with stability derivatives

### Implementation Details

#### CSV Database Format:
```csv
# Mach, AoA_deg, CD, CL, CY, Cl, Cm, Cn
0.0, 0.0, 0.30, 0.00, 0.0, 0.00, 0.00, 0.0
0.0, 5.0, 0.32, 0.16, 0.0, 0.00, -0.01, 0.0
...
```

#### Key Classes:
1. **AerodynamicCoefficients**:
   - Stores force coefficients (CD, CL, CY)
   - Stores moment coefficients (Cl, Cm, Cn)
   - Supports interpolation and scaling
   - Extended version includes stability derivatives

2. **AerodynamicDatabase**:
   - Loads CSV files with coefficient data
   - 2D interpolation (Mach number, angle of attack)
   - Caching for performance
   - Multiple configuration support

3. **AerodynamicCalculator**:
   - Static methods for flow calculations
   - Mach number, dynamic pressure, Reynolds number
   - Angle of attack and sideslip calculations
   - Frame transformation matrices

4. **AerodynamicForceModel**:
   - Calculates aerodynamic forces
   - Integrates with atmosphere models
   - Transforms forces to inertial frame
   - Dynamic pressure limiting option

5. **AerodynamicTorqueModel**:
   - Calculates aerodynamic moments
   - Center of pressure effects
   - Stability analysis support

### Frame Conventions
- **Body Frame**: X-forward, Y-right, Z-down
- **Wind Frame**: X-along velocity, Y-right, Z-down
- **Stability Frame**: Rotated by AoA only

### Testing Coverage
All components include comprehensive unit tests:
- Coefficient validation and interpolation
- Database loading and caching
- Flow property calculations
- Force and torque computations
- Integration tests with realistic scenarios

### Common Use Cases
1. **Launch Vehicle Ascent**: High drag at transonic speeds
2. **Reentry Simulation**: High angle of attack aerodynamics
3. **Aircraft Simulation**: Full 6-DOF with stability derivatives
4. **Stability Analysis**: Center of pressure vs center of mass

### Integration with ILOSS
- Uses existing `AtmosphericModel` for density/temperature
- Compatible with `ForceAggregator` and `DynamicsEngine`
- Works with 6-DOF state propagation
- Supports both 3-DOF and 6-DOF simulations