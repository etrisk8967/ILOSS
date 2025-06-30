# 6-DOF Dynamics Integration Guide

## Overview

The ILOSS physics engine now supports full 6 degree-of-freedom (6-DOF) dynamics simulation, allowing accurate modeling of both translational motion (position and velocity) and rotational motion (attitude and angular velocity) of spacecraft and other objects. This guide explains the new architecture and provides examples of how to use the 6-DOF capabilities.

## Key Components

### 1. State Representations

#### StateVector (3-DOF)
The basic `StateVector` class represents translational state only:
- Position (3D vector)
- Velocity (3D vector)
- Mass (scalar)
- Time

#### DynamicsState (6-DOF)
The `DynamicsState` class extends `StateVector` to include rotational state:
- All StateVector components
- Attitude (quaternion)
- Angular velocity (3D vector in body frame)

### 2. Generic Integrators

The new template-based integrator framework allows numerical integration of any state type that satisfies the `IntegrableState` concept.

#### Available Integrators
- `GenericRK4Integrator<State, System>` - Fixed-step 4th-order Runge-Kutta
- `GenericRK78Integrator<State, System>` - Adaptive 7(8) order Runge-Kutta-Fehlberg

#### IntegrableState Concept
Any state type must provide:
- Addition operator (`operator+`)
- Scalar multiplication (`operator*`)
- Time management (`getTime()`, `setTime()`)
- Dimension query (`getDimension()`)
- Vector conversion (`toVector()`, `fromVector()`)
- Error norm calculation (`errorNorm()`)

### 3. Dynamics Engine

The `DynamicsEngine` class manages 6-DOF dynamics:
- Calculates forces using `ForceAggregator`
- Calculates torques using `TorqueAggregator`
- Implements Euler's equations for rigid body rotation
- Handles coupling between translational and rotational motion

### 4. Attitude-Aware Force Models

Force models can now use attitude information when available:
- `IAttitudeAwareForceModel` interface extends `ForceModel`
- Provides `calculateAccelerationWithAttitude()` method
- Updated models: `DragForceModel`, `SolarRadiationPressureModel`

## Usage Examples

### Example 1: Basic 6-DOF Integration

```cpp
#include "physics/dynamics/DynamicsEngine.h"
#include "physics/dynamics/DynamicsState.h"
#include "physics/dynamics/SimpleMassProperties.h"
#include "physics/integrators/DynamicsIntegratorAdapter.h"
#include "physics/integrators/GenericRK4Integrator.h"

// Create spacecraft mass properties
auto massProps = std::make_shared<SimpleMassProperties>(
    100.0,    // mass (kg)
    10.0,     // Ixx (kg⋅m²)
    15.0,     // Iyy (kg⋅m²)
    20.0      // Izz (kg⋅m²)
);

// Create force and torque aggregators
auto forceAgg = std::make_shared<ForceAggregator>();
auto torqueAgg = std::make_shared<TorqueAggregator>();

// Add force models (e.g., gravity)
forceAgg->addForceModel(std::make_unique<TwoBodyForceModel>(EARTH_MU));

// Create dynamics engine
auto engine = std::make_shared<DynamicsEngine>(massProps, forceAgg, torqueAgg);

// Create integrator adapter
auto adapter = std::make_shared<DynamicsIntegratorAdapter>(engine);

// Configure integrator
IntegratorConfig config;
config.initialStepSize = 1.0;  // 1 second

// Create generic RK4 integrator
GenericRK4Integrator<DynamicsState, DynamicsIntegratorAdapter> integrator(config);

// Set initial state
DynamicsState initialState(
    Vector3D(7000000.0, 0.0, 0.0),     // position (m)
    Vector3D(0.0, 7500.0, 0.0),        // velocity (m/s)
    100.0,                             // mass (kg)
    0.0,                               // time (s)
    Quaternion::identity(),            // attitude
    Vector3D(0.0, 0.0, 0.1)           // angular velocity (rad/s)
);

// Integrate for 100 seconds
DynamicsState finalState = integrator.integrate(
    initialState, *adapter, 100.0
);
```

### Example 2: Attitude-Aware Drag

```cpp
// Create attitude-aware drag model
auto dragModel = std::make_shared<DragForceModel>();
ForceModelConfig dragConfig;
dragConfig.setParameter("drag_coefficient", 2.2);
dragConfig.setParameter("area", 5.0);  // Reference area in m²
dragModel->initialize(dragConfig);

// Add to force aggregator using adapter
forceAgg->addForceModel(
    std::make_unique<AttitudeAwareForceModelAdapter>(dragModel)
);

// The drag model will now use attitude information when available
// to calculate the actual cross-sectional area
```

### Example 3: Gravity Gradient Stabilization

```cpp
// Add gravity gradient torque for passive stabilization
torques::GravityGradientConfig ggConfig;
ggConfig.centralBodyMu = EARTH_MU;
ggConfig.massProperties = massProps;

torqueAgg->addModel(
    std::make_unique<GravityGradientTorque>(ggConfig)
);

// Spacecraft will naturally align with the local vertical
// due to gravity gradient torques
```

### Example 4: Adaptive Integration with Error Control

```cpp
// Configure adaptive integrator
IntegratorConfig adaptiveConfig;
adaptiveConfig.initialStepSize = 10.0;
adaptiveConfig.minStepSize = 0.1;
adaptiveConfig.maxStepSize = 100.0;
adaptiveConfig.relativeTolerance = 1e-10;
adaptiveConfig.absoluteTolerance = 1e-12;

// Create RK78 adaptive integrator
GenericRK78Integrator<DynamicsState, DynamicsIntegratorAdapter> adaptiveIntegrator(adaptiveConfig);

// Integrate with automatic step size control
DynamicsState finalState = adaptiveIntegrator.integrate(
    initialState, *adapter, targetTime
);

// Check integration statistics
const auto& stats = adaptiveIntegrator.getStatistics();
std::cout << "Total steps: " << stats.totalSteps << std::endl;
std::cout << "Rejected steps: " << stats.rejectedSteps << std::endl;
std::cout << "Max error: " << stats.maxEstimatedError << std::endl;
```

## Advanced Topics

### Custom Area Models

You can extend the drag and SRP models to implement custom area calculations:

```cpp
class FlatPlateDragModel : public DragForceModel {
protected:
    double calculateAttitudeDependentArea(
        const Vector3D& velocityDirection,
        const Quaternion& attitude) const override {
        
        // For a flat plate, area = A_ref * |cos(θ)|
        // where θ is angle between plate normal and velocity
        Vector3D plateNormal(0, 0, 1);  // Assume Z is normal
        double cosTheta = std::abs(velocityDirection.dot(plateNormal));
        return getReferenceArea() * cosTheta;
    }
};
```

### Coupled Dynamics

When the center of mass is offset from the center of pressure/thrust, forces create torques:

```cpp
// Create mass properties with CoM offset
auto massProps = std::make_shared<SimpleMassProperties>(
    500.0,                          // mass
    100.0, 200.0, 150.0,           // moments of inertia
    Vector3D(0.1, 0.0, 0.05)       // CoM offset (m)
);

// Forces applied at geometric center will create torques
// This coupling is handled automatically by DynamicsEngine
```

### Integration with Existing Code

The new system maintains backward compatibility:

```cpp
// Existing 3-DOF code continues to work
StateVector state3DOF = ...;
Vector3D acceleration = forceModel->calculateAcceleration(state3DOF, time);

// Attitude-aware models provide default behavior when attitude is unavailable
// They assume reference area and nominal orientation
```

## Performance Considerations

### Step Size Selection
- For circular orbits: step size ~ 1% of orbital period
- With gravity gradient: step size ~ 0.1% of libration period
- With fast tumbling: step size ~ 1% of rotation period

### Adaptive vs Fixed-Step
- Use RK4 for real-time applications with predictable dynamics
- Use RK78 for high accuracy or when dynamics vary significantly
- Monitor rejected steps in RK78 - high rejection rate indicates stiff dynamics

### Memory Usage
- DynamicsState is larger than StateVector (13 vs 7 doubles)
- Generic integrators may have slightly higher overhead than specialized versions
- RK78 requires more memory for intermediate calculations

## Validation and Testing

The 6-DOF implementation includes comprehensive tests:

1. **Conservation Laws**: Angular momentum and energy conservation in torque-free motion
2. **Gravity Gradient**: Libration dynamics and stability
3. **Coupled Dynamics**: Force-torque coupling with CoM offset
4. **Attitude-Dependent Forces**: Drag and SRP with varying orientation
5. **Numerical Accuracy**: Comparison of integrator performance

Run the test suite:
```bash
./test_6DOF_Integration
./test_CoupledDynamicsIntegration
```

## Migration Guide

To upgrade existing 3-DOF simulations to 6-DOF:

1. Replace `StateVector` with `DynamicsState`
2. Add mass properties (`SimpleMassProperties` or custom)
3. Replace `ForceAggregator` integration with `DynamicsEngine`
4. Use `DynamicsIntegratorAdapter` with generic integrators
5. Optionally add torque models for rotational dynamics
6. Update force models to attitude-aware versions if needed

## References

1. Vallado, D. A. "Fundamentals of Astrodynamics and Applications"
2. Hughes, P. C. "Spacecraft Attitude Dynamics"
3. Wertz, J. R. "Spacecraft Attitude Determination and Control"
4. Montenbruck, O. & Gill, E. "Satellite Orbits"

## API Reference

See the Doxygen documentation for detailed API information:
- `physics/dynamics/DynamicsState.h`
- `physics/dynamics/DynamicsEngine.h`
- `physics/integrators/GenericIntegrator.h`
- `physics/forces/IAttitudeAwareForceModel.h`