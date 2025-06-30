#include <iostream>
#include "physics/dynamics/DynamicsState.h"
#include "physics/dynamics/DynamicsEngine.h"
#include "physics/dynamics/SimpleMassProperties.h"
#include "physics/dynamics/TorqueAggregator.h"
#include "physics/forces/ForceAggregator.h"
#include "physics/integrators/DynamicsIntegratorAdapter.h"
#include "core/math/MathConstants.h"

using namespace iloss::physics;
using namespace iloss::physics::dynamics;
using namespace iloss::physics::forces;
using namespace iloss::physics::integrators;
using namespace iloss::math;
using namespace iloss::math::constants;

int main() {
    std::cout << "Testing quaternion derivative computation...\n";
    
    // Create mass properties
    auto massProperties = std::make_shared<SimpleMassProperties>(100.0, 10.0, 15.0, 20.0);
    
    // Create empty force and torque aggregators
    auto forceAggregator = std::make_shared<ForceAggregator>();
    auto torqueAggregator = std::make_shared<TorqueAggregator>();
    
    // Create dynamics engine
    auto dynamicsEngine = std::make_shared<DynamicsEngine>(
        massProperties, forceAggregator, torqueAggregator
    );
    
    // Create integrator adapter
    DynamicsIntegratorAdapter adapter(dynamicsEngine);
    
    // Create initial state with zero angular velocity
    Vector3D position(7000000.0, 0.0, 0.0);  // 7000 km radius
    Vector3D velocity(0.0, 7546.0, 0.0);     // Circular velocity
    Quaternion attitude = Quaternion::identity();
    Vector3D angularVelocity(0.0, 0.0, 0.0); // Zero angular velocity
    
    DynamicsState state(position, velocity, 100.0, 0.0, 
                       attitude, angularVelocity);
    
    std::cout << "Initial state:\n";
    std::cout << "  Attitude: " << attitude.w() << ", " << attitude.x() 
              << ", " << attitude.y() << ", " << attitude.z() << "\n";
    std::cout << "  Angular velocity: " << angularVelocity.x() << ", " 
              << angularVelocity.y() << ", " << angularVelocity.z() << "\n";
    
    try {
        // Compute derivative
        std::cout << "\nComputing derivative...\n";
        DynamicsState derivative = adapter.computeDerivative(state);
        
        std::cout << "Derivative attitude quaternion: " 
                  << derivative.getAttitude().w() << ", " 
                  << derivative.getAttitude().x() << ", " 
                  << derivative.getAttitude().y() << ", " 
                  << derivative.getAttitude().z() << "\n";
        std::cout << "Derivative attitude norm: " 
                  << derivative.getAttitude().norm() << "\n";
        
        // Try scaling the derivative
        std::cout << "\nScaling derivative by 1.0...\n";
        DynamicsState scaled = derivative * 1.0;
        std::cout << "Success!\n";
        
        // Try scaling by 0.5
        std::cout << "\nScaling derivative by 0.5...\n";
        DynamicsState scaled2 = derivative * 0.5;
        std::cout << "Success!\n";
        
        // Try adding to original state
        std::cout << "\nAdding scaled derivative to original state...\n";
        DynamicsState newState = state + scaled2;
        std::cout << "Success!\n";
        std::cout << "New attitude: " << newState.getAttitude().w() << ", " 
                  << newState.getAttitude().x() << ", " 
                  << newState.getAttitude().y() << ", " 
                  << newState.getAttitude().z() << "\n";
        
    } catch (const std::exception& e) {
        std::cout << "ERROR: " << e.what() << "\n";
        return 1;
    }
    
    std::cout << "\nAll tests passed!\n";
    return 0;
}