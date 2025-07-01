/**
 * @file propulsion_example.cpp
 * @brief Example demonstrating the ILOSS propulsion system
 * 
 * This example shows how to:
 * - Create a propulsion system with engines and tanks
 * - Configure thrust vector control
 * - Integrate propulsion with the force aggregator
 * - Simulate a launch with mass depletion
 */

#include <iostream>
#include <iomanip>
#include <memory>

// Propulsion components
#include "physics/forces/propulsion/PropulsionSystem.h"
#include "physics/forces/propulsion/Engine.h"
#include "physics/forces/propulsion/PropellantTank.h"
#include "physics/forces/propulsion/ThrustVectorControl.h"
#include "physics/forces/propulsion/ThrustForceModel.h"

// Physics components
#include "physics/forces/ForceAggregator.h"
#include "physics/forces/twobody/TwoBodyForceModel.h"
#include "physics/dynamics/DynamicsEngine.h"
#include "physics/integrators/GenericRK4Integrator.h"
#include "physics/integrators/DynamicsIntegratorAdapter.h"

// Core components
#include "core/time/Time.h"
#include "core/math/Vector3D.h"
#include "core/math/Quaternion.h"
#include "core/constants/PhysicalConstants.h"
#include "core/logging/Logger.h"

using namespace iloss;
using namespace iloss::physics;
using namespace iloss::physics::propulsion;
using namespace iloss::physics::forces;
using namespace iloss::physics::dynamics;
using namespace iloss::physics::integrators;
using namespace iloss::math;
using namespace iloss::time;
using namespace iloss::constants;
using namespace iloss::logging;

/**
 * @brief Create a Falcon 9-like first stage propulsion system
 */
std::shared_ptr<PropulsionSystem> createFalcon9FirstStage() {
    // Configure propulsion system with vehicle dry mass
    PropulsionSystem::Configuration propConfig;
    propConfig.dryMass = 25600.0;  // kg (approximate Falcon 9 first stage dry mass)
    propConfig.dryCenterOfMass = Vector3D(0, 0, 20.0);  // 20m from base
    
    // Approximate inertia tensor for a cylinder
    double radius = 1.85;  // m
    double length = 42.0;  // m
    double Ixx = propConfig.dryMass * (3 * radius * radius + length * length) / 12.0;
    double Iyy = Ixx;
    double Izz = propConfig.dryMass * radius * radius / 2.0;
    propConfig.dryInertiaTensor = Matrix3D::identity();
    propConfig.dryInertiaTensor(0, 0) = Ixx;
    propConfig.dryInertiaTensor(1, 1) = Iyy;
    propConfig.dryInertiaTensor(2, 2) = Izz;
    
    auto propulsion = std::make_shared<PropulsionSystem>(propConfig);
    
    // Add propellant tank
    auto mainTank = std::make_shared<PropellantTank>(
        "MainTank",                 // name
        PropellantType::RP1_LOX,    // type
        411000.0,                   // max capacity (kg)
        411000.0                    // current mass (kg) - full tank
    );
    propulsion->addTank(mainTank);
    
    // Configure Merlin 1D engine
    Engine::Configuration engineConfig;
    engineConfig.name = "Merlin-1D";
    engineConfig.vacuumThrust = 914000.0;    // N (per engine)
    engineConfig.vacuumIsp = 311.0;          // s
    engineConfig.seaLevelThrust = 845000.0;  // N
    engineConfig.seaLevelIsp = 282.0;        // s
    engineConfig.minThrottle = 0.4;          // 40% minimum
    engineConfig.maxThrottle = 1.0;          // 100%
    engineConfig.nozzleExitArea = 1.0;       // m² (approximate)
    engineConfig.gimbalRange = 0.0872665;    // 5 degrees
    
    // Add 9 engines in octaweb configuration
    for (int i = 0; i < 9; i++) {
        auto engine = std::make_shared<Engine>(engineConfig);
        engine->start(1.0);  // Start at full throttle
        
        // Calculate engine position (8 in circle + 1 center)
        PropulsionSystem::EngineMounting mounting;
        mounting.engineId = "Merlin-" + std::to_string(i + 1);
        
        if (i < 8) {
            // Outer ring
            double angle = i * 2.0 * M_PI / 8.0;
            mounting.position = Vector3D(
                1.5 * std::cos(angle),   // 1.5m radius
                1.5 * std::sin(angle),
                0.0                      // At base of stage
            );
        } else {
            // Center engine
            mounting.position = Vector3D(0, 0, 0);
        }
        
        mounting.orientation = Vector3D(0, 0, -1);  // Thrust along -Z
        mounting.assignedTanks = {"MainTank"};
        
        // Add TVC for all engines
        auto tvc = std::make_shared<ThrustVectorControl>();
        mounting.tvc = tvc;
        
        propulsion->addEngine(engine, mounting);
    }
    
    return propulsion;
}

/**
 * @brief Simulate a launch with propulsion
 */
void simulateLaunch() {
    std::cout << "\n=== ILOSS Propulsion System Example ===" << std::endl;
    std::cout << "Simulating Falcon 9-like first stage burn\n" << std::endl;
    
    // Set up logging
    Logger::getInstance().setLogLevel(LogLevel::Info);
    
    // Create propulsion system
    auto propulsion = createFalcon9FirstStage();
    std::cout << "Created propulsion system: " << propulsion->toString() << std::endl;
    
    // Create initial state (on launch pad at Cape Canaveral)
    double lat = 28.396837 * DEG_TO_RAD;
    double lon = -80.605659 * DEG_TO_RAD;
    double alt = 0.0;  // Sea level
    
    Vector3D position = EarthModel::geodeticToECEF(lat, lon, alt);
    Vector3D velocity = Vector3D::zero();  // Initially at rest
    
    Time epoch(2025, 1, 1, 0, 0, 0.0);
    StateVector initialState(position, velocity, propulsion->getMass(), epoch);
    
    // Create dynamics state with vertical attitude
    Quaternion attitude = Quaternion::identity();  // No rotation
    Vector3D angularVelocity = Vector3D::zero();
    DynamicsState state(initialState, attitude, angularVelocity);
    
    // Create force models
    auto forceAggregator = std::make_shared<ForceAggregator>();
    
    // Add two-body gravity
    auto gravity = std::make_shared<TwoBodyForceModel>("Earth Gravity");
    forceAggregator->addForceModel(gravity);
    
    // Add thrust
    auto thrustModel = std::make_shared<ThrustForceModel>("Thrust", propulsion);
    forceAggregator->addForceModel(thrustModel);
    
    // Create dynamics engine
    auto massProps = std::dynamic_pointer_cast<IMassProperties>(propulsion);
    DynamicsEngine dynamics(forceAggregator, nullptr, massProps);
    
    // Create integrator
    DynamicsIntegratorAdapter adapter(dynamics);
    GenericRK4Integrator<DynamicsState> integrator(adapter);
    
    // Simulation parameters
    double timeStep = 1.0;     // 1 second steps
    double printInterval = 10.0;  // Print every 10 seconds
    double maxTime = 162.0;    // Typical first stage burn time
    
    // Run simulation
    double lastPrintTime = 0.0;
    std::cout << "\nTime(s)  Alt(km)  Vel(m/s)  Mass(kg)  Thrust(kN)  Isp(s)" << std::endl;
    std::cout << "-------  -------  --------  --------  ----------  ------" << std::endl;
    
    while (state.getTime().getTime() <= maxTime) {
        // Get current telemetry
        double currentTime = state.getTime().getTime();
        Vector3D pos = state.getPosition();
        double lat_deg, lon_deg, altitude;
        EarthModel::ecefToGeodetic(pos, lat_deg, lon_deg, altitude);
        
        double velocity_mag = state.getVelocity().magnitude();
        double mass = propulsion->getMass();
        double thrust = thrustModel->getTotalThrust();
        double isp = propulsion->getEffectiveIsp();
        
        // Print telemetry
        if (currentTime - lastPrintTime >= printInterval || currentTime == 0.0) {
            std::cout << std::fixed << std::setprecision(0) << std::setw(7) << currentTime
                     << std::setprecision(1) << std::setw(9) << altitude / 1000.0
                     << std::setprecision(0) << std::setw(10) << velocity_mag
                     << std::setw(10) << mass
                     << std::setw(12) << thrust / 1000.0
                     << std::setw(8) << isp
                     << std::endl;
            lastPrintTime = currentTime;
        }
        
        // Check for propellant depletion
        if (!propulsion->canOperate()) {
            std::cout << "\nPropellant depleted at T+" << currentTime << " seconds" << std::endl;
            break;
        }
        
        // Integrate to next time
        Time nextTime = state.getTime() + timeStep;
        state = integrator.integrate(state, state.getTime(), nextTime);
    }
    
    // Print final statistics
    std::cout << "\n=== Final Statistics ===" << std::endl;
    std::cout << "Burn duration: " << state.getTime().getTime() << " seconds" << std::endl;
    std::cout << "Final altitude: " << std::fixed << std::setprecision(1);
    
    Vector3D finalPos = state.getPosition();
    double lat_f, lon_f, alt_f;
    EarthModel::ecefToGeodetic(finalPos, lat_f, lon_f, alt_f);
    std::cout << alt_f / 1000.0 << " km" << std::endl;
    
    std::cout << "Final velocity: " << std::setprecision(0) 
              << state.getVelocity().magnitude() << " m/s" << std::endl;
    std::cout << "Propellant consumed: " 
              << 411000.0 - propulsion->getSystemState().totalPropellantMass << " kg" << std::endl;
    std::cout << "Remaining delta-V: " << std::setprecision(0)
              << propulsion->getRemainingDeltaV() << " m/s" << std::endl;
    
    // Demonstrate gimbal control
    std::cout << "\n=== Demonstrating Thrust Vector Control ===" << std::endl;
    
    // Command gimbal angles
    std::cout << "Commanding 2° pitch and 1° yaw on center engine..." << std::endl;
    thrustModel->setEngineGimbal("Merlin-9", 2.0 * DEG_TO_RAD, 1.0 * DEG_TO_RAD);
    
    // Update TVC
    propulsion->update(0.1, 0.0);
    
    // Calculate torque
    Vector3D torque = propulsion->getTotalTorque(0.0);
    std::cout << "Resulting torque: " << torque.toString() << " N·m" << std::endl;
    std::cout << "Torque magnitude: " << std::setprecision(0) 
              << torque.magnitude() << " N·m" << std::endl;
}

int main() {
    try {
        simulateLaunch();
        std::cout << "\nExample completed successfully!" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}