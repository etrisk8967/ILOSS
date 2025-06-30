/**
 * @file 6dof_satellite_simulation.cpp
 * @brief Example demonstrating 6-DOF satellite dynamics simulation
 * 
 * This example shows how to:
 * - Set up a complete 6-DOF dynamics simulation
 * - Include various force and torque models
 * - Use adaptive integration
 * - Monitor and visualize the results
 */

#include "physics/dynamics/DynamicsEngine.h"
#include "physics/dynamics/DynamicsState.h"
#include "physics/dynamics/SimpleMassProperties.h"
#include "physics/dynamics/TorqueAggregator.h"
#include "physics/dynamics/torques/GravityGradientTorque.h"
#include "physics/forces/ForceAggregator.h"
#include "physics/forces/twobody/TwoBodyForceModel.h"
#include "physics/forces/drag/DragForceModel.h"
#include "physics/forces/srp/SolarRadiationPressureModel.h"
#include "physics/integrators/DynamicsIntegratorAdapter.h"
#include "physics/integrators/GenericRK78Integrator.h"
#include "core/math/MathConstants.h"
#include "core/time/Time.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>

using namespace iloss;
using namespace iloss::physics;
using namespace iloss::physics::dynamics;
using namespace iloss::physics::forces;
using namespace iloss::physics::integrators;
using namespace iloss::math;
using namespace iloss::math::constants;

/**
 * @brief Satellite configuration structure
 */
struct SatelliteConfig {
    // Physical properties
    double mass = 500.0;                    // kg
    double Ixx = 100.0;                     // kg⋅m²
    double Iyy = 300.0;                     // kg⋅m² (elongated shape)
    double Izz = 250.0;                     // kg⋅m²
    Vector3D comOffset{0.05, 0.0, 0.02};    // m (small offset)
    
    // Drag parameters
    double dragCoeff = 2.2;
    double dragArea = 5.0;                  // m²
    
    // SRP parameters
    double srpCoeff = 1.5;
    double srpArea = 20.0;                  // m² (solar panels)
    
    // Initial orbit
    double altitude = 600000.0;             // m (600 km)
    double inclination = 97.8 * DEG_TO_RAD; // Sun-synchronous
    double raan = 0.0;                      // Right ascension
    
    // Initial attitude
    double rollOffset = 5.0 * DEG_TO_RAD;   // rad
    double pitchOffset = -2.0 * DEG_TO_RAD; // rad
    double yawOffset = 0.0;                 // rad
    
    // Initial angular velocity
    Vector3D omega0{0.001, 0.0, 0.002};     // rad/s
};

/**
 * @brief Data logger for simulation results
 */
class SimulationLogger {
private:
    std::ofstream m_file;
    std::vector<double> m_times;
    std::vector<DynamicsState> m_states;
    std::vector<Vector3D> m_forces;
    std::vector<Vector3D> m_torques;
    
public:
    SimulationLogger(const std::string& filename) : m_file(filename) {
        // Write header
        m_file << "Time,X,Y,Z,Vx,Vy,Vz,Qw,Qx,Qy,Qz,Wx,Wy,Wz,"
               << "Fx,Fy,Fz,Tx,Ty,Tz,Altitude,Speed,AngSpeed\n";
    }
    
    void log(const DynamicsState& state, const Vector3D& force, const Vector3D& torque) {
        m_times.push_back(state.getTime());
        m_states.push_back(state);
        m_forces.push_back(force);
        m_torques.push_back(torque);
        
        // Calculate derived quantities
        double altitude = state.getRadius() - EARTH_RADIUS_EQUATORIAL;
        double speed = state.getVelocity().magnitude();
        double angSpeed = state.getAngularVelocity().magnitude();
        
        // Write to file
        m_file << std::fixed << std::setprecision(6)
               << state.getTime() << ","
               << state.getPosition().x() << ","
               << state.getPosition().y() << ","
               << state.getPosition().z() << ","
               << state.getVelocity().x() << ","
               << state.getVelocity().y() << ","
               << state.getVelocity().z() << ","
               << state.getAttitude().w() << ","
               << state.getAttitude().x() << ","
               << state.getAttitude().y() << ","
               << state.getAttitude().z() << ","
               << state.getAngularVelocity().x() << ","
               << state.getAngularVelocity().y() << ","
               << state.getAngularVelocity().z() << ","
               << force.x() << ","
               << force.y() << ","
               << force.z() << ","
               << torque.x() << ","
               << torque.y() << ","
               << torque.z() << ","
               << altitude << ","
               << speed << ","
               << angSpeed << "\n";
    }
    
    void printSummary() const {
        if (m_states.empty()) return;
        
        std::cout << "\n=== Simulation Summary ===\n";
        std::cout << "Duration: " << m_times.back() << " seconds\n";
        std::cout << "Data points: " << m_states.size() << "\n";
        
        // Find min/max altitude
        double minAlt = 1e9, maxAlt = -1e9;
        for (const auto& state : m_states) {
            double alt = state.getRadius() - EARTH_RADIUS_EQUATORIAL;
            minAlt = std::min(minAlt, alt);
            maxAlt = std::max(maxAlt, alt);
        }
        
        std::cout << "Altitude range: " << minAlt/1000.0 << " - " 
                  << maxAlt/1000.0 << " km\n";
        
        // Angular momentum change
        auto massProps = std::make_shared<SimpleMassProperties>(
            m_states[0].getMass(), 100.0, 300.0, 250.0
        );
        Vector3D L0 = m_states[0].getAngularMomentumInertial(*massProps);
        Vector3D Lf = m_states.back().getAngularMomentumInertial(*massProps);
        
        std::cout << "Angular momentum change: " 
                  << (Lf - L0).magnitude() / L0.magnitude() * 100.0 << "%\n";
    }
};

/**
 * @brief Create initial state from orbital elements
 */
DynamicsState createInitialState(const SatelliteConfig& config) {
    // Calculate orbital parameters
    double radius = EARTH_RADIUS_EQUATORIAL + config.altitude;
    double speed = std::sqrt(EARTH_MU / radius);
    
    // Position at ascending node
    Vector3D position(
        radius * std::cos(config.raan),
        radius * std::sin(config.raan),
        0.0
    );
    
    // Velocity perpendicular to position, inclined
    Vector3D velocityDir(
        -std::sin(config.raan) * std::cos(config.inclination),
        std::cos(config.raan) * std::cos(config.inclination),
        std::sin(config.inclination)
    );
    Vector3D velocity = velocityDir * speed;
    
    // Initial attitude (RPY relative to orbital frame)
    Quaternion roll = Quaternion::fromAxisAngle(Vector3D::unitX(), config.rollOffset);
    Quaternion pitch = Quaternion::fromAxisAngle(Vector3D::unitY(), config.pitchOffset);
    Quaternion yaw = Quaternion::fromAxisAngle(Vector3D::unitZ(), config.yawOffset);
    Quaternion attitude = yaw * pitch * roll;
    
    return DynamicsState(
        position,
        velocity,
        config.mass,
        0.0,
        attitude,
        config.omega0
    );
}

/**
 * @brief Main simulation function
 */
void runSimulation() {
    std::cout << "=== 6-DOF Satellite Simulation Example ===\n";
    
    // Configuration
    SatelliteConfig config;
    
    // Create mass properties
    auto massProps = std::make_shared<SimpleMassProperties>(
        config.mass, config.Ixx, config.Iyy, config.Izz, config.comOffset
    );
    
    // Create force aggregator
    auto forceAgg = std::make_shared<ForceAggregator>();
    
    // Add two-body gravity
    std::cout << "Adding two-body gravity...\n";
    forceAgg->addForceModel(
        std::make_unique<twobody::TwoBodyForceModel>(EARTH_MU)
    );
    
    // Add atmospheric drag
    std::cout << "Adding atmospheric drag...\n";
    auto dragModel = std::make_shared<drag::DragForceModel>();
    ForceModelConfig dragConfig;
    dragConfig.setParameter("drag_coefficient", config.dragCoeff);
    dragConfig.setParameter("area", config.dragArea);
    dragModel->initialize(dragConfig);
    forceAgg->addForceModel(
        std::make_unique<AttitudeAwareForceModelAdapter>(dragModel)
    );
    
    // Add solar radiation pressure
    std::cout << "Adding solar radiation pressure...\n";
    auto srpModel = std::make_shared<srp::SolarRadiationPressureModel>();
    ForceModelConfig srpConfig;
    srpConfig.setParameter("reflectivity_coefficient", config.srpCoeff);
    srpConfig.setParameter("area", config.srpArea);
    srpModel->initialize(srpConfig);
    forceAgg->addForceModel(
        std::make_unique<AttitudeAwareForceModelAdapter>(srpModel)
    );
    
    // Create torque aggregator
    auto torqueAgg = std::make_shared<TorqueAggregator>();
    
    // Add gravity gradient torque
    std::cout << "Adding gravity gradient torque...\n";
    torques::GravityGradientConfig ggConfig;
    ggConfig.centralBodyMu = EARTH_MU;
    ggConfig.massProperties = massProps;
    torqueAgg->addModel(
        std::make_unique<torques::GravityGradientTorque>(ggConfig)
    );
    
    // Create dynamics engine
    auto engine = std::make_shared<DynamicsEngine>(
        massProps, forceAgg, torqueAgg
    );
    
    // Create integrator adapter
    auto adapter = std::make_shared<DynamicsIntegratorAdapter>(engine);
    
    // Configure adaptive integrator
    IntegratorConfig intConfig;
    intConfig.initialStepSize = 10.0;        // 10 seconds
    intConfig.minStepSize = 0.1;             // 0.1 seconds
    intConfig.maxStepSize = 60.0;            // 1 minute
    intConfig.relativeTolerance = 1e-10;
    intConfig.absoluteTolerance = 1e-12;
    intConfig.enableStatistics = true;
    
    // Create RK78 integrator
    GenericRK78Integrator<DynamicsState, DynamicsIntegratorAdapter> integrator(intConfig);
    
    // Create initial state
    std::cout << "\nCreating initial state...\n";
    DynamicsState initialState = createInitialState(config);
    
    std::cout << "Initial position: " << initialState.getPosition() << " m\n";
    std::cout << "Initial velocity: " << initialState.getVelocity() << " m/s\n";
    std::cout << "Initial attitude: " << initialState.getAttitude() << "\n";
    std::cout << "Initial angular velocity: " << initialState.getAngularVelocity() << " rad/s\n";
    
    // Calculate orbital period
    double radius = initialState.getRadius();
    double period = 2.0 * M_PI * std::sqrt(std::pow(radius, 3) / EARTH_MU);
    std::cout << "\nOrbital period: " << period / 60.0 << " minutes\n";
    
    // Simulation duration (1 orbit)
    double simulationTime = period;
    
    // Create logger
    SimulationLogger logger("6dof_simulation_output.csv");
    
    // Callback for logging
    size_t logCounter = 0;
    auto callback = [&](const DynamicsState& state, double time) {
        // Log every 10 seconds
        if (logCounter++ % 10 == 0) {
            // Calculate current forces and torques for logging
            auto derivatives = engine->calculateDerivatives(state, time);
            Vector3D force = derivatives.totalForce;
            Vector3D torque = derivatives.totalTorque;
            
            logger.log(state, force, torque);
            
            // Progress update every minute
            if (static_cast<int>(time) % 60 == 0) {
                double progress = time / simulationTime * 100.0;
                std::cout << "\rProgress: " << std::fixed << std::setprecision(1) 
                          << progress << "%" << std::flush;
            }
        }
        return true;  // Continue integration
    };
    
    // Run simulation
    std::cout << "\nRunning simulation for " << simulationTime / 60.0 << " minutes...\n";
    auto startTime = std::chrono::high_resolution_clock::now();
    
    try {
        DynamicsState finalState = integrator.integrate(
            initialState, *adapter, simulationTime, callback
        );
        
        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime);
        
        std::cout << "\n\nSimulation completed in " << duration.count() << " seconds\n";
        
        // Print final state
        std::cout << "\nFinal state:\n";
        std::cout << "Position: " << finalState.getPosition() << " m\n";
        std::cout << "Velocity: " << finalState.getVelocity() << " m/s\n";
        std::cout << "Attitude: " << finalState.getAttitude() << "\n";
        std::cout << "Angular velocity: " << finalState.getAngularVelocity() << " rad/s\n";
        
        // Print integrator statistics
        const auto& stats = integrator.getStatistics();
        std::cout << "\nIntegrator statistics:\n";
        std::cout << "Total steps: " << stats.totalSteps << "\n";
        std::cout << "Accepted steps: " << stats.acceptedSteps << "\n";
        std::cout << "Rejected steps: " << stats.rejectedSteps << "\n";
        std::cout << "Acceptance rate: " << stats.getAcceptanceRate() * 100.0 << "%\n";
        std::cout << "Average step size: " << stats.averageStepSize << " s\n";
        std::cout << "Max error estimate: " << stats.maxEstimatedError << "\n";
        
        // Print simulation summary
        logger.printSummary();
        
        std::cout << "\nResults written to: 6dof_simulation_output.csv\n";
        
    } catch (const std::exception& e) {
        std::cerr << "\nError during simulation: " << e.what() << std::endl;
        return;
    }
}

/**
 * @brief Plot helper function (requires gnuplot)
 */
void generatePlots() {
    std::cout << "\nGenerating plots with gnuplot...\n";
    
    // Create gnuplot script
    std::ofstream script("plot_6dof.gnuplot");
    script << "set datafile separator ','\n";
    script << "set grid\n";
    script << "\n";
    script << "# Altitude vs time\n";
    script << "set terminal png size 800,600\n";
    script << "set output 'altitude.png'\n";
    script << "set xlabel 'Time (s)'\n";
    script << "set ylabel 'Altitude (km)'\n";
    script << "set title '6-DOF Satellite Altitude'\n";
    script << "plot 'h_simulation_output.csv' using 1:($22/1000) with lines title 'Altitude'\n";
    script << "\n";
    script << "# Angular velocity\n";
    script << "set output 'angular_velocity.png'\n";
    script << "set ylabel 'Angular Velocity (rad/s)'\n";
    script << "set title 'Satellite Angular Velocity Components'\n";
    script << "plot 'h_simulation_output.csv' using 1:12 with lines title 'Wx', \\\n";
    script << "     '' using 1:13 with lines title 'Wy', \\\n";
    script << "     '' using 1:14 with lines title 'Wz'\n";
    script << "\n";
    script << "# 3D trajectory\n";
    script << "set terminal png size 800,800\n";
    script << "set output 'trajectory_3d.png'\n";
    script << "set xlabel 'X (km)'\n";
    script << "set ylabel 'Y (km)'\n";
    script << "set zlabel 'Z (km)'\n";
    script << "set title '6-DOF Satellite Trajectory'\n";
    script << "set view equal xyz\n";
    script << "splot 'h_simulation_output.csv' using ($2/1000):($3/1000):($4/1000) with lines notitle\n";
    script.close();
    
    // Execute gnuplot
    system("gnuplot plot_6dof.gnuplot");
    std::cout << "Plots saved as PNG files\n";
}

int main(int argc, char* argv[]) {
    try {
        // Run the simulation
        runSimulation();
        
        // Generate plots if requested
        if (argc > 1 && std::string(argv[1]) == "--plot") {
            generatePlots();
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}