#include <gtest/gtest.h>
#include "physics/aerodynamics/AerodynamicForceModel.h"
#include "physics/aerodynamics/AerodynamicTorqueModel.h"
#include "physics/aerodynamics/AerodynamicDatabase.h"
#include "physics/dynamics/DynamicsState.h"
#include "core/constants/AtmosphericModel.h"
#include "core/math/Vector3D.h"
#include "core/math/Quaternion.h"
#include "core/time/Time.h"
#include <memory>
#include <filesystem>
#include <fstream>

using namespace iloss::physics::aerodynamics;
using namespace iloss::physics::dynamics;
using namespace iloss::constants;
using namespace iloss::math;
using namespace iloss::time;

class AerodynamicsIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Use standard atmosphere model
        atmosphere = std::make_shared<StandardAtmosphere1976>();
        
        // Load test coefficient database
        coeffDatabase = std::make_shared<AerodynamicDatabase>();
        
        // Create test CSV file with realistic data
        createTestCoefficientFile();
        coeffDatabase->loadFromCSV(testCoeffFile);
        
        // Rocket configuration
        rocketConfig.referenceArea = 50.47;      // ~4m diameter rocket
        rocketConfig.referenceLength = 4.0;      // diameter
        rocketConfig.referenceSpan = 4.0;        // same as diameter for axisymmetric
        rocketConfig.centerOfPressureOffset = Vector3D(0.0, 0.0, 0.0);  // At COM initially
        
        torqueConfig.referenceArea = rocketConfig.referenceArea;
        torqueConfig.referenceLength = rocketConfig.referenceLength;
        torqueConfig.referenceSpan = rocketConfig.referenceSpan;
        torqueConfig.centerOfPressure = Vector3D(15.0, 0.0, 0.0);  // 15m from nose
        torqueConfig.centerOfMass = Vector3D(20.0, 0.0, 0.0);      // 20m from nose (stable)
        
        // Create models
        forceModel = std::make_unique<AerodynamicForceModel>(
            atmosphere, coeffDatabase, rocketConfig);
        torqueModel = std::make_unique<AerodynamicTorqueModel>(
            atmosphere, coeffDatabase, torqueConfig);
    }
    
    void TearDown() override {
        // Clean up test file
        if (std::filesystem::exists(testCoeffFile)) {
            std::filesystem::remove(testCoeffFile);
        }
    }
    
    void createTestCoefficientFile() {
        std::ofstream file(testCoeffFile);
        file << R"(# Rocket aerodynamic coefficients
# Mach, AoA_deg, CD, CL, CY, Cl, Cm, Cn
0.0, 0.0, 0.75, 0.0, 0.0, 0.0, 0.0, 0.0
0.0, 2.0, 0.76, 0.08, 0.0, 0.0, -0.002, 0.0
0.0, 5.0, 0.78, 0.20, 0.0, 0.0, -0.005, 0.0
0.0, 10.0, 0.82, 0.40, 0.0, 0.0, -0.010, 0.0
0.5, 0.0, 0.75, 0.0, 0.0, 0.0, 0.0, 0.0
0.5, 2.0, 0.76, 0.08, 0.0, 0.0, -0.002, 0.0
0.5, 5.0, 0.78, 0.20, 0.0, 0.0, -0.005, 0.0
0.5, 10.0, 0.82, 0.40, 0.0, 0.0, -0.010, 0.0
0.9, 0.0, 0.82, 0.0, 0.0, 0.0, 0.0, 0.0
0.9, 2.0, 0.83, 0.08, 0.0, 0.0, -0.002, 0.0
0.9, 5.0, 0.85, 0.20, 0.0, 0.0, -0.005, 0.0
0.9, 10.0, 0.90, 0.40, 0.0, 0.0, -0.010, 0.0
1.2, 0.0, 0.70, 0.0, 0.0, 0.0, 0.0, 0.0
1.2, 2.0, 0.71, 0.06, 0.0, 0.0, -0.0015, 0.0
1.2, 5.0, 0.73, 0.15, 0.0, 0.0, -0.004, 0.0
1.2, 10.0, 0.77, 0.30, 0.0, 0.0, -0.008, 0.0
2.0, 0.0, 0.50, 0.0, 0.0, 0.0, 0.0, 0.0
2.0, 2.0, 0.51, 0.04, 0.0, 0.0, -0.001, 0.0
2.0, 5.0, 0.52, 0.10, 0.0, 0.0, -0.003, 0.0
2.0, 10.0, 0.55, 0.20, 0.0, 0.0, -0.006, 0.0)";
        file.close();
    }
    
    std::shared_ptr<StandardAtmosphere1976> atmosphere;
    std::shared_ptr<AerodynamicDatabase> coeffDatabase;
    std::unique_ptr<AerodynamicForceModel> forceModel;
    std::unique_ptr<AerodynamicTorqueModel> torqueModel;
    AerodynamicConfig rocketConfig;
    AerodynamicTorqueConfig torqueConfig;
    const std::string testCoeffFile = "test_rocket_coeffs.csv";
};

TEST_F(AerodynamicsIntegrationTest, RocketAscentDrag) {
    // Simulate rocket at 10 km altitude, Mach 2
    double altitude = 10000.0;  // 10 km
    double radius = 6371000.0 + altitude;
    
    StateVector stateVec;
    stateVec.setPosition(Vector3D(radius, 0.0, 0.0));
    stateVec.setVelocity(Vector3D(600.0, 0.0, 0.0));  // ~Mach 2 at altitude
    stateVec.setMass(50000.0);  // 50 ton rocket
    
    DynamicsState state(stateVec, Quaternion::identity(), Vector3D::zero());
    Time currentTime;
    
    Vector3D acceleration = forceModel->calculateAccelerationWithAttitude(state, currentTime);
    
    // Should have significant drag deceleration
    EXPECT_LT(acceleration.x(), 0.0);
    
    // Verify against expected values
    double density = atmosphere->getDensity(stateVec.getPosition(), 0.0);
    double q = 0.5 * density * 600.0 * 600.0;
    double dragForce = q * rocketConfig.referenceArea * 0.5;  // CD ~ 0.5 at Mach 2
    double expectedAccel = -dragForce / stateVec.getMass();
    
    EXPECT_NEAR(acceleration.x(), expectedAccel, std::abs(expectedAccel) * 0.3);
}

TEST_F(AerodynamicsIntegrationTest, AngleOfAttackStability) {
    // Test aerodynamic stability with angle of attack
    double altitude = 5000.0;
    double radius = 6371000.0 + altitude;
    
    StateVector stateVec;
    stateVec.setPosition(Vector3D(radius, 0.0, 0.0));
    stateVec.setVelocity(Vector3D(250.0, 0.0, 0.0));  // ~Mach 0.75
    stateVec.setMass(50000.0);
    
    // Create 5 degree angle of attack
    double alpha = 5.0 * M_PI / 180.0;
    Quaternion attitude = Quaternion::fromAxisAngle(Vector3D(0, 1, 0), alpha);
    
    DynamicsState state(stateVec, attitude, Vector3D::zero());
    
    // Calculate forces and torques
    Vector3D forceAccel = forceModel->calculateAccelerationWithAttitude(state, 0.0);
    Vector3D torque = torqueModel->calculateTorque(state, 0.0);
    
    // Should have both drag and lift
    EXPECT_LT(forceAccel.x(), 0.0);  // Drag
    EXPECT_NE(forceAccel.magnitude(), std::abs(forceAccel.x()));  // Not pure drag
    
    // Should have restoring pitch moment (negative for positive alpha)
    EXPECT_LT(torque.y(), 0.0);  // Nose-down moment
}

TEST_F(AerodynamicsIntegrationTest, TransonicEffects) {
    // Test transonic regime where coefficients change rapidly
    double altitude = 11000.0;  // Near tropopause
    double radius = 6371000.0 + altitude;
    
    StateVector stateVec;
    stateVec.setPosition(Vector3D(radius, 0.0, 0.0));
    stateVec.setMass(50000.0);
    
    // Test at different Mach numbers
    std::vector<double> machNumbers = {0.8, 0.9, 1.0, 1.1, 1.2};
    std::vector<double> dragAccels;
    
    for (double mach : machNumbers) {
        double temperature = atmosphere->getTemperature(stateVec.getPosition(), 0.0);
        double soundSpeed = AerodynamicCalculator::calculateSpeedOfSound(temperature);
        double velocity = mach * soundSpeed;
        
        stateVec.setVelocity(Vector3D(velocity, 0.0, 0.0));
        DynamicsState state(stateVec, Quaternion::identity(), Vector3D::zero());
        
        Vector3D accel = forceModel->calculateAccelerationWithAttitude(state, 0.0);
        dragAccels.push_back(std::abs(accel.x()));
    }
    
    // Drag should peak near Mach 1
    auto maxIt = std::max_element(dragAccels.begin(), dragAccels.end());
    int maxIndex = std::distance(dragAccels.begin(), maxIt);
    
    EXPECT_GE(maxIndex, 1);  // Peak should be at Mach 0.9 or higher
    EXPECT_LE(maxIndex, 3);  // Peak should be at Mach 1.1 or lower
}

TEST_F(AerodynamicsIntegrationTest, AltitudeEffects) {
    // Test drag reduction with altitude
    std::vector<double> altitudes = {0.0, 10000.0, 30000.0, 50000.0, 80000.0};
    std::vector<double> dragForces;
    
    StateVector stateVec;
    stateVec.setVelocity(Vector3D(500.0, 0.0, 0.0));  // Constant velocity
    stateVec.setMass(50000.0);
    
    for (double alt : altitudes) {
        double radius = 6371000.0 + alt;
        stateVec.setPosition(Vector3D(radius, 0.0, 0.0));
        
        DynamicsState state(stateVec, Quaternion::identity(), Vector3D::zero());
        
        Vector3D accel = forceModel->calculateAccelerationWithAttitude(state, 0.0);
        dragForces.push_back(std::abs(accel.x()) * stateVec.getMass());
    }
    
    // Drag should decrease with altitude
    for (size_t i = 1; i < dragForces.size(); ++i) {
        EXPECT_LT(dragForces[i], dragForces[i-1]);
    }
    
    // At 80 km, drag should be very small
    EXPECT_LT(dragForces.back(), dragForces.front() * 0.001);
}

TEST_F(AerodynamicsIntegrationTest, CenterOfPressureMovement) {
    // Test how center of pressure affects stability
    double altitude = 5000.0;
    double radius = 6371000.0 + altitude;
    
    StateVector stateVec;
    stateVec.setPosition(Vector3D(radius, 0.0, 0.0));
    stateVec.setVelocity(Vector3D(300.0, 0.0, 0.0));
    stateVec.setMass(50000.0);
    
    double alpha = 5.0 * M_PI / 180.0;
    Quaternion attitude = Quaternion::fromAxisAngle(Vector3D(0, 1, 0), alpha);
    DynamicsState state(stateVec, attitude, Vector3D::zero());
    
    // Test 1: COP ahead of COM (stable)
    Vector3D torqueStable = torqueModel->calculateTorque(state, 0.0);
    EXPECT_LT(torqueStable.y(), 0.0);  // Restoring moment
    
    // Test 2: COP behind COM (unstable)
    AerodynamicTorqueConfig unstableConfig = torqueConfig;
    unstableConfig.centerOfPressure = Vector3D(25.0, 0.0, 0.0);  // Behind COM
    torqueModel->setConfig(unstableConfig);
    
    Vector3D torqueUnstable = torqueModel->calculateTorque(state, 0.0);
    EXPECT_GT(torqueUnstable.y(), 0.0);  // Destabilizing moment
}

TEST_F(AerodynamicsIntegrationTest, MaxQConditions) {
    // Find maximum dynamic pressure during ascent
    double maxQ = 0.0;
    double maxQAltitude = 0.0;
    double maxQVelocity = 0.0;
    
    StateVector stateVec;
    stateVec.setMass(50000.0);
    
    // Simulate ascent profile
    for (double alt = 0.0; alt <= 50000.0; alt += 1000.0) {
        double radius = 6371000.0 + alt;
        stateVec.setPosition(Vector3D(radius, 0.0, 0.0));
        
        // Simple velocity profile (increases then decreases)
        double velocity;
        if (alt < 10000.0) {
            velocity = 100.0 + alt * 0.04;  // Accelerating
        } else {
            velocity = 500.0 + (alt - 10000.0) * 0.02;  // Still accelerating but less
        }
        
        stateVec.setVelocity(Vector3D(velocity, 0.0, 0.0));
        
        // Get flow properties
        DynamicsState state(stateVec, Quaternion::identity(), Vector3D::zero());
        forceModel->calculateAccelerationWithAttitude(state, 0.0);
        
        auto flowProps = forceModel->getLastFlowProperties();
        if (flowProps && flowProps->dynamicPressure > maxQ) {
            maxQ = flowProps->dynamicPressure;
            maxQAltitude = alt;
            maxQVelocity = velocity;
        }
    }
    
    // Max Q typically occurs between 10-20 km
    EXPECT_GE(maxQAltitude, 8000.0);
    EXPECT_LE(maxQAltitude, 25000.0);
    EXPECT_GT(maxQ, 10000.0);  // Should be significant
}

TEST_F(AerodynamicsIntegrationTest, DatabaseInterpolation) {
    // Test smooth interpolation between data points
    double altitude = 10000.0;
    double radius = 6371000.0 + altitude;
    
    StateVector stateVec;
    stateVec.setPosition(Vector3D(radius, 0.0, 0.0));
    stateVec.setMass(50000.0);
    
    // Test at intermediate Mach numbers
    std::vector<double> testMachs = {0.25, 0.7, 0.95, 1.1, 1.6};
    
    for (double mach : testMachs) {
        double temperature = atmosphere->getTemperature(stateVec.getPosition(), 0.0);
        double soundSpeed = AerodynamicCalculator::calculateSpeedOfSound(temperature);
        double velocity = mach * soundSpeed;
        
        stateVec.setVelocity(Vector3D(velocity, 0.0, 0.0));
        
        // Test at intermediate angle
        double alpha = 3.5 * M_PI / 180.0;  // Between data points
        Quaternion attitude = Quaternion::fromAxisAngle(Vector3D(0, 1, 0), alpha);
        
        DynamicsState state(stateVec, attitude, Vector3D::zero());
        
        // Should not throw and should give reasonable values
        EXPECT_NO_THROW({
            Vector3D accel = forceModel->calculateAccelerationWithAttitude(state, 0.0);
            EXPECT_LT(accel.x(), 0.0);  // Always have drag
            EXPECT_GT(accel.magnitude(), 0.0);  // Non-zero force
        });
    }
}

TEST_F(AerodynamicsIntegrationTest, RealisticRocketFlight) {
    // Simulate a more realistic rocket trajectory segment
    double dt = 0.1;  // 100ms timestep
    
    // Initial conditions at 5 km altitude
    StateVector state;
    state.setPosition(Vector3D(6376000.0, 0.0, 0.0));
    state.setVelocity(Vector3D(400.0, 100.0, 0.0));  // Some vertical velocity
    state.setMass(45000.0);  // Lost some propellant
    
    // Small angle of attack due to trajectory
    double alpha = 2.0 * M_PI / 180.0;
    Quaternion attitude = Quaternion::fromAxisAngle(Vector3D(0, 1, 0), alpha);
    Vector3D angularVel(0.0, 0.0, 0.0);
    
    // Simulate 10 seconds
    for (int i = 0; i < 100; ++i) {
        DynamicsState dynState(state, attitude, angularVel);
        
        // Get accelerations
        Vector3D aeroAccel = forceModel->calculateAccelerationWithAttitude(dynState, 0.0);
        Vector3D torque = torqueModel->calculateTorque(dynState, 0.0);
        
        // Simple integration (just for testing)
        Vector3D vel = state.getVelocity();
        vel = vel + aeroAccel * dt;
        state.setVelocity(vel);
        
        Vector3D pos = state.getPosition();
        pos = pos + vel * dt;
        state.setPosition(pos);
        
        // Angular integration (simplified)
        double inertia = 100000.0;  // kg⋅m²
        Vector3D angAccel = torque / inertia;
        angularVel = angularVel + angAccel * dt;
        
        // Update attitude (simplified)
        if (angularVel.magnitude() > 0) {
            double angle = angularVel.magnitude() * dt;
            Vector3D axis = angularVel.normalized();
            Quaternion rotation = Quaternion::fromAxisAngle(axis, angle);
            attitude = rotation * attitude;
            attitude.normalize();
        }
    }
    
    // After 10 seconds, should have:
    // - Reduced velocity due to drag
    // - Changed altitude
    // - Possibly changed attitude due to aerodynamic moments
    
    EXPECT_LT(state.getVelocity().magnitude(), 500.0);  // Lost some speed
    EXPECT_GT(state.getPosition().magnitude(), 6376000.0);  // Gained altitude
}