#include <gtest/gtest.h>
#include "physics/aerodynamics/AerodynamicTorqueModel.h"
#include "physics/aerodynamics/AerodynamicDatabase.h"
#include "physics/dynamics/DynamicsState.h"
#include "core/constants/AtmosphericModel.h"
#include "core/math/Vector3D.h"
#include "core/math/Quaternion.h"
#include <memory>
#include <any>
#include <cmath>

using namespace iloss::physics::aerodynamics;
using namespace iloss::physics::dynamics;
using namespace iloss::constants;
using namespace iloss::math;

// Mock atmosphere model for testing
class MockAtmosphereModel : public AtmosphericModel {
public:
    double testDensity = 1.225;      // kg/m³
    double testTemperature = 288.15;  // K
    double testPressure = 101325.0;   // Pa
    
    double getDensity(const Vector3D& /*position*/, double /*julianDate*/) const override {
        return testDensity;
    }
    
    double getTemperature(const Vector3D& /*position*/, double /*julianDate*/) const override {
        return testTemperature;
    }
    
    double getPressure(const Vector3D& /*position*/, double /*julianDate*/) const override {
        return testPressure;
    }
    
    std::string getName() const override {
        return "Mock Atmosphere";
    }
    
    bool isValidAltitude(double /*altitude*/) const override {
        return true;
    }
    
    double getMinAltitude() const override { return 0.0; }
    double getMaxAltitude() const override { return 1000000.0; }
};

class AerodynamicTorqueModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create mock atmosphere
        mockAtmosphere = std::make_shared<MockAtmosphereModel>();
        
        // Create coefficient database with test data including moments
        coeffDatabase = std::make_shared<AerodynamicDatabase>();
        std::string testData = R"(# Test data with moments
# Mach, AoA_deg, CD, CL, CY, Cl, Cm, Cn
0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0
0.0, 5.0, 0.32, 0.16, 0.0, 0.0, -0.01, 0.0
0.0, 0.0, 0.3, 0.0, 0.05, 0.01, 0.0, -0.005
1.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0
1.0, 5.0, 0.42, 0.19, 0.0, 0.0, -0.015, 0.0)";
        coeffDatabase->loadFromString(testData);
        
        // Default configuration
        defaultConfig.referenceArea = 10.0;       // m²
        defaultConfig.referenceLength = 2.0;      // m
        defaultConfig.referenceSpan = 5.0;        // m
        defaultConfig.centerOfPressure = Vector3D(1.0, 0.0, 0.0);  // 1m from origin
        defaultConfig.centerOfMass = Vector3D(0.5, 0.0, 0.0);      // 0.5m from origin
        
        // Create torque model
        torqueModel = std::make_unique<AerodynamicTorqueModel>(
            mockAtmosphere, coeffDatabase, defaultConfig);
    }
    
    std::shared_ptr<MockAtmosphereModel> mockAtmosphere;
    std::shared_ptr<AerodynamicDatabase> coeffDatabase;
    AerodynamicTorqueConfig defaultConfig;
    std::unique_ptr<AerodynamicTorqueModel> torqueModel;
};

TEST_F(AerodynamicTorqueModelTest, Constructor) {
    EXPECT_NO_THROW(AerodynamicTorqueModel(mockAtmosphere, coeffDatabase));
    EXPECT_NO_THROW(AerodynamicTorqueModel(mockAtmosphere, coeffDatabase, defaultConfig));
    
    // Should throw with null atmosphere
    EXPECT_THROW(AerodynamicTorqueModel(nullptr, coeffDatabase), std::invalid_argument);
    
    // Should throw with null database
    EXPECT_THROW(AerodynamicTorqueModel(mockAtmosphere, nullptr), std::invalid_argument);
}

TEST_F(AerodynamicTorqueModelTest, BasicProperties) {
    EXPECT_EQ(torqueModel->getType(), TorqueModelType::Aerodynamic);
    EXPECT_EQ(torqueModel->getName(), "AerodynamicTorque");
    EXPECT_TRUE(torqueModel->isEnabled());
    
    torqueModel->setEnabled(false);
    EXPECT_FALSE(torqueModel->isEnabled());
}

TEST_F(AerodynamicTorqueModelTest, ZeroVelocityNoTorque) {
    // Create state with zero velocity
    StateVector stateVec;
    stateVec.setPosition(Vector3D(7000000.0, 0.0, 0.0));
    stateVec.setVelocity(Vector3D(0.0, 0.0, 0.0));
    stateVec.setMass(1000.0);
    
    DynamicsState state(stateVec, Quaternion::identity(), Vector3D::zero());
    
    Vector3D torque = torqueModel->calculateTorque(state, 0.0);
    
    // Should be zero torque with zero velocity
    EXPECT_NEAR(torque.magnitude(), 0.0, 1e-10);
}

TEST_F(AerodynamicTorqueModelTest, PureMomentCoefficients) {
    // Create state with forward velocity
    StateVector stateVec;
    stateVec.setPosition(Vector3D(6471000.0, 0.0, 0.0));
    stateVec.setVelocity(Vector3D(340.0, 0.0, 0.0));  // Mach 1 at sea level
    stateVec.setMass(1000.0);
    
    // Small angle of attack to get non-zero Cm
    double alpha = 5.0 * M_PI / 180.0;
    Quaternion attitude = Quaternion::fromAxisAngle(Vector3D(0, 1, 0), alpha);
    
    DynamicsState state(stateVec, attitude, Vector3D::zero());
    
    Vector3D torque = torqueModel->calculateTorque(state, 0.0);
    
    // Should have pitch moment (negative Cm means nose-down)
    EXPECT_NE(torque.y(), 0.0);
    
    // Calculate expected moment
    double q = 0.5 * mockAtmosphere->testDensity * 340.0 * 340.0;
    double expectedMoment = q * defaultConfig.referenceArea * defaultConfig.referenceLength * (-0.015);
    
    EXPECT_NEAR(torque.y(), expectedMoment, std::abs(expectedMoment) * 0.2);  // 20% tolerance
}

TEST_F(AerodynamicTorqueModelTest, ForceOffsetTorque) {
    // Create state with pure drag (no moments from coefficients)
    StateVector stateVec;
    stateVec.setPosition(Vector3D(6471000.0, 0.0, 0.0));
    stateVec.setVelocity(Vector3D(1000.0, 0.0, 0.0));
    stateVec.setMass(1000.0);
    
    DynamicsState state(stateVec, Quaternion::identity(), Vector3D::zero());
    
    Vector3D torque = torqueModel->calculateTorque(state, 0.0);
    
    // With COP ahead of COM and drag force, should create pitch-up moment
    // Drag acts at COP (1.0m), COM is at 0.5m, so moment arm is 0.5m
    double q = 0.5 * mockAtmosphere->testDensity * 1000.0 * 1000.0;
    double dragForce = q * defaultConfig.referenceArea * 0.3;  // CD = 0.3
    double expectedTorque = 0.5 * dragForce;  // r × F
    
    // Drag in -X creates positive pitch (nose up) when COP is ahead of COM
    EXPECT_GT(torque.y(), 0.0);
    EXPECT_NEAR(torque.y(), expectedTorque, expectedTorque * 0.2);
}

TEST_F(AerodynamicTorqueModelTest, DisabledReturnsZero) {
    torqueModel->setEnabled(false);
    
    StateVector stateVec;
    stateVec.setPosition(Vector3D(6471000.0, 0.0, 0.0));
    stateVec.setVelocity(Vector3D(1000.0, 0.0, 0.0));
    stateVec.setMass(1000.0);
    
    DynamicsState state(stateVec, Quaternion::identity(), Vector3D::zero());
    
    Vector3D torque = torqueModel->calculateTorque(state, 0.0);
    
    EXPECT_DOUBLE_EQ(torque.magnitude(), 0.0);
}

TEST_F(AerodynamicTorqueModelTest, MinimumDensityThreshold) {
    // Set atmosphere to below threshold
    mockAtmosphere->testDensity = 1e-13;
    
    StateVector stateVec;
    stateVec.setPosition(Vector3D(6871000.0, 0.0, 0.0));
    stateVec.setVelocity(Vector3D(7000.0, 0.0, 0.0));
    stateVec.setMass(1000.0);
    
    DynamicsState state(stateVec, Quaternion::identity(), Vector3D::zero());
    
    Vector3D torque = torqueModel->calculateTorque(state, 0.0);
    
    // Should be zero due to density threshold
    EXPECT_DOUBLE_EQ(torque.magnitude(), 0.0);
}

TEST_F(AerodynamicTorqueModelTest, Configuration) {
    AerodynamicTorqueConfig newConfig;
    newConfig.referenceArea = 20.0;
    newConfig.referenceLength = 4.0;
    newConfig.referenceSpan = 10.0;
    newConfig.centerOfPressure = Vector3D(2.0, 0.0, 0.0);
    newConfig.centerOfMass = Vector3D(1.0, 0.0, 0.0);
    newConfig.coefficientConfig = "default";
    newConfig.minimumDensity = 1e-10;
    newConfig.useDynamicCenterOfPressure = true;
    
    torqueModel->configure(std::any(newConfig));
    
    const auto& config = torqueModel->getConfig();
    EXPECT_DOUBLE_EQ(config.referenceArea, 20.0);
    EXPECT_DOUBLE_EQ(config.referenceLength, 4.0);
    EXPECT_DOUBLE_EQ(config.referenceSpan, 10.0);
    EXPECT_TRUE(config.useDynamicCenterOfPressure);
}

TEST_F(AerodynamicTorqueModelTest, InvalidConfiguration) {
    // Wrong type should throw
    EXPECT_THROW(torqueModel->configure(std::any(42)), std::invalid_argument);
    
    // Invalid config values should throw
    AerodynamicTorqueConfig badConfig = defaultConfig;
    badConfig.referenceArea = -1.0;
    EXPECT_THROW(torqueModel->setConfig(badConfig), std::invalid_argument);
}

TEST_F(AerodynamicTorqueModelTest, Clone) {
    auto cloned = torqueModel->clone();
    ASSERT_NE(cloned, nullptr);
    
    // Should have same properties
    EXPECT_EQ(cloned->getType(), TorqueModelType::Aerodynamic);
    EXPECT_EQ(cloned->getName(), "AerodynamicTorque");
    EXPECT_EQ(cloned->isEnabled(), torqueModel->isEnabled());
}

TEST_F(AerodynamicTorqueModelTest, CalculateCenterOfPressure) {
    double cm = -0.05;
    double cn = 0.5;
    double refPoint = 0.25;  // Quarter chord
    
    double xcp = torqueModel->calculateCenterOfPressure(cm, cn, refPoint);
    
    // Expected: Xcp = 0.25 * L - (-0.05 * L) / 0.5 = 0.25L + 0.1L = 0.35L
    double expected = (refPoint + 0.1) * defaultConfig.referenceLength;
    EXPECT_NEAR(xcp, expected, 1e-6);
    
    // Test with zero normal force
    xcp = torqueModel->calculateCenterOfPressure(cm, 0.0, refPoint);
    EXPECT_DOUBLE_EQ(xcp, refPoint * defaultConfig.referenceLength);
}

TEST_F(AerodynamicTorqueModelTest, DynamicCenterOfPressure) {
    // Enable dynamic COP
    AerodynamicTorqueConfig config = defaultConfig;
    config.useDynamicCenterOfPressure = true;
    torqueModel->setConfig(config);
    
    // Create state with angle of attack for non-zero CL and CM
    StateVector stateVec;
    stateVec.setPosition(Vector3D(6471000.0, 0.0, 0.0));
    stateVec.setVelocity(Vector3D(340.0, 0.0, 0.0));  // Mach 1
    stateVec.setMass(1000.0);
    
    double alpha = 5.0 * M_PI / 180.0;
    Quaternion attitude = Quaternion::fromAxisAngle(Vector3D(0, 1, 0), alpha);
    
    DynamicsState state(stateVec, attitude, Vector3D::zero());
    
    Vector3D torque = torqueModel->calculateTorque(state, 0.0);
    
    // Should have non-zero torque with dynamic COP
    EXPECT_GT(torque.magnitude(), 0.0);
}

TEST_F(AerodynamicTorqueModelTest, CombinedMoments) {
    // Test with both force offset and moment coefficients
    StateVector stateVec;
    stateVec.setPosition(Vector3D(6471000.0, 0.0, 0.0));
    stateVec.setVelocity(Vector3D(340.0, 0.0, 0.0));
    stateVec.setMass(1000.0);
    
    // Create some angle of attack
    double alpha = 5.0 * M_PI / 180.0;
    Quaternion attitude = Quaternion::fromAxisAngle(Vector3D(0, 1, 0), alpha);
    
    DynamicsState state(stateVec, attitude, Vector3D::zero());
    
    Vector3D torque = torqueModel->calculateTorque(state, 0.0);
    
    // Calculate expected components
    double q = 0.5 * mockAtmosphere->testDensity * 340.0 * 340.0;
    double qS = q * defaultConfig.referenceArea;
    
    // Moment from coefficient
    double momentFromCoeff = qS * defaultConfig.referenceLength * (-0.015);
    
    // Force for offset calculation
    double dragForce = qS * 0.42;  // CD at Mach 1, 5 deg
    double liftForce = qS * 0.19;  // CL at Mach 1, 5 deg
    
    // The total torque should include both effects
    EXPECT_NE(torque.y(), momentFromCoeff);  // Not just the coefficient
    EXPECT_GT(torque.magnitude(), std::abs(momentFromCoeff));  // Combined effect
}

TEST_F(AerodynamicTorqueModelTest, AllAxesTorques) {
    // Create a configuration that generates moments on all axes
    // This requires sideslip and/or roll rate, but we'll use coefficient data
    
    // Add test data with non-zero lateral moments
    std::string lateralData = R"(# Lateral moments
# Mach, AoA_deg, CD, CL, CY, Cl, Cm, Cn
0.5, 0.0, 0.3, 0.0, 0.1, 0.02, -0.01, -0.015)";
    
    coeffDatabase->loadFromString(lateralData, "lateral");
    
    AerodynamicTorqueConfig config = defaultConfig;
    config.coefficientConfig = "lateral";
    torqueModel->setConfig(config);
    
    StateVector stateVec;
    stateVec.setPosition(Vector3D(6471000.0, 0.0, 0.0));
    stateVec.setVelocity(Vector3D(170.0, 0.0, 0.0));  // Mach 0.5
    stateVec.setMass(1000.0);
    
    DynamicsState state(stateVec, Quaternion::identity(), Vector3D::zero());
    
    Vector3D torque = torqueModel->calculateTorque(state, 0.0);
    
    // Should have moments on all axes from coefficients
    EXPECT_NE(torque.x(), 0.0);  // Roll from Cl
    EXPECT_NE(torque.y(), 0.0);  // Pitch from Cm
    EXPECT_NE(torque.z(), 0.0);  // Yaw from Cn
}