#include <gtest/gtest.h>
#include "physics/aerodynamics/AerodynamicForceModel.h"
#include "physics/aerodynamics/AerodynamicDatabase.h"
#include "physics/dynamics/DynamicsState.h"
#include "core/constants/AtmosphericModel.h"
#include "core/math/Vector3D.h"
#include "core/math/Quaternion.h"
#include "core/time/Time.h"
#include <memory>
#include <cmath>

using namespace iloss::physics::aerodynamics;
using namespace iloss::physics::dynamics;
using namespace iloss::physics::forces;
using namespace iloss::constants;
using namespace iloss::math;
using namespace iloss::time;

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

class AerodynamicForceModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create mock atmosphere
        mockAtmosphere = std::make_shared<MockAtmosphereModel>();
        
        // Create coefficient database with test data
        coeffDatabase = std::make_shared<AerodynamicDatabase>();
        std::string testData = R"(# Test data
# Mach, AoA_deg, CD, CL, CY, Cl, Cm, Cn
0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0
0.0, 5.0, 0.32, 0.16, 0.0, 0.0, -0.01, 0.0
1.0, 0.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0
1.0, 5.0, 0.42, 0.19, 0.0, 0.0, -0.015, 0.0)";
        coeffDatabase->loadFromString(testData);
        
        // Default configuration
        defaultConfig.referenceArea = 10.0;  // m²
        defaultConfig.referenceLength = 2.0;  // m
        defaultConfig.referenceSpan = 5.0;    // m
        
        // Create force model
        forceModel = std::make_unique<AerodynamicForceModel>(
            mockAtmosphere, coeffDatabase, defaultConfig);
    }
    
    std::shared_ptr<MockAtmosphereModel> mockAtmosphere;
    std::shared_ptr<AerodynamicDatabase> coeffDatabase;
    AerodynamicConfig defaultConfig;
    std::unique_ptr<AerodynamicForceModel> forceModel;
};

TEST_F(AerodynamicForceModelTest, Constructor) {
    EXPECT_NO_THROW(AerodynamicForceModel(mockAtmosphere, coeffDatabase));
    EXPECT_NO_THROW(AerodynamicForceModel(mockAtmosphere, coeffDatabase, defaultConfig));
    
    // Should throw with null atmosphere
    EXPECT_THROW(AerodynamicForceModel(nullptr, coeffDatabase), std::invalid_argument);
    
    // Should throw with null database
    EXPECT_THROW(AerodynamicForceModel(mockAtmosphere, nullptr), std::invalid_argument);
}

TEST_F(AerodynamicForceModelTest, ZeroVelocityNoForce) {
    // Create state with zero velocity
    StateVector stateVec;
    stateVec.setPosition(Vector3D(7000000.0, 0.0, 0.0));  // ~630 km altitude
    stateVec.setVelocity(Vector3D(0.0, 0.0, 0.0));
    stateVec.setMass(1000.0);
    
    DynamicsState state(stateVec, Quaternion::identity(), Vector3D::zero());
    Time currentTime;
    
    Vector3D acceleration = forceModel->calculateAccelerationWithAttitude(state, currentTime);
    
    // Should be zero acceleration with zero velocity
    EXPECT_NEAR(acceleration.magnitude(), 0.0, 1e-10);
}

TEST_F(AerodynamicForceModelTest, PureDragForce) {
    // Create state with forward velocity only
    StateVector stateVec;
    stateVec.setPosition(Vector3D(6471000.0, 0.0, 0.0));  // ~100 km altitude
    stateVec.setVelocity(Vector3D(7000.0, 0.0, 0.0));     // Orbital velocity
    stateVec.setMass(1000.0);
    
    DynamicsState state(stateVec, Quaternion::identity(), Vector3D::zero());
    Time currentTime;
    
    Vector3D acceleration = forceModel->calculateAccelerationWithAttitude(state, currentTime);
    
    // Drag should oppose velocity (negative X direction)
    EXPECT_LT(acceleration.x(), 0.0);
    EXPECT_NEAR(acceleration.y(), 0.0, 1e-10);
    EXPECT_NEAR(acceleration.z(), 0.0, 1e-10);
    
    // Calculate expected drag magnitude
    double velocity = 7000.0;
    double speedOfSound = AerodynamicCalculator::calculateSpeedOfSound(mockAtmosphere->testTemperature);
    double mach = velocity / speedOfSound;
    double q = 0.5 * mockAtmosphere->testDensity * velocity * velocity;
    
    // Get CD at Mach number and zero AoA
    AerodynamicCoefficients coeffs = coeffDatabase->getCoefficients(mach, 0.0);
    double expectedDragForce = q * defaultConfig.referenceArea * coeffs.getDragCoefficient();
    double expectedAccel = expectedDragForce / stateVec.getMass();
    
    EXPECT_NEAR(std::abs(acceleration.x()), expectedAccel, expectedAccel * 0.1);  // 10% tolerance
}

TEST_F(AerodynamicForceModelTest, AngleOfAttackGeneratesLift) {
    // Create state with velocity at angle to body axis
    StateVector stateVec;
    stateVec.setPosition(Vector3D(6471000.0, 0.0, 0.0));
    stateVec.setVelocity(Vector3D(7000.0, 0.0, 0.0));
    stateVec.setMass(1000.0);
    
    // Rotate vehicle to create angle of attack
    double pitchAngle = 5.0 * M_PI / 180.0;  // 5 degrees
    Quaternion attitude = Quaternion::fromAxisAngle(Vector3D(0, 1, 0), pitchAngle);
    
    DynamicsState state(stateVec, attitude, Vector3D::zero());
    Time currentTime;
    
    Vector3D acceleration = forceModel->calculateAccelerationWithAttitude(state, currentTime);
    
    // Should have both drag (negative X) and lift components
    EXPECT_LT(acceleration.x(), 0.0);  // Drag
    EXPECT_NE(acceleration.magnitude(), std::abs(acceleration.x()));  // Not pure drag
}

TEST_F(AerodynamicForceModelTest, MinimumDensityThreshold) {
    // Set very low density threshold
    AerodynamicConfig config = defaultConfig;
    config.minimumDensity = 1e-6;
    forceModel->setConfig(config);
    
    // Set atmosphere to below threshold
    mockAtmosphere->testDensity = 1e-7;
    
    StateVector stateVec;
    stateVec.setPosition(Vector3D(6871000.0, 0.0, 0.0));  // High altitude
    stateVec.setVelocity(Vector3D(7000.0, 0.0, 0.0));
    stateVec.setMass(1000.0);
    
    DynamicsState state(stateVec, Quaternion::identity(), Vector3D::zero());
    Time currentTime;
    
    Vector3D acceleration = forceModel->calculateAccelerationWithAttitude(state, currentTime);
    
    // Should be zero due to density threshold
    EXPECT_DOUBLE_EQ(acceleration.magnitude(), 0.0);
}

TEST_F(AerodynamicForceModelTest, GetSetReferenceArea) {
    EXPECT_DOUBLE_EQ(forceModel->getReferenceArea(), defaultConfig.referenceArea);
    
    forceModel->setReferenceArea(20.0);
    EXPECT_DOUBLE_EQ(forceModel->getReferenceArea(), 20.0);
    
    // Should throw for invalid area
    EXPECT_THROW(forceModel->setReferenceArea(0.0), std::invalid_argument);
    EXPECT_THROW(forceModel->setReferenceArea(-1.0), std::invalid_argument);
}

TEST_F(AerodynamicForceModelTest, Configuration) {
    ForceModelConfig config;
    config.setParameter("reference_area", 15.0);
    config.setParameter("reference_length", 3.0);
    config.setParameter("reference_span", 7.0);
    config.setParameter("cop_offset_x", 0.5);
    config.setParameter("cop_offset_y", 0.0);
    config.setParameter("cop_offset_z", 0.1);
    config.setParameter("enable_q_limit", true);
    config.setParameter("max_q", 50000.0);
    config.setParameter("min_density", 1e-10);
    config.setParameter("enabled", true);
    
    EXPECT_TRUE(forceModel->initialize(config));
    
    const auto& aeroConfig = forceModel->getConfig();
    EXPECT_DOUBLE_EQ(aeroConfig.referenceArea, 15.0);
    EXPECT_DOUBLE_EQ(aeroConfig.referenceLength, 3.0);
    EXPECT_DOUBLE_EQ(aeroConfig.referenceSpan, 7.0);
    EXPECT_DOUBLE_EQ(aeroConfig.centerOfPressureOffset.x(), 0.5);
    EXPECT_DOUBLE_EQ(aeroConfig.centerOfPressureOffset.z(), 0.1);
    EXPECT_TRUE(aeroConfig.enableDynamicPressureLimit);
    EXPECT_DOUBLE_EQ(aeroConfig.maxDynamicPressure, 50000.0);
    EXPECT_TRUE(forceModel->isEnabled());
}

TEST_F(AerodynamicForceModelTest, Validation) {
    EXPECT_TRUE(forceModel->validate());
    
    // Invalid reference area
    AerodynamicConfig badConfig = defaultConfig;
    badConfig.referenceArea = 0.0;
    forceModel->setConfig(badConfig);
    EXPECT_FALSE(forceModel->validate());
    
    // Non-existent coefficient config
    badConfig = defaultConfig;
    badConfig.coefficientConfig = "nonexistent";
    forceModel->setConfig(badConfig);
    EXPECT_FALSE(forceModel->validate());
}

TEST_F(AerodynamicForceModelTest, Clone) {
    auto cloned = forceModel->clone();
    ASSERT_NE(cloned, nullptr);
    
    // Should be same type
    auto clonedAero = dynamic_cast<AerodynamicForceModel*>(cloned.get());
    ASSERT_NE(clonedAero, nullptr);
    
    // Should have same configuration
    EXPECT_DOUBLE_EQ(clonedAero->getReferenceArea(), forceModel->getReferenceArea());
}

TEST_F(AerodynamicForceModelTest, DynamicPressureLimit) {
    // Enable Q limit
    AerodynamicConfig config = defaultConfig;
    config.enableDynamicPressureLimit = true;
    config.maxDynamicPressure = 10000.0;  // Low limit for testing
    forceModel->setConfig(config);
    
    // High velocity to exceed Q limit
    StateVector stateVec;
    stateVec.setPosition(Vector3D(6471000.0, 0.0, 0.0));
    stateVec.setVelocity(Vector3D(8000.0, 0.0, 0.0));  // High velocity
    stateVec.setMass(1000.0);
    
    DynamicsState state(stateVec, Quaternion::identity(), Vector3D::zero());
    Time currentTime;
    
    // Calculate actual Q
    double actualQ = 0.5 * mockAtmosphere->testDensity * 8000.0 * 8000.0;
    EXPECT_GT(actualQ, config.maxDynamicPressure);  // Verify we exceed limit
    
    Vector3D acceleration = forceModel->calculateAccelerationWithAttitude(state, currentTime);
    
    // Force should be limited by max Q
    double expectedLimitedForce = config.maxDynamicPressure * defaultConfig.referenceArea * 0.4;  // CD~0.4
    double expectedAccel = expectedLimitedForce / stateVec.getMass();
    
    EXPECT_NEAR(std::abs(acceleration.x()), expectedAccel, expectedAccel * 0.2);  // 20% tolerance
}

TEST_F(AerodynamicForceModelTest, LastFlowProperties) {
    StateVector stateVec;
    stateVec.setPosition(Vector3D(6471000.0, 0.0, 0.0));
    stateVec.setVelocity(Vector3D(7000.0, 0.0, 0.0));
    stateVec.setMass(1000.0);
    
    DynamicsState state(stateVec, Quaternion::identity(), Vector3D::zero());
    Time currentTime;
    
    // Initially no cached properties
    EXPECT_FALSE(forceModel->getLastFlowProperties().has_value());
    EXPECT_FALSE(forceModel->getLastCoefficients().has_value());
    
    // Calculate forces
    forceModel->calculateAccelerationWithAttitude(state, currentTime);
    
    // Should now have cached values
    auto flowProps = forceModel->getLastFlowProperties();
    ASSERT_TRUE(flowProps.has_value());
    EXPECT_GT(flowProps->mach, 0.0);
    EXPECT_GT(flowProps->dynamicPressure, 0.0);
    EXPECT_NEAR(flowProps->angleOfAttack, 0.0, 1e-6);
    
    auto coeffs = forceModel->getLastCoefficients();
    ASSERT_TRUE(coeffs.has_value());
    EXPECT_GT(coeffs->getDragCoefficient(), 0.0);
}

TEST_F(AerodynamicForceModelTest, CalculateBodyForces) {
    FlowProperties flowProps;
    flowProps.mach = 0.5;
    flowProps.dynamicPressure = 10000.0;
    flowProps.angleOfAttack = 5.0 * M_PI / 180.0;
    flowProps.sideslipAngle = 0.0;
    
    AerodynamicCoefficients coeffs(0.3, 0.15, 0.0);
    
    Vector3D bodyForce = forceModel->calculateBodyForces(flowProps, coeffs);
    
    // Check force magnitudes are reasonable
    EXPECT_GT(bodyForce.magnitude(), 0.0);
    
    // With positive AoA and CL, should have lift component
    double qS = flowProps.dynamicPressure * defaultConfig.referenceArea;
    double expectedDrag = qS * coeffs.getDragCoefficient();
    double expectedLift = qS * coeffs.getLiftCoefficient();
    
    // Rough check - forces should be in right ballpark
    EXPECT_GT(bodyForce.magnitude(), expectedLift * 0.5);
    EXPECT_LT(bodyForce.magnitude(), (expectedDrag + expectedLift) * 2.0);
}