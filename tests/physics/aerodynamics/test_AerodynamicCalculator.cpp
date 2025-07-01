#include <gtest/gtest.h>
#include "physics/aerodynamics/AerodynamicCalculator.h"
#include "core/math/Vector3D.h"
#include "core/math/Matrix3D.h"
#include "core/math/Quaternion.h"
#include <cmath>

using namespace iloss::physics::aerodynamics;
using namespace iloss::math;

class AerodynamicCalculatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Standard atmosphere at sea level
        seaLevelDensity = 1.225;  // kg/m³
        seaLevelSpeedOfSound = 340.3;  // m/s
        
        // Test velocities
        testVelocity = 100.0;  // m/s
        
        // Standard gravity
        g0 = 9.80665;  // m/s²
    }

    double seaLevelDensity;
    double seaLevelSpeedOfSound;
    double testVelocity;
    double g0;
};

TEST_F(AerodynamicCalculatorTest, CalculateMachNumber) {
    double velocity = 340.3;  // Speed of sound
    double mach = AerodynamicCalculator::calculateMachNumber(velocity, seaLevelSpeedOfSound);
    EXPECT_NEAR(mach, 1.0, 1e-6);
    
    // Subsonic
    mach = AerodynamicCalculator::calculateMachNumber(170.15, seaLevelSpeedOfSound);
    EXPECT_NEAR(mach, 0.5, 1e-6);
    
    // Supersonic
    mach = AerodynamicCalculator::calculateMachNumber(680.6, seaLevelSpeedOfSound);
    EXPECT_NEAR(mach, 2.0, 1e-6);
    
    // Zero velocity
    mach = AerodynamicCalculator::calculateMachNumber(0.0, seaLevelSpeedOfSound);
    EXPECT_DOUBLE_EQ(mach, 0.0);
    
    // Invalid speed of sound
    EXPECT_THROW(AerodynamicCalculator::calculateMachNumber(100.0, 0.0), std::invalid_argument);
}

TEST_F(AerodynamicCalculatorTest, CalculateDynamicPressure) {
    double q = AerodynamicCalculator::calculateDynamicPressure(seaLevelDensity, testVelocity);
    double expected = 0.5 * seaLevelDensity * testVelocity * testVelocity;
    EXPECT_DOUBLE_EQ(q, expected);
    
    // Zero velocity
    q = AerodynamicCalculator::calculateDynamicPressure(seaLevelDensity, 0.0);
    EXPECT_DOUBLE_EQ(q, 0.0);
    
    // Negative density should throw
    EXPECT_THROW(AerodynamicCalculator::calculateDynamicPressure(-1.0, testVelocity), 
                 std::invalid_argument);
}

TEST_F(AerodynamicCalculatorTest, CalculateAeroAnglesBasic) {
    // Test 1: Pure forward velocity (no angle of attack or sideslip)
    Vector3D velocityBody(100.0, 0.0, 0.0);
    auto angles = AerodynamicCalculator::calculateAeroAngles(velocityBody);
    
    ASSERT_TRUE(angles.has_value());
    EXPECT_NEAR(angles->first, 0.0, 1e-6);   // Alpha = 0
    EXPECT_NEAR(angles->second, 0.0, 1e-6);  // Beta = 0
    
    // Test 2: Pure downward velocity (90 degree angle of attack)
    velocityBody = Vector3D(0.0, 0.0, 100.0);
    angles = AerodynamicCalculator::calculateAeroAngles(velocityBody);
    
    ASSERT_TRUE(angles.has_value());
    EXPECT_NEAR(angles->first, M_PI/2, 1e-6);   // Alpha = 90 degrees
    EXPECT_NEAR(angles->second, 0.0, 1e-6);     // Beta = 0
    
    // Test 3: 45 degree angle of attack
    velocityBody = Vector3D(100.0, 0.0, 100.0);
    angles = AerodynamicCalculator::calculateAeroAngles(velocityBody);
    
    ASSERT_TRUE(angles.has_value());
    EXPECT_NEAR(angles->first, M_PI/4, 1e-6);   // Alpha = 45 degrees
    EXPECT_NEAR(angles->second, 0.0, 1e-6);     // Beta = 0
}

TEST_F(AerodynamicCalculatorTest, CalculateAeroAnglesSideslip) {
    // Test with sideslip
    Vector3D velocityBody(100.0, 50.0, 0.0);  // Forward and right
    auto angles = AerodynamicCalculator::calculateAeroAngles(velocityBody);
    
    ASSERT_TRUE(angles.has_value());
    EXPECT_NEAR(angles->first, 0.0, 1e-6);  // No angle of attack
    EXPECT_GT(angles->second, 0.0);         // Positive sideslip (nose right)
    
    // Calculate expected sideslip
    double expectedBeta = std::atan2(50.0, 100.0);
    EXPECT_NEAR(angles->second, expectedBeta, 1e-6);
}

TEST_F(AerodynamicCalculatorTest, CalculateAeroAnglesLowVelocity) {
    // Test with very low velocity
    Vector3D velocityBody(0.05, 0.0, 0.0);  // Below default threshold
    auto angles = AerodynamicCalculator::calculateAeroAngles(velocityBody);
    
    EXPECT_FALSE(angles.has_value());  // Should return nullopt
    
    // Test with custom threshold
    angles = AerodynamicCalculator::calculateAeroAngles(velocityBody, 0.01);
    EXPECT_TRUE(angles.has_value());
}

TEST_F(AerodynamicCalculatorTest, CalculateFlowProperties) {
    Vector3D velocityInertial(100.0, 0.0, 0.0);
    Quaternion attitude = Quaternion::identity();  // No rotation
    
    FlowProperties props = AerodynamicCalculator::calculateFlowProperties(
        velocityInertial, attitude, seaLevelDensity, seaLevelSpeedOfSound);
    
    EXPECT_TRUE(props.isValid());
    EXPECT_NEAR(props.mach, 100.0 / seaLevelSpeedOfSound, 1e-6);
    EXPECT_NEAR(props.dynamicPressure, 0.5 * seaLevelDensity * 100.0 * 100.0, 1e-6);
    EXPECT_NEAR(props.angleOfAttack, 0.0, 1e-6);
    EXPECT_NEAR(props.sideslipAngle, 0.0, 1e-6);
}

TEST_F(AerodynamicCalculatorTest, CalculateFlowPropertiesWithAttitude) {
    // Velocity in inertial frame (eastward)
    Vector3D velocityInertial(100.0, 0.0, 0.0);
    
    // Rotate vehicle 30 degrees about Z axis (yaw)
    double yawAngle = M_PI / 6;  // 30 degrees
    Quaternion attitude = Quaternion::fromAxisAngle(Vector3D(0, 0, 1), yawAngle);
    
    FlowProperties props = AerodynamicCalculator::calculateFlowProperties(
        velocityInertial, attitude, seaLevelDensity, seaLevelSpeedOfSound);
    
    // In body frame, velocity should have both X and Y components
    EXPECT_GT(props.velocityBodyFrame.x(), 0.0);
    EXPECT_LT(props.velocityBodyFrame.y(), 0.0);  // Negative Y due to rotation
    EXPECT_NEAR(props.velocityBodyFrame.z(), 0.0, 1e-10);
    
    // Should have sideslip angle
    EXPECT_LT(props.sideslipAngle, 0.0);  // Negative beta
}

TEST_F(AerodynamicCalculatorTest, BodyToWindTransformation) {
    // Test with no angles
    Vector3D vectorBody(1.0, 2.0, 3.0);
    Vector3D vectorWind = AerodynamicCalculator::bodyToWind(vectorBody, 0.0, 0.0);
    
    // Should be unchanged
    EXPECT_NEAR(vectorWind.x(), 1.0, 1e-10);
    EXPECT_NEAR(vectorWind.y(), 2.0, 1e-10);
    EXPECT_NEAR(vectorWind.z(), 3.0, 1e-10);
    
    // Test with 90 degree angle of attack
    vectorWind = AerodynamicCalculator::bodyToWind(vectorBody, M_PI/2, 0.0);
    
    // X and Z should swap (with sign change)
    EXPECT_NEAR(vectorWind.x(), -3.0, 1e-10);
    EXPECT_NEAR(vectorWind.y(), 2.0, 1e-10);
    EXPECT_NEAR(vectorWind.z(), 1.0, 1e-10);
}

TEST_F(AerodynamicCalculatorTest, WindToBodyTransformation) {
    // Test inverse transformation
    Vector3D vectorWind(1.0, 2.0, 3.0);
    double alpha = 0.1;  // Small angle
    double beta = 0.05;
    
    Vector3D vectorBody = AerodynamicCalculator::windToBody(vectorWind, alpha, beta);
    Vector3D vectorWindRecovered = AerodynamicCalculator::bodyToWind(vectorBody, alpha, beta);
    
    // Should recover original
    EXPECT_NEAR(vectorWindRecovered.x(), vectorWind.x(), 1e-10);
    EXPECT_NEAR(vectorWindRecovered.y(), vectorWind.y(), 1e-10);
    EXPECT_NEAR(vectorWindRecovered.z(), vectorWind.z(), 1e-10);
}

TEST_F(AerodynamicCalculatorTest, TransformationMatrices) {
    double alpha = 0.2;
    double beta = 0.1;
    
    Matrix3D bodyToWind = AerodynamicCalculator::getBodyToWindMatrix(alpha, beta);
    Matrix3D windToBody = AerodynamicCalculator::getWindToBodyMatrix(alpha, beta);
    
    // Matrices should be inverses of each other
    Matrix3D identity = bodyToWind * windToBody;
    
    // Check diagonal elements are 1
    EXPECT_NEAR(identity(0, 0), 1.0, 1e-10);
    EXPECT_NEAR(identity(1, 1), 1.0, 1e-10);
    EXPECT_NEAR(identity(2, 2), 1.0, 1e-10);
    
    // Check off-diagonal elements are 0
    EXPECT_NEAR(identity(0, 1), 0.0, 1e-10);
    EXPECT_NEAR(identity(0, 2), 0.0, 1e-10);
    EXPECT_NEAR(identity(1, 0), 0.0, 1e-10);
    EXPECT_NEAR(identity(1, 2), 0.0, 1e-10);
    EXPECT_NEAR(identity(2, 0), 0.0, 1e-10);
    EXPECT_NEAR(identity(2, 1), 0.0, 1e-10);
}

TEST_F(AerodynamicCalculatorTest, BodyToStability) {
    Vector3D vectorBody(100.0, 50.0, 25.0);
    double alpha = M_PI / 6;  // 30 degrees
    
    Vector3D vectorStability = AerodynamicCalculator::bodyToStability(vectorBody, alpha);
    
    // Y component should be unchanged
    EXPECT_DOUBLE_EQ(vectorStability.y(), vectorBody.y());
    
    // X and Z should be rotated
    double ca = std::cos(alpha);
    double sa = std::sin(alpha);
    EXPECT_NEAR(vectorStability.x(), ca * vectorBody.x() + sa * vectorBody.z(), 1e-10);
    EXPECT_NEAR(vectorStability.z(), -sa * vectorBody.x() + ca * vectorBody.z(), 1e-10);
}

TEST_F(AerodynamicCalculatorTest, CalculateTotalAngleOfAttack) {
    // Test 1: Pure forward velocity
    Vector3D velocityBody(100.0, 0.0, 0.0);
    double totalAoA = AerodynamicCalculator::calculateTotalAngleOfAttack(velocityBody);
    EXPECT_NEAR(totalAoA, 0.0, 1e-10);
    
    // Test 2: 45 degree total angle
    velocityBody = Vector3D(100.0, 0.0, 100.0);
    totalAoA = AerodynamicCalculator::calculateTotalAngleOfAttack(velocityBody);
    EXPECT_NEAR(totalAoA, M_PI/4, 1e-10);
    
    // Test 3: Pure sideways velocity
    velocityBody = Vector3D(0.0, 100.0, 0.0);
    totalAoA = AerodynamicCalculator::calculateTotalAngleOfAttack(velocityBody);
    EXPECT_NEAR(totalAoA, M_PI/2, 1e-10);
    
    // Test 4: Backward velocity
    velocityBody = Vector3D(-100.0, 0.0, 0.0);
    totalAoA = AerodynamicCalculator::calculateTotalAngleOfAttack(velocityBody);
    EXPECT_NEAR(totalAoA, M_PI, 1e-10);
}

TEST_F(AerodynamicCalculatorTest, CalculateReynoldsNumber) {
    double density = 1.225;      // kg/m³
    double velocity = 100.0;     // m/s
    double length = 5.0;         // m
    double viscosity = 1.8e-5;   // Pa·s (approximate for air at sea level)
    
    double Re = AerodynamicCalculator::calculateReynoldsNumber(
        density, velocity, length, viscosity);
    
    double expected = density * velocity * length / viscosity;
    EXPECT_DOUBLE_EQ(Re, expected);
    
    // Should be in typical range for aircraft
    EXPECT_GT(Re, 1e6);  // Turbulent flow
    
    // Test invalid inputs
    EXPECT_THROW(AerodynamicCalculator::calculateReynoldsNumber(
        density, velocity, length, 0.0), std::invalid_argument);
    EXPECT_THROW(AerodynamicCalculator::calculateReynoldsNumber(
        density, velocity, 0.0, viscosity), std::invalid_argument);
}

TEST_F(AerodynamicCalculatorTest, CalculateCenterOfPressure) {
    double cm = -0.05;      // Nose-down moment
    double cn = 0.5;        // Normal force coefficient
    double refLength = 2.0; // m
    double refPoint = 0.5;  // 0.5m from nose
    
    double xcp = AerodynamicCalculator::calculateCenterOfPressure(
        cm, cn, refLength, refPoint);
    
    // Center of pressure should be ahead of reference point for nose-down moment
    EXPECT_LT(xcp, refPoint);
    
    // Test with zero normal force
    xcp = AerodynamicCalculator::calculateCenterOfPressure(
        cm, 0.0, refLength, refPoint);
    EXPECT_DOUBLE_EQ(xcp, refPoint);  // Returns reference point
}

TEST_F(AerodynamicCalculatorTest, IsTransonic) {
    EXPECT_FALSE(AerodynamicCalculator::isTransonic(0.5));   // Subsonic
    EXPECT_TRUE(AerodynamicCalculator::isTransonic(0.9));    // Transonic
    EXPECT_TRUE(AerodynamicCalculator::isTransonic(1.0));    // Transonic
    EXPECT_TRUE(AerodynamicCalculator::isTransonic(1.1));    // Transonic
    EXPECT_FALSE(AerodynamicCalculator::isTransonic(1.5));   // Supersonic
    
    // Test with custom bounds
    EXPECT_TRUE(AerodynamicCalculator::isTransonic(0.75, 0.7, 1.3));
    EXPECT_FALSE(AerodynamicCalculator::isTransonic(0.65, 0.7, 1.3));
}

TEST_F(AerodynamicCalculatorTest, CalculateSpeedOfSound) {
    // Test at standard temperature (15°C = 288.15K)
    double T = 288.15;
    double a = AerodynamicCalculator::calculateSpeedOfSound(T);
    EXPECT_NEAR(a, 340.3, 0.1);  // Should be close to standard value
    
    // Test at different temperatures
    double a_cold = AerodynamicCalculator::calculateSpeedOfSound(223.15);  // -50°C
    double a_hot = AerodynamicCalculator::calculateSpeedOfSound(323.15);   // 50°C
    
    EXPECT_LT(a_cold, a);
    EXPECT_GT(a_hot, a);
    
    // Test invalid temperature
    EXPECT_THROW(AerodynamicCalculator::calculateSpeedOfSound(0.0), std::invalid_argument);
}

TEST_F(AerodynamicCalculatorTest, FlowPropertiesWithWind) {
    Vector3D velocityInertial(100.0, 0.0, 0.0);
    Vector3D windVelocity(20.0, 0.0, 0.0);  // Tailwind
    Quaternion attitude = Quaternion::identity();
    
    FlowProperties props = AerodynamicCalculator::calculateFlowProperties(
        velocityInertial, attitude, seaLevelDensity, seaLevelSpeedOfSound, windVelocity);
    
    // Relative velocity should be reduced
    double relativeVelocity = 80.0;  // 100 - 20
    EXPECT_NEAR(props.velocityBodyFrame.magnitude(), relativeVelocity, 1e-6);
    EXPECT_NEAR(props.mach, relativeVelocity / seaLevelSpeedOfSound, 1e-6);
    
    // Dynamic pressure should be reduced
    double expectedQ = 0.5 * seaLevelDensity * relativeVelocity * relativeVelocity;
    EXPECT_NEAR(props.dynamicPressure, expectedQ, 1e-6);
}