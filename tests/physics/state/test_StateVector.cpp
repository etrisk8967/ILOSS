#include <gtest/gtest.h>
#include "physics/state/StateVector.h"
#include "core/math/MathConstants.h"
#include <cmath>

using namespace iloss::math;
using namespace iloss::time;
using iloss::coordinates::CoordinateSystemType;

class StateVectorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create some test positions and velocities
        testPosition = Vector3D(7000000.0, 0.0, 0.0);  // 7000 km radius
        testVelocity = Vector3D(0.0, 7500.0, 0.0);     // 7.5 km/s velocity
        testMass = 1000.0;  // 1000 kg
        testTime = Time(0.0);  // Epoch time
    }

    Vector3D testPosition;
    Vector3D testVelocity;
    double testMass;
    Time testTime;
};

// Test default constructor
TEST_F(StateVectorTest, DefaultConstructor) {
    iloss::physics::StateVector sv;
    EXPECT_FALSE(sv.isValid());
    EXPECT_EQ(sv.getPosition(), Vector3D::zero());
    EXPECT_EQ(sv.getVelocity(), Vector3D::zero());
    EXPECT_EQ(sv.getMass(), 0.0);
}

// Test parameterized constructor
TEST_F(StateVectorTest, ParameterizedConstructor) {
    iloss::physics::StateVector sv(testPosition, testVelocity, testMass, testTime);
    
    EXPECT_TRUE(sv.isValid());
    EXPECT_EQ(sv.getPosition(), testPosition);
    EXPECT_EQ(sv.getVelocity(), testVelocity);
    EXPECT_EQ(sv.getMass(), testMass);
    EXPECT_EQ(sv.getTime(), testTime);
    EXPECT_EQ(sv.getCoordinateSystem(), CoordinateSystemType::ECI_J2000);
}

// Test getters and setters
TEST_F(StateVectorTest, GettersAndSetters) {
    iloss::physics::StateVector sv;
    
    sv.setPosition(testPosition);
    EXPECT_EQ(sv.getPosition(), testPosition);
    
    sv.setVelocity(testVelocity);
    EXPECT_EQ(sv.getVelocity(), testVelocity);
    
    sv.setMass(testMass);
    EXPECT_EQ(sv.getMass(), testMass);
    
    sv.setTime(testTime);
    EXPECT_EQ(sv.getTime(), testTime);
    
    sv.setCoordinateSystem(CoordinateSystemType::ECEF_WGS84);
    EXPECT_EQ(sv.getCoordinateSystem(), CoordinateSystemType::ECEF_WGS84);
}

// Test mass validation
TEST_F(StateVectorTest, MassValidation) {
    iloss::physics::StateVector sv;
    
    // Negative mass should throw
    EXPECT_THROW(sv.setMass(-100.0), std::invalid_argument);
    
    // Zero mass should throw
    EXPECT_THROW(sv.setMass(0.0), std::invalid_argument);
    
    // Positive mass should work
    EXPECT_NO_THROW(sv.setMass(100.0));
}

// Test computed properties
TEST_F(StateVectorTest, ComputedProperties) {
    iloss::physics::StateVector sv(testPosition, testVelocity, testMass, testTime);
    
    // Test radius
    EXPECT_DOUBLE_EQ(sv.getRadius(), testPosition.magnitude());
    
    // Test speed
    EXPECT_DOUBLE_EQ(sv.getSpeed(), testVelocity.magnitude());
    
    // Test specific energy
    double expectedEnergy = testVelocity.magnitudeSquared() / 2.0 - 
                           constants::EARTH_MU / testPosition.magnitude();
    EXPECT_NEAR(sv.getSpecificEnergy(), expectedEnergy, 1e-9);
    
    // Test specific angular momentum
    Vector3D expectedH = testPosition.cross(testVelocity);
    EXPECT_EQ(sv.getSpecificAngularMomentum(), expectedH);
    
    // Test flight path angle (should be 0 for circular orbit)
    EXPECT_NEAR(sv.getFlightPathAngle(), 0.0, 1e-10);
}

// Test applyDeltaV
TEST_F(StateVectorTest, ApplyDeltaV) {
    iloss::physics::StateVector sv(testPosition, testVelocity, testMass, testTime);
    
    Vector3D deltaV(100.0, 0.0, 0.0);  // 100 m/s prograde burn
    double deltaMass = -10.0;  // 10 kg fuel consumed
    
    iloss::physics::StateVector newSv = sv.applyDeltaV(deltaV, deltaMass);
    
    EXPECT_EQ(newSv.getPosition(), testPosition);  // Position unchanged
    EXPECT_EQ(newSv.getVelocity(), testVelocity + deltaV);
    EXPECT_EQ(newSv.getMass(), testMass + deltaMass);
    EXPECT_EQ(newSv.getTime(), testTime);  // Time unchanged
    
    // Test that applying delta-v to invalid state throws
    iloss::physics::StateVector invalidSv;
    EXPECT_THROW(invalidSv.applyDeltaV(deltaV, deltaMass), std::runtime_error);
}

// Test linear interpolation
TEST_F(StateVectorTest, LinearInterpolation) {
    iloss::physics::StateVector sv1(testPosition, testVelocity, testMass, testTime);
    
    Vector3D pos2 = testPosition + testVelocity * 100.0;  // 100 seconds later
    Vector3D vel2 = testVelocity;  // Constant velocity
    Time time2(100.0);
    iloss::physics::StateVector sv2(pos2, vel2, testMass + 10.0, time2);
    
    // Interpolate at t=0.5 (50% between states)
    iloss::physics::StateVector interpSv = sv1.interpolateLinear(sv2, 0.5);
    
    Vector3D expectedPos = testPosition + testVelocity * 50.0;
    EXPECT_NEAR((interpSv.getPosition() - expectedPos).magnitude(), 0.0, 1e-9);
    EXPECT_EQ(interpSv.getVelocity(), testVelocity);
    EXPECT_DOUBLE_EQ(interpSv.getMass(), testMass + 5.0);
    
    // Test interpolation with different coordinate systems
    iloss::physics::StateVector sv3 = sv2;
    sv3.setCoordinateSystem(CoordinateSystemType::ECEF_WGS84);
    EXPECT_THROW(sv1.interpolateLinear(sv3, 0.5), std::invalid_argument);
}

// Test fromOrbitalElements
TEST_F(StateVectorTest, FromOrbitalElements) {
    // Test circular orbit
    double a = 7000000.0;  // 7000 km semi-major axis
    double e = 0.0;        // Circular
    double i = 0.0;        // Equatorial
    double omega = 0.0;    // Argument of periapsis
    double Omega = 0.0;    // RAAN
    double nu = 0.0;       // True anomaly
    
    iloss::physics::StateVector sv = iloss::physics::StateVector::fromOrbitalElements(a, e, i, omega, Omega, nu, 
                                                     testMass, testTime);
    
    EXPECT_TRUE(sv.isValid());
    EXPECT_NEAR(sv.getRadius(), a, 1e-9);
    
    // For circular orbit, speed should be sqrt(mu/a)
    double expectedSpeed = std::sqrt(constants::EARTH_MU / a);
    EXPECT_NEAR(sv.getSpeed(), expectedSpeed, 1e-9);
    
    // Test invalid orbital elements
    EXPECT_THROW(iloss::physics::StateVector::fromOrbitalElements(-a, e, i, omega, Omega, nu, 
                                                  testMass, testTime), 
                 std::invalid_argument);
    EXPECT_THROW(iloss::physics::StateVector::fromOrbitalElements(a, -0.1, i, omega, Omega, nu, 
                                                  testMass, testTime), 
                 std::invalid_argument);
}

// Test validation
TEST_F(StateVectorTest, Validation) {
    iloss::physics::StateVector sv;
    
    // Invalid state should not validate
    EXPECT_FALSE(sv.validate());
    EXPECT_FALSE(sv.getValidationError().empty());
    
    // Valid state should validate
    sv = iloss::physics::StateVector(testPosition, testVelocity, testMass, testTime);
    EXPECT_TRUE(sv.validate());
    EXPECT_TRUE(sv.getValidationError().empty());
    
    // Test position too close to Earth center
    Vector3D tooClose(50.0, 0.0, 0.0);  // 50m from center
    sv.setPosition(tooClose);
    EXPECT_FALSE(sv.validate());
    EXPECT_FALSE(sv.getValidationError().empty());
    
    // Test unrealistic velocity
    Vector3D tooFast(150000.0, 0.0, 0.0);  // 150 km/s
    sv.setPosition(testPosition);
    sv.setVelocity(tooFast);
    EXPECT_FALSE(sv.validate());
}

// Test equality operators
TEST_F(StateVectorTest, EqualityOperators) {
    iloss::physics::StateVector sv1(testPosition, testVelocity, testMass, testTime);
    iloss::physics::StateVector sv2(testPosition, testVelocity, testMass, testTime);
    
    EXPECT_TRUE(sv1 == sv2);
    EXPECT_FALSE(sv1 != sv2);
    
    // Change position slightly
    sv2.setPosition(testPosition + Vector3D(1e-6, 0.0, 0.0));
    EXPECT_FALSE(sv1 == sv2);
    EXPECT_TRUE(sv1 != sv2);
    
    // Different coordinate systems
    sv2 = sv1;
    sv2.setCoordinateSystem(CoordinateSystemType::ECEF_WGS84);
    EXPECT_FALSE(sv1 == sv2);
}

// Test toString
TEST_F(StateVectorTest, ToString) {
    iloss::physics::StateVector sv(testPosition, testVelocity, testMass, testTime);
    std::string str = sv.toString();
    
    EXPECT_FALSE(str.empty());
    EXPECT_NE(str.find("Pos="), std::string::npos);
    EXPECT_NE(str.find("Vel="), std::string::npos);
    EXPECT_NE(str.find("Mass="), std::string::npos);
    EXPECT_NE(str.find("Frame=ECI_J2000"), std::string::npos);
    
    // Test invalid state string
    iloss::physics::StateVector invalidSv;
    std::string invalidStr = invalidSv.toString();
    EXPECT_NE(invalidStr.find("INVALID"), std::string::npos);
}

// Test edge cases
TEST_F(StateVectorTest, EdgeCases) {
    // Test with NaN values
    iloss::physics::StateVector sv;
    sv.setPosition(Vector3D(std::nan(""), 0.0, 0.0));
    EXPECT_FALSE(sv.validate());
    
    // Test with infinity
    sv.setPosition(Vector3D(std::numeric_limits<double>::infinity(), 0.0, 0.0));
    EXPECT_FALSE(sv.validate());
    
    // Test very small but valid orbit
    Vector3D smallOrbit(6400000.0, 0.0, 0.0);  // Just above Earth surface
    sv = iloss::physics::StateVector(smallOrbit, Vector3D(0.0, 7900.0, 0.0), testMass, testTime);
    EXPECT_TRUE(sv.validate());
}

// Test specific energy at origin
TEST_F(StateVectorTest, SpecificEnergyAtOrigin) {
    iloss::physics::StateVector sv(Vector3D(1e-20, 0.0, 0.0), testVelocity, testMass, testTime);
    EXPECT_THROW(sv.getSpecificEnergy(), std::runtime_error);
}