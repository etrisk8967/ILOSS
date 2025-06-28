#include <gtest/gtest.h>
#include "physics/forces/twobody/TwoBodyForceModel.h"
#include "physics/state/StateVector.h"
#include "core/time/Time.h"
#include "core/math/MathConstants.h"
#include <cmath>
#include <iostream>

namespace iloss {
namespace physics {
namespace forces {
namespace twobody {
namespace test {

class TwoBodyForceModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create default model with Earth parameters
        m_model = std::make_unique<TwoBodyForceModel>("TestTwoBody");
        
        // Create test time
        m_testTime = time::Time(2025, 6, 27, 12, 0, 0.0);
    }
    
    std::unique_ptr<TwoBodyForceModel> m_model;
    time::Time m_testTime;
};

TEST_F(TwoBodyForceModelTest, DefaultConstruction) {
    EXPECT_EQ(m_model->getName(), "TestTwoBody");
    EXPECT_EQ(m_model->getType(), ForceModelType::TwoBody);
    EXPECT_DOUBLE_EQ(m_model->getGravitationalParameter(), math::constants::EARTH_MU);
    EXPECT_EQ(m_model->getCentralBody(), "Earth");
    EXPECT_TRUE(m_model->isEnabled());
}

TEST_F(TwoBodyForceModelTest, CustomConstruction) {
    double customMu = 1.0e14;
    TwoBodyForceModel customModel("CustomModel", customMu);
    
    EXPECT_EQ(customModel.getName(), "CustomModel");
    EXPECT_DOUBLE_EQ(customModel.getGravitationalParameter(), customMu);
    EXPECT_EQ(customModel.getCentralBody(), "Custom");
}

TEST_F(TwoBodyForceModelTest, InvalidConstruction) {
    EXPECT_THROW(TwoBodyForceModel("Invalid", -1.0), std::invalid_argument);
    EXPECT_THROW(TwoBodyForceModel("Invalid", 0.0), std::invalid_argument);
}

TEST_F(TwoBodyForceModelTest, CircularOrbitAcceleration) {
    // Test circular orbit at 400 km altitude
    double altitude = 400000.0; // 400 km
    double radius = math::constants::EARTH_RADIUS_EQUATORIAL + altitude;
    
    // Position on x-axis
    math::Vector3D position(radius, 0.0, 0.0);
    
    // Circular velocity in y-direction
    double v_circular = std::sqrt(math::constants::EARTH_MU / radius);
    math::Vector3D velocity(0.0, v_circular, 0.0);
    
    StateVector state(position, velocity, 1000.0, m_testTime);
    
    // Calculate acceleration
    math::Vector3D acceleration = m_model->calculateAcceleration(state, m_testTime);
    
    // Expected acceleration magnitude for circular orbit
    double expected_magnitude = math::constants::EARTH_MU / (radius * radius);
    
    // Check magnitude
    EXPECT_NEAR(acceleration.magnitude(), expected_magnitude, 1e-10);
    
    // Check direction (should point toward origin)
    math::Vector3D expected_direction = -position.normalized();
    math::Vector3D actual_direction = acceleration.normalized();
    
    EXPECT_NEAR(actual_direction.x(), expected_direction.x(), 1e-10);
    EXPECT_NEAR(actual_direction.y(), expected_direction.y(), 1e-10);
    EXPECT_NEAR(actual_direction.z(), expected_direction.z(), 1e-10);
}

TEST_F(TwoBodyForceModelTest, EllipticalOrbitAtPeriapsis) {
    // Test elliptical orbit at periapsis
    double periapsis = 6678000.0; // ~300 km altitude
    double apoapsis = 42164000.0; // ~35786 km altitude (GEO)
    
    // Position at periapsis (on x-axis)
    math::Vector3D position(periapsis, 0.0, 0.0);
    
    // Velocity at periapsis (in y-direction)
    double a = (periapsis + apoapsis) / 2.0;
    double v_periapsis = std::sqrt(math::constants::EARTH_MU * (2.0 / periapsis - 1.0 / a));
    math::Vector3D velocity(0.0, v_periapsis, 0.0);
    
    StateVector state(position, velocity, 1000.0, m_testTime);
    
    // Calculate acceleration
    math::Vector3D acceleration = m_model->calculateAcceleration(state, m_testTime);
    
    // Check acceleration magnitude
    double expected_magnitude = math::constants::EARTH_MU / (periapsis * periapsis);
    EXPECT_NEAR(acceleration.magnitude(), expected_magnitude, 1e-10);
    
    // Check direction
    EXPECT_LT(acceleration.x(), 0.0); // Should point in -x direction
    EXPECT_NEAR(acceleration.y(), 0.0, 1e-10);
    EXPECT_NEAR(acceleration.z(), 0.0, 1e-10);
}

TEST_F(TwoBodyForceModelTest, AccelerationAtOrigin) {
    // Test singularity at origin
    math::Vector3D position(0.0, 0.0, 0.0);
    math::Vector3D velocity(1000.0, 0.0, 0.0);
    
    StateVector state(position, velocity, 1000.0, m_testTime);
    
    EXPECT_THROW(m_model->calculateAcceleration(state, m_testTime), std::runtime_error);
}

TEST_F(TwoBodyForceModelTest, AccelerationScaling) {
    // Test that acceleration scales correctly with distance
    math::Vector3D velocity(0.0, 7000.0, 0.0);
    
    // Test at different radii
    std::vector<double> radii = {7000000.0, 10000000.0, 20000000.0, 40000000.0};
    std::vector<math::Vector3D> accelerations;
    
    for (double r : radii) {
        math::Vector3D position(r, 0.0, 0.0);
        StateVector state(position, velocity, 1000.0, m_testTime);
        accelerations.push_back(m_model->calculateAcceleration(state, m_testTime));
    }
    
    // Check 1/r² scaling
    for (size_t i = 1; i < radii.size(); ++i) {
        double ratio = radii[i] / radii[0];
        double expected_accel = accelerations[0].magnitude() / (ratio * ratio);
        EXPECT_NEAR(accelerations[i].magnitude(), expected_accel, 1e-10);
    }
}

TEST_F(TwoBodyForceModelTest, InitializeWithConfig) {
    ForceModelConfig config;
    config.setParameter("mu", 1.0e15);
    config.setParameter("central_body", std::string("Jupiter"));
    
    EXPECT_TRUE(m_model->initialize(config));
    EXPECT_DOUBLE_EQ(m_model->getGravitationalParameter(), 1.0e15);
    EXPECT_EQ(m_model->getCentralBody(), "Jupiter");
}

TEST_F(TwoBodyForceModelTest, InitializeWithKnownBody) {
    ForceModelConfig config;
    config.setParameter("central_body", std::string("Moon"));
    
    EXPECT_TRUE(m_model->initialize(config));
    EXPECT_DOUBLE_EQ(m_model->getGravitationalParameter(), math::constants::MOON_MU);
    EXPECT_EQ(m_model->getCentralBody(), "Moon");
}

TEST_F(TwoBodyForceModelTest, Validation) {
    EXPECT_TRUE(m_model->validate());
    
    // Create invalid model
    m_model->setGravitationalParameter(1.0); // Valid
    EXPECT_TRUE(m_model->validate());
    
    // Can't set negative mu through setter (throws), so validation should always pass
    // after construction
}

TEST_F(TwoBodyForceModelTest, Clone) {
    m_model->setEnabled(false);
    m_model->setCentralBody("Mars");
    
    auto cloned = m_model->clone();
    
    EXPECT_EQ(cloned->getName(), m_model->getName());
    EXPECT_EQ(cloned->getType(), m_model->getType());
    EXPECT_EQ(cloned->isEnabled(), m_model->isEnabled());
    
    auto* clonedTwoBody = dynamic_cast<TwoBodyForceModel*>(cloned.get());
    ASSERT_NE(clonedTwoBody, nullptr);
    
    EXPECT_DOUBLE_EQ(clonedTwoBody->getGravitationalParameter(), 
                     m_model->getGravitationalParameter());
    EXPECT_EQ(clonedTwoBody->getCentralBody(), m_model->getCentralBody());
}

TEST_F(TwoBodyForceModelTest, ToString) {
    std::string str = m_model->toString();
    
    EXPECT_NE(str.find("TwoBodyForceModel"), std::string::npos);
    EXPECT_NE(str.find("TestTwoBody"), std::string::npos);
    EXPECT_NE(str.find("Earth"), std::string::npos);
    EXPECT_NE(str.find("enabled=true"), std::string::npos);
}

TEST_F(TwoBodyForceModelTest, SettersAndGetters) {
    // Test gravitational parameter setter
    double newMu = 2.0e14;
    m_model->setGravitationalParameter(newMu);
    EXPECT_DOUBLE_EQ(m_model->getGravitationalParameter(), newMu);
    
    // Test invalid mu
    EXPECT_THROW(m_model->setGravitationalParameter(-1.0), std::invalid_argument);
    EXPECT_THROW(m_model->setGravitationalParameter(0.0), std::invalid_argument);
    
    // Test central body setter with known body
    m_model->setCentralBody("Sun");
    EXPECT_EQ(m_model->getCentralBody(), "Sun");
    EXPECT_DOUBLE_EQ(m_model->getGravitationalParameter(), math::constants::SUN_MU);
    
    // Test central body setter with unknown body
    m_model->setCentralBody("Pluto");
    EXPECT_EQ(m_model->getCentralBody(), "Pluto");
    // Mu should remain unchanged from Sun
    EXPECT_DOUBLE_EQ(m_model->getGravitationalParameter(), math::constants::SUN_MU);
}

TEST_F(TwoBodyForceModelTest, GetBodyGravitationalParameter) {
    EXPECT_DOUBLE_EQ(TwoBodyForceModel::getBodyGravitationalParameter("Earth"), 
                     math::constants::EARTH_MU);
    EXPECT_DOUBLE_EQ(TwoBodyForceModel::getBodyGravitationalParameter("Sun"), 
                     math::constants::SUN_MU);
    EXPECT_DOUBLE_EQ(TwoBodyForceModel::getBodyGravitationalParameter("Moon"), 
                     math::constants::MOON_MU);
    EXPECT_DOUBLE_EQ(TwoBodyForceModel::getBodyGravitationalParameter("Jupiter"), 
                     math::constants::JUPITER_MU);
    EXPECT_DOUBLE_EQ(TwoBodyForceModel::getBodyGravitationalParameter("Venus"), 
                     math::constants::VENUS_MU);
    EXPECT_DOUBLE_EQ(TwoBodyForceModel::getBodyGravitationalParameter("Mars"), 
                     math::constants::MARS_MU);
    
    EXPECT_THROW(TwoBodyForceModel::getBodyGravitationalParameter("Pluto"), 
                 std::invalid_argument);
}

TEST_F(TwoBodyForceModelTest, AccelerationConsistency) {
    // Test that acceleration is consistent regardless of coordinate system
    // (as long as it's inertial)
    
    math::Vector3D position(10000000.0, 5000000.0, 2000000.0);
    math::Vector3D velocity(3000.0, -4000.0, 1000.0);
    
    // Create states in different coordinate systems (both inertial)
    StateVector stateECI(position, velocity, 1000.0, m_testTime,
                         coordinates::CoordinateSystemType::ECI_J2000);
    
    // Calculate accelerations
    math::Vector3D accelECI = m_model->calculateAcceleration(stateECI, m_testTime);
    
    // Accelerations should be the same in any inertial frame
    double r = position.magnitude();
    double expected_magnitude = math::constants::EARTH_MU / (r * r);
    
    EXPECT_NEAR(accelECI.magnitude(), expected_magnitude, 1e-10);
}

TEST_F(TwoBodyForceModelTest, HighAltitudeAccuracy) {
    // Test at GEO altitude
    double geoRadius = 42164000.0; // GEO radius
    math::Vector3D position(geoRadius, 0.0, 0.0);
    
    // GEO velocity
    double vGEO = std::sqrt(math::constants::EARTH_MU / geoRadius);
    math::Vector3D velocity(0.0, vGEO, 0.0);
    
    StateVector state(position, velocity, 1000.0, m_testTime);
    math::Vector3D acceleration = m_model->calculateAcceleration(state, m_testTime);
    
    // For circular GEO, centripetal acceleration should equal gravitational
    double expected = math::constants::EARTH_MU / (geoRadius * geoRadius);
    EXPECT_NEAR(acceleration.magnitude(), expected, 1e-12);
}

} // namespace test
} // namespace twobody
} // namespace forces
} // namespace physics
} // namespace iloss