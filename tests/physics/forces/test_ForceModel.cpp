#include <gtest/gtest.h>
#include "physics/forces/ForceModel.h"
#include "physics/forces/SimpleGravityModel.h"
#include "physics/state/StateVector.h"
#include "core/time/Time.h"
#include "core/math/Vector3D.h"
#include <chrono>

using namespace iloss::physics::forces;
using namespace iloss::time;
using namespace iloss::math;
using iloss::physics::StateVector;  // Explicitly use physics::StateVector
using iloss::coordinates::CoordinateSystemType;

/**
 * @brief Test fixture for ForceModel tests
 */
class ForceModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a default state vector at 400km altitude
        Vector3D position(6778137.0, 0.0, 0.0); // Earth radius + 400km
        Vector3D velocity(0.0, 7668.0, 0.0);     // Circular orbit velocity
        double mass = 1000.0;                     // 1000 kg
        Time time(2025, 6, 28, 12, 0, 0.0);
        
        m_defaultState = StateVector(position, velocity, mass, time);
    }

    StateVector m_defaultState;
};

/**
 * @brief Test ForceModelConfig class
 */
TEST_F(ForceModelTest, ForceModelConfigTest) {
    ForceModelConfig config;
    
    // Test setting and getting parameters
    config.setParameter("mu", 3.986004418e14);
    config.setParameter("central_body", std::string("Earth"));
    config.setParameter("degree", 8);
    config.setParameter("order", 8);
    config.setParameter("enable_j2", true);
    
    // Test getting parameters with correct types
    EXPECT_DOUBLE_EQ(config.getParameter<double>("mu"), 3.986004418e14);
    EXPECT_EQ(config.getParameter<std::string>("central_body"), "Earth");
    EXPECT_EQ(config.getParameter<int>("degree"), 8);
    EXPECT_EQ(config.getParameter<int>("order"), 8);
    EXPECT_TRUE(config.getParameter<bool>("enable_j2"));
    
    // Test default values
    EXPECT_DOUBLE_EQ(config.getParameter<double>("nonexistent", 1.0), 1.0);
    EXPECT_EQ(config.getParameter<std::string>("missing", "default"), "default");
    
    // Test parameter existence check
    EXPECT_TRUE(config.hasParameter("mu"));
    EXPECT_TRUE(config.hasParameter("central_body"));
    EXPECT_FALSE(config.hasParameter("nonexistent"));
    
    // Test clearing parameters
    config.clear();
    EXPECT_FALSE(config.hasParameter("mu"));
    EXPECT_FALSE(config.hasParameter("central_body"));
}

/**
 * @brief Test ForceModelType enum conversions
 */
TEST_F(ForceModelTest, ForceModelTypeConversionTest) {
    // Test all force model type conversions
    EXPECT_EQ(ForceModel::forceModelTypeToString(ForceModelType::TwoBody), "TwoBody");
    EXPECT_EQ(ForceModel::forceModelTypeToString(ForceModelType::GravityField), "GravityField");
    EXPECT_EQ(ForceModel::forceModelTypeToString(ForceModelType::ThirdBody), "ThirdBody");
    EXPECT_EQ(ForceModel::forceModelTypeToString(ForceModelType::Drag), "Drag");
    EXPECT_EQ(ForceModel::forceModelTypeToString(ForceModelType::SolarRadiation), "SolarRadiation");
    EXPECT_EQ(ForceModel::forceModelTypeToString(ForceModelType::EarthRadiation), "EarthRadiation");
    EXPECT_EQ(ForceModel::forceModelTypeToString(ForceModelType::Relativistic), "Relativistic");
    EXPECT_EQ(ForceModel::forceModelTypeToString(ForceModelType::Thrust), "Thrust");
    EXPECT_EQ(ForceModel::forceModelTypeToString(ForceModelType::UserDefined), "UserDefined");
}

/**
 * @brief Test SimpleGravityModel implementation
 */
TEST_F(ForceModelTest, SimpleGravityModelBasicTest) {
    SimpleGravityModel gravity("EarthGravity");
    
    // Test basic properties
    EXPECT_EQ(gravity.getName(), "EarthGravity");
    EXPECT_EQ(gravity.getType(), ForceModelType::TwoBody);
    EXPECT_TRUE(gravity.isEnabled());
    
    // Test enable/disable
    gravity.setEnabled(false);
    EXPECT_FALSE(gravity.isEnabled());
    gravity.setEnabled(true);
    EXPECT_TRUE(gravity.isEnabled());
    
    // Test string representation
    std::string str = gravity.toString();
    EXPECT_TRUE(str.find("EarthGravity") != std::string::npos);
    EXPECT_TRUE(str.find("TwoBody") != std::string::npos);
}

/**
 * @brief Test SimpleGravityModel initialization
 */
TEST_F(ForceModelTest, SimpleGravityModelInitializationTest) {
    SimpleGravityModel gravity("TestGravity");
    
    // Test default initialization
    EXPECT_TRUE(gravity.validate());
    EXPECT_DOUBLE_EQ(gravity.getGravitationalParameter(), 3.986004418e14);
    
    // Test custom initialization
    ForceModelConfig config;
    config.setParameter("mu", 4.9048695e12);  // Moon's gravitational parameter
    config.setParameter("central_body", std::string("Moon"));
    
    EXPECT_TRUE(gravity.initialize(config));
    EXPECT_DOUBLE_EQ(gravity.getGravitationalParameter(), 4.9048695e12);
    EXPECT_TRUE(gravity.validate());
}

/**
 * @brief Test SimpleGravityModel acceleration calculation
 */
TEST_F(ForceModelTest, SimpleGravityModelAccelerationTest) {
    SimpleGravityModel gravity("EarthGravity");
    
    // Calculate acceleration at default state
    Time time(2025, 6, 28, 12, 0, 0.0);
    Vector3D acceleration = gravity.calculateAcceleration(m_defaultState, time);
    
    // Check acceleration direction (should be towards Earth center)
    EXPECT_LT(acceleration.x(), 0.0);  // Negative x direction
    EXPECT_NEAR(acceleration.y(), 0.0, 1e-10);  // Near zero in y
    EXPECT_NEAR(acceleration.z(), 0.0, 1e-10);  // Near zero in z
    
    // Check acceleration magnitude
    double r = m_defaultState.getPosition().magnitude();
    double expectedMag = gravity.getGravitationalParameter() / (r * r);
    EXPECT_NEAR(acceleration.magnitude(), expectedMag, 1e-10);
    
    // Test at different position
    Vector3D position2(0.0, 7378137.0, 0.0);  // 1000km altitude
    StateVector state2(position2, Vector3D::zero(), 1000.0, time);
    Vector3D acceleration2 = gravity.calculateAcceleration(state2, time);
    
    // Check that acceleration decreases with altitude
    EXPECT_LT(acceleration2.magnitude(), acceleration.magnitude());
}

/**
 * @brief Test SimpleGravityModel singularity handling
 */
TEST_F(ForceModelTest, SimpleGravityModelSingularityTest) {
    SimpleGravityModel gravity("EarthGravity");
    Time time(2025, 6, 28, 12, 0, 0.0);
    
    // Test near singularity (very close to center)
    Vector3D nearZeroPos(0.1, 0.1, 0.1);  // 0.17m from center
    StateVector nearZeroState(nearZeroPos, Vector3D::zero(), 1000.0, time);
    
    Vector3D acceleration = gravity.calculateAcceleration(nearZeroState, time);
    
    // Should return zero acceleration to avoid singularity
    EXPECT_EQ(acceleration.magnitude(), 0.0);
}

/**
 * @brief Test SimpleGravityModel cloning
 */
TEST_F(ForceModelTest, SimpleGravityModelCloneTest) {
    SimpleGravityModel original("OriginalGravity");
    
    // Configure the original
    ForceModelConfig config;
    config.setParameter("mu", 1.327e20);  // Sun's gravitational parameter
    original.initialize(config);
    original.setEnabled(false);
    
    // Clone the model
    auto cloned = original.clone();
    
    // Verify clone properties
    EXPECT_EQ(cloned->getName(), "OriginalGravity");
    EXPECT_EQ(cloned->getType(), ForceModelType::TwoBody);
    EXPECT_FALSE(cloned->isEnabled());
    
    // Verify cloned parameters
    auto* clonedGravity = dynamic_cast<SimpleGravityModel*>(cloned.get());
    ASSERT_NE(clonedGravity, nullptr);
    EXPECT_DOUBLE_EQ(clonedGravity->getGravitationalParameter(), 1.327e20);
    
    // Verify independence of clone
    original.setEnabled(true);
    EXPECT_FALSE(cloned->isEnabled());
}

/**
 * @brief Test force model with different coordinate systems
 */
TEST_F(ForceModelTest, SimpleGravityModelCoordinateSystemTest) {
    SimpleGravityModel gravity("EarthGravity");
    Time time(2025, 6, 28, 12, 0, 0.0);
    
    // Test with ECI coordinates (default)
    Vector3D positionECI(6778137.0, 0.0, 0.0);
    StateVector stateECI(positionECI, Vector3D::zero(), 1000.0, time, 
                         CoordinateSystemType::ECI_J2000);
    Vector3D accelECI = gravity.calculateAcceleration(stateECI, time);
    
    // Test with ECEF coordinates (should give same result for two-body)
    StateVector stateECEF(positionECI, Vector3D::zero(), 1000.0, time,
                          CoordinateSystemType::ECEF_WGS84);
    Vector3D accelECEF = gravity.calculateAcceleration(stateECEF, time);
    
    // For simple two-body gravity, acceleration should be the same
    // regardless of coordinate system (as long as position is the same)
    EXPECT_NEAR(accelECI.magnitude(), accelECEF.magnitude(), 1e-10);
}

/**
 * @brief Performance test for acceleration calculation
 */
TEST_F(ForceModelTest, SimpleGravityModelPerformanceTest) {
    SimpleGravityModel gravity("EarthGravity");
    Time time(2025, 6, 28, 12, 0, 0.0);
    
    // Time many acceleration calculations
    const int iterations = 100000;
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < iterations; ++i) {
        Vector3D acceleration = gravity.calculateAcceleration(m_defaultState, time);
        // Prevent optimization
        EXPECT_GT(acceleration.magnitude(), 0.0);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    double avgTimeUs = static_cast<double>(duration.count()) / iterations;
    
    // Log performance
    std::cout << "Average time per acceleration calculation: " 
              << avgTimeUs << " microseconds" << std::endl;
    
    // Ensure reasonable performance (less than 10 microseconds per calculation)
    EXPECT_LT(avgTimeUs, 10.0);
}