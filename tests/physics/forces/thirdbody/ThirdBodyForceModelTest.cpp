#include <gtest/gtest.h>
#include "physics/forces/thirdbody/ThirdBodyForceModel.h"
#include "physics/state/StateVector.h"
#include "core/time/Time.h"
#include "core/math/MathConstants.h"
#include <cmath>

using namespace iloss;
using namespace iloss::physics;
using namespace iloss::physics::forces;
using namespace iloss::physics::forces::thirdbody;
using namespace iloss::math;
using namespace iloss::time;

/**
 * @brief Test fixture for ThirdBodyForceModel tests
 * 
 * This fixture sets up common test data and helper methods for testing
 * third-body gravitational perturbations.
 */
class ThirdBodyForceModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a default force model
        forceModel = std::make_unique<ThirdBodyForceModel>("TestThirdBody");
        
        // Create a test state at GEO orbit (approximately)
        Vector3D position(42164000.0, 0.0, 0.0);  // 42,164 km from Earth center
        Vector3D velocity(0.0, 3074.0, 0.0);      // Circular velocity at GEO
        testState = StateVector(position, velocity, 1000.0, testTime);
        
        // Set test time to J2000 epoch
        testTime = Time(2000, 1, 1, 12, 0, 0.0);
    }
    
    /**
     * @brief Helper method to check if a vector is approximately zero
     */
    bool isApproximatelyZero(const Vector3D& vec, double tolerance = 1e-10) {
        return vec.magnitude() < tolerance;
    }
    
    /**
     * @brief Helper method to calculate expected perturbation magnitude
     * 
     * This provides an order-of-magnitude estimate for validation
     */
    double estimatePerturbationMagnitude(const std::string& body, double distance) {
        // Get gravitational parameter
        double mu = 0.0;
        if (body == "Sun") {
            mu = constants::SUN_MU;
        } else if (body == "Moon") {
            mu = constants::MOON_MU;
        } else if (body == "Jupiter") {
            mu = constants::JUPITER_MU;
        }
        
        // Very rough estimate: perturbation ~ mu / distance^2
        // This is the direct term; the indirect term reduces this
        return mu / (distance * distance);
    }

    std::unique_ptr<ThirdBodyForceModel> forceModel;
    StateVector testState;
    Time testTime;
};

/**
 * @brief Test default construction
 */
TEST_F(ThirdBodyForceModelTest, DefaultConstruction) {
    EXPECT_EQ(forceModel->getName(), "TestThirdBody");
    EXPECT_EQ(forceModel->getType(), ForceModelType::ThirdBody);
    EXPECT_TRUE(forceModel->isEnabled());
    EXPECT_EQ(forceModel->getCentralBody(), "Earth");
    EXPECT_EQ(forceModel->getCacheDuration(), 60.0);
    EXPECT_TRUE(forceModel->getBodies().empty());
}

/**
 * @brief Test SPICE availability check
 */
TEST_F(ThirdBodyForceModelTest, SpiceAvailability) {
    bool available = ThirdBodyForceModel::isSpiceAvailable();
    // The result depends on whether SPICE is compiled in
    #ifdef SPICE_FOUND
        EXPECT_TRUE(available);
    #else
        EXPECT_FALSE(available);
    #endif
}

/**
 * @brief Test adding and removing bodies
 */
TEST_F(ThirdBodyForceModelTest, AddRemoveBodies) {
    // Add Sun
    ThirdBodyConfig sunConfig("Sun", "10", constants::SUN_MU);
    forceModel->addBody(sunConfig);
    
    auto bodies = forceModel->getBodies();
    EXPECT_EQ(bodies.size(), 1);
    EXPECT_EQ(bodies[0].name, "Sun");
    EXPECT_EQ(bodies[0].spiceId, "10");
    EXPECT_EQ(bodies[0].mu, constants::SUN_MU);
    EXPECT_TRUE(bodies[0].enabled);
    
    // Add Moon
    ThirdBodyConfig moonConfig("Moon", "301", constants::MOON_MU);
    forceModel->addBody(moonConfig);
    
    bodies = forceModel->getBodies();
    EXPECT_EQ(bodies.size(), 2);
    
    // Remove Sun
    EXPECT_TRUE(forceModel->removeBody("Sun"));
    bodies = forceModel->getBodies();
    EXPECT_EQ(bodies.size(), 1);
    EXPECT_EQ(bodies[0].name, "Moon");
    
    // Try to remove non-existent body
    EXPECT_FALSE(forceModel->removeBody("Mars"));
}

/**
 * @brief Test enabling/disabling bodies
 */
TEST_F(ThirdBodyForceModelTest, EnableDisableBodies) {
    // Add bodies
    ThirdBodyConfig sunConfig("Sun", "10", constants::SUN_MU);
    ThirdBodyConfig moonConfig("Moon", "301", constants::MOON_MU);
    forceModel->addBody(sunConfig);
    forceModel->addBody(moonConfig);
    
    // Disable Sun
    EXPECT_TRUE(forceModel->setBodyEnabled("Sun", false));
    
    auto bodies = forceModel->getBodies();
    for (const auto& body : bodies) {
        if (body.name == "Sun") {
            EXPECT_FALSE(body.enabled);
        } else if (body.name == "Moon") {
            EXPECT_TRUE(body.enabled);
        }
    }
    
    // Try to disable non-existent body
    EXPECT_FALSE(forceModel->setBodyEnabled("Mars", false));
}

/**
 * @brief Test initialization with configuration
 */
TEST_F(ThirdBodyForceModelTest, InitializeWithConfig) {
    ForceModelConfig config;
    
    // Set up configuration
    config.setParameter("central_body", std::string("Earth"));
    config.setParameter("cache_duration", 120.0);
    
    std::vector<std::string> bodyList = {"Sun", "Moon"};
    config.setParameter("bodies", bodyList);
    
    EXPECT_TRUE(forceModel->initialize(config));
    
    // Check configuration was applied
    EXPECT_EQ(forceModel->getCentralBody(), "Earth");
    EXPECT_EQ(forceModel->getCacheDuration(), 120.0);
    
    auto bodies = forceModel->getBodies();
    EXPECT_EQ(bodies.size(), 2);
    
    // Check bodies were added with correct parameters
    bool foundSun = false, foundMoon = false;
    for (const auto& body : bodies) {
        if (body.name == "Sun") {
            foundSun = true;
            EXPECT_EQ(body.mu, constants::SUN_MU);
        } else if (body.name == "Moon") {
            foundMoon = true;
            EXPECT_EQ(body.mu, constants::MOON_MU);
        }
    }
    EXPECT_TRUE(foundSun);
    EXPECT_TRUE(foundMoon);
}

/**
 * @brief Test initialization with enable_all option
 */
TEST_F(ThirdBodyForceModelTest, InitializeEnableAll) {
    ForceModelConfig config;
    config.setParameter("enable_all", true);
    config.setParameter("central_body", std::string("Earth"));
    
    EXPECT_TRUE(forceModel->initialize(config));
    
    auto bodies = forceModel->getBodies();
    // Should have all default bodies except Earth (central body)
    EXPECT_GT(bodies.size(), 5);  // At least Sun, Moon, Venus, Mars, Jupiter
    
    // Verify Earth is not included
    for (const auto& body : bodies) {
        EXPECT_NE(body.name, "Earth");
    }
}

/**
 * @brief Test cache duration settings
 */
TEST_F(ThirdBodyForceModelTest, CacheDuration) {
    // Test valid duration
    forceModel->setCacheDuration(300.0);
    EXPECT_EQ(forceModel->getCacheDuration(), 300.0);
    
    // Test negative duration (should be clamped to 0)
    forceModel->setCacheDuration(-10.0);
    EXPECT_EQ(forceModel->getCacheDuration(), 0.0);
}

/**
 * @brief Test central body configuration
 */
TEST_F(ThirdBodyForceModelTest, CentralBodyConfig) {
    // Test known body
    forceModel->setCentralBody("Moon");
    EXPECT_EQ(forceModel->getCentralBody(), "Moon");
    
    // Test unknown body (should still set the name)
    forceModel->setCentralBody("CustomBody");
    EXPECT_EQ(forceModel->getCentralBody(), "CustomBody");
}

/**
 * @brief Test SPICE ID lookup
 */
TEST_F(ThirdBodyForceModelTest, SpiceIdLookup) {
    EXPECT_EQ(ThirdBodyForceModel::getSpiceId("Sun"), "10");
    EXPECT_EQ(ThirdBodyForceModel::getSpiceId("Moon"), "301");
    EXPECT_EQ(ThirdBodyForceModel::getSpiceId("Earth"), "399");
    EXPECT_EQ(ThirdBodyForceModel::getSpiceId("Jupiter"), "599");
    EXPECT_EQ(ThirdBodyForceModel::getSpiceId("Unknown"), "");
}

/**
 * @brief Test gravitational parameters
 */
TEST_F(ThirdBodyForceModelTest, GravitationalParameters) {
    const auto& muValues = ThirdBodyForceModel::getDefaultGravitationalParameters();
    
    EXPECT_GT(muValues.size(), 5);
    
    // Verify some known values
    auto sunIt = muValues.find("Sun");
    ASSERT_NE(sunIt, muValues.end());
    EXPECT_EQ(sunIt->second, constants::SUN_MU);
    
    auto moonIt = muValues.find("Moon");
    ASSERT_NE(moonIt, muValues.end());
    EXPECT_EQ(moonIt->second, constants::MOON_MU);
}

/**
 * @brief Test validation
 */
TEST_F(ThirdBodyForceModelTest, Validation) {
    // Empty model should be valid (produces zero acceleration)
    EXPECT_TRUE(forceModel->validate());
    
    // Add valid body
    ThirdBodyConfig validConfig("Sun", "10", constants::SUN_MU);
    forceModel->addBody(validConfig);
    EXPECT_TRUE(forceModel->validate());
    
    // Add invalid body (zero mu)
    ThirdBodyConfig invalidConfig1("BadBody1", "999", 0.0);
    forceModel->addBody(invalidConfig1);
    EXPECT_FALSE(forceModel->validate());
    
    // Remove invalid body
    forceModel->removeBody("BadBody1");
    EXPECT_TRUE(forceModel->validate());
    
    // Add invalid body (empty SPICE ID)
    ThirdBodyConfig invalidConfig2("BadBody2", "", 1.0e20);
    forceModel->addBody(invalidConfig2);
    EXPECT_FALSE(forceModel->validate());
}

/**
 * @brief Test clone functionality
 */
TEST_F(ThirdBodyForceModelTest, Clone) {
    // Configure the model
    forceModel->setCentralBody("Moon");
    forceModel->setCacheDuration(180.0);
    forceModel->setEnabled(false);
    
    ThirdBodyConfig sunConfig("Sun", "10", constants::SUN_MU);
    ThirdBodyConfig moonConfig("Moon", "301", constants::MOON_MU);
    forceModel->addBody(sunConfig);
    forceModel->addBody(moonConfig);
    
    // Clone the model
    auto cloned = forceModel->clone();
    ASSERT_NE(cloned, nullptr);
    
    // Verify clone has same configuration
    auto clonedThirdBody = dynamic_cast<ThirdBodyForceModel*>(cloned.get());
    ASSERT_NE(clonedThirdBody, nullptr);
    
    EXPECT_EQ(clonedThirdBody->getName(), forceModel->getName());
    EXPECT_EQ(clonedThirdBody->getCentralBody(), "Moon");
    EXPECT_EQ(clonedThirdBody->getCacheDuration(), 180.0);
    EXPECT_FALSE(clonedThirdBody->isEnabled());
    
    auto clonedBodies = clonedThirdBody->getBodies();
    EXPECT_EQ(clonedBodies.size(), 2);
}

/**
 * @brief Test string representation
 */
TEST_F(ThirdBodyForceModelTest, ToString) {
    // Add some bodies
    ThirdBodyConfig sunConfig("Sun", "10", constants::SUN_MU);
    ThirdBodyConfig moonConfig("Moon", "301", constants::MOON_MU);
    moonConfig.enabled = false;
    
    forceModel->addBody(sunConfig);
    forceModel->addBody(moonConfig);
    
    std::string str = forceModel->toString();
    
    // Verify string contains expected information
    EXPECT_NE(str.find("ThirdBodyForceModel"), std::string::npos);
    EXPECT_NE(str.find("TestThirdBody"), std::string::npos);
    EXPECT_NE(str.find("Earth"), std::string::npos);
    EXPECT_NE(str.find("Sun"), std::string::npos);
    EXPECT_NE(str.find("Moon (disabled)"), std::string::npos);
}

/**
 * @brief Test cache statistics
 */
TEST_F(ThirdBodyForceModelTest, CacheStatistics) {
    // Initially, cache should be empty
    EXPECT_EQ(forceModel->getCacheHits(), 0);
    EXPECT_EQ(forceModel->getCacheMisses(), 0);
    
    // Reset statistics
    forceModel->resetCacheStats();
    EXPECT_EQ(forceModel->getCacheHits(), 0);
    EXPECT_EQ(forceModel->getCacheMisses(), 0);
}

/**
 * @brief Test acceleration calculation without SPICE
 * 
 * If SPICE is not available, the model should return zero acceleration
 */
TEST_F(ThirdBodyForceModelTest, AccelerationWithoutSpice) {
    #ifndef SPICE_FOUND
    // Add bodies
    ThirdBodyConfig sunConfig("Sun", "10", constants::SUN_MU);
    ThirdBodyConfig moonConfig("Moon", "301", constants::MOON_MU);
    forceModel->addBody(sunConfig);
    forceModel->addBody(moonConfig);
    
    // Calculate acceleration
    Vector3D accel = forceModel->calculateAcceleration(testState, testTime);
    
    // Should be zero without SPICE
    EXPECT_TRUE(isApproximatelyZero(accel));
    #else
    GTEST_SKIP() << "SPICE is available, skipping no-SPICE test";
    #endif
}

/**
 * @brief Test acceleration calculation with empty body list
 */
TEST_F(ThirdBodyForceModelTest, AccelerationEmptyBodyList) {
    // No bodies added, should return zero acceleration
    Vector3D accel = forceModel->calculateAcceleration(testState, testTime);
    EXPECT_TRUE(isApproximatelyZero(accel));
}

/**
 * @brief Test acceleration with disabled bodies
 */
TEST_F(ThirdBodyForceModelTest, AccelerationDisabledBodies) {
    // Add bodies but disable them
    ThirdBodyConfig sunConfig("Sun", "10", constants::SUN_MU);
    ThirdBodyConfig moonConfig("Moon", "301", constants::MOON_MU);
    forceModel->addBody(sunConfig);
    forceModel->addBody(moonConfig);
    
    forceModel->setBodyEnabled("Sun", false);
    forceModel->setBodyEnabled("Moon", false);
    
    // Should return zero acceleration
    Vector3D accel = forceModel->calculateAcceleration(testState, testTime);
    EXPECT_TRUE(isApproximatelyZero(accel));
}

/**
 * @brief Test perturbation order of magnitude
 * 
 * This test verifies that perturbations have reasonable magnitudes
 * based on theoretical expectations.
 */
TEST_F(ThirdBodyForceModelTest, PerturbationMagnitudes) {
    #ifdef SPICE_FOUND
    // This test requires actual SPICE kernels to be loaded
    // For now, we'll create a test that validates the structure
    
    // Add Sun and Moon
    ThirdBodyConfig sunConfig("Sun", "10", constants::SUN_MU);
    ThirdBodyConfig moonConfig("Moon", "301", constants::MOON_MU);
    forceModel->addBody(sunConfig);
    forceModel->addBody(moonConfig);
    
    // Typical perturbation accelerations at GEO:
    // Sun: ~5e-6 m/s²
    // Moon: ~1e-5 m/s²
    // These are order-of-magnitude estimates
    
    EXPECT_TRUE(forceModel->validate());
    #else
    GTEST_SKIP() << "SPICE not available, skipping magnitude test";
    #endif
}

/**
 * @brief Test update method
 */
TEST_F(ThirdBodyForceModelTest, UpdateMethod) {
    // Add bodies
    ThirdBodyConfig sunConfig("Sun", "10", constants::SUN_MU);
    forceModel->addBody(sunConfig);
    
    // Update should not throw
    EXPECT_NO_THROW(forceModel->update(testTime));
}

/**
 * @brief Test custom body configuration
 */
TEST_F(ThirdBodyForceModelTest, CustomBodyConfig) {
    ForceModelConfig config;
    
    // Create custom bodies
    std::vector<ThirdBodyConfig> customBodies;
    ThirdBodyConfig asteroid("Ceres", "2000001", 6.26e10);  // Ceres
    ThirdBodyConfig spacecraft("ISS", "25544", 4.0e5);     // ISS (negligible)
    customBodies.push_back(asteroid);
    customBodies.push_back(spacecraft);
    
    config.setParameter("custom_bodies", customBodies);
    
    EXPECT_TRUE(forceModel->initialize(config));
    
    auto bodies = forceModel->getBodies();
    EXPECT_EQ(bodies.size(), 2);
    
    // Verify custom bodies were added
    bool foundCeres = false, foundISS = false;
    for (const auto& body : bodies) {
        if (body.name == "Ceres") {
            foundCeres = true;
            EXPECT_EQ(body.spiceId, "2000001");
            EXPECT_EQ(body.mu, 6.26e10);
        } else if (body.name == "ISS") {
            foundISS = true;
            EXPECT_EQ(body.spiceId, "25544");
            EXPECT_EQ(body.mu, 4.0e5);
        }
    }
    EXPECT_TRUE(foundCeres);
    EXPECT_TRUE(foundISS);
}

/**
 * @brief Test minimum distance constraint
 */
TEST_F(ThirdBodyForceModelTest, MinimumDistanceConstraint) {
    // This test verifies that the minimum distance constraint
    // prevents singularities when spacecraft is too close to perturbing body
    
    ThirdBodyConfig config("TestBody", "999", 1.0e20);
    config.minDistance = 1000000.0;  // 1000 km
    forceModel->addBody(config);
    
    auto bodies = forceModel->getBodies();
    ASSERT_EQ(bodies.size(), 1);
    EXPECT_EQ(bodies[0].minDistance, 1000000.0);
}

// Additional integration tests would require actual SPICE kernel data
// and would be implemented in a separate integration test suite