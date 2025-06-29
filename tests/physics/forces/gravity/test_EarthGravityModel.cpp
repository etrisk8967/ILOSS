#include <gtest/gtest.h>
#include "physics/forces/gravity/EarthGravityModel.h"
#include "physics/state/StateVector.h"
#include "core/constants/EarthModel.h"
#include "core/math/MathConstants.h"
#include "core/time/Time.h"
#include <cmath>
#include <fstream>
#include <chrono>

using namespace iloss::physics::forces;
using namespace iloss::physics;
using namespace iloss::math;
using namespace iloss::constants;
using namespace iloss::time;

class EarthGravityModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test positions
        double issAltitude = 408000.0; // ISS altitude in meters
        double geoAltitude = 35786000.0; // GEO altitude in meters
        
        // Position at ISS orbit (circular, equatorial)
        issPosition = Vector3D(EarthModel::EQUATORIAL_RADIUS + issAltitude, 0.0, 0.0);
        
        // Position at GEO
        geoPosition = Vector3D(EarthModel::EQUATORIAL_RADIUS + geoAltitude, 0.0, 0.0);
        
        // Position at pole (surface)
        polePosition = Vector3D(0.0, 0.0, EarthModel::POLAR_RADIUS);
        
        // Off-equatorial position
        double r = EarthModel::EQUATORIAL_RADIUS + issAltitude;
        offEquatorPosition = Vector3D(r * 0.8, 0.0, r * 0.6); // 36.87 degrees latitude
        
        // Create test time
        testTime = Time::now();
        
        // Create default configuration
        defaultConfig.maxDegree = 8;
        defaultConfig.maxOrder = 8;
        defaultConfig.useCache = false; // Disable cache for deterministic tests
        
        // Create sample coefficients for testing
        createTestCoefficients();
    }
    
    void createTestCoefficients() {
        // Add J2, J3, J4 coefficients
        testCoefficients.push_back({2, 0, -constants::EARTH_J2, 0.0});
        testCoefficients.push_back({3, 0, -constants::EARTH_J3, 0.0});
        testCoefficients.push_back({4, 0, -constants::EARTH_J4, 0.0});
        
        // Add some tesseral coefficients
        testCoefficients.push_back({2, 2, 2.43e-6, -1.40e-6});
        testCoefficients.push_back({3, 1, 2.03e-6, 2.48e-7});
        testCoefficients.push_back({3, 3, 7.21e-7, 1.47e-6});
    }
    
    StateVector createStateVector(const Vector3D& position) {
        // Create dummy velocity (not used in gravity calculation)
        Vector3D velocity(0.0, 7700.0, 0.0); // Typical ISS velocity
        double mass = 1000.0; // 1000 kg satellite
        
        return StateVector(position, velocity, mass, testTime);
    }
    
    // Test positions
    Vector3D issPosition;
    Vector3D geoPosition;
    Vector3D polePosition;
    Vector3D offEquatorPosition;
    
    // Test time
    Time testTime;
    
    // Configuration
    EarthGravityConfig defaultConfig;
    
    // Test coefficients
    std::vector<SphericalHarmonicCoefficient> testCoefficients;
};

// Basic functionality tests

TEST_F(EarthGravityModelTest, Construction) {
    EarthGravityModel model("TestGravity", defaultConfig);
    
    EXPECT_EQ(model.getName(), "TestGravity");
    EXPECT_EQ(model.getType(), ForceModelType::GravityField);
    EXPECT_EQ(model.getMaxDegree(), 8);
    EXPECT_EQ(model.getMaxOrder(), 8);
}

TEST_F(EarthGravityModelTest, Initialization) {
    EarthGravityModel model;
    
    ForceModelConfig config;
    config.setParameter("maxDegree", 10);
    config.setParameter("maxOrder", 10);
    config.setParameter("useCache", true);
    
    EXPECT_TRUE(model.initialize(config));
    EXPECT_EQ(model.getMaxDegree(), 4); // Limited by default coefficients (J2-J4)
    EXPECT_EQ(model.getMaxOrder(), 0);  // Only zonal harmonics by default
}

TEST_F(EarthGravityModelTest, LoadCoefficients) {
    EarthGravityModel model("TestGravity", defaultConfig);
    
    EXPECT_TRUE(model.loadCoefficients(testCoefficients));
    
    // After loading, degree/order should be limited by loaded coefficients
    model.setDegreeOrder(10, 10);
    EXPECT_EQ(model.getMaxDegree(), 4); // Limited by loaded coefficients
    EXPECT_EQ(model.getMaxOrder(), 3);  // Limited by loaded coefficients
}

// Physics validation tests

TEST_F(EarthGravityModelTest, PointMassLimit) {
    // With only degree 0, should match point mass gravity
    EarthGravityConfig config;
    config.maxDegree = 0;
    config.maxOrder = 0;
    config.useCache = false;
    
    EarthGravityModel shModel("SH_PointMass", config);
    
    StateVector state = createStateVector(issPosition);
    Vector3D acc = shModel.calculateAcceleration(state, testTime);
    
    // Compare with simple point mass
    double r = issPosition.magnitude();
    double expectedMag = EarthModel::MU / (r * r);
    
    EXPECT_NEAR(acc.magnitude(), expectedMag, expectedMag * 1e-10);
    
    // Direction should be toward center
    Vector3D expectedDir = -issPosition.normalized();
    Vector3D actualDir = acc.normalized();
    
    EXPECT_NEAR(actualDir.x(), expectedDir.x(), 1e-10);
    EXPECT_NEAR(actualDir.y(), expectedDir.y(), 1e-10);
    EXPECT_NEAR(actualDir.z(), expectedDir.z(), 1e-10);
}

TEST_F(EarthGravityModelTest, J2Effect) {
    // Test with J2 only
    EarthGravityConfig config;
    config.maxDegree = 2;
    config.maxOrder = 0;
    config.useCache = false;
    
    EarthGravityModel model("J2Model", config);
    model.loadCoefficients(testCoefficients);
    
    // Test at equator vs pole
    StateVector equatorState = createStateVector(issPosition);
    StateVector poleState = createStateVector(polePosition);
    
    Vector3D accEquator = model.calculateAcceleration(equatorState, testTime);
    Vector3D accPole = model.calculateAcceleration(poleState, testTime);
    
    // J2 effect should be different at pole vs equator
    double rEquator = issPosition.magnitude();
    double rPole = polePosition.magnitude();
    
    // Normalize by point mass to isolate J2 effect
    double pointMassEquator = EarthModel::MU / (rEquator * rEquator);
    double pointMassPole = EarthModel::MU / (rPole * rPole);
    
    double j2EffectEquator = std::abs(accEquator.magnitude() - pointMassEquator) / pointMassEquator;
    double j2EffectPole = std::abs(accPole.magnitude() - pointMassPole) / pointMassPole;
    
    // J2 effect should be present
    EXPECT_GT(j2EffectEquator, 1e-5);
    EXPECT_GT(j2EffectPole, 1e-5);
}

TEST_F(EarthGravityModelTest, TesseralHarmonics) {
    // Test with tesseral harmonics
    EarthGravityModel model("TesseralModel", defaultConfig);
    model.loadCoefficients(testCoefficients);
    
    // Position with non-zero longitude
    Vector3D position(issPosition.x() * 0.707, issPosition.x() * 0.707, 0.0); // 45 degrees longitude
    StateVector state = createStateVector(position);
    
    Vector3D acc = model.calculateAcceleration(state, testTime);
    
    // With tesseral harmonics, there should be a tangential component
    // Project acceleration onto local vertical and horizontal
    Vector3D radial = position.normalized();
    Vector3D tangential = Vector3D(-position.y(), position.x(), 0.0).normalized();
    
    double radialComponent = acc.dot(radial);
    double tangentialComponent = acc.dot(tangential);
    
    // Radial component should dominate
    EXPECT_LT(radialComponent, 0.0); // Negative (toward Earth)
    
    // But there should be a small tangential component from tesseral harmonics
    EXPECT_NE(tangentialComponent, 0.0);
    EXPECT_LT(std::abs(tangentialComponent / radialComponent), 0.01); // Small but non-zero
}

// Performance tests

TEST_F(EarthGravityModelTest, CachingPerformance) {
    EarthGravityConfig cachedConfig = defaultConfig;
    cachedConfig.useCache = true;
    cachedConfig.cacheResolution = 10.0; // 10 meter resolution
    
    EarthGravityModel cachedModel("CachedModel", cachedConfig);
    cachedModel.loadCoefficients(testCoefficients);
    
    EarthGravityConfig uncachedConfig = defaultConfig;
    uncachedConfig.useCache = false;
    
    EarthGravityModel uncachedModel("UncachedModel", uncachedConfig);
    uncachedModel.loadCoefficients(testCoefficients);
    
    StateVector state = createStateVector(issPosition);
    
    // Warm up cache
    for (int i = 0; i < 10; ++i) {
        cachedModel.calculateAcceleration(state, testTime);
    }
    
    // Time cached access
    auto startCached = std::chrono::high_resolution_clock::now();
    const int iterations = 1000;
    for (int i = 0; i < iterations; ++i) {
        cachedModel.calculateAcceleration(state, testTime);
    }
    auto endCached = std::chrono::high_resolution_clock::now();
    
    // Time uncached access
    auto startUncached = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        uncachedModel.calculateAcceleration(state, testTime);
    }
    auto endUncached = std::chrono::high_resolution_clock::now();
    
    auto cachedDuration = std::chrono::duration_cast<std::chrono::microseconds>(endCached - startCached);
    auto uncachedDuration = std::chrono::duration_cast<std::chrono::microseconds>(endUncached - startUncached);
    
    // Cached should be significantly faster for repeated calculations
    EXPECT_LT(cachedDuration.count(), uncachedDuration.count() / 2);
    
    // Check cache statistics
    size_t hits, misses, size;
    cachedModel.getCacheStatistics(hits, misses, size);
    
    EXPECT_GT(hits, iterations - 10); // Most should be hits
    EXPECT_LT(misses, 20); // Only initial accesses should miss
    EXPECT_GT(size, 0); // Cache should contain entries
}

TEST_F(EarthGravityModelTest, HighDegreePerformance) {
    // Test that calculation time scales reasonably with degree/order
    std::vector<int> degrees = {4, 8, 16, 32};
    std::vector<double> times;
    
    StateVector state = createStateVector(issPosition);
    
    for (int degree : degrees) {
        EarthGravityConfig config;
        config.maxDegree = degree;
        config.maxOrder = degree;
        config.useCache = false;
        
        EarthGravityModel model("HighDegree", config);
        
        // Create fake coefficients up to this degree
        std::vector<SphericalHarmonicCoefficient> coeffs;
        for (int n = 0; n <= degree; ++n) {
            for (int m = 0; m <= n && m <= degree; ++m) {
                coeffs.push_back({n, m, 1e-6, 1e-6});
            }
        }
        model.loadCoefficients(coeffs);
        
        auto start = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < 100; ++i) {
            model.calculateAcceleration(state, testTime);
        }
        auto end = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        times.push_back(duration.count() / 100.0); // Average time per calculation
    }
    
    // Time should scale roughly as O(n²) with degree
    // Check that doubling degree increases time by less than 5x
    for (size_t i = 1; i < times.size(); ++i) {
        double ratio = times[i] / times[i-1];
        EXPECT_LT(ratio, 5.0);
    }
}

// Gradient calculation tests

TEST_F(EarthGravityModelTest, GravityGradient) {
    EarthGravityModel model("GradientModel", defaultConfig);
    model.loadCoefficients(testCoefficients);
    
    Matrix3D gradient = model.calculateGravityGradient(issPosition);
    
    // Gravity gradient should be symmetric for conservative field
    for (int i = 0; i < 3; ++i) {
        for (int j = i + 1; j < 3; ++j) {
            EXPECT_NEAR(gradient(i, j), gradient(j, i), 1e-8);
        }
    }
    
    // Trace should be zero (Laplace's equation in vacuum)
    double trace = gradient(0, 0) + gradient(1, 1) + gradient(2, 2);
    EXPECT_NEAR(trace, 0.0, 1e-7); // Relaxed tolerance due to numerical differentiation
    
    // For position along x-axis, certain components should be zero
    EXPECT_NEAR(gradient(0, 1), 0.0, 1e-10); // dax/dy
    EXPECT_NEAR(gradient(0, 2), 0.0, 1e-10); // dax/dz
    EXPECT_NEAR(gradient(1, 2), 0.0, 1e-10); // day/dz
}

// Potential calculation tests

TEST_F(EarthGravityModelTest, Potential) {
    EarthGravityModel model("PotentialModel", defaultConfig);
    model.loadCoefficients(testCoefficients);
    
    double potential = model.calculatePotential(issPosition);
    
    // Potential should be negative (bound orbit)
    EXPECT_LT(potential, 0.0);
    
    // Compare with point mass potential
    double r = issPosition.magnitude();
    double pointMassPotential = -EarthModel::MU / r;
    
    // Should be close to point mass but not exactly equal due to harmonics
    EXPECT_NEAR(potential, pointMassPotential, std::abs(pointMassPotential) * 0.01);
    
    // Test gradient relationship: acceleration = -grad(potential)
    const double delta = 1.0; // 1 meter
    Vector3D acc = model.calculateAcceleration(createStateVector(issPosition), testTime);
    
    // Numerical gradient
    Vector3D posX = issPosition + Vector3D(delta, 0, 0);
    Vector3D posY = issPosition + Vector3D(0, delta, 0);
    Vector3D posZ = issPosition + Vector3D(0, 0, delta);
    
    double potX = model.calculatePotential(posX);
    double potY = model.calculatePotential(posY);
    double potZ = model.calculatePotential(posZ);
    
    Vector3D numericalGrad((potX - potential) / delta,
                          (potY - potential) / delta,
                          (potZ - potential) / delta);
    
    // Acceleration should equal negative gradient
    // Using absolute tolerance for small components due to numerical differentiation errors
    const double absTol = 2e-4; // Reasonable tolerance for numerical derivatives
    EXPECT_NEAR(acc.x(), -numericalGrad.x(), std::max(std::abs(acc.x()) * 0.01, absTol));
    EXPECT_NEAR(acc.y(), -numericalGrad.y(), std::max(std::abs(acc.y()) * 0.01, absTol));
    EXPECT_NEAR(acc.z(), -numericalGrad.z(), std::max(std::abs(acc.z()) * 0.01, absTol));
}

// Edge case tests

TEST_F(EarthGravityModelTest, NearSingularity) {
    EarthGravityModel model("SingularityModel", defaultConfig);
    
    // Very close to Earth's center
    Vector3D nearCenter(100.0, 0.0, 0.0); // 100 meters from center
    StateVector state = createStateVector(nearCenter);
    
    Vector3D acc = model.calculateAcceleration(state, testTime);
    
    // Should handle gracefully without crashing
    EXPECT_TRUE(std::isfinite(acc.x()));
    EXPECT_TRUE(std::isfinite(acc.y()));
    EXPECT_TRUE(std::isfinite(acc.z()));
}

TEST_F(EarthGravityModelTest, HighAltitude) {
    EarthGravityModel model("HighAltModel", defaultConfig);
    model.loadCoefficients(testCoefficients);
    
    // Test at lunar distance
    Vector3D lunarDistance(384400000.0, 0.0, 0.0); // ~384,400 km
    StateVector state = createStateVector(lunarDistance);
    
    Vector3D acc = model.calculateAcceleration(state, testTime);
    
    // Should still calculate correctly
    EXPECT_TRUE(std::isfinite(acc.x()));
    EXPECT_TRUE(std::isfinite(acc.y()));
    EXPECT_TRUE(std::isfinite(acc.z()));
    
    // At this distance, should be very close to point mass
    double r = lunarDistance.magnitude();
    double expectedMag = EarthModel::MU / (r * r);
    
    // At lunar distance, higher order terms contribute ~1e-9 relative effect
    EXPECT_NEAR(acc.magnitude(), expectedMag, expectedMag * 1e-5);
}

// Validation tests

TEST_F(EarthGravityModelTest, Validation) {
    EarthGravityModel model;
    
    // Default should be valid
    EXPECT_TRUE(model.validate());
    
    // Test invalid configurations
    EarthGravityConfig invalidConfig;
    invalidConfig.maxDegree = -1;
    EarthGravityModel invalidModel("Invalid", invalidConfig);
    EXPECT_FALSE(invalidModel.validate());
    
    invalidConfig.maxDegree = 10;
    invalidConfig.maxOrder = 11; // Order > degree
    EarthGravityModel invalidModel2("Invalid2", invalidConfig);
    EXPECT_FALSE(invalidModel2.validate());
}

// Clone test

TEST_F(EarthGravityModelTest, Clone) {
    EarthGravityModel model("Original", defaultConfig);
    model.loadCoefficients(testCoefficients);
    model.setEnabled(false);
    
    auto cloned = model.clone();
    
    EXPECT_EQ(cloned->getName(), model.getName());
    EXPECT_EQ(cloned->getType(), model.getType());
    EXPECT_EQ(cloned->isEnabled(), model.isEnabled());
    
    // Cast to verify specific properties
    auto* clonedGravity = dynamic_cast<EarthGravityModel*>(cloned.get());
    ASSERT_NE(clonedGravity, nullptr);
    
    EXPECT_EQ(clonedGravity->getMaxDegree(), model.getMaxDegree());
    EXPECT_EQ(clonedGravity->getMaxOrder(), model.getMaxOrder());
    
    // Should produce same results
    StateVector state = createStateVector(issPosition);
    Vector3D acc1 = model.calculateAcceleration(state, testTime);
    Vector3D acc2 = clonedGravity->calculateAcceleration(state, testTime);
    
    EXPECT_NEAR(acc1.x(), acc2.x(), 1e-15);
    EXPECT_NEAR(acc1.y(), acc2.y(), 1e-15);
    EXPECT_NEAR(acc1.z(), acc2.z(), 1e-15);
}

// Integration test with coefficient loader

TEST_F(EarthGravityModelTest, CoefficientLoaderValidation) {
    // Test file validation
    EXPECT_FALSE(EGM2008CoefficientLoader::validateFile("nonexistent.txt"));
    
    // Create a test coefficient file
    std::string testFile = "test_coeffs.txt";
    std::ofstream file(testFile);
    file << "# Test coefficient file\n";
    file << "2 0 " << -constants::EARTH_J2 << " 0.0\n";
    file << "3 0 " << -constants::EARTH_J3 << " 0.0\n";
    file << "4 0 " << -constants::EARTH_J4 << " 0.0\n";
    file << "2 2 2.43e-6 -1.40e-6\n";
    file.close();
    
    EXPECT_TRUE(EGM2008CoefficientLoader::validateFile(testFile));
    
    // Load coefficients
    std::vector<SphericalHarmonicCoefficient> loadedCoeffs;
    EXPECT_TRUE(EGM2008CoefficientLoader::loadCoefficients(testFile, loadedCoeffs));
    EXPECT_EQ(loadedCoeffs.size(), 4);
    
    // Test with degree/order limits
    loadedCoeffs.clear();
    EXPECT_TRUE(EGM2008CoefficientLoader::loadCoefficients(testFile, loadedCoeffs, 3, 0));
    EXPECT_EQ(loadedCoeffs.size(), 2); // Only J2 and J3
    
    // Clean up
    std::remove(testFile.c_str());
}

// Conservation of energy test

TEST_F(EarthGravityModelTest, EnergyConservation) {
    EarthGravityModel model("EnergyModel", defaultConfig);
    model.loadCoefficients(testCoefficients);
    
    // Create circular orbit state
    double r = issPosition.magnitude();
    double v = std::sqrt(EarthModel::MU / r); // Circular velocity
    Vector3D velocity(0.0, v, 0.0);
    
    StateVector state(issPosition, velocity, 1000.0, testTime);
    
    // Calculate specific energy
    double kineticEnergy = 0.5 * velocity.magnitudeSquared();
    double potentialEnergy = model.calculatePotential(issPosition);
    double totalEnergy = kineticEnergy + potentialEnergy;
    
    // For circular orbit, total energy should be negative
    EXPECT_LT(totalEnergy, 0.0);
    
    // Specific orbital energy for circular orbit: -μ/(2a)
    double expectedEnergy = -EarthModel::MU / (2.0 * r);
    
    // Should be close but not exact due to non-spherical gravity
    EXPECT_NEAR(totalEnergy, expectedEnergy, std::abs(expectedEnergy) * 0.01);
}