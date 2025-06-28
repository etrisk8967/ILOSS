#pragma once

#include <gtest/gtest.h>
#include "core/logging/Logger.h"
#include "core/config/ConfigManager.h"
#include <filesystem>
#include <random>
#include <memory>
#include <nlohmann/json.hpp>

namespace iloss {
namespace test {

/**
 * @brief Base test fixture providing common functionality for all ILOSS tests
 * 
 * This fixture provides:
 * - Automatic test directory setup and cleanup
 * - Logging configuration for tests
 * - Random number generation utilities
 * - File system helpers
 * - Performance timing utilities
 */
class TestFixtureBase : public ::testing::Test {
protected:
    void SetUp() override {
        // Create unique test directory
        m_testDir = createTestDirectory();
        
        // Initialize logging for tests
        initializeTestLogging();
        
        // Initialize random number generator with fixed seed for reproducibility
        m_rng = std::make_unique<std::mt19937>(m_randomSeed);
        
        // Record test start time
        m_testStartTime = std::chrono::steady_clock::now();
    }
    
    void TearDown() override {
        // Record test end time
        auto testEndTime = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            testEndTime - m_testStartTime).count();
        
        // Log test duration if verbose
        if (m_verboseLogging) {
            LOG_INFO("Test completed in {} ms", duration);
        }
        
        // Clean up test directory if requested
        if (m_cleanupTestDir && std::filesystem::exists(m_testDir)) {
            std::filesystem::remove_all(m_testDir);
        }
    }
    
    // Test directory management
    std::filesystem::path getTestDir() const { return m_testDir; }
    std::filesystem::path getTestDataPath(const std::string& filename) const {
        return m_testDir / filename;
    }
    
    void setCleanupTestDir(bool cleanup) { m_cleanupTestDir = cleanup; }
    
    // Random number generation
    template<typename T>
    T randomUniform(T min, T max) {
        if constexpr (std::is_integral_v<T>) {
            std::uniform_int_distribution<T> dist(min, max);
            return dist(*m_rng);
        } else {
            std::uniform_real_distribution<T> dist(min, max);
            return dist(*m_rng);
        }
    }
    
    template<typename T>
    T randomNormal(T mean, T stddev) {
        std::normal_distribution<T> dist(mean, stddev);
        return dist(*m_rng);
    }
    
    void setRandomSeed(unsigned int seed) {
        m_randomSeed = seed;
        m_rng->seed(seed);
    }
    
    // File helpers
    void writeTestFile(const std::string& filename, const std::string& content) {
        auto path = getTestDataPath(filename);
        std::ofstream file(path);
        ASSERT_TRUE(file.is_open()) << "Failed to create test file: " << path;
        file << content;
    }
    
    std::string readTestFile(const std::string& filename) {
        auto path = getTestDataPath(filename);
        std::ifstream file(path);
        EXPECT_TRUE(file.is_open()) << "Failed to open test file: " << path;
        std::stringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }
    
    // Performance helpers
    void startTimer(const std::string& name = "default") {
        m_timers[name] = std::chrono::steady_clock::now();
    }
    
    double stopTimer(const std::string& name = "default") {
        auto it = m_timers.find(name);
        if (it == m_timers.end()) {
            return 0.0;
        }
        
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            now - it->second).count();
        m_timers.erase(it);
        return duration / 1000.0; // Return milliseconds
    }
    
    // Logging control
    void setVerboseLogging(bool verbose) { m_verboseLogging = verbose; }
    
    // Common assertions for floating point comparisons
    void expectNear(double actual, double expected, double tolerance, 
                   const std::string& message = "") {
        EXPECT_NEAR(actual, expected, tolerance) << message;
    }
    
    void expectVectorNear(const std::vector<double>& actual,
                         const std::vector<double>& expected,
                         double tolerance) {
        ASSERT_EQ(actual.size(), expected.size()) 
            << "Vector sizes don't match";
        for (size_t i = 0; i < actual.size(); ++i) {
            EXPECT_NEAR(actual[i], expected[i], tolerance)
                << "Mismatch at index " << i;
        }
    }
    
protected:
    // Configuration for derived fixtures
    bool m_cleanupTestDir = true;
    bool m_verboseLogging = false;
    unsigned int m_randomSeed = 12345;
    
private:
    std::filesystem::path createTestDirectory() {
        auto testInfo = ::testing::UnitTest::GetInstance()->current_test_info();
        std::string dirName = std::string(testInfo->test_suite_name()) + "_" +
                             std::string(testInfo->name()) + "_" +
                             std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
        
        auto testDir = std::filesystem::temp_directory_path() / "iloss_tests" / dirName;
        std::filesystem::create_directories(testDir);
        return testDir;
    }
    
    void initializeTestLogging() {
        // Configure logging for tests - less verbose by default
        // Note: Logger API may need adjustment based on actual implementation
        auto& logger = logging::Logger::getInstance();
        
        // TODO: Configure logging level and file output when Logger API supports it
        // For now, just get the logger instance
        (void)logger; // Suppress unused variable warning
        (void)m_verboseLogging; // Suppress unused variable warning
    }
    
    std::filesystem::path m_testDir;
    std::unique_ptr<std::mt19937> m_rng;
    std::chrono::steady_clock::time_point m_testStartTime;
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> m_timers;
};

/**
 * @brief Test fixture for physics-related tests
 * 
 * Provides additional utilities specific to physics simulations
 */
class PhysicsTestFixture : public TestFixtureBase {
protected:
    void SetUp() override {
        TestFixtureBase::SetUp();
        
        // Set up common physics test parameters
        m_positionTolerance = 1e-9;  // meters
        m_velocityTolerance = 1e-12; // m/s
        m_timeTolerance = 1e-9;      // seconds
        m_angleTolerance = 1e-12;    // radians
    }
    
    // Physics-specific comparison helpers
    void expectPositionNear(double actual, double expected, 
                           const std::string& message = "") {
        expectNear(actual, expected, m_positionTolerance, 
                  "Position comparison failed: " + message);
    }
    
    void expectVelocityNear(double actual, double expected,
                           const std::string& message = "") {
        expectNear(actual, expected, m_velocityTolerance,
                  "Velocity comparison failed: " + message);
    }
    
    void expectAngleNear(double actual, double expected,
                        const std::string& message = "") {
        // Handle angle wrapping
        double diff = std::fmod(actual - expected + M_PI, 2 * M_PI) - M_PI;
        EXPECT_NEAR(std::abs(diff), 0.0, m_angleTolerance)
            << "Angle comparison failed: " << message;
    }
    
    // Generate test orbits
    struct TestOrbit {
        double semiMajorAxis;
        double eccentricity;
        double inclination;
        double raan;
        double argOfPerigee;
        double trueAnomaly;
    };
    
    TestOrbit generateRandomOrbit() {
        return {
            randomUniform(6500000.0, 50000000.0),  // 6500-50000 km
            randomUniform(0.0, 0.9),               // e < 0.9
            randomUniform(0.0, M_PI),              // 0-180 degrees
            randomUniform(0.0, 2 * M_PI),          // 0-360 degrees
            randomUniform(0.0, 2 * M_PI),          // 0-360 degrees
            randomUniform(0.0, 2 * M_PI)           // 0-360 degrees
        };
    }
    
protected:
    double m_positionTolerance;
    double m_velocityTolerance;
    double m_timeTolerance;
    double m_angleTolerance;
};

/**
 * @brief Test fixture for database-related tests
 * 
 * Provides database setup and teardown functionality
 */
class DatabaseTestFixture : public TestFixtureBase {
protected:
    void SetUp() override {
        TestFixtureBase::SetUp();
        
        // Create test database
        m_dbPath = getTestDataPath("test.db");
        m_dbPathStr = m_dbPath.string();
        
        // Initialize database manager with test database
        initializeTestDatabase();
    }
    
    void TearDown() override {
        // Close database connections
        cleanupTestDatabase();
        
        TestFixtureBase::TearDown();
    }
    
    const std::string& getDbPath() const { return m_dbPathStr; }
    
    // Database utilities
    void executeSQL(const std::string& sql);
    bool tableExists(const std::string& tableName);
    int getRowCount(const std::string& tableName);
    
protected:
    virtual void initializeTestDatabase() {
        // Override in derived classes to set up schema
    }
    
    virtual void cleanupTestDatabase() {
        // Override in derived classes for custom cleanup
    }
    
private:
    std::filesystem::path m_dbPath;
    std::string m_dbPathStr;
};

/**
 * @brief Test fixture for configuration-related tests
 */
class ConfigTestFixture : public TestFixtureBase {
protected:
    void SetUp() override {
        TestFixtureBase::SetUp();
        
        // Create test config directory
        m_configDir = getTestDir() / "config";
        std::filesystem::create_directories(m_configDir);
        
        // Reset configuration manager
        config::ConfigManager::getInstance().reset();
    }
    
    void createTestConfig(const std::string& name, const nlohmann::json& content) {
        auto path = m_configDir / (name + ".json");
        std::ofstream file(path);
        file << content.dump(4);
    }
    
    std::filesystem::path getConfigPath(const std::string& name) const {
        return m_configDir / (name + ".json");
    }
    
protected:
    std::filesystem::path m_configDir;
};

} // namespace test
} // namespace iloss