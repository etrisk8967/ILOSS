/**
 * @file test_main.cpp
 * @brief Main test runner for ILOSS
 */

// Define BUILD_TESTS for the test wrapper
#define BUILD_TESTS

#include "core/external/TestWrapper.h"
#include "core/external/EigenWrapper.h"
#include "core/external/SQLiteWrapper.h"
#include "core/external/GeographicLibWrapper.h"
#include "core/external/LoggingWrapper.h"
#include "core/external/GDALWrapper.h"
#include "core/external/SPICEWrapper.h"
#include "core/Platform.h"

#include <iostream>

// Test Eigen integration
TEST(IntegrationTest, EigenBasics) {
    using namespace iloss::math;
    
    Vector3d v1(1.0, 2.0, 3.0);
    Vector3d v2(4.0, 5.0, 6.0);
    
    Vector3d v3 = v1 + v2;
    EXPECT_DOUBLE_EQ(v3.x(), 5.0);
    EXPECT_DOUBLE_EQ(v3.y(), 7.0);
    EXPECT_DOUBLE_EQ(v3.z(), 9.0);
    
    double dot = v1.dot(v2);
    EXPECT_DOUBLE_EQ(dot, 32.0);
    
    Matrix3d m = Matrix3d::Identity();
    Vector3d v4 = m * v1;
    EXPECT_VECTOR3_NEAR(v1, v4, 1e-10);
}

// Test SQLite integration
TEST(IntegrationTest, SQLiteBasics) {
    using namespace iloss::db;
    
    // Create in-memory database
    SQLiteDatabase db(":memory:");
    ASSERT_TRUE(db);
    
    // Create table
    db.execute("CREATE TABLE test (id INTEGER PRIMARY KEY, value TEXT)");
    
    // Insert data
    SQLiteStatement insertStmt(db, "INSERT INTO test (value) VALUES (?)");
    insertStmt.bind(1, "Hello ILOSS");
    insertStmt.execute();
    
    // Query data
    SQLiteStatement selectStmt(db, "SELECT id, value FROM test");
    ASSERT_TRUE(selectStmt.step());
    EXPECT_EQ(selectStmt.getInt(0), 1);
    EXPECT_EQ(selectStmt.getString(1), "Hello ILOSS");
    EXPECT_FALSE(selectStmt.step());
}

// Test platform detection
TEST(IntegrationTest, PlatformDetection) {
    std::cout << "Platform: " << iloss::getPlatformName() << std::endl;
    std::cout << "Architecture: " << iloss::getArchitectureName() << std::endl;
    std::cout << "Compiler: " << iloss::getCompilerName() << std::endl;
    
#ifdef ILOSS_PLATFORM_LINUX
    EXPECT_STREQ(iloss::getPlatformName(), "Linux");
#elif defined(ILOSS_PLATFORM_WINDOWS)
    EXPECT_STREQ(iloss::getPlatformName(), "Windows");
#endif
}

// Test GeographicLib integration
TEST(IntegrationTest, GeographicLibBasics) {
    using namespace iloss::geo;
    
    // Test geodetic to ECEF conversion
    GeodeticPosition geo(45.0, -122.0, 1000.0);  // 45°N, 122°W, 1000m altitude
    ECEFPosition ecef = GeoUtils::geodeticToECEF(geo);
    
    // Verify conversion (approximate values)
    EXPECT_NEAR(ecef.x, -2.4e6, 1e5);  // X coordinate
    EXPECT_NEAR(ecef.y, -3.8e6, 1e5);  // Y coordinate (corrected)
    EXPECT_NEAR(ecef.z, 4.5e6, 1e5);   // Z coordinate
    
    // Test inverse conversion
    GeodeticPosition geo2 = GeoUtils::ecefToGeodetic(ecef);
    EXPECT_NEAR(geo2.latitude, geo.latitude, 1e-6);
    EXPECT_NEAR(geo2.longitude, geo.longitude, 1e-6);
    EXPECT_NEAR(geo2.altitude, geo.altitude, 1e-3);
    
    // Test geodesic calculations
    GeodeticPosition p1(40.0, -74.0, 0.0);  // New York area
    GeodeticPosition p2(51.5, -0.1, 0.0);   // London area
    
    double distance, azimuth1, azimuth2;
    GeoUtils::geodesicInverse(p1, p2, distance, azimuth1, azimuth2);
    
    // Distance should be roughly 5,500 km
    EXPECT_GT(distance, 5.4e6);
    EXPECT_LT(distance, 5.7e6);
}

// Test spdlog integration
TEST(IntegrationTest, LoggingBasics) {
    using namespace iloss::logging;
    
    // Initialize logging to a test file
    LogManager::getInstance().initialize("ILOSS_TEST", "test.log", false, true);
    
    // Test different log levels
    LOG_TRACE("This is a trace message");
    LOG_DEBUG("This is a debug message");
    LOG_INFO("Integration test for logging");
    LOG_WARN("This is a warning");
    LOG_ERROR("This is an error (not really)");
    
    // Test module-specific logger
    auto moduleLogger = LogManager::getInstance().createLogger("TestModule");
    moduleLogger->info("Module-specific log message");
    
    // Flush logs
    LogManager::getInstance().flush();
    
    // Verify we can get the logger
    EXPECT_NE(LogManager::getInstance().getLogger(), nullptr);
}

// Test GDAL availability
TEST(IntegrationTest, GDALAvailability) {
    using namespace iloss::geo;
    
    // Check if GDAL is available
    bool gdalAvailable = GDALUtils::isGDALAvailable();
    std::cout << "GDAL available: " << (gdalAvailable ? "Yes" : "No") << std::endl;
    
    if (gdalAvailable) {
        GDALInitializer gdal;
        auto drivers = GDALUtils::getSupportedDrivers();
        std::cout << "GDAL supports " << drivers.size() << " drivers" << std::endl;
        EXPECT_GT(drivers.size(), 0);
    }
}

// Test SPICE availability
TEST(IntegrationTest, SPICEAvailability) {
    using namespace iloss::spice;
    
    bool spiceAvailable = SPICEUtils::isAvailable();
    std::cout << "SPICE available: " << (spiceAvailable ? "Yes" : "No") << std::endl;
    
    if (!spiceAvailable) {
        std::cout << "SPICE toolkit not configured. "
                  << "Download from https://naif.jpl.nasa.gov/naif/toolkit_C.html" << std::endl;
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    std::cout << "ILOSS Integration Test Suite\n";
    std::cout << "============================\n";
    
    return RUN_ALL_TESTS();
}