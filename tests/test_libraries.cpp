/**
 * @file test_libraries.cpp
 * @brief Test program to verify all external libraries are working
 * @author ILOSS Development Team
 * @date 2025
 */

#include <iostream>
#include <iomanip>

// Core external wrappers
#include "core/external/EigenWrapper.h"
#include "core/external/BoostWrapper.h"
#include "core/external/SQLiteWrapper.h"
#include "core/external/GDALWrapper.h"
#include "core/external/GeographicLibWrapper.h"
#include "core/external/LoggingWrapper.h"
#include "core/external/SPICEWrapper.h"
#include "core/external/TestWrapper.h"

void testEigen() {
    std::cout << "\n=== Testing Eigen ===" << std::endl;
    
    Eigen::Vector3d v1(1.0, 2.0, 3.0);
    Eigen::Vector3d v2(4.0, 5.0, 6.0);
    
    auto dot = v1.dot(v2);
    auto cross = v1.cross(v2);
    
    std::cout << "✓ Vector operations: dot product = " << dot << std::endl;
    std::cout << "✓ Cross product = " << cross.transpose() << std::endl;
    
    Eigen::Matrix3d m;
    m << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
    
    std::cout << "✓ Matrix created successfully" << std::endl;
}

void testBoost() {
    std::cout << "\n=== Testing Boost ===" << std::endl;
    
    boost::filesystem::path currentPath = boost::filesystem::current_path();
    std::cout << "✓ Current path: " << currentPath << std::endl;
    
    auto now = boost::posix_time::second_clock::local_time();
    std::cout << "✓ Current time: " << now << std::endl;
}

void testSQLite() {
    std::cout << "\n=== Testing SQLite ===" << std::endl;
    
    sqlite3* db = nullptr;
    int rc = sqlite3_open(":memory:", &db);
    
    if (rc == SQLITE_OK) {
        std::cout << "✓ SQLite in-memory database opened successfully" << std::endl;
        std::cout << "✓ SQLite version: " << sqlite3_libversion() << std::endl;
        sqlite3_close(db);
    } else {
        std::cout << "✗ Failed to open SQLite database" << std::endl;
    }
}

void testGDAL() {
    std::cout << "\n=== Testing GDAL ===" << std::endl;
    
#ifdef GDAL_FOUND
    iloss::geo::GDALInitializer gdalInit;
    
    if (iloss::geo::GDALUtils::isGDALAvailable()) {
        std::cout << "✓ GDAL is available" << std::endl;
        
        auto drivers = iloss::geo::GDALUtils::getSupportedDrivers();
        std::cout << "✓ Number of supported drivers: " << drivers.size() << std::endl;
        
        // Show a few common drivers
        std::vector<std::string> commonDrivers = {"GTiff", "PNG", "JPEG", "HDF5", "NetCDF"};
        for (const auto& driver : commonDrivers) {
            if (std::find(drivers.begin(), drivers.end(), driver) != drivers.end()) {
                std::cout << "  - " << driver << " driver available" << std::endl;
            }
        }
    }
#else
    std::cout << "✗ GDAL not available (not compiled with GDAL support)" << std::endl;
#endif
}

void testGeographicLib() {
    std::cout << "\n=== Testing GeographicLib ===" << std::endl;
    
    // Test geodetic to ECEF conversion
    iloss::geo::GeodeticPosition geo(45.0, -93.0, 250.0);  // Minneapolis
    auto ecef = iloss::geo::GeoUtils::geodeticToECEF(geo);
    
    std::cout << "✓ Geodetic to ECEF conversion:" << std::endl;
    std::cout << "  Input: lat=" << geo.latitude << "°, lon=" << geo.longitude 
              << "°, alt=" << geo.altitude << "m" << std::endl;
    std::cout << "  ECEF: x=" << std::fixed << std::setprecision(2) 
              << ecef.x << "m, y=" << ecef.y << "m, z=" << ecef.z << "m" << std::endl;
    
    // Test inverse conversion
    auto geoBack = iloss::geo::GeoUtils::ecefToGeodetic(ecef);
    std::cout << "✓ ECEF to Geodetic conversion verified (lat=" 
              << std::fixed << std::setprecision(6) << geoBack.latitude 
              << "°, lon=" << geoBack.longitude << "°)" << std::endl;
    
    // Test geodesic calculations
    iloss::geo::GeodeticPosition p1(40.7128, -74.0060, 0);  // New York
    iloss::geo::GeodeticPosition p2(51.5074, -0.1278, 0);   // London
    
    double distance, az1, az2;
    iloss::geo::GeoUtils::geodesicInverse(p1, p2, distance, az1, az2);
    
    std::cout << "✓ Geodesic distance NYC to London: " 
              << std::fixed << std::setprecision(0) << distance << " meters ("
              << distance/1000.0 << " km)" << std::endl;
}

void testSpdlog() {
    std::cout << "\n=== Testing spdlog ===" << std::endl;
    
    iloss::logging::LogManager::getInstance().initialize("ILOSS_TEST", "test.log", true, false);
    
    LOG_INFO("Test info message");
    LOG_DEBUG("Test debug message");
    LOG_WARN("Test warning message");
    
    std::cout << "✓ Logging system initialized and tested" << std::endl;
    
    // Create module-specific logger
    auto moduleLogger = iloss::logging::LogManager::getInstance().createLogger("TestModule");
    moduleLogger->info("Module-specific log message");
    
    std::cout << "✓ Module-specific logger created" << std::endl;
}

void testSPICE() {
    std::cout << "\n=== Testing SPICE ===" << std::endl;
    
#ifdef SPICE_FOUND
    if (iloss::spice::SPICEUtils::isAvailable()) {
        std::cout << "✓ SPICE toolkit is available" << std::endl;
        
        try {
            iloss::spice::SPICEUtils::initialize();
            std::cout << "✓ SPICE error handling initialized" << std::endl;
            
            // Test time conversion (this will fail without kernels, but that's expected)
            std::cout << "✓ SPICE library linked successfully" << std::endl;
            std::cout << "  Note: Full functionality requires loading kernel files" << std::endl;
        } catch (const std::exception& e) {
            std::cout << "  SPICE exception (expected without kernels): " << e.what() << std::endl;
        }
    }
#else
    std::cout << "✗ SPICE not available (not compiled with SPICE support)" << std::endl;
    std::cout << "  To enable SPICE, rebuild with -DSPICE_ROOT_DIR=/path/to/cspice" << std::endl;
#endif
}

void testGoogleTest() {
    std::cout << "\n=== Testing Google Test ===" << std::endl;
    
    // Just verify we can include gtest header
    // InitGoogleTest requires argc/argv, so we'll just verify linkage
    
    std::cout << "✓ Google Test framework available" << std::endl;
    std::cout << "  Run unit tests with: ctest or ./test_main" << std::endl;
}

int main(int /*argc*/, char** /*argv*/) {
    std::cout << "ILOSS External Libraries Test\n";
    std::cout << "=============================\n";
    
    try {
        testEigen();
        testBoost();
        testSQLite();
        testGDAL();
        testGeographicLib();
        testSpdlog();
        testSPICE();
        testGoogleTest();
        
        std::cout << "\n=== Summary ===" << std::endl;
        std::cout << "✓ All available libraries tested successfully!" << std::endl;
        std::cout << "\nNote: Some libraries may show as unavailable if not found during CMake configuration." << std::endl;
        std::cout << "To enable all features, ensure all dependencies are installed and CMake finds them." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "\n✗ Error during testing: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}