#include <gtest/gtest.h>
#include "physics/aerodynamics/AerodynamicDatabase.h"
#include <sstream>
#include <fstream>
#include <filesystem>

using namespace iloss::physics::aerodynamics;

class AerodynamicDatabaseTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test CSV data
        testCSVData = R"(# Test aerodynamic data
# Mach, AoA_deg, CD, CL, CY, Cl, Cm, Cn
0.0, 0.0, 0.30, 0.00, 0.0, 0.00, 0.00, 0.0
0.0, 5.0, 0.32, 0.16, 0.0, 0.00, -0.01, 0.0
0.5, 0.0, 0.31, 0.00, 0.0, 0.00, 0.00, 0.0
0.5, 5.0, 0.33, 0.17, 0.0, 0.00, -0.012, 0.0
1.0, 0.0, 0.40, 0.00, 0.0, 0.00, 0.00, 0.0
1.0, 5.0, 0.42, 0.19, 0.0, 0.00, -0.015, 0.0)";

        // Create temporary test file
        testFileName = "test_aero_data.csv";
        std::ofstream file(testFileName);
        file << testCSVData;
        file.close();
    }

    void TearDown() override {
        // Remove temporary test file
        if (std::filesystem::exists(testFileName)) {
            std::filesystem::remove(testFileName);
        }
    }

    std::string testCSVData;
    std::string testFileName;
};

TEST_F(AerodynamicDatabaseTest, DefaultConstructor) {
    AerodynamicDatabase db;
    
    // Should have no configurations initially
    EXPECT_EQ(db.getConfigurationNames().size(), 0);
}

TEST_F(AerodynamicDatabaseTest, ConfiguredConstructor) {
    AerodynamicDatabase::Config config;
    config.enableCaching = false;
    config.cacheSize = 500;
    config.allowExtrapolation = false;
    
    AerodynamicDatabase db(config);
    EXPECT_EQ(db.getConfigurationNames().size(), 0);
}

TEST_F(AerodynamicDatabaseTest, LoadFromString) {
    AerodynamicDatabase db;
    
    ASSERT_NO_THROW(db.loadFromString(testCSVData));
    
    // Check configuration was loaded
    EXPECT_TRUE(db.hasConfiguration("default"));
    EXPECT_EQ(db.getConfigurationNames().size(), 1);
}

TEST_F(AerodynamicDatabaseTest, LoadFromCSVFile) {
    AerodynamicDatabase db;
    
    ASSERT_NO_THROW(db.loadFromCSV(testFileName));
    
    EXPECT_TRUE(db.hasConfiguration("default"));
}

TEST_F(AerodynamicDatabaseTest, LoadMultipleConfigurations) {
    AerodynamicDatabase db;
    
    db.loadFromString(testCSVData, "clean");
    db.loadFromString(testCSVData, "gear_down");
    
    EXPECT_TRUE(db.hasConfiguration("clean"));
    EXPECT_TRUE(db.hasConfiguration("gear_down"));
    EXPECT_FALSE(db.hasConfiguration("default"));
    
    auto configs = db.getConfigurationNames();
    EXPECT_EQ(configs.size(), 2);
}

TEST_F(AerodynamicDatabaseTest, GetCoefficientsExactMatch) {
    AerodynamicDatabase db;
    db.loadFromString(testCSVData);
    
    // Test exact match at Mach 0.5, AoA 5 degrees
    double aoa_rad = 5.0 * M_PI / 180.0;
    AerodynamicCoefficients coeffs = db.getCoefficients(0.5, aoa_rad);
    
    EXPECT_NEAR(coeffs.getDragCoefficient(), 0.33, 1e-6);
    EXPECT_NEAR(coeffs.getLiftCoefficient(), 0.17, 1e-6);
    EXPECT_NEAR(coeffs.getPitchMomentCoefficient(), -0.012, 1e-6);
}

TEST_F(AerodynamicDatabaseTest, GetCoefficientsInterpolation) {
    AerodynamicDatabase db;
    db.loadFromString(testCSVData);
    
    // Test interpolation at Mach 0.25, AoA 0
    AerodynamicCoefficients coeffs = db.getCoefficients(0.25, 0.0);
    
    // Should interpolate between Mach 0.0 and 0.5
    EXPECT_NEAR(coeffs.getDragCoefficient(), 0.305, 1e-6);  // (0.30 + 0.31) / 2
    EXPECT_NEAR(coeffs.getLiftCoefficient(), 0.0, 1e-6);
}

TEST_F(AerodynamicDatabaseTest, GetCoefficients2DInterpolation) {
    AerodynamicDatabase db;
    db.loadFromString(testCSVData);
    
    // Test 2D interpolation at Mach 0.25, AoA 2.5 degrees
    double aoa_rad = 2.5 * M_PI / 180.0;
    AerodynamicCoefficients coeffs = db.getCoefficients(0.25, aoa_rad);
    
    // This requires bilinear interpolation between all four corners
    // Expected values calculated manually
    EXPECT_GT(coeffs.getDragCoefficient(), 0.30);
    EXPECT_LT(coeffs.getDragCoefficient(), 0.33);
    EXPECT_GT(coeffs.getLiftCoefficient(), 0.0);
    EXPECT_LT(coeffs.getLiftCoefficient(), 0.17);
}

TEST_F(AerodynamicDatabaseTest, GetDataRange) {
    AerodynamicDatabase db;
    db.loadFromString(testCSVData);
    
    double minMach, maxMach, minAoA, maxAoA;
    EXPECT_TRUE(db.getDataRange("default", minMach, maxMach, minAoA, maxAoA));
    
    EXPECT_DOUBLE_EQ(minMach, 0.0);
    EXPECT_DOUBLE_EQ(maxMach, 1.0);
    EXPECT_DOUBLE_EQ(minAoA, 0.0);
    EXPECT_NEAR(maxAoA, 5.0 * M_PI / 180.0, 1e-6);
}

TEST_F(AerodynamicDatabaseTest, ExtrapolationHandling) {
    AerodynamicDatabase::Config config;
    config.allowExtrapolation = true;
    config.extrapolationWarningThreshold = 0.1;
    
    AerodynamicDatabase db(config);
    db.loadFromString(testCSVData);
    
    // Test extrapolation beyond data range
    // Should not throw with extrapolation allowed
    EXPECT_NO_THROW(db.getCoefficients(2.0, 0.0));  // Mach 2.0 is beyond range
    EXPECT_NO_THROW(db.getCoefficients(0.5, 0.2));  // ~11.5 degrees, beyond range
}

TEST_F(AerodynamicDatabaseTest, NoExtrapolationThrows) {
    AerodynamicDatabase::Config config;
    config.allowExtrapolation = false;
    
    AerodynamicDatabase db(config);
    db.loadFromString(testCSVData);
    
    // Should throw when trying to extrapolate
    EXPECT_THROW(db.getCoefficients(2.0, 0.0), std::runtime_error);
}

TEST_F(AerodynamicDatabaseTest, InvalidConfigurationThrows) {
    AerodynamicDatabase db;
    db.loadFromString(testCSVData);
    
    // Should throw for non-existent configuration
    EXPECT_THROW(db.getCoefficients(0.5, 0.0, "nonexistent"), std::runtime_error);
}

TEST_F(AerodynamicDatabaseTest, EmptyDataThrows) {
    AerodynamicDatabase db;
    
    // Should throw when no data loaded
    EXPECT_THROW(db.getCoefficients(0.5, 0.0), std::runtime_error);
}

TEST_F(AerodynamicDatabaseTest, ParseErrors) {
    AerodynamicDatabase db;
    
    // Test malformed CSV
    std::string badCSV = "# Bad data\nNot,Valid,Numbers";
    EXPECT_THROW(db.loadFromString(badCSV), std::runtime_error);
    
    // Test insufficient columns
    std::string shortCSV = "# Short data\n0.0, 0.0";
    EXPECT_THROW(db.loadFromString(shortCSV), std::runtime_error);
}

TEST_F(AerodynamicDatabaseTest, CachingBehavior) {
    AerodynamicDatabase::Config config;
    config.enableCaching = true;
    config.cacheSize = 10;
    
    AerodynamicDatabase db(config);
    db.loadFromString(testCSVData);
    
    // First call - cache miss
    AerodynamicCoefficients coeffs1 = db.getCoefficients(0.3, 0.05);
    
    // Second call with same parameters - should hit cache
    AerodynamicCoefficients coeffs2 = db.getCoefficients(0.3, 0.05);
    
    EXPECT_EQ(coeffs1.getDragCoefficient(), coeffs2.getDragCoefficient());
    
    // Check cache statistics
    size_t hits, misses, size;
    db.getCacheStats(hits, misses, size);
    
    EXPECT_EQ(hits, 1);
    EXPECT_EQ(misses, 1);
    EXPECT_EQ(size, 1);
}

TEST_F(AerodynamicDatabaseTest, ClearCache) {
    AerodynamicDatabase db;
    db.loadFromString(testCSVData);
    
    // Generate some cache entries
    db.getCoefficients(0.3, 0.05);
    db.getCoefficients(0.4, 0.06);
    
    size_t hits, misses, size;
    db.getCacheStats(hits, misses, size);
    EXPECT_GT(size, 0);
    
    // Clear cache
    db.clearCache();
    
    db.getCacheStats(hits, misses, size);
    EXPECT_EQ(size, 0);
}

TEST_F(AerodynamicDatabaseTest, ValidateData) {
    AerodynamicDatabase db;
    db.loadFromString(testCSVData);
    
    EXPECT_TRUE(db.validateData("default"));
}

TEST_F(AerodynamicDatabaseTest, ExportToCSV) {
    AerodynamicDatabase db;
    db.loadFromString(testCSVData);
    
    std::string exportFile = "test_export.csv";
    
    ASSERT_NO_THROW(db.exportToCSV(exportFile));
    
    // Verify file was created
    EXPECT_TRUE(std::filesystem::exists(exportFile));
    
    // Clean up
    std::filesystem::remove(exportFile);
}

TEST_F(AerodynamicDatabaseTest, SparseDataHandling) {
    // Test with non-rectangular grid (missing data points)
    std::string sparseData = R"(# Sparse data
0.0, 0.0, 0.30, 0.00, 0.0, 0.00, 0.00, 0.0
0.0, 5.0, 0.32, 0.16, 0.0, 0.00, -0.01, 0.0
1.0, 0.0, 0.40, 0.00, 0.0, 0.00, 0.00, 0.0
1.0, 5.0, 0.42, 0.19, 0.0, 0.00, -0.015, 0.0)";
    
    AerodynamicDatabase db;
    db.loadFromString(sparseData);
    
    // Should still work with nearest neighbor fallback
    EXPECT_NO_THROW(db.getCoefficients(0.5, 2.5 * M_PI / 180.0));
}

TEST_F(AerodynamicDatabaseTest, LargeDataset) {
    // Generate a larger dataset for performance testing
    std::stringstream largeData;
    largeData << "# Large dataset\n";
    
    for (double mach = 0.0; mach <= 5.0; mach += 0.1) {
        for (double aoa = -20.0; aoa <= 20.0; aoa += 2.0) {
            largeData << mach << ", " << aoa << ", "
                     << 0.3 + 0.1 * mach << ", "  // CD
                     << 0.1 * aoa / 5.0 << ", "    // CL
                     << "0.0, 0.0, "
                     << -0.01 * aoa / 5.0 << ", "  // Cm
                     << "0.0\n";
        }
    }
    
    AerodynamicDatabase db;
    ASSERT_NO_THROW(db.loadFromString(largeData.str()));
    
    // Test performance with many lookups
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < 1000; ++i) {
        double mach = 2.5;
        double aoa = 0.05 * (i % 20 - 10);  // -0.5 to 0.5 rad
        db.getCoefficients(mach, aoa);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    // Should complete 1000 lookups in reasonable time (< 100ms)
    EXPECT_LT(duration.count(), 100);
}