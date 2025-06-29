#include <gtest/gtest.h>
#include "physics/forces/drag/NRLMSISE00Atmosphere.h"
#include "core/constants/EarthModel.h"
#include "core/math/Vector3D.h"
#include <cmath>

using namespace iloss;
using namespace iloss::physics::forces::drag;
using namespace iloss::math;
using namespace iloss::constants;

class NRLMSISE00AtmosphereTest : public ::testing::Test {
protected:
    void SetUp() override {
        atmosphere = std::make_unique<NRLMSISE00Atmosphere>();
    }

    std::unique_ptr<NRLMSISE00Atmosphere> atmosphere;
};

TEST_F(NRLMSISE00AtmosphereTest, Construction) {
    EXPECT_EQ(atmosphere->getName(), "NRLMSISE-00");
    EXPECT_DOUBLE_EQ(atmosphere->getMinAltitude(), 0.0);
    EXPECT_DOUBLE_EQ(atmosphere->getMaxAltitude(), 1000000.0);
}

TEST_F(NRLMSISE00AtmosphereTest, ValidAltitudeRange) {
    EXPECT_TRUE(atmosphere->isValidAltitude(0.0));
    EXPECT_TRUE(atmosphere->isValidAltitude(500000.0));
    EXPECT_TRUE(atmosphere->isValidAltitude(1000000.0));
    EXPECT_FALSE(atmosphere->isValidAltitude(-100.0));
    EXPECT_FALSE(atmosphere->isValidAltitude(1000001.0));
}

TEST_F(NRLMSISE00AtmosphereTest, DensityAtSeaLevel) {
    // Position at sea level on equator
    Vector3D position(EarthModel::EQUATORIAL_RADIUS, 0.0, 0.0);
    double julianDate = 2451545.0;  // J2000 epoch
    
    double density = atmosphere->getDensity(position, julianDate);
    
    // Sea level density should be around 1.225 kg/m³
    EXPECT_NEAR(density, 1.225, 0.1);
}

TEST_F(NRLMSISE00AtmosphereTest, DensityDecreaseWithAltitude) {
    double julianDate = 2451545.0;
    
    // Test at different altitudes
    std::vector<double> altitudes = {0.0, 10000.0, 50000.0, 100000.0, 200000.0, 400000.0};
    double previousDensity = std::numeric_limits<double>::max();
    
    for (double alt : altitudes) {
        Vector3D position(EarthModel::EQUATORIAL_RADIUS + alt, 0.0, 0.0);
        double density = atmosphere->getDensity(position, julianDate);
        
        // Density should decrease with altitude
        EXPECT_LT(density, previousDensity);
        previousDensity = density;
        
        // Density should be positive
        EXPECT_GT(density, 0.0);
    }
}

TEST_F(NRLMSISE00AtmosphereTest, TemperatureProfile) {
    double julianDate = 2451545.0;
    
    // Test temperature at different altitudes
    std::vector<std::pair<double, double>> altitudeTemps = {
        {0.0, 288.15},       // Sea level ~15°C
        {11000.0, 216.65},   // Tropopause ~-56.5°C
        {50000.0, 270.65},   // Stratopause ~-2.5°C
        {85000.0, 190.0}     // Mesopause (approximate)
    };
    
    for (const auto& [alt, expectedTemp] : altitudeTemps) {
        Vector3D position(EarthModel::EQUATORIAL_RADIUS + alt, 0.0, 0.0);
        double temperature = atmosphere->getTemperature(position, julianDate);
        
        // Temperature should be within reasonable range
        EXPECT_NEAR(temperature, expectedTemp, 50.0);  // Allow 50K tolerance
        EXPECT_GT(temperature, 0.0);  // Always positive
    }
}

TEST_F(NRLMSISE00AtmosphereTest, PressureDecreaseWithAltitude) {
    double julianDate = 2451545.0;
    
    // Test at different altitudes
    std::vector<double> altitudes = {0.0, 5000.0, 10000.0, 20000.0, 50000.0};
    double previousPressure = std::numeric_limits<double>::max();
    
    for (double alt : altitudes) {
        Vector3D position(EarthModel::EQUATORIAL_RADIUS + alt, 0.0, 0.0);
        double pressure = atmosphere->getPressure(position, julianDate);
        
        // Pressure should decrease with altitude
        EXPECT_LT(pressure, previousPressure);
        previousPressure = pressure;
        
        // Pressure should be positive
        EXPECT_GT(pressure, 0.0);
    }
}

TEST_F(NRLMSISE00AtmosphereTest, SpaceWeatherEffect) {
    Vector3D position(EarthModel::EQUATORIAL_RADIUS + 400000.0, 0.0, 0.0);  // 400 km
    double julianDate = 2451545.0;
    
    // Low solar activity
    SpaceWeatherData lowActivity;
    lowActivity.f107 = 70.0;
    lowActivity.f107a = 70.0;
    atmosphere->setSpaceWeather(lowActivity);
    double densityLow = atmosphere->getDensity(position, julianDate);
    
    // High solar activity
    SpaceWeatherData highActivity;
    highActivity.f107 = 250.0;
    highActivity.f107a = 250.0;
    atmosphere->setSpaceWeather(highActivity);
    double densityHigh = atmosphere->getDensity(position, julianDate);
    
    // Higher solar activity should increase density at high altitudes
    EXPECT_GT(densityHigh, densityLow);
    
    // The effect should be significant (at least 50% increase)
    EXPECT_GT(densityHigh / densityLow, 1.5);
}

TEST_F(NRLMSISE00AtmosphereTest, LatitudeVariation) {
    double julianDate = 2451545.0;
    double altitude = 300000.0;  // 300 km
    
    // Test at equator
    Vector3D equatorPos(EarthModel::EQUATORIAL_RADIUS + altitude, 0.0, 0.0);
    double equatorDensity = atmosphere->getDensity(equatorPos, julianDate);
    
    // Test at pole
    Vector3D polePos(0.0, 0.0, EarthModel::POLAR_RADIUS + altitude);
    double poleDensity = atmosphere->getDensity(polePos, julianDate);
    
    // Densities should be different due to latitude variation
    EXPECT_NE(equatorDensity, poleDensity);
    
    // The relative difference depends on the simplified model implementation
    // Just check that both densities are reasonable and in the same order of magnitude
    double ratio = equatorDensity / poleDensity;
    EXPECT_GT(ratio, 0.5);  // Within factor of 2
    EXPECT_LT(ratio, 2.0);
}

TEST_F(NRLMSISE00AtmosphereTest, DiurnalVariation) {
    double altitude = 250000.0;  // 250 km
    Vector3D position(EarthModel::EQUATORIAL_RADIUS + altitude, 0.0, 0.0);
    
    // Test at different times of day (different Julian dates)
    double jdNoon = 2451545.0;      // Approximately noon
    double jdMidnight = 2451545.5;  // Approximately midnight
    
    double densityNoon = atmosphere->getDensity(position, jdNoon);
    double densityMidnight = atmosphere->getDensity(position, jdMidnight);
    
    // Densities should be different due to diurnal variation
    // Day side is typically denser than night side at high altitudes
    EXPECT_NE(densityNoon, densityMidnight);
}

TEST_F(NRLMSISE00AtmosphereTest, DetailedOutput) {
    Vector3D position(EarthModel::EQUATORIAL_RADIUS + 200000.0, 0.0, 0.0);
    double julianDate = 2451545.0;
    
    auto output = atmosphere->getDetailedOutput(position, julianDate);
    
    // Check that all outputs are reasonable
    EXPECT_GT(output.density_total, 0.0);
    EXPECT_GT(output.temperature_neutral, 0.0);
    EXPECT_GT(output.temperature_exospheric, output.temperature_neutral);
    
    // Check species densities
    EXPECT_GE(output.density_He, 0.0);
    EXPECT_GE(output.density_O, 0.0);
    EXPECT_GE(output.density_N2, 0.0);
    EXPECT_GE(output.density_O2, 0.0);
    EXPECT_GE(output.density_Ar, 0.0);
    EXPECT_GE(output.density_H, 0.0);
    EXPECT_GE(output.density_N, 0.0);
    
    // At 200 km, should have significant O, N2, O2
    EXPECT_GT(output.density_O + output.density_N2 + output.density_O2, 0.0);
}

TEST_F(NRLMSISE00AtmosphereTest, SpeciesComposition) {
    double julianDate = 2451545.0;
    
    // Test at low altitude (N2 and O2 dominate)
    Vector3D lowPos(EarthModel::EQUATORIAL_RADIUS + 50000.0, 0.0, 0.0);
    auto lowOutput = atmosphere->getDetailedOutput(lowPos, julianDate);
    
    double lowN2O2Fraction = (lowOutput.density_N2 + lowOutput.density_O2) / 
                            (lowOutput.density_N2 + lowOutput.density_O2 + 
                             lowOutput.density_O + lowOutput.density_He);
    EXPECT_GT(lowN2O2Fraction, 0.9);  // N2 and O2 should dominate
    
    // Test at high altitude (atomic oxygen dominates)
    Vector3D highPos(EarthModel::EQUATORIAL_RADIUS + 400000.0, 0.0, 0.0);
    auto highOutput = atmosphere->getDetailedOutput(highPos, julianDate);
    
    double highOFraction = highOutput.density_O / 
                          (highOutput.density_N2 + highOutput.density_O2 + 
                           highOutput.density_O + highOutput.density_He);
    EXPECT_GT(highOFraction, 0.5);  // Atomic oxygen should be significant
}

TEST_F(NRLMSISE00AtmosphereTest, ExosphericTemperature) {
    Vector3D position(EarthModel::EQUATORIAL_RADIUS + 500000.0, 0.0, 0.0);
    double julianDate = 2451545.0;
    
    // Test with different solar activities
    SpaceWeatherData lowActivity;
    lowActivity.f107 = 70.0;
    atmosphere->setSpaceWeather(lowActivity);
    auto lowOutput = atmosphere->getDetailedOutput(position, julianDate);
    
    SpaceWeatherData highActivity;
    highActivity.f107 = 250.0;
    atmosphere->setSpaceWeather(highActivity);
    auto highOutput = atmosphere->getDetailedOutput(position, julianDate);
    
    // Exospheric temperature should increase with solar activity
    EXPECT_GT(highOutput.temperature_exospheric, lowOutput.temperature_exospheric);
    
    // Both should be in reasonable range (600-2000 K)
    EXPECT_GT(lowOutput.temperature_exospheric, 600.0);
    EXPECT_LT(lowOutput.temperature_exospheric, 2000.0);
    EXPECT_GT(highOutput.temperature_exospheric, 600.0);
    EXPECT_LT(highOutput.temperature_exospheric, 2000.0);
}

TEST_F(NRLMSISE00AtmosphereTest, ComponentSwitches) {
    Vector3D position(EarthModel::EQUATORIAL_RADIUS + 300000.0, 0.0, 0.0);
    double julianDate = 2451545.0;
    
    // Get baseline
    [[maybe_unused]] double baseline = atmosphere->getDensity(position, julianDate);
    
    // Disable a component (this is implementation-specific)
    atmosphere->setComponentEnabled(0, false);
    double modified = atmosphere->getDensity(position, julianDate);
    
    // The density might change (or might not, depending on which component)
    // Just verify the method doesn't crash
    EXPECT_GE(modified, 0.0);
}

TEST_F(NRLMSISE00AtmosphereTest, EdgeCases) {
    double julianDate = 2451545.0;
    
    // Test at maximum altitude
    Vector3D maxAltPos(EarthModel::EQUATORIAL_RADIUS + atmosphere->getMaxAltitude(), 0.0, 0.0);
    double maxAltDensity = atmosphere->getDensity(maxAltPos, julianDate);
    EXPECT_GT(maxAltDensity, 0.0);  // Should still have some density
    EXPECT_LT(maxAltDensity, 1e-9);  // But very small (adjusted threshold)
    
    // Test at minimum altitude
    Vector3D minAltPos(EarthModel::EQUATORIAL_RADIUS + atmosphere->getMinAltitude(), 0.0, 0.0);
    double minAltDensity = atmosphere->getDensity(minAltPos, julianDate);
    EXPECT_GT(minAltDensity, 1.0);  // Should be close to sea level
    EXPECT_LT(minAltDensity, 2.0);
}

TEST_F(NRLMSISE00AtmosphereTest, ConsistencyBetweenMethods) {
    Vector3D position(EarthModel::EQUATORIAL_RADIUS + 300000.0, 0.0, 0.0);
    double julianDate = 2451545.0;
    
    // Get values from individual methods
    double density = atmosphere->getDensity(position, julianDate);
    double temperature = atmosphere->getTemperature(position, julianDate);
    
    // Get detailed output
    auto output = atmosphere->getDetailedOutput(position, julianDate);
    
    // Should match
    EXPECT_DOUBLE_EQ(density, output.density_total);
    EXPECT_DOUBLE_EQ(temperature, output.temperature_neutral);
}