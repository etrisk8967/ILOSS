#include <gtest/gtest.h>
#include "physics/forces/drag/DragForceModel.h"
#include "physics/forces/drag/NRLMSISE00Atmosphere.h"
#include "physics/state/StateVector.h"
#include "core/time/Time.h"
#include "core/constants/EarthModel.h"
#include <cmath>

using namespace iloss;
using namespace iloss::physics;
using namespace iloss::physics::forces::drag;
using namespace iloss::math;
using namespace iloss::time;
using namespace iloss::constants;

class DragForceModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a default drag model
        dragModel = std::make_unique<DragForceModel>();
    }

    std::unique_ptr<DragForceModel> dragModel;
};

TEST_F(DragForceModelTest, DefaultConstruction) {
    EXPECT_EQ(dragModel->getName(), "AtmosphericDrag");
    EXPECT_EQ(dragModel->getType(), forces::ForceModelType::Drag);
    EXPECT_TRUE(dragModel->isEnabled());
    EXPECT_DOUBLE_EQ(dragModel->getDragCoefficient(), 2.2);
    EXPECT_DOUBLE_EQ(dragModel->getCrossSectionalArea(), 10.0);
    EXPECT_TRUE(dragModel->isAtmosphericRotationEnabled());
    EXPECT_FALSE(dragModel->isWindEnabled());
}

TEST_F(DragForceModelTest, SettersAndGetters) {
    // Test drag coefficient
    dragModel->setDragCoefficient(2.5);
    EXPECT_DOUBLE_EQ(dragModel->getDragCoefficient(), 2.5);
    
    // Test cross-sectional area
    dragModel->setCrossSectionalArea(20.0);
    EXPECT_DOUBLE_EQ(dragModel->getCrossSectionalArea(), 20.0);
    
    // Test ballistic coefficient
    dragModel->setBallisticCoefficient(50.0);
    EXPECT_DOUBLE_EQ(dragModel->getBallisticCoefficient(), 50.0);
    
    // Test atmospheric rotation
    dragModel->setAtmosphericRotation(false);
    EXPECT_FALSE(dragModel->isAtmosphericRotationEnabled());
    
    // Test wind
    dragModel->setWind(true);
    EXPECT_TRUE(dragModel->isWindEnabled());
    
    Vector3D windVel(10.0, -5.0, 2.0);
    dragModel->setWindVelocity(windVel);
    EXPECT_EQ(dragModel->getWindVelocity(), windVel);
}

TEST_F(DragForceModelTest, InvalidSetters) {
    // Test negative drag coefficient
    EXPECT_THROW(dragModel->setDragCoefficient(-1.0), std::invalid_argument);
    
    // Test negative area
    EXPECT_THROW(dragModel->setCrossSectionalArea(-5.0), std::invalid_argument);
    
    // Test negative ballistic coefficient
    EXPECT_THROW(dragModel->setBallisticCoefficient(-10.0), std::invalid_argument);
}

TEST_F(DragForceModelTest, ConfigurationInitialization) {
    forces::ForceModelConfig config;
    config.setParameter("drag_coefficient", 2.0);
    config.setParameter("area", 15.0);
    config.setParameter("enable_atmospheric_rotation", true);
    config.setParameter("enable_wind", true);
    config.setParameter("wind_velocity", Vector3D(5.0, 0.0, 0.0));
    
    EXPECT_TRUE(dragModel->initialize(config));
    
    EXPECT_DOUBLE_EQ(dragModel->getDragCoefficient(), 2.0);
    EXPECT_DOUBLE_EQ(dragModel->getCrossSectionalArea(), 15.0);
    EXPECT_TRUE(dragModel->isAtmosphericRotationEnabled());
    EXPECT_TRUE(dragModel->isWindEnabled());
    EXPECT_EQ(dragModel->getWindVelocity(), Vector3D(5.0, 0.0, 0.0));
}

TEST_F(DragForceModelTest, BallisticCoefficientMode) {
    forces::ForceModelConfig config;
    config.setParameter("ballistic_coefficient", 100.0);
    
    EXPECT_TRUE(dragModel->initialize(config));
    EXPECT_DOUBLE_EQ(dragModel->getBallisticCoefficient(), 100.0);
}

TEST_F(DragForceModelTest, Validation) {
    EXPECT_TRUE(dragModel->validate());
    
    // Test with invalid drag coefficient
    dragModel->setDragCoefficient(0.0);  // Zero is valid
    EXPECT_TRUE(dragModel->validate());
    
    // Can't set negative values due to exceptions, so validation should always pass
    // after successful construction/configuration
}

TEST_F(DragForceModelTest, ZeroDragAtHighAltitude) {
    // Create a state at very high altitude (above atmosphere)
    Vector3D highPosition(0.0, 0.0, EarthModel::EQUATORIAL_RADIUS + 1500000.0); // 1500 km altitude
    Vector3D velocity(7000.0, 0.0, 0.0);  // Typical orbital velocity
    Time currentTime(2025, 6, 27, 12, 0, 0.0);
    StateVector state(highPosition, velocity, 1000.0, currentTime);
    
    Vector3D acceleration = dragModel->calculateAcceleration(state, currentTime);
    
    // Should be zero acceleration at this altitude
    EXPECT_NEAR(acceleration.magnitude(), 0.0, 1e-10);
}

TEST_F(DragForceModelTest, DragAtLowAltitude) {
    // Create a state at low altitude (200 km) - place it at the equator on X axis for clarity
    double altitude = 200000.0;  // 200 km
    Vector3D position(EarthModel::EQUATORIAL_RADIUS + altitude, 0.0, 0.0);
    Vector3D velocity(0.0, 7800.0, 0.0);  // Velocity in Y direction (eastward)
    Time currentTime(2025, 6, 27, 12, 0, 0.0);
    StateVector state(position, velocity, 1000.0, currentTime,
                     coordinates::CoordinateSystemType::ECEF_WGS84);
    
    // Let's also check the altitude calculation
    double calculatedAltitude = EarthModel::getAltitude(position);
    EXPECT_NEAR(calculatedAltitude, altitude, 1.0);  // Should be close to 200km
    
    Vector3D acceleration = dragModel->calculateAcceleration(state, currentTime);
    
    // Should have non-zero drag acceleration
    EXPECT_GT(acceleration.magnitude(), 0.0);
    
    // Drag should oppose velocity (in Y direction)
    EXPECT_LT(acceleration.y(), 0.0);
}

TEST_F(DragForceModelTest, DragScalesWithDensity) {
    // Test at two different altitudes
    Time currentTime(2025, 6, 27, 12, 0, 0.0);
    
    // Lower altitude (higher density)
    Vector3D pos1(EarthModel::EQUATORIAL_RADIUS + 200000.0, 0.0, 0.0);
    Vector3D vel(0.0, 7800.0, 0.0);  // Eastward velocity
    StateVector state1(pos1, vel, 1000.0, currentTime,
                      coordinates::CoordinateSystemType::ECEF_WGS84);
    Vector3D accel1 = dragModel->calculateAcceleration(state1, currentTime);
    
    // Higher altitude (lower density)
    Vector3D pos2(EarthModel::EQUATORIAL_RADIUS + 400000.0, 0.0, 0.0);
    StateVector state2(pos2, vel, 1000.0, currentTime,
                      coordinates::CoordinateSystemType::ECEF_WGS84);
    Vector3D accel2 = dragModel->calculateAcceleration(state2, currentTime);
    
    // Drag at lower altitude should be greater
    EXPECT_GT(accel1.magnitude(), accel2.magnitude());
}

TEST_F(DragForceModelTest, DragScalesWithVelocitySquared) {
    // Test with two different velocities at same position
    Vector3D position(EarthModel::EQUATORIAL_RADIUS + 300000.0, 0.0, 0.0);
    Time currentTime(2025, 6, 27, 12, 0, 0.0);
    
    // Disable atmospheric rotation for this test to get exact velocity squared scaling
    dragModel->setAtmosphericRotation(false);
    
    // First velocity
    Vector3D vel1(0.0, 4000.0, 0.0);  // Eastward
    StateVector state1(position, vel1, 1000.0, currentTime,
                      coordinates::CoordinateSystemType::ECEF_WGS84);
    Vector3D accel1 = dragModel->calculateAcceleration(state1, currentTime);
    
    // Double the velocity
    Vector3D vel2(0.0, 8000.0, 0.0);  // Eastward, double speed
    StateVector state2(position, vel2, 1000.0, currentTime,
                      coordinates::CoordinateSystemType::ECEF_WGS84);
    Vector3D accel2 = dragModel->calculateAcceleration(state2, currentTime);
    
    // Drag should scale with velocity squared (4x)
    double ratio = accel2.magnitude() / accel1.magnitude();
    EXPECT_NEAR(ratio, 4.0, 0.001);  // Much tighter tolerance without atmospheric rotation
}

TEST_F(DragForceModelTest, DragScalesWithArea) {
    Vector3D position(EarthModel::EQUATORIAL_RADIUS + 300000.0, 0.0, 0.0);
    Vector3D velocity(0.0, 7500.0, 0.0);  // Eastward
    Time currentTime(2025, 6, 27, 12, 0, 0.0);
    StateVector state(position, velocity, 1000.0, currentTime,
                     coordinates::CoordinateSystemType::ECEF_WGS84);
    
    // First calculation with default area
    Vector3D accel1 = dragModel->calculateAcceleration(state, currentTime);
    
    // Double the area
    dragModel->setCrossSectionalArea(20.0);
    Vector3D accel2 = dragModel->calculateAcceleration(state, currentTime);
    
    // Drag should double
    double ratio = accel2.magnitude() / accel1.magnitude();
    EXPECT_NEAR(ratio, 2.0, 1e-6);
}

TEST_F(DragForceModelTest, DragScalesInverselyWithMass) {
    Vector3D position(EarthModel::EQUATORIAL_RADIUS + 300000.0, 0.0, 0.0);
    Vector3D velocity(0.0, 7500.0, 0.0);  // Eastward
    Time currentTime(2025, 6, 27, 12, 0, 0.0);
    
    // First mass
    StateVector state1(position, velocity, 1000.0, currentTime,
                      coordinates::CoordinateSystemType::ECEF_WGS84);
    Vector3D accel1 = dragModel->calculateAcceleration(state1, currentTime);
    
    // Double the mass
    StateVector state2(position, velocity, 2000.0, currentTime,
                      coordinates::CoordinateSystemType::ECEF_WGS84);
    Vector3D accel2 = dragModel->calculateAcceleration(state2, currentTime);
    
    // Acceleration should be halved
    double ratio = accel2.magnitude() / accel1.magnitude();
    EXPECT_NEAR(ratio, 0.5, 1e-6);
}

TEST_F(DragForceModelTest, AtmosphericRotationEffect) {
    // Position at equator
    Vector3D position(EarthModel::EQUATORIAL_RADIUS + 300000.0, 0.0, 0.0);
    Vector3D velocity(0.0, 7500.0, 0.0);  // Velocity in Y direction (eastward)
    Time currentTime(2025, 6, 27, 12, 0, 0.0);
    StateVector state(position, velocity, 1000.0, currentTime,
                     coordinates::CoordinateSystemType::ECEF_WGS84);
    
    // With atmospheric rotation
    dragModel->setAtmosphericRotation(true);
    Vector3D accel1 = dragModel->calculateAcceleration(state, currentTime);
    
    // Without atmospheric rotation
    dragModel->setAtmosphericRotation(false);
    Vector3D accel2 = dragModel->calculateAcceleration(state, currentTime);
    
    // The accelerations should be different
    EXPECT_GT((accel1 - accel2).magnitude(), 1e-11);
    
    // With rotation, eastward velocity experiences less drag
    EXPECT_LT(accel1.magnitude(), accel2.magnitude());
}

TEST_F(DragForceModelTest, WindEffect) {
    Vector3D position(EarthModel::EQUATORIAL_RADIUS + 300000.0, 0.0, 0.0);
    Vector3D velocity(7500.0, 0.0, 0.0);
    Time currentTime(2025, 6, 27, 12, 0, 0.0);
    StateVector state(position, velocity, 1000.0, currentTime,
                     coordinates::CoordinateSystemType::ECEF_WGS84);
    
    // Without wind
    dragModel->setWind(false);
    Vector3D accel1 = dragModel->calculateAcceleration(state, currentTime);
    
    // With tailwind (reduces drag)
    dragModel->setWind(true);
    dragModel->setWindVelocity(Vector3D(1000.0, 0.0, 0.0));  // 1 km/s tailwind
    Vector3D accel2 = dragModel->calculateAcceleration(state, currentTime);
    
    // Drag should be reduced with tailwind
    EXPECT_LT(accel2.magnitude(), accel1.magnitude());
}

TEST_F(DragForceModelTest, DisabledModel) {
    Vector3D position(0.0, 0.0, EarthModel::EQUATORIAL_RADIUS + 300000.0);
    Vector3D velocity(7500.0, 0.0, 0.0);
    Time currentTime(2025, 6, 27, 12, 0, 0.0);
    StateVector state(position, velocity, 1000.0, currentTime);
    
    // Disable the model
    dragModel->setEnabled(false);
    
    Vector3D acceleration = dragModel->calculateAcceleration(state, currentTime);
    
    // Should return zero acceleration when disabled
    EXPECT_EQ(acceleration.magnitude(), 0.0);
}

TEST_F(DragForceModelTest, Clone) {
    // Configure the model
    dragModel->setDragCoefficient(2.5);
    dragModel->setCrossSectionalArea(15.0);
    dragModel->setAtmosphericRotation(false);
    dragModel->setWind(true);
    dragModel->setWindVelocity(Vector3D(10.0, 5.0, 0.0));
    dragModel->setEnabled(false);
    
    // Clone it
    auto cloned = dragModel->clone();
    
    // Check that all properties were copied
    auto clonedDrag = dynamic_cast<DragForceModel*>(cloned.get());
    ASSERT_NE(clonedDrag, nullptr);
    
    EXPECT_EQ(clonedDrag->getDragCoefficient(), dragModel->getDragCoefficient());
    EXPECT_EQ(clonedDrag->getCrossSectionalArea(), dragModel->getCrossSectionalArea());
    EXPECT_EQ(clonedDrag->isAtmosphericRotationEnabled(), dragModel->isAtmosphericRotationEnabled());
    EXPECT_EQ(clonedDrag->isWindEnabled(), dragModel->isWindEnabled());
    EXPECT_EQ(clonedDrag->getWindVelocity(), dragModel->getWindVelocity());
    EXPECT_EQ(clonedDrag->isEnabled(), dragModel->isEnabled());
}

TEST_F(DragForceModelTest, ToString) {
    std::string str = dragModel->toString();
    EXPECT_NE(str.find("AtmosphericDrag"), std::string::npos);
    EXPECT_NE(str.find("Drag"), std::string::npos);
    EXPECT_NE(str.find("Cd="), std::string::npos);
    EXPECT_NE(str.find("A="), std::string::npos);
    
    // Test with ballistic coefficient
    dragModel->setBallisticCoefficient(50.0);
    str = dragModel->toString();
    EXPECT_NE(str.find("BC="), std::string::npos);
}

TEST_F(DragForceModelTest, ECEFCoordinateHandling) {
    // Test that the model correctly handles ECEF coordinates
    Vector3D ecefPosition(EarthModel::EQUATORIAL_RADIUS + 300000.0, 0.0, 0.0);
    Vector3D ecefVelocity(0.0, 7500.0, 0.0);
    Time currentTime(2025, 6, 27, 12, 0, 0.0);
    
    // Create state in ECEF coordinates
    StateVector ecefState(ecefPosition, ecefVelocity, 1000.0, currentTime,
                         coordinates::CoordinateSystemType::ECEF_WGS84);
    
    Vector3D acceleration = dragModel->calculateAcceleration(ecefState, currentTime);
    
    // Should get non-zero drag
    EXPECT_GT(acceleration.magnitude(), 0.0);
}

TEST_F(DragForceModelTest, ECEFSimpleDrag) {
    // Simple test with ECEF coordinates directly
    // Position at 200km altitude above equator
    double altitude = 200000.0; // 200 km
    Vector3D position(EarthModel::EQUATORIAL_RADIUS + altitude, 0.0, 0.0);
    Vector3D velocity(0.0, 7800.0, 0.0); // Eastward velocity
    Time currentTime(2025, 6, 27, 12, 0, 0.0);
    
    // Explicitly create in ECEF
    StateVector state(position, velocity, 1000.0, currentTime,
                     coordinates::CoordinateSystemType::ECEF_WGS84);
    
    Vector3D acceleration = dragModel->calculateAcceleration(state, currentTime);
    
    // Should have drag
    EXPECT_GT(acceleration.magnitude(), 0.0);
    
    // Drag should be westward (opposite to eastward velocity)
    EXPECT_LT(acceleration.y(), 0.0);
}

// Test with NRLMSISE-00 atmosphere
TEST(DragForceModelNRLMSISE00Test, BasicOperation) {
    auto nrlmsise = std::make_shared<NRLMSISE00Atmosphere>();
    DragForceModel dragModel(nrlmsise);
    
    // Place satellite at equator instead of pole for clearer test
    Vector3D position(EarthModel::EQUATORIAL_RADIUS + 400000.0, 0.0, 0.0);
    Vector3D velocity(0.0, 7500.0, 0.0);  // Eastward velocity
    Time currentTime(2025, 6, 27, 12, 0, 0.0);
    StateVector state(position, velocity, 1000.0, currentTime,
                     coordinates::CoordinateSystemType::ECEF_WGS84);
    
    // Check atmospheric density first
    double density = nrlmsise->getDensity(position, currentTime.getJulianDate());
    EXPECT_GT(density, 0.0) << "Atmospheric density should be non-zero at 400 km";
    
    Vector3D acceleration = dragModel.calculateAcceleration(state, currentTime);
    
    // Should get non-zero drag
    EXPECT_GT(acceleration.magnitude(), 0.0);
}

TEST(DragForceModelNRLMSISE00Test, SpaceWeatherEffect) {
    auto nrlmsise = std::make_shared<NRLMSISE00Atmosphere>();
    DragForceModel dragModel(nrlmsise);
    
    // Place satellite at equator
    Vector3D position(EarthModel::EQUATORIAL_RADIUS + 400000.0, 0.0, 0.0);
    Vector3D velocity(0.0, 7500.0, 0.0);  // Eastward velocity
    Time currentTime(2025, 6, 27, 12, 0, 0.0);
    StateVector state(position, velocity, 1000.0, currentTime,
                     coordinates::CoordinateSystemType::ECEF_WGS84);
    
    // Low solar activity
    SpaceWeatherData lowActivity;
    lowActivity.f107 = 70.0;
    lowActivity.f107a = 70.0;
    nrlmsise->setSpaceWeather(lowActivity);
    Vector3D accel1 = dragModel.calculateAcceleration(state, currentTime);
    
    // High solar activity
    SpaceWeatherData highActivity;
    highActivity.f107 = 250.0;
    highActivity.f107a = 250.0;
    nrlmsise->setSpaceWeather(highActivity);
    Vector3D accel2 = dragModel.calculateAcceleration(state, currentTime);
    
    // Higher solar activity should increase drag at high altitudes
    EXPECT_GT(accel2.magnitude(), accel1.magnitude());
}