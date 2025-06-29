#include <gtest/gtest.h>
#include "physics/forces/twobody/TwoBodyAnalyticalPropagator.h"
#include "core/math/MathConstants.h"
#include <cmath>

namespace iloss {
namespace physics {
namespace forces {
namespace twobody {
namespace test {

class TwoBodyAnalyticalPropagatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        m_propagator = std::make_unique<TwoBodyAnalyticalPropagator>();
        m_testTime = time::Time(2025, 6, 27, 12, 0, 0.0);
    }
    
    // Helper to create circular orbit
    StateVector createCircularOrbit(double radius, double inclination = 0.0) {
        math::Vector3D position(radius, 0.0, 0.0);
        double v = std::sqrt(math::constants::EARTH_MU / radius);
        math::Vector3D velocity(0.0, v * std::cos(inclination), v * std::sin(inclination));
        return StateVector(position, velocity, 1000.0, m_testTime);
    }
    
    // Helper to create elliptical orbit
    StateVector createEllipticalOrbit(double periapsis, double apoapsis) {
        math::Vector3D position(periapsis, 0.0, 0.0);
        double a = (periapsis + apoapsis) / 2.0;
        double v = std::sqrt(math::constants::EARTH_MU * (2.0 / periapsis - 1.0 / a));
        math::Vector3D velocity(0.0, v, 0.0);
        return StateVector(position, velocity, 1000.0, m_testTime);
    }
    
    std::unique_ptr<TwoBodyAnalyticalPropagator> m_propagator;
    time::Time m_testTime;
};

TEST_F(TwoBodyAnalyticalPropagatorTest, Construction) {
    EXPECT_DOUBLE_EQ(m_propagator->getGravitationalParameter(), math::constants::EARTH_MU);
    EXPECT_EQ(m_propagator->getCentralBody(), "Earth");
    
    // Custom construction
    TwoBodyAnalyticalPropagator moonProp(math::constants::MOON_MU, "Moon");
    EXPECT_DOUBLE_EQ(moonProp.getGravitationalParameter(), math::constants::MOON_MU);
    EXPECT_EQ(moonProp.getCentralBody(), "Moon");
    
    // Invalid construction
    EXPECT_THROW(TwoBodyAnalyticalPropagator(-1.0, "Invalid"), std::invalid_argument);
}

TEST_F(TwoBodyAnalyticalPropagatorTest, PropagateToTime) {
    StateVector initial = createCircularOrbit(7000000.0);
    
    // Propagate forward by 1 hour
    time::Time targetTime = m_testTime + 3600.0;
    StateVector final = m_propagator->propagateToTime(initial, targetTime);
    
    EXPECT_EQ(final.getTime(), targetTime);
    
    // Check orbital radius is preserved for circular orbit
    EXPECT_NEAR(final.getPosition().magnitude(), initial.getPosition().magnitude(), 1.0);
    
    // Check energy conservation
    double initialEnergy = initial.getSpecificEnergy(math::constants::EARTH_MU);
    double finalEnergy = final.getSpecificEnergy(math::constants::EARTH_MU);
    EXPECT_NEAR(initialEnergy, finalEnergy, 1e-8);
}

TEST_F(TwoBodyAnalyticalPropagatorTest, PropagateByDuration) {
    StateVector initial = createCircularOrbit(8000000.0);
    
    double dt = 1800.0; // 30 minutes
    StateVector final = m_propagator->propagateByDuration(initial, dt);
    
    EXPECT_DOUBLE_EQ(final.getTime().getTime() - initial.getTime().getTime(), dt);
    
    // Test negative duration (backward propagation)
    StateVector backward = m_propagator->propagateByDuration(final, -dt);
    
    // Should return to initial position
    EXPECT_NEAR(backward.getPosition().x(), initial.getPosition().x(), 1e-6);
    EXPECT_NEAR(backward.getPosition().y(), initial.getPosition().y(), 1e-6);
    EXPECT_NEAR(backward.getPosition().z(), initial.getPosition().z(), 1e-6);
}

TEST_F(TwoBodyAnalyticalPropagatorTest, PropagateToTrueAnomaly) {
    // Start at periapsis of elliptical orbit
    StateVector initial = createEllipticalOrbit(6678000.0, 42164000.0);
    
    // Propagate to nu = 90 degrees
    double targetNu = math::constants::PI / 2.0;
    StateVector final = m_propagator->propagateToTrueAnomaly(initial, targetNu);
    
    // Verify true anomaly
    OrbitalElements elements = m_propagator->determineOrbit(
        final.getPosition(), final.getVelocity(), final.getTime());
    EXPECT_NEAR(elements.nu, targetNu, 1e-6);
    
    // Test backward propagation
    double backwardNu = 3.0 * math::constants::PI / 2.0; // 270 degrees
    StateVector backward = m_propagator->propagateToTrueAnomaly(initial, backwardNu, -1);
    
    // Should have propagated backward
    EXPECT_LT(backward.getTime().getTime(), initial.getTime().getTime());
}

TEST_F(TwoBodyAnalyticalPropagatorTest, GenerateTrajectory) {
    StateVector initial = createCircularOrbit(7500000.0);
    
    double duration = 3600.0; // 1 hour
    double timeStep = 60.0; // 1 minute
    time::Time endTime = m_testTime + duration;
    
    auto trajectory = m_propagator->generateTrajectory(initial, m_testTime, endTime, timeStep);
    
    ASSERT_NE(trajectory, nullptr);
    EXPECT_GT(trajectory->size(), 0);
    
    // Check first and last states
    EXPECT_EQ(trajectory->getEarliestState().getTime(), m_testTime);
    EXPECT_EQ(trajectory->getLatestState().getTime(), endTime);
    
    // Check number of points
    size_t expectedPoints = static_cast<size_t>(duration / timeStep) + 1;
    EXPECT_EQ(trajectory->size(), expectedPoints);
    
    // Check energy conservation along trajectory
    double initialEnergy = initial.getSpecificEnergy(math::constants::EARTH_MU);
    auto allStates = trajectory->getAllStates();
    for (const auto& state : allStates) {
        double energy = state.getSpecificEnergy(math::constants::EARTH_MU);
        EXPECT_NEAR(energy, initialEnergy, 5e-8);
    }
}

TEST_F(TwoBodyAnalyticalPropagatorTest, DetermineOrbit) {
    // Known circular orbit
    double radius = 7000000.0;
    math::Vector3D position(radius, 0.0, 0.0);
    double v = std::sqrt(math::constants::EARTH_MU / radius);
    math::Vector3D velocity(0.0, v, 0.0);
    
    OrbitalElements elements = m_propagator->determineOrbit(position, velocity, m_testTime);
    
    EXPECT_NEAR(elements.a, radius, 1e-6);
    EXPECT_NEAR(elements.e, 0.0, 1e-10);
    EXPECT_NEAR(elements.i, 0.0, 1e-10);
    
    // Known elliptical orbit
    double periapsis = 6678000.0;
    double apoapsis = 42164000.0;
    position = math::Vector3D(periapsis, 0.0, 0.0);
    double a = (periapsis + apoapsis) / 2.0;
    v = std::sqrt(math::constants::EARTH_MU * (2.0 / periapsis - 1.0 / a));
    velocity = math::Vector3D(0.0, v, 0.0);
    
    elements = m_propagator->determineOrbit(position, velocity, m_testTime);
    
    double expected_e = (apoapsis - periapsis) / (apoapsis + periapsis);
    EXPECT_NEAR(elements.a, a, 1e-6);
    EXPECT_NEAR(elements.e, expected_e, 1e-10);
}

TEST_F(TwoBodyAnalyticalPropagatorTest, HohmannTransfer) {
    double r1 = 6678000.0; // LEO
    double r2 = 42164000.0; // GEO
    
    auto [dv1, dv2] = m_propagator->calculateHohmannTransfer(r1, r2);
    
    // Known values for LEO to GEO transfer
    EXPECT_NEAR(dv1, 2420.0, 10.0); // ~2.42 km/s
    EXPECT_NEAR(dv2, 1470.0, 10.0); // ~1.47 km/s
    
    // Total delta-v
    double total_dv = dv1 + dv2;
    EXPECT_NEAR(total_dv, 3890.0, 20.0); // ~3.89 km/s
    
    // Test reverse transfer
    auto [dv1_rev, dv2_rev] = m_propagator->calculateHohmannTransfer(r2, r1);
    
    // Should be same magnitudes
    EXPECT_NEAR(dv1_rev, dv2, 1e-6);
    EXPECT_NEAR(dv2_rev, dv1, 1e-6);
}

TEST_F(TwoBodyAnalyticalPropagatorTest, BiellipticTransfer) {
    double r1 = 6678000.0; // LEO
    double r2 = 42164000.0; // GEO
    double rt = 100000000.0; // High transfer orbit
    
    auto dvs = m_propagator->calculateBiellipticTransfer(r1, r2, rt);
    
    ASSERT_EQ(dvs.size(), 3);
    
    // All delta-v values should be positive
    for (double dv : dvs) {
        EXPECT_GT(dv, 0.0);
    }
    
    // For very high transfer orbits, bi-elliptic can be more efficient
    // than Hohmann for large radius ratios
    double total_bielliptic = dvs[0] + dvs[1] + dvs[2];
    EXPECT_GT(total_bielliptic, 0.0);  // Total delta-v should be positive
    
    // Test invalid input
    EXPECT_THROW(m_propagator->calculateBiellipticTransfer(r1, r2, r1 / 2.0),
                 std::invalid_argument);
}

TEST_F(TwoBodyAnalyticalPropagatorTest, PlaneChange) {
    double v = 7500.0; // m/s
    double deltaI = 10.0 * math::constants::DEG_TO_RAD; // 10 degrees
    
    double dv = m_propagator->calculatePlaneChange(v, deltaI);
    
    // Expected: dv = 2 * v * sin(deltaI/2)
    double expected = 2.0 * v * std::sin(deltaI / 2.0);
    EXPECT_NEAR(dv, expected, 1e-10);
    
    // Test 90 degree plane change
    deltaI = 90.0 * math::constants::DEG_TO_RAD;
    dv = m_propagator->calculatePlaneChange(v, deltaI);
    
    // For 90 degrees: dv = v * sqrt(2)
    EXPECT_NEAR(dv, v * std::sqrt(2.0), 1e-10);
}

TEST_F(TwoBodyAnalyticalPropagatorTest, CombinedManeuver) {
    double r1 = 6678000.0; // LEO
    double r2 = 42164000.0; // GEO
    double deltaI = 28.5 * math::constants::DEG_TO_RAD; // Inclination change
    
    double dv_combined = m_propagator->calculateCombinedManeuver(r1, r2, deltaI);
    
    // Should be less than separate maneuvers
    auto [dv1_hohmann, dv2_hohmann] = m_propagator->calculateHohmannTransfer(r1, r2);
    double v_geo = std::sqrt(math::constants::EARTH_MU / r2);
    double dv_plane = m_propagator->calculatePlaneChange(v_geo, deltaI);
    
    double dv_separate = dv1_hohmann + dv2_hohmann + dv_plane;
    
    EXPECT_LT(dv_combined, dv_separate);
}

TEST_F(TwoBodyAnalyticalPropagatorTest, TimeToTrueAnomaly) {
    // Circular orbit
    StateVector initial = createCircularOrbit(7000000.0);
    
    // Time to reach 90 degrees
    double targetNu = math::constants::PI / 2.0;
    double dt = m_propagator->timeToTrueAnomaly(initial, targetNu);
    
    // For circular orbit, should be 1/4 period
    double period = 2.0 * math::constants::PI * 
                   std::sqrt(7000000.0 * 7000000.0 * 7000000.0 / math::constants::EARTH_MU);
    EXPECT_NEAR(dt, period / 4.0, 1.0);
    
    // Test backward direction
    double dt_back = m_propagator->timeToTrueAnomaly(initial, targetNu, -1);
    EXPECT_NEAR(dt_back, -3.0 * period / 4.0, 1.0);
}

TEST_F(TwoBodyAnalyticalPropagatorTest, NextPeriapsisTime) {
    // Start at some point in elliptical orbit
    double periapsis = 6678000.0;
    double apoapsis = 42164000.0;
    StateVector initial = createEllipticalOrbit(periapsis, apoapsis);
    
    // Propagate to nu = 90 degrees
    initial = m_propagator->propagateToTrueAnomaly(initial, math::constants::PI / 2.0);
    
    time::Time nextPeri = m_propagator->nextPeriapsisTime(initial);
    
    // Propagate to that time and verify
    StateVector atPeri = m_propagator->propagateToTime(initial, nextPeri);
    
    // Should be at periapsis
    EXPECT_NEAR(atPeri.getPosition().magnitude(), periapsis, 100.0); // 100m tolerance
}

TEST_F(TwoBodyAnalyticalPropagatorTest, NextApoapsisTime) {
    // Start at periapsis of elliptical orbit
    double periapsis = 6678000.0;
    double apoapsis = 42164000.0;
    StateVector initial = createEllipticalOrbit(periapsis, apoapsis);
    
    time::Time nextApo = m_propagator->nextApoapsisTime(initial);
    
    // Propagate to that time and verify
    StateVector atApo = m_propagator->propagateToTime(initial, nextApo);
    
    // Should be at apoapsis
    EXPECT_NEAR(atApo.getPosition().magnitude(), apoapsis, 100.0); // 100m tolerance
    
    // Test hyperbolic orbit (should throw)
    double v_escape = std::sqrt(2.0 * math::constants::EARTH_MU / periapsis);
    math::Vector3D position(periapsis, 0.0, 0.0);
    math::Vector3D velocity(0.0, v_escape * 1.1, 0.0);
    StateVector hyperbolic(position, velocity, 1000.0, m_testTime);
    
    EXPECT_THROW(m_propagator->nextApoapsisTime(hyperbolic), std::runtime_error);
}

TEST_F(TwoBodyAnalyticalPropagatorTest, GroundTrack) {
    // Create inclined orbit
    double radius = 7000000.0;
    double inclination = 51.6 * math::constants::DEG_TO_RAD; // ISS inclination
    StateVector initial = createCircularOrbit(radius, inclination);
    
    double duration = 5400.0; // 1.5 hours (approximately one orbit)
    double timeStep = 60.0; // 1 minute
    
    auto groundTrack = m_propagator->calculateGroundTrack(initial, duration, timeStep);
    
    ASSERT_FALSE(groundTrack.empty());
    
    // Check latitude bounds
    for (const auto& [lat, lon] : groundTrack) {
        EXPECT_LE(std::abs(lat), inclination + 0.01); // Small tolerance
        EXPECT_GE(lon, -math::constants::PI);
        EXPECT_LE(lon, math::constants::PI);
    }
    
    // Check that longitude changes (Earth rotation)
    if (groundTrack.size() > 1) {
        double lon1 = groundTrack[0].second;
        double lon2 = groundTrack.back().second;
        EXPECT_NE(lon1, lon2);
    }
}

TEST_F(TwoBodyAnalyticalPropagatorTest, CanSeeGroundLocation) {
    OrbitalElements elements;
    elements.a = 7000000.0;
    elements.e = 0.0;
    elements.i = 60.0 * math::constants::DEG_TO_RAD;
    
    // Location at 45 degrees latitude (should be visible)
    double lat = 45.0 * math::constants::DEG_TO_RAD;
    double lon = 0.0;
    
    EXPECT_TRUE(m_propagator->canSeeGroundLocation(elements, lat, lon));
    
    // Location at 70 degrees latitude (should not be visible from 60° inclined orbit)
    lat = 70.0 * math::constants::DEG_TO_RAD;
    EXPECT_FALSE(m_propagator->canSeeGroundLocation(elements, lat, lon));
    
    // Hyperbolic orbit
    elements.e = 1.5;
    EXPECT_FALSE(m_propagator->canSeeGroundLocation(elements, 0.0, 0.0));
}

TEST_F(TwoBodyAnalyticalPropagatorTest, LambertProblem) {
    // Simple Lambert problem: transfer between two points on circular orbit
    double radius = 8000000.0;
    double angle = 60.0 * math::constants::DEG_TO_RAD;
    
    math::Vector3D r1(radius, 0.0, 0.0);
    math::Vector3D r2(radius * std::cos(angle), radius * std::sin(angle), 0.0);
    
    // Time for circular orbit transfer
    double n = std::sqrt(math::constants::EARTH_MU / (radius * radius * radius));
    double tof = angle / n;
    
    auto solutions = m_propagator->solveLambertProblem(r1, r2, tof);
    
    ASSERT_FALSE(solutions.empty());
    
    // Verify solution
    if (!solutions.empty() && solutions[0].isValid()) {
        // Check that velocities are reasonable
        EXPECT_GT(solutions[0].v1.magnitude(), 0.0);
        EXPECT_GT(solutions[0].v2.magnitude(), 0.0);
        EXPECT_LT(solutions[0].v1.magnitude(), 20000.0); // Less than 20 km/s
        EXPECT_LT(solutions[0].v2.magnitude(), 20000.0);
    }
}

TEST_F(TwoBodyAnalyticalPropagatorTest, EnergyConservation) {
    // Test various orbits for energy conservation
    std::vector<StateVector> testOrbits = {
        createCircularOrbit(7000000.0),
        createCircularOrbit(42164000.0),
        createEllipticalOrbit(6678000.0, 42164000.0),
        createEllipticalOrbit(7000000.0, 50000000.0)
    };
    
    for (const auto& initial : testOrbits) {
        double initialEnergy = initial.getSpecificEnergy(math::constants::EARTH_MU);
        
        // Propagate for various durations
        std::vector<double> durations = {60.0, 600.0, 3600.0, 7200.0};
        
        for (double dt : durations) {
            StateVector final = m_propagator->propagateByDuration(initial, dt);
            double finalEnergy = final.getSpecificEnergy(math::constants::EARTH_MU);
            
            EXPECT_NEAR(finalEnergy, initialEnergy, 5e-8)
                << "Energy not conserved after " << dt << " seconds";
        }
    }
}

TEST_F(TwoBodyAnalyticalPropagatorTest, AngularMomentumConservation) {
    // Test angular momentum conservation
    StateVector initial = createEllipticalOrbit(7000000.0, 35000000.0);
    
    math::Vector3D h_initial = initial.getPosition().cross(initial.getVelocity());
    
    // Propagate through multiple points in orbit
    double period = 2.0 * math::constants::PI * 
                   std::sqrt(21000000.0 * 21000000.0 * 21000000.0 / math::constants::EARTH_MU);
    
    for (int i = 1; i <= 8; ++i) {
        double dt = period * i / 8.0;
        StateVector current = m_propagator->propagateByDuration(initial, dt);
        
        math::Vector3D h_current = current.getPosition().cross(current.getVelocity());
        
        // Magnitude should be conserved
        EXPECT_NEAR(h_current.magnitude(), h_initial.magnitude(), 1e-4);
        
        // Direction should be conserved
        math::Vector3D h_dir_initial = h_initial.normalized();
        math::Vector3D h_dir_current = h_current.normalized();
        
        EXPECT_NEAR(h_dir_current.x(), h_dir_initial.x(), 1e-10);
        EXPECT_NEAR(h_dir_current.y(), h_dir_initial.y(), 1e-10);
        EXPECT_NEAR(h_dir_current.z(), h_dir_initial.z(), 1e-10);
    }
}

} // namespace test
} // namespace twobody
} // namespace forces
} // namespace physics
} // namespace iloss