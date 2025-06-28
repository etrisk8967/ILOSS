#include <gtest/gtest.h>
#include "physics/forces/twobody/KeplerPropagator.h"
#include "core/time/Time.h"
#include "core/math/MathConstants.h"
#include <cmath>

namespace iloss {
namespace physics {
namespace forces {
namespace twobody {
namespace test {

class KeplerPropagatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        m_propagator = std::make_unique<KeplerPropagator>();
        m_testTime = time::Time(2025, 6, 27, 12, 0, 0.0);
    }
    
    // Helper function to create circular orbit state
    StateVector createCircularOrbitState(double radius, double inclination = 0.0) {
        math::Vector3D position(radius, 0.0, 0.0);
        double velocity_mag = std::sqrt(math::constants::EARTH_MU / radius);
        math::Vector3D velocity(0.0, velocity_mag * std::cos(inclination), 
                               velocity_mag * std::sin(inclination));
        return StateVector(position, velocity, 1000.0, m_testTime);
    }
    
    // Helper function to create elliptical orbit state at periapsis
    StateVector createEllipticalOrbitState(double periapsis, double apoapsis) {
        math::Vector3D position(periapsis, 0.0, 0.0);
        double a = (periapsis + apoapsis) / 2.0;
        double v = std::sqrt(math::constants::EARTH_MU * (2.0 / periapsis - 1.0 / a));
        math::Vector3D velocity(0.0, v, 0.0);
        return StateVector(position, velocity, 1000.0, m_testTime);
    }
    
    std::unique_ptr<KeplerPropagator> m_propagator;
    time::Time m_testTime;
};

// Test OrbitalElements struct
TEST_F(KeplerPropagatorTest, OrbitalElementsConstruction) {
    OrbitalElements elements;
    
    EXPECT_DOUBLE_EQ(elements.a, 0.0);
    EXPECT_DOUBLE_EQ(elements.e, 0.0);
    EXPECT_DOUBLE_EQ(elements.i, 0.0);
    EXPECT_DOUBLE_EQ(elements.omega, 0.0);
    EXPECT_DOUBLE_EQ(elements.Omega, 0.0);
    EXPECT_DOUBLE_EQ(elements.nu, 0.0);
}

TEST_F(KeplerPropagatorTest, OrbitalElementsValidation) {
    OrbitalElements elements;
    
    // Valid circular orbit
    elements.a = 7000000.0;
    elements.e = 0.0;
    elements.i = 0.5;
    EXPECT_TRUE(elements.isValid());
    
    // Invalid: negative semi-major axis for elliptical orbit
    elements.a = -7000000.0;
    elements.e = 0.5;
    EXPECT_FALSE(elements.isValid());
    
    // Valid: negative semi-major axis for hyperbolic orbit
    elements.e = 1.5;
    EXPECT_TRUE(elements.isValid());
    
    // Invalid: negative eccentricity
    elements.a = 7000000.0;
    elements.e = -0.1;
    EXPECT_FALSE(elements.isValid());
    
    // Invalid: inclination out of range
    elements.e = 0.1;
    elements.i = 4.0; // > π
    EXPECT_FALSE(elements.isValid());
}

TEST_F(KeplerPropagatorTest, OrbitTypeClassification) {
    OrbitalElements elements;
    elements.a = 7000000.0;
    
    // Circular
    elements.e = 1e-9;
    EXPECT_EQ(elements.getOrbitType(), "circular");
    
    // Elliptical
    elements.e = 0.5;
    EXPECT_EQ(elements.getOrbitType(), "elliptical");
    
    // Parabolic
    elements.e = 1.0;
    EXPECT_EQ(elements.getOrbitType(), "parabolic");
    
    // Hyperbolic
    elements.e = 1.5;
    EXPECT_EQ(elements.getOrbitType(), "hyperbolic");
}

TEST_F(KeplerPropagatorTest, ApsisCalculations) {
    OrbitalElements elements;
    elements.a = 10000000.0; // 10,000 km
    elements.e = 0.2;
    
    double periapsis = elements.getPeriapsis();
    double apoapsis = elements.getApoapsis();
    
    EXPECT_DOUBLE_EQ(periapsis, 8000000.0); // a(1-e)
    EXPECT_DOUBLE_EQ(apoapsis, 12000000.0); // a(1+e)
    
    // Parabolic orbit
    elements.e = 1.0;
    EXPECT_DOUBLE_EQ(elements.getApoapsis(), std::numeric_limits<double>::infinity());
    
    // Hyperbolic orbit
    elements.e = 1.5;
    EXPECT_DOUBLE_EQ(elements.getApoapsis(), std::numeric_limits<double>::infinity());
}

TEST_F(KeplerPropagatorTest, StateToElementsCircularOrbit) {
    // Create circular orbit at 400 km altitude
    double radius = math::constants::EARTH_RADIUS_EQUATORIAL + 400000.0;
    StateVector state = createCircularOrbitState(radius);
    
    OrbitalElements elements = KeplerPropagator::stateToElements(state, math::constants::EARTH_MU);
    
    // Check elements
    EXPECT_NEAR(elements.a, radius, 1e-6);
    EXPECT_NEAR(elements.e, 0.0, 1e-10);
    EXPECT_NEAR(elements.i, 0.0, 1e-10);
    EXPECT_NEAR(elements.nu, 0.0, 1e-10); // At x-axis
    
    // Check derived parameters
    double expected_n = std::sqrt(math::constants::EARTH_MU / (radius * radius * radius));
    EXPECT_NEAR(elements.n, expected_n, 1e-10);
    EXPECT_NEAR(elements.T, math::constants::TWO_PI / expected_n, 1e-6);
}

TEST_F(KeplerPropagatorTest, StateToElementsEllipticalOrbit) {
    // Create elliptical orbit (LEO to GEO transfer)
    double periapsis = 6678000.0; // ~300 km altitude
    double apoapsis = 42164000.0; // GEO radius
    StateVector state = createEllipticalOrbitState(periapsis, apoapsis);
    
    OrbitalElements elements = KeplerPropagator::stateToElements(state, math::constants::EARTH_MU);
    
    double expected_a = (periapsis + apoapsis) / 2.0;
    double expected_e = (apoapsis - periapsis) / (apoapsis + periapsis);
    
    EXPECT_NEAR(elements.a, expected_a, 1e-6);
    EXPECT_NEAR(elements.e, expected_e, 1e-10);
    EXPECT_NEAR(elements.i, 0.0, 1e-10); // Equatorial
    EXPECT_NEAR(elements.nu, 0.0, 1e-10); // At periapsis
    EXPECT_NEAR(elements.omega, 0.0, 1e-10); // Undefined for equatorial
}

TEST_F(KeplerPropagatorTest, StateToElementsInclinedOrbit) {
    // Create inclined circular orbit
    double radius = 7000000.0;
    double inclination = 45.0 * math::constants::DEG_TO_RAD;
    StateVector state = createCircularOrbitState(radius, inclination);
    
    OrbitalElements elements = KeplerPropagator::stateToElements(state, math::constants::EARTH_MU);
    
    EXPECT_NEAR(elements.a, radius, 1e-6);
    EXPECT_NEAR(elements.e, 0.0, 1e-10);
    EXPECT_NEAR(elements.i, inclination, 1e-10);
}

TEST_F(KeplerPropagatorTest, ElementsToStateCircularOrbit) {
    OrbitalElements elements;
    elements.a = 7000000.0;
    elements.e = 0.0;
    elements.i = 0.0;
    elements.omega = 0.0;
    elements.Omega = 0.0;
    elements.nu = math::constants::PI / 2.0; // 90 degrees
    
    // Calculate derived parameters
    elements.p = elements.a; // For circular orbit
    elements.n = std::sqrt(math::constants::EARTH_MU / (elements.a * elements.a * elements.a));
    
    StateVector state = KeplerPropagator::elementsToState(elements, math::constants::EARTH_MU, m_testTime);
    
    // At nu = 90°, position should be on y-axis
    EXPECT_NEAR(state.getPosition().x(), 0.0, 1e-6);
    EXPECT_NEAR(state.getPosition().y(), elements.a, 1e-6);
    EXPECT_NEAR(state.getPosition().z(), 0.0, 1e-6);
    
    // Velocity should be in -x direction
    double v_circular = std::sqrt(math::constants::EARTH_MU / elements.a);
    EXPECT_NEAR(state.getVelocity().x(), -v_circular, 1e-6);
    EXPECT_NEAR(state.getVelocity().y(), 0.0, 1e-6);
    EXPECT_NEAR(state.getVelocity().z(), 0.0, 1e-6);
}

TEST_F(KeplerPropagatorTest, KeplerEquationElliptical) {
    // Test Kepler's equation solver for various eccentricities
    std::vector<double> eccentricities = {0.0, 0.1, 0.3, 0.5, 0.7, 0.9, 0.99};
    std::vector<double> mean_anomalies = {0.0, math::constants::PI/4, math::constants::PI/2, 
                                         math::constants::PI, 3*math::constants::PI/2};
    
    for (double e : eccentricities) {
        for (double M : mean_anomalies) {
            double E = KeplerPropagator::solveKeplerEllipse(M, e);
            
            // Verify solution
            double M_check = E - e * std::sin(E);
            M_check = math::constants::normalizeAnglePositive(M_check);
            double M_normalized = math::constants::normalizeAnglePositive(M);
            
            EXPECT_NEAR(M_check, M_normalized, 1e-10) 
                << "Failed for e=" << e << ", M=" << M;
        }
    }
}

TEST_F(KeplerPropagatorTest, KeplerEquationHyperbolic) {
    // Test hyperbolic Kepler equation
    std::vector<double> eccentricities = {1.1, 1.5, 2.0, 3.0};
    std::vector<double> mean_anomalies = {-2.0, -1.0, 0.0, 1.0, 2.0};
    
    for (double e : eccentricities) {
        for (double M : mean_anomalies) {
            double H = KeplerPropagator::solveKeplerHyperbola(M, e);
            
            // Verify solution
            double M_check = e * std::sinh(H) - H;
            
            EXPECT_NEAR(M_check, M, 1e-10) 
                << "Failed for e=" << e << ", M=" << M;
        }
    }
}

TEST_F(KeplerPropagatorTest, AnomalyConversions) {
    // Test conversions between true, eccentric, and mean anomaly
    double e = 0.3;
    std::vector<double> true_anomalies = {0.0, math::constants::PI/4, math::constants::PI/2, 
                                         math::constants::PI, 3*math::constants::PI/2};
    
    for (double nu : true_anomalies) {
        // True to eccentric
        double E = KeplerPropagator::trueToEccentricAnomaly(nu, e);
        
        // Eccentric back to true
        double nu_check = KeplerPropagator::eccentricToTrueAnomaly(E, e);
        EXPECT_NEAR(nu_check, nu, 1e-10);
        
        // True to mean
        double M = KeplerPropagator::trueToMeanAnomaly(nu, e);
        
        // Mean back to true
        double nu_check2 = KeplerPropagator::meanToTrueAnomaly(M, e);
        EXPECT_NEAR(nu_check2, nu, 1e-10);
    }
}

TEST_F(KeplerPropagatorTest, CircularOrbitPropagation) {
    // Create circular orbit
    double radius = 7000000.0;
    StateVector initial = createCircularOrbitState(radius);
    
    // Calculate period
    double period = 2.0 * math::constants::PI * std::sqrt(radius * radius * radius / math::constants::EARTH_MU);
    
    // Propagate for 1/4 period
    time::Time targetTime = m_testTime + period / 4.0;
    StateVector propagated = m_propagator->propagate(initial, targetTime);
    
    // Should be at 90 degrees true anomaly
    EXPECT_NEAR(propagated.getPosition().x(), 0.0, 100.0); // 100m tolerance
    EXPECT_NEAR(propagated.getPosition().y(), radius, 100.0);
    EXPECT_NEAR(propagated.getPosition().z(), 0.0, 100.0);
    
    // Check conservation of energy
    double initial_energy = initial.getSpecificEnergy(math::constants::EARTH_MU);
    double final_energy = propagated.getSpecificEnergy(math::constants::EARTH_MU);
    EXPECT_NEAR(initial_energy, final_energy, 1e-9);
}

TEST_F(KeplerPropagatorTest, EllipticalOrbitPropagation) {
    // Create elliptical orbit
    double periapsis = 6678000.0;
    double apoapsis = 42164000.0;
    StateVector initial = createEllipticalOrbitState(periapsis, apoapsis);
    
    // Calculate period
    double a = (periapsis + apoapsis) / 2.0;
    double period = 2.0 * math::constants::PI * std::sqrt(a * a * a / math::constants::EARTH_MU);
    
    // Propagate for half period (should be at apoapsis)
    time::Time targetTime = m_testTime + period / 2.0;
    StateVector propagated = m_propagator->propagate(initial, targetTime);
    
    // Should be at apoapsis
    double r = propagated.getPosition().magnitude();
    EXPECT_NEAR(r, apoapsis, 1000.0); // 1 km tolerance
    
    // Velocity should be minimum at apoapsis
    double v_apoapsis = std::sqrt(math::constants::EARTH_MU * (2.0 / apoapsis - 1.0 / a));
    EXPECT_NEAR(propagated.getVelocity().magnitude(), v_apoapsis, 1.0); // 1 m/s tolerance
}

TEST_F(KeplerPropagatorTest, BackwardPropagation) {
    // Test propagating backward in time
    double radius = 8000000.0;
    StateVector initial = createCircularOrbitState(radius);
    
    // Propagate backward by 1 hour
    double dt = -3600.0;
    time::Time targetTime = m_testTime + dt;
    StateVector propagated = m_propagator->propagate(initial, targetTime);
    
    // Propagate forward again
    StateVector final = m_propagator->propagate(propagated, m_testTime);
    
    // Should return to initial state
    EXPECT_NEAR(final.getPosition().x(), initial.getPosition().x(), 1e-6);
    EXPECT_NEAR(final.getPosition().y(), initial.getPosition().y(), 1e-6);
    EXPECT_NEAR(final.getPosition().z(), initial.getPosition().z(), 1e-6);
    EXPECT_NEAR(final.getVelocity().x(), initial.getVelocity().x(), 1e-9);
    EXPECT_NEAR(final.getVelocity().y(), initial.getVelocity().y(), 1e-9);
    EXPECT_NEAR(final.getVelocity().z(), initial.getVelocity().z(), 1e-9);
}

TEST_F(KeplerPropagatorTest, FlightPathAngle) {
    // Test flight path angle calculation
    double e = 0.3;
    
    // At periapsis (nu = 0), flight path angle should be 0
    double gamma = KeplerPropagator::calculateFlightPathAngle(0.0, e);
    EXPECT_NEAR(gamma, 0.0, 1e-10);
    
    // At apoapsis (nu = π), flight path angle should be 0
    gamma = KeplerPropagator::calculateFlightPathAngle(math::constants::PI, e);
    EXPECT_NEAR(gamma, 0.0, 1e-10);
    
    // At nu = π/2, should be positive
    gamma = KeplerPropagator::calculateFlightPathAngle(math::constants::PI / 2.0, e);
    EXPECT_GT(gamma, 0.0);
}

TEST_F(KeplerPropagatorTest, HyperbolicOrbit) {
    // Create hyperbolic orbit
    double periapsis = 7000000.0;
    double v_infinity = 3000.0; // 3 km/s excess velocity
    
    // Calculate required velocity at periapsis
    double v_escape = std::sqrt(2.0 * math::constants::EARTH_MU / periapsis);
    double v_periapsis = std::sqrt(v_escape * v_escape + v_infinity * v_infinity);
    
    math::Vector3D position(periapsis, 0.0, 0.0);
    math::Vector3D velocity(0.0, v_periapsis, 0.0);
    StateVector initial(position, velocity, 1000.0, m_testTime);
    
    // Convert to elements
    OrbitalElements elements = KeplerPropagator::stateToElements(initial, math::constants::EARTH_MU);
    
    EXPECT_GT(elements.e, 1.0); // Should be hyperbolic
    EXPECT_LT(elements.a, 0.0); // Negative semi-major axis
    
    // Propagate forward
    time::Time targetTime = m_testTime + 3600.0; // 1 hour
    StateVector propagated = m_propagator->propagate(initial, targetTime);
    
    // Should be moving away
    EXPECT_GT(propagated.getPosition().magnitude(), initial.getPosition().magnitude());
}

TEST_F(KeplerPropagatorTest, MassPreservation) {
    // Test that mass is preserved during propagation
    double radius = 7500000.0;
    math::Vector3D position(radius, 0.0, 0.0);
    double v = std::sqrt(math::constants::EARTH_MU / radius);
    math::Vector3D velocity(0.0, v, 0.0);
    
    double mass = 1500.0; // kg
    StateVector initial(position, velocity, mass, m_testTime);
    
    // Propagate
    time::Time targetTime = m_testTime + 3600.0;
    StateVector propagated = m_propagator->propagate(initial, targetTime);
    
    EXPECT_DOUBLE_EQ(propagated.getMass(), mass);
}

TEST_F(KeplerPropagatorTest, EdgeCases) {
    // Test near-zero position (should throw)
    math::Vector3D position(1e-10, 0.0, 0.0);
    math::Vector3D velocity(7000.0, 0.0, 0.0);
    StateVector state(position, velocity, 1000.0, m_testTime);
    
    EXPECT_THROW(KeplerPropagator::stateToElements(state, math::constants::EARTH_MU), 
                 std::runtime_error);
    
    // Test zero angular momentum (radial trajectory, should throw)
    position = math::Vector3D(7000000.0, 0.0, 0.0);
    velocity = math::Vector3D(5000.0, 0.0, 0.0); // Radial velocity
    state = StateVector(position, velocity, 1000.0, m_testTime);
    
    EXPECT_THROW(KeplerPropagator::stateToElements(state, math::constants::EARTH_MU), 
                 std::runtime_error);
}

} // namespace test
} // namespace twobody
} // namespace forces
} // namespace physics
} // namespace iloss