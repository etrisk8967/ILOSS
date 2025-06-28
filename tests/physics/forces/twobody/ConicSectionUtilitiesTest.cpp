#include <gtest/gtest.h>
#include "physics/forces/twobody/ConicSectionUtilities.h"
#include "core/math/MathConstants.h"
#include <cmath>

namespace iloss {
namespace physics {
namespace forces {
namespace twobody {
namespace test {

class ConicSectionUtilitiesTest : public ::testing::Test {
protected:
    void SetUp() override {
        m_mu = math::constants::EARTH_MU;
    }
    
    double m_mu;
};

TEST_F(ConicSectionUtilitiesTest, OrbitClassification) {
    using OrbitType = ConicSectionUtilities::OrbitType;
    
    // Test with default tolerance
    EXPECT_EQ(ConicSectionUtilities::classifyOrbit(0.0), OrbitType::CIRCULAR);
    EXPECT_EQ(ConicSectionUtilities::classifyOrbit(1e-9), OrbitType::CIRCULAR);
    EXPECT_EQ(ConicSectionUtilities::classifyOrbit(0.5), OrbitType::ELLIPTICAL);
    EXPECT_EQ(ConicSectionUtilities::classifyOrbit(0.99), OrbitType::ELLIPTICAL);
    EXPECT_EQ(ConicSectionUtilities::classifyOrbit(1.0), OrbitType::PARABOLIC);
    EXPECT_EQ(ConicSectionUtilities::classifyOrbit(1.5), OrbitType::HYPERBOLIC);
    
    // Test with custom tolerance
    double tolerance = 0.01;
    EXPECT_EQ(ConicSectionUtilities::classifyOrbit(0.005, tolerance), OrbitType::CIRCULAR);
    EXPECT_EQ(ConicSectionUtilities::classifyOrbit(0.02, tolerance), OrbitType::ELLIPTICAL);
    EXPECT_EQ(ConicSectionUtilities::classifyOrbit(0.995, tolerance), OrbitType::PARABOLIC);
    EXPECT_EQ(ConicSectionUtilities::classifyOrbit(1.005, tolerance), OrbitType::PARABOLIC);
}

TEST_F(ConicSectionUtilitiesTest, SpecificEnergy) {
    // Circular orbit at 400 km altitude
    double radius = math::constants::EARTH_RADIUS_EQUATORIAL + 400000.0;
    math::Vector3D position(radius, 0.0, 0.0);
    double v_circular = std::sqrt(m_mu / radius);
    math::Vector3D velocity(0.0, v_circular, 0.0);
    
    double energy = ConicSectionUtilities::calculateSpecificEnergy(position, velocity, m_mu);
    
    // For circular orbit, energy = -μ/(2a) = -μ/(2r)
    double expected_energy = -m_mu / (2.0 * radius);
    EXPECT_NEAR(energy, expected_energy, 1e-9);
    
    // Test bound orbit (negative energy)
    EXPECT_TRUE(ConicSectionUtilities::isBoundOrbit(energy));
    
    // Test escape trajectory (zero energy)
    double v_escape = std::sqrt(2.0 * m_mu / radius);
    velocity = math::Vector3D(0.0, v_escape, 0.0);
    energy = ConicSectionUtilities::calculateSpecificEnergy(position, velocity, m_mu);
    EXPECT_NEAR(energy, 0.0, 1e-9);
    
    // Test hyperbolic trajectory (positive energy)
    velocity = math::Vector3D(0.0, v_escape * 1.5, 0.0);
    energy = ConicSectionUtilities::calculateSpecificEnergy(position, velocity, m_mu);
    EXPECT_GT(energy, 0.0);
    EXPECT_FALSE(ConicSectionUtilities::isBoundOrbit(energy));
}

TEST_F(ConicSectionUtilitiesTest, AngularMomentum) {
    math::Vector3D position(7000000.0, 0.0, 0.0);
    math::Vector3D velocity(0.0, 7500.0, 0.0);
    
    math::Vector3D h = ConicSectionUtilities::calculateSpecificAngularMomentum(position, velocity);
    
    // For this configuration, h should point in +z direction
    EXPECT_NEAR(h.x(), 0.0, 1e-10);
    EXPECT_NEAR(h.y(), 0.0, 1e-10);
    EXPECT_GT(h.z(), 0.0);
    
    // Magnitude should be r × v
    double expected_magnitude = position.magnitude() * velocity.magnitude();
    EXPECT_NEAR(h.magnitude(), expected_magnitude, 1e-10);
}

TEST_F(ConicSectionUtilitiesTest, EccentricityVector) {
    // Circular orbit
    double radius = 7000000.0;
    math::Vector3D position(radius, 0.0, 0.0);
    double v_circular = std::sqrt(m_mu / radius);
    math::Vector3D velocity(0.0, v_circular, 0.0);
    
    math::Vector3D e_vec = ConicSectionUtilities::calculateEccentricityVector(position, velocity, m_mu);
    
    // For circular orbit, eccentricity should be near zero
    EXPECT_NEAR(e_vec.magnitude(), 0.0, 1e-10);
    
    // Elliptical orbit at periapsis
    double periapsis = 6678000.0;
    double apoapsis = 42164000.0;
    double a = (periapsis + apoapsis) / 2.0;
    double v_periapsis = std::sqrt(m_mu * (2.0 / periapsis - 1.0 / a));
    
    position = math::Vector3D(periapsis, 0.0, 0.0);
    velocity = math::Vector3D(0.0, v_periapsis, 0.0);
    
    e_vec = ConicSectionUtilities::calculateEccentricityVector(position, velocity, m_mu);
    
    double expected_e = (apoapsis - periapsis) / (apoapsis + periapsis);
    EXPECT_NEAR(e_vec.magnitude(), expected_e, 1e-10);
    
    // At periapsis, eccentricity vector should point along position vector
    EXPECT_GT(e_vec.x(), 0.0);
    EXPECT_NEAR(e_vec.y(), 0.0, 1e-10);
    EXPECT_NEAR(e_vec.z(), 0.0, 1e-10);
}

TEST_F(ConicSectionUtilitiesTest, SemiMajorAxis) {
    // Circular orbit
    double energy = -m_mu / (2.0 * 7000000.0);
    double a = ConicSectionUtilities::calculateSemiMajorAxis(energy, m_mu);
    EXPECT_NEAR(a, 7000000.0, 1e-6);
    
    // Parabolic orbit (zero energy)
    a = ConicSectionUtilities::calculateSemiMajorAxis(0.0, m_mu);
    EXPECT_TRUE(std::isinf(a));
    
    // Hyperbolic orbit (positive energy)
    energy = m_mu / (2.0 * 7000000.0);
    a = ConicSectionUtilities::calculateSemiMajorAxis(energy, m_mu);
    EXPECT_LT(a, 0.0);
    EXPECT_NEAR(a, -7000000.0, 1e-6);
}

TEST_F(ConicSectionUtilitiesTest, SemiLatusRectum) {
    // For circular orbit, p = a
    double h_mag = std::sqrt(m_mu * 7000000.0);
    double p = ConicSectionUtilities::calculateSemiLatusRectum(h_mag, m_mu);
    EXPECT_NEAR(p, 7000000.0, 1e-6);
}

TEST_F(ConicSectionUtilitiesTest, RadiusAtTrueAnomaly) {
    double p = 7000000.0;
    double e = 0.3;
    
    // At periapsis (nu = 0)
    double r = ConicSectionUtilities::calculateRadius(p, e, 0.0);
    EXPECT_NEAR(r, p / (1.0 + e), 1e-6);
    
    // At apoapsis (nu = π)
    r = ConicSectionUtilities::calculateRadius(p, e, math::constants::PI);
    EXPECT_NEAR(r, p / (1.0 - e), 1e-6);
    
    // At nu = π/2
    r = ConicSectionUtilities::calculateRadius(p, e, math::constants::PI / 2.0);
    EXPECT_NEAR(r, p, 1e-6);
}

TEST_F(ConicSectionUtilitiesTest, VelocityAtTrueAnomaly) {
    double p = 7000000.0;
    double e = 0.2;
    
    // At periapsis
    double v = ConicSectionUtilities::calculateVelocity(m_mu, p, e, 0.0);
    double expected_v = std::sqrt(m_mu / p * (1.0 + 2.0 * e + e * e));
    EXPECT_NEAR(v, expected_v, 1e-6);
    
    // At apoapsis
    v = ConicSectionUtilities::calculateVelocity(m_mu, p, e, math::constants::PI);
    expected_v = std::sqrt(m_mu / p * (1.0 - 2.0 * e + e * e));
    EXPECT_NEAR(v, expected_v, 1e-6);
}

TEST_F(ConicSectionUtilitiesTest, TrueAnomalyAtRadius) {
    double p = 8000000.0;
    double e = 0.3;
    
    // Test at periapsis radius
    double r_p = p / (1.0 + e);
    double nu = ConicSectionUtilities::calculateTrueAnomalyAtRadius(r_p, p, e, true);
    EXPECT_NEAR(nu, 0.0, 1e-10);
    
    // Test at apoapsis radius
    double r_a = p / (1.0 - e);
    nu = ConicSectionUtilities::calculateTrueAnomalyAtRadius(r_a, p, e, true);
    EXPECT_NEAR(std::abs(nu), math::constants::PI, 1e-10);
    
    // Test invalid radius (below periapsis)
    EXPECT_THROW(ConicSectionUtilities::calculateTrueAnomalyAtRadius(r_p * 0.5, p, e),
                 std::invalid_argument);
}

TEST_F(ConicSectionUtilitiesTest, TimeOfFlight) {
    double a = 10000000.0;
    double e = 0.2;
    
    // Time from periapsis to apoapsis (half period)
    double tof = ConicSectionUtilities::calculateTimeOfFlight(0.0, math::constants::PI, a, e, m_mu);
    double period = 2.0 * math::constants::PI * std::sqrt(a * a * a / m_mu);
    EXPECT_NEAR(tof, period / 2.0, 1e-6);
    
    // Quarter orbit
    tof = ConicSectionUtilities::calculateTimeOfFlight(0.0, math::constants::PI / 2.0, a, e, m_mu);
    EXPECT_LT(tof, period / 4.0); // Less than quarter period due to Kepler's second law
}

TEST_F(ConicSectionUtilitiesTest, MeanMotionAndPeriod) {
    double a = 7000000.0;
    
    double n = ConicSectionUtilities::calculateMeanMotion(a, m_mu);
    double expected_n = std::sqrt(m_mu / (a * a * a));
    EXPECT_NEAR(n, expected_n, 1e-10);
    
    double T = ConicSectionUtilities::calculatePeriod(a, m_mu);
    EXPECT_NEAR(T, 2.0 * math::constants::PI / n, 1e-10);
    
    // Non-elliptical orbit
    T = ConicSectionUtilities::calculatePeriod(-a, m_mu);
    EXPECT_TRUE(std::isinf(T));
}

TEST_F(ConicSectionUtilitiesTest, VisVivaEquation) {
    double r = 7000000.0;
    double a = 8000000.0;
    
    double v = ConicSectionUtilities::calculateVisVivaVelocity(r, a, m_mu);
    double expected_v = std::sqrt(m_mu * (2.0 / r - 1.0 / a));
    EXPECT_NEAR(v, expected_v, 1e-10);
    
    // Circular orbit (r = a)
    v = ConicSectionUtilities::calculateVisVivaVelocity(r, r, m_mu);
    double v_circular = ConicSectionUtilities::calculateCircularVelocity(r, m_mu);
    EXPECT_NEAR(v, v_circular, 1e-10);
    
    // Escape velocity (a = ∞)
    v = ConicSectionUtilities::calculateVisVivaVelocity(r, 
        std::numeric_limits<double>::infinity(), m_mu);
    double v_escape = ConicSectionUtilities::calculateEscapeVelocity(r, m_mu);
    EXPECT_NEAR(v, v_escape, 1e-10);
}

TEST_F(ConicSectionUtilitiesTest, HyperbolicParameters) {
    double a = -10000000.0; // Negative for hyperbola
    double e = 1.5;
    
    // Excess velocity
    double v_inf = ConicSectionUtilities::calculateHyperbolicExcessVelocity(a, m_mu);
    EXPECT_GT(v_inf, 0.0);
    EXPECT_NEAR(v_inf, std::sqrt(m_mu / std::abs(a)), 1e-10);
    
    // Turning angle
    double delta = ConicSectionUtilities::calculateHyperbolicTurningAngle(e);
    EXPECT_GT(delta, 0.0);
    EXPECT_LT(delta, math::constants::PI);
    
    // Impact parameter
    double b = ConicSectionUtilities::calculateImpactParameter(a, e);
    EXPECT_GT(b, 0.0);
    EXPECT_NEAR(b, std::abs(a) * std::sqrt(e * e - 1.0), 1e-10);
}

TEST_F(ConicSectionUtilitiesTest, SphereOfInfluence) {
    // Earth around Sun
    double a_earth = math::constants::AU;
    double m_earth = math::constants::EARTH_MASS;
    double m_sun = math::constants::SUN_MU / math::constants::GRAVITATIONAL_CONSTANT;
    
    double r_soi = ConicSectionUtilities::calculateSphereOfInfluence(a_earth, m_earth, m_sun);
    
    // Earth's SOI is approximately 0.01 AU or ~1.5 million km
    EXPECT_GT(r_soi, 900000000.0); // Greater than 900,000 km
    EXPECT_LT(r_soi, 1500000000.0); // Less than 1.5 million km
}

TEST_F(ConicSectionUtilitiesTest, HillSphere) {
    // Earth around Sun
    double a_earth = math::constants::AU;
    double e_earth = 0.0167;
    double m_earth = math::constants::EARTH_MASS;
    double m_sun = math::constants::SUN_MU / math::constants::GRAVITATIONAL_CONSTANT;
    
    double r_hill = ConicSectionUtilities::calculateHillSphere(a_earth, e_earth, m_earth, m_sun);
    
    // Earth's Hill sphere is approximately 1.5 million km
    EXPECT_GT(r_hill, 1400000000.0); // Greater than 1.4 million km
    EXPECT_LT(r_hill, 1600000000.0); // Less than 1.6 million km
}

TEST_F(ConicSectionUtilitiesTest, OrbitPointSampling) {
    OrbitalElements elements;
    elements.a = 7000000.0;
    elements.e = 0.2;
    elements.i = 30.0 * math::constants::DEG_TO_RAD;
    elements.omega = 45.0 * math::constants::DEG_TO_RAD;
    elements.Omega = 60.0 * math::constants::DEG_TO_RAD;
    elements.nu = 0.0;
    elements.p = elements.a * (1.0 - elements.e * elements.e);
    
    int numPoints = 10;
    auto points = ConicSectionUtilities::sampleOrbitPoints(elements, m_mu, numPoints);
    
    EXPECT_EQ(points.size(), numPoints);
    
    // Check that first point is at periapsis
    double r_periapsis = elements.a * (1.0 - elements.e);
    EXPECT_NEAR(points[0].magnitude(), r_periapsis, 100.0); // 100m tolerance
    
    // Check that points are properly spaced
    for (size_t i = 1; i < points.size(); ++i) {
        EXPECT_GT((points[i] - points[i-1]).magnitude(), 0.0);
    }
}

TEST_F(ConicSectionUtilitiesTest, Apsides) {
    double a = 10000000.0;
    double e = 0.3;
    
    auto [r_p, r_a] = ConicSectionUtilities::calculateApsides(a, e);
    
    EXPECT_NEAR(r_p, a * (1.0 - e), 1e-10);
    EXPECT_NEAR(r_a, a * (1.0 + e), 1e-10);
    
    // Parabolic orbit
    auto [r_p_para, r_a_para] = ConicSectionUtilities::calculateApsides(a, 1.0);
    EXPECT_DOUBLE_EQ(r_p_para, 0.0);
    EXPECT_TRUE(std::isinf(r_a_para));
    
    // Hyperbolic orbit
    auto [r_p_hyp, r_a_hyp] = ConicSectionUtilities::calculateApsides(-a, 1.5);
    EXPECT_LT(r_p_hyp, 0.0); // Negative periapsis for negative a
    EXPECT_TRUE(std::isinf(r_a_hyp));
}

TEST_F(ConicSectionUtilitiesTest, ApsidalVelocities) {
    double a = 10000000.0;
    double e = 0.2;
    
    auto [v_p, v_a] = ConicSectionUtilities::calculateApsidalVelocities(m_mu, a, e);
    
    // Use vis-viva to verify
    double r_p = a * (1.0 - e);
    double r_a = a * (1.0 + e);
    double expected_v_p = std::sqrt(m_mu * (2.0 / r_p - 1.0 / a));
    double expected_v_a = std::sqrt(m_mu * (2.0 / r_a - 1.0 / a));
    
    EXPECT_NEAR(v_p, expected_v_p, 1e-10);
    EXPECT_NEAR(v_a, expected_v_a, 1e-10);
    
    // Periapsis velocity should be greater than apoapsis velocity
    EXPECT_GT(v_p, v_a);
}

TEST_F(ConicSectionUtilitiesTest, ClosedOrbitCheck) {
    EXPECT_TRUE(ConicSectionUtilities::isClosedOrbit(0.0));
    EXPECT_TRUE(ConicSectionUtilities::isClosedOrbit(0.5));
    EXPECT_TRUE(ConicSectionUtilities::isClosedOrbit(0.999));
    EXPECT_FALSE(ConicSectionUtilities::isClosedOrbit(1.0));
    EXPECT_FALSE(ConicSectionUtilities::isClosedOrbit(1.5));
}

TEST_F(ConicSectionUtilitiesTest, EdgeCases) {
    // Test with very small position
    math::Vector3D position(1e-10, 0.0, 0.0);
    math::Vector3D velocity(7000.0, 0.0, 0.0);
    
    EXPECT_THROW(ConicSectionUtilities::calculateSpecificEnergy(position, velocity, m_mu),
                 std::runtime_error);
    
    EXPECT_THROW(ConicSectionUtilities::calculateEccentricityVector(position, velocity, m_mu),
                 std::runtime_error);
    
    // Test invalid parameters
    EXPECT_THROW(ConicSectionUtilities::calculateSemiLatusRectum(1000.0, -m_mu),
                 std::invalid_argument);
    
    EXPECT_THROW(ConicSectionUtilities::calculateRadius(-1000.0, 0.5, 0.0),
                 std::invalid_argument);
}

} // namespace test
} // namespace twobody
} // namespace forces
} // namespace physics
} // namespace iloss