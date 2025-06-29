#include "physics/forces/twobody/ConicSectionUtilities.h"
#include "core/logging/Logger.h"
#include "core/math/MathConstants.h"
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace iloss {
namespace physics {
namespace forces {
namespace twobody {

ConicSectionUtilities::OrbitType ConicSectionUtilities::classifyOrbit(double e, double tolerance) {
    if (std::abs(e) < tolerance) {
        return OrbitType::CIRCULAR;
    } else if (std::abs(e - 1.0) < tolerance) {
        return OrbitType::PARABOLIC;
    } else if (e < 1.0) {
        return OrbitType::ELLIPTICAL;
    } else {
        return OrbitType::HYPERBOLIC;
    }
}

double ConicSectionUtilities::calculateSpecificEnergy(const math::Vector3D& position,
                                                     const math::Vector3D& velocity,
                                                     double mu) {
    double r = position.magnitude();
    double v = velocity.magnitude();
    
    if (r < math::constants::POSITION_TOLERANCE) {
        throw std::runtime_error("Position too close to origin for energy calculation");
    }
    
    return (v * v / 2.0) - (mu / r);
}

math::Vector3D ConicSectionUtilities::calculateSpecificAngularMomentum(const math::Vector3D& position,
                                                                      const math::Vector3D& velocity) {
    return position.cross(velocity);
}

math::Vector3D ConicSectionUtilities::calculateEccentricityVector(const math::Vector3D& position,
                                                                 const math::Vector3D& velocity,
                                                                 double mu) {
    double r = position.magnitude();
    
    if (r < math::constants::POSITION_TOLERANCE) {
        throw std::runtime_error("Position too close to origin for eccentricity calculation");
    }
    
    math::Vector3D h = position.cross(velocity);
    math::Vector3D e_vec = (velocity.cross(h) / mu) - (position / r);
    
    return e_vec;
}

double ConicSectionUtilities::calculateSemiMajorAxis(double energy, double mu) {
    if (std::abs(energy) < 1e-12) {
        // Parabolic orbit
        return std::numeric_limits<double>::infinity();
    }
    
    return -mu / (2.0 * energy);
}

double ConicSectionUtilities::calculateSemiLatusRectum(double h_mag, double mu) {
    if (mu <= 0.0) {
        throw std::invalid_argument("Gravitational parameter must be positive");
    }
    
    return (h_mag * h_mag) / mu;
}

double ConicSectionUtilities::calculateRadius(double p, double e, double nu) {
    if (p <= 0.0) {
        throw std::invalid_argument("Semi-latus rectum must be positive");
    }
    
    double denominator = 1.0 + e * std::cos(nu);
    
    if (std::abs(denominator) < 1e-12) {
        throw std::runtime_error("Radius calculation failed: denominator too small");
    }
    
    return p / denominator;
}

double ConicSectionUtilities::calculateVelocity(double mu, double p, double e, double nu) {
    if (p <= 0.0 || mu <= 0.0) {
        throw std::invalid_argument("Semi-latus rectum and mu must be positive");
    }
    
    // v² = μ/p * (1 + 2e*cos(ν) + e²)
    double v_squared = (mu / p) * (1.0 + 2.0 * e * std::cos(nu) + e * e);
    
    if (v_squared < 0.0) {
        throw std::runtime_error("Velocity calculation failed: negative velocity squared");
    }
    
    return std::sqrt(v_squared);
}

double ConicSectionUtilities::calculateFlightPathAngle(double e, double nu) {
    return std::atan2(e * std::sin(nu), 1.0 + e * std::cos(nu));
}

double ConicSectionUtilities::calculateTrueAnomalyAtRadius(double r, double p, double e, bool ascending) {
    if (r <= 0.0 || p <= 0.0) {
        throw std::invalid_argument("Radius and semi-latus rectum must be positive");
    }
    
    // r = p / (1 + e*cos(ν))
    // cos(ν) = (p/r - 1) / e
    
    if (e < 1e-12) {
        // Circular orbit - true anomaly is undefined
        return 0.0;
    }
    
    double cos_nu = (p / r - 1.0) / e;
    
    // Check if radius is achievable for this orbit
    if (e < 1.0) {
        // Elliptical - check if radius is within periapsis and apoapsis
        double r_p = p / (1.0 + e);
        double r_a = p / (1.0 - e);
        if (r < r_p || r > r_a) {
            throw std::invalid_argument("Radius not achievable for this elliptical orbit");
        }
    } else if (e == 1.0) {
        // Parabolic - check if radius is greater than periapsis
        double r_p = p / 2.0;
        if (r < r_p) {
            throw std::invalid_argument("Radius not achievable for this parabolic orbit");
        }
    }
    
    // Clamp cos_nu to valid range
    cos_nu = std::max(-1.0, std::min(1.0, cos_nu));
    
    double nu = std::acos(cos_nu);
    if (!ascending) {
        nu = -nu;
    }
    
    return nu;
}

double ConicSectionUtilities::calculateTimeOfFlight(double nu1, double nu2, double a, double e, double mu) {
    if (e >= 1.0) {
        throw std::invalid_argument("Time of flight calculation only valid for elliptical orbits");
    }
    
    if (a <= 0.0 || mu <= 0.0) {
        throw std::invalid_argument("Semi-major axis and mu must be positive");
    }
    
    // Calculate mean motion
    double n = std::sqrt(mu / (a * a * a));
    
    // Convert true anomalies to mean anomalies
    double M1 = KeplerPropagator::trueToMeanAnomaly(nu1, e);
    double M2 = KeplerPropagator::trueToMeanAnomaly(nu2, e);
    
    // Calculate time difference
    double deltaM = M2 - M1;
    
    // Handle wrap-around for forward propagation
    if (deltaM < 0) {
        deltaM += math::constants::TWO_PI;
    }
    
    return deltaM / n;
}

double ConicSectionUtilities::calculateMeanMotion(double a, double mu) {
    if (a <= 0.0 || mu <= 0.0) {
        throw std::invalid_argument("Semi-major axis and mu must be positive");
    }
    
    return std::sqrt(mu / (a * a * a));
}

double ConicSectionUtilities::calculatePeriod(double a, double mu) {
    if (a <= 0.0) {
        // Non-elliptical orbit
        return std::numeric_limits<double>::infinity();
    }
    
    double n = calculateMeanMotion(a, mu);
    return math::constants::TWO_PI / n;
}

double ConicSectionUtilities::calculateVisVivaVelocity(double r, double a, double mu) {
    if (r <= 0.0 || mu <= 0.0) {
        throw std::invalid_argument("Radius and mu must be positive");
    }
    
    double v_squared;
    
    if (std::isinf(a)) {
        // Parabolic orbit
        v_squared = 2.0 * mu / r;
    } else {
        // Elliptical or hyperbolic
        v_squared = mu * (2.0 / r - 1.0 / a);
    }
    
    if (v_squared < 0.0) {
        throw std::runtime_error("Vis-viva calculation failed: negative velocity squared");
    }
    
    return std::sqrt(v_squared);
}

double ConicSectionUtilities::calculateEscapeVelocity(double r, double mu) {
    if (r <= 0.0 || mu <= 0.0) {
        throw std::invalid_argument("Radius and mu must be positive");
    }
    
    return std::sqrt(2.0 * mu / r);
}

double ConicSectionUtilities::calculateCircularVelocity(double r, double mu) {
    if (r <= 0.0 || mu <= 0.0) {
        throw std::invalid_argument("Radius and mu must be positive");
    }
    
    return std::sqrt(mu / r);
}

bool ConicSectionUtilities::isBoundOrbit(double energy) {
    return energy < 0.0;
}

bool ConicSectionUtilities::isClosedOrbit(double e) {
    return e < 1.0;
}

double ConicSectionUtilities::calculateHyperbolicExcessVelocity(double a, double mu) {
    if (a >= 0.0) {
        throw std::invalid_argument("Semi-major axis must be negative for hyperbolic orbit");
    }
    
    if (mu <= 0.0) {
        throw std::invalid_argument("Gravitational parameter must be positive");
    }
    
    return std::sqrt(-mu / a);
}

double ConicSectionUtilities::calculateHyperbolicTurningAngle(double e) {
    if (e <= 1.0) {
        throw std::invalid_argument("Eccentricity must be greater than 1 for hyperbolic orbit");
    }
    
    // δ = 2 * asin(1/e)
    return 2.0 * std::asin(1.0 / e);
}

double ConicSectionUtilities::calculateImpactParameter(double a, double e) {
    if (a >= 0.0) {
        throw std::invalid_argument("Semi-major axis must be negative for hyperbolic orbit");
    }
    
    if (e <= 1.0) {
        throw std::invalid_argument("Eccentricity must be greater than 1 for hyperbolic orbit");
    }
    
    // b = |a| * sqrt(e² - 1)
    return std::abs(a) * std::sqrt(e * e - 1.0);
}

double ConicSectionUtilities::calculateSphereOfInfluence(double a_orbit, double m_body, double m_parent) {
    if (a_orbit <= 0.0 || m_body <= 0.0 || m_parent <= 0.0) {
        throw std::invalid_argument("All parameters must be positive");
    }
    
    // r_SOI = a * (m_body / m_parent)^(2/5)
    return a_orbit * std::pow(m_body / m_parent, 0.4);
}

double ConicSectionUtilities::calculateHillSphere(double a_orbit, double e_orbit, 
                                                 double m_body, double m_parent) {
    if (a_orbit <= 0.0 || m_body <= 0.0 || m_parent <= 0.0) {
        throw std::invalid_argument("Masses and semi-major axis must be positive");
    }
    
    if (e_orbit < 0.0 || e_orbit >= 1.0) {
        throw std::invalid_argument("Eccentricity must be in [0, 1) for Hill sphere calculation");
    }
    
    // r_Hill = a * (1 - e) * (m_body / (3 * m_parent))^(1/3)
    double periapsis = a_orbit * (1.0 - e_orbit);
    return periapsis * std::cbrt(m_body / (3.0 * m_parent));
}

std::vector<math::Vector3D> ConicSectionUtilities::sampleOrbitPoints(const OrbitalElements& elements,
                                                                    double mu,
                                                                    int numPoints,
                                                                    double startNu,
                                                                    double endNu) {
    if (numPoints < 2) {
        throw std::invalid_argument("Number of points must be at least 2");
    }
    
    std::vector<math::Vector3D> points;
    points.reserve(numPoints);
    
    // Handle different orbit types
    OrbitType orbitType = classifyOrbit(elements.e);
    
    if (orbitType == OrbitType::HYPERBOLIC) {
        // For hyperbolic orbits, limit the range to physically meaningful values
        double nu_max = std::acos(-1.0 / elements.e) - 0.01; // Slightly less than asymptote
        startNu = std::max(startNu, -nu_max);
        endNu = std::min(endNu, nu_max);
    }
    
    // Sample points uniformly in true anomaly
    double deltaNu = (endNu - startNu) / (numPoints - 1);
    
    for (int i = 0; i < numPoints; ++i) {
        double nu = startNu + i * deltaNu;
        
        // Create temporary elements with current true anomaly
        OrbitalElements tempElements = elements;
        tempElements.nu = nu;
        
        // Convert to state vector and extract position
        try {
            StateVector state = KeplerPropagator::elementsToState(tempElements, mu, time::Time());
            points.push_back(state.getPosition());
        } catch (const std::exception& e) {
            LOG_WARN("ConicSectionUtilities", "Failed to sample point at nu={}: {}", 
                     math::constants::radiansToDegrees(nu), e.what());
        }
    }
    
    return points;
}

std::pair<double, double> ConicSectionUtilities::calculateApsides(double a, double e) {
    if (e < 0.0) {
        throw std::invalid_argument("Eccentricity must be non-negative");
    }
    
    double periapsis = a * (1.0 - e);
    double apoapsis;
    
    if (e >= 1.0) {
        // Parabolic or hyperbolic - no apoapsis
        apoapsis = std::numeric_limits<double>::infinity();
    } else {
        apoapsis = a * (1.0 + e);
    }
    
    return std::make_pair(periapsis, apoapsis);
}

std::pair<double, double> ConicSectionUtilities::calculateApsidalVelocities(double mu, double a, double e) {
    if (mu <= 0.0) {
        throw std::invalid_argument("Gravitational parameter must be positive");
    }
    
    if (e < 0.0) {
        throw std::invalid_argument("Eccentricity must be non-negative");
    }
    
    auto [r_p, r_a] = calculateApsides(a, e);
    
    double v_periapsis;
    double v_apoapsis;
    
    if (e < 1.0) {
        // Elliptical orbit
        v_periapsis = std::sqrt(mu * (1.0 + e) / (a * (1.0 - e)));
        v_apoapsis = std::sqrt(mu * (1.0 - e) / (a * (1.0 + e)));
    } else if (std::abs(e - 1.0) < 1e-10) {
        // Parabolic orbit
        v_periapsis = std::sqrt(2.0 * mu / r_p);
        v_apoapsis = 0.0; // At infinity
    } else {
        // Hyperbolic orbit
        v_periapsis = std::sqrt(mu * (e + 1.0) / (std::abs(a) * (e - 1.0)));
        double v_inf = calculateHyperbolicExcessVelocity(a, mu);
        v_apoapsis = v_inf; // Excess velocity at infinity
    }
    
    return std::make_pair(v_periapsis, v_apoapsis);
}

} // namespace twobody
} // namespace forces
} // namespace physics
} // namespace iloss