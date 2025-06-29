#include "physics/forces/twobody/KeplerPropagator.h"
#include "core/logging/Logger.h"
#include "core/math/MathConstants.h"
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <iomanip>

namespace iloss {
namespace physics {
namespace forces {
namespace twobody {

// OrbitalElements implementation

OrbitalElements::OrbitalElements()
    : a(0.0), e(0.0), i(0.0), omega(0.0), Omega(0.0), nu(0.0),
      M(0.0), E(0.0), p(0.0), n(0.0), T(0.0) {}

bool OrbitalElements::isValid() const {
    // Semi-major axis must be positive for elliptical orbits
    if (e < 1.0 && a <= 0.0) return false;
    
    // Eccentricity must be non-negative
    if (e < 0.0) return false;
    
    // Inclination must be in [0, π]
    if (i < 0.0 || i > math::constants::PI) return false;
    
    // For circular orbits, argument of periapsis is undefined
    // For equatorial orbits, RAAN is undefined
    
    return true;
}

std::string OrbitalElements::getOrbitType() const {
    if (std::abs(e) < 1e-8) return "circular";
    else if (e < 1.0) return "elliptical";
    else if (std::abs(e - 1.0) < 1e-8) return "parabolic";
    else return "hyperbolic";
}

double OrbitalElements::getApoapsis() const {
    if (e >= 1.0) return std::numeric_limits<double>::infinity();
    return a * (1.0 + e);
}

double OrbitalElements::getPeriapsis() const {
    return a * (1.0 - e);
}

std::string OrbitalElements::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "OrbitalElements[type=" << getOrbitType()
        << ", a=" << a / 1000.0 << " km"
        << ", e=" << e
        << ", i=" << math::constants::radiansToDegrees(i) << "°"
        << ", ω=" << math::constants::radiansToDegrees(omega) << "°"
        << ", Ω=" << math::constants::radiansToDegrees(Omega) << "°"
        << ", ν=" << math::constants::radiansToDegrees(nu) << "°"
        << ", T=" << T / 3600.0 << " hours]";
    return oss.str();
}

// KeplerPropagator implementation

KeplerPropagator::KeplerPropagator(double mu)
    : m_mu(mu) {
    if (m_mu <= 0.0) {
        throw std::invalid_argument("Gravitational parameter must be positive");
    }
    LOG_INFO("KeplerPropagator", "Created Kepler propagator with μ = {} m³/s²", m_mu);
}

StateVector KeplerPropagator::propagate(const StateVector& initialState,
                                       const time::Time& targetTime) {
    
    // Calculate time difference
    double dt = targetTime.getTime() - initialState.getTime().getTime();
    
    // Convert state to orbital elements
    OrbitalElements elements = stateToElements(initialState, m_mu);
    
    LOG_TRACE("KeplerPropagator", "Propagating {} orbit by {} seconds", 
              elements.getOrbitType(), dt);
    
    // Propagate mean anomaly
    double M_final = propagateMeanAnomaly(elements.M, elements.n, dt);
    
    // Update mean anomaly and solve for new true anomaly
    elements.M = M_final;
    elements.nu = meanToTrueAnomaly(M_final, elements.e);
    
    // Convert back to state vector
    StateVector propagatedState = elementsToState(elements, m_mu, targetTime);
    
    // Preserve mass from initial state
    propagatedState.setMass(initialState.getMass());
    
    return propagatedState;
}

StateVector KeplerPropagator::propagateByDuration(const StateVector& initialState,
                                                 double deltaTime) {
    time::Time targetTime = initialState.getTime() + deltaTime;
    return propagate(initialState, targetTime);
}

OrbitalElements KeplerPropagator::stateToElements(const StateVector& state, double mu) {
    OrbitalElements elements;
    
    const math::Vector3D& r = state.getPosition();
    const math::Vector3D& v = state.getVelocity();
    
    double r_mag = r.magnitude();
    double v_mag = v.magnitude();
    
    if (r_mag < math::constants::POSITION_TOLERANCE) {
        throw std::runtime_error("Cannot convert state to elements: position at origin");
    }
    
    // Calculate specific angular momentum
    math::Vector3D h = r.cross(v);
    double h_mag = h.magnitude();
    
    if (h_mag < 1e-10) {
        throw std::runtime_error("Cannot convert state to elements: zero angular momentum (radial trajectory)");
    }
    
    // Calculate node vector
    math::Vector3D n(-h.y(), h.x(), 0.0);
    double n_mag = n.magnitude();
    
    // Calculate eccentricity vector
    math::Vector3D e_vec = ((v.cross(h)) / mu) - (r / r_mag);
    elements.e = e_vec.magnitude();
    
    // Calculate specific energy
    double energy = (v_mag * v_mag / 2.0) - (mu / r_mag);
    
    // Semi-major axis
    if (std::abs(elements.e - 1.0) < 1e-10) {
        // Parabolic orbit
        elements.a = std::numeric_limits<double>::infinity();
        elements.p = h_mag * h_mag / mu;
    } else {
        // Elliptical or hyperbolic
        elements.a = -mu / (2.0 * energy);
        elements.p = elements.a * (1.0 - elements.e * elements.e);
    }
    
    // Inclination
    elements.i = std::acos(h.z() / h_mag);
    
    // Right ascension of ascending node
    if (n_mag > 1e-10) {
        elements.Omega = std::acos(n.x() / n_mag);
        if (n.y() < 0) {
            elements.Omega = math::constants::TWO_PI - elements.Omega;
        }
    } else {
        // Equatorial orbit - RAAN is undefined, set to 0
        elements.Omega = 0.0;
    }
    
    // Argument of periapsis
    if (n_mag > 1e-10 && elements.e > 1e-10) {
        double cos_omega = n.dot(e_vec) / (n_mag * elements.e);
        cos_omega = std::max(-1.0, std::min(1.0, cos_omega)); // Clamp for numerical safety
        elements.omega = std::acos(cos_omega);
        if (e_vec.z() < 0) {
            elements.omega = math::constants::TWO_PI - elements.omega;
        }
    } else if (elements.e > 1e-10) {
        // Equatorial non-circular orbit
        elements.omega = std::atan2(e_vec.y(), e_vec.x());
        elements.omega = math::constants::normalizeAnglePositive(elements.omega);
    } else {
        // Circular orbit - argument of periapsis is undefined, set to 0
        elements.omega = 0.0;
    }
    
    // True anomaly
    if (elements.e > 1e-10) {
        double cos_nu = e_vec.dot(r) / (elements.e * r_mag);
        cos_nu = std::max(-1.0, std::min(1.0, cos_nu)); // Clamp for numerical safety
        elements.nu = std::acos(cos_nu);
        if (r.dot(v) < 0) {
            elements.nu = math::constants::TWO_PI - elements.nu;
        }
    } else {
        // Circular orbit - measure from ascending node or reference direction
        if (n_mag > 1e-10) {
            double cos_nu = n.dot(r) / (n_mag * r_mag);
            cos_nu = std::max(-1.0, std::min(1.0, cos_nu));
            elements.nu = std::acos(cos_nu);
            if (r.z() < 0) {
                elements.nu = math::constants::TWO_PI - elements.nu;
            }
        } else {
            // Circular equatorial orbit
            elements.nu = std::atan2(r.y(), r.x());
        }
    }
    
    // Calculate anomalies
    elements.E = trueToEccentricAnomaly(elements.nu, elements.e);
    elements.M = trueToMeanAnomaly(elements.nu, elements.e);
    
    // Mean motion and period
    if (elements.e < 1.0) {
        elements.n = std::sqrt(mu / (elements.a * elements.a * elements.a));
        elements.T = math::constants::TWO_PI / elements.n;
    } else {
        // Hyperbolic or parabolic
        elements.n = std::sqrt(mu / std::abs(elements.a * elements.a * elements.a));
        elements.T = std::numeric_limits<double>::infinity();
    }
    
    return elements;
}

StateVector KeplerPropagator::elementsToState(const OrbitalElements& elements,
                                             double mu,
                                             const time::Time& time) {
    
    if (!elements.isValid()) {
        throw std::runtime_error("Invalid orbital elements");
    }
    
    // Calculate position and velocity in perifocal coordinates
    double r_mag, v_rad, v_tan;
    
    if (std::abs(elements.e - 1.0) < 1e-10) {
        // Parabolic orbit
        r_mag = elements.p / (1.0 + std::cos(elements.nu));
        v_rad = std::sqrt(mu / elements.p) * std::sin(elements.nu);
        v_tan = std::sqrt(mu / elements.p) * (1.0 + std::cos(elements.nu));
    } else {
        // Elliptical or hyperbolic
        r_mag = elements.p / (1.0 + elements.e * std::cos(elements.nu));
        double h_mag = std::sqrt(mu * elements.p);
        v_rad = (mu * elements.e * std::sin(elements.nu)) / h_mag;
        v_tan = h_mag / r_mag;
    }
    
    // Position in perifocal frame
    double cos_nu = std::cos(elements.nu);
    double sin_nu = std::sin(elements.nu);
    math::Vector3D r_pqw(r_mag * cos_nu, r_mag * sin_nu, 0.0);
    
    // Velocity in perifocal frame
    // v_p = dr/dt * cos(nu) - r * dnu/dt * sin(nu)
    // v_q = dr/dt * sin(nu) + r * dnu/dt * cos(nu)
    // For circular/elliptical orbits: dr/dt = v_rad, r * dnu/dt = v_tan
    math::Vector3D v_pqw(v_rad * cos_nu - v_tan * sin_nu,
                         v_rad * sin_nu + v_tan * cos_nu,
                         0.0);
    
    // Rotation matrices
    double cos_omega = std::cos(elements.omega);
    double sin_omega = std::sin(elements.omega);
    double cos_Omega = std::cos(elements.Omega);
    double sin_Omega = std::sin(elements.Omega);
    double cos_i = std::cos(elements.i);
    double sin_i = std::sin(elements.i);
    
    // Combined rotation matrix from perifocal to inertial
    math::Matrix3D R;
    R(0, 0) = cos_Omega * cos_omega - sin_Omega * sin_omega * cos_i;
    R(0, 1) = -cos_Omega * sin_omega - sin_Omega * cos_omega * cos_i;
    R(0, 2) = sin_Omega * sin_i;
    
    R(1, 0) = sin_Omega * cos_omega + cos_Omega * sin_omega * cos_i;
    R(1, 1) = -sin_Omega * sin_omega + cos_Omega * cos_omega * cos_i;
    R(1, 2) = -cos_Omega * sin_i;
    
    R(2, 0) = sin_omega * sin_i;
    R(2, 1) = cos_omega * sin_i;
    R(2, 2) = cos_i;
    
    // Transform to inertial frame
    math::Vector3D r_inertial = R * r_pqw;
    math::Vector3D v_inertial = R * v_pqw;
    
    // Create state vector (assuming ECI_J2000 coordinate system)
    StateVector state(r_inertial, v_inertial, 0.0, time,
                     iloss::coordinates::CoordinateSystemType::ECI_J2000);
    
    return state;
}

double KeplerPropagator::solveKeplerEllipse(double M, double e, 
                                           double tolerance, int maxIterations) {
    
    // Normalize mean anomaly to [0, 2π]
    M = math::constants::normalizeAnglePositive(M);
    
    // Initial guess using series expansion
    double E = M;
    if (e > 0.8) {
        E = math::constants::PI;
    } else if (e > 0.3) {
        E = M + e * std::sin(M);
    }
    
    // Newton-Raphson iteration
    int iteration = 0;
    double delta;
    
    do {
        double sin_E = std::sin(E);
        double cos_E = std::cos(E);
        double f = E - e * sin_E - M;
        double f_prime = 1.0 - e * cos_E;
        
        // Avoid division by very small numbers
        if (std::abs(f_prime) < 1e-12) {
            f_prime = 1e-12;
        }
        
        delta = f / f_prime;
        E -= delta;
        
        iteration++;
        
        if (iteration >= maxIterations) {
            LOG_WARN("KeplerPropagator", "Kepler equation convergence slow: {} iterations, error={}", 
                     iteration, std::abs(delta));
            break;
        }
        
    } while (std::abs(delta) > tolerance);
    
    return math::constants::normalizeAnglePositive(E);
}

double KeplerPropagator::solveKeplerHyperbola(double M, double e,
                                             double tolerance, int maxIterations) {
    
    // Initial guess
    double H = M;
    if (std::abs(M) > 1.0) {
        H = M / std::abs(M) * std::log(2.0 * std::abs(M) / e + 1.8);
    }
    
    // Newton-Raphson iteration
    int iteration = 0;
    double delta;
    
    do {
        double sinh_H = std::sinh(H);
        double cosh_H = std::cosh(H);
        double f = e * sinh_H - H - M;
        double f_prime = e * cosh_H - 1.0;
        
        // Avoid division by very small numbers
        if (std::abs(f_prime) < 1e-12) {
            f_prime = 1e-12;
        }
        
        delta = f / f_prime;
        H -= delta;
        
        iteration++;
        
        if (iteration >= maxIterations) {
            LOG_WARN("KeplerPropagator", "Hyperbolic Kepler equation convergence slow: {} iterations", 
                     iteration);
            break;
        }
        
    } while (std::abs(delta) > tolerance);
    
    return H;
}

double KeplerPropagator::solveKeplerParabola(double M) {
    // For parabolic orbits, use Barker's equation
    // tan(ν/2) = (3M + sqrt(9M² + 1))^(1/3) - (3M - sqrt(9M² + 1))^(1/3)
    
    double sqrt_term = std::sqrt(9.0 * M * M + 1.0);
    double term1 = std::cbrt(3.0 * M + sqrt_term);
    double term2 = std::cbrt(std::abs(3.0 * M - sqrt_term));
    if (3.0 * M - sqrt_term < 0) term2 = -term2;
    
    double tan_half_nu = term1 - term2;
    double nu = 2.0 * std::atan(tan_half_nu);
    
    return nu;
}

double KeplerPropagator::trueToEccentricAnomaly(double nu, double e) {
    if (e < 1.0) {
        // Elliptical orbit
        double E = std::atan2(std::sqrt(1.0 - e * e) * std::sin(nu),
                             e + std::cos(nu));
        return math::constants::normalizeAnglePositive(E);
    } else if (e > 1.0) {
        // Hyperbolic orbit
        double H = std::asinh(std::sqrt(e * e - 1.0) * std::sin(nu) /
                             (1.0 + e * std::cos(nu)));
        return H;
    } else {
        // Parabolic orbit - no eccentric anomaly
        return nu;
    }
}

double KeplerPropagator::eccentricToTrueAnomaly(double E, double e) {
    if (e < 1.0) {
        // Elliptical orbit
        double nu = std::atan2(std::sqrt(1.0 - e * e) * std::sin(E),
                              std::cos(E) - e);
        return math::constants::normalizeAnglePositive(nu);
    } else if (e > 1.0) {
        // Hyperbolic orbit
        double nu = std::atan2(std::sqrt(e * e - 1.0) * std::sinh(E),
                              e - std::cosh(E));
        return nu;
    } else {
        // Parabolic orbit
        return E;
    }
}

double KeplerPropagator::meanToTrueAnomaly(double M, double e) {
    if (e < 1.0) {
        // Elliptical orbit
        double E = solveKeplerEllipse(M, e);
        return eccentricToTrueAnomaly(E, e);
    } else if (e > 1.0) {
        // Hyperbolic orbit
        double H = solveKeplerHyperbola(M, e);
        return eccentricToTrueAnomaly(H, e);
    } else {
        // Parabolic orbit
        return solveKeplerParabola(M);
    }
}

double KeplerPropagator::trueToMeanAnomaly(double nu, double e) {
    if (e < 1.0) {
        // Elliptical orbit
        double E = trueToEccentricAnomaly(nu, e);
        double M = E - e * std::sin(E);
        return math::constants::normalizeAnglePositive(M);
    } else if (e > 1.0) {
        // Hyperbolic orbit
        double H = trueToEccentricAnomaly(nu, e);
        double M = e * std::sinh(H) - H;
        return M;
    } else {
        // Parabolic orbit
        double tan_half_nu = std::tan(nu / 2.0);
        double M = tan_half_nu / 3.0 * (1.0 + tan_half_nu * tan_half_nu);
        return M;
    }
}

double KeplerPropagator::calculateFlightPathAngle(double nu, double e) {
    return std::atan2(e * std::sin(nu), 1.0 + e * std::cos(nu));
}

void KeplerPropagator::setGravitationalParameter(double mu) {
    if (mu <= 0.0) {
        throw std::invalid_argument("Gravitational parameter must be positive");
    }
    m_mu = mu;
}

double KeplerPropagator::propagateMeanAnomaly(double M0, double n, double dt) {
    double M = M0 + n * dt;
    // Only normalize for elliptical orbits
    if (n > 0) {
        M = math::constants::normalizeAnglePositive(M);
    }
    return M;
}

} // namespace twobody
} // namespace forces
} // namespace physics
} // namespace iloss