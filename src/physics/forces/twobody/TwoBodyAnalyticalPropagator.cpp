#include "physics/forces/twobody/TwoBodyAnalyticalPropagator.h"
#include "core/logging/Logger.h"
#include "core/coordinates/CoordinateTransformer.h"
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace iloss {
namespace physics {
namespace forces {
namespace twobody {

// LambertSolution implementation

bool LambertSolution::isValid() const {
    return std::isfinite(v1.magnitude()) && std::isfinite(v2.magnitude()) &&
           std::isfinite(a) && std::isfinite(e) && e >= 0.0;
}

// TwoBodyAnalyticalPropagator implementation

TwoBodyAnalyticalPropagator::TwoBodyAnalyticalPropagator(double mu, const std::string& centralBody)
    : m_mu(mu), m_centralBody(centralBody) {
    
    if (m_mu <= 0.0) {
        throw std::invalid_argument("Gravitational parameter must be positive");
    }
    
    // Create force model and propagator
    m_forceModel = std::make_unique<TwoBodyForceModel>("TwoBody_" + centralBody, m_mu);
    m_keplerPropagator = std::make_unique<KeplerPropagator>(m_mu);
    
    LOG_INFO("TwoBodyAnalyticalPropagator", "Created propagator for {} with μ = {} m³/s²", 
             centralBody, m_mu);
}

StateVector TwoBodyAnalyticalPropagator::propagateToTime(const StateVector& initialState,
                                                        const time::Time& targetTime) const {
    return m_keplerPropagator->propagate(initialState, targetTime);
}

StateVector TwoBodyAnalyticalPropagator::propagateByDuration(const StateVector& initialState,
                                                            double deltaTime) const {
    time::Time targetTime = initialState.getTime() + deltaTime;
    return m_keplerPropagator->propagate(initialState, targetTime);
}

StateVector TwoBodyAnalyticalPropagator::propagateToTrueAnomaly(const StateVector& initialState,
                                                               double targetNu,
                                                               int direction) const {
    
    // Convert state to elements (not used but shows we could use elements if needed)
    // OrbitalElements elements = KeplerPropagator::stateToElements(initialState, m_mu);
    
    // Calculate time to reach target true anomaly
    double dt = timeToTrueAnomaly(initialState, targetNu, direction);
    
    // Propagate by time
    return propagateByDuration(initialState, dt);
}

std::unique_ptr<StateHistory> TwoBodyAnalyticalPropagator::generateTrajectory(
    const StateVector& initialState,
    const time::Time& startTime,
    const time::Time& endTime,
    double timeStep) const {
    
    if (timeStep <= 0.0) {
        throw std::invalid_argument("Time step must be positive");
    }
    
    auto history = std::make_unique<StateHistory>();
    
    // Start from initial state or propagate to start time
    StateVector currentState = initialState;
    if (initialState.getTime() != startTime) {
        currentState = propagateToTime(initialState, startTime);
    }
    
    // Generate trajectory points
    time::Time currentTime = startTime;
    while (currentTime <= endTime) {
        history->addState(currentState);
        
        // Propagate to next time step
        currentTime = currentTime + timeStep;
        if (currentTime <= endTime) {
            currentState = propagateToTime(currentState, currentTime);
        }
    }
    
    // Add final point if not already at end time
    if (history->empty() || history->getLatestState().getTime() < endTime) {
        currentState = propagateToTime(currentState, endTime);
        history->addState(currentState);
    }
    
    return history;
}

std::vector<LambertSolution> TwoBodyAnalyticalPropagator::solveLambertProblem(
    const math::Vector3D& r1,
    const math::Vector3D& r2,
    double tof,
    bool isRetrograde,
    int revolutions) const {
    
    if (tof <= 0.0) {
        throw std::invalid_argument("Time of flight must be positive");
    }
    
    std::vector<LambertSolution> solutions;
    
    // For now, implement only the zero-revolution case
    if (revolutions == 0) {
        LambertSolution solution = solveLambertUniversal(r1, r2, tof, isRetrograde);
        if (solution.isValid()) {
            solutions.push_back(solution);
        }
    } else {
        LOG_WARN("TwoBodyAnalyticalPropagator", 
                 "Multi-revolution Lambert solutions not yet implemented");
    }
    
    return solutions;
}

OrbitalElements TwoBodyAnalyticalPropagator::determineOrbit(const math::Vector3D& position,
                                                           const math::Vector3D& velocity,
                                                           const time::Time& time) const {
    
    // Create state vector and convert to elements
    StateVector state(position, velocity, 0.0, time, iloss::coordinates::CoordinateSystemType::ECI_J2000);
    return KeplerPropagator::stateToElements(state, m_mu);
}

std::pair<double, double> TwoBodyAnalyticalPropagator::calculateHohmannTransfer(double r1, double r2) const {
    
    if (r1 <= 0.0 || r2 <= 0.0) {
        throw std::invalid_argument("Orbit radii must be positive");
    }
    
    // Calculate velocities in circular orbits
    double v1 = std::sqrt(m_mu / r1);
    double v2 = std::sqrt(m_mu / r2);
    
    // Semi-major axis of transfer ellipse
    double a_transfer = (r1 + r2) / 2.0;
    
    // Velocities at periapsis and apoapsis of transfer ellipse
    double v_transfer_peri = std::sqrt(m_mu * (2.0 / r1 - 1.0 / a_transfer));
    double v_transfer_apo = std::sqrt(m_mu * (2.0 / r2 - 1.0 / a_transfer));
    
    // Delta-v values
    double dv1 = std::abs(v_transfer_peri - v1);
    double dv2 = std::abs(v2 - v_transfer_apo);
    
    return std::make_pair(dv1, dv2);
}

std::vector<double> TwoBodyAnalyticalPropagator::calculateBiellipticTransfer(
    double r1, double r2, double rt) const {
    
    if (r1 <= 0.0 || r2 <= 0.0 || rt <= 0.0) {
        throw std::invalid_argument("All radii must be positive");
    }
    
    if (rt < std::max(r1, r2)) {
        throw std::invalid_argument("Transfer radius must be greater than both orbit radii");
    }
    
    // Circular velocities
    double v1 = std::sqrt(m_mu / r1);
    double v2 = std::sqrt(m_mu / r2);
    // double vt = std::sqrt(m_mu / rt);  // Not used in calculation
    
    // First transfer ellipse (r1 to rt)
    double a1 = (r1 + rt) / 2.0;
    double v1_dep = std::sqrt(m_mu * (2.0 / r1 - 1.0 / a1));
    double vt_arr1 = std::sqrt(m_mu * (2.0 / rt - 1.0 / a1));
    
    // Second transfer ellipse (rt to r2)
    double a2 = (rt + r2) / 2.0;
    double vt_dep = std::sqrt(m_mu * (2.0 / rt - 1.0 / a2));
    double v2_arr = std::sqrt(m_mu * (2.0 / r2 - 1.0 / a2));
    
    // Delta-v values
    double dv1 = std::abs(v1_dep - v1);
    double dv2 = std::abs(vt_dep - vt_arr1);
    double dv3 = std::abs(v2 - v2_arr);
    
    return {dv1, dv2, dv3};
}

double TwoBodyAnalyticalPropagator::calculatePlaneChange(double v, double deltaI) const {
    
    if (v <= 0.0) {
        throw std::invalid_argument("Velocity must be positive");
    }
    
    // Delta-v = 2 * v * sin(deltaI / 2)
    return 2.0 * v * std::sin(deltaI / 2.0);
}

double TwoBodyAnalyticalPropagator::calculateCombinedManeuver(double r1, double r2, double deltaI) const {
    
    // For optimal combined maneuver, perform plane change at apoapsis
    auto [dv1_hohmann, dv2_hohmann] = calculateHohmannTransfer(r1, r2);
    
    // Velocity at apoapsis of transfer orbit
    double a_transfer = (r1 + r2) / 2.0;
    double v_apo = std::sqrt(m_mu * (2.0 / r2 - 1.0 / a_transfer));
    
    // Combined maneuver at apoapsis
    double v2 = std::sqrt(m_mu / r2);
    double dv2_combined = std::sqrt(v_apo * v_apo + v2 * v2 - 
                                   2.0 * v_apo * v2 * std::cos(deltaI));
    
    return dv1_hohmann + dv2_combined;
}

double TwoBodyAnalyticalPropagator::timeToTrueAnomaly(const StateVector& currentState,
                                                      double targetNu,
                                                      int direction) const {
    
    // Convert state to elements
    OrbitalElements elements = KeplerPropagator::stateToElements(currentState, m_mu);
    
    if (elements.e >= 1.0) {
        LOG_WARN("TwoBodyAnalyticalPropagator", 
                 "Time to true anomaly for parabolic/hyperbolic orbits may be inaccurate");
    }
    
    // Normalize angles
    double currentNu = elements.nu;
    targetNu = math::constants::normalizeAnglePositive(targetNu);
    
    // Calculate angular difference
    double deltaNu = targetNu - currentNu;
    
    // Adjust for direction
    if (direction > 0 && deltaNu < 0) {
        deltaNu += math::constants::TWO_PI;
    } else if (direction < 0 && deltaNu > 0) {
        deltaNu -= math::constants::TWO_PI;
    }
    
    // For circular orbits, time is proportional to angle
    if (elements.e < 1e-8) {
        return std::abs(deltaNu) / elements.n;
    }
    
    // For elliptical orbits, use mean anomaly
    if (elements.e < 1.0) {
        double M1 = KeplerPropagator::trueToMeanAnomaly(currentNu, elements.e);
        double M2 = KeplerPropagator::trueToMeanAnomaly(targetNu, elements.e);
        double deltaM = M2 - M1;
        
        if (direction > 0 && deltaM < 0) {
            deltaM += math::constants::TWO_PI;
        } else if (direction < 0 && deltaM > 0) {
            deltaM -= math::constants::TWO_PI;
        }
        
        return deltaM / elements.n;
    }
    
    // For hyperbolic orbits
    double M1 = KeplerPropagator::trueToMeanAnomaly(currentNu, elements.e);
    double M2 = KeplerPropagator::trueToMeanAnomaly(targetNu, elements.e);
    return (M2 - M1) / elements.n;
}

time::Time TwoBodyAnalyticalPropagator::nextPeriapsisTime(const StateVector& currentState) const {
    
    // Time to reach true anomaly = 0
    double dt = timeToTrueAnomaly(currentState, 0.0, 1);
    return currentState.getTime() + dt;
}

time::Time TwoBodyAnalyticalPropagator::nextApoapsisTime(const StateVector& currentState) const {
    
    // Convert state to elements
    OrbitalElements elements = KeplerPropagator::stateToElements(currentState, m_mu);
    
    if (elements.e >= 1.0) {
        throw std::runtime_error("No apoapsis for parabolic/hyperbolic orbits");
    }
    
    // Time to reach true anomaly = π
    double dt = timeToTrueAnomaly(currentState, math::constants::PI, 1);
    return currentState.getTime() + dt;
}

std::vector<std::pair<double, double>> TwoBodyAnalyticalPropagator::calculateGroundTrack(
    const StateVector& initialState,
    double duration,
    double timeStep) const {
    
    std::vector<std::pair<double, double>> groundTrack;
    
    // Generate trajectory
    time::Time startTime = initialState.getTime();
    time::Time endTime = startTime + duration;
    auto trajectory = generateTrajectory(initialState, startTime, endTime, timeStep);
    
    // Convert each position to ground track
    coordinates::CoordinateTransformer transformer;
    
    // Generate time points and get states
    time::Time currentTime = startTime;
    for (size_t i = 0; i < trajectory->size(); ++i) {
        auto stateOpt = trajectory->getStateAtTime(currentTime);
        if (!stateOpt) {
            // Skip if no state at this time
            currentTime = currentTime + timeStep;
            continue;
        }
        StateVector state = *stateOpt;
        
        // Transform position to ECEF using EarthModel utilities
        math::Vector3D eciPos = state.getPosition();
        
        // Simple transformation (ignoring Earth rotation for now)
        // In a full implementation, would use proper ECI to ECEF transformation
        math::Vector3D ecefPos = eciPos;  // Simplified
        
        // Convert to geodetic coordinates
        double x = ecefPos.x();
        double y = ecefPos.y();
        double z = ecefPos.z();
        
        // Calculate longitude
        double lon = std::atan2(y, x);
        
        // Calculate latitude (simplified, ignoring Earth's oblateness)
        double r_xy = std::sqrt(x * x + y * y);
        double lat = std::atan2(z, r_xy);
        
        groundTrack.push_back(std::make_pair(lat, lon));
        
        // Increment time
        currentTime = currentTime + timeStep;
    }
    
    return groundTrack;
}

bool TwoBodyAnalyticalPropagator::canSeeGroundLocation(const OrbitalElements& elements,
                                                       double latitude,
                                                       double /*longitude*/,
                                                       double elevationAngle) const {
    
    // For a circular or elliptical orbit, check if the orbit's ground track
    // can pass within view of the location
    
    if (elements.e >= 1.0) {
        return false; // Parabolic/hyperbolic orbits don't have repeating ground tracks
    }
    
    // Maximum latitude reached by orbit
    double maxLat = elements.i;
    
    // Check if orbit can reach the latitude
    if (std::abs(latitude) > maxLat + elevationAngle) {
        return false;
    }
    
    // For now, return true if latitude is reachable
    // A more sophisticated check would consider the actual visibility cone
    return true;
}

LambertSolution TwoBodyAnalyticalPropagator::solveLambertUniversal(
    const math::Vector3D& r1,
    const math::Vector3D& r2,
    double tof,
    bool isRetrograde) const {
    
    LambertSolution solution;
    
    double r1_mag = r1.magnitude();
    double r2_mag = r2.magnitude();
    
    if (r1_mag < math::constants::POSITION_TOLERANCE || 
        r2_mag < math::constants::POSITION_TOLERANCE) {
        throw std::invalid_argument("Position vectors must be non-zero");
    }
    
    // Calculate chord and semi-perimeter
    math::Vector3D chord = r2 - r1;
    double c = chord.magnitude();
    double s = (r1_mag + r2_mag + c) / 2.0;
    
    // Calculate transfer angle
    double cos_dnu = r1.dot(r2) / (r1_mag * r2_mag);
    cos_dnu = std::max(-1.0, std::min(1.0, cos_dnu));
    
    // Determine if transfer is possible
    math::Vector3D h = r1.cross(r2);
    if (h.magnitude() < 1e-10) {
        LOG_WARN("TwoBodyAnalyticalPropagator", "Colinear vectors in Lambert problem");
        solution.a = 0.0;
        solution.e = 0.0;
        return solution;
    }
    
    // Check retrograde flag
    if ((h.z() < 0 && !isRetrograde) || (h.z() > 0 && isRetrograde)) {
        cos_dnu = -cos_dnu;
    }
    
    double sin_dnu = std::sqrt(1.0 - cos_dnu * cos_dnu);
    if (isRetrograde) sin_dnu = -sin_dnu;
    
    // Lambda parameter (would be used in full implementation)
    // double lambda = std::sqrt(r1_mag * r2_mag) * cos_dnu / s;
    
    // Time of flight function (simplified universal variable approach)
    // This is a simplified implementation - a full implementation would use
    // iterative solving of the universal variable equation
    
    // Estimate semi-major axis
    double a_min = s / 2.0;
    double tof_parabolic = std::sqrt(2.0 / m_mu) * (std::pow(s, 1.5) - std::pow(s - c, 1.5)) / 3.0;
    
    if (tof < tof_parabolic) {
        // Elliptical transfer - use approximate solution
        solution.a = a_min * 1.1; // Initial guess
        solution.e = std::sqrt(1.0 - (2.0 * std::sqrt(r1_mag * r2_mag) * cos_dnu) / (r1_mag + r2_mag));
    } else {
        // Hyperbolic transfer
        solution.a = -a_min * 0.9;
        solution.e = std::sqrt(1.0 + (2.0 * std::sqrt(r1_mag * r2_mag) * cos_dnu) / (r1_mag + r2_mag));
    }
    
    // Calculate velocities (simplified)
    double v1_r = (r2_mag - r1_mag) / tof;
    double v1_t = std::sqrt(m_mu / (r1_mag * solution.a)) * std::sqrt(2.0 - r1_mag / solution.a);
    
    // Construct velocity vectors (simplified - full implementation would be more complex)
    math::Vector3D v1_dir = chord / c;
    solution.v1 = v1_dir * v1_r + r1.cross(v1_dir) * (v1_t / r1_mag);
    
    // Similar calculation for v2
    solution.v2 = solution.v1; // Placeholder
    
    solution.isRetrograde = isRetrograde;
    solution.revolutions = 0;
    
    LOG_INFO("TwoBodyAnalyticalPropagator", 
             "Lambert solution: a={} km, e={}, TOF={} hours",
             solution.a / 1000.0, solution.e, tof / 3600.0);
    
    return solution;
}

} // namespace twobody
} // namespace forces
} // namespace physics
} // namespace iloss