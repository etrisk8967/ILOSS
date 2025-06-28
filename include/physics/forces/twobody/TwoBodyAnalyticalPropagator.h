#pragma once

#include "physics/forces/twobody/TwoBodyForceModel.h"
#include "physics/forces/twobody/KeplerPropagator.h"
#include "physics/forces/twobody/ConicSectionUtilities.h"
#include "physics/state/StateHistory.h"
#include <memory>
#include <vector>

namespace iloss {
namespace physics {
namespace forces {
namespace twobody {

/**
 * @brief Lambert's problem solution
 * 
 * Contains the solution to Lambert's problem (orbit determination from
 * two position vectors and time of flight).
 */
struct LambertSolution {
    math::Vector3D v1;      ///< Velocity at first position (m/s)
    math::Vector3D v2;      ///< Velocity at second position (m/s)
    double a;               ///< Semi-major axis of transfer orbit (m)
    double e;               ///< Eccentricity of transfer orbit
    bool isRetrograde;      ///< True if transfer is retrograde
    int revolutions;        ///< Number of complete revolutions
    
    /**
     * @brief Check if solution is valid
     * @return True if velocities are finite
     */
    bool isValid() const;
};

/**
 * @brief Comprehensive two-body analytical propagator
 * 
 * This class provides a complete analytical propagation system for two-body
 * dynamics, combining the force model, Kepler propagator, and utility functions
 * into a unified interface.
 * 
 * Features:
 * - State propagation using Kepler's laws
 * - Orbit determination from observations
 * - Lambert's problem solver
 * - Maneuver planning
 * - Special orbit calculations
 * - Efficient batch propagation
 */
class TwoBodyAnalyticalPropagator {
public:
    /**
     * @brief Constructor with gravitational parameter
     * @param mu Gravitational parameter (m³/s²)
     * @param centralBody Name of central body
     */
    explicit TwoBodyAnalyticalPropagator(double mu = math::constants::EARTH_MU,
                                        const std::string& centralBody = "Earth");
    
    /**
     * @brief Destructor
     */
    ~TwoBodyAnalyticalPropagator() = default;
    
    // Basic propagation methods
    
    /**
     * @brief Propagate state to a specific time
     * 
     * @param initialState Initial state vector
     * @param targetTime Target time
     * @return State at target time
     */
    StateVector propagateToTime(const StateVector& initialState,
                               const time::Time& targetTime) const;
    
    /**
     * @brief Propagate state by a time interval
     * 
     * @param initialState Initial state vector
     * @param deltaTime Time interval (seconds)
     * @return State after time interval
     */
    StateVector propagateByDuration(const StateVector& initialState,
                                   double deltaTime) const;
    
    /**
     * @brief Propagate to a specific true anomaly
     * 
     * @param initialState Initial state vector
     * @param targetNu Target true anomaly (rad)
     * @param direction +1 for forward, -1 for backward
     * @return State at target true anomaly
     */
    StateVector propagateToTrueAnomaly(const StateVector& initialState,
                                      double targetNu,
                                      int direction = 1) const;
    
    /**
     * @brief Generate state history over time span
     * 
     * @param initialState Initial state vector
     * @param startTime Start time
     * @param endTime End time
     * @param timeStep Time step (seconds)
     * @return State history
     */
    std::unique_ptr<StateHistory> generateTrajectory(const StateVector& initialState,
                                                     const time::Time& startTime,
                                                     const time::Time& endTime,
                                                     double timeStep) const;
    
    // Orbit determination methods
    
    /**
     * @brief Determine orbit from two position vectors and time
     * 
     * Solves Lambert's problem to find the orbit connecting two positions.
     * 
     * @param r1 First position vector (m)
     * @param r2 Second position vector (m)
     * @param tof Time of flight (seconds)
     * @param isRetrograde True for retrograde transfer
     * @param revolutions Number of complete revolutions (0 for direct)
     * @return Lambert solution(s)
     */
    std::vector<LambertSolution> solveLambertProblem(const math::Vector3D& r1,
                                                     const math::Vector3D& r2,
                                                     double tof,
                                                     bool isRetrograde = false,
                                                     int revolutions = 0) const;
    
    /**
     * @brief Determine orbit from position and velocity
     * 
     * @param position Position vector (m)
     * @param velocity Velocity vector (m/s)
     * @param time Epoch time
     * @return Orbital elements
     */
    OrbitalElements determineOrbit(const math::Vector3D& position,
                                  const math::Vector3D& velocity,
                                  const time::Time& time) const;
    
    // Special orbit calculations
    
    /**
     * @brief Calculate Hohmann transfer between circular orbits
     * 
     * @param r1 Initial orbit radius (m)
     * @param r2 Final orbit radius (m)
     * @return Pair of (delta-v1, delta-v2) in m/s
     */
    std::pair<double, double> calculateHohmannTransfer(double r1, double r2) const;
    
    /**
     * @brief Calculate bi-elliptic transfer
     * 
     * @param r1 Initial orbit radius (m)
     * @param r2 Final orbit radius (m)
     * @param rt Transfer orbit apoapsis (m)
     * @return Vector of [delta-v1, delta-v2, delta-v3] in m/s
     */
    std::vector<double> calculateBiellipticTransfer(double r1, double r2, double rt) const;
    
    /**
     * @brief Calculate plane change maneuver
     * 
     * @param v Velocity magnitude (m/s)
     * @param deltaI Inclination change (rad)
     * @return Delta-v magnitude (m/s)
     */
    double calculatePlaneChange(double v, double deltaI) const;
    
    /**
     * @brief Calculate combined plane change with Hohmann transfer
     * 
     * @param r1 Initial orbit radius (m)
     * @param r2 Final orbit radius (m)
     * @param deltaI Inclination change (rad)
     * @return Total delta-v (m/s)
     */
    double calculateCombinedManeuver(double r1, double r2, double deltaI) const;
    
    // Orbit analysis methods
    
    /**
     * @brief Find time to reach a specific true anomaly
     * 
     * @param currentState Current state
     * @param targetNu Target true anomaly (rad)
     * @param direction +1 for forward, -1 for backward
     * @return Time to reach target (seconds)
     */
    double timeToTrueAnomaly(const StateVector& currentState,
                            double targetNu,
                            int direction = 1) const;
    
    /**
     * @brief Find next periapsis passage time
     * 
     * @param currentState Current state
     * @return Time to next periapsis
     */
    time::Time nextPeriapsisTime(const StateVector& currentState) const;
    
    /**
     * @brief Find next apoapsis passage time
     * 
     * @param currentState Current state
     * @return Time to next apoapsis
     */
    time::Time nextApoapsisTime(const StateVector& currentState) const;
    
    /**
     * @brief Calculate ground track
     * 
     * @param initialState Initial state
     * @param duration Duration (seconds)
     * @param timeStep Time step (seconds)
     * @return Vector of (latitude, longitude) pairs in radians
     */
    std::vector<std::pair<double, double>> calculateGroundTrack(
        const StateVector& initialState,
        double duration,
        double timeStep) const;
    
    /**
     * @brief Check if orbit passes over a ground location
     * 
     * @param elements Orbital elements
     * @param latitude Ground latitude (rad)
     * @param longitude Ground longitude (rad)
     * @param elevationAngle Minimum elevation angle (rad)
     * @return True if orbit can see the location
     */
    bool canSeeGroundLocation(const OrbitalElements& elements,
                             double latitude,
                             double longitude,
                             double elevationAngle = 0.0) const;
    
    // Getters
    double getGravitationalParameter() const { return m_mu; }
    const std::string& getCentralBody() const { return m_centralBody; }
    const TwoBodyForceModel& getForceModel() const { return *m_forceModel; }
    const KeplerPropagator& getKeplerPropagator() const { return *m_keplerPropagator; }
    
private:
    double m_mu;                                    ///< Gravitational parameter (m³/s²)
    std::string m_centralBody;                      ///< Name of central body
    std::unique_ptr<TwoBodyForceModel> m_forceModel;      ///< Force model
    std::unique_ptr<KeplerPropagator> m_keplerPropagator; ///< Kepler propagator
    
    /**
     * @brief Solve Lambert's problem using universal variables
     * 
     * @param r1 First position vector (m)
     * @param r2 Second position vector (m)
     * @param tof Time of flight (seconds)
     * @param isRetrograde True for retrograde transfer
     * @return Lambert solution
     */
    LambertSolution solveLambertUniversal(const math::Vector3D& r1,
                                          const math::Vector3D& r2,
                                          double tof,
                                          bool isRetrograde) const;
};

} // namespace twobody
} // namespace forces
} // namespace physics
} // namespace iloss