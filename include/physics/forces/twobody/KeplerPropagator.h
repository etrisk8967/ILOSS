#pragma once

#include "physics/state/StateVector.h"
#include "core/time/Time.h"
#include "core/math/Vector3D.h"
#include "core/math/MathConstants.h"
#include <memory>

namespace iloss {
namespace physics {
namespace forces {
namespace twobody {

/**
 * @brief Classical orbital elements (Keplerian elements)
 * 
 * Describes an orbit using six parameters that remain constant
 * in the absence of perturbations.
 */
struct OrbitalElements {
    double a;       ///< Semi-major axis (m)
    double e;       ///< Eccentricity (dimensionless)
    double i;       ///< Inclination (rad)
    double omega;   ///< Argument of periapsis (rad)
    double Omega;   ///< Right ascension of ascending node (rad)
    double nu;      ///< True anomaly (rad)
    
    // Additional derived parameters
    double M;       ///< Mean anomaly (rad)
    double E;       ///< Eccentric anomaly (rad)
    double p;       ///< Semi-latus rectum (m)
    double n;       ///< Mean motion (rad/s)
    double T;       ///< Orbital period (s)
    
    /**
     * @brief Default constructor
     */
    OrbitalElements();
    
    /**
     * @brief Check if orbit is valid
     * @return True if elements represent a valid orbit
     */
    bool isValid() const;
    
    /**
     * @brief Get orbit type as string
     * @return "circular", "elliptical", "parabolic", or "hyperbolic"
     */
    std::string getOrbitType() const;
    
    /**
     * @brief Get apoapsis distance
     * @return Apoapsis distance in meters (infinity for parabolic/hyperbolic)
     */
    double getApoapsis() const;
    
    /**
     * @brief Get periapsis distance
     * @return Periapsis distance in meters
     */
    double getPeriapsis() const;
    
    /**
     * @brief Convert to string representation
     * @return String with orbital elements
     */
    std::string toString() const;
};

/**
 * @brief Kepler orbit propagator for two-body dynamics
 * 
 * Implements analytical orbit propagation using Kepler's laws.
 * This propagator solves Kepler's equation to determine the position
 * and velocity of an orbiting body at any given time.
 * 
 * Features:
 * - Handles all conic sections (circular, elliptical, parabolic, hyperbolic)
 * - Efficient analytical propagation (no numerical integration)
 * - Exact solution for two-body problem
 * - Supports forward and backward propagation
 */
class KeplerPropagator : public IStatePropagator {
public:
    /**
     * @brief Constructor with gravitational parameter
     * @param mu Gravitational parameter of central body (m³/s²)
     */
    explicit KeplerPropagator(double mu = math::constants::EARTH_MU);
    
    /**
     * @brief Destructor
     */
    virtual ~KeplerPropagator() = default;
    
    /**
     * @brief Propagate state forward or backward in time
     * 
     * Uses Kepler's laws to analytically propagate the orbit.
     * 
     * @param initialState Initial state vector
     * @param targetTime Target time for propagation
     * @return Propagated state at target time
     * @throws std::runtime_error if propagation fails
     */
    StateVector propagate(const StateVector& initialState,
                         const time::Time& targetTime) override;
    
    /**
     * @brief Propagate state by a time duration
     * 
     * @param initialState Initial state vector
     * @param deltaTime Time duration (seconds)
     * @return Propagated state
     */
    StateVector propagateByDuration(const StateVector& initialState,
                                   double deltaTime) override;
    
    /**
     * @brief Convert state vector to orbital elements
     * 
     * @param state State vector (position and velocity)
     * @param mu Gravitational parameter (m³/s²)
     * @return Classical orbital elements
     * @throws std::runtime_error if state is invalid
     */
    static OrbitalElements stateToElements(const StateVector& state, double mu);
    
    /**
     * @brief Convert orbital elements to state vector
     * 
     * @param elements Classical orbital elements
     * @param mu Gravitational parameter (m³/s²)
     * @param time Time for the state (stored in StateVector)
     * @return State vector (position and velocity)
     * @throws std::runtime_error if elements are invalid
     */
    static StateVector elementsToState(const OrbitalElements& elements, 
                                      double mu,
                                      const time::Time& time);
    
    /**
     * @brief Solve Kepler's equation for elliptical orbits
     * 
     * Solves M = E - e*sin(E) for E given M and e
     * 
     * @param M Mean anomaly (rad)
     * @param e Eccentricity
     * @param tolerance Convergence tolerance (default: 1e-12)
     * @param maxIterations Maximum iterations (default: 50)
     * @return Eccentric anomaly E (rad)
     * @throws std::runtime_error if convergence fails
     */
    static double solveKeplerEllipse(double M, double e, 
                                    double tolerance = 1e-12,
                                    int maxIterations = 50);
    
    /**
     * @brief Solve Kepler's equation for hyperbolic orbits
     * 
     * Solves M = e*sinh(H) - H for H given M and e
     * 
     * @param M Mean anomaly (rad)
     * @param e Eccentricity (>1)
     * @param tolerance Convergence tolerance (default: 1e-12)
     * @param maxIterations Maximum iterations (default: 50)
     * @return Hyperbolic anomaly H (rad)
     * @throws std::runtime_error if convergence fails
     */
    static double solveKeplerHyperbola(double M, double e,
                                      double tolerance = 1e-12,
                                      int maxIterations = 50);
    
    /**
     * @brief Solve Kepler's equation for parabolic orbits
     * 
     * @param M Mean anomaly (rad)
     * @return True anomaly (rad)
     */
    static double solveKeplerParabola(double M);
    
    /**
     * @brief Convert true anomaly to eccentric anomaly
     * 
     * @param nu True anomaly (rad)
     * @param e Eccentricity
     * @return Eccentric anomaly (rad) or hyperbolic anomaly for e>1
     */
    static double trueToEccentricAnomaly(double nu, double e);
    
    /**
     * @brief Convert eccentric anomaly to true anomaly
     * 
     * @param E Eccentric anomaly (rad) or hyperbolic anomaly for e>1
     * @param e Eccentricity
     * @return True anomaly (rad)
     */
    static double eccentricToTrueAnomaly(double E, double e);
    
    /**
     * @brief Convert mean anomaly to true anomaly
     * 
     * @param M Mean anomaly (rad)
     * @param e Eccentricity
     * @return True anomaly (rad)
     */
    static double meanToTrueAnomaly(double M, double e);
    
    /**
     * @brief Convert true anomaly to mean anomaly
     * 
     * @param nu True anomaly (rad)
     * @param e Eccentricity
     * @return Mean anomaly (rad)
     */
    static double trueToMeanAnomaly(double nu, double e);
    
    /**
     * @brief Calculate flight path angle
     * 
     * @param nu True anomaly (rad)
     * @param e Eccentricity
     * @return Flight path angle (rad)
     */
    static double calculateFlightPathAngle(double nu, double e);
    
    // Getters and setters
    double getGravitationalParameter() const { return m_mu; }
    void setGravitationalParameter(double mu);
    
private:
    double m_mu;  ///< Gravitational parameter (m³/s²)
    
    /**
     * @brief Propagate mean anomaly forward in time
     * 
     * @param M0 Initial mean anomaly (rad)
     * @param n Mean motion (rad/s)
     * @param dt Time difference (s)
     * @return Final mean anomaly (rad)
     */
    static double propagateMeanAnomaly(double M0, double n, double dt);
};

} // namespace twobody
} // namespace forces
} // namespace physics
} // namespace iloss