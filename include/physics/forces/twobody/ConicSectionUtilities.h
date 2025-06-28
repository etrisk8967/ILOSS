#pragma once

#include "core/math/Vector3D.h"
#include "physics/forces/twobody/KeplerPropagator.h"
#include <vector>
#include <utility>

namespace iloss {
namespace physics {
namespace forces {
namespace twobody {

/**
 * @brief Utilities for conic section calculations
 * 
 * Provides functions for working with conic sections in orbital mechanics,
 * including orbit classification, parameter calculations, and special points.
 */
class ConicSectionUtilities {
public:
    /**
     * @brief Orbit classification based on eccentricity
     */
    enum class OrbitType {
        CIRCULAR,    ///< e ≈ 0 (within tolerance)
        ELLIPTICAL,  ///< 0 < e < 1
        PARABOLIC,   ///< e ≈ 1 (within tolerance)
        HYPERBOLIC   ///< e > 1
    };
    
    /**
     * @brief Classify orbit based on eccentricity
     * 
     * @param e Eccentricity
     * @param tolerance Tolerance for circular/parabolic classification (default: 1e-8)
     * @return Orbit type
     */
    static OrbitType classifyOrbit(double e, double tolerance = 1e-8);
    
    /**
     * @brief Calculate specific energy from state
     * 
     * @param position Position vector (m)
     * @param velocity Velocity vector (m/s)
     * @param mu Gravitational parameter (m³/s²)
     * @return Specific energy (m²/s²)
     */
    static double calculateSpecificEnergy(const math::Vector3D& position,
                                         const math::Vector3D& velocity,
                                         double mu);
    
    /**
     * @brief Calculate specific angular momentum
     * 
     * @param position Position vector (m)
     * @param velocity Velocity vector (m/s)
     * @return Specific angular momentum vector (m²/s)
     */
    static math::Vector3D calculateSpecificAngularMomentum(const math::Vector3D& position,
                                                          const math::Vector3D& velocity);
    
    /**
     * @brief Calculate eccentricity vector
     * 
     * @param position Position vector (m)
     * @param velocity Velocity vector (m/s)
     * @param mu Gravitational parameter (m³/s²)
     * @return Eccentricity vector (dimensionless)
     */
    static math::Vector3D calculateEccentricityVector(const math::Vector3D& position,
                                                     const math::Vector3D& velocity,
                                                     double mu);
    
    /**
     * @brief Calculate semi-major axis from energy
     * 
     * @param energy Specific energy (m²/s²)
     * @param mu Gravitational parameter (m³/s²)
     * @return Semi-major axis (m), infinity for parabolic
     */
    static double calculateSemiMajorAxis(double energy, double mu);
    
    /**
     * @brief Calculate semi-latus rectum
     * 
     * @param h_mag Magnitude of specific angular momentum (m²/s)
     * @param mu Gravitational parameter (m³/s²)
     * @return Semi-latus rectum (m)
     */
    static double calculateSemiLatusRectum(double h_mag, double mu);
    
    /**
     * @brief Calculate radius at true anomaly
     * 
     * @param p Semi-latus rectum (m)
     * @param e Eccentricity
     * @param nu True anomaly (rad)
     * @return Radius (m)
     */
    static double calculateRadius(double p, double e, double nu);
    
    /**
     * @brief Calculate velocity at true anomaly
     * 
     * @param mu Gravitational parameter (m³/s²)
     * @param p Semi-latus rectum (m)
     * @param e Eccentricity
     * @param nu True anomaly (rad)
     * @return Velocity magnitude (m/s)
     */
    static double calculateVelocity(double mu, double p, double e, double nu);
    
    /**
     * @brief Calculate flight path angle at true anomaly
     * 
     * @param e Eccentricity
     * @param nu True anomaly (rad)
     * @return Flight path angle (rad)
     */
    static double calculateFlightPathAngle(double e, double nu);
    
    /**
     * @brief Calculate true anomaly at radius
     * 
     * @param r Radius (m)
     * @param p Semi-latus rectum (m)
     * @param e Eccentricity
     * @param ascending True for ascending branch, false for descending
     * @return True anomaly (rad)
     */
    static double calculateTrueAnomalyAtRadius(double r, double p, double e, bool ascending = true);
    
    /**
     * @brief Calculate time of flight between two true anomalies
     * 
     * @param nu1 Initial true anomaly (rad)
     * @param nu2 Final true anomaly (rad)
     * @param a Semi-major axis (m)
     * @param e Eccentricity
     * @param mu Gravitational parameter (m³/s²)
     * @return Time of flight (s)
     */
    static double calculateTimeOfFlight(double nu1, double nu2, double a, double e, double mu);
    
    /**
     * @brief Calculate mean motion
     * 
     * @param a Semi-major axis (m)
     * @param mu Gravitational parameter (m³/s²)
     * @return Mean motion (rad/s)
     */
    static double calculateMeanMotion(double a, double mu);
    
    /**
     * @brief Calculate orbital period
     * 
     * @param a Semi-major axis (m)
     * @param mu Gravitational parameter (m³/s²)
     * @return Period (s), infinity for non-elliptical
     */
    static double calculatePeriod(double a, double mu);
    
    /**
     * @brief Calculate vis-viva velocity
     * 
     * Uses the vis-viva equation: v² = μ(2/r - 1/a)
     * 
     * @param r Radius (m)
     * @param a Semi-major axis (m)
     * @param mu Gravitational parameter (m³/s²)
     * @return Velocity magnitude (m/s)
     */
    static double calculateVisVivaVelocity(double r, double a, double mu);
    
    /**
     * @brief Calculate escape velocity at radius
     * 
     * @param r Radius (m)
     * @param mu Gravitational parameter (m³/s²)
     * @return Escape velocity (m/s)
     */
    static double calculateEscapeVelocity(double r, double mu);
    
    /**
     * @brief Calculate circular velocity at radius
     * 
     * @param r Radius (m)
     * @param mu Gravitational parameter (m³/s²)
     * @return Circular velocity (m/s)
     */
    static double calculateCircularVelocity(double r, double mu);
    
    /**
     * @brief Check if orbit is bound (elliptical)
     * 
     * @param energy Specific energy (m²/s²)
     * @return True if orbit is bound
     */
    static bool isBoundOrbit(double energy);
    
    /**
     * @brief Check if orbit is closed (periodic)
     * 
     * @param e Eccentricity
     * @return True if orbit is closed
     */
    static bool isClosedOrbit(double e);
    
    /**
     * @brief Calculate hyperbolic excess velocity
     * 
     * @param a Semi-major axis (negative for hyperbola)
     * @param mu Gravitational parameter (m³/s²)
     * @return V-infinity (m/s)
     */
    static double calculateHyperbolicExcessVelocity(double a, double mu);
    
    /**
     * @brief Calculate turning angle for hyperbolic orbit
     * 
     * @param e Eccentricity (>1)
     * @return Total turning angle (rad)
     */
    static double calculateHyperbolicTurningAngle(double e);
    
    /**
     * @brief Calculate impact parameter for hyperbolic orbit
     * 
     * @param a Semi-major axis (negative)
     * @param e Eccentricity (>1)
     * @return Impact parameter (m)
     */
    static double calculateImpactParameter(double a, double e);
    
    /**
     * @brief Calculate sphere of influence radius
     * 
     * @param a_orbit Semi-major axis of body's orbit around parent (m)
     * @param m_body Mass of body (kg)
     * @param m_parent Mass of parent body (kg)
     * @return Sphere of influence radius (m)
     */
    static double calculateSphereOfInfluence(double a_orbit, double m_body, double m_parent);
    
    /**
     * @brief Calculate Hill sphere radius
     * 
     * @param a_orbit Semi-major axis of body's orbit around parent (m)
     * @param e_orbit Eccentricity of body's orbit
     * @param m_body Mass of body (kg)
     * @param m_parent Mass of parent body (kg)
     * @return Hill sphere radius (m)
     */
    static double calculateHillSphere(double a_orbit, double e_orbit, 
                                     double m_body, double m_parent);
    
    /**
     * @brief Sample points along an orbit
     * 
     * @param elements Orbital elements
     * @param mu Gravitational parameter (m³/s²)
     * @param numPoints Number of points to sample
     * @param startNu Starting true anomaly (rad)
     * @param endNu Ending true anomaly (rad)
     * @return Vector of position vectors along orbit
     */
    static std::vector<math::Vector3D> sampleOrbitPoints(const OrbitalElements& elements,
                                                         double mu,
                                                         int numPoints,
                                                         double startNu = 0.0,
                                                         double endNu = math::constants::TWO_PI);
    
    /**
     * @brief Calculate apsides (periapsis and apoapsis)
     * 
     * @param a Semi-major axis (m)
     * @param e Eccentricity
     * @return Pair of (periapsis, apoapsis) in meters
     */
    static std::pair<double, double> calculateApsides(double a, double e);
    
    /**
     * @brief Calculate velocities at apsides
     * 
     * @param mu Gravitational parameter (m³/s²)
     * @param a Semi-major axis (m)
     * @param e Eccentricity
     * @return Pair of (periapsis velocity, apoapsis velocity) in m/s
     */
    static std::pair<double, double> calculateApsidalVelocities(double mu, double a, double e);
};

} // namespace twobody
} // namespace forces
} // namespace physics
} // namespace iloss