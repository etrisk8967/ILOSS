#pragma once

#include "core/math/Vector3D.h"
#include "core/math/Matrix3D.h"
#include "core/math/Quaternion.h"
#include <optional>

namespace iloss {
namespace physics {
namespace aerodynamics {

/**
 * @brief Structure containing aerodynamic flow properties
 */
struct FlowProperties {
    double mach;                ///< Mach number
    double dynamicPressure;     ///< Dynamic pressure (Pa)
    double angleOfAttack;       ///< Angle of attack (radians)
    double sideslipAngle;       ///< Sideslip angle (radians)
    iloss::math::Vector3D velocityBodyFrame;    ///< Velocity in body frame (m/s)
    iloss::math::Vector3D velocityWindFrame;    ///< Velocity in wind frame (m/s)
    double totalAngleOfAttack;  ///< Total angle of attack for high-alpha cases (radians)
    
    /**
     * @brief Check if flow properties are valid
     */
    bool isValid() const {
        return std::isfinite(mach) && mach >= 0.0 &&
               std::isfinite(dynamicPressure) && dynamicPressure >= 0.0 &&
               std::isfinite(angleOfAttack) &&
               std::isfinite(sideslipAngle);
    }
};

/**
 * @brief Calculator for aerodynamic parameters and reference frame transformations
 * 
 * This class provides methods to calculate essential aerodynamic parameters such as
 * Mach number, angle of attack, sideslip angle, and dynamic pressure. It also handles
 * transformations between different reference frames used in aerodynamics.
 * 
 * Reference Frames:
 * - **Body Frame**: Fixed to vehicle (X-forward, Y-right, Z-down)
 * - **Wind Frame**: Aligned with velocity vector (X-along velocity)
 * - **Stability Frame**: Intermediate frame for aerodynamic analysis
 * 
 * Sign Conventions:
 * - Angle of Attack (α): Positive nose up
 * - Sideslip Angle (β): Positive nose right
 * - All angles in radians internally
 */
class AerodynamicCalculator {
public:
    /**
     * @brief Calculate Mach number
     * 
     * @param velocity Velocity magnitude (m/s)
     * @param speedOfSound Speed of sound (m/s)
     * @return Mach number (dimensionless)
     * @throws std::invalid_argument if speedOfSound <= 0
     */
    static double calculateMachNumber(double velocity, double speedOfSound);

    /**
     * @brief Calculate dynamic pressure
     * 
     * q = 0.5 * ρ * V²
     * 
     * @param density Air density (kg/m³)
     * @param velocity Velocity magnitude (m/s)
     * @return Dynamic pressure (Pa)
     * @throws std::invalid_argument if density < 0
     */
    static double calculateDynamicPressure(double density, double velocity);

    /**
     * @brief Calculate angle of attack and sideslip from velocity in body frame
     * 
     * Computes aerodynamic angles from the velocity vector expressed in body coordinates.
     * Returns nullopt if velocity magnitude is too small to determine angles reliably.
     * 
     * @param velocityBody Velocity vector in body frame (m/s)
     * @param minVelocity Minimum velocity for angle computation (default 0.1 m/s)
     * @return Pair of (angle of attack, sideslip angle) in radians, or nullopt
     */
    static std::optional<std::pair<double, double>> calculateAeroAngles(
        const iloss::math::Vector3D& velocityBody,
        double minVelocity = 0.1);

    /**
     * @brief Calculate flow properties from state and atmospheric conditions
     * 
     * This is a convenience method that computes all relevant aerodynamic parameters
     * at once, which is more efficient than calling individual methods.
     * 
     * @param velocityInertial Velocity in inertial frame (m/s)
     * @param attitude Vehicle attitude quaternion (inertial to body)
     * @param density Air density (kg/m³)
     * @param speedOfSound Speed of sound (m/s)
     * @param windVelocity Wind velocity in inertial frame (m/s, default zero)
     * @return Complete flow properties structure
     */
    static FlowProperties calculateFlowProperties(
        const iloss::math::Vector3D& velocityInertial,
        const iloss::math::Quaternion& attitude,
        double density,
        double speedOfSound,
        const iloss::math::Vector3D& windVelocity = iloss::math::Vector3D());

    /**
     * @brief Transform from body frame to wind frame
     * 
     * The wind frame has X-axis aligned with the relative velocity vector.
     * 
     * @param vectorBody Vector in body frame
     * @param angleOfAttack Angle of attack (radians)
     * @param sideslipAngle Sideslip angle (radians)
     * @return Vector in wind frame
     */
    static iloss::math::Vector3D bodyToWind(const iloss::math::Vector3D& vectorBody,
                                    double angleOfAttack,
                                    double sideslipAngle);

    /**
     * @brief Transform from wind frame to body frame
     * 
     * @param vectorWind Vector in wind frame
     * @param angleOfAttack Angle of attack (radians)
     * @param sideslipAngle Sideslip angle (radians)
     * @return Vector in body frame
     */
    static iloss::math::Vector3D windToBody(const iloss::math::Vector3D& vectorWind,
                                    double angleOfAttack,
                                    double sideslipAngle);

    /**
     * @brief Get transformation matrix from body to wind frame
     * 
     * @param angleOfAttack Angle of attack (radians)
     * @param sideslipAngle Sideslip angle (radians)
     * @return 3x3 transformation matrix
     */
    static iloss::math::Matrix3D getBodyToWindMatrix(double angleOfAttack,
                                             double sideslipAngle);

    /**
     * @brief Get transformation matrix from wind to body frame
     * 
     * @param angleOfAttack Angle of attack (radians)
     * @param sideslipAngle Sideslip angle (radians)
     * @return 3x3 transformation matrix
     */
    static iloss::math::Matrix3D getWindToBodyMatrix(double angleOfAttack,
                                             double sideslipAngle);

    /**
     * @brief Transform from body frame to stability frame
     * 
     * The stability frame is rotated by angle of attack only (no sideslip).
     * Useful for longitudinal stability analysis.
     * 
     * @param vectorBody Vector in body frame
     * @param angleOfAttack Angle of attack (radians)
     * @return Vector in stability frame
     */
    static iloss::math::Vector3D bodyToStability(const iloss::math::Vector3D& vectorBody,
                                         double angleOfAttack);

    /**
     * @brief Calculate total angle of attack for high-alpha aerodynamics
     * 
     * For large angles where traditional α and β breakdown, use total angle.
     * α_total = arccos(V_x / |V|) where V_x is forward velocity component.
     * 
     * @param velocityBody Velocity in body frame
     * @return Total angle of attack (radians)
     */
    static double calculateTotalAngleOfAttack(const iloss::math::Vector3D& velocityBody);

    /**
     * @brief Calculate Reynolds number
     * 
     * Re = ρ * V * L / μ
     * 
     * @param density Air density (kg/m³)
     * @param velocity Velocity magnitude (m/s)
     * @param characteristicLength Reference length (m)
     * @param dynamicViscosity Dynamic viscosity (Pa·s)
     * @return Reynolds number (dimensionless)
     */
    static double calculateReynoldsNumber(double density,
                                        double velocity,
                                        double characteristicLength,
                                        double dynamicViscosity);

    /**
     * @brief Calculate center of pressure location from moment coefficients
     * 
     * Estimates the center of pressure location along the body X-axis
     * based on the ratio of pitching moment to normal force.
     * 
     * @param cm Pitching moment coefficient
     * @param cn Normal force coefficient (typically CL for small angles)
     * @param referenceLength Reference length (m)
     * @param momentReferencePoint Moment reference point from nose (m)
     * @return Center of pressure location from nose (m)
     */
    static double calculateCenterOfPressure(double cm,
                                          double cn,
                                          double referenceLength,
                                          double momentReferencePoint);

    /**
     * @brief Check if flow is in transonic regime
     * 
     * @param mach Mach number
     * @param lowerBound Lower Mach boundary (default 0.8)
     * @param upperBound Upper Mach boundary (default 1.2)
     * @return true if Mach number is in transonic range
     */
    static bool isTransonic(double mach, 
                           double lowerBound = 0.8,
                           double upperBound = 1.2);

    /**
     * @brief Calculate speed of sound from temperature
     * 
     * For ideal gas: a = sqrt(γ * R * T)
     * 
     * @param temperature Temperature (K)
     * @param gamma Specific heat ratio (default 1.4 for air)
     * @param gasConstant Specific gas constant (default 287.053 J/kg/K for air)
     * @return Speed of sound (m/s)
     */
    static double calculateSpeedOfSound(double temperature,
                                      double gamma = 1.4,
                                      double gasConstant = 287.053);

private:
    // Constants
    static constexpr double MIN_VELOCITY_FOR_ANGLES = 0.1;  // m/s
    static constexpr double EPSILON = 1e-10;
};

} // namespace aerodynamics
} // namespace physics
} // namespace iloss