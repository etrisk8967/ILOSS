#pragma once

#include <cstddef>  // For size_t

/**
 * @file PropulsionConstants.h
 * @brief Physical constants and standard values for propulsion calculations
 * 
 * This file contains fundamental constants used throughout the propulsion
 * system implementation, including standard gravity, atmospheric pressure,
 * and common conversion factors.
 */

namespace iloss {
namespace physics {
namespace propulsion {

/**
 * @brief Standard Earth gravity at sea level
 * 
 * This is the standard acceleration due to gravity used for specific impulse
 * calculations and weight conversions. Value from CODATA 2018.
 */
constexpr double STANDARD_GRAVITY = 9.80665;  // m/s²

/**
 * @brief Standard sea level atmospheric pressure
 * 
 * Used for calculating sea level thrust from vacuum thrust and nozzle exit area.
 * Value from US Standard Atmosphere 1976.
 */
constexpr double STANDARD_ATMOSPHERE = 101325.0;  // Pa (N/m²)

/**
 * @brief Vacuum of space pressure
 * 
 * Theoretical perfect vacuum pressure used for vacuum thrust calculations.
 */
constexpr double VACUUM_PRESSURE = 0.0;  // Pa

/**
 * @brief Conversion factor from pounds to Newtons
 */
constexpr double LBF_TO_NEWTON = 4.44822;  // N/lbf

/**
 * @brief Conversion factor from pounds-mass to kilograms
 */
constexpr double LBM_TO_KG = 0.453592;  // kg/lbm

/**
 * @brief Default minimum throttle setting
 * 
 * Used when no minimum throttle is specified for an engine.
 * Many engines cannot throttle below a certain threshold.
 */
constexpr double DEFAULT_MIN_THROTTLE = 0.0;  // 0%

/**
 * @brief Default maximum throttle setting
 * 
 * Most engines are designed to operate at 100% throttle maximum.
 */
constexpr double DEFAULT_MAX_THROTTLE = 1.0;  // 100%

/**
 * @brief Default gimbal range in radians
 * 
 * Typical gimbal range for modern rocket engines (±5 degrees).
 */
constexpr double DEFAULT_GIMBAL_RANGE = 0.0872665;  // radians (5 degrees)

/**
 * @brief Default gimbal rate limit
 * 
 * Maximum angular rate for thrust vector control gimbaling.
 */
constexpr double DEFAULT_GIMBAL_RATE = 0.174533;  // rad/s (10 deg/s)

/**
 * @brief Minimum propellant mass fraction
 * 
 * When propellant mass falls below this fraction of initial mass,
 * the tank is considered empty to avoid numerical issues.
 */
constexpr double MIN_PROPELLANT_FRACTION = 1e-6;  // 0.0001%

/**
 * @brief Default thrust vector in body frame
 * 
 * Most engines thrust along the negative Z axis in body frame
 * (opposite to the vehicle's "forward" direction).
 */
struct DefaultThrustVector {
    static constexpr double X = 0.0;
    static constexpr double Y = 0.0;
    static constexpr double Z = -1.0;
};

/**
 * @brief Common propellant types
 */
enum class PropellantType {
    RP1_LOX,           ///< Refined petroleum with liquid oxygen (Falcon 9, Saturn V first stage)
    LH2_LOX,           ///< Liquid hydrogen with liquid oxygen (Space Shuttle, SLS)
    UDMH_N2O4,         ///< Unsymmetrical dimethylhydrazine with nitrogen tetroxide (Proton)
    MMH_N2O4,          ///< Monomethylhydrazine with nitrogen tetroxide (Space Shuttle OMS)
    SOLID,             ///< Solid propellant (SRBs)
    XENON,             ///< Xenon for ion/Hall thrusters
    HYDRAZINE,         ///< Monopropellant hydrazine
    CUSTOM             ///< User-defined propellant
};

/**
 * @brief Get string representation of propellant type
 */
inline const char* propellantTypeToString(PropellantType type) {
    switch (type) {
        case PropellantType::RP1_LOX:    return "RP-1/LOX";
        case PropellantType::LH2_LOX:    return "LH2/LOX";
        case PropellantType::UDMH_N2O4:  return "UDMH/N2O4";
        case PropellantType::MMH_N2O4:   return "MMH/N2O4";
        case PropellantType::SOLID:      return "Solid";
        case PropellantType::XENON:      return "Xenon";
        case PropellantType::HYDRAZINE:  return "Hydrazine";
        case PropellantType::CUSTOM:     return "Custom";
        default:                         return "Unknown";
    }
}

/**
 * @brief Engine cycle types
 */
enum class EngineCycle {
    PRESSURE_FED,      ///< Simple pressure-fed system
    GAS_GENERATOR,     ///< Gas generator cycle (Merlin, F-1)
    STAGED_COMBUSTION, ///< Staged combustion cycle (RD-180, RS-25)
    EXPANDER,          ///< Expander cycle (RL-10)
    ELECTRIC,          ///< Electric propulsion (ion, Hall effect)
    SOLID,             ///< Solid rocket motor
    MONOPROPELLANT     ///< Monopropellant decomposition
};

/**
 * @brief Thrust curve interpolation methods
 */
enum class ThrustCurveInterpolation {
    LINEAR,            ///< Linear interpolation between points
    CUBIC_SPLINE,      ///< Smooth cubic spline interpolation
    STEP              ///< Step function (nearest neighbor)
};

/**
 * @brief Nozzle expansion ratio limits
 */
struct NozzleExpansionLimits {
    static constexpr double MIN_RATIO = 1.0;    ///< Minimum area ratio (throat)
    static constexpr double MAX_RATIO = 300.0;  ///< Maximum practical area ratio
};

/**
 * @brief Specific impulse ranges for different propulsion types (seconds)
 */
struct SpecificImpulseRanges {
    // Chemical propulsion
    static constexpr double SOLID_MIN = 200.0;
    static constexpr double SOLID_MAX = 300.0;
    static constexpr double LIQUID_MIN = 250.0;
    static constexpr double LIQUID_MAX = 465.0;  // LH2/LOX vacuum
    
    // Electric propulsion
    static constexpr double ION_MIN = 2000.0;
    static constexpr double ION_MAX = 10000.0;
    static constexpr double HALL_MIN = 1000.0;
    static constexpr double HALL_MAX = 3000.0;
    
    // Monopropellant
    static constexpr double MONOPROP_MIN = 150.0;
    static constexpr double MONOPROP_MAX = 250.0;
};

/**
 * @brief Maximum number of engines per stage (for array sizing)
 */
constexpr size_t MAX_ENGINES_PER_STAGE = 33;  // SpaceX Super Heavy has 33 engines

/**
 * @brief Maximum number of propellant tanks
 */
constexpr size_t MAX_PROPELLANT_TANKS = 10;

} // namespace propulsion
} // namespace physics
} // namespace iloss