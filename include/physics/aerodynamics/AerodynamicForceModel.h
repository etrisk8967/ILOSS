#pragma once

#include "physics/forces/IAttitudeAwareForceModel.h"
#include "physics/aerodynamics/AerodynamicDatabase.h"
#include "physics/aerodynamics/AerodynamicCalculator.h"
#include "core/constants/AtmosphericModel.h"
#include "physics/dynamics/DynamicsState.h"
#include "physics/state/StateVector.h"
#include "core/math/Vector3D.h"
#include "core/math/Quaternion.h"
#include "core/math/Matrix3D.h"
#include "core/time/Time.h"
#include <memory>
#include <string>
#include <optional>

namespace iloss {
namespace physics {
namespace aerodynamics {

/**
 * @brief Configuration structure for aerodynamic force calculations
 */
struct AerodynamicConfig {
    double referenceArea = 1.0;           ///< Reference area (m²)
    double referenceLength = 1.0;         ///< Reference length for moments (m)
    double referenceSpan = 1.0;           ///< Reference span for lateral moments (m)
    iloss::math::Vector3D centerOfPressureOffset;  ///< Center of pressure offset from COM (m)
    std::string coefficientConfig = "default";      ///< Coefficient database configuration
    bool enableDynamicPressureLimit = false;       ///< Limit dynamic pressure for stability
    double maxDynamicPressure = 100000.0;          ///< Maximum dynamic pressure (Pa)
    double minimumDensity = 1e-12;                 ///< Minimum density threshold (kg/m³)
};

/**
 * @brief Aerodynamic force model implementation
 * 
 * This class implements aerodynamic forces for atmospheric flight, including:
 * - Lift, drag, and side forces based on aerodynamic coefficients
 * - Dependency on Mach number and angle of attack
 * - Integration with atmosphere models for density and temperature
 * - Proper frame transformations from wind to body to inertial frames
 * 
 * The model requires:
 * - An aerodynamic coefficient database (loaded from CSV)
 * - An atmosphere model for density and temperature
 * - Vehicle attitude for proper force direction
 * 
 * Force calculation process:
 * 1. Calculate atmospheric properties at current altitude
 * 2. Transform velocity to body frame and compute flow angles
 * 3. Look up aerodynamic coefficients based on Mach and AoA
 * 4. Calculate forces in wind frame
 * 5. Transform forces to inertial frame
 * 
 * @note All forces are converted to accelerations (F/m) for the integrator
 */
class AerodynamicForceModel : public forces::IAttitudeAwareForceModel {
public:
    /**
     * @brief Constructor
     * 
     * @param atmosphereModel Shared pointer to atmosphere model
     * @param coefficientDatabase Shared pointer to aerodynamic database
     * @param config Configuration parameters
     */
    AerodynamicForceModel(
        std::shared_ptr<constants::AtmosphericModel> atmosphereModel,
        std::shared_ptr<AerodynamicDatabase> coefficientDatabase,
        const AerodynamicConfig& config = AerodynamicConfig());

    /**
     * @brief Destructor
     */
    virtual ~AerodynamicForceModel() = default;

    /**
     * @brief Calculate acceleration with full dynamics state
     * 
     * @param state Full dynamics state including attitude
     * @param time Current time
     * @return Acceleration in inertial frame (m/s²)
     */
    iloss::math::Vector3D calculateAccelerationWithAttitude(
        const dynamics::DynamicsState& state,
        const time::Time& time) const override;

    /**
     * @brief Initialize from configuration
     * 
     * Expected parameters:
     * - reference_area: Reference area in m²
     * - reference_length: Reference length in m
     * - reference_span: Reference span in m
     * - cop_offset_x/y/z: Center of pressure offset components
     * - coefficient_config: Name of coefficient configuration to use
     * - enable_q_limit: Enable dynamic pressure limiting
     * - max_q: Maximum dynamic pressure in Pa
     * 
     * @param config Force model configuration
     * @return true if initialization successful
     */
    bool initialize(const forces::ForceModelConfig& config) override;

    /**
     * @brief Validate the force model configuration
     * 
     * @return true if configuration is valid
     */
    bool validate() const override;

    /**
     * @brief Update time-dependent parameters
     * 
     * @param time Current simulation time
     */
    void update(const time::Time& time) override;

    /**
     * @brief Clone the force model
     * 
     * @return Unique pointer to cloned force model
     */
    std::unique_ptr<forces::ForceModel> clone() const override;

    /**
     * @brief Get reference area
     * 
     * @return Reference area in m²
     */
    double getReferenceArea() const override { return m_config.referenceArea; }

    /**
     * @brief Set reference area
     * 
     * @param area Reference area in m²
     */
    void setReferenceArea(double area) override;

    /**
     * @brief Get the current aerodynamic configuration
     * 
     * @return Current configuration
     */
    const AerodynamicConfig& getConfig() const { return m_config; }

    /**
     * @brief Set the aerodynamic configuration
     * 
     * @param config New configuration
     */
    void setConfig(const AerodynamicConfig& config);

    /**
     * @brief Get the last calculated flow properties (for debugging/analysis)
     * 
     * @return Optional flow properties from last calculation
     */
    std::optional<FlowProperties> getLastFlowProperties() const { return m_lastFlowProperties; }

    /**
     * @brief Get the last calculated coefficients (for debugging/analysis)
     * 
     * @return Optional coefficients from last calculation
     */
    std::optional<AerodynamicCoefficients> getLastCoefficients() const { return m_lastCoefficients; }

    /**
     * @brief Calculate aerodynamic forces in body frame
     * 
     * This method is exposed for testing and analysis purposes.
     * 
     * @param flowProps Flow properties
     * @param coefficients Aerodynamic coefficients
     * @return Force vector in body frame (N)
     */
    iloss::math::Vector3D calculateBodyForces(
        const FlowProperties& flowProps,
        const AerodynamicCoefficients& coefficients) const;

protected:
    /**
     * @brief Calculate forces in wind frame
     * 
     * @param dynamicPressure Dynamic pressure (Pa)
     * @param coefficients Aerodynamic coefficients
     * @return Force vector in wind frame (N)
     */
    iloss::math::Vector3D calculateWindForces(
        double dynamicPressure,
        const AerodynamicCoefficients& coefficients) const;

    /**
     * @brief Apply dynamic pressure limiting if enabled
     * 
     * @param q Dynamic pressure (Pa)
     * @return Limited dynamic pressure (Pa)
     */
    double limitDynamicPressure(double q) const;

private:
    std::shared_ptr<constants::AtmosphericModel> m_atmosphereModel;
    std::shared_ptr<AerodynamicDatabase> m_coefficientDatabase;
    AerodynamicConfig m_config;
    
    // Cached values for analysis
    mutable std::optional<FlowProperties> m_lastFlowProperties;
    mutable std::optional<AerodynamicCoefficients> m_lastCoefficients;
    
    // Constants
    static constexpr double MIN_VELOCITY_FOR_AERO = 0.1;  // m/s
};

} // namespace aerodynamics
} // namespace physics
} // namespace iloss