#pragma once

#include "physics/dynamics/ITorqueModel.h"
#include "physics/dynamics/DynamicsState.h"
#include "physics/aerodynamics/AerodynamicDatabase.h"
#include "physics/aerodynamics/AerodynamicCalculator.h"
#include "physics/aerodynamics/AerodynamicForceModel.h"
#include "core/constants/AtmosphericModel.h"
#include "core/math/Vector3D.h"
#include "core/math/Matrix3D.h"
#include <memory>
#include <string>

namespace iloss {
namespace physics {
namespace aerodynamics {

/**
 * @brief Configuration for aerodynamic torque calculations
 */
struct AerodynamicTorqueConfig {
    double referenceArea = 1.0;              ///< Reference area (m²)
    double referenceLength = 1.0;            ///< Reference length for pitch moments (m)
    double referenceSpan = 1.0;              ///< Reference span for roll/yaw moments (m)
    iloss::math::Vector3D centerOfPressure;  ///< Center of pressure location in body frame (m)
    iloss::math::Vector3D centerOfMass;      ///< Center of mass location in body frame (m)
    std::string coefficientConfig = "default"; ///< Coefficient database configuration
    double minimumDensity = 1e-12;           ///< Minimum density threshold (kg/m³)
    bool useDynamicCenterOfPressure = false; ///< Calculate COP from coefficients
};

/**
 * @brief Aerodynamic torque model implementation
 * 
 * This class calculates aerodynamic torques (moments) acting on a vehicle due to
 * atmospheric forces. The torques arise from the offset between the center of
 * pressure (where aerodynamic forces act) and the center of mass (about which
 * the vehicle rotates).
 * 
 * The model supports:
 * - Direct moment coefficients (Cl, Cm, Cn) from the aerodynamic database
 * - Torques due to force offset (when COP ≠ COM)
 * - Dynamic center of pressure calculation based on moment coefficients
 * - Integration with atmosphere models and coefficient databases
 * 
 * Torque calculation process:
 * 1. Calculate atmospheric properties and flow conditions
 * 2. Look up aerodynamic coefficients (including moment coefficients)
 * 3. Calculate moment from coefficients: M = q·S·L·C
 * 4. Calculate additional torque from force offset: T = r × F
 * 5. Sum all torques in body frame
 * 
 * @note All torques are returned in the body-fixed frame about the center of mass
 */
class AerodynamicTorqueModel : public dynamics::ITorqueModel {
public:
    /**
     * @brief Constructor
     * 
     * @param atmosphereModel Shared pointer to atmosphere model
     * @param coefficientDatabase Shared pointer to aerodynamic database
     * @param config Configuration parameters
     */
    AerodynamicTorqueModel(
        std::shared_ptr<constants::AtmosphericModel> atmosphereModel,
        std::shared_ptr<AerodynamicDatabase> coefficientDatabase,
        const AerodynamicTorqueConfig& config = AerodynamicTorqueConfig());

    /**
     * @brief Constructor that shares resources with a force model
     * 
     * This constructor is useful when you have both an AerodynamicForceModel
     * and AerodynamicTorqueModel, allowing them to share atmosphere and
     * coefficient database resources.
     * 
     * @param forceModel Existing aerodynamic force model to share resources with
     * @param config Configuration parameters
     */
    AerodynamicTorqueModel(
        const AerodynamicForceModel& forceModel,
        const AerodynamicTorqueConfig& config = AerodynamicTorqueConfig());

    /**
     * @brief Destructor
     */
    virtual ~AerodynamicTorqueModel() = default;

    /**
     * @brief Calculate torque on the body
     * 
     * @param state Current dynamics state
     * @param time Current simulation time
     * @return Torque vector in body frame (N·m)
     */
    iloss::math::Vector3D calculateTorque(
        const dynamics::DynamicsState& state, 
        double time) const override;

    /**
     * @brief Get the type of this torque model
     * 
     * @return TorqueModelType::Aerodynamic
     */
    dynamics::TorqueModelType getType() const override {
        return dynamics::TorqueModelType::Aerodynamic;
    }

    /**
     * @brief Get a descriptive name for this model
     * 
     * @return "AerodynamicTorque"
     */
    std::string getName() const override { return "AerodynamicTorque"; }

    /**
     * @brief Check if this model is enabled
     * 
     * @return true if enabled
     */
    bool isEnabled() const override { return m_enabled; }

    /**
     * @brief Enable or disable this model
     * 
     * @param enabled true to enable
     */
    void setEnabled(bool enabled) override { m_enabled = enabled; }

    /**
     * @brief Configure the torque model
     * 
     * The config should be an AerodynamicTorqueConfig struct wrapped in std::any
     * 
     * @param config Configuration parameters
     * @throws std::bad_any_cast if config is not AerodynamicTorqueConfig
     */
    void configure(const dynamics::TorqueModelConfig& config) override;

    /**
     * @brief Clone this torque model
     * 
     * @return Deep copy of this model
     */
    std::unique_ptr<dynamics::ITorqueModel> clone() const override;

    /**
     * @brief Get the current configuration
     * 
     * @return Current configuration
     */
    const AerodynamicTorqueConfig& getConfig() const { return m_config; }

    /**
     * @brief Set the configuration
     * 
     * @param config New configuration
     */
    void setConfig(const AerodynamicTorqueConfig& config);

    /**
     * @brief Calculate the center of pressure from moment coefficients
     * 
     * This method uses the relationship between moment and normal force
     * coefficients to estimate the center of pressure location.
     * 
     * @param cm Pitching moment coefficient
     * @param cn Normal force coefficient (usually CL for small angles)
     * @param referencePoint Reference point for moments (typically 0.25c)
     * @return Center of pressure location along body X-axis (m)
     */
    double calculateCenterOfPressure(
        double cm, 
        double cn,
        double referencePoint = 0.25) const;

protected:
    /**
     * @brief Calculate torque from moment coefficients
     * 
     * @param dynamicPressure Dynamic pressure (Pa)
     * @param coefficients Aerodynamic coefficients including moments
     * @return Torque vector in body frame from coefficients (N·m)
     */
    iloss::math::Vector3D calculateCoefficientTorque(
        double dynamicPressure,
        const AerodynamicCoefficients& coefficients) const;

    /**
     * @brief Calculate torque from force offset
     * 
     * @param force Aerodynamic force in body frame (N)
     * @param centerOfPressure Center of pressure in body frame (m)
     * @return Torque vector in body frame from offset (N·m)
     */
    iloss::math::Vector3D calculateOffsetTorque(
        const iloss::math::Vector3D& force,
        const iloss::math::Vector3D& centerOfPressure) const;

private:
    std::shared_ptr<constants::AtmosphericModel> m_atmosphereModel;
    std::shared_ptr<AerodynamicDatabase> m_coefficientDatabase;
    AerodynamicTorqueConfig m_config;
    bool m_enabled = true;
    
    // For sharing calculator with force model
    std::shared_ptr<AerodynamicForceModel> m_sharedForceModel;
    
    // Constants
    static constexpr double MIN_VELOCITY_FOR_AERO = 0.1;  // m/s
};

} // namespace aerodynamics
} // namespace physics
} // namespace iloss