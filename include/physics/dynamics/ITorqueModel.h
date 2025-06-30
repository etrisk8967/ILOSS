#pragma once

#include "core/math/Vector3D.h"
#include "core/math/Quaternion.h"
#include <memory>
#include <string>
#include <any>

namespace iloss {
namespace physics {
namespace dynamics {

// Forward declarations
class DynamicsState;

/**
 * @brief Configuration for torque models
 * 
 * Uses std::any to allow flexible configuration similar to ForceModelConfig
 */
using TorqueModelConfig = std::any;

/**
 * @brief Types of torque models
 */
enum class TorqueModelType {
    GravityGradient,    ///< Gravity gradient torque
    Aerodynamic,        ///< Aerodynamic torque
    SolarRadiation,     ///< Solar radiation pressure torque
    Magnetic,           ///< Magnetic torque (Earth's field interaction)
    ReactionWheel,      ///< Reaction wheel torque
    Thruster,           ///< Thruster torque (off-center thrust)
    User                ///< User-defined torque model
};

/**
 * @brief Abstract interface for torque models
 * 
 * This interface defines the contract for all torque models in the dynamics system.
 * Torque models calculate the torque acting on a rigid body based on its current
 * state and environmental conditions.
 */
class ITorqueModel {
public:
    virtual ~ITorqueModel() = default;

    /**
     * @brief Calculate torque on the body
     * @param state Current dynamics state (position, velocity, attitude, angular velocity)
     * @param time Current simulation time
     * @return Torque vector in body frame (N⋅m)
     * 
     * The returned torque should be expressed in the body-fixed coordinate frame.
     * This is the torque about the center of mass of the body.
     */
    virtual math::Vector3D calculateTorque(const DynamicsState& state, double time) const = 0;

    /**
     * @brief Get the type of this torque model
     * @return Torque model type
     */
    virtual TorqueModelType getType() const = 0;

    /**
     * @brief Get a descriptive name for this model
     * @return Human-readable name
     */
    virtual std::string getName() const = 0;

    /**
     * @brief Check if this model is enabled
     * @return True if model should contribute to total torque
     */
    virtual bool isEnabled() const = 0;

    /**
     * @brief Enable or disable this model
     * @param enabled True to enable, false to disable
     */
    virtual void setEnabled(bool enabled) = 0;

    /**
     * @brief Configure the torque model
     * @param config Configuration parameters
     * 
     * The config parameter can contain any type-specific configuration.
     * Each torque model defines its own configuration structure.
     */
    virtual void configure(const TorqueModelConfig& config) = 0;

    /**
     * @brief Clone this torque model
     * @return Deep copy of this model
     */
    virtual std::unique_ptr<ITorqueModel> clone() const = 0;
};

} // namespace dynamics
} // namespace physics
} // namespace iloss