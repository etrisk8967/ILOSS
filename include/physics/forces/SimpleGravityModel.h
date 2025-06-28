#pragma once

#include "physics/forces/ForceModel.h"

namespace iloss {
namespace physics {
namespace forces {

/**
 * @brief Simple two-body gravitational force model
 * 
 * This is a basic implementation of Newton's law of universal gravitation
 * for demonstration and testing purposes. The full implementation will
 * be provided in Task 18.
 */
class SimpleGravityModel : public ForceModel {
public:
    /**
     * @brief Constructor
     * @param name Name of the force model instance
     */
    explicit SimpleGravityModel(const std::string& name);

    /**
     * @brief Calculate gravitational acceleration
     * @param state Current state vector
     * @param time Current time (not used in this simple model)
     * @return Acceleration vector in m/s²
     */
    math::Vector3D calculateAcceleration(
        const StateVector& state,
        const time::Time& time) const override;

    /**
     * @brief Initialize the model with configuration
     * @param config Configuration parameters
     * @return True if successful
     */
    bool initialize(const ForceModelConfig& config) override;

    /**
     * @brief Validate the model configuration
     * @return True if valid
     */
    bool validate() const override;

    /**
     * @brief Clone the force model
     * @return Cloned model
     */
    std::unique_ptr<ForceModel> clone() const override;

    /**
     * @brief Get string representation
     * @return Description string
     */
    std::string toString() const override;

    /**
     * @brief Get the gravitational parameter
     * @return mu in m³/s²
     */
    double getGravitationalParameter() const { return m_mu; }

private:
    double m_mu;  ///< Gravitational parameter (GM) in m³/s²
    
    /**
     * @brief Default gravitational parameter for Earth
     */
    static constexpr double EARTH_MU = 3.986004418e14;
};

} // namespace forces
} // namespace physics
} // namespace iloss