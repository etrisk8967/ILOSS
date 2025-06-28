#pragma once

#include "physics/forces/ForceModel.h"
#include "core/math/MathConstants.h"
#include <map>
#include <string>

namespace iloss {
namespace physics {
namespace forces {
namespace twobody {

/**
 * @brief Two-body gravitational force model
 * 
 * Implements Newton's law of universal gravitation for orbital mechanics.
 * This model treats the central body as a point mass and calculates the
 * gravitational acceleration acting on a satellite.
 * 
 * The force is given by: F = -μ * m * r / |r|³
 * where μ = GM is the gravitational parameter of the central body.
 * 
 * This model is accurate for:
 * - High altitude orbits where Earth's shape effects are negligible
 * - Preliminary mission analysis
 * - Analytical orbit propagation
 * - Kepler orbit calculations
 */
class TwoBodyForceModel : public ForceModel {
public:
    /**
     * @brief Constructor with default Earth parameters
     * @param name Name of the force model instance
     */
    explicit TwoBodyForceModel(const std::string& name = "TwoBody");

    /**
     * @brief Constructor with custom gravitational parameter
     * @param name Name of the force model instance
     * @param mu Gravitational parameter (m³/s²)
     */
    TwoBodyForceModel(const std::string& name, double mu);

    /**
     * @brief Calculate gravitational acceleration
     * 
     * Computes acceleration as: a = -μ * r / |r|³
     * 
     * @param state Current state vector (position must be in inertial frame)
     * @param time Current time (not used in two-body model)
     * @return Acceleration vector in m/s²
     * @throws std::runtime_error if position is at origin
     */
    math::Vector3D calculateAcceleration(
        const StateVector& state,
        const time::Time& time) const override;

    /**
     * @brief Initialize the model with configuration
     * 
     * Configuration parameters:
     * - "mu": Gravitational parameter (m³/s²), default: Earth's μ
     * - "central_body": Name of central body, default: "Earth"
     * 
     * @param config Configuration parameters
     * @return True if successful
     */
    bool initialize(const ForceModelConfig& config) override;

    /**
     * @brief Validate the model configuration
     * @return True if gravitational parameter is positive
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

    // Getters
    double getGravitationalParameter() const { return m_mu; }
    const std::string& getCentralBody() const { return m_centralBody; }
    
    // Debug method
    std::string debugState() const {
        return "TwoBodyForceModel[body='" + m_centralBody + "', mu=" + std::to_string(m_mu) + "]";
    }

    // Setters
    void setGravitationalParameter(double mu);
    void setCentralBody(const std::string& body);

    /**
     * @brief Get gravitational parameter for known celestial bodies
     * @param body Name of the body (Earth, Sun, Moon, Jupiter, Venus, Mars)
     * @return Gravitational parameter in m³/s²
     * @throws std::invalid_argument if body is not recognized
     */
    static double getBodyGravitationalParameter(const std::string& body);

private:
    double m_mu;                ///< Gravitational parameter (GM) in m³/s²
    std::string m_centralBody;  ///< Name of the central body

    /**
     * @brief Default gravitational parameters for common bodies
     */
    static const std::map<std::string, double> BODY_MU_VALUES;
};

} // namespace twobody
} // namespace forces
} // namespace physics
} // namespace iloss