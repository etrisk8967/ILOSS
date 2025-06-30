#pragma once

#include "physics/dynamics/ITorqueModel.h"
#include "physics/dynamics/IMassProperties.h"
#include "physics/dynamics/DynamicsState.h"
#include "core/math/MathConstants.h"
#include <memory>

namespace iloss {
namespace physics {
namespace dynamics {
namespace torques {

/**
 * @brief Configuration for gravity gradient torque model
 */
struct GravityGradientConfig {
    double centralBodyMu = math::constants::EARTH_MU;  ///< Gravitational parameter (m³/s²)
    std::shared_ptr<IMassProperties> massProperties;  ///< Mass properties of the body
};

/**
 * @brief Gravity gradient torque model
 * 
 * This model calculates the torque due to the gravity gradient effect,
 * which occurs because different parts of an extended body experience
 * slightly different gravitational forces. This creates a torque that
 * tends to align the body's minimum moment of inertia axis with the
 * local vertical.
 * 
 * The gravity gradient torque is given by:
 * τ = (3μ/r³) * (r̂ × I·r̂)
 * 
 * where:
 * - μ is the gravitational parameter
 * - r is the position vector magnitude
 * - r̂ is the unit position vector in body frame
 * - I is the inertia tensor
 */
class GravityGradientTorque : public ITorqueModel {
public:
    /**
     * @brief Default constructor
     */
    GravityGradientTorque() 
        : m_enabled(true)
        , m_mu(math::constants::EARTH_MU) {}

    /**
     * @brief Construct with configuration
     * @param config Configuration parameters
     */
    explicit GravityGradientTorque(const GravityGradientConfig& config)
        : m_enabled(true)
        , m_mu(config.centralBodyMu)
        , m_massProperties(config.massProperties) {}

    // ITorqueModel interface implementation
    math::Vector3D calculateTorque(const DynamicsState& state, double time) const override {
        if (!m_enabled || !m_massProperties) {
            return math::Vector3D();  // Zero torque
        }

        // Get position in inertial frame
        math::Vector3D positionInertial = state.getPosition();
        double r = positionInertial.magnitude();
        
        if (r < 1.0) {  // Avoid singularity at origin
            return math::Vector3D();
        }

        // Transform position to body frame
        math::Vector3D positionBody = state.inertialToBody(positionInertial);
        math::Vector3D rHatBody = positionBody / r;

        // Update mass properties if time-varying
        if (m_massProperties->isTimeVarying()) {
            m_massProperties->updateTime(time);
        }

        // Get inertia tensor
        math::Matrix3D inertia = m_massProperties->getInertiaTensor();

        // Calculate gravity gradient torque
        // τ = (3μ/r³) * (r̂ × I·r̂)
        double coefficient = 3.0 * m_mu / (r * r * r);
        math::Vector3D IrHat = inertia * rHatBody;
        math::Vector3D torque = rHatBody.cross(IrHat) * coefficient;

        return torque;
    }

    TorqueModelType getType() const override {
        return TorqueModelType::GravityGradient;
    }

    std::string getName() const override {
        return "Gravity Gradient Torque";
    }

    bool isEnabled() const override {
        return m_enabled;
    }

    void setEnabled(bool enabled) override {
        m_enabled = enabled;
    }

    void configure(const TorqueModelConfig& config) override {
        try {
            const auto& ggConfig = std::any_cast<const GravityGradientConfig&>(config);
            m_mu = ggConfig.centralBodyMu;
            m_massProperties = ggConfig.massProperties;
        } catch (const std::bad_any_cast&) {
            throw std::invalid_argument("Invalid configuration type for GravityGradientTorque");
        }
    }

    std::unique_ptr<ITorqueModel> clone() const override {
        auto cloned = std::make_unique<GravityGradientTorque>();
        cloned->m_enabled = m_enabled;
        cloned->m_mu = m_mu;
        cloned->m_massProperties = m_massProperties;
        return cloned;
    }

    /**
     * @brief Set gravitational parameter
     * @param mu Gravitational parameter in m³/s²
     */
    void setGravitationalParameter(double mu) {
        m_mu = mu;
    }

    /**
     * @brief Get gravitational parameter
     * @return Gravitational parameter in m³/s²
     */
    double getGravitationalParameter() const {
        return m_mu;
    }

    /**
     * @brief Set mass properties
     * @param massProperties Mass properties of the body
     */
    void setMassProperties(std::shared_ptr<IMassProperties> massProperties) {
        m_massProperties = massProperties;
    }

private:
    bool m_enabled;                                     ///< Enable flag
    double m_mu;                                        ///< Gravitational parameter
    std::shared_ptr<IMassProperties> m_massProperties; ///< Mass properties
};

} // namespace torques
} // namespace dynamics
} // namespace physics
} // namespace iloss