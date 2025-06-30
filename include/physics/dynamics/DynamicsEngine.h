#pragma once

#include "physics/dynamics/DynamicsState.h"
#include "physics/dynamics/IMassProperties.h"
#include "physics/dynamics/TorqueAggregator.h"
#include "physics/forces/ForceAggregator.h"
#include "core/math/Vector3D.h"
#include "core/math/Matrix3D.h"
#include "core/math/Quaternion.h"
#include <memory>

namespace iloss {
namespace physics {
namespace dynamics {

/**
 * @brief 6-DOF rigid body dynamics engine
 * 
 * This class implements the complete 6 degree-of-freedom dynamics for a rigid body,
 * including both translational motion (from Newton's laws) and rotational motion
 * (from Euler's equations). It integrates with the existing force model system
 * and adds torque models for complete dynamics simulation.
 * 
 * The dynamics equations solved are:
 * - Translational: F = ma (in inertial frame)
 * - Rotational: τ = Iω̇ + ω × Iω (in body frame)
 * - Kinematic: q̇ = 0.5 * q ⊗ ω_quat
 */
class DynamicsEngine {
public:
    /**
     * @brief Structure to hold dynamics state derivatives
     */
    struct DynamicsStateDerivatives {
        // Translational derivatives
        math::Vector3D velocity;            ///< Position derivative (= velocity)
        math::Vector3D acceleration;        ///< Velocity derivative

        // Rotational derivatives
        math::Quaternion attitudeRate;      ///< Attitude quaternion derivative
        math::Vector3D angularAcceleration; ///< Angular velocity derivative

        // Additional information
        double time = 0.0;                  ///< Time of calculation
        math::Vector3D totalForce;          ///< Total force in inertial frame
        math::Vector3D totalTorque;         ///< Total torque in body frame

        /**
         * @brief Convert derivatives to a flat vector for integration
         * @return Vector containing all derivative components
         * 
         * Order: [vx, vy, vz, ax, ay, az, qw', qx', qy', qz', ωx', ωy', ωz']
         */
        std::vector<double> toVector() const {
            return {
                velocity.x(), velocity.y(), velocity.z(),
                acceleration.x(), acceleration.y(), acceleration.z(),
                attitudeRate.w(), attitudeRate.x(), attitudeRate.y(), attitudeRate.z(),
                angularAcceleration.x(), angularAcceleration.y(), angularAcceleration.z()
            };
        }
    };

    /**
     * @brief Construct dynamics engine with mass properties
     * @param massProperties Mass properties of the rigid body
     * @param forceAggregator Force model aggregator (optional)
     * @param torqueAggregator Torque model aggregator (optional)
     */
    DynamicsEngine(std::shared_ptr<IMassProperties> massProperties,
                   std::shared_ptr<forces::ForceAggregator> forceAggregator = nullptr,
                   std::shared_ptr<TorqueAggregator> torqueAggregator = nullptr)
        : m_massProperties(massProperties)
        , m_forceAggregator(forceAggregator ? forceAggregator : std::make_shared<forces::ForceAggregator>())
        , m_torqueAggregator(torqueAggregator ? torqueAggregator : std::make_shared<TorqueAggregator>()) {
        
        if (!massProperties) {
            throw std::invalid_argument("Mass properties cannot be null");
        }
    }

    /**
     * @brief Calculate derivatives for 6-DOF dynamics
     * @param state Current dynamics state
     * @param time Current simulation time
     * @return State derivatives (velocity, acceleration, attitude rate, angular acceleration)
     */
    DynamicsStateDerivatives calculateDerivatives(const DynamicsState& state, double time) const;

    /**
     * @brief Get the force aggregator
     * @return Shared pointer to force aggregator
     */
    std::shared_ptr<forces::ForceAggregator> getForceAggregator() const {
        return m_forceAggregator;
    }

    /**
     * @brief Get the torque aggregator
     * @return Shared pointer to torque aggregator
     */
    std::shared_ptr<TorqueAggregator> getTorqueAggregator() const {
        return m_torqueAggregator;
    }

    /**
     * @brief Get the mass properties
     * @return Shared pointer to mass properties
     */
    std::shared_ptr<IMassProperties> getMassProperties() const {
        return m_massProperties;
    }

    /**
     * @brief Set new mass properties
     * @param massProperties New mass properties
     */
    void setMassProperties(std::shared_ptr<IMassProperties> massProperties) {
        if (!massProperties) {
            throw std::invalid_argument("Mass properties cannot be null");
        }
        m_massProperties = massProperties;
    }

    /**
     * @brief Apply derivatives to update state (for integration)
     * @param state Current state
     * @param derivatives State derivatives
     * @param dt Time step
     * @return Updated state
     */
    static DynamicsState applyDerivatives(const DynamicsState& state,
                                         const DynamicsStateDerivatives& derivatives,
                                         double dt);

private:
    std::shared_ptr<IMassProperties> m_massProperties;     ///< Mass properties of the body
    std::shared_ptr<forces::ForceAggregator> m_forceAggregator;  ///< Force models
    std::shared_ptr<TorqueAggregator> m_torqueAggregator;        ///< Torque models
};

} // namespace dynamics
} // namespace physics
} // namespace iloss