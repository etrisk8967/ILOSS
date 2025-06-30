#pragma once

#include "physics/dynamics/DynamicsEngine.h"
#include "physics/dynamics/DynamicsState.h"
#include "physics/integrators/StateConcepts.h"
#include <memory>

namespace iloss {
namespace physics {
namespace integrators {

/**
 * @brief Adapter to use DynamicsEngine with generic integrators
 * 
 * This adapter class wraps a DynamicsEngine to provide the computeDerivative
 * method required by the generic integrator framework. It converts between
 * the DynamicsEngine's derivative structure and the DynamicsState format
 * expected by the integrators.
 * 
 * The adapter allows the existing DynamicsEngine to be used seamlessly with
 * the new generic integrators (GenericRK4Integrator, GenericRK78Integrator)
 * without modifying the DynamicsEngine implementation.
 */
class DynamicsIntegratorAdapter {
private:
    std::shared_ptr<dynamics::DynamicsEngine> m_dynamicsEngine;
    
public:
    /**
     * @brief Constructor
     * @param dynamicsEngine The dynamics engine to wrap
     */
    explicit DynamicsIntegratorAdapter(std::shared_ptr<dynamics::DynamicsEngine> dynamicsEngine)
        : m_dynamicsEngine(dynamicsEngine) {
        if (!dynamicsEngine) {
            throw std::invalid_argument("DynamicsEngine cannot be null");
        }
    }
    
    /**
     * @brief Compute the derivative of a DynamicsState
     * 
     * This method is required by the DerivativeProvider concept and is called
     * by the generic integrators. It converts the DynamicsEngine's derivative
     * structure into a DynamicsState that represents the rate of change.
     * 
     * @param state Current dynamics state
     * @return DynamicsState representing the derivative
     */
    dynamics::DynamicsState computeDerivative(const dynamics::DynamicsState& state) {
        // Get time from state
        double time = state.getTimeAsDouble();
        
        // Calculate derivatives using DynamicsEngine
        auto derivatives = m_dynamicsEngine->calculateDerivatives(state, time);
        
        // Convert derivatives to DynamicsState format
        // For integration purposes, the "state" derivative contains:
        // - Position derivative = velocity
        // - Velocity derivative = acceleration
        // - Attitude derivative = attitude rate quaternion
        // - Angular velocity derivative = angular acceleration
        
        // Create a derivative state with the rates of change
        // Use the special factory method that doesn't normalize the quaternion derivative
        dynamics::DynamicsState derivativeState = dynamics::DynamicsState::createDerivativeState(
            derivatives.velocity,                    // Position rate = velocity
            derivatives.acceleration,                // Velocity rate = acceleration
            0.0,                                    // Mass rate (typically 0 for unpowered flight)
            0.0,                                    // Time (not used in derivative)
            derivatives.attitudeRate,               // Attitude rate (quaternion derivative)
            derivatives.angularAcceleration,        // Angular velocity rate
            state.getCoordinateSystem()
        );
        
        // Important: The derivative state represents rates of change, not absolute values
        // The integrator will handle the proper scaling and addition
        
        return derivativeState;
    }
    
    /**
     * @brief Get the wrapped dynamics engine
     * @return Shared pointer to the dynamics engine
     */
    std::shared_ptr<dynamics::DynamicsEngine> getDynamicsEngine() const {
        return m_dynamicsEngine;
    }
    
    /**
     * @brief Get the force aggregator from the dynamics engine
     * @return Shared pointer to force aggregator
     */
    std::shared_ptr<forces::ForceAggregator> getForceAggregator() const {
        return m_dynamicsEngine->getForceAggregator();
    }
    
    /**
     * @brief Get the torque aggregator from the dynamics engine
     * @return Shared pointer to torque aggregator
     */
    std::shared_ptr<dynamics::TorqueAggregator> getTorqueAggregator() const {
        return m_dynamicsEngine->getTorqueAggregator();
    }
    
    /**
     * @brief Get the mass properties from the dynamics engine
     * @return Shared pointer to mass properties
     */
    std::shared_ptr<dynamics::IMassProperties> getMassProperties() const {
        return m_dynamicsEngine->getMassProperties();
    }
};

// Verify that DynamicsIntegratorAdapter satisfies the DerivativeProvider concept
static_assert(DerivativeProvider<DynamicsIntegratorAdapter, dynamics::DynamicsState>,
              "DynamicsIntegratorAdapter must satisfy DerivativeProvider concept");

} // namespace integrators
} // namespace physics
} // namespace iloss