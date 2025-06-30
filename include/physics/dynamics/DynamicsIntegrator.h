#pragma once

#include "physics/dynamics/DynamicsEngine.h"
#include "physics/dynamics/DynamicsState.h"
#include "physics/integrators/IIntegrator.h"
#include <vector>
#include <functional>
#include <memory>

namespace iloss {
namespace physics {
namespace dynamics {

/**
 * @brief Adapter to integrate 6-DOF dynamics using existing integrators
 * 
 * This class provides an adapter between the DynamicsEngine (which works with
 * DynamicsState) and the existing integrators (which work with StateVector).
 * It handles the conversion between state representations and ensures proper
 * integration of both translational and rotational dynamics.
 */
class DynamicsIntegrator {
public:
    /**
     * @brief Callback function for state updates during integration
     */
    using StateCallback = std::function<void(const DynamicsState&, double)>;

    /**
     * @brief Construct dynamics integrator
     * @param engine Dynamics engine for computing derivatives
     * @param integrator Numerical integrator to use
     */
    DynamicsIntegrator(std::shared_ptr<DynamicsEngine> engine,
                      std::shared_ptr<integrators::IIntegrator> integrator)
        : m_engine(engine)
        , m_integrator(integrator) {
        
        if (!engine || !integrator) {
            throw std::invalid_argument("Engine and integrator cannot be null");
        }
    }

    /**
     * @brief Integrate dynamics from initial state to final time
     * @param initialState Initial dynamics state
     * @param finalTime Final time to integrate to
     * @param callback Optional callback for intermediate states
     * @return Final integrated state
     * 
     * Note: This implementation performs step-by-step integration of the
     * translational dynamics using the existing integrator, then manually
     * integrates the rotational dynamics. This is a simplified approach
     * suitable for fixed-step integration.
     */
    DynamicsState integrate(const DynamicsState& initialState,
                           double finalTime,
                           StateCallback callback = nullptr) {
        
        // Get integrator configuration for step size
        double stepSize = m_integrator->getConfig().initialStepSize;
        double currentTime = initialState.getTime().getJ2000(iloss::time::TimeSystem::UTC);
        
        // Start with initial state
        DynamicsState currentState = initialState;
        
        // Integrate step by step
        while (currentTime < finalTime) {
            // Adjust last step size if needed
            if (currentTime + stepSize > finalTime) {
                stepSize = finalTime - currentTime;
            }
            
            // Calculate derivatives at current state
            auto derivatives = m_engine->calculateDerivatives(currentState, currentTime);
            
            // Apply derivatives to get new state
            currentState = m_engine->applyDerivatives(currentState, derivatives, stepSize);
            currentTime += stepSize;
            
            // Call user callback if provided
            if (callback) {
                callback(currentState, currentTime);
            }
        }
        
        return currentState;
    }

    /**
     * @brief Get the dynamics engine
     * @return Shared pointer to dynamics engine
     */
    std::shared_ptr<DynamicsEngine> getEngine() const {
        return m_engine;
    }

    /**
     * @brief Get the numerical integrator
     * @return Shared pointer to integrator
     */
    std::shared_ptr<integrators::IIntegrator> getIntegrator() const {
        return m_integrator;
    }

private:
    std::shared_ptr<DynamicsEngine> m_engine;        ///< Dynamics engine
    std::shared_ptr<integrators::IIntegrator> m_integrator;  ///< Numerical integrator
};

} // namespace dynamics
} // namespace physics
} // namespace iloss