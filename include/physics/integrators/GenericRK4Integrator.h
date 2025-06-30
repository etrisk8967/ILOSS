#pragma once

#include "physics/integrators/GenericIntegrator.h"
#include "physics/integrators/StateConcepts.h"
#include <chrono>

namespace iloss {
namespace physics {
namespace integrators {

/**
 * @brief Generic fixed-step Runge-Kutta 4th order integrator
 * 
 * This template-based RK4 integrator works with any state type that satisfies
 * the IntegrableState concept, including both StateVector (3-DOF) and 
 * DynamicsState (6-DOF).
 * 
 * The RK4 method provides 4th order accuracy for smooth problems and is:
 * - Stable for moderate step sizes
 * - Energy-preserving for conservative systems
 * - Simple and reliable
 * - Fixed step size (not adaptive)
 * 
 * @tparam State State type that satisfies IntegrableState concept
 * @tparam System System type that provides state derivatives
 */
template<IntegrableState State, typename System>
class GenericRK4Integrator : public GenericIntegratorBase<State, System> {
private:
    /**
     * @brief Add scaled state to another state
     * 
     * Computes: state + scale * delta
     * This is used in the RK4 algorithm for intermediate calculations.
     * 
     * @param state Base state
     * @param delta State to add
     * @param scale Scaling factor
     * @return New state
     */
    State addScaledState(const State& state, const State& delta, double scale) const {
        // Use the required arithmetic operations from IntegrableState
        return state + delta * scale;
    }
    
public:
    /**
     * @brief Constructor with configuration
     * @param config Integrator configuration
     */
    explicit GenericRK4Integrator(const IntegratorConfig& config = IntegratorConfig())
        : GenericIntegratorBase<State, System>(config) {}
    
    /**
     * @brief Destructor
     */
    virtual ~GenericRK4Integrator() = default;
    
    /**
     * @brief Perform a single RK4 integration step
     * 
     * Executes the classic RK4 algorithm:
     * 1. k1 = h * f(t, y)
     * 2. k2 = h * f(t + h/2, y + k1/2)
     * 3. k3 = h * f(t + h/2, y + k2/2)
     * 4. k4 = h * f(t + h, y + k3)
     * 5. y_new = y + (k1 + 2*k2 + 2*k3 + k4)/6
     * 
     * @param currentState Current state
     * @param system System providing derivatives
     * @param requestedStepSize Time step size (h)
     * @return Step result with new state
     */
    typename IGenericIntegrator<State, System>::StepResult step(
        const State& currentState,
        System& system,
        double requestedStepSize) override {
        
        auto startTime = std::chrono::high_resolution_clock::now();
        
        const double h = requestedStepSize;
        const double t0 = currentState.getTimeAsDouble();
        
        // RK4 algorithm
        // k1 = h * f(t0, y0)
        State derivative1 = this->computeDerivative(currentState, system);
        State k1 = derivative1 * h;
        
        // k2 = h * f(t0 + h/2, y0 + k1/2)
        State temp = addScaledState(currentState, k1, 0.5);
        temp.setTime(t0 + h * 0.5);
        State derivative2 = this->computeDerivative(temp, system);
        State k2 = derivative2 * h;
        
        // k3 = h * f(t0 + h/2, y0 + k2/2)
        temp = addScaledState(currentState, k2, 0.5);
        temp.setTime(t0 + h * 0.5);
        State derivative3 = this->computeDerivative(temp, system);
        State k3 = derivative3 * h;
        
        // k4 = h * f(t0 + h, y0 + k3)
        temp = currentState + k3;
        temp.setTime(t0 + h);
        State derivative4 = this->computeDerivative(temp, system);
        State k4 = derivative4 * h;
        
        // Combine: y_new = y0 + (k1 + 2*k2 + 2*k3 + k4)/6
        State newState = currentState + 
                         (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (1.0 / 6.0);
        newState.setTime(t0 + h);
        
        // Update statistics
        if (this->m_config.enableStatistics) {
            this->m_statistics.totalSteps++;
            this->m_statistics.acceptedSteps++;
            this->updateStepSizeStatistics(h);
            
            auto endTime = std::chrono::high_resolution_clock::now();
            auto stepDuration = std::chrono::duration_cast<std::chrono::microseconds>(
                endTime - startTime);
            
            // Update average step time
            double n = static_cast<double>(this->m_statistics.totalSteps);
            auto avgMicros = this->m_statistics.averageStepTime.count();
            avgMicros = static_cast<std::chrono::microseconds::rep>(
                (avgMicros * (n - 1) + stepDuration.count()) / n);
            this->m_statistics.averageStepTime = std::chrono::microseconds(avgMicros);
        }
        
        // Prepare result
        typename IGenericIntegrator<State, System>::StepResult result;
        result.newState = newState;
        result.actualStepSize = h;
        result.estimatedError = 0.0;  // RK4 doesn't estimate error
        result.stepAccepted = true;   // RK4 always accepts steps
        
        return result;
    }
    
    /**
     * @brief Get integrator type name
     * @return "GenericRK4"
     */
    std::string getType() const override { 
        return "GenericRK4"; 
    }
    
    /**
     * @brief Check if this is an adaptive integrator
     * @return false (RK4 is fixed-step)
     */
    bool isAdaptive() const override { 
        return false; 
    }
    
    /**
     * @brief Get the order of the integration method
     * @return 4 (fourth-order accurate)
     */
    int getOrder() const override { 
        return 4; 
    }
};

/**
 * @brief Type alias for RK4 integrator with StateVector
 */
using RK4StateVectorIntegrator = GenericRK4Integrator<StateVector, forces::ForceAggregator>;

/**
 * @brief Type alias for RK4 integrator with DynamicsState
 */
using RK4DynamicsIntegrator = GenericRK4Integrator<::iloss::physics::dynamics::DynamicsState, 
                                                   DynamicsIntegratorAdapter>;

} // namespace integrators
} // namespace physics
} // namespace iloss