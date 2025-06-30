#pragma once

#include "physics/integrators/StateConcepts.h"
#include "physics/integrators/IIntegrator.h"
#include <concepts>
#include <chrono>
#include <functional>
#include <type_traits>

namespace iloss {
namespace physics {
namespace integrators {

/**
 * @brief Generic integrator interface that works with any IntegrableState
 * 
 * This template-based interface allows numerical integrators to work with
 * both 3-DOF (StateVector) and 6-DOF (DynamicsState) representations,
 * as well as any future state types that satisfy the IntegrableState concept.
 * 
 * @tparam State State type that satisfies IntegrableState concept
 * @tparam System System type that provides state derivatives
 */
template<IntegrableState State, typename System>
class IGenericIntegrator {
public:
    /**
     * @brief Result of a single integration step
     */
    struct StepResult {
        State newState;                     ///< The new state after integration
        double actualStepSize;              ///< Actual step size used
        double estimatedError;              ///< Estimated error (for adaptive methods)
        bool stepAccepted;                  ///< Whether the step was accepted
        
        StepResult() : actualStepSize(0.0), estimatedError(0.0), stepAccepted(true) {}
    };
    
    /**
     * @brief Callback function type for intermediate results
     * @param state Current state
     * @param time Current time
     * @return true to continue integration, false to stop
     */
    using StepCallback = std::function<bool(const State& state, double time)>;
    
    virtual ~IGenericIntegrator() = default;
    
    /**
     * @brief Integrate a state from initial time to final time
     * 
     * @param initialState Initial state
     * @param system System that provides derivatives
     * @param targetTime Target time to integrate to
     * @param callback Optional callback for intermediate results
     * @return Final state at target time
     * @throws std::runtime_error if integration fails
     */
    virtual State integrate(
        const State& initialState,
        System& system,
        double targetTime,
        StepCallback callback = nullptr) = 0;
    
    /**
     * @brief Perform a single integration step
     * 
     * @param currentState Current state
     * @param system System that provides derivatives
     * @param requestedStepSize Requested time step
     * @return Step result containing new state and statistics
     */
    virtual StepResult step(
        const State& currentState,
        System& system,
        double requestedStepSize) = 0;
    
    /**
     * @brief Get the current configuration
     * @return Current integrator configuration
     */
    virtual const IntegratorConfig& getConfig() const = 0;
    
    /**
     * @brief Set new configuration
     * @param config New configuration to apply
     */
    virtual void setConfig(const IntegratorConfig& config) = 0;
    
    /**
     * @brief Get integration statistics
     * @return Current statistics
     */
    virtual const IntegratorStatistics& getStatistics() const = 0;
    
    /**
     * @brief Reset integration statistics
     */
    virtual void resetStatistics() = 0;
    
    /**
     * @brief Get the integrator type name
     * @return String identifier for the integrator type
     */
    virtual std::string getType() const = 0;
    
    /**
     * @brief Check if this is an adaptive integrator
     * @return true if adaptive, false if fixed-step
     */
    virtual bool isAdaptive() const = 0;
    
    /**
     * @brief Get the order of the integration method
     * @return Order of accuracy
     */
    virtual int getOrder() const = 0;
};

/**
 * @brief Base class providing common functionality for generic integrators
 * 
 * This template class implements common functionality shared by all generic integrators,
 * providing statistics tracking and configuration management.
 * 
 * @tparam State State type that satisfies IntegrableState concept
 * @tparam System System type that provides state derivatives
 */
template<IntegrableState State, typename System>
class GenericIntegratorBase : public IGenericIntegrator<State, System> {
protected:
    IntegratorConfig m_config;
    mutable IntegratorStatistics m_statistics;
    
    /**
     * @brief Update step size statistics
     * @param stepSize Step size used
     */
    void updateStepSizeStatistics(double stepSize) {
        if (m_config.enableStatistics) {
            m_statistics.minStepSizeUsed = std::min(m_statistics.minStepSizeUsed, stepSize);
            m_statistics.maxStepSizeUsed = std::max(m_statistics.maxStepSizeUsed, stepSize);
            
            // Update running average
            double n = static_cast<double>(m_statistics.totalSteps);
            if (n > 0) {
                m_statistics.averageStepSize = 
                    (m_statistics.averageStepSize * (n - 1.0) + stepSize) / n;
            } else {
                m_statistics.averageStepSize = stepSize;
            }
        }
    }
    
    /**
     * @brief Update error statistics
     * @param error Estimated error
     */
    void updateErrorStatistics(double error) {
        if (m_config.enableStatistics && error > 0) {
            m_statistics.maxEstimatedError = std::max(m_statistics.maxEstimatedError, error);
            
            // Update running average
            double n = static_cast<double>(m_statistics.acceptedSteps);
            m_statistics.averageError = 
                (m_statistics.averageError * n + error) / (n + 1.0);
        }
    }
    
    /**
     * @brief Compute derivative using the system
     * 
     * This helper method ensures proper derivative computation and tracks
     * function evaluations for statistics.
     * 
     * @param state Current state
     * @param system System providing derivatives
     * @return State derivative
     */
    State computeDerivative(const State& state, System& system) {
        if (m_config.enableStatistics) {
            m_statistics.functionEvaluations++;
        }
        
        if constexpr (DerivativeProvider<System, State>) {
            return system.computeDerivative(state);
        } else {
            static_assert(DerivativeProvider<System, State>,
                         "System must provide computeDerivative method");
        }
    }
    
public:
    /**
     * @brief Constructor with configuration
     * @param config Integrator configuration
     */
    explicit GenericIntegratorBase(const IntegratorConfig& config = IntegratorConfig())
        : m_config(config) {
        if (!m_config.isValid()) {
            throw std::invalid_argument("Invalid integrator configuration");
        }
    }
    
    const IntegratorConfig& getConfig() const override {
        return m_config;
    }
    
    void setConfig(const IntegratorConfig& config) override {
        if (!config.isValid()) {
            throw std::invalid_argument("Invalid integrator configuration");
        }
        m_config = config;
    }
    
    const IntegratorStatistics& getStatistics() const override {
        return m_statistics;
    }
    
    void resetStatistics() override {
        m_statistics.reset();
    }
    
    /**
     * @brief Default integration implementation
     * 
     * This provides a basic integration loop using the step() method.
     * Derived classes can override for more sophisticated implementations.
     */
    State integrate(
        const State& initialState,
        System& system,
        double targetTime,
        typename IGenericIntegrator<State, System>::StepCallback callback = nullptr) override {
        
        auto startTime = std::chrono::high_resolution_clock::now();
        
        State currentState = initialState;
        double currentTime = currentState.getTimeAsDouble();
        double dt = m_config.initialStepSize;
        
        size_t iterations = 0;
        
        while (currentTime < targetTime && iterations < m_config.maxIterations) {
            // Adjust step size to not overshoot target
            if (currentTime + dt > targetTime) {
                dt = targetTime - currentTime;
            }
            
            // Take a step
            auto result = this->step(currentState, system, dt);
            
            if (result.stepAccepted) {
                currentState = result.newState;
                currentTime = currentState.getTimeAsDouble();
                
                // Call callback if provided
                if (callback && !callback(currentState, currentTime)) {
                    break; // User requested stop
                }
                
                // Update step size for next iteration (adaptive methods)
                if (this->isAdaptive() && result.actualStepSize > 0) {
                    dt = result.actualStepSize;
                }
            }
            
            iterations++;
        }
        
        if (iterations >= m_config.maxIterations) {
            throw std::runtime_error("Integration exceeded maximum iterations");
        }
        
        // Update timing statistics
        if (m_config.enableStatistics) {
            auto endTime = std::chrono::high_resolution_clock::now();
            m_statistics.totalIntegrationTime = 
                std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
            
            if (m_statistics.totalSteps > 0) {
                m_statistics.averageStepTime = 
                    m_statistics.totalIntegrationTime / m_statistics.totalSteps;
            }
        }
        
        return currentState;
    }
};

/**
 * @brief Adapter to use legacy IIntegrator with StateVector as a generic integrator
 * 
 * This adapter allows existing integrators to be used in the generic framework
 * without modification, providing backward compatibility.
 */
class LegacyIntegratorAdapter : public IGenericIntegrator<StateVector, forces::ForceAggregator> {
private:
    std::unique_ptr<IIntegrator> m_legacyIntegrator;
    
public:
    explicit LegacyIntegratorAdapter(std::unique_ptr<IIntegrator> integrator)
        : m_legacyIntegrator(std::move(integrator)) {}
    
    StateVector integrate(
        const StateVector& initialState,
        forces::ForceAggregator& system,
        double targetTime,
        StepCallback callback = nullptr) override {
        
        // Convert callback if provided
        IIntegrator::StepCallback legacyCallback = nullptr;
        if (callback) {
            legacyCallback = [&callback](const StateVector& state, double time) {
                return callback(state, time);
            };
        }
        
        return m_legacyIntegrator->integrate(initialState, system, targetTime, legacyCallback);
    }
    
    StepResult step(
        const StateVector& currentState,
        forces::ForceAggregator& system,
        double requestedStepSize) override {
        
        auto legacyResult = m_legacyIntegrator->step(currentState, system, requestedStepSize);
        
        StepResult result;
        result.newState = legacyResult.newState;
        result.actualStepSize = legacyResult.actualStepSize;
        result.estimatedError = legacyResult.estimatedError;
        result.stepAccepted = legacyResult.stepAccepted;
        
        return result;
    }
    
    const IntegratorConfig& getConfig() const override {
        return m_legacyIntegrator->getConfig();
    }
    
    void setConfig(const IntegratorConfig& config) override {
        m_legacyIntegrator->setConfig(config);
    }
    
    const IntegratorStatistics& getStatistics() const override {
        return m_legacyIntegrator->getStatistics();
    }
    
    void resetStatistics() override {
        m_legacyIntegrator->resetStatistics();
    }
    
    std::string getType() const override {
        return "Legacy:" + m_legacyIntegrator->getType();
    }
    
    bool isAdaptive() const override {
        return m_legacyIntegrator->isAdaptive();
    }
    
    int getOrder() const override {
        return m_legacyIntegrator->getOrder();
    }
};

} // namespace integrators
} // namespace physics
} // namespace iloss