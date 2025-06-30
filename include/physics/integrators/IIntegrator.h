#pragma once

#include <memory>
#include <vector>
#include <functional>
#include <chrono>
#include "physics/state/StateVector.h"

namespace iloss {
namespace physics {

// Forward declarations
namespace forces {
class ForceAggregator;
} // namespace forces

namespace integrators {

/**
 * @brief Configuration parameters for numerical integrators
 * 
 * This class provides a flexible configuration system for numerical integrators,
 * supporting both fixed-step and adaptive methods with error control.
 */
class IntegratorConfig {
public:
    // Step size parameters
    double initialStepSize = 1.0;          ///< Initial time step in seconds
    double minStepSize = 1e-6;             ///< Minimum allowed step size
    double maxStepSize = 3600.0;           ///< Maximum allowed step size
    
    // Error control parameters (for adaptive methods)
    double absoluteTolerance = 1e-9;        ///< Absolute error tolerance
    double relativeTolerance = 1e-12;       ///< Relative error tolerance
    
    // Safety factors for adaptive step sizing
    double safetyFactor = 0.9;              ///< Safety factor for step size adjustment
    double minScaleFactor = 0.1;            ///< Minimum step size scale factor
    double maxScaleFactor = 5.0;            ///< Maximum step size scale factor
    
    // Integration control
    size_t maxIterations = 1000000;         ///< Maximum iterations before timeout
    bool enableStatistics = true;           ///< Enable statistics collection
    
    /**
     * @brief Check if configuration is valid
     * @return true if valid, false otherwise
     */
    bool isValid() const {
        return initialStepSize > 0 && 
               minStepSize > 0 && 
               maxStepSize > minStepSize &&
               absoluteTolerance > 0 &&
               relativeTolerance > 0 &&
               safetyFactor > 0 && safetyFactor <= 1.0 &&
               minScaleFactor > 0 && minScaleFactor < 1.0 &&
               maxScaleFactor > 1.0 &&
               maxIterations > 0;
    }
};

/**
 * @brief Statistics collected during numerical integration
 * 
 * This class tracks performance metrics and statistics during integration
 * to help analyze and optimize the integration process.
 */
class IntegratorStatistics {
public:
    // Step count statistics
    size_t totalSteps = 0;                  ///< Total number of integration steps
    size_t acceptedSteps = 0;               ///< Number of accepted steps (adaptive)
    size_t rejectedSteps = 0;               ///< Number of rejected steps (adaptive)
    
    // Step size statistics
    double minStepSizeUsed = std::numeric_limits<double>::max();
    double maxStepSizeUsed = 0.0;
    double averageStepSize = 0.0;
    
    // Error statistics (for adaptive methods)
    double maxEstimatedError = 0.0;         ///< Maximum estimated error encountered
    double averageError = 0.0;              ///< Average estimated error
    
    // Function evaluation counts
    size_t functionEvaluations = 0;         ///< Total force/acceleration evaluations
    
    // Timing statistics
    std::chrono::microseconds totalIntegrationTime{0};
    std::chrono::microseconds averageStepTime{0};
    
    /**
     * @brief Reset all statistics to initial values
     */
    void reset() {
        totalSteps = 0;
        acceptedSteps = 0;
        rejectedSteps = 0;
        minStepSizeUsed = std::numeric_limits<double>::max();
        maxStepSizeUsed = 0.0;
        averageStepSize = 0.0;
        maxEstimatedError = 0.0;
        averageError = 0.0;
        functionEvaluations = 0;
        totalIntegrationTime = std::chrono::microseconds{0};
        averageStepTime = std::chrono::microseconds{0};
    }
    
    /**
     * @brief Get step acceptance rate for adaptive methods
     * @return Acceptance rate (0-1)
     */
    double getAcceptanceRate() const {
        size_t totalAttempts = acceptedSteps + rejectedSteps;
        return totalAttempts > 0 ? static_cast<double>(acceptedSteps) / totalAttempts : 1.0;
    }
};

/**
 * @brief Abstract base class for numerical integrators
 * 
 * This interface defines the contract for all numerical integration methods
 * used in the ILOSS physics engine. Integrators propagate StateVectors forward
 * in time using forces computed by a ForceAggregator.
 * 
 * Thread Safety: Implementations should be thread-safe for use in parallel simulations.
 */
class IIntegrator {
public:
    /**
     * @brief Result of a single integration step
     */
    struct StepResult {
        StateVector newState;               ///< The new state after integration
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
    using StepCallback = std::function<bool(const StateVector& state, double time)>;
    
    virtual ~IIntegrator() = default;
    
    /**
     * @brief Integrate a state vector from initial time to final time
     * 
     * @param initialState Initial state vector
     * @param forceModel Force model to compute accelerations
     * @param targetTime Target time to integrate to
     * @param callback Optional callback for intermediate results
     * @return Final state vector at target time
     * @throws std::runtime_error if integration fails
     */
    virtual StateVector integrate(
        const StateVector& initialState,
        forces::ForceAggregator& forceModel,
        double targetTime,
        StepCallback callback = nullptr) = 0;
    
    /**
     * @brief Perform a single integration step
     * 
     * This method is useful for custom integration loops or testing.
     * 
     * @param currentState Current state vector
     * @param forceModel Force model to compute accelerations
     * @param requestedStepSize Requested time step (may be adjusted by adaptive methods)
     * @return Step result containing new state and statistics
     */
    virtual StepResult step(
        const StateVector& currentState,
        forces::ForceAggregator& forceModel,
        double requestedStepSize) = 0;
    
    /**
     * @brief Get the current configuration
     * @return Current integrator configuration
     */
    virtual const IntegratorConfig& getConfig() const = 0;
    
    /**
     * @brief Set new configuration
     * @param config New configuration to apply
     * @throws std::invalid_argument if configuration is invalid
     */
    virtual void setConfig(const IntegratorConfig& config) = 0;
    
    /**
     * @brief Get integration statistics
     * @return Current statistics (may be empty if statistics are disabled)
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
     * @return Order of accuracy (e.g., 4 for RK4)
     */
    virtual int getOrder() const = 0;
};

/**
 * @brief Base class providing common functionality for integrators
 * 
 * This class implements common functionality shared by all integrators,
 * reducing code duplication in concrete implementations.
 */
class IntegratorBase : public IIntegrator {
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
            // Note: This should be called AFTER totalSteps is incremented
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
     * @brief Update error statistics (for adaptive methods)
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
    
public:
    /**
     * @brief Constructor with configuration
     * @param config Integrator configuration
     */
    explicit IntegratorBase(const IntegratorConfig& config = IntegratorConfig())
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
};

} // namespace integrators
} // namespace physics
} // namespace iloss