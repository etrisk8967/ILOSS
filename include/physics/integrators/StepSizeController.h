#pragma once

#include <vector>
#include <deque>
#include <algorithm>
#include <cmath>

namespace iloss {
namespace physics {
namespace integrators {

/**
 * @brief Advanced step size controller for adaptive integration methods
 * 
 * This class implements sophisticated step size control algorithms including:
 * - PI (Proportional-Integral) control
 * - Step size history tracking
 * - Stability detection
 * - Step size prediction
 * 
 * The controller helps adaptive integrators maintain optimal step sizes
 * that balance accuracy and computational efficiency.
 */
class StepSizeController {
public:
    /**
     * @brief Configuration for step size control
     */
    struct Config {
        // Basic limits
        double minStepSize = 1e-6;      ///< Minimum allowed step size
        double maxStepSize = 3600.0;    ///< Maximum allowed step size
        
        // Safety factors
        double safetyFactor = 0.9;      ///< Safety factor for conservative stepping
        double minScaleFactor = 0.1;    ///< Minimum step size reduction factor
        double maxScaleFactor = 5.0;    ///< Maximum step size increase factor
        
        // PI controller parameters
        bool usePIControl = true;       ///< Enable PI control
        double kP = 0.8;               ///< Proportional gain
        double kI = 0.3;               ///< Integral gain
        
        // History tracking
        size_t historySize = 10;        ///< Number of steps to track
        
        // Stability detection
        double stabilityThreshold = 0.1; ///< Variation threshold for stability
        size_t stabilityWindow = 5;      ///< Window size for stability check
        
        // Default constructor
        Config() = default;
    };
    
    /**
     * @brief Step information for history tracking
     */
    struct StepInfo {
        double stepSize;        ///< Step size used
        double errorEstimate;   ///< Error estimate
        bool accepted;          ///< Whether step was accepted
        double errorRatio;      ///< Ratio of tolerance to error
    };
    
private:
    Config m_config;
    std::deque<StepInfo> m_history;
    double m_lastErrorRatio = 1.0;
    double m_integralTerm = 0.0;
    
    /**
     * @brief Basic step size calculation
     */
    double basicStepSizeCalculation(
        double currentStepSize,
        double errorRatio,
        int methodOrder) const;
    
    /**
     * @brief PI controlled step size calculation
     */
    double piControlledStepSize(
        double currentStepSize,
        double errorRatio,
        int methodOrder);
    
    /**
     * @brief Check if recent steps indicate stable behavior
     */
    bool isStable() const;
    
    /**
     * @brief Apply safety limits to step size
     */
    double applyLimits(double stepSize, double currentStepSize) const;
    
public:
    /**
     * @brief Default constructor
     */
    StepSizeController();
    
    /**
     * @brief Constructor with configuration
     * @param config Step size controller configuration
     */
    explicit StepSizeController(const Config& config);
    
    /**
     * @brief Compute new step size based on error estimate
     * 
     * @param currentStepSize Current step size
     * @param errorEstimate Estimated error from integration
     * @param tolerance Error tolerance
     * @param methodOrder Order of the integration method
     * @param stepAccepted Whether the current step was accepted
     * @return Recommended new step size
     */
    double computeNewStepSize(
        double currentStepSize,
        double errorEstimate,
        double tolerance,
        int methodOrder,
        bool stepAccepted);
    
    /**
     * @brief Get optimal initial step size estimate
     * 
     * This method estimates a good initial step size based on the
     * problem characteristics and tolerance requirements.
     * 
     * @param stateNorm Norm of the initial state
     * @param derivativeNorm Norm of the initial derivative
     * @param tolerance Error tolerance
     * @param methodOrder Order of the integration method
     * @return Estimated initial step size
     */
    double estimateInitialStepSize(
        double stateNorm,
        double derivativeNorm,
        double tolerance,
        int methodOrder) const;
    
    /**
     * @brief Record step information for history tracking
     * 
     * @param info Step information to record
     */
    void recordStep(const StepInfo& info);
    
    /**
     * @brief Get average accepted step size from history
     * @return Average step size of accepted steps
     */
    double getAverageStepSize() const;
    
    /**
     * @brief Get step acceptance rate from history
     * @return Fraction of steps that were accepted
     */
    double getAcceptanceRate() const;
    
    /**
     * @brief Check if integration is currently stable
     * @return true if recent steps show stable behavior
     */
    bool checkStability() const { return isStable(); }
    
    /**
     * @brief Reset controller state
     */
    void reset();
    
    /**
     * @brief Get current configuration
     * @return Current configuration
     */
    const Config& getConfig() const { return m_config; }
    
    /**
     * @brief Set new configuration
     * @param config New configuration to apply
     */
    void setConfig(const Config& config) { m_config = config; }
    
    /**
     * @brief Get step history
     * @return Vector of recent step information
     */
    std::vector<StepInfo> getHistory() const {
        return std::vector<StepInfo>(m_history.begin(), m_history.end());
    }
    
    /**
     * @brief Predict next step size based on history
     * 
     * Uses historical data to predict an optimal step size for the next step.
     * This can be useful for pre-conditioning the integrator.
     * 
     * @return Predicted step size, or 0 if insufficient history
     */
    double predictNextStepSize() const;
};

} // namespace integrators
} // namespace physics
} // namespace iloss