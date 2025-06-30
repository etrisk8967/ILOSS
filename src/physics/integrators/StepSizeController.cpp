#include "physics/integrators/StepSizeController.h"
#include <numeric>
#include <limits>

namespace iloss {
namespace physics {
namespace integrators {

StepSizeController::StepSizeController()
    : m_config() {
}

StepSizeController::StepSizeController(const Config& config)
    : m_config(config) {
}

double StepSizeController::basicStepSizeCalculation(
    double currentStepSize,
    double errorRatio,
    int methodOrder) const {
    
    // Basic step size formula: h_new = h_old * (tolerance/error)^(1/(order+1))
    // errorRatio = tolerance/error
    
    if (errorRatio < std::numeric_limits<double>::epsilon()) {
        // Error is too large, reduce step size significantly
        return currentStepSize * m_config.minScaleFactor;
    }
    
    // Calculate scale factor
    double exponent = 1.0 / (methodOrder + 1.0);
    double scaleFactor = m_config.safetyFactor * std::pow(errorRatio, exponent);
    
    // Apply scale factor limits
    scaleFactor = std::max(m_config.minScaleFactor, 
                          std::min(m_config.maxScaleFactor, scaleFactor));
    
    return currentStepSize * scaleFactor;
}

double StepSizeController::piControlledStepSize(
    double currentStepSize,
    double errorRatio,
    int methodOrder) {
    
    // PI controller for step size adjustment
    // This provides smoother step size changes and better stability
    
    if (m_history.empty()) {
        // First step, use basic calculation
        m_lastErrorRatio = errorRatio;
        return basicStepSizeCalculation(currentStepSize, errorRatio, methodOrder);
    }
    
    // PI control formula
    double exponent = 1.0 / (methodOrder + 1.0);
    
    // Proportional term
    double proportionalTerm = std::pow(errorRatio, m_config.kP * exponent);
    
    // Integral term (based on previous error ratio)
    double integralTerm = std::pow(errorRatio / m_lastErrorRatio, m_config.kI * exponent);
    
    // Combined scale factor
    double scaleFactor = m_config.safetyFactor * proportionalTerm * integralTerm;
    
    // Apply limits
    scaleFactor = std::max(m_config.minScaleFactor, 
                          std::min(m_config.maxScaleFactor, scaleFactor));
    
    // Update last error ratio
    m_lastErrorRatio = errorRatio;
    
    return currentStepSize * scaleFactor;
}

bool StepSizeController::isStable() const {
    if (m_history.size() < m_config.stabilityWindow) {
        return false;  // Not enough history
    }
    
    // Calculate recent step sizes
    std::vector<double> recentSteps;
    size_t count = 0;
    for (auto it = m_history.rbegin(); 
         it != m_history.rend() && count < m_config.stabilityWindow; 
         ++it, ++count) {
        if (it->accepted) {
            recentSteps.push_back(it->stepSize);
        }
    }
    
    if (recentSteps.size() < m_config.stabilityWindow) {
        return false;  // Not enough accepted steps
    }
    
    // Calculate mean and standard deviation
    double mean = std::accumulate(recentSteps.begin(), recentSteps.end(), 0.0) / 
                  recentSteps.size();
    
    double variance = 0.0;
    for (double step : recentSteps) {
        double diff = step - mean;
        variance += diff * diff;
    }
    variance /= recentSteps.size();
    double stdDev = std::sqrt(variance);
    
    // Check if variation is within threshold
    double coefficientOfVariation = stdDev / mean;
    return coefficientOfVariation < m_config.stabilityThreshold;
}

double StepSizeController::applyLimits(double stepSize, double currentStepSize) const {
    // Apply absolute limits
    stepSize = std::max(m_config.minStepSize, 
                       std::min(m_config.maxStepSize, std::abs(stepSize)));
    
    // Apply relative limits (don't change too drastically in one step)
    double maxChange = currentStepSize * m_config.maxScaleFactor;
    double minChange = currentStepSize * m_config.minScaleFactor;
    
    stepSize = std::max(minChange, std::min(maxChange, stepSize));
    
    // Preserve sign
    return (currentStepSize < 0) ? -stepSize : stepSize;
}

double StepSizeController::computeNewStepSize(
    double currentStepSize,
    double errorEstimate,
    double tolerance,
    int methodOrder,
    bool stepAccepted) {
    
    // Calculate error ratio
    double errorRatio = tolerance / std::max(errorEstimate, 
                                            std::numeric_limits<double>::epsilon());
    
    // Calculate new step size
    double newStepSize;
    if (m_config.usePIControl && m_history.size() > 1) {
        newStepSize = piControlledStepSize(currentStepSize, errorRatio, methodOrder);
    } else {
        newStepSize = basicStepSizeCalculation(currentStepSize, errorRatio, methodOrder);
    }
    
    if (!stepAccepted) {
        // Step was rejected, be more conservative
        // Apply additional reduction factor for rejected steps
        newStepSize *= 0.8;  // Reduce by 20% more for rejected steps
    }
    
    // Apply limits
    newStepSize = applyLimits(newStepSize, currentStepSize);
    
    // Record step information
    StepInfo info;
    info.stepSize = std::abs(currentStepSize);
    info.errorEstimate = errorEstimate;
    info.accepted = stepAccepted;
    info.errorRatio = errorRatio;
    recordStep(info);
    
    // If integration is stable, allow larger increases
    if (isStable() && errorRatio > 2.0) {
        double stableScaleFactor = std::min(2.0 * m_config.maxScaleFactor, 10.0);
        newStepSize = std::min(newStepSize * 1.5, currentStepSize * stableScaleFactor);
        // Re-apply limits after potential increase
        newStepSize = applyLimits(newStepSize, currentStepSize);
    }
    
    return newStepSize;
}

double StepSizeController::estimateInitialStepSize(
    double stateNorm,
    double derivativeNorm,
    double tolerance,
    int methodOrder) const {
    
    // Estimate initial step size based on problem scales
    // This uses the formula: h0 = (tolerance / ||f||)^(1/(order+1))
    
    if (derivativeNorm < std::numeric_limits<double>::epsilon()) {
        // Derivative is essentially zero, use maximum step size
        return m_config.maxStepSize;
    }
    
    // Scale tolerance by state norm for relative accuracy
    double scaledTolerance = tolerance * std::max(stateNorm, 1.0);
    
    // Estimate step size
    double h0 = std::pow(scaledTolerance / derivativeNorm, 1.0 / (methodOrder + 1.0));
    
    // Apply safety factor
    h0 *= m_config.safetyFactor;
    
    // Apply limits
    return std::max(m_config.minStepSize, std::min(m_config.maxStepSize, h0));
}

void StepSizeController::recordStep(const StepInfo& info) {
    m_history.push_back(info);
    
    // Maintain history size limit
    while (m_history.size() > m_config.historySize) {
        m_history.pop_front();
    }
}

double StepSizeController::getAverageStepSize() const {
    if (m_history.empty()) {
        return 0.0;
    }
    
    double sum = 0.0;
    size_t count = 0;
    
    for (const auto& info : m_history) {
        if (info.accepted) {
            sum += info.stepSize;
            count++;
        }
    }
    
    return count > 0 ? sum / count : 0.0;
}

double StepSizeController::getAcceptanceRate() const {
    if (m_history.empty()) {
        return 1.0;
    }
    
    size_t accepted = 0;
    for (const auto& info : m_history) {
        if (info.accepted) {
            accepted++;
        }
    }
    
    return static_cast<double>(accepted) / m_history.size();
}

void StepSizeController::reset() {
    m_history.clear();
    m_lastErrorRatio = 1.0;
    m_integralTerm = 0.0;
}

double StepSizeController::predictNextStepSize() const {
    if (m_history.size() < 3) {
        return 0.0;  // Not enough history
    }
    
    // Use recent accepted steps for prediction
    std::vector<double> recentSteps;
    for (auto it = m_history.rbegin(); it != m_history.rend(); ++it) {
        if (it->accepted) {
            recentSteps.push_back(it->stepSize);
            if (recentSteps.size() >= 5) break;
        }
    }
    
    if (recentSteps.size() < 2) {
        return 0.0;
    }
    
    // For increasing trend, predict slightly higher than weighted average
    // Check if there's an increasing trend
    bool increasing = true;
    for (size_t i = 1; i < recentSteps.size(); ++i) {
        if (recentSteps[i] > recentSteps[i-1]) {
            increasing = false;
            break;
        }
    }
    
    // Simple prediction: weighted average with more weight on recent steps
    double weightedSum = 0.0;
    double totalWeight = 0.0;
    double weight = 1.0;
    
    for (double step : recentSteps) {
        weightedSum += step * weight;
        totalWeight += weight;
        weight *= 0.7;  // Decay factor
    }
    
    double prediction = weightedSum / totalWeight;
    
    // For increasing trends, predict slightly higher
    if (increasing && recentSteps.size() >= 3) {
        double trend = (recentSteps[0] - recentSteps[recentSteps.size()-1]) / (recentSteps.size() - 1);
        if (trend > 0) {
            prediction += trend * 0.5; // Add half of the average trend
        }
    }
    
    return prediction;
}

} // namespace integrators
} // namespace physics
} // namespace iloss