#include "physics/integrators/RK78Integrator.h"
#include "physics/state/StateVector.h"
#include "physics/forces/ForceAggregator.h"
#include "core/logging/Logger.h"
#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace iloss {
namespace physics {
namespace integrators {

using namespace iloss::logging;
using namespace iloss::math;

// Initialize the Runge-Kutta matrix coefficients (Fehlberg 7(8) tableau)
const std::array<std::array<double, 12>, 13> RK78Integrator::a = {{
    {}, // Stage 0 (not used)
    {2.0/27.0}, // Stage 1
    {1.0/36.0, 1.0/12.0}, // Stage 2
    {1.0/24.0, 0.0, 1.0/8.0}, // Stage 3
    {5.0/12.0, 0.0, -25.0/16.0, 25.0/16.0}, // Stage 4
    {1.0/20.0, 0.0, 0.0, 1.0/4.0, 1.0/5.0}, // Stage 5
    {-25.0/108.0, 0.0, 0.0, 125.0/108.0, -65.0/27.0, 125.0/54.0}, // Stage 6
    {31.0/300.0, 0.0, 0.0, 0.0, 61.0/225.0, -2.0/9.0, 13.0/900.0}, // Stage 7
    {2.0, 0.0, 0.0, -53.0/6.0, 704.0/45.0, -107.0/9.0, 67.0/90.0, 3.0}, // Stage 8
    {-91.0/108.0, 0.0, 0.0, 23.0/108.0, -976.0/135.0, 311.0/54.0, -19.0/60.0, 17.0/6.0, -1.0/12.0}, // Stage 9
    {2383.0/4100.0, 0.0, 0.0, -341.0/164.0, 4496.0/1025.0, -301.0/82.0, 2133.0/4100.0, 45.0/82.0, 45.0/164.0, 18.0/41.0}, // Stage 10
    {3.0/205.0, 0.0, 0.0, 0.0, 0.0, -6.0/41.0, -3.0/205.0, -3.0/41.0, 3.0/41.0, 6.0/41.0, 0.0}, // Stage 11
    {-1777.0/4100.0, 0.0, 0.0, -341.0/164.0, 4496.0/1025.0, -289.0/82.0, 2193.0/4100.0, 51.0/82.0, 33.0/164.0, 12.0/41.0, 0.0, 1.0} // Stage 12
}};

RK78Integrator::RK78Integrator(const IntegratorConfig& config)
    : IntegratorBase(config) {
}

StateVector RK78Integrator::computeDerivative(
    const StateVector& state,
    const Vector3D& acceleration) const {
    
    StateVector derivative;
    derivative.setPosition(state.getVelocity());  // dr/dt = v
    derivative.setVelocity(acceleration);         // dv/dt = a
    derivative.setMass(state.getMass());          // Mass preserved
    derivative.setTime(state.getTime());          // Time handled separately
    derivative.setCoordinateSystem(state.getCoordinateSystem());
    
    return derivative;
}

StateVector RK78Integrator::addWeightedDerivatives(
    const StateVector& state,
    const std::array<StateVector, STAGES>& k,
    const std::array<double, STAGES>& weights,
    double stepSize) const {
    
    Vector3D newPosition = state.getPosition();
    Vector3D newVelocity = state.getVelocity();
    
    // Add weighted contributions from each stage
    for (int i = 0; i < STAGES; ++i) {
        if (std::abs(weights[i]) > std::numeric_limits<double>::epsilon()) {
            newPosition = newPosition + k[i].getPosition() * (stepSize * weights[i]);
            newVelocity = newVelocity + k[i].getVelocity() * (stepSize * weights[i]);
        }
    }
    
    StateVector result;
    result.setPosition(newPosition);
    result.setVelocity(newVelocity);
    result.setMass(state.getMass());
    iloss::time::Time newTime = state.getTime();
    newTime.addSeconds(stepSize);
    result.setTime(newTime);
    result.setCoordinateSystem(state.getCoordinateSystem());
    
    return result;
}

StateVector RK78Integrator::computeStageState(
    const StateVector& baseState,
    const std::array<StateVector, STAGES>& k,
    int stage,
    double stepSize) const {
    
    Vector3D stagePosition = baseState.getPosition();
    Vector3D stageVelocity = baseState.getVelocity();
    
    // Add contributions from previous stages according to RK tableau
    for (int j = 0; j < stage; ++j) {
        if (std::abs(a[stage][j]) > std::numeric_limits<double>::epsilon()) {
            stagePosition = stagePosition + k[j].getPosition() * (stepSize * a[stage][j]);
            stageVelocity = stageVelocity + k[j].getVelocity() * (stepSize * a[stage][j]);
        }
    }
    
    StateVector stageState;
    stageState.setPosition(stagePosition);
    stageState.setVelocity(stageVelocity);
    stageState.setMass(baseState.getMass());
    iloss::time::Time stageTime = baseState.getTime();
    stageTime.addSeconds(stepSize * c[stage]);
    stageState.setTime(stageTime);
    stageState.setCoordinateSystem(baseState.getCoordinateSystem());
    
    return stageState;
}

double RK78Integrator::estimateError(
    const StateVector& solution7,
    const StateVector& solution8,
    const StateVector& currentState) const {
    
    // Compute error as the difference between 7th and 8th order solutions
    Vector3D positionError = solution8.getPosition() - solution7.getPosition();
    Vector3D velocityError = solution8.getVelocity() - solution7.getVelocity();
    
    // Compute error norms
    double positionErrorNorm = positionError.magnitude();
    double velocityErrorNorm = velocityError.magnitude();
    
    // Scale by current state magnitude for relative error
    double positionScale = std::max(currentState.getPosition().magnitude(), 1.0);
    double velocityScale = std::max(currentState.getVelocity().magnitude(), 1.0);
    
    // Combined error metric (RMS of position and velocity errors)
    double relativePositionError = positionErrorNorm / positionScale;
    double relativeVelocityError = velocityErrorNorm / velocityScale;
    
    // Weight position and velocity errors equally in the combined metric
    double combinedError = std::sqrt(0.5 * (relativePositionError * relativePositionError +
                                           relativeVelocityError * relativeVelocityError));
    
    return combinedError;
}

double RK78Integrator::computeNewStepSize(
    double currentStepSize,
    double errorEstimate,
    int order) const {
    
    // Compute tolerance (use the more restrictive of absolute and relative)
    double tolerance = m_config.relativeTolerance;
    
    if (errorEstimate < std::numeric_limits<double>::epsilon()) {
        // Error is essentially zero, increase step size
        return currentStepSize * m_config.maxScaleFactor;
    }
    
    // Compute optimal step size using error control formula
    // h_new = h_old * (tolerance / error)^(1/(order+1))
    double errorRatio = tolerance / errorEstimate;
    double scaleFactor = m_config.safetyFactor * std::pow(errorRatio, 1.0 / (order + 1));
    
    // Apply limits to scale factor
    scaleFactor = std::max(m_config.minScaleFactor, 
                          std::min(m_config.maxScaleFactor, scaleFactor));
    
    // Compute new step size with limits
    double newStepSize = currentStepSize * scaleFactor;
    newStepSize = std::max(m_config.minStepSize, 
                          std::min(m_config.maxStepSize, std::abs(newStepSize)));
    
    // Preserve sign of original step size
    return (currentStepSize < 0) ? -newStepSize : newStepSize;
}

StateVector RK78Integrator::integrate(
    const StateVector& initialState,
    forces::ForceAggregator& forceModel,
    double targetTime,
    StepCallback callback) {
    
    // Validate inputs
    if (!initialState.isValid()) {
        throw std::invalid_argument("RK78Integrator: Invalid initial state");
    }
    
    double initialTime = initialState.getTime().getJ2000(iloss::time::TimeSystem::UTC);
    double timeSpan = targetTime - initialTime;
    
    if (std::abs(timeSpan) < std::numeric_limits<double>::epsilon()) {
        return initialState;
    }
    
    // Reset statistics
    if (m_config.enableStatistics) {
        resetStatistics();
    }
    
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Perform adaptive integration
    StateVector finalState = adaptiveIntegration(
        initialState, forceModel, targetTime, callback);
    
    auto endTime = std::chrono::high_resolution_clock::now();
    
    if (m_config.enableStatistics) {
        m_statistics.totalIntegrationTime = 
            std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
        
        if (m_statistics.totalSteps > 0) {
            m_statistics.averageStepTime = 
                m_statistics.totalIntegrationTime / m_statistics.totalSteps;
        }
    }
    
    return finalState;
}

StateVector RK78Integrator::adaptiveIntegration(
    const StateVector& currentState,
    forces::ForceAggregator& forceModel,
    double targetTime,
    StepCallback callback) {
    
    StateVector state = currentState;
    double currentTime = state.getTime().getJ2000(iloss::time::TimeSystem::UTC);
    double remainingTime = targetTime - currentTime;
    double stepDirection = (remainingTime > 0) ? 1.0 : -1.0;
    
    // Initialize step size
    double stepSize = m_config.initialStepSize * stepDirection;
    
    // Apply max step size limit
    if (std::abs(stepSize) > m_config.maxStepSize) {
        stepSize = m_config.maxStepSize * stepDirection;
    }
    
    if (std::abs(stepSize) > std::abs(remainingTime)) {
        stepSize = remainingTime;
    }
    
    size_t iterationCount = 0;
    bool lastStep = false;
    
    // Use a more reasonable epsilon for time comparisons (1 microsecond)
    const double TIME_EPSILON = 1e-6;
    
    while (!lastStep && iterationCount < m_config.maxIterations) {
        // Check if this is the last step
        if (std::abs(stepSize) >= std::abs(targetTime - currentTime) - TIME_EPSILON) {
            stepSize = targetTime - currentTime;
            lastStep = true;
        }
        
        // Try a step
        StepResult result = step(state, forceModel, stepSize);
        
        if (result.stepAccepted) {
            // Step accepted, update state
            state = result.newState;
            currentTime = state.getTime().getJ2000(iloss::time::TimeSystem::UTC);
            
            // Call user callback
            if (callback && !callback(state, currentTime)) {
                break;
            }
            
            // Use the suggested step size for next iteration
            if (!lastStep && result.estimatedError > 0) {
                stepSize = computeNewStepSize(stepSize, result.estimatedError, getOrder());
                
                // Ensure step size respects limits
                if (std::abs(stepSize) > m_config.maxStepSize) {
                    stepSize = m_config.maxStepSize * stepDirection;
                }
                
                // Ensure we don't overshoot the target
                double remainingTime = std::abs(targetTime - currentTime);
                if (std::abs(stepSize) > remainingTime && remainingTime > TIME_EPSILON) {
                    stepSize = targetTime - currentTime;
                }
            }
        } else {
            // Step rejected, try again with smaller step size
            stepSize = result.actualStepSize;  // This contains the suggested smaller step
            
            // Check if step size is too small
            if (std::abs(stepSize) < m_config.minStepSize) {
                throw std::runtime_error("RK78Integrator: Step size below minimum threshold");
            }
        }
        
        iterationCount++;
    }
    
    if (iterationCount >= m_config.maxIterations) {
        throw std::runtime_error("RK78Integrator: Maximum iterations exceeded");
    }
    
    return state;
}

IIntegrator::StepResult RK78Integrator::step(
    const StateVector& currentState,
    forces::ForceAggregator& forceModel,
    double requestedStepSize) {
    
    auto stepStartTime = std::chrono::high_resolution_clock::now();
    
    if (!currentState.isValid()) {
        throw std::invalid_argument("RK78Integrator: Invalid state in step");
    }
    
    const double h = requestedStepSize;
    
    // Stage 0: k1 = f(t0, y0)
    k_vectors[0] = computeDerivative(currentState, 
                                    forceModel.calculateTotalAcceleration(currentState, currentState.getTime()));
    
    if (m_config.enableStatistics) {
        m_statistics.functionEvaluations++;
    }
    
    // Stages 1-12: Compute k_i using RK tableau
    for (int stage = 1; stage < STAGES; ++stage) {
        StateVector stageState = computeStageState(currentState, k_vectors, stage, h);
        k_vectors[stage] = computeDerivative(stageState, 
                                           forceModel.calculateTotalAcceleration(stageState, stageState.getTime()));
        
        if (m_config.enableStatistics) {
            m_statistics.functionEvaluations++;
        }
    }
    
    // Compute 7th order solution
    StateVector solution7 = addWeightedDerivatives(currentState, k_vectors, b7, h);
    
    // Compute 8th order solution for error estimation
    StateVector solution8 = addWeightedDerivatives(currentState, k_vectors, b8, h);
    
    // Estimate error
    double errorEstimate = estimateError(solution7, solution8, currentState);
    
    // Determine if step should be accepted
    bool stepAccepted = (errorEstimate <= m_config.relativeTolerance);
    
    // Prepare result
    StepResult result;
    
    if (stepAccepted) {
        result.newState = solution7;  // Use 7th order solution
        result.actualStepSize = h;
        result.estimatedError = errorEstimate;
        result.stepAccepted = true;
        
        // Validate result
        if (!result.newState.isValid()) {
            throw std::runtime_error("RK78Integrator: Integration produced invalid state");
        }
    } else {
        // Step rejected, suggest new step size
        double newStepSize = computeNewStepSize(h, errorEstimate, getOrder());
        result.actualStepSize = newStepSize;
        result.estimatedError = errorEstimate;
        result.stepAccepted = false;
    }
    
    // Update statistics
    if (m_config.enableStatistics) {
        m_statistics.totalSteps++;
        
        if (stepAccepted) {
            m_statistics.acceptedSteps++;
            updateStepSizeStatistics(std::abs(h));
            updateErrorStatistics(errorEstimate);
        } else {
            m_statistics.rejectedSteps++;
        }
        
        auto stepEndTime = std::chrono::high_resolution_clock::now();
        auto stepDuration = std::chrono::duration_cast<std::chrono::microseconds>(
            stepEndTime - stepStartTime);
        
        // Update average step time
        double n = static_cast<double>(m_statistics.totalSteps);
        auto avgMicros = m_statistics.averageStepTime.count();
        m_statistics.averageStepTime = std::chrono::microseconds(
            static_cast<long>((avgMicros * (n - 1) + stepDuration.count()) / n)
        );
    }
    
    return result;
}

} // namespace integrators
} // namespace physics
} // namespace iloss