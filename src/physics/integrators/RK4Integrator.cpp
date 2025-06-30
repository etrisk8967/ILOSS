#include "physics/integrators/RK4Integrator.h"
#include "physics/state/StateVector.h"
#include "physics/forces/ForceAggregator.h"
#include "core/logging/Logger.h"
#include <stdexcept>
#include <cmath>

namespace iloss {
namespace physics {
namespace integrators {

using namespace iloss::logging;
using namespace iloss::math;

RK4Integrator::RK4Integrator(const IntegratorConfig& config)
    : IntegratorBase(config) {
    // RK4 is a fixed-step method, so we don't use adaptive parameters
}

StateVector RK4Integrator::computeDerivative(
    const StateVector& state,
    const Vector3D& acceleration,
    double massFlowRate) const {
    
    // Create derivative state
    // For orbital mechanics:
    // dr/dt = v (position derivative is velocity)
    // dv/dt = a (velocity derivative is acceleration)
    // dm/dt = massFlowRate (mass derivative)
    
    StateVector derivative;
    derivative.setPosition(state.getVelocity());  // dr/dt = v
    derivative.setVelocity(acceleration);         // dv/dt = a
    derivative.setMass(state.getMass());          // Mass doesn't change in derivative
    derivative.setTime(state.getTime());          // Time is handled separately
    derivative.setCoordinateSystem(state.getCoordinateSystem());
    
    // Note: We store mass in the derivative object but handle mass flow separately
    // This is because StateVector expects a valid mass value
    
    return derivative;
}

StateVector RK4Integrator::addScaledDerivative(
    const StateVector& state,
    const StateVector& derivative,
    double scale) const {
    
    StateVector result;
    
    // Position: r_new = r + scale * dr/dt
    result.setPosition(state.getPosition() + derivative.getPosition() * scale);
    
    // Velocity: v_new = v + scale * dv/dt
    result.setVelocity(state.getVelocity() + derivative.getVelocity() * scale);
    
    // Mass: For now, assume no mass change (propulsion will be handled separately)
    result.setMass(state.getMass());
    
    // Time: t_new = t + scale
    result.setTime(iloss::time::Time(state.getTime().getJ2000() + scale + iloss::time::TimeConstants::J2000_EPOCH, iloss::time::TimeSystem::UTC));
    
    // Preserve coordinate system
    result.setCoordinateSystem(state.getCoordinateSystem());
    
    return result;
}

StateVector RK4Integrator::integrate(
    const StateVector& initialState,
    forces::ForceAggregator& forceModel,
    double targetTime,
    StepCallback callback) {
    
    // Validate inputs
    if (!initialState.isValid()) {
        throw std::invalid_argument("RK4Integrator: Invalid initial state");
    }
    
    double initialTime = initialState.getTime().getJ2000();
    double timeSpan = targetTime - initialTime;
    
    if (std::abs(timeSpan) < std::numeric_limits<double>::epsilon()) {
        // No integration needed
        return initialState;
    }
    
    // Reset statistics if enabled
    if (m_config.enableStatistics) {
        resetStatistics();
    }
    
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Perform multi-step integration
    StateVector finalState = multiStepIntegration(
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

StateVector RK4Integrator::multiStepIntegration(
    const StateVector& currentState,
    forces::ForceAggregator& forceModel,
    double targetTime,
    StepCallback callback) {
    
    StateVector state = currentState;
    double currentTime = state.getTime().getJ2000();
    double remainingTime = targetTime - currentTime;
    
    // Determine step direction (forward or backward in time)
    double stepDirection = (remainingTime > 0) ? 1.0 : -1.0;
    remainingTime = std::abs(remainingTime);
    
    size_t iterationCount = 0;
    
    while (remainingTime > std::numeric_limits<double>::epsilon() && 
           iterationCount < m_config.maxIterations) {
        
        // Determine step size
        double stepSize = std::min(m_config.initialStepSize, remainingTime) * stepDirection;
        
        // Perform one RK4 step
        StepResult result = step(state, forceModel, stepSize);
        
        if (!result.stepAccepted) {
            throw std::runtime_error("RK4Integrator: Step rejected (should not happen for fixed-step)");
        }
        
        state = result.newState;
        
        // Call user callback if provided
        if (callback && !callback(state, state.getTime().getJ2000())) {
            break;
        }
        
        // Update remaining time
        currentTime = state.getTime().getJ2000();
        remainingTime = std::abs(targetTime - currentTime);
        
        iterationCount++;
    }
    
    if (iterationCount >= m_config.maxIterations) {
        throw std::runtime_error("RK4Integrator: Maximum iterations exceeded");
    }
    
    return state;
}

IIntegrator::StepResult RK4Integrator::step(
    const StateVector& currentState,
    forces::ForceAggregator& forceModel,
    double requestedStepSize) {
    
    auto stepStartTime = std::chrono::high_resolution_clock::now();
    
    if (!currentState.isValid()) {
        throw std::invalid_argument("RK4Integrator: Invalid state in step");
    }
    
    if (std::abs(requestedStepSize) < std::numeric_limits<double>::epsilon()) {
        throw std::invalid_argument("RK4Integrator: Step size too small");
    }
    
    const double h = requestedStepSize;
    const double h2 = h * 0.5;
    const double h6 = h / 6.0;
    
    // RK4 algorithm:
    // k1 = f(t, y)
    Vector3D k1_accel = forceModel.calculateTotalAcceleration(currentState, currentState.getTime());
    StateVector k1 = computeDerivative(currentState, k1_accel);
    
    if (m_config.enableStatistics) {
        m_statistics.functionEvaluations++;
    }
    
    // k2 = f(t + h/2, y + h*k1/2)
    StateVector state2 = addScaledDerivative(currentState, k1, h2);
    Vector3D k2_accel = forceModel.calculateTotalAcceleration(state2, state2.getTime());
    StateVector k2 = computeDerivative(state2, k2_accel);
    
    if (m_config.enableStatistics) {
        m_statistics.functionEvaluations++;
    }
    
    // k3 = f(t + h/2, y + h*k2/2)
    StateVector state3 = addScaledDerivative(currentState, k2, h2);
    Vector3D k3_accel = forceModel.calculateTotalAcceleration(state3, state3.getTime());
    StateVector k3 = computeDerivative(state3, k3_accel);
    
    if (m_config.enableStatistics) {
        m_statistics.functionEvaluations++;
    }
    
    // k4 = f(t + h, y + h*k3)
    StateVector state4 = addScaledDerivative(currentState, k3, h);
    Vector3D k4_accel = forceModel.calculateTotalAcceleration(state4, state4.getTime());
    StateVector k4 = computeDerivative(state4, k4_accel);
    
    if (m_config.enableStatistics) {
        m_statistics.functionEvaluations++;
    }
    
    // Combine: y_new = y + h/6 * (k1 + 2*k2 + 2*k3 + k4)
    Vector3D newPosition = currentState.getPosition() + 
        h6 * (k1.getPosition() + 2.0 * k2.getPosition() + 
              2.0 * k3.getPosition() + k4.getPosition());
    
    Vector3D newVelocity = currentState.getVelocity() + 
        h6 * (k1.getVelocity() + 2.0 * k2.getVelocity() + 
              2.0 * k3.getVelocity() + k4.getVelocity());
    
    // Create result
    StepResult result;
    result.newState.setPosition(newPosition);
    result.newState.setVelocity(newVelocity);
    result.newState.setMass(currentState.getMass());  // No mass change for now
    result.newState.setTime(currentState.getTime().getTime() + h);
    result.newState.setCoordinateSystem(currentState.getCoordinateSystem());
    result.actualStepSize = h;
    result.estimatedError = 0.0;  // RK4 doesn't provide error estimate
    result.stepAccepted = true;    // Fixed-step always accepts
    
    // Validate result
    if (!result.newState.isValid()) {
        throw std::runtime_error("RK4Integrator: Integration produced invalid state");
    }
    
    // Update statistics
    if (m_config.enableStatistics) {
        m_statistics.totalSteps++;
        m_statistics.acceptedSteps++;
        updateStepSizeStatistics(std::abs(h));
        
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