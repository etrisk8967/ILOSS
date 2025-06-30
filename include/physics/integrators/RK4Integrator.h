#pragma once

#include "physics/integrators/IIntegrator.h"
#include "core/math/Vector3D.h"

namespace iloss {
namespace physics {

// Forward declarations
class StateVector;
namespace forces {
class ForceAggregator;
} // namespace forces

namespace integrators {

/**
 * @brief Fixed-step Runge-Kutta 4th order integrator
 * 
 * The RK4 integrator is a classic numerical integration method that provides
 * 4th order accuracy for smooth problems. It uses four function evaluations
 * per step to achieve higher accuracy than simpler methods like Euler.
 * 
 * The RK4 method is:
 * - Stable for moderate step sizes
 * - Preserves energy well for conservative systems
 * - Simple to implement and understand
 * - Not adaptive (fixed step size)
 * 
 * Thread Safety: This implementation is thread-safe. Each instance maintains
 * its own state and statistics.
 */
class RK4Integrator : public IntegratorBase {
private:
    /**
     * @brief Apply RK4 formula to compute state derivative
     * 
     * For orbital mechanics, this computes:
     * - dr/dt = v (velocity)
     * - dv/dt = a (acceleration from forces)
     * - dm/dt = mass flow rate (if applicable)
     * 
     * @param state Current state
     * @param acceleration Acceleration vector from force model
     * @param massFlowRate Rate of mass change (usually 0 for unpowered flight)
     * @return State derivative [velocity, acceleration, mass_rate]
     */
    StateVector computeDerivative(
        const StateVector& state,
        const math::Vector3D& acceleration,
        double massFlowRate = 0.0) const;
    
    /**
     * @brief Add scaled derivative to state
     * 
     * Computes: state + scale * derivative
     * Properly handles position, velocity, mass, and time components
     * 
     * @param state Base state
     * @param derivative State derivative
     * @param scale Scaling factor (typically h or h/2)
     * @return New state
     */
    StateVector addScaledDerivative(
        const StateVector& state,
        const StateVector& derivative,
        double scale) const;
    
    /**
     * @brief Perform integration over multiple steps
     * 
     * @param currentState Starting state
     * @param forceModel Force model for accelerations
     * @param targetTime Target time to reach
     * @param callback Optional callback for each step
     * @return Final state at target time
     */
    StateVector multiStepIntegration(
        const StateVector& currentState,
        forces::ForceAggregator& forceModel,
        double targetTime,
        StepCallback callback);
    
public:
    /**
     * @brief Constructor with configuration
     * @param config Integrator configuration (step size from initialStepSize)
     */
    explicit RK4Integrator(const IntegratorConfig& config = IntegratorConfig());
    
    /**
     * @brief Destructor
     */
    virtual ~RK4Integrator() = default;
    
    /**
     * @brief Integrate from initial state to target time
     * 
     * Uses fixed step size from configuration. If the total time span
     * is not evenly divisible by the step size, the last step will be
     * adjusted to reach the target time exactly.
     * 
     * @param initialState Initial state vector
     * @param forceModel Force model to compute accelerations
     * @param targetTime Target time to integrate to
     * @param callback Optional callback for intermediate results
     * @return Final state vector at target time
     * @throws std::runtime_error if integration fails
     * @throws std::invalid_argument if target time is before initial time
     */
    StateVector integrate(
        const StateVector& initialState,
        forces::ForceAggregator& forceModel,
        double targetTime,
        StepCallback callback = nullptr) override;
    
    /**
     * @brief Perform a single RK4 integration step
     * 
     * Executes one step of the RK4 algorithm:
     * 1. Compute k1 = f(t, y)
     * 2. Compute k2 = f(t + h/2, y + h*k1/2)
     * 3. Compute k3 = f(t + h/2, y + h*k2/2)
     * 4. Compute k4 = f(t + h, y + h*k3)
     * 5. Update: y_new = y + h/6 * (k1 + 2*k2 + 2*k3 + k4)
     * 
     * @param currentState Current state vector
     * @param forceModel Force model to compute accelerations
     * @param requestedStepSize Time step size (h)
     * @return Step result with new state
     */
    StepResult step(
        const StateVector& currentState,
        forces::ForceAggregator& forceModel,
        double requestedStepSize) override;
    
    /**
     * @brief Get integrator type name
     * @return "RK4"
     */
    std::string getType() const override { return "RK4"; }
    
    /**
     * @brief Check if this is an adaptive integrator
     * @return false (RK4 is fixed-step)
     */
    bool isAdaptive() const override { return false; }
    
    /**
     * @brief Get the order of the integration method
     * @return 4 (fourth-order accurate)
     */
    int getOrder() const override { return 4; }
};

} // namespace integrators
} // namespace physics
} // namespace iloss