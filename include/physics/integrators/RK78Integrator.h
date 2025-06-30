#pragma once

#include "physics/integrators/IIntegrator.h"
#include "core/math/Vector3D.h"
#include <array>

namespace iloss {
namespace physics {

// Forward declarations
class StateVector;
namespace forces {
class ForceAggregator;
} // namespace forces

namespace integrators {

/**
 * @brief Adaptive Runge-Kutta-Fehlberg 7(8) integrator
 * 
 * The RK78 integrator is a high-order adaptive integration method that provides
 * excellent accuracy for smooth problems. It uses an embedded 8th-order solution
 * to estimate the error in the 7th-order solution, allowing for automatic step
 * size adjustment.
 * 
 * Key features:
 * - 7th order accuracy with 8th order error estimation
 * - Adaptive step size control based on error tolerance
 * - 13 function evaluations per step
 * - Excellent for high-precision orbital propagation
 * - FSAL (First Same As Last) property for efficiency
 * 
 * This implementation uses the Fehlberg 7(8) coefficients, which are optimized
 * for minimal truncation error.
 * 
 * Thread Safety: This implementation is thread-safe. Each instance maintains
 * its own state and statistics.
 */
class RK78Integrator : public IntegratorBase {
private:
    // Butcher tableau coefficients for Fehlberg 7(8)
    // These are the standard coefficients from Fehlberg's 1968 paper
    static constexpr int STAGES = 13;
    
    // Time coefficients (c_i)
    static constexpr std::array<double, STAGES> c = {
        0.0, 
        2.0/27.0, 
        1.0/9.0, 
        1.0/6.0, 
        5.0/12.0, 
        1.0/2.0, 
        5.0/6.0, 
        1.0/6.0, 
        2.0/3.0, 
        1.0/3.0, 
        1.0, 
        0.0, 
        1.0
    };
    
    // Runge-Kutta matrix coefficients (a_ij)
    // Stored as a flattened lower triangular matrix
    static const std::array<std::array<double, 12>, 13> a;
    
    // 7th order solution coefficients (b_i)
    static constexpr std::array<double, STAGES> b7 = {
        41.0/840.0,
        0.0,
        0.0,
        0.0,
        0.0,
        34.0/105.0,
        9.0/35.0,
        9.0/35.0,
        9.0/280.0,
        9.0/280.0,
        41.0/840.0,
        0.0,
        0.0
    };
    
    // 8th order solution coefficients (b*_i) for error estimation
    static constexpr std::array<double, STAGES> b8 = {
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        34.0/105.0,
        9.0/35.0,
        9.0/35.0,
        9.0/280.0,
        9.0/280.0,
        0.0,
        41.0/840.0,
        41.0/840.0
    };
    
    /**
     * @brief Storage for k vectors (derivatives at each stage)
     * 
     * We store these as member variables to avoid repeated allocations
     * in the inner integration loop.
     */
    mutable std::array<StateVector, STAGES> k_vectors;
    
    /**
     * @brief Compute state derivative
     */
    StateVector computeDerivative(
        const StateVector& state,
        const math::Vector3D& acceleration) const;
    
    /**
     * @brief Add weighted combination of derivatives to state
     */
    StateVector addWeightedDerivatives(
        const StateVector& state,
        const std::array<StateVector, STAGES>& k,
        const std::array<double, STAGES>& weights,
        double stepSize) const;
    
    /**
     * @brief Compute stage value using RK tableau
     */
    StateVector computeStageState(
        const StateVector& baseState,
        const std::array<StateVector, STAGES>& k,
        int stage,
        double stepSize) const;
    
    /**
     * @brief Estimate integration error
     */
    double estimateError(
        const StateVector& solution7,
        const StateVector& solution8,
        const StateVector& currentState) const;
    
    /**
     * @brief Compute new step size based on error
     */
    double computeNewStepSize(
        double currentStepSize,
        double errorEstimate,
        int order) const;
    
    /**
     * @brief Perform adaptive multi-step integration
     */
    StateVector adaptiveIntegration(
        const StateVector& currentState,
        forces::ForceAggregator& forceModel,
        double targetTime,
        StepCallback callback);
    
public:
    /**
     * @brief Constructor with configuration
     * @param config Integrator configuration with error tolerances
     */
    explicit RK78Integrator(const IntegratorConfig& config = IntegratorConfig());
    
    /**
     * @brief Destructor
     */
    virtual ~RK78Integrator() = default;
    
    /**
     * @brief Integrate from initial state to target time
     * 
     * Uses adaptive step size control to maintain the specified error
     * tolerances. The step size will be automatically adjusted based
     * on the estimated error.
     * 
     * @param initialState Initial state vector
     * @param forceModel Force model to compute accelerations
     * @param targetTime Target time to integrate to
     * @param callback Optional callback for intermediate results
     * @return Final state vector at target time
     * @throws std::runtime_error if integration fails
     * @throws std::invalid_argument if inputs are invalid
     */
    StateVector integrate(
        const StateVector& initialState,
        forces::ForceAggregator& forceModel,
        double targetTime,
        StepCallback callback = nullptr) override;
    
    /**
     * @brief Perform a single RK78 integration step
     * 
     * Executes one step of the RK78 algorithm with embedded error estimation.
     * The step may be rejected if the error exceeds tolerances, in which case
     * stepAccepted will be false in the result.
     * 
     * @param currentState Current state vector
     * @param forceModel Force model to compute accelerations
     * @param requestedStepSize Requested time step size
     * @return Step result with new state and error estimate
     */
    StepResult step(
        const StateVector& currentState,
        forces::ForceAggregator& forceModel,
        double requestedStepSize) override;
    
    /**
     * @brief Get integrator type name
     * @return "RK78"
     */
    std::string getType() const override { return "RK78"; }
    
    /**
     * @brief Check if this is an adaptive integrator
     * @return true (RK78 is adaptive)
     */
    bool isAdaptive() const override { return true; }
    
    /**
     * @brief Get the order of the integration method
     * @return 7 (seventh-order accurate with eighth-order error estimate)
     */
    int getOrder() const override { return 7; }
};

} // namespace integrators
} // namespace physics
} // namespace iloss