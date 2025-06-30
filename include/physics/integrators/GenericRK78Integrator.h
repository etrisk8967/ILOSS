#pragma once

#include "physics/integrators/GenericIntegrator.h"
#include "physics/integrators/StateConcepts.h"
#include <array>
#include <chrono>
#include <cmath>
#include <algorithm>

namespace iloss {
namespace physics {
namespace integrators {

/**
 * @brief Generic adaptive Runge-Kutta-Fehlberg 7(8) integrator
 * 
 * This template-based RK78 integrator works with any state type that satisfies
 * the IntegrableState concept, including both StateVector (3-DOF) and 
 * DynamicsState (6-DOF).
 * 
 * Key features:
 * - 7th order accuracy with 8th order error estimation
 * - Adaptive step size control based on error tolerance
 * - 13 function evaluations per step
 * - Excellent for high-precision propagation
 * - FSAL (First Same As Last) property for efficiency
 * 
 * This implementation uses the Fehlberg 7(8) coefficients, which are optimized
 * for minimal truncation error.
 * 
 * @tparam State State type that satisfies IntegrableState and ErrorEstimableState concepts
 * @tparam System System type that provides state derivatives
 */
template<typename State, typename System>
    requires IntegrableState<State> && ErrorEstimableState<State>
class GenericRK78Integrator : public GenericIntegratorBase<State, System> {
private:
    // Butcher tableau coefficients for Fehlberg 7(8)
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
    mutable std::array<State, STAGES> k_vectors;
    
    /**
     * @brief Compute weighted combination of states
     * 
     * @param baseState Base state
     * @param k Array of derivative states
     * @param weights Weight coefficients
     * @param stepSize Time step
     * @return Weighted combination
     */
    State computeWeightedCombination(
        const State& baseState,
        const std::array<State, STAGES>& k,
        const std::array<double, STAGES>& weights,
        double stepSize) const {
        
        State result = baseState;
        
        // Add weighted contributions
        for (int i = 0; i < STAGES; ++i) {
            if (std::abs(weights[i]) > std::numeric_limits<double>::epsilon()) {
                result = result + k[i] * (stepSize * weights[i]);
            }
        }
        
        result.setTime(baseState.getTime() + stepSize);
        return result;
    }
    
    /**
     * @brief Compute stage state using RK tableau
     * 
     * @param baseState Base state
     * @param k Array of derivative states computed so far
     * @param stage Current stage index
     * @param stepSize Time step
     * @return State at the given stage
     */
    State computeStageState(
        const State& baseState,
        const std::array<State, STAGES>& k,
        int stage,
        double stepSize) const {
        
        State stageState = baseState;
        
        // Add contributions from previous stages according to RK tableau
        for (int j = 0; j < stage; ++j) {
            if (std::abs(a[stage][j]) > std::numeric_limits<double>::epsilon()) {
                stageState = stageState + k[j] * (stepSize * a[stage][j]);
            }
        }
        
        stageState.setTime(baseState.getTime() + stepSize * c[stage]);
        return stageState;
    }
    
    /**
     * @brief Compute new step size based on error
     * 
     * @param currentStepSize Current step size
     * @param errorEstimate Estimated error
     * @param order Order of the method
     * @return Recommended new step size
     */
    double computeNewStepSize(
        double currentStepSize,
        double errorEstimate,
        int order) const {
        
        // Compute tolerance
        double tolerance = this->m_config.relativeTolerance;
        
        if (errorEstimate < std::numeric_limits<double>::epsilon()) {
            // Error is essentially zero, increase step size
            return currentStepSize * this->m_config.maxScaleFactor;
        }
        
        // Compute optimal step size using error control formula
        // h_new = h_old * (tolerance / error)^(1/(order+1))
        double errorRatio = tolerance / errorEstimate;
        double scaleFactor = this->m_config.safetyFactor * 
                            std::pow(errorRatio, 1.0 / (order + 1));
        
        // Apply limits to scale factor
        scaleFactor = std::max(this->m_config.minScaleFactor, 
                              std::min(this->m_config.maxScaleFactor, scaleFactor));
        
        double newStepSize = currentStepSize * scaleFactor;
        
        // Apply absolute limits
        newStepSize = std::max(this->m_config.minStepSize, 
                              std::min(this->m_config.maxStepSize, newStepSize));
        
        return newStepSize;
    }
    
public:
    /**
     * @brief Constructor with configuration
     * @param config Integrator configuration with error tolerances
     */
    explicit GenericRK78Integrator(const IntegratorConfig& config = IntegratorConfig())
        : GenericIntegratorBase<State, System>(config) {}
    
    /**
     * @brief Destructor
     */
    virtual ~GenericRK78Integrator() = default;
    
    /**
     * @brief Perform a single RK78 integration step
     * 
     * Executes one step of the RK78 algorithm with embedded error estimation.
     * The step may be rejected if the error exceeds tolerances.
     * 
     * @param currentState Current state
     * @param system System providing derivatives
     * @param requestedStepSize Requested time step size
     * @return Step result with new state and error estimate
     */
    typename IGenericIntegrator<State, System>::StepResult step(
        const State& currentState,
        System& system,
        double requestedStepSize) override {
        
        auto startTime = std::chrono::high_resolution_clock::now();
        
        const double h = requestedStepSize;
        
        // Compute derivatives at each stage
        for (int stage = 0; stage < STAGES; ++stage) {
            State stageState;
            
            if (stage == 0) {
                stageState = currentState;
            } else {
                stageState = computeStageState(currentState, k_vectors, stage, h);
            }
            
            k_vectors[stage] = this->computeDerivative(stageState, system);
        }
        
        // Compute 7th order solution
        State solution7 = computeWeightedCombination(currentState, k_vectors, b7, h);
        
        // Compute 8th order solution for error estimation
        State solution8 = computeWeightedCombination(currentState, k_vectors, b8, h);
        
        // Estimate error
        double errorEstimate = solution7.errorNorm(solution8);
        
        // Determine if step should be accepted
        bool stepAccepted = errorEstimate <= this->m_config.relativeTolerance;
        
        // Compute recommended step size for next iteration
        double newStepSize = computeNewStepSize(h, errorEstimate, 7);
        
        // Update statistics
        if (this->m_config.enableStatistics) {
            this->m_statistics.totalSteps++;
            
            if (stepAccepted) {
                this->m_statistics.acceptedSteps++;
                this->updateStepSizeStatistics(h);
                this->updateErrorStatistics(errorEstimate);
            } else {
                this->m_statistics.rejectedSteps++;
            }
            
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
        result.newState = solution7;  // Use 7th order solution
        result.actualStepSize = newStepSize;
        result.estimatedError = errorEstimate;
        result.stepAccepted = stepAccepted;
        
        return result;
    }
    
    /**
     * @brief Get integrator type name
     * @return "GenericRK78"
     */
    std::string getType() const override { 
        return "GenericRK78"; 
    }
    
    /**
     * @brief Check if this is an adaptive integrator
     * @return true (RK78 is adaptive)
     */
    bool isAdaptive() const override { 
        return true; 
    }
    
    /**
     * @brief Get the order of the integration method
     * @return 7 (seventh-order accurate with eighth-order error estimate)
     */
    int getOrder() const override { 
        return 7; 
    }
};

// Initialize the Runge-Kutta matrix coefficients (Fehlberg 7(8) tableau)
template<typename State, typename System>
    requires IntegrableState<State> && ErrorEstimableState<State>
const std::array<std::array<double, 12>, 13> GenericRK78Integrator<State, System>::a = {{
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

/**
 * @brief Type alias for RK78 integrator with StateVector
 */
using RK78StateVectorIntegrator = GenericRK78Integrator<StateVector, forces::ForceAggregator>;

/**
 * @brief Type alias for RK78 integrator with DynamicsState
 */
using RK78DynamicsIntegrator = GenericRK78Integrator<::iloss::physics::dynamics::DynamicsState, 
                                                     DynamicsIntegratorAdapter>;

} // namespace integrators
} // namespace physics
} // namespace iloss