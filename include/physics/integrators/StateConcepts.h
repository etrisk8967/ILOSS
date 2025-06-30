#pragma once

#include <concepts>
#include <type_traits>
#include <Eigen/Core>
#include "core/coordinates/CoordinateSystem.h"

namespace iloss {
namespace physics {
namespace integrators {

/**
 * @brief Concept defining requirements for an integrable state type
 * 
 * Any state type that can be integrated must satisfy this concept.
 * This includes both StateVector (3-DOF) and DynamicsState (6-DOF).
 * 
 * Required operations:
 * - Default constructible
 * - Copy constructible and assignable
 * - Addition with another state
 * - Scalar multiplication
 * - getDimension() method returning state dimension
 * - getTime() and setTime() methods
 * - toVector() method returning Eigen vector representation
 * - fromVector() static method creating state from Eigen vector
 */
template<typename T>
concept IntegrableState = requires(T state, const T& other, double scalar, double time) {
    // Construction requirements
    { T() } -> std::same_as<T>;
    { T(other) } -> std::same_as<T>;
    
    // Arithmetic operations for integration
    { state + other } -> std::same_as<T>;
    { state * scalar } -> std::same_as<T>;
    { scalar * state } -> std::same_as<T>;
    
    // Dimension query
    { state.getDimension() } -> std::convertible_to<std::size_t>;
    
    // Time management
    { state.getTimeAsDouble() } -> std::convertible_to<double>;
    { state.setTime(time) } -> std::same_as<void>;
    
    // Vector conversion for generic algorithms
    { state.toVector() } -> std::convertible_to<Eigen::VectorXd>;
    { T::fromVector(state.toVector(), time) } -> std::same_as<T>;
};

/**
 * @brief Concept for state derivative providers
 * 
 * Any system that can provide derivatives for integration must satisfy this concept.
 * This includes ForceAggregator (for 3-DOF) and DynamicsEngine (for 6-DOF).
 */
template<typename System, typename State>
concept DerivativeProvider = requires(System system, const State& state) {
    // Must be able to compute derivative at given state
    { system.computeDerivative(state) } -> std::same_as<State>;
};

/**
 * @brief Trait to extract state derivative type
 * 
 * For most states, the derivative is the same type as the state.
 * This can be specialized for states that have different derivative types.
 */
template<typename State>
struct StateDerivativeTrait {
    using type = State;
};

template<typename State>
using StateDerivative = typename StateDerivativeTrait<State>::type;

/**
 * @brief Helper to check if a state type supports error estimation
 * 
 * States that support adaptive integration must provide error norm calculation.
 */
template<typename T>
concept ErrorEstimableState = IntegrableState<T> && requires(const T& state1, const T& state2) {
    { state1.errorNorm(state2) } -> std::convertible_to<double>;
};

/**
 * @brief Helper to check if a state type supports coordinate transformations
 * 
 * Some integrators may need to transform states between coordinate systems.
 */
template<typename T>
concept TransformableState = IntegrableState<T> && requires(T state, coordinates::CoordinateSystem from, coordinates::CoordinateSystem to) {
    { state.transformTo(from, to) } -> std::same_as<void>;
    { state.getCoordinateSystem() } -> std::convertible_to<coordinates::CoordinateSystem>;
};

} // namespace integrators
} // namespace physics
} // namespace iloss