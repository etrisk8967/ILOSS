#pragma once

#include "physics/state/StateVector.h"
#include <vector>
#include <deque>
#include <map>
#include <algorithm>
#include <limits>

namespace iloss {
namespace physics {

/**
 * @brief Manages a time-ordered history of state vectors
 * 
 * This class provides efficient storage and retrieval of state vectors
 * over time, with support for interpolation and bounded memory usage.
 * States are automatically sorted by time for quick access.
 */
class StateHistory {
public:
    /**
     * @brief Default constructor
     * @param maxSize Maximum number of states to store (0 = unlimited)
     */
    explicit StateHistory(size_t maxSize = 0);

    /**
     * @brief Copy constructor
     */
    StateHistory(const StateHistory& other) = default;

    /**
     * @brief Move constructor
     */
    StateHistory(StateHistory&& other) = default;

    /**
     * @brief Copy assignment operator
     */
    StateHistory& operator=(const StateHistory& other) = default;

    /**
     * @brief Move assignment operator
     */
    StateHistory& operator=(StateHistory&& other) = default;

    /**
     * @brief Destructor
     */
    ~StateHistory() = default;

    /**
     * @brief Add a state to the history
     * @param state State vector to add
     * @note If maxSize is set and exceeded, oldest states are removed
     */
    void addState(const StateVector& state);

    /**
     * @brief Add multiple states at once
     * @param states Vector of states to add
     */
    void addStates(const std::vector<StateVector>& states);

    /**
     * @brief Get state at exact time
     * @param time Time to query
     * @return State vector if found, nullopt otherwise
     */
    std::optional<StateVector> getStateAtTime(const time::Time& time) const;

    /**
     * @brief Get state by interpolation at given time
     * @param time Time to query
     * @param method Interpolation method (linear or cubic)
     * @return Interpolated state vector
     * @throws std::runtime_error if time is outside history bounds
     */
    StateVector getInterpolatedState(const time::Time& time, 
                                    const std::string& method = "linear") const;

    /**
     * @brief Get the nearest state to a given time
     * @param time Time to query
     * @return Nearest state vector
     * @throws std::runtime_error if history is empty
     */
    StateVector getNearestState(const time::Time& time) const;

    /**
     * @brief Get states within a time range
     * @param startTime Start of time range
     * @param endTime End of time range
     * @return Vector of states within the range
     */
    std::vector<StateVector> getStatesInRange(const time::Time& startTime,
                                             const time::Time& endTime) const;

    /**
     * @brief Get the earliest state in history
     * @return Earliest state
     * @throws std::runtime_error if history is empty
     */
    const StateVector& getEarliestState() const;

    /**
     * @brief Get the latest state in history
     * @return Latest state
     * @throws std::runtime_error if history is empty
     */
    const StateVector& getLatestState() const;

    /**
     * @brief Get all states in chronological order
     * @return Vector of all states
     */
    std::vector<StateVector> getAllStates() const;

    /**
     * @brief Clear all states from history
     */
    void clear();

    /**
     * @brief Remove states older than a given time
     * @param cutoffTime Time before which to remove states
     * @return Number of states removed
     */
    size_t removeStatesBefore(const time::Time& cutoffTime);

    /**
     * @brief Remove states newer than a given time
     * @param cutoffTime Time after which to remove states
     * @return Number of states removed
     */
    size_t removeStatesAfter(const time::Time& cutoffTime);

    /**
     * @brief Get the number of states in history
     * @return Number of states
     */
    size_t size() const { return m_states.size(); }

    /**
     * @brief Check if history is empty
     * @return True if no states are stored
     */
    bool empty() const { return m_states.empty(); }

    /**
     * @brief Get the maximum size limit
     * @return Maximum size (0 if unlimited)
     */
    size_t getMaxSize() const { return m_maxSize; }

    /**
     * @brief Set the maximum size limit
     * @param maxSize New maximum size (0 = unlimited)
     * @note If current size exceeds new limit, oldest states are removed
     */
    void setMaxSize(size_t maxSize);

    /**
     * @brief Get time span of history
     * @return Time difference between earliest and latest states in seconds
     */
    double getTimeSpan() const;

    /**
     * @brief Compute statistics over the history
     * @return Map of statistic names to values
     */
    std::map<std::string, double> computeStatistics() const;

    /**
     * @brief Resample history at regular intervals
     * @param dt Time step in seconds
     * @param method Interpolation method
     * @return New StateHistory with resampled states
     */
    StateHistory resample(double dt, const std::string& method = "linear") const;

    /**
     * @brief Find state discontinuities
     * @param positionTolerance Position jump threshold in meters
     * @param velocityTolerance Velocity jump threshold in m/s
     * @return Times where discontinuities occur
     */
    std::vector<time::Time> findDiscontinuities(double positionTolerance = 1000.0,
                                                double velocityTolerance = 100.0) const;

private:
    /**
     * @brief Internal storage using time as key for fast lookup
     * Using multimap to handle potential duplicate times
     */
    std::multimap<double, StateVector> m_states;
    
    /**
     * @brief Maximum number of states to store (0 = unlimited)
     */
    size_t m_maxSize;

    /**
     * @brief Enforce size limit by removing oldest states
     */
    void enforceMaxSize();

    /**
     * @brief Perform linear interpolation between states
     * @param state1 First state
     * @param state2 Second state
     * @param time Target time
     * @return Interpolated state
     */
    StateVector interpolateLinear(const StateVector& state1,
                                 const StateVector& state2,
                                 const time::Time& time) const;

    /**
     * @brief Perform cubic interpolation between states
     * @param states Four states for cubic interpolation
     * @param time Target time
     * @return Interpolated state
     */
    StateVector interpolateCubic(const std::vector<StateVector>& states,
                                const time::Time& time) const;
};

} // namespace physics
} // namespace iloss