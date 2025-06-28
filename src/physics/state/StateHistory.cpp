#include "physics/state/StateHistory.h"
#include <stdexcept>
#include <algorithm>
#include <cmath>

namespace iloss {
namespace physics {

StateHistory::StateHistory(size_t maxSize)
    : m_maxSize(maxSize)
{
}

void StateHistory::addState(const StateVector& state)
{
    if (!state.isValid()) {
        throw std::invalid_argument("Cannot add invalid state to history");
    }
    
    double timeKey = state.getTime().getTime();
    m_states.emplace(timeKey, state);
    
    enforceMaxSize();
}

void StateHistory::addStates(const std::vector<StateVector>& states)
{
    for (const auto& state : states) {
        addState(state);
    }
}

std::optional<StateVector> StateHistory::getStateAtTime(const time::Time& time) const
{
    double timeKey = time.getTime();
    auto it = m_states.find(timeKey);
    
    if (it != m_states.end()) {
        return it->second;
    }
    
    return std::nullopt;
}

StateVector StateHistory::getInterpolatedState(const time::Time& time, 
                                              const std::string& method) const
{
    if (m_states.empty()) {
        throw std::runtime_error("Cannot interpolate from empty history");
    }
    
    double timeKey = time.getTime();
    
    // Check if we have an exact match
    auto exact = getStateAtTime(time);
    if (exact.has_value()) {
        return exact.value();
    }
    
    // Find surrounding states
    auto upper = m_states.lower_bound(timeKey);
    
    // Check bounds
    if (upper == m_states.begin()) {
        throw std::runtime_error("Requested time is before earliest state in history");
    }
    if (upper == m_states.end()) {
        throw std::runtime_error("Requested time is after latest state in history");
    }
    
    if (method == "linear") {
        auto lower = std::prev(upper);
        return interpolateLinear(lower->second, upper->second, time);
    }
    else if (method == "cubic") {
        // For cubic interpolation, we need 4 points
        if (m_states.size() < 4) {
            // Fall back to linear
            auto lower = std::prev(upper);
            return interpolateLinear(lower->second, upper->second, time);
        }
        
        std::vector<StateVector> cubicStates;
        auto lower = std::prev(upper);
        
        // Get point before lower if possible
        if (lower != m_states.begin()) {
            cubicStates.push_back(std::prev(lower)->second);
        } else {
            cubicStates.push_back(lower->second);
        }
        
        cubicStates.push_back(lower->second);
        cubicStates.push_back(upper->second);
        
        // Get point after upper if possible
        auto afterUpper = std::next(upper);
        if (afterUpper != m_states.end()) {
            cubicStates.push_back(afterUpper->second);
        } else {
            cubicStates.push_back(upper->second);
        }
        
        return interpolateCubic(cubicStates, time);
    }
    else {
        throw std::invalid_argument("Unknown interpolation method: " + method);
    }
}

StateVector StateHistory::getNearestState(const time::Time& time) const
{
    if (m_states.empty()) {
        throw std::runtime_error("Cannot get nearest state from empty history");
    }
    
    double timeKey = time.getTime();
    
    // Find the first state >= time
    auto upper = m_states.lower_bound(timeKey);
    
    if (upper == m_states.begin()) {
        return upper->second;
    }
    if (upper == m_states.end()) {
        return std::prev(upper)->second;
    }
    
    // Check which is closer
    auto lower = std::prev(upper);
    double lowerDiff = std::abs(timeKey - lower->first);
    double upperDiff = std::abs(upper->first - timeKey);
    
    return (lowerDiff < upperDiff) ? lower->second : upper->second;
}

std::vector<StateVector> StateHistory::getStatesInRange(const time::Time& startTime,
                                                        const time::Time& endTime) const
{
    std::vector<StateVector> result;
    
    double startKey = startTime.getTime();
    double endKey = endTime.getTime();
    
    auto start = m_states.lower_bound(startKey);
    auto end = m_states.upper_bound(endKey);
    
    for (auto it = start; it != end; ++it) {
        result.push_back(it->second);
    }
    
    return result;
}

const StateVector& StateHistory::getEarliestState() const
{
    if (m_states.empty()) {
        throw std::runtime_error("History is empty");
    }
    return m_states.begin()->second;
}

const StateVector& StateHistory::getLatestState() const
{
    if (m_states.empty()) {
        throw std::runtime_error("History is empty");
    }
    return std::prev(m_states.end())->second;
}

std::vector<StateVector> StateHistory::getAllStates() const
{
    std::vector<StateVector> result;
    result.reserve(m_states.size());
    
    for (const auto& pair : m_states) {
        result.push_back(pair.second);
    }
    
    return result;
}

void StateHistory::clear()
{
    m_states.clear();
}

size_t StateHistory::removeStatesBefore(const time::Time& cutoffTime)
{
    double cutoffKey = cutoffTime.getTime();
    auto cutoff = m_states.lower_bound(cutoffKey);
    
    size_t removed = std::distance(m_states.begin(), cutoff);
    m_states.erase(m_states.begin(), cutoff);
    
    return removed;
}

size_t StateHistory::removeStatesAfter(const time::Time& cutoffTime)
{
    double cutoffKey = cutoffTime.getTime();
    auto cutoff = m_states.upper_bound(cutoffKey);
    
    size_t removed = std::distance(cutoff, m_states.end());
    m_states.erase(cutoff, m_states.end());
    
    return removed;
}

void StateHistory::setMaxSize(size_t maxSize)
{
    m_maxSize = maxSize;
    enforceMaxSize();
}

double StateHistory::getTimeSpan() const
{
    if (m_states.empty()) {
        return 0.0;
    }
    
    double earliest = m_states.begin()->first;
    double latest = std::prev(m_states.end())->first;
    
    return latest - earliest;
}

std::map<std::string, double> StateHistory::computeStatistics() const
{
    std::map<std::string, double> stats;
    
    if (m_states.empty()) {
        return stats;
    }
    
    // Basic statistics
    stats["count"] = static_cast<double>(m_states.size());
    stats["timespan_seconds"] = getTimeSpan();
    
    // Compute min/max/mean altitude
    double minAltitude = std::numeric_limits<double>::max();
    double maxAltitude = std::numeric_limits<double>::min();
    double sumAltitude = 0.0;
    
    // Compute min/max/mean speed
    double minSpeed = std::numeric_limits<double>::max();
    double maxSpeed = std::numeric_limits<double>::min();
    double sumSpeed = 0.0;
    
    // Compute total mass change
    double initialMass = m_states.begin()->second.getMass();
    double finalMass = std::prev(m_states.end())->second.getMass();
    
    for (const auto& pair : m_states) {
        const StateVector& state = pair.second;
        
        double altitude = state.getRadius() - 6371000.0; // Approximate Earth radius
        double speed = state.getSpeed();
        
        minAltitude = std::min(minAltitude, altitude);
        maxAltitude = std::max(maxAltitude, altitude);
        sumAltitude += altitude;
        
        minSpeed = std::min(minSpeed, speed);
        maxSpeed = std::max(maxSpeed, speed);
        sumSpeed += speed;
    }
    
    stats["min_altitude_m"] = minAltitude;
    stats["max_altitude_m"] = maxAltitude;
    stats["mean_altitude_m"] = sumAltitude / m_states.size();
    
    stats["min_speed_m/s"] = minSpeed;
    stats["max_speed_m/s"] = maxSpeed;
    stats["mean_speed_m/s"] = sumSpeed / m_states.size();
    
    stats["initial_mass_kg"] = initialMass;
    stats["final_mass_kg"] = finalMass;
    stats["mass_change_kg"] = finalMass - initialMass;
    
    return stats;
}

StateHistory StateHistory::resample(double dt, const std::string& method) const
{
    if (m_states.empty()) {
        return StateHistory(m_maxSize);
    }
    
    StateHistory resampled(m_maxSize);
    
    double startTime = m_states.begin()->first;
    double endTime = std::prev(m_states.end())->first;
    
    for (double t = startTime; t <= endTime; t += dt) {
        time::Time sampleTime(t);
        try {
            StateVector interpolated = getInterpolatedState(sampleTime, method);
            resampled.addState(interpolated);
        } catch (const std::exception&) {
            // Skip points that can't be interpolated
        }
    }
    
    return resampled;
}

std::vector<time::Time> StateHistory::findDiscontinuities(double positionTolerance,
                                                          double velocityTolerance) const
{
    std::vector<time::Time> discontinuities;
    
    if (m_states.size() < 2) {
        return discontinuities;
    }
    
    auto prev = m_states.begin();
    for (auto curr = std::next(prev); curr != m_states.end(); ++curr, ++prev) {
        const StateVector& prevState = prev->second;
        const StateVector& currState = curr->second;
        
        double posDiff = (currState.getPosition() - prevState.getPosition()).magnitude();
        double velDiff = (currState.getVelocity() - prevState.getVelocity()).magnitude();
        
        if (posDiff > positionTolerance || velDiff > velocityTolerance) {
            discontinuities.push_back(currState.getTime());
        }
    }
    
    return discontinuities;
}

void StateHistory::enforceMaxSize()
{
    if (m_maxSize > 0 && m_states.size() > m_maxSize) {
        size_t toRemove = m_states.size() - m_maxSize;
        auto removeEnd = m_states.begin();
        std::advance(removeEnd, toRemove);
        m_states.erase(m_states.begin(), removeEnd);
    }
}

StateVector StateHistory::interpolateLinear(const StateVector& state1,
                                           const StateVector& state2,
                                           const time::Time& time) const
{
    double t1 = state1.getTime().getTime();
    double t2 = state2.getTime().getTime();
    double t = time.getTime();
    
    if (std::abs(t2 - t1) < 1e-9) {
        return state1;  // States are at same time
    }
    
    double alpha = (t - t1) / (t2 - t1);
    return state1.interpolateLinear(state2, alpha);
}

StateVector StateHistory::interpolateCubic(const std::vector<StateVector>& states,
                                          const time::Time& time) const
{
    if (states.size() != 4) {
        throw std::invalid_argument("Cubic interpolation requires exactly 4 states");
    }
    
    // Extract times
    double t0 = states[0].getTime().getTime();
    double t1 = states[1].getTime().getTime();
    double t2 = states[2].getTime().getTime();
    double t3 = states[3].getTime().getTime();
    double t = time.getTime();
    
    // Normalize time to [0, 1] between t1 and t2
    double u = (t - t1) / (t2 - t1);
    
    // Catmull-Rom spline coefficients
    double u2 = u * u;
    double u3 = u2 * u;
    
    double c0 = -0.5 * u3 + u2 - 0.5 * u;
    double c1 = 1.5 * u3 - 2.5 * u2 + 1.0;
    double c2 = -1.5 * u3 + 2.0 * u2 + 0.5 * u;
    double c3 = 0.5 * u3 - 0.5 * u2;
    
    // Interpolate position
    math::Vector3D pos = states[0].getPosition() * c0 +
                        states[1].getPosition() * c1 +
                        states[2].getPosition() * c2 +
                        states[3].getPosition() * c3;
    
    // Interpolate velocity
    math::Vector3D vel = states[0].getVelocity() * c0 +
                        states[1].getVelocity() * c1 +
                        states[2].getVelocity() * c2 +
                        states[3].getVelocity() * c3;
    
    // Linear interpolation for mass (cubic doesn't make physical sense for mass)
    double mass = states[1].getMass() * (1.0 - u) + states[2].getMass() * u;
    
    return StateVector(pos, vel, mass, time, states[1].getCoordinateSystem());
}

} // namespace physics
} // namespace iloss