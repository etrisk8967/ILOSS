#include "physics/forces/propulsion/Engine.h"
#include "core/logging/Logger.h"
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace iloss {
namespace physics {
namespace propulsion {

using namespace iloss::logging;

Engine::Engine(const Configuration& config)
    : m_config(config)
    , m_state()
    , m_interpolation(ThrustCurveInterpolation::LINEAR) {
    
    validateConfiguration();
    
    // Calculate sea level values if not provided
    if (m_config.seaLevelThrust <= 0.0 && m_config.nozzleExitArea > 0.0) {
        // Approximate sea level thrust reduction due to back pressure
        m_config.seaLevelThrust = m_config.vacuumThrust - 
            STANDARD_ATMOSPHERE * m_config.nozzleExitArea;
        
        Logger::getInstance().logf(LogLevel::Debug, LogCategory::Physics,
            "Calculated sea level thrust for '{}': {:.0f} N",
            m_config.name, m_config.seaLevelThrust);
    }
    
    if (m_config.seaLevelIsp <= 0.0 && m_config.seaLevelThrust > 0.0) {
        // Approximate sea level Isp reduction (typically 10-15% lower)
        m_config.seaLevelIsp = m_config.vacuumIsp * 0.88;
        
        Logger::getInstance().logf(LogLevel::Debug, LogCategory::Physics,
            "Estimated sea level Isp for '{}': {:.1f} s",
            m_config.name, m_config.seaLevelIsp);
    }
    
    Logger::getInstance().logf(LogLevel::Info, LogCategory::Physics,
        "Created engine '{}': Thrust={:.0f}N (vac), Isp={:.0f}s (vac)",
        m_config.name, m_config.vacuumThrust, m_config.vacuumIsp);
}

Engine::Engine(const Configuration& config,
               const std::vector<ThrustPoint>& thrustCurve,
               ThrustCurveInterpolation interpolation)
    : m_config(config)
    , m_state()
    , m_thrustCurve(thrustCurve)
    , m_interpolation(interpolation) {
    
    validateConfiguration();
    validateThrustCurve();
    
    // Sort thrust curve by pressure for interpolation
    std::sort(m_thrustCurve.begin(), m_thrustCurve.end(),
        [](const ThrustPoint& a, const ThrustPoint& b) {
            return a.pressure < b.pressure;
        });
    
    Logger::getInstance().logf(LogLevel::Info, LogCategory::Physics,
        "Created engine '{}' with {} point thrust curve",
        m_config.name, m_thrustCurve.size());
}

void Engine::validateConfiguration() const {
    if (m_config.name.empty()) {
        throw std::invalid_argument("Engine name cannot be empty");
    }
    
    if (m_config.vacuumThrust <= 0.0) {
        throw std::invalid_argument("Vacuum thrust must be positive: " +
            std::to_string(m_config.vacuumThrust));
    }
    
    if (m_config.vacuumIsp <= 0.0) {
        throw std::invalid_argument("Vacuum Isp must be positive: " +
            std::to_string(m_config.vacuumIsp));
    }
    
    if (m_config.minThrottle < 0.0 || m_config.minThrottle > 1.0) {
        throw std::invalid_argument("Min throttle must be in [0,1]: " +
            std::to_string(m_config.minThrottle));
    }
    
    if (m_config.maxThrottle < 0.0 || m_config.maxThrottle > 1.0) {
        throw std::invalid_argument("Max throttle must be in [0,1]: " +
            std::to_string(m_config.maxThrottle));
    }
    
    if (m_config.minThrottle > m_config.maxThrottle) {
        throw std::invalid_argument("Min throttle cannot exceed max throttle");
    }
    
    if (m_config.nozzleExitArea < 0.0) {
        throw std::invalid_argument("Nozzle exit area cannot be negative: " +
            std::to_string(m_config.nozzleExitArea));
    }
    
    if (m_config.gimbalRange < 0.0) {
        throw std::invalid_argument("Gimbal range cannot be negative: " +
            std::to_string(m_config.gimbalRange));
    }
    
    if (m_config.gimbalRate < 0.0) {
        throw std::invalid_argument("Gimbal rate cannot be negative: " +
            std::to_string(m_config.gimbalRate));
    }
}

void Engine::validateThrustCurve() const {
    if (m_thrustCurve.empty()) {
        throw std::invalid_argument("Thrust curve cannot be empty");
    }
    
    for (const auto& point : m_thrustCurve) {
        if (point.pressure < 0.0) {
            throw std::invalid_argument("Thrust curve pressure cannot be negative");
        }
        if (point.thrust <= 0.0) {
            throw std::invalid_argument("Thrust curve thrust must be positive");
        }
        if (point.isp <= 0.0) {
            throw std::invalid_argument("Thrust curve Isp must be positive");
        }
    }
}

double Engine::getThrust(double ambientPressure) const {
    if (!m_state.isActive) {
        return 0.0;
    }
    
    double thrust;
    
    if (!m_thrustCurve.empty()) {
        // Use thrust curve
        double isp;
        interpolateThrustCurve(ambientPressure, thrust, isp);
        thrust *= m_state.throttleSetting;
    } else {
        // Use basic calculation
        thrust = calculateBasicThrust(ambientPressure, m_state.throttleSetting);
    }
    
    return thrust;
}

double Engine::getIsp(double ambientPressure) const {
    if (!m_state.isActive) {
        return 0.0;
    }
    
    if (!m_thrustCurve.empty()) {
        // Use thrust curve
        double thrust, isp;
        interpolateThrustCurve(ambientPressure, thrust, isp);
        return isp;
    } else {
        // Use basic calculation
        return calculateBasicIsp(ambientPressure);
    }
}

double Engine::getMassFlowRate(double ambientPressure) const {
    if (!m_state.isActive) {
        return 0.0;
    }
    
    double thrust = getThrust(ambientPressure);
    double isp = getIsp(ambientPressure);
    
    if (thrust > 0.0 && isp > 0.0) {
        // ṁ = F / (g₀ * Isp)
        return -thrust / (STANDARD_GRAVITY * isp);  // Negative for consumption
    }
    
    return 0.0;
}

double Engine::getExhaustVelocity(double ambientPressure) const {
    // v_e = g₀ * Isp
    return STANDARD_GRAVITY * getIsp(ambientPressure);
}

bool Engine::start(double initialThrottle) {
    if (m_state.isActive) {
        Logger::getInstance().logf(LogLevel::Warning, LogCategory::Physics,
            "Engine '{}' is already running", m_config.name);
        return true;
    }
    
    // Validate throttle setting
    double throttle = std::clamp(initialThrottle, 
        m_config.minThrottle, m_config.maxThrottle);
    
    if (throttle <= 0.0) {
        Logger::getInstance().logf(LogLevel::Error, LogCategory::Physics,
            "Cannot start engine '{}' with zero throttle", m_config.name);
        return false;
    }
    
    m_state.isActive = true;
    m_state.throttleSetting = throttle;
    
    Logger::getInstance().logf(LogLevel::Info, LogCategory::Physics,
        "Started engine '{}' at {:.0f}% throttle",
        m_config.name, throttle * 100.0);
    
    return true;
}

void Engine::shutdown() {
    if (!m_state.isActive) {
        return;
    }
    
    m_state.isActive = false;
    m_state.throttleSetting = 0.0;
    
    Logger::getInstance().logf(LogLevel::Info, LogCategory::Physics,
        "Shutdown engine '{}' after {:.1f}s burn time",
        m_config.name, m_state.totalBurnTime);
}

double Engine::setThrottle(double throttle) {
    if (!m_state.isActive) {
        Logger::getInstance().logf(LogLevel::Warning, LogCategory::Physics,
            "Cannot set throttle on inactive engine '{}'", m_config.name);
        return 0.0;
    }
    
    double clampedThrottle = std::clamp(throttle,
        m_config.minThrottle, m_config.maxThrottle);
    
    m_state.throttleSetting = clampedThrottle;
    
    if (clampedThrottle != throttle) {
        Logger::getInstance().logf(LogLevel::Debug, LogCategory::Physics,
            "Engine '{}' throttle clamped to {:.0f}%",
            m_config.name, clampedThrottle * 100.0);
    }
    
    return clampedThrottle;
}

bool Engine::setGimbalAngles(double pitch, double yaw) {
    if (!hasGimbal()) {
        if (pitch != 0.0 || yaw != 0.0) {
            Logger::getInstance().logf(LogLevel::Warning, LogCategory::Physics,
                "Engine '{}' has no gimbal capability", m_config.name);
            return false;
        }
        return true;
    }
    
    // Check if angles are within limits
    double totalAngle = std::sqrt(pitch * pitch + yaw * yaw);
    if (totalAngle > m_config.gimbalRange) {
        Logger::getInstance().logf(LogLevel::Warning, LogCategory::Physics,
            "Gimbal angles exceed range for engine '{}': {:.1f}° > {:.1f}°",
            m_config.name, 
            totalAngle * 180.0 / M_PI,
            m_config.gimbalRange * 180.0 / M_PI);
        
        // Scale angles to stay within range
        double scale = m_config.gimbalRange / totalAngle;
        pitch *= scale;
        yaw *= scale;
    }
    
    m_state.gimbalPitch = pitch;
    m_state.gimbalYaw = yaw;
    return true;
}

void Engine::updateStatistics(double deltaTime, double ambientPressure) {
    if (!m_state.isActive || deltaTime <= 0.0) {
        return;
    }
    
    m_state.totalBurnTime += deltaTime;
    
    double thrust = getThrust(ambientPressure);
    m_state.totalImpulse += thrust * deltaTime;
}

void Engine::resetStatistics() {
    m_state.totalBurnTime = 0.0;
    m_state.totalImpulse = 0.0;
}

std::string Engine::toString() const {
    std::ostringstream oss;
    oss << "Engine[" << m_config.name << "]: ";
    
    if (m_state.isActive) {
        oss << "ACTIVE @ " << std::fixed << std::setprecision(0)
            << (m_state.throttleSetting * 100.0) << "% throttle";
        if (hasGimbal() && (m_state.gimbalPitch != 0.0 || m_state.gimbalYaw != 0.0)) {
            oss << ", gimbal=(" 
                << std::setprecision(1)
                << (m_state.gimbalPitch * 180.0 / M_PI) << "°, "
                << (m_state.gimbalYaw * 180.0 / M_PI) << "°)";
        }
    } else {
        oss << "INACTIVE";
    }
    
    oss << " [" << std::setprecision(0)
        << m_config.vacuumThrust / 1000.0 << " kN, "
        << m_config.vacuumIsp << "s Isp]";
    
    if (m_state.totalBurnTime > 0.0) {
        oss << " Total: " << std::setprecision(1)
            << m_state.totalBurnTime << "s, "
            << std::setprecision(0)
            << m_state.totalImpulse / 1000.0 << " kN·s";
    }
    
    return oss.str();
}

std::unique_ptr<Engine> Engine::clone() const {
    if (!m_thrustCurve.empty()) {
        return std::make_unique<Engine>(m_config, m_thrustCurve, m_interpolation);
    } else {
        return std::make_unique<Engine>(m_config);
    }
}

bool Engine::validate() const {
    try {
        validateConfiguration();
        if (!m_thrustCurve.empty()) {
            validateThrustCurve();
        }
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

double Engine::getThrustCoefficient(double ambientPressure) const {
    // CF = F / (At * Pc)
    // This is a simplified calculation
    double thrust = getThrust(ambientPressure);
    if (thrust <= 0.0 || m_config.nozzleExitArea <= 0.0) {
        return 0.0;
    }
    
    // Estimate chamber pressure from thrust and nozzle area
    // This is approximate - real CF calculation requires chamber pressure
    double estimatedCF = 1.5;  // Typical value for modern engines
    return estimatedCF;
}

double Engine::calculateBasicThrust(double ambientPressure, double throttle) const {
    double thrust;
    
    if (ambientPressure <= 0.0) {
        // Vacuum conditions
        thrust = m_config.vacuumThrust;
    } else if (ambientPressure >= STANDARD_ATMOSPHERE) {
        // Sea level or higher pressure
        thrust = m_config.seaLevelThrust > 0.0 ? 
            m_config.seaLevelThrust : m_config.vacuumThrust;
    } else {
        // Interpolate between vacuum and sea level
        if (m_config.seaLevelThrust > 0.0) {
            double fraction = ambientPressure / STANDARD_ATMOSPHERE;
            thrust = lerp(0.0, m_config.vacuumThrust,
                         1.0, m_config.seaLevelThrust,
                         fraction);
        } else {
            // Use pressure correction if we have nozzle area
            if (m_config.nozzleExitArea > 0.0) {
                thrust = m_config.vacuumThrust - ambientPressure * m_config.nozzleExitArea;
            } else {
                thrust = m_config.vacuumThrust;
            }
        }
    }
    
    return thrust * throttle;
}

double Engine::calculateBasicIsp(double ambientPressure) const {
    if (ambientPressure <= 0.0) {
        // Vacuum conditions
        return m_config.vacuumIsp;
    } else if (ambientPressure >= STANDARD_ATMOSPHERE) {
        // Sea level or higher
        return m_config.seaLevelIsp > 0.0 ? 
            m_config.seaLevelIsp : m_config.vacuumIsp;
    } else {
        // Interpolate between vacuum and sea level
        if (m_config.seaLevelIsp > 0.0) {
            double fraction = ambientPressure / STANDARD_ATMOSPHERE;
            return lerp(0.0, m_config.vacuumIsp,
                       1.0, m_config.seaLevelIsp,
                       fraction);
        } else {
            // Estimate Isp reduction with pressure
            double ispReduction = 1.0 - 0.12 * (ambientPressure / STANDARD_ATMOSPHERE);
            return m_config.vacuumIsp * ispReduction;
        }
    }
}

void Engine::interpolateThrustCurve(double pressure, 
                                   double& thrust, double& isp) const {
    if (m_thrustCurve.empty()) {
        thrust = 0.0;
        isp = 0.0;
        return;
    }
    
    // Handle edge cases
    if (pressure <= m_thrustCurve.front().pressure) {
        thrust = m_thrustCurve.front().thrust;
        isp = m_thrustCurve.front().isp;
        return;
    }
    
    if (pressure >= m_thrustCurve.back().pressure) {
        thrust = m_thrustCurve.back().thrust;
        isp = m_thrustCurve.back().isp;
        return;
    }
    
    // Find interpolation points
    auto it = std::lower_bound(m_thrustCurve.begin(), m_thrustCurve.end(), pressure,
        [](const ThrustPoint& point, double p) {
            return point.pressure < p;
        });
    
    if (it == m_thrustCurve.begin()) {
        thrust = it->thrust;
        isp = it->isp;
        return;
    }
    
    auto prev = std::prev(it);
    
    // Interpolate based on method
    switch (m_interpolation) {
        case ThrustCurveInterpolation::LINEAR:
            thrust = lerp(prev->pressure, prev->thrust,
                         it->pressure, it->thrust, pressure);
            isp = lerp(prev->pressure, prev->isp,
                      it->pressure, it->isp, pressure);
            break;
            
        case ThrustCurveInterpolation::STEP:
            // Use nearest neighbor
            if (pressure - prev->pressure < it->pressure - pressure) {
                thrust = prev->thrust;
                isp = prev->isp;
            } else {
                thrust = it->thrust;
                isp = it->isp;
            }
            break;
            
        case ThrustCurveInterpolation::CUBIC_SPLINE:
            // TODO: Implement cubic spline interpolation
            // For now, fall back to linear
            thrust = lerp(prev->pressure, prev->thrust,
                         it->pressure, it->thrust, pressure);
            isp = lerp(prev->pressure, prev->isp,
                      it->pressure, it->isp, pressure);
            break;
    }
}

double Engine::lerp(double x0, double y0, double x1, double y1, double x) {
    if (x1 == x0) {
        return y0;
    }
    double t = (x - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
}

} // namespace propulsion
} // namespace physics
} // namespace iloss