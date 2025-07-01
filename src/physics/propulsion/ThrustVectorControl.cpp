#include "physics/forces/propulsion/ThrustVectorControl.h"
#include "core/logging/Logger.h"
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace iloss {
namespace physics {
namespace propulsion {

using namespace iloss::logging;
using namespace iloss::math;

ThrustVectorControl::ThrustVectorControl(const Configuration& config)
    : m_config(config)
    , m_state() {
    
    validateConfiguration();
    
    Logger::getInstance().logf(LogLevel::Info, LogCategory::Physics,
        "Created TVC system: pitch={}°, yaw={}°, rates={}°/s",
        m_config.enablePitch ? 
            std::to_string(m_config.maxPitchAngle * 180.0 / M_PI) : "disabled",
        m_config.enableYaw ? 
            std::to_string(m_config.maxYawAngle * 180.0 / M_PI) : "disabled",
        m_config.maxPitchRate * 180.0 / M_PI);
}

void ThrustVectorControl::validateConfiguration() const {
    if (m_config.maxPitchAngle < 0.0) {
        throw std::invalid_argument("Max pitch angle cannot be negative");
    }
    
    if (m_config.maxYawAngle < 0.0) {
        throw std::invalid_argument("Max yaw angle cannot be negative");
    }
    
    if (m_config.maxPitchRate <= 0.0) {
        throw std::invalid_argument("Max pitch rate must be positive");
    }
    
    if (m_config.maxYawRate <= 0.0) {
        throw std::invalid_argument("Max yaw rate must be positive");
    }
    
    if (m_config.actuatorDamping < 0.0 || m_config.actuatorDamping > 1.0) {
        throw std::invalid_argument("Actuator damping must be in [0,1]");
    }
    
    if (m_config.actuatorFrequency <= 0.0) {
        throw std::invalid_argument("Actuator frequency must be positive");
    }
}

bool ThrustVectorControl::setCommandedAngles(double pitch, double yaw) {
    bool withinLimits = true;
    
    // Handle pitch
    if (m_config.enablePitch) {
        if (std::abs(pitch) > m_config.maxPitchAngle) {
            withinLimits = false;
            m_state.commandedPitch = clampAngle(pitch, m_config.maxPitchAngle);
            
            Logger::getInstance().logf(LogLevel::Debug, LogCategory::Physics,
                "Pitch command {:.1f}° clamped to {:.1f}°",
                pitch * 180.0 / M_PI, 
                m_state.commandedPitch * 180.0 / M_PI);
        } else {
            m_state.commandedPitch = pitch;
        }
    } else {
        m_state.commandedPitch = 0.0;
        if (pitch != 0.0) {
            withinLimits = false;
            Logger::getInstance().logf(LogLevel::Debug, LogCategory::Physics,
                "Pitch command ignored (axis disabled)");
        }
    }
    
    // Handle yaw
    if (m_config.enableYaw) {
        if (std::abs(yaw) > m_config.maxYawAngle) {
            withinLimits = false;
            m_state.commandedYaw = clampAngle(yaw, m_config.maxYawAngle);
            
            Logger::getInstance().logf(LogLevel::Debug, LogCategory::Physics,
                "Yaw command {:.1f}° clamped to {:.1f}°",
                yaw * 180.0 / M_PI,
                m_state.commandedYaw * 180.0 / M_PI);
        } else {
            m_state.commandedYaw = yaw;
        }
    } else {
        m_state.commandedYaw = 0.0;
        if (yaw != 0.0) {
            withinLimits = false;
            Logger::getInstance().logf(LogLevel::Debug, LogCategory::Physics,
                "Yaw command ignored (axis disabled)");
        }
    }
    
    return withinLimits;
}

void ThrustVectorControl::update(double deltaTime) {
    if (deltaTime <= 0.0) {
        return;
    }
    
    // Update pitch
    if (m_config.enablePitch) {
        if (m_config.actuatorDamping < 1.0 && m_config.actuatorFrequency > 0.0) {
            // Use actuator dynamics model
            m_state.currentPitch = modelActuatorDynamics(
                m_state.currentPitch, m_state.commandedPitch,
                m_state.pitchRate, deltaTime);
        } else {
            // Simple rate limiting
            m_state.currentPitch = applyRateLimit(
                m_state.currentPitch, m_state.commandedPitch,
                m_config.maxPitchRate, deltaTime);
            m_state.pitchRate = (m_state.currentPitch - m_state.currentPitch) / deltaTime;
        }
    }
    
    // Update yaw
    if (m_config.enableYaw) {
        if (m_config.actuatorDamping < 1.0 && m_config.actuatorFrequency > 0.0) {
            // Use actuator dynamics model
            m_state.currentYaw = modelActuatorDynamics(
                m_state.currentYaw, m_state.commandedYaw,
                m_state.yawRate, deltaTime);
        } else {
            // Simple rate limiting
            m_state.currentYaw = applyRateLimit(
                m_state.currentYaw, m_state.commandedYaw,
                m_config.maxYawRate, deltaTime);
            m_state.yawRate = (m_state.currentYaw - m_state.currentYaw) / deltaTime;
        }
    }
}

Vector3D ThrustVectorControl::transformThrustVector(const Vector3D& nominalThrust) const {
    if (!isActive() || 
        (m_state.currentPitch == 0.0 && m_state.currentYaw == 0.0)) {
        // No transformation needed
        return nominalThrust;
    }
    
    // Get rotation matrix and apply to thrust vector
    Matrix3D rotation = getGimbalRotationMatrix();
    return rotation * nominalThrust;
}

Matrix3D ThrustVectorControl::getGimbalRotationMatrix() const {
    // Build rotation matrix for gimbal angles
    // Order: Yaw first, then pitch (typical for rocket engines)
    
    double cp = std::cos(m_state.currentPitch);
    double sp = std::sin(m_state.currentPitch);
    double cy = std::cos(m_state.currentYaw);
    double sy = std::sin(m_state.currentYaw);
    
    // Combined rotation matrix: R = R_pitch * R_yaw
    Matrix3D rotation;
    rotation(0, 0) = cy;
    rotation(0, 1) = 0.0;
    rotation(0, 2) = sy;
    
    rotation(1, 0) = sp * sy;
    rotation(1, 1) = cp;
    rotation(1, 2) = -sp * cy;
    
    rotation(2, 0) = -cp * sy;
    rotation(2, 1) = sp;
    rotation(2, 2) = cp * cy;
    
    return rotation;
}

bool ThrustVectorControl::calculateRequiredAngles(const Vector3D& desiredDirection,
                                                  double& pitch, double& yaw) const {
    // Normalize the desired direction
    Vector3D direction = desiredDirection.normalized();
    
    // The nominal thrust direction is along -Z axis
    // We need to find pitch and yaw to rotate -Z to match direction
    
    // Extract angles from the direction vector
    // Assuming small angle approximations for typical gimbal ranges
    
    // For small angles:
    // direction.x() ≈ -yaw (for thrust along -Z)
    // direction.y() ≈ pitch
    // direction.z() ≈ -1 (mostly along -Z)
    
    if (direction.z() > -0.5) {
        // Direction is too far from -Z axis
        Logger::getInstance().logf(LogLevel::Warning, LogCategory::Physics,
            "Desired thrust direction too far from nominal");
        return false;
    }
    
    // Calculate required angles
    yaw = -std::asin(std::clamp(direction.x(), -1.0, 1.0));
    pitch = std::asin(std::clamp(direction.y() / std::cos(yaw), -1.0, 1.0));
    
    // Check if angles are within limits
    bool achievable = true;
    
    if (m_config.enablePitch && std::abs(pitch) > m_config.maxPitchAngle) {
        pitch = clampAngle(pitch, m_config.maxPitchAngle);
        achievable = false;
    }
    
    if (m_config.enableYaw && std::abs(yaw) > m_config.maxYawAngle) {
        yaw = clampAngle(yaw, m_config.maxYawAngle);
        achievable = false;
    }
    
    if (!m_config.enablePitch) pitch = 0.0;
    if (!m_config.enableYaw) yaw = 0.0;
    
    return achievable;
}

bool ThrustVectorControl::isAtCommandedPosition(double tolerance) const {
    if (m_config.enablePitch) {
        if (std::abs(m_state.currentPitch - m_state.commandedPitch) > tolerance) {
            return false;
        }
    }
    
    if (m_config.enableYaw) {
        if (std::abs(m_state.currentYaw - m_state.commandedYaw) > tolerance) {
            return false;
        }
    }
    
    return true;
}

double ThrustVectorControl::getMaxDeflection() const {
    // Calculate maximum combined deflection angle
    double maxPitch = m_config.enablePitch ? m_config.maxPitchAngle : 0.0;
    double maxYaw = m_config.enableYaw ? m_config.maxYawAngle : 0.0;
    
    // Maximum occurs at corner of gimbal envelope
    return std::sqrt(maxPitch * maxPitch + maxYaw * maxYaw);
}

void ThrustVectorControl::reset(bool immediate) {
    m_state.commandedPitch = 0.0;
    m_state.commandedYaw = 0.0;
    
    if (immediate) {
        m_state.currentPitch = 0.0;
        m_state.currentYaw = 0.0;
        m_state.pitchRate = 0.0;
        m_state.yawRate = 0.0;
        
        Logger::getInstance().logf(LogLevel::Debug, LogCategory::Physics,
            "TVC reset to neutral (immediate)");
    } else {
        Logger::getInstance().logf(LogLevel::Debug, LogCategory::Physics,
            "TVC commanded to neutral");
    }
}

double ThrustVectorControl::getGimbalAuthority() const {
    // Gimbal authority is approximately sin(max_deflection)
    // For small angles, this is approximately the angle in radians
    double maxDeflection = getMaxDeflection();
    return std::sin(maxDeflection);
}

bool ThrustVectorControl::validate() const {
    try {
        validateConfiguration();
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

std::string ThrustVectorControl::toString() const {
    std::ostringstream oss;
    oss << "TVC[";
    
    if (!isActive()) {
        oss << "DISABLED";
    } else {
        oss << "current=(" << std::fixed << std::setprecision(1)
            << m_state.currentPitch * 180.0 / M_PI << "°, "
            << m_state.currentYaw * 180.0 / M_PI << "°)";
        
        if (!isAtCommandedPosition()) {
            oss << " → (" 
                << m_state.commandedPitch * 180.0 / M_PI << "°, "
                << m_state.commandedYaw * 180.0 / M_PI << "°)";
        }
        
        oss << ", authority=" << std::setprecision(1) 
            << getGimbalAuthority() * 100.0 << "%";
    }
    
    oss << "]";
    return oss.str();
}

std::unique_ptr<ThrustVectorControl> ThrustVectorControl::clone() const {
    auto clone = std::make_unique<ThrustVectorControl>(m_config);
    clone->m_state = m_state;
    return clone;
}

double ThrustVectorControl::applyRateLimit(double current, double commanded,
                                          double maxRate, double deltaTime) const {
    double error = commanded - current;
    double maxChange = maxRate * deltaTime;
    
    if (std::abs(error) <= maxChange) {
        // Can reach commanded position in this timestep
        return commanded;
    } else {
        // Apply rate limit
        return current + std::copysign(maxChange, error);
    }
}

double ThrustVectorControl::modelActuatorDynamics(double current, double commanded,
                                                  double& currentRate, double deltaTime) const {
    // Simple second-order actuator model
    // ẍ + 2ζωₙẋ + ωₙ²x = ωₙ²u
    
    double omega_n = 2.0 * M_PI * m_config.actuatorFrequency;  // Natural frequency
    double zeta = m_config.actuatorDamping;                    // Damping ratio
    
    // Calculate acceleration
    double error = commanded - current;
    double acceleration = omega_n * omega_n * error - 
                         2.0 * zeta * omega_n * currentRate;
    
    // Limit acceleration to respect rate limits
    double maxAccel = m_config.maxPitchRate / deltaTime;  // Approximate
    acceleration = std::clamp(acceleration, -maxAccel, maxAccel);
    
    // Update rate and position
    currentRate += acceleration * deltaTime;
    
    // Apply rate limits
    double maxRate = (current == m_state.currentPitch) ? 
        m_config.maxPitchRate : m_config.maxYawRate;
    currentRate = std::clamp(currentRate, -maxRate, maxRate);
    
    // Update position
    double newPosition = current + currentRate * deltaTime;
    
    // Apply position limits
    double maxAngle = (current == m_state.currentPitch) ? 
        m_config.maxPitchAngle : m_config.maxYawAngle;
    return clampAngle(newPosition, maxAngle);
}

double ThrustVectorControl::clampAngle(double angle, double limit) {
    return std::clamp(angle, -limit, limit);
}

} // namespace propulsion
} // namespace physics
} // namespace iloss