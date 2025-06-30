#include "physics/forces/twobody/TwoBodyForceModel.h"
#include "core/logging/Logger.h"
#include <stdexcept>
#include <sstream>
#include <cmath>

namespace iloss {
namespace physics {
namespace forces {
namespace twobody {

using namespace iloss::logging;

// Static member initialization
const std::map<std::string, double> TwoBodyForceModel::BODY_MU_VALUES = {
    {"Earth", math::constants::EARTH_MU},
    {"Sun", math::constants::SUN_MU},
    {"Moon", math::constants::MOON_MU},
    {"Jupiter", math::constants::JUPITER_MU},
    {"Venus", math::constants::VENUS_MU},
    {"Mars", math::constants::MARS_MU}
};

TwoBodyForceModel::TwoBodyForceModel(const std::string& name)
    : ForceModel(name, ForceModelType::TwoBody),
      m_mu(math::constants::EARTH_MU),
      m_centralBody("Earth") {
    ILOSS_LOGF_INFO(Physics, "Created two-body force model '{}' with Earth parameters (μ = {} m³/s²)", 
                    getName(), m_mu);
}

TwoBodyForceModel::TwoBodyForceModel(const std::string& name, double mu)
    : ForceModel(name, ForceModelType::TwoBody),
      m_mu(mu),
      m_centralBody("Custom") {
    if (m_mu <= 0.0) {
        throw std::invalid_argument("Gravitational parameter must be positive");
    }
    ILOSS_LOGF_INFO(Physics, "Created two-body force model '{}' with μ = {} m³/s²", 
                    getName(), m_mu);
}

math::Vector3D TwoBodyForceModel::calculateAcceleration(
    const StateVector& state,
    const time::Time& time) const {
    
    // Get position vector
    const math::Vector3D& position = state.getPosition();
    
    // Calculate distance from origin
    double r = position.magnitude();
    
    // Check for singularity at origin
    if (r < math::constants::POSITION_TOLERANCE) {
        ILOSS_LOGF_ERROR(Physics, "TwoBodyForceModel: Position too close to origin: {} m", r);
        throw std::runtime_error("Two-body force calculation failed: position at origin");
    }
    
    // Calculate acceleration: a = -μ * r / |r|³
    double factor = -m_mu / (r * r * r);
    math::Vector3D acceleration = position * factor;
    
    ILOSS_LOGF_TRACE(Physics, "TwoBodyForceModel: Calculated acceleration at r={} km: a={} m/s²", 
                     r / 1000.0, acceleration.magnitude());
    
    return acceleration;
}

bool TwoBodyForceModel::initialize(const ForceModelConfig& config) {
    try {
        // Check if we have both mu and central_body - need special handling
        bool hasMu = config.hasParameter("mu");
        bool hasBody = config.hasParameter("central_body");
        
        if (hasBody) {
            std::string bodyName = config.getParameter<std::string>("central_body");
            m_centralBody = bodyName;
            
            // Only update mu from body if mu wasn't explicitly provided
            if (!hasMu) {
                auto it = BODY_MU_VALUES.find(bodyName);
                if (it != BODY_MU_VALUES.end()) {
                    m_mu = it->second;
                    ILOSS_LOGF_INFO(Physics, "Set central body to {} (μ = {} m³/s²)", 
                                    m_centralBody, m_mu);
                } else {
                    ILOSS_LOGF_WARN(Physics, "TwoBodyForceModel: Unknown central body '{}', keeping current μ value", 
                                    bodyName);
                }
            }
        }
        
        // Set custom mu if provided (overwrites body's default if both are given)
        if (hasMu) {
            m_mu = config.getParameter<double>("mu");
            ILOSS_LOGF_INFO(Physics, "Set custom gravitational parameter to {} m³/s²", m_mu);
        }
        
        // Validate configuration
        if (!validate()) {
            ILOSS_LOG_ERROR(Physics, "TwoBodyForceModel: Invalid configuration");
            return false;
        }
        
        return true;
        
    } catch (const std::exception& e) {
        ILOSS_LOGF_ERROR(Physics, "TwoBodyForceModel: Failed to initialize: {}", e.what());
        return false;
    }
}

bool TwoBodyForceModel::validate() const {
    if (m_mu <= 0.0) {
        ILOSS_LOGF_ERROR(Physics, "TwoBodyForceModel: Invalid gravitational parameter: {} m³/s²", m_mu);
        return false;
    }
    
    return true;
}

std::unique_ptr<ForceModel> TwoBodyForceModel::clone() const {
    auto cloned = std::make_unique<TwoBodyForceModel>(getName(), m_mu);
    cloned->m_centralBody = m_centralBody;
    cloned->setEnabled(isEnabled());
    return cloned;
}

std::string TwoBodyForceModel::toString() const {
    std::ostringstream oss;
    oss << "TwoBodyForceModel[name=" << getName()
        << ", central_body=" << m_centralBody
        << ", mu=" << m_mu << " m³/s²"
        << ", enabled=" << (isEnabled() ? "true" : "false")
        << "]";
    return oss.str();
}

void TwoBodyForceModel::setGravitationalParameter(double mu) {
    if (mu <= 0.0) {
        throw std::invalid_argument("Gravitational parameter must be positive");
    }
    m_mu = mu;
    ILOSS_LOGF_INFO(Physics, "Updated gravitational parameter to {} m³/s²", m_mu);
}

void TwoBodyForceModel::setCentralBody(const std::string& body) {
    auto it = BODY_MU_VALUES.find(body);
    if (it != BODY_MU_VALUES.end()) {
        m_centralBody = body;
        m_mu = it->second;
        ILOSS_LOGF_INFO(Physics, "Set central body to {} (μ = {} m³/s²)", 
                        m_centralBody, m_mu);
    } else {
        m_centralBody = body;
        ILOSS_LOGF_WARN(Physics, "TwoBodyForceModel: Unknown central body '{}', gravitational parameter unchanged", 
                        body);
    }
}

double TwoBodyForceModel::getBodyGravitationalParameter(const std::string& body) {
    auto it = BODY_MU_VALUES.find(body);
    if (it != BODY_MU_VALUES.end()) {
        return it->second;
    }
    
    throw std::invalid_argument("Unknown celestial body: " + body);
}

} // namespace twobody
} // namespace forces
} // namespace physics
} // namespace iloss