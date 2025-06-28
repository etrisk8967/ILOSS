#include "physics/forces/SimpleGravityModel.h"
#include <cmath>
#include <sstream>

namespace iloss {
namespace physics {
namespace forces {

SimpleGravityModel::SimpleGravityModel(const std::string& name)
    : ForceModel(name, ForceModelType::TwoBody)
    , m_mu(EARTH_MU) {
}

math::Vector3D SimpleGravityModel::calculateAcceleration(
    const StateVector& state,
    const time::Time& /*time*/) const {
    
    // Get position vector
    const math::Vector3D& position = state.getPosition();
    
    // Calculate radius
    double r = position.magnitude();
    
    // Avoid singularity at origin
    if (r < 1.0) {  // Less than 1 meter from center
        return math::Vector3D::zero();
    }
    
    // Calculate acceleration: a = -mu/r³ * r_vec
    double factor = -m_mu / (r * r * r);
    
    return position * factor;
}

bool SimpleGravityModel::initialize(const ForceModelConfig& config) {
    m_config = config;
    
    // Get gravitational parameter if provided
    m_mu = config.getParameter<double>("mu", EARTH_MU);
    
    // Get central body name if provided (for logging)
    std::string centralBody = config.getParameter<std::string>("central_body", "Earth");
    
    return true;
}

bool SimpleGravityModel::validate() const {
    // Check that gravitational parameter is positive
    if (m_mu <= 0.0) {
        return false;
    }
    
    return true;
}

std::unique_ptr<ForceModel> SimpleGravityModel::clone() const {
    auto cloned = std::make_unique<SimpleGravityModel>(m_name);
    cloned->m_mu = m_mu;
    cloned->m_config = m_config;
    cloned->m_enabled = m_enabled;
    return cloned;
}

std::string SimpleGravityModel::toString() const {
    std::stringstream ss;
    ss << ForceModel::toString();
    ss << " [mu=" << std::scientific << m_mu << " m³/s²]";
    return ss.str();
}

} // namespace forces
} // namespace physics
} // namespace iloss