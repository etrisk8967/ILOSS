#include "physics/aerodynamics/AerodynamicForceModel.h"
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace iloss {
namespace physics {
namespace aerodynamics {

AerodynamicForceModel::AerodynamicForceModel(
    std::shared_ptr<constants::AtmosphericModel> atmosphereModel,
    std::shared_ptr<AerodynamicDatabase> coefficientDatabase,
    const AerodynamicConfig& config)
    : IAttitudeAwareForceModel("AerodynamicForce", forces::ForceModelType::Aerodynamic)
    , m_atmosphereModel(atmosphereModel)
    , m_coefficientDatabase(coefficientDatabase)
    , m_config(config) {
    
    if (!m_atmosphereModel) {
        throw std::invalid_argument("Atmosphere model cannot be null");
    }
    if (!m_coefficientDatabase) {
        throw std::invalid_argument("Coefficient database cannot be null");
    }
}

iloss::math::Vector3D AerodynamicForceModel::calculateAccelerationWithAttitude(
    const dynamics::DynamicsState& state,
    const time::Time& time) const {
    
    // Clear cached values
    m_lastFlowProperties.reset();
    m_lastCoefficients.reset();
    
    // Get position and velocity
    iloss::math::Vector3D position = state.getPosition();
    iloss::math::Vector3D velocityInertial = state.getVelocity();
    
    // Calculate altitude (assuming spherical Earth for now)
    double altitude = position.magnitude() - 6371000.0;  // Earth radius approximation
    
    // Get atmospheric properties
    // Convert position to ECEF (assuming position is in inertial frame)
    // For now, we'll use a simple approximation
    double julianDate = time.getJulianDate();
    double density = m_atmosphereModel->getDensity(position, julianDate);
    double temperature = m_atmosphereModel->getTemperature(position, julianDate);
    
    // Check for negligible atmosphere
    if (density < m_config.minimumDensity) {
        return iloss::math::Vector3D::zero();  // No aerodynamic forces
    }
    
    // Calculate flow properties
    FlowProperties flowProps = AerodynamicCalculator::calculateFlowProperties(
        velocityInertial,
        state.getAttitude(),
        density,
        AerodynamicCalculator::calculateSpeedOfSound(temperature)
    );
    
    // Store for analysis
    m_lastFlowProperties = flowProps;
    
    // Check for very low velocity
    if (flowProps.velocityBodyFrame.magnitude() < MIN_VELOCITY_FOR_AERO) {
        return iloss::math::Vector3D::zero();  // No aerodynamic forces at very low speed
    }
    
    // Get aerodynamic coefficients
    AerodynamicCoefficients coeffs = m_coefficientDatabase->getCoefficients(
        flowProps.mach,
        flowProps.angleOfAttack,
        m_config.coefficientConfig
    );
    
    // Store for analysis
    m_lastCoefficients = coeffs;
    
    // Apply dynamic pressure limiting if enabled
    double q = flowProps.dynamicPressure;
    if (m_config.enableDynamicPressureLimit) {
        q = limitDynamicPressure(q);
    }
    
    // Calculate forces in wind frame
    iloss::math::Vector3D forceWind = calculateWindForces(q, coeffs);
    
    // Transform to body frame
    iloss::math::Vector3D forceBody = AerodynamicCalculator::windToBody(
        forceWind,
        flowProps.angleOfAttack,
        flowProps.sideslipAngle
    );
    
    // Transform to inertial frame
    iloss::math::Vector3D forceInertial = state.bodyToInertial(forceBody);
    
    // Convert force to acceleration (F = ma, so a = F/m)
    double mass = state.getMass();
    if (mass <= 0.0) {
        throw std::runtime_error("Invalid mass for aerodynamic acceleration calculation");
    }
    
    return forceInertial / mass;
}

bool AerodynamicForceModel::initialize(const forces::ForceModelConfig& config) {
    // Set reference area
    if (config.hasParameter("reference_area")) {
        m_config.referenceArea = config.getParameter("reference_area");
    }
    
    // Set reference lengths
    if (config.hasParameter("reference_length")) {
        m_config.referenceLength = config.getParameter("reference_length");
    }
    if (config.hasParameter("reference_span")) {
        m_config.referenceSpan = config.getParameter("reference_span");
    }
    
    // Set center of pressure offset
    if (config.hasParameter("cop_offset_x")) {
        m_config.centerOfPressureOffset.setX(config.getParameter("cop_offset_x"));
    }
    if (config.hasParameter("cop_offset_y")) {
        m_config.centerOfPressureOffset.setY(config.getParameter("cop_offset_y"));
    }
    if (config.hasParameter("cop_offset_z")) {
        m_config.centerOfPressureOffset.setZ(config.getParameter("cop_offset_z"));
    }
    
    // Set coefficient configuration
    if (config.hasParameter("coefficient_config")) {
        m_config.coefficientConfig = config.getStringParameter("coefficient_config");
    }
    
    // Set dynamic pressure limiting
    if (config.hasParameter("enable_q_limit")) {
        m_config.enableDynamicPressureLimit = config.getBoolParameter("enable_q_limit");
    }
    if (config.hasParameter("max_q")) {
        m_config.maxDynamicPressure = config.getParameter("max_q");
    }
    
    // Set minimum density
    if (config.hasParameter("min_density")) {
        m_config.minimumDensity = config.getParameter("min_density");
    }
    
    // Update ForceModel configuration
    setEnabled(config.getBoolParameter("enabled", true));
    
    return validate();
}

bool AerodynamicForceModel::validate() const {
    // Check configuration validity
    if (m_config.referenceArea <= 0.0) {
        return false;
    }
    if (m_config.referenceLength <= 0.0) {
        return false;
    }
    if (m_config.referenceSpan <= 0.0) {
        return false;
    }
    if (m_config.maxDynamicPressure <= 0.0) {
        return false;
    }
    if (m_config.minimumDensity < 0.0) {
        return false;
    }
    
    // Check that coefficient configuration exists
    if (!m_coefficientDatabase->hasConfiguration(m_config.coefficientConfig)) {
        return false;
    }
    
    // Check atmosphere model is valid
    if (!m_atmosphereModel) {
        return false;
    }
    
    return true;
}

void AerodynamicForceModel::update(const time::Time& time) {
    // Atmosphere models in this architecture don't have an update method
    // They calculate values on demand based on position and time
}

std::unique_ptr<forces::ForceModel> AerodynamicForceModel::clone() const {
    auto cloned = std::make_unique<AerodynamicForceModel>(
        m_atmosphereModel,  // Shared pointers are copied
        m_coefficientDatabase,
        m_config
    );
    
    // Copy base class properties
    cloned->setEnabled(isEnabled());
    
    return cloned;
}

void AerodynamicForceModel::setReferenceArea(double area) {
    if (area <= 0.0) {
        throw std::invalid_argument("Reference area must be positive");
    }
    m_config.referenceArea = area;
}

void AerodynamicForceModel::setConfig(const AerodynamicConfig& config) {
    // Validate new configuration
    if (config.referenceArea <= 0.0 || 
        config.referenceLength <= 0.0 || 
        config.referenceSpan <= 0.0) {
        throw std::invalid_argument("Reference dimensions must be positive");
    }
    if (config.maxDynamicPressure <= 0.0) {
        throw std::invalid_argument("Maximum dynamic pressure must be positive");
    }
    if (config.minimumDensity < 0.0) {
        throw std::invalid_argument("Minimum density cannot be negative");
    }
    
    m_config = config;
}

iloss::math::Vector3D AerodynamicForceModel::calculateBodyForces(
    const FlowProperties& flowProps,
    const AerodynamicCoefficients& coefficients) const {
    
    // Calculate forces in wind frame first
    iloss::math::Vector3D forceWind = calculateWindForces(
        flowProps.dynamicPressure, 
        coefficients
    );
    
    // Transform to body frame
    return AerodynamicCalculator::windToBody(
        forceWind,
        flowProps.angleOfAttack,
        flowProps.sideslipAngle
    );
}

iloss::math::Vector3D AerodynamicForceModel::calculateWindForces(
    double dynamicPressure,
    const AerodynamicCoefficients& coefficients) const {
    
    // Force = q * S * C
    // where q = dynamic pressure, S = reference area, C = coefficient
    
    double qS = dynamicPressure * m_config.referenceArea;
    
    // In wind frame:
    // X: Drag (opposite to velocity)
    // Y: Side force
    // Z: Lift
    
    return iloss::math::Vector3D(
        -qS * coefficients.getDragCoefficient(),   // Drag opposes velocity
        qS * coefficients.getSideForceCoefficient(),
        -qS * coefficients.getLiftCoefficient()     // Lift is negative Z in wind frame
    );
}

double AerodynamicForceModel::limitDynamicPressure(double q) const {
    if (m_config.enableDynamicPressureLimit) {
        return std::min(q, m_config.maxDynamicPressure);
    }
    return q;
}

} // namespace aerodynamics
} // namespace physics
} // namespace iloss