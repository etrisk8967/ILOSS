#include "physics/aerodynamics/AerodynamicTorqueModel.h"
#include <cmath>
#include <stdexcept>
#include <any>

namespace iloss {
namespace physics {
namespace aerodynamics {

AerodynamicTorqueModel::AerodynamicTorqueModel(
    std::shared_ptr<constants::AtmosphericModel> atmosphereModel,
    std::shared_ptr<AerodynamicDatabase> coefficientDatabase,
    const AerodynamicTorqueConfig& config)
    : m_atmosphereModel(atmosphereModel)
    , m_coefficientDatabase(coefficientDatabase)
    , m_config(config) {
    
    if (!m_atmosphereModel) {
        throw std::invalid_argument("Atmosphere model cannot be null");
    }
    if (!m_coefficientDatabase) {
        throw std::invalid_argument("Coefficient database cannot be null");
    }
}

AerodynamicTorqueModel::AerodynamicTorqueModel(
    const AerodynamicForceModel& forceModel,
    const AerodynamicTorqueConfig& config)
    : m_config(config) {
    
    // Extract shared resources from force model
    // Note: This assumes the force model exposes these or we need to add getters
    // For now, we'll throw an exception as we need to modify the force model
    throw std::runtime_error("Shared resource constructor not yet implemented - need getters in AerodynamicForceModel");
}

iloss::math::Vector3D AerodynamicTorqueModel::calculateTorque(
    const dynamics::DynamicsState& state, double time) const {
    
    if (!m_enabled) {
        return iloss::math::Vector3D::zero();
    }
    
    // Get position and velocity
    iloss::math::Vector3D position = state.getPosition();
    iloss::math::Vector3D velocityInertial = state.getVelocity();
    
    // Calculate altitude (assuming spherical Earth for now)
    double altitude = position.magnitude() - 6371000.0;  // Earth radius approximation
    
    // Get atmospheric properties
    double julianDate = time;
    double density = m_atmosphereModel->getDensity(position, julianDate);
    double temperature = m_atmosphereModel->getTemperature(position, julianDate);
    
    // Check for negligible atmosphere
    if (density < m_config.minimumDensity) {
        return iloss::math::Vector3D::zero();  // No aerodynamic torques
    }
    
    // Calculate flow properties
    FlowProperties flowProps = AerodynamicCalculator::calculateFlowProperties(
        velocityInertial,
        state.getAttitude(),
        density,
        AerodynamicCalculator::calculateSpeedOfSound(temperature)
    );
    
    // Check for very low velocity
    if (flowProps.velocityBodyFrame.magnitude() < MIN_VELOCITY_FOR_AERO) {
        return iloss::math::Vector3D::zero();  // No aerodynamic torques at very low speed
    }
    
    // Get aerodynamic coefficients
    AerodynamicCoefficients coeffs = m_coefficientDatabase->getCoefficients(
        flowProps.mach,
        flowProps.angleOfAttack,
        m_config.coefficientConfig
    );
    
    // Calculate torque from moment coefficients
    iloss::math::Vector3D torqueFromCoeffs = calculateCoefficientTorque(
        flowProps.dynamicPressure, 
        coeffs
    );
    
    // Calculate aerodynamic forces for offset torque calculation
    double qS = flowProps.dynamicPressure * m_config.referenceArea;
    
    // Forces in wind frame
    iloss::math::Vector3D forceWind(
        -qS * coeffs.getDragCoefficient(),
        qS * coeffs.getSideForceCoefficient(),
        -qS * coeffs.getLiftCoefficient()
    );
    
    // Transform to body frame
    iloss::math::Vector3D forceBody = AerodynamicCalculator::windToBody(
        forceWind,
        flowProps.angleOfAttack,
        flowProps.sideslipAngle
    );
    
    // Determine center of pressure
    iloss::math::Vector3D cop = m_config.centerOfPressure;
    
    if (m_config.useDynamicCenterOfPressure) {
        // Calculate COP from moment coefficients
        // For pitch: X_cp = X_ref - (Cm * c) / Cn
        double cn = coeffs.getLiftCoefficient() * std::cos(flowProps.angleOfAttack) +
                   coeffs.getDragCoefficient() * std::sin(flowProps.angleOfAttack);
        
        if (std::abs(cn) > 1e-6) {
            double xcpFromNose = calculateCenterOfPressure(
                coeffs.getPitchMomentCoefficient(),
                cn,
                0.25 * m_config.referenceLength  // Typical reference at quarter chord
            );
            cop.setX(xcpFromNose);
        }
    }
    
    // Calculate torque from force offset
    iloss::math::Vector3D torqueFromOffset = calculateOffsetTorque(forceBody, cop);
    
    // Total torque is sum of both contributions
    return torqueFromCoeffs + torqueFromOffset;
}

void AerodynamicTorqueModel::configure(const dynamics::TorqueModelConfig& config) {
    try {
        m_config = std::any_cast<AerodynamicTorqueConfig>(config);
    } catch (const std::bad_any_cast& e) {
        throw std::invalid_argument("Invalid configuration type for AerodynamicTorqueModel");
    }
}

std::unique_ptr<dynamics::ITorqueModel> AerodynamicTorqueModel::clone() const {
    auto cloned = std::make_unique<AerodynamicTorqueModel>(
        m_atmosphereModel,
        m_coefficientDatabase,
        m_config
    );
    cloned->setEnabled(m_enabled);
    return cloned;
}

void AerodynamicTorqueModel::setConfig(const AerodynamicTorqueConfig& config) {
    // Validate configuration
    if (config.referenceArea <= 0.0 || 
        config.referenceLength <= 0.0 || 
        config.referenceSpan <= 0.0) {
        throw std::invalid_argument("Reference dimensions must be positive");
    }
    if (config.minimumDensity < 0.0) {
        throw std::invalid_argument("Minimum density cannot be negative");
    }
    
    m_config = config;
}

double AerodynamicTorqueModel::calculateCenterOfPressure(
    double cm, double cn, double referencePoint) const {
    
    if (std::abs(cn) < 1e-10) {
        // No normal force - COP is undefined
        return referencePoint * m_config.referenceLength;
    }
    
    // Center of pressure from moment balance:
    // Cm = (Xcp - Xref) * Cn / c
    // Therefore: Xcp = Xref - (Cm * c) / Cn
    
    return referencePoint * m_config.referenceLength - 
           (cm * m_config.referenceLength) / cn;
}

iloss::math::Vector3D AerodynamicTorqueModel::calculateCoefficientTorque(
    double dynamicPressure,
    const AerodynamicCoefficients& coefficients) const {
    
    // Moment = q * S * L * C
    // where q = dynamic pressure, S = reference area, 
    // L = reference length (or span for lateral moments), C = moment coefficient
    
    double qS = dynamicPressure * m_config.referenceArea;
    
    // Roll moment uses span as reference
    double rollMoment = qS * m_config.referenceSpan * 
                       coefficients.getRollMomentCoefficient();
    
    // Pitch moment uses length as reference
    double pitchMoment = qS * m_config.referenceLength * 
                        coefficients.getPitchMomentCoefficient();
    
    // Yaw moment uses span as reference
    double yawMoment = qS * m_config.referenceSpan * 
                      coefficients.getYawMomentCoefficient();
    
    return iloss::math::Vector3D(rollMoment, pitchMoment, yawMoment);
}

iloss::math::Vector3D AerodynamicTorqueModel::calculateOffsetTorque(
    const iloss::math::Vector3D& force,
    const iloss::math::Vector3D& centerOfPressure) const {
    
    // Torque = r × F
    // where r is the vector from COM to COP
    iloss::math::Vector3D r = centerOfPressure - m_config.centerOfMass;
    return r.cross(force);
}

} // namespace aerodynamics
} // namespace physics
} // namespace iloss