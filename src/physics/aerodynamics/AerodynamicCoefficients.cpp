#include "physics/aerodynamics/AerodynamicCoefficients.h"
#include <sstream>
#include <iomanip>
#include <cmath>

namespace iloss {
namespace physics {
namespace aerodynamics {

// Constants for validation
constexpr double MAX_REASONABLE_COEFFICIENT = 100.0;  // Most coefficients should be < 10
constexpr double EPSILON = 1e-10;

// AerodynamicCoefficients implementation

AerodynamicCoefficients::AerodynamicCoefficients()
    : m_cd(0.0), m_cl(0.0), m_cy(0.0), 
      m_croll(0.0), m_cpitch(0.0), m_cyaw(0.0) {
}

AerodynamicCoefficients::AerodynamicCoefficients(double cd, double cl, double cy, 
                                               double croll, double cpitch, double cyaw)
    : m_cd(cd), m_cl(cl), m_cy(cy), 
      m_croll(croll), m_cpitch(cpitch), m_cyaw(cyaw) {
    validateCoefficient(m_cd, "CD");
    validateCoefficient(m_cl, "CL");
    validateCoefficient(m_cy, "CY");
    validateCoefficient(m_croll, "Cl");
    validateCoefficient(m_cpitch, "Cm");
    validateCoefficient(m_cyaw, "Cn");
}

AerodynamicCoefficients::AerodynamicCoefficients(double cd, double cl, double cy)
    : m_cd(cd), m_cl(cl), m_cy(cy), 
      m_croll(0.0), m_cpitch(0.0), m_cyaw(0.0) {
    validateCoefficient(m_cd, "CD");
    validateCoefficient(m_cl, "CL");
    validateCoefficient(m_cy, "CY");
}

void AerodynamicCoefficients::setDragCoefficient(double cd) {
    validateCoefficient(cd, "CD");
    m_cd = cd;
}

void AerodynamicCoefficients::setLiftCoefficient(double cl) {
    validateCoefficient(cl, "CL");
    m_cl = cl;
}

void AerodynamicCoefficients::setSideForceCoefficient(double cy) {
    validateCoefficient(cy, "CY");
    m_cy = cy;
}

void AerodynamicCoefficients::setRollMomentCoefficient(double croll) {
    validateCoefficient(croll, "Cl");
    m_croll = croll;
}

void AerodynamicCoefficients::setPitchMomentCoefficient(double cpitch) {
    validateCoefficient(cpitch, "Cm");
    m_cpitch = cpitch;
}

void AerodynamicCoefficients::setYawMomentCoefficient(double cyaw) {
    validateCoefficient(cyaw, "Cn");
    m_cyaw = cyaw;
}

void AerodynamicCoefficients::setForceCoefficients(double cd, double cl, double cy) {
    validateCoefficient(cd, "CD");
    validateCoefficient(cl, "CL");
    validateCoefficient(cy, "CY");
    m_cd = cd;
    m_cl = cl;
    m_cy = cy;
}

void AerodynamicCoefficients::setMomentCoefficients(double croll, double cpitch, double cyaw) {
    validateCoefficient(croll, "Cl");
    validateCoefficient(cpitch, "Cm");
    validateCoefficient(cyaw, "Cn");
    m_croll = croll;
    m_cpitch = cpitch;
    m_cyaw = cyaw;
}

bool AerodynamicCoefficients::isValid() const {
    // Check for finite values
    if (!std::isfinite(m_cd) || !std::isfinite(m_cl) || !std::isfinite(m_cy) ||
        !std::isfinite(m_croll) || !std::isfinite(m_cpitch) || !std::isfinite(m_cyaw)) {
        return false;
    }
    
    // Check for reasonable bounds
    if (std::abs(m_cd) > MAX_REASONABLE_COEFFICIENT ||
        std::abs(m_cl) > MAX_REASONABLE_COEFFICIENT ||
        std::abs(m_cy) > MAX_REASONABLE_COEFFICIENT ||
        std::abs(m_croll) > MAX_REASONABLE_COEFFICIENT ||
        std::abs(m_cpitch) > MAX_REASONABLE_COEFFICIENT ||
        std::abs(m_cyaw) > MAX_REASONABLE_COEFFICIENT) {
        return false;
    }
    
    // Drag coefficient should typically be positive (though can be negative in some cases)
    // No strict sign requirements for other coefficients
    
    return true;
}

std::string AerodynamicCoefficients::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "AeroCoeff[CD=" << m_cd 
        << ", CL=" << m_cl 
        << ", CY=" << m_cy
        << ", Cl=" << m_croll
        << ", Cm=" << m_cpitch
        << ", Cn=" << m_cyaw << "]";
    return oss.str();
}

AerodynamicCoefficients AerodynamicCoefficients::scale(double scaleFactor) const {
    return AerodynamicCoefficients(
        m_cd * scaleFactor,
        m_cl * scaleFactor,
        m_cy * scaleFactor,
        m_croll * scaleFactor,
        m_cpitch * scaleFactor,
        m_cyaw * scaleFactor
    );
}

AerodynamicCoefficients AerodynamicCoefficients::interpolate(
    const AerodynamicCoefficients& other, double factor) const {
    
    if (factor < 0.0 || factor > 1.0) {
        throw std::invalid_argument("Interpolation factor must be between 0 and 1");
    }
    
    double invFactor = 1.0 - factor;
    
    return AerodynamicCoefficients(
        m_cd * invFactor + other.m_cd * factor,
        m_cl * invFactor + other.m_cl * factor,
        m_cy * invFactor + other.m_cy * factor,
        m_croll * invFactor + other.m_croll * factor,
        m_cpitch * invFactor + other.m_cpitch * factor,
        m_cyaw * invFactor + other.m_cyaw * factor
    );
}

bool AerodynamicCoefficients::operator==(const AerodynamicCoefficients& other) const {
    return std::abs(m_cd - other.m_cd) < EPSILON &&
           std::abs(m_cl - other.m_cl) < EPSILON &&
           std::abs(m_cy - other.m_cy) < EPSILON &&
           std::abs(m_croll - other.m_croll) < EPSILON &&
           std::abs(m_cpitch - other.m_cpitch) < EPSILON &&
           std::abs(m_cyaw - other.m_cyaw) < EPSILON;
}

bool AerodynamicCoefficients::operator!=(const AerodynamicCoefficients& other) const {
    return !(*this == other);
}

AerodynamicCoefficients AerodynamicCoefficients::operator+(
    const AerodynamicCoefficients& other) const {
    return AerodynamicCoefficients(
        m_cd + other.m_cd,
        m_cl + other.m_cl,
        m_cy + other.m_cy,
        m_croll + other.m_croll,
        m_cpitch + other.m_cpitch,
        m_cyaw + other.m_cyaw
    );
}

AerodynamicCoefficients& AerodynamicCoefficients::operator+=(
    const AerodynamicCoefficients& other) {
    m_cd += other.m_cd;
    m_cl += other.m_cl;
    m_cy += other.m_cy;
    m_croll += other.m_croll;
    m_cpitch += other.m_cpitch;
    m_cyaw += other.m_cyaw;
    return *this;
}

void AerodynamicCoefficients::validateCoefficient(double value, 
                                                 const std::string& name) const {
    if (!std::isfinite(value)) {
        throw std::invalid_argument(name + " coefficient must be finite");
    }
    if (std::abs(value) > MAX_REASONABLE_COEFFICIENT) {
        throw std::invalid_argument(name + " coefficient exceeds reasonable bounds");
    }
}

// ExtendedAerodynamicCoefficients implementation

ExtendedAerodynamicCoefficients::ExtendedAerodynamicCoefficients()
    : AerodynamicCoefficients() {
}

ExtendedAerodynamicCoefficients::ExtendedAerodynamicCoefficients(
    const AerodynamicCoefficients& base)
    : AerodynamicCoefficients(base) {
}

AerodynamicCoefficients ExtendedAerodynamicCoefficients::applyIncrements(
    double alpha, double beta,
    double p_normalized, double q_normalized, double r_normalized) const {
    
    // Start with base coefficients
    AerodynamicCoefficients result(*this);
    
    // Apply static stability derivative effects
    // Note: These are linear approximations, valid for small angles
    
    // Longitudinal effects (alpha)
    result.setLiftCoefficient(getLiftCoefficient() + m_clAlpha * alpha);
    result.setDragCoefficient(getDragCoefficient() + m_cdAlpha * alpha);
    result.setPitchMomentCoefficient(getPitchMomentCoefficient() + m_cmAlpha * alpha);
    
    // Lateral-directional effects (beta)
    result.setSideForceCoefficient(getSideForceCoefficient() + m_cyBeta * beta);
    result.setRollMomentCoefficient(getRollMomentCoefficient() + m_clBeta * beta);
    result.setYawMomentCoefficient(getYawMomentCoefficient() + m_cnBeta * beta);
    
    // Dynamic effects (angular rates)
    // Pitch rate effects
    result.setLiftCoefficient(result.getLiftCoefficient() + m_clQ * q_normalized);
    result.setPitchMomentCoefficient(result.getPitchMomentCoefficient() + 
                                   m_cmQ * q_normalized);
    
    // Roll rate effects
    result.setRollMomentCoefficient(result.getRollMomentCoefficient() + 
                                   m_clP * p_normalized);
    result.setYawMomentCoefficient(result.getYawMomentCoefficient() + 
                                  m_cnP * p_normalized);
    
    // Yaw rate effects
    result.setRollMomentCoefficient(result.getRollMomentCoefficient() + 
                                   m_clR * r_normalized);
    result.setYawMomentCoefficient(result.getYawMomentCoefficient() + 
                                  m_cnR * r_normalized);
    
    return result;
}

bool ExtendedAerodynamicCoefficients::hasStabilityDerivatives() const {
    return m_clAlpha != 0.0 || m_cdAlpha != 0.0 || m_cmAlpha != 0.0 ||
           m_cyBeta != 0.0 || m_clBeta != 0.0 || m_cnBeta != 0.0 ||
           m_clQ != 0.0 || m_cmQ != 0.0 || m_clP != 0.0 || 
           m_cnP != 0.0 || m_clR != 0.0 || m_cnR != 0.0;
}

} // namespace aerodynamics
} // namespace physics
} // namespace iloss