#include "physics/aerodynamics/AerodynamicCalculator.h"
#include <cmath>
#include <stdexcept>

namespace iloss {
namespace physics {
namespace aerodynamics {

double AerodynamicCalculator::calculateMachNumber(double velocity, double speedOfSound) {
    if (speedOfSound <= 0.0) {
        throw std::invalid_argument("Speed of sound must be positive");
    }
    return velocity / speedOfSound;
}

double AerodynamicCalculator::calculateDynamicPressure(double density, double velocity) {
    if (density < 0.0) {
        throw std::invalid_argument("Density cannot be negative");
    }
    return 0.5 * density * velocity * velocity;
}

std::optional<std::pair<double, double>> AerodynamicCalculator::calculateAeroAngles(
    const iloss::math::Vector3D& velocityBody, double minVelocity) {
    
    double vmag = velocityBody.magnitude();
    
    // Cannot determine angles if velocity is too small
    if (vmag < minVelocity) {
        return std::nullopt;
    }
    
    // Extract velocity components in body frame
    // Body frame: X-forward, Y-right, Z-down
    double u = velocityBody.x();  // Forward velocity
    double v = velocityBody.y();  // Right velocity
    double w = velocityBody.z();  // Down velocity
    
    // Calculate angle of attack
    // α = atan2(w, u) - positive for nose up
    double alpha = std::atan2(w, u);
    
    // Calculate sideslip angle
    // β = asin(v / |V|) - positive for nose right
    // Use atan2 for better numerical behavior
    double horizontalSpeed = std::sqrt(u * u + w * w);
    double beta = std::atan2(v, horizontalSpeed);
    
    // Alternative formulation that's more stable at high angles:
    // β = atan2(v, sqrt(u² + w²))
    
    return std::make_pair(alpha, beta);
}

FlowProperties AerodynamicCalculator::calculateFlowProperties(
    const iloss::math::Vector3D& velocityInertial,
    const iloss::math::Quaternion& attitude,
    double density,
    double speedOfSound,
    const iloss::math::Vector3D& windVelocity) {
    
    FlowProperties props;
    
    // Calculate relative velocity (airspeed)
    iloss::math::Vector3D relativeVelocityInertial = velocityInertial - windVelocity;
    
    // Transform to body frame
    props.velocityBodyFrame = attitude.rotateVector(relativeVelocityInertial);
    
    // Calculate velocity magnitude
    double velocityMag = relativeVelocityInertial.magnitude();
    
    // Calculate Mach number and dynamic pressure
    props.mach = calculateMachNumber(velocityMag, speedOfSound);
    props.dynamicPressure = calculateDynamicPressure(density, velocityMag);
    
    // Calculate aerodynamic angles
    auto angles = calculateAeroAngles(props.velocityBodyFrame);
    if (angles.has_value()) {
        props.angleOfAttack = angles->first;
        props.sideslipAngle = angles->second;
    } else {
        // Very low speed - set angles to zero
        props.angleOfAttack = 0.0;
        props.sideslipAngle = 0.0;
    }
    
    // Calculate total angle of attack for high-alpha cases
    props.totalAngleOfAttack = calculateTotalAngleOfAttack(props.velocityBodyFrame);
    
    // Calculate velocity in wind frame (aligned with velocity vector)
    if (velocityMag > EPSILON) {
        props.velocityWindFrame = iloss::math::Vector3D(velocityMag, 0.0, 0.0);
    } else {
        props.velocityWindFrame = iloss::math::Vector3D();
    }
    
    return props;
}

iloss::math::Vector3D AerodynamicCalculator::bodyToWind(
    const iloss::math::Vector3D& vectorBody,
    double angleOfAttack,
    double sideslipAngle) {
    
    iloss::math::Matrix3D transform = getBodyToWindMatrix(angleOfAttack, sideslipAngle);
    return transform * vectorBody;
}

iloss::math::Vector3D AerodynamicCalculator::windToBody(
    const iloss::math::Vector3D& vectorWind,
    double angleOfAttack,
    double sideslipAngle) {
    
    iloss::math::Matrix3D transform = getWindToBodyMatrix(angleOfAttack, sideslipAngle);
    return transform * vectorWind;
}

iloss::math::Matrix3D AerodynamicCalculator::getBodyToWindMatrix(
    double angleOfAttack, double sideslipAngle) {
    
    // The transformation from body to wind frame involves two rotations:
    // 1. Rotate by -beta about body Z-axis
    // 2. Rotate by -alpha about the new Y-axis
    
    double ca = std::cos(angleOfAttack);
    double sa = std::sin(angleOfAttack);
    double cb = std::cos(sideslipAngle);
    double sb = std::sin(sideslipAngle);
    
    // Combined transformation matrix
    iloss::math::Matrix3D transform;
    transform(0, 0) = ca * cb;
    transform(0, 1) = sb;
    transform(0, 2) = sa * cb;
    
    transform(1, 0) = -ca * sb;
    transform(1, 1) = cb;
    transform(1, 2) = -sa * sb;
    
    transform(2, 0) = -sa;
    transform(2, 1) = 0.0;
    transform(2, 2) = ca;
    
    return transform;
}

iloss::math::Matrix3D AerodynamicCalculator::getWindToBodyMatrix(
    double angleOfAttack, double sideslipAngle) {
    
    // This is the inverse of body-to-wind transformation
    // For orthogonal matrices, inverse = transpose
    double ca = std::cos(angleOfAttack);
    double sa = std::sin(angleOfAttack);
    double cb = std::cos(sideslipAngle);
    double sb = std::sin(sideslipAngle);
    
    iloss::math::Matrix3D transform;
    transform(0, 0) = ca * cb;
    transform(0, 1) = -ca * sb;
    transform(0, 2) = -sa;
    
    transform(1, 0) = sb;
    transform(1, 1) = cb;
    transform(1, 2) = 0.0;
    
    transform(2, 0) = sa * cb;
    transform(2, 1) = -sa * sb;
    transform(2, 2) = ca;
    
    return transform;
}

iloss::math::Vector3D AerodynamicCalculator::bodyToStability(
    const iloss::math::Vector3D& vectorBody, double angleOfAttack) {
    
    // Stability frame is rotated by angle of attack only
    // Rotation is about body Y-axis by -alpha
    double ca = std::cos(angleOfAttack);
    double sa = std::sin(angleOfAttack);
    
    return iloss::math::Vector3D(
        ca * vectorBody.x() + sa * vectorBody.z(),
        vectorBody.y(),
        -sa * vectorBody.x() + ca * vectorBody.z()
    );
}

double AerodynamicCalculator::calculateTotalAngleOfAttack(
    const iloss::math::Vector3D& velocityBody) {
    
    double vmag = velocityBody.magnitude();
    if (vmag < MIN_VELOCITY_FOR_ANGLES) {
        return 0.0;
    }
    
    // Total angle is between velocity vector and body X-axis
    double cosAlphaTotal = velocityBody.x() / vmag;
    
    // Clamp to valid range for acos
    cosAlphaTotal = std::max(-1.0, std::min(1.0, cosAlphaTotal));
    
    return std::acos(cosAlphaTotal);
}

double AerodynamicCalculator::calculateReynoldsNumber(
    double density, double velocity, double characteristicLength, double dynamicViscosity) {
    
    if (dynamicViscosity <= 0.0) {
        throw std::invalid_argument("Dynamic viscosity must be positive");
    }
    if (characteristicLength <= 0.0) {
        throw std::invalid_argument("Characteristic length must be positive");
    }
    
    return density * velocity * characteristicLength / dynamicViscosity;
}

double AerodynamicCalculator::calculateCenterOfPressure(
    double cm, double cn, double referenceLength, double momentReferencePoint) {
    
    if (std::abs(cn) < EPSILON) {
        // No normal force - center of pressure is undefined
        // Return moment reference point as default
        return momentReferencePoint;
    }
    
    // Center of pressure location from moment balance:
    // Cm = (Xcp - Xref) * Cn / c
    // Therefore: Xcp = Xref - (Cm * c) / Cn
    
    return momentReferencePoint - (cm * referenceLength) / cn;
}

bool AerodynamicCalculator::isTransonic(double mach, double lowerBound, double upperBound) {
    return mach >= lowerBound && mach <= upperBound;
}

double AerodynamicCalculator::calculateSpeedOfSound(
    double temperature, double gamma, double gasConstant) {
    
    if (temperature <= 0.0) {
        throw std::invalid_argument("Temperature must be positive");
    }
    
    return std::sqrt(gamma * gasConstant * temperature);
}

} // namespace aerodynamics
} // namespace physics
} // namespace iloss