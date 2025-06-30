#include "physics/dynamics/DynamicsState.h"
#include "core/logging/Logger.h"
#include <cmath>

namespace iloss {
namespace physics {
namespace dynamics {

DynamicsState DynamicsState::operator+(const DynamicsState& other) const
{
    if (getCoordinateSystem() != other.getCoordinateSystem()) {
        throw std::invalid_argument("Cannot add dynamics states in different coordinate systems");
    }
    
    // Add translational components using base class operator
    StateVector baseSum = static_cast<const StateVector&>(*this) + static_cast<const StateVector&>(other);
    
    // For numerical integration, quaternions are added component-wise
    // (when 'other' represents a derivative scaled by dt)
    math::Quaternion newAttitude(
        m_attitude.w() + other.m_attitude.w(),
        m_attitude.x() + other.m_attitude.x(),
        m_attitude.y() + other.m_attitude.y(),
        m_attitude.z() + other.m_attitude.z()
    );
    
    // Check if quaternion is too small to normalize (can happen during integration)
    double qNorm = newAttitude.norm();
    if (qNorm < 1e-10) {
        // If quaternion is essentially zero, use the original attitude
        // This can happen when adding a zero derivative in RK4 intermediate steps
        newAttitude = m_attitude;
    } else {
        // Normalize to maintain unit quaternion constraint
        newAttitude.normalize();
    }
    
    // Add angular velocities
    math::Vector3D newAngularVelocity = m_angularVelocity + other.m_angularVelocity;
    
    // When creating the result state, we need to be careful about normalization
    // If we're adding a derivative (quaternion far from unit), don't normalize
    bool shouldNormalize = true;
    
    // Check if 'other' appears to be a derivative state
    // Derivative states have quaternions with norms far from 1.0
    double otherQNorm = other.m_attitude.norm();
    if (std::abs(otherQNorm - 1.0) > 0.1 || otherQNorm < 0.1) {
        // This is likely a derivative state
        shouldNormalize = (qNorm > 0.1);  // Only normalize if result is not near zero
    }
    
    return DynamicsState(baseSum, newAttitude, newAngularVelocity, shouldNormalize);
}

DynamicsState DynamicsState::operator*(double scalar) const
{
    // Scale translational components using base class operator
    StateVector baseScaled = static_cast<const StateVector&>(*this) * scalar;
    
    // For derivatives in numerical integration, the quaternion components
    // should be scaled directly (they represent quaternion derivatives)
    math::Quaternion scaledQuaternion(
        m_attitude.w() * scalar,
        m_attitude.x() * scalar,
        m_attitude.y() * scalar,
        m_attitude.z() * scalar
    );
    
    // Scale angular velocity
    math::Vector3D scaledAngularVelocity = m_angularVelocity * scalar;
    
    // We need to detect if this is a derivative state being scaled.
    // Derivative states have quaternions that represent rates, not orientations.
    // A key indicator is if the original quaternion norm is far from 1.0
    double originalNorm = m_attitude.norm();
    bool isDerivative = std::abs(originalNorm - 1.0) > 0.1;
    
    // Debug logging
    auto& logger = logging::Logger::getInstance();
    logger.debug(logging::LogCategory::Physics, 
        "DynamicsState::operator* - scalar=" + std::to_string(scalar) +
        ", originalNorm=" + std::to_string(originalNorm) +
        ", scaledNorm=" + std::to_string(scaledQuaternion.norm()) +
        ", isDerivative=" + std::to_string(isDerivative));
    
    if (isDerivative) {
        // This is a derivative state - create using the factory method
        // that doesn't normalize the quaternion
        DynamicsState result = DynamicsState::createDerivativeState(
            baseScaled.getVelocity(),
            math::Vector3D(), // acceleration not used here
            baseScaled.getMass(),
            baseScaled.getTimeAsDouble(),
            scaledQuaternion,
            scaledAngularVelocity,
            baseScaled.getCoordinateSystem()
        );
        // Set the position from the scaled base
        result.StateVector::operator=(baseScaled);
        return result;
    } else {
        // This is a regular state - create normally
        return DynamicsState(
            baseScaled.getPosition(),
            baseScaled.getVelocity(),
            baseScaled.getMass(),
            baseScaled.getTimeAsDouble(),
            scaledQuaternion,
            scaledAngularVelocity,
            baseScaled.getCoordinateSystem()
        );
    }
}

DynamicsState operator*(double scalar, const DynamicsState& state)
{
    return state * scalar;
}

Eigen::VectorXd DynamicsState::toVector() const
{
    Eigen::VectorXd vec(14);  // 7 from StateVector + 4 quaternion + 3 angular velocity
    
    // First 7 elements from base StateVector
    Eigen::VectorXd baseVec = StateVector::toVector();
    vec.head<7>() = baseVec;
    
    // Quaternion components (w, x, y, z)
    vec(7) = m_attitude.w();
    vec(8) = m_attitude.x();
    vec(9) = m_attitude.y();
    vec(10) = m_attitude.z();
    
    // Angular velocity components
    vec(11) = m_angularVelocity.x();
    vec(12) = m_angularVelocity.y();
    vec(13) = m_angularVelocity.z();
    
    return vec;
}

DynamicsState DynamicsState::createDerivativeState(
    const math::Vector3D& velocity,
    const math::Vector3D& acceleration,
    double massRate,
    double time,
    const math::Quaternion& attitudeRate,
    const math::Vector3D& angularAcceleration,
    coordinates::CoordinateSystemType coordinateSystem)
{
    // Create a derivative state that stores actual derivatives
    // When scaled by dt and added to a state, this will produce the correct result
    // The key insight is that for RK4, the derivative must be stored such that
    // state + derivative * h gives the correct next state
    
    // Create zero vectors for the base - we'll set the actual derivatives below
    math::Vector3D zeroVec(0.0, 0.0, 0.0);
    
    // Create a base state with a dummy mass (will be overwritten)
    DynamicsState derivState(zeroVec, zeroVec, 1.0, time, 
                           math::Quaternion::identity(), zeroVec, coordinateSystem);
    
    // Now set the actual derivative values
    // For integration to work correctly with state + deriv * dt:
    // - Position component should hold position derivative (velocity)
    // - Velocity component should hold velocity derivative (acceleration)
    derivState.setPosition(velocity);
    derivState.setVelocity(acceleration);
    
    // For mass, we need to handle it specially since StateVector doesn't support
    // mass derivatives properly. We'll keep the mass constant (massRate = 0)
    // This is a limitation of the current design.
    derivState.setMass(1.0);  // Dummy value, mass changes not supported
    
    // Set attitude rate without normalization
    derivState.m_attitude = attitudeRate;
    
    // Set angular acceleration
    derivState.m_angularVelocity = angularAcceleration;
    
    return derivState;
}

DynamicsState DynamicsState::fromVector(const Eigen::VectorXd& vec, 
                                        double timeJ2000,
                                        coordinates::CoordinateSystemType coordinateSystem)
{
    if (vec.size() != 14) {
        throw std::invalid_argument("Vector must have dimension 14 for DynamicsState");
    }
    
    // Extract base state vector components
    Eigen::VectorXd baseVec(7);
    baseVec = vec.head<7>();
    StateVector baseState = StateVector::fromVector(baseVec, timeJ2000, coordinateSystem);
    
    // Extract quaternion
    math::Quaternion attitude(vec(7), vec(8), vec(9), vec(10));
    attitude.normalize(); // Ensure unit quaternion
    
    // Extract angular velocity
    math::Vector3D angularVelocity(vec(11), vec(12), vec(13));
    
    return DynamicsState(baseState, attitude, angularVelocity);
}

double DynamicsState::errorNorm(const DynamicsState& other) const
{
    // Get base state error
    double baseError = StateVector::errorNorm(other);
    
    // Attitude error - use quaternion difference
    math::Quaternion qDiff = m_attitude.conjugate() * other.m_attitude;
    // Convert to angle error (2 * acos(|w|) gives the rotation angle)
    double angleError = 2.0 * std::acos(std::min(std::abs(qDiff.w()), 1.0));
    
    // Angular velocity error in rad/s
    double angVelError = (m_angularVelocity - other.m_angularVelocity).magnitude();
    
    // Weighted error norm
    // Scale factors chosen to balance different units
    const double angleScale = 0.1;      // 0.1 rad (~5.7 degrees)
    const double angVelScale = 0.01;    // 0.01 rad/s
    
    double scaledAngleError = angleError / angleScale;
    double scaledAngVelError = angVelError / angVelScale;
    
    // Combine errors (RMS with base state error)
    double totalError = std::sqrt((baseError * baseError + 
                                   scaledAngleError * scaledAngleError + 
                                   scaledAngVelError * scaledAngVelError) / 3.0);
    
    return totalError;
}

} // namespace dynamics
} // namespace physics
} // namespace iloss