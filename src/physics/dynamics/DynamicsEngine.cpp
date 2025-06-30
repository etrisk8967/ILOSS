#include "physics/dynamics/DynamicsEngine.h"
#include "core/logging/Logger.h"

namespace iloss {
namespace physics {
namespace dynamics {

DynamicsEngine::DynamicsStateDerivatives DynamicsEngine::calculateDerivatives(const DynamicsState& state, double time) const {
    DynamicsStateDerivatives derivatives;

    // Update mass properties if time-varying
    if (m_massProperties->isTimeVarying()) {
        m_massProperties->updateTime(time);
    }

    // Get current mass properties
    double mass = m_massProperties->getMass();
    math::Matrix3D inertia = m_massProperties->getInertiaTensor();
    math::Matrix3D invInertia = m_massProperties->getInverseInertiaTensor();
    math::Vector3D centerOfMass = m_massProperties->getCenterOfMass();

    // --- Translational Dynamics ---
    // Calculate total force in inertial frame
    math::Vector3D totalForce = m_forceAggregator->calculateTotalAcceleration(state, time) * mass;
    
    // Newton's second law: a = F/m
    derivatives.velocity = state.getVelocity();
    derivatives.acceleration = totalForce / mass;

    // --- Rotational Dynamics ---
    // Calculate total torque in body frame
    math::Vector3D totalTorque = m_torqueAggregator->calculateTotalTorque(state, time);

    // If center of mass is not at origin, add torque from forces
    if (centerOfMass.magnitude() > 1e-10) {
        // Transform force to body frame and calculate torque about CoM
        math::Vector3D forceBody = state.inertialToBody(totalForce);
        math::Vector3D torqueFromForce = centerOfMass.cross(forceBody);
        totalTorque = totalTorque + torqueFromForce;
    }

    // Get current angular velocity in body frame
    math::Vector3D omega = state.getAngularVelocity();

    // Euler's equations: I*ω̇ = τ - ω × (I*ω)
    math::Vector3D angularMomentum = inertia * omega;
    math::Vector3D gyroscopicTorque = omega.cross(angularMomentum);
    derivatives.angularAcceleration = invInertia * (totalTorque - gyroscopicTorque);

    // --- Attitude Kinematics ---
    // Quaternion derivative: q̇ = 0.5 * q ⊗ ω_quat
    // where ω_quat = (0, ωx, ωy, ωz)
    math::Quaternion omegaQuat(0.0, omega);
    derivatives.attitudeRate = state.getAttitude() * omegaQuat * 0.5;
    
    // Debug: Check if attitude rate is zero
    if (derivatives.attitudeRate.norm() < 1e-15) {
        // If angular velocity is zero, the quaternion derivative is also zero
        // Use a special zero quaternion representation
        derivatives.attitudeRate = math::Quaternion(0.0, 0.0, 0.0, 0.0);
    }

    // Store additional information
    derivatives.time = time;
    derivatives.totalForce = totalForce;
    derivatives.totalTorque = totalTorque;

    return derivatives;
}

DynamicsState DynamicsEngine::applyDerivatives(const DynamicsState& state,
                                              const DynamicsStateDerivatives& derivatives,
                                              double dt) {
    // Update position and velocity
    math::Vector3D newPosition = state.getPosition() + derivatives.velocity * dt;
    math::Vector3D newVelocity = state.getVelocity() + derivatives.acceleration * dt;

    // Update attitude (with normalization)
    math::Quaternion newAttitude = state.getAttitude() + derivatives.attitudeRate * dt;
    newAttitude.normalize();

    // Update angular velocity
    math::Vector3D newAngularVelocity = state.getAngularVelocity() + derivatives.angularAcceleration * dt;

    // Log if significant changes
    auto& logger = logging::Logger::getInstance();
    if (derivatives.totalForce.magnitude() > 1e3 || derivatives.totalTorque.magnitude() > 1e2) {
        logger.debug(logging::LogCategory::Physics, 
            "DynamicsEngine: Large forces/torques: F=" + std::to_string(derivatives.totalForce.magnitude()) + 
            " N, T=" + std::to_string(derivatives.totalTorque.magnitude()) + " N⋅m");
    }

    // Create new state
    return DynamicsState(
        newPosition,
        newVelocity,
        state.getMass(),
        state.getTime().getJ2000(iloss::time::TimeSystem::UTC) + dt,  // J2000 seconds + dt
        newAttitude,
        newAngularVelocity,
        state.getCoordinateSystem()
    );
}

} // namespace dynamics
} // namespace physics
} // namespace iloss