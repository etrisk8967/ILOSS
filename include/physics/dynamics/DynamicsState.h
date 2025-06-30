#pragma once

#include "physics/state/StateVector.h"
#include "physics/dynamics/IMassProperties.h"
#include "core/math/Quaternion.h"
#include "core/math/Vector3D.h"
#include "core/math/Matrix3D.h"
#include "core/time/Time.h"
#include <Eigen/Core>
#include <memory>

namespace iloss {
namespace physics {
namespace dynamics {

/**
 * @brief Extended state vector for 6-DOF dynamics
 * 
 * This class extends the basic StateVector to include rotational state:
 * - Attitude (as quaternion)
 * - Angular velocity (in body frame)
 * 
 * Together with position and velocity from StateVector, this provides
 * complete 6-DOF state representation.
 */
class DynamicsState : public StateVector {
public:
    /**
     * @brief Default constructor
     * 
     * Initializes to zero position/velocity, unit quaternion (no rotation),
     * and zero angular velocity
     */
    DynamicsState() 
        : StateVector()
        , m_attitude(math::Quaternion::identity())
        , m_angularVelocity(math::Vector3D()) {}

    /**
     * @brief Construct from translational state and rotational state
     * @param position Position vector in specified coordinate system
     * @param velocity Velocity vector in specified coordinate system
     * @param mass Mass in kg
     * @param time Time in seconds since J2000 epoch
     * @param attitude Attitude quaternion (body to inertial rotation)
     * @param angularVelocity Angular velocity in body frame (rad/s)
     * @param coordinateSystem Coordinate system for position/velocity
     */
    DynamicsState(const math::Vector3D& position,
                  const math::Vector3D& velocity,
                  double mass,
                  double time,
                  const math::Quaternion& attitude,
                  const math::Vector3D& angularVelocity,
                  coordinates::CoordinateSystemType coordinateSystem = coordinates::CoordinateSystemType::ECI_J2000)
        : StateVector(position, velocity, mass, time::Time(time + time::TimeConstants::J2000_EPOCH, time::TimeSystem::UTC), coordinateSystem)
        , m_attitude(attitude.normalized())  // Ensure unit quaternion
        , m_angularVelocity(angularVelocity) {}

    /**
     * @brief Construct from existing StateVector and rotational state
     * @param stateVector Base translational state
     * @param attitude Attitude quaternion
     * @param angularVelocity Angular velocity in body frame
     * @param normalizeQuaternion If true, normalize the quaternion (default: true)
     */
    DynamicsState(const StateVector& stateVector,
                  const math::Quaternion& attitude,
                  const math::Vector3D& angularVelocity,
                  bool normalizeQuaternion = true)
        : StateVector(stateVector)
        , m_attitude(normalizeQuaternion ? attitude.normalized() : attitude)
        , m_angularVelocity(angularVelocity) {}

    /**
     * @brief Get attitude quaternion
     * @return Quaternion representing rotation from body to inertial frame
     */
    const math::Quaternion& getAttitude() const { return m_attitude; }

    /**
     * @brief Get angular velocity
     * @return Angular velocity vector in body frame (rad/s)
     */
    const math::Vector3D& getAngularVelocity() const { return m_angularVelocity; }

    /**
     * @brief Set attitude quaternion
     * @param attitude New attitude (will be normalized)
     */
    void setAttitude(const math::Quaternion& attitude) {
        m_attitude = attitude.normalized();
    }

    /**
     * @brief Set angular velocity
     * @param angularVelocity New angular velocity in body frame (rad/s)
     */
    void setAngularVelocity(const math::Vector3D& angularVelocity) {
        m_angularVelocity = angularVelocity;
    }

    /**
     * @brief Get rotation matrix from body to inertial frame
     * @return 3x3 rotation matrix
     */
    math::Matrix3D getRotationMatrix() const {
        return m_attitude.toRotationMatrix();
    }

    /**
     * @brief Transform vector from body frame to inertial frame
     * @param bodyVector Vector in body frame
     * @return Vector in inertial frame
     */
    math::Vector3D bodyToInertial(const math::Vector3D& bodyVector) const {
        return m_attitude.rotate(bodyVector);
    }

    /**
     * @brief Transform vector from inertial frame to body frame
     * @param inertialVector Vector in inertial frame
     * @return Vector in body frame
     */
    math::Vector3D inertialToBody(const math::Vector3D& inertialVector) const {
        return m_attitude.conjugate().rotate(inertialVector);
    }

    /**
     * @brief Get angular momentum in body frame
     * @param massProperties Mass properties of the body
     * @return Angular momentum vector in body frame (kg⋅m²/s)
     */
    math::Vector3D getAngularMomentumBody(const IMassProperties& massProperties) const {
        return massProperties.getInertiaTensor() * m_angularVelocity;
    }

    /**
     * @brief Get angular momentum in inertial frame
     * @param massProperties Mass properties of the body
     * @return Angular momentum vector in inertial frame (kg⋅m²/s)
     */
    math::Vector3D getAngularMomentumInertial(const IMassProperties& massProperties) const {
        math::Vector3D L_body = getAngularMomentumBody(massProperties);
        return bodyToInertial(L_body);
    }

    /**
     * @brief Get kinetic energy of rotation
     * @param massProperties Mass properties of the body
     * @return Rotational kinetic energy (J)
     */
    double getRotationalKineticEnergy(const IMassProperties& massProperties) const {
        math::Vector3D L = getAngularMomentumBody(massProperties);
        return 0.5 * m_angularVelocity.dot(L);
    }

    /**
     * @brief Get total kinetic energy (translational + rotational)
     * @param massProperties Mass properties of the body
     * @return Total kinetic energy (J)
     */
    double getTotalKineticEnergy(const IMassProperties& massProperties) const {
        double translational = 0.5 * getMass() * getVelocity().magnitudeSquared();
        double rotational = getRotationalKineticEnergy(massProperties);
        return translational + rotational;
    }

    /**
     * @brief Validate the dynamics state
     * @return True if state is valid
     */
    bool isValid() const {
        if (!StateVector::isValid()) {
            return false;
        }

        // Check quaternion is normalized
        double qNorm = m_attitude.norm();
        if (std::abs(qNorm - 1.0) > 1e-6) {
            return false;
        }

        // Check angular velocity is finite
        if (!std::isfinite(m_angularVelocity.magnitude())) {
            return false;
        }

        return true;
    }

    /**
     * @brief Clone this dynamics state
     * @return Deep copy of the state
     */
    std::unique_ptr<DynamicsState> clone() const {
        return std::make_unique<DynamicsState>(*this);
    }

    /**
     * @brief Interpolate between two dynamics states
     * @param other Other dynamics state
     * @param t Interpolation parameter [0, 1]
     * @return Interpolated state
     * 
     * Uses linear interpolation for position, velocity, and angular velocity.
     * Uses SLERP for attitude quaternion interpolation.
     */
    DynamicsState interpolate(const DynamicsState& other, double t) const {
        // Linear interpolation for position and velocity
        math::Vector3D posInterp = getPosition() * (1.0 - t) + other.getPosition() * t;
        math::Vector3D velInterp = getVelocity() * (1.0 - t) + other.getVelocity() * t;
        
        // Interpolate time in J2000 seconds
        double timeInterp = getTimeAsDouble() * (1.0 - t) + other.getTimeAsDouble() * t;
        
        // SLERP for attitude
        math::Quaternion attitudeInterp = m_attitude.slerp(other.m_attitude, t);
        
        // Linear interpolation for angular velocity
        math::Vector3D angVelInterp = m_angularVelocity * (1.0 - t) + other.m_angularVelocity * t;
        
        return DynamicsState(posInterp, velInterp, getMass(), timeInterp, 
                           attitudeInterp, angVelInterp, getCoordinateSystem());
    }

    // Arithmetic operators for integration
    /**
     * @brief Addition operator for dynamics states
     * @param other Other dynamics state to add
     * @return Sum of the two states
     * @throws std::invalid_argument if coordinate systems don't match
     */
    DynamicsState operator+(const DynamicsState& other) const;

    /**
     * @brief Scalar multiplication operator
     * @param scalar Scalar value to multiply by
     * @return Scaled dynamics state
     */
    DynamicsState operator*(double scalar) const;

    /**
     * @brief Friend scalar multiplication operator
     * @param scalar Scalar value to multiply by
     * @param state Dynamics state to scale
     * @return Scaled dynamics state
     */
    friend DynamicsState operator*(double scalar, const DynamicsState& state);

    // Integration support methods
    /**
     * @brief Get the dimension of the dynamics state
     * @return Dimension (14: 3 position + 3 velocity + 1 mass + 4 quaternion + 3 angular velocity)
     */
    std::size_t getDimension() const { return 14; }
    
    /**
     * @brief Get time as a double (seconds since J2000)
     * @return Time in seconds since J2000
     */
    double getTimeAsDouble() const { return StateVector::getTimeAsDouble(); }

    /**
     * @brief Convert dynamics state to Eigen vector representation
     * @return Vector containing [position(3), velocity(3), mass(1), quaternion(4), angular_velocity(3)]
     */
    Eigen::VectorXd toVector() const;

    /**
     * @brief Create a derivative state for integration
     * @param velocity Position derivative (= velocity)
     * @param acceleration Velocity derivative
     * @param massRate Mass rate of change
     * @param time Time
     * @param attitudeRate Quaternion derivative (NOT normalized)
     * @param angularAcceleration Angular acceleration
     * @param coordinateSystem Coordinate system
     * @return Derivative state suitable for integration
     */
    static DynamicsState createDerivativeState(
        const math::Vector3D& velocity,
        const math::Vector3D& acceleration,
        double massRate,
        double time,
        const math::Quaternion& attitudeRate,
        const math::Vector3D& angularAcceleration,
        coordinates::CoordinateSystemType coordinateSystem = coordinates::CoordinateSystemType::ECI_J2000);
    
    /**
     * @brief Create dynamics state from Eigen vector representation
     * @param vec Vector containing [position(3), velocity(3), mass(1), quaternion(4), angular_velocity(3)]
     * @param timeJ2000 Time in seconds since J2000
     * @param coordinateSystem Coordinate system (default: ECI J2000)
     * @return New DynamicsState instance
     */
    static DynamicsState fromVector(const Eigen::VectorXd& vec, 
                                    double timeJ2000,
                                    coordinates::CoordinateSystemType coordinateSystem = 
                                        coordinates::CoordinateSystemType::ECI_J2000);

    /**
     * @brief Calculate error norm between two dynamics states
     * @param other Other state to compare with
     * @return Weighted error norm suitable for adaptive integration
     */
    double errorNorm(const DynamicsState& other) const;

private:
    math::Quaternion m_attitude;        ///< Attitude quaternion (body to inertial)
    math::Vector3D m_angularVelocity;   ///< Angular velocity in body frame (rad/s)
};

} // namespace dynamics
} // namespace physics
} // namespace iloss