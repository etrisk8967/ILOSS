#pragma once

#include "core/math/Vector3D.h"
#include "core/math/Matrix3D.h"

namespace iloss {
namespace physics {
namespace dynamics {

/**
 * @brief Interface for mass properties of a rigid body
 * 
 * This interface defines the mass properties required for rigid body dynamics:
 * - Total mass
 * - Center of mass location
 * - Inertia tensor
 * 
 * The inertia tensor should be expressed in the body-fixed coordinate frame
 * with origin at the center of mass.
 */
class IMassProperties {
public:
    virtual ~IMassProperties() = default;

    /**
     * @brief Get the total mass of the body
     * @return Mass in kilograms
     */
    virtual double getMass() const = 0;

    /**
     * @brief Get the center of mass location in body frame
     * @return Center of mass position vector in meters
     * 
     * This is the position of the center of mass relative to the body frame origin.
     * For a spacecraft, this might change as fuel is consumed.
     */
    virtual math::Vector3D getCenterOfMass() const = 0;

    /**
     * @brief Get the inertia tensor about the center of mass
     * @return 3x3 inertia tensor in kg⋅m²
     * 
     * The inertia tensor should be expressed in the body-fixed coordinate frame
     * and computed about the center of mass. The tensor should be symmetric and
     * positive definite.
     */
    virtual math::Matrix3D getInertiaTensor() const = 0;

    /**
     * @brief Get the inverse of the inertia tensor
     * @return 3x3 inverse inertia tensor in kg⁻¹⋅m⁻²
     * 
     * This is provided as a separate method for efficiency, as the inverse
     * is frequently needed in dynamics calculations and can be precomputed.
     */
    virtual math::Matrix3D getInverseInertiaTensor() const = 0;

    /**
     * @brief Check if mass properties are time-varying
     * @return True if properties change with time (e.g., fuel consumption)
     * 
     * If this returns true, the dynamics engine will query the mass properties
     * at each time step. If false, it may cache the values for efficiency.
     */
    virtual bool isTimeVarying() const = 0;

    /**
     * @brief Update mass properties for a given time
     * @param time Current simulation time
     * 
     * This method is called before querying mass properties if isTimeVarying()
     * returns true. It allows the implementation to update internal state based
     * on the current time (e.g., compute remaining fuel mass).
     */
    virtual void updateTime(double time) = 0;
};

} // namespace dynamics
} // namespace physics
} // namespace iloss