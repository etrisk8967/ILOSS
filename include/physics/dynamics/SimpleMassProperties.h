#pragma once

#include "physics/dynamics/IMassProperties.h"
#include <stdexcept>

namespace iloss {
namespace physics {
namespace dynamics {

/**
 * @brief Simple implementation of mass properties for a rigid body
 * 
 * This class provides a straightforward implementation of IMassProperties
 * for bodies with constant mass properties. It's suitable for satellites,
 * spacecraft stages after burn completion, or any rigid body with fixed
 * mass distribution.
 */
class SimpleMassProperties : public IMassProperties {
public:
    /**
     * @brief Construct with mass and inertia tensor
     * @param mass Total mass in kg
     * @param inertiaTensor Inertia tensor about center of mass in kg⋅m²
     * @param centerOfMass Center of mass location in body frame (default: origin)
     * @throws std::invalid_argument if mass <= 0 or inertia tensor is not positive definite
     */
    SimpleMassProperties(double mass, 
                        const math::Matrix3D& inertiaTensor,
                        const math::Vector3D& centerOfMass = math::Vector3D())
        : m_mass(mass)
        , m_centerOfMass(centerOfMass)
        , m_inertiaTensor(inertiaTensor) {
        
        if (mass <= 0.0) {
            throw std::invalid_argument("Mass must be positive");
        }

        // Verify inertia tensor is symmetric and has positive diagonal elements
        validateInertiaTensor(inertiaTensor);
        
        // Precompute inverse for efficiency
        m_inverseInertiaTensor = m_inertiaTensor.inverse();
    }

    /**
     * @brief Construct with mass and principal moments of inertia
     * @param mass Total mass in kg
     * @param Ixx Moment of inertia about x-axis in kg⋅m²
     * @param Iyy Moment of inertia about y-axis in kg⋅m²
     * @param Izz Moment of inertia about z-axis in kg⋅m²
     * @param centerOfMass Center of mass location in body frame (default: origin)
     * @throws std::invalid_argument if mass <= 0 or any moment <= 0
     */
    SimpleMassProperties(double mass, double Ixx, double Iyy, double Izz,
                        const math::Vector3D& centerOfMass = math::Vector3D())
        : m_mass(mass)
        , m_centerOfMass(centerOfMass) {
        
        if (mass <= 0.0) {
            throw std::invalid_argument("Mass must be positive");
        }
        if (Ixx <= 0.0 || Iyy <= 0.0 || Izz <= 0.0) {
            throw std::invalid_argument("Principal moments of inertia must be positive");
        }

        // Create diagonal inertia tensor
        m_inertiaTensor = math::Matrix3D(
            Ixx, 0.0, 0.0,
            0.0, Iyy, 0.0,
            0.0, 0.0, Izz
        );

        // Validate the tensor (includes triangle inequality check)
        validateInertiaTensor(m_inertiaTensor);

        // Precompute inverse
        m_inverseInertiaTensor = math::Matrix3D(
            1.0/Ixx, 0.0,     0.0,
            0.0,     1.0/Iyy, 0.0,
            0.0,     0.0,     1.0/Izz
        );
    }

    // IMassProperties interface implementation
    double getMass() const override { return m_mass; }
    
    math::Vector3D getCenterOfMass() const override { return m_centerOfMass; }
    
    math::Matrix3D getInertiaTensor() const override { return m_inertiaTensor; }
    
    math::Matrix3D getInverseInertiaTensor() const override { return m_inverseInertiaTensor; }
    
    bool isTimeVarying() const override { return false; }
    
    void updateTime(double /*time*/) override {
        // No-op for constant mass properties
    }

    /**
     * @brief Set new mass value
     * @param mass New mass in kg
     * @throws std::invalid_argument if mass <= 0
     */
    void setMass(double mass) {
        if (mass <= 0.0) {
            throw std::invalid_argument("Mass must be positive");
        }
        m_mass = mass;
    }

    /**
     * @brief Set new center of mass location
     * @param centerOfMass New center of mass in body frame
     */
    void setCenterOfMass(const math::Vector3D& centerOfMass) {
        m_centerOfMass = centerOfMass;
    }

    /**
     * @brief Set new inertia tensor
     * @param inertiaTensor New inertia tensor in kg⋅m²
     * @throws std::invalid_argument if tensor is not valid
     */
    void setInertiaTensor(const math::Matrix3D& inertiaTensor) {
        validateInertiaTensor(inertiaTensor);
        m_inertiaTensor = inertiaTensor;
        m_inverseInertiaTensor = m_inertiaTensor.inverse();
    }

private:
    /**
     * @brief Validate that inertia tensor is symmetric and positive definite
     * @param tensor Inertia tensor to validate
     * @throws std::invalid_argument if validation fails
     */
    void validateInertiaTensor(const math::Matrix3D& tensor) const {
        // Check symmetry
        const double tolerance = 1e-10;
        for (int i = 0; i < 3; ++i) {
            for (int j = i + 1; j < 3; ++j) {
                if (std::abs(tensor(i, j) - tensor(j, i)) > tolerance) {
                    throw std::invalid_argument("Inertia tensor must be symmetric");
                }
            }
        }

        // Check diagonal elements are positive (necessary but not sufficient for positive definite)
        if (tensor(0, 0) <= 0.0 || tensor(1, 1) <= 0.0 || tensor(2, 2) <= 0.0) {
            throw std::invalid_argument("Diagonal elements of inertia tensor must be positive");
        }

        // Check triangle inequality for principal moments
        // For a physical inertia tensor: Ixx + Iyy >= Izz (and cyclic permutations)
        double Ixx = tensor(0, 0);
        double Iyy = tensor(1, 1);
        double Izz = tensor(2, 2);
        
        if (Ixx + Iyy < Izz || Iyy + Izz < Ixx || Izz + Ixx < Iyy) {
            throw std::invalid_argument("Inertia tensor violates triangle inequality");
        }
    }

    double m_mass;                              ///< Total mass in kg
    math::Vector3D m_centerOfMass;              ///< Center of mass in body frame
    math::Matrix3D m_inertiaTensor;             ///< Inertia tensor about CoM
    math::Matrix3D m_inverseInertiaTensor;      ///< Precomputed inverse
};

} // namespace dynamics
} // namespace physics
} // namespace iloss