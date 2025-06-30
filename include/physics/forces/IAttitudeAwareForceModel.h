#pragma once

#include "physics/forces/ForceModel.h"
#include "physics/dynamics/DynamicsState.h"
#include "core/math/Vector3D.h"
#include "core/math/Quaternion.h"
#include "core/time/Time.h"
#include <memory>

namespace iloss {
namespace physics {
namespace forces {

/**
 * @brief Interface for force models that require attitude information
 * 
 * This interface extends the base ForceModel to support force calculations
 * that depend on the spacecraft's attitude. Examples include:
 * - Atmospheric drag (requires cross-sectional area based on attitude)
 * - Solar radiation pressure (requires surface area and orientation)
 * - Magnetic forces (requires magnetic moment orientation)
 * 
 * The interface provides two calculation methods:
 * 1. The standard calculateAcceleration() for backward compatibility
 * 2. The new calculateAccelerationWithAttitude() for attitude-aware calculations
 * 
 * Implementations should override calculateAccelerationWithAttitude() to
 * properly account for attitude effects, and may provide a simplified
 * implementation of calculateAcceleration() for cases where attitude is unknown.
 */
class IAttitudeAwareForceModel : public ForceModel {
public:
    /**
     * @brief Constructor
     * @param name Name of the force model
     * @param type Type of the force model
     */
    IAttitudeAwareForceModel(const std::string& name, ForceModelType type)
        : ForceModel(name, type) {}
    
    /**
     * @brief Virtual destructor
     */
    virtual ~IAttitudeAwareForceModel() = default;
    
    /**
     * @brief Calculate acceleration with full dynamics state including attitude
     * 
     * This method receives the complete dynamics state including attitude
     * and angular velocity, allowing for accurate force calculations that
     * depend on spacecraft orientation.
     * 
     * @param state Full dynamics state including position, velocity, attitude, and angular velocity
     * @param time Current time
     * @return Acceleration vector in the inertial frame (m/s²)
     */
    virtual math::Vector3D calculateAccelerationWithAttitude(
        const dynamics::DynamicsState& state,
        const time::Time& time) const = 0;
    
    /**
     * @brief Calculate acceleration without attitude information
     * 
     * This method maintains backward compatibility with the base ForceModel
     * interface. Implementations should provide a reasonable default behavior
     * when attitude information is not available (e.g., using average area,
     * assuming nadir pointing, etc.).
     * 
     * The default implementation creates a DynamicsState with identity
     * attitude and calls calculateAccelerationWithAttitude().
     * 
     * @param state Basic state vector (position, velocity, mass)
     * @param time Current time
     * @return Acceleration vector in the inertial frame (m/s²)
     */
    virtual math::Vector3D calculateAcceleration(
        const StateVector& state,
        const time::Time& time) const override {
        
        // Create a DynamicsState with default attitude (identity quaternion)
        // This represents no rotation from the inertial frame
        dynamics::DynamicsState dynamicsState(
            state,
            math::Quaternion::identity(),  // Identity quaternion (no rotation)
            math::Vector3D::zero()         // Zero angular velocity
        );
        
        // Use the attitude-aware calculation
        return calculateAccelerationWithAttitude(dynamicsState, time);
    }
    
    /**
     * @brief Check if this force model requires attitude information
     * 
     * This can be used by integrators and propagators to determine
     * whether they need to maintain full 6-DOF state for this force model.
     * 
     * @return Always returns true for attitude-aware force models
     */
    virtual bool requiresAttitude() const {
        return true;
    }
    
    /**
     * @brief Get the reference area for the force calculation
     * 
     * Many attitude-dependent forces (drag, SRP) depend on an effective area.
     * This method allows querying the reference area used by the model.
     * 
     * @return Reference area in m²
     */
    virtual double getReferenceArea() const = 0;
    
    /**
     * @brief Set the reference area for the force calculation
     * 
     * @param area Reference area in m²
     */
    virtual void setReferenceArea(double area) = 0;
};

/**
 * @brief Adapter to use attitude-aware force models in standard force aggregator
 * 
 * This adapter allows IAttitudeAwareForceModel implementations to be used
 * in the standard ForceAggregator without modification. It provides default
 * attitude when only StateVector is available.
 */
class AttitudeAwareForceModelAdapter : public ForceModel {
private:
    std::shared_ptr<IAttitudeAwareForceModel> m_attitudeAwareModel;
    math::Quaternion m_defaultAttitude;
    math::Vector3D m_defaultAngularVelocity;
    
public:
    /**
     * @brief Constructor with default attitude
     * @param model Attitude-aware force model to wrap
     * @param defaultAttitude Default attitude to use when not available (default: identity)
     * @param defaultAngularVelocity Default angular velocity (default: zero)
     */
    explicit AttitudeAwareForceModelAdapter(
        std::shared_ptr<IAttitudeAwareForceModel> model,
        const math::Quaternion& defaultAttitude = math::Quaternion::identity(),
        const math::Vector3D& defaultAngularVelocity = math::Vector3D::zero())
        : ForceModel(model->getName() + "_Adapter", model->getType())
        , m_attitudeAwareModel(model)
        , m_defaultAttitude(defaultAttitude)
        , m_defaultAngularVelocity(defaultAngularVelocity) {}
    
    math::Vector3D calculateAcceleration(
        const StateVector& state,
        const time::Time& time) const override {
        
        // Create DynamicsState with default attitude
        dynamics::DynamicsState dynamicsState(
            state,
            m_defaultAttitude,
            m_defaultAngularVelocity
        );
        
        return m_attitudeAwareModel->calculateAccelerationWithAttitude(dynamicsState, time);
    }
    
    bool initialize(const ForceModelConfig& config) override {
        return m_attitudeAwareModel->initialize(config);
    }
    
    void update(const time::Time& time) override {
        m_attitudeAwareModel->update(time);
    }
    
    bool validate() const override {
        return m_attitudeAwareModel->validate();
    }
    
    std::unique_ptr<ForceModel> clone() const override {
        // Clone the underlying model and create a new shared_ptr from it
        auto clonedModel = m_attitudeAwareModel->clone();
        auto attitudeAwareClone = dynamic_cast<IAttitudeAwareForceModel*>(clonedModel.release());
        if (!attitudeAwareClone) {
            throw std::runtime_error("Failed to clone attitude-aware force model");
        }
        
        return std::make_unique<AttitudeAwareForceModelAdapter>(
            std::shared_ptr<IAttitudeAwareForceModel>(attitudeAwareClone),
            m_defaultAttitude,
            m_defaultAngularVelocity
        );
    }
    
    /**
     * @brief Get the wrapped attitude-aware model
     * @return Shared pointer to the attitude-aware force model
     */
    std::shared_ptr<IAttitudeAwareForceModel> getAttitudeAwareModel() const {
        return m_attitudeAwareModel;
    }
    
    /**
     * @brief Set default attitude for when DynamicsState is not available
     * @param attitude Default attitude quaternion
     */
    void setDefaultAttitude(const math::Quaternion& attitude) {
        m_defaultAttitude = attitude;
    }
    
    /**
     * @brief Set default angular velocity
     * @param angularVelocity Default angular velocity in body frame
     */
    void setDefaultAngularVelocity(const math::Vector3D& angularVelocity) {
        m_defaultAngularVelocity = angularVelocity;
    }
};

} // namespace forces
} // namespace physics
} // namespace iloss