#pragma once

#include "physics/forces/IAttitudeAwareForceModel.h"
#include "physics/forces/propulsion/PropulsionSystem.h"
#include "core/constants/AtmosphericModel.h"
#include <memory>
#include <string>

namespace iloss {
namespace physics {
namespace propulsion {

/**
 * @brief Force model that calculates thrust acceleration from a propulsion system
 * 
 * This class integrates the PropulsionSystem with the force aggregation framework.
 * It implements IAttitudeAwareForceModel to properly account for:
 * - Thrust vector orientation based on spacecraft attitude
 * - Mass depletion during burns
 * - Atmospheric effects on thrust
 * - Multiple engines with thrust vector control
 * 
 * The ThrustForceModel works in conjunction with the PropulsionSystem, which
 * manages engines, tanks, and provides time-varying mass properties to the
 * dynamics engine.
 * 
 * Thread-safety: This class is NOT thread-safe. External synchronization required.
 */
class ThrustForceModel : public forces::IAttitudeAwareForceModel {
public:
    /**
     * @brief Configuration for thrust force model
     */
    struct Configuration {
        bool enableAtmosphericEffects;      ///< Account for back pressure
        bool enableMassUpdate;              ///< Update mass during integration
        double thrustEfficiency;            ///< Thrust efficiency factor (0-1)
        double thrustUncertainty;           ///< Thrust uncertainty (fraction)
        bool logThrustEvents;               ///< Log significant thrust events
        
        /**
         * @brief Default constructor with standard values
         */
        Configuration()
            : enableAtmosphericEffects(true)
            , enableMassUpdate(true)
            , thrustEfficiency(1.0)
            , thrustUncertainty(0.0)
            , logThrustEvents(true) {}
    };
    
    /**
     * @brief Construct thrust force model with propulsion system
     * 
     * @param name Force model name
     * @param propulsionSystem Shared pointer to propulsion system
     * @param atmosphereModel Optional atmosphere model for pressure
     * @param config Configuration parameters
     */
    ThrustForceModel(const std::string& name,
                     std::shared_ptr<PropulsionSystem> propulsionSystem,
                     std::shared_ptr<constants::AtmosphericModel> atmosphereModel = nullptr,
                     const Configuration& config = Configuration());
    
    /**
     * @brief Destructor
     */
    virtual ~ThrustForceModel() = default;
    
    // IAttitudeAwareForceModel implementation
    
    math::Vector3D calculateAccelerationWithAttitude(
        const dynamics::DynamicsState& state,
        const time::Time& time) const override;
    
    bool initialize(const forces::ForceModelConfig& config) override;
    void update(const time::Time& time) override;
    bool validate() const override;
    std::unique_ptr<forces::ForceModel> clone() const override;
    
    double getReferenceArea() const override { return 0.0; }  // Not applicable
    void setReferenceArea(double /*area*/) override {}  // Not applicable
    
    // Propulsion-specific methods
    
    /**
     * @brief Get the propulsion system
     * @return Shared pointer to propulsion system
     */
    std::shared_ptr<PropulsionSystem> getPropulsionSystem() const {
        return m_propulsionSystem;
    }
    
    /**
     * @brief Get current atmospheric pressure
     * 
     * @param position Position in ECEF coordinates (m)
     * @param time Current time
     * @return Atmospheric pressure (Pa)
     */
    double getAtmosphericPressure(const math::Vector3D& position,
                                  const time::Time& time) const;
    
    /**
     * @brief Set throttle for a specific engine
     * 
     * @param engineId Engine identifier
     * @param throttle Throttle setting (0-1)
     * @return True if successful
     */
    bool setEngineThrottle(const std::string& engineId, double throttle);
    
    /**
     * @brief Set gimbal angles for a specific engine
     * 
     * @param engineId Engine identifier
     * @param pitch Pitch angle (rad)
     * @param yaw Yaw angle (rad)
     * @return True if successful
     */
    bool setEngineGimbal(const std::string& engineId, double pitch, double yaw);
    
    /**
     * @brief Start an engine
     * 
     * @param engineId Engine identifier
     * @param throttle Initial throttle (0-1)
     * @return True if successful
     */
    bool startEngine(const std::string& engineId, double throttle = 1.0);
    
    /**
     * @brief Shutdown an engine
     * 
     * @param engineId Engine identifier
     */
    void shutdownEngine(const std::string& engineId);
    
    /**
     * @brief Shutdown all engines
     */
    void shutdownAllEngines();
    
    /**
     * @brief Get current total thrust magnitude
     * @return Total thrust in Newtons
     */
    double getTotalThrust() const { return m_lastTotalThrust; }
    
    /**
     * @brief Get current mass flow rate
     * @return Total mass flow rate in kg/s (positive for consumption)
     */
    double getMassFlowRate() const { return m_lastMassFlowRate; }
    
    /**
     * @brief Check if any engines are active
     * @return True if at least one engine is running
     */
    bool isActive() const;
    
    /**
     * @brief Get effective specific impulse of active engines
     * @return Effective Isp in seconds
     */
    double getEffectiveIsp() const {
        return m_propulsionSystem->getEffectiveIsp();
    }
    
    /**
     * @brief Get string representation of force model state
     * @return Human-readable status
     */
    std::string toString() const override;

private:
    std::shared_ptr<PropulsionSystem> m_propulsionSystem;              ///< Propulsion system
    std::shared_ptr<constants::AtmosphericModel> m_atmosphereModel;   ///< Atmosphere model
    Configuration m_configuration;                                      ///< Model configuration
    mutable double m_lastAtmosphericPressure = 0.0;                   ///< Cached pressure
    mutable double m_lastTotalThrust = 0.0;                           ///< Cached thrust
    mutable double m_lastMassFlowRate = 0.0;                          ///< Cached flow rate
    mutable time::Time m_lastUpdateTime;                               ///< Last update time
    double m_simulationTime = 0.0;                                     ///< Current sim time
    
    /**
     * @brief Update propulsion system state
     * 
     * @param deltaTime Time since last update (s)
     * @param pressure Atmospheric pressure (Pa)
     */
    void updatePropulsionSystem(double deltaTime, double pressure) const;
    
    /**
     * @brief Apply thrust uncertainty if configured
     * 
     * @param thrust Nominal thrust vector
     * @return Thrust with uncertainty applied
     */
    math::Vector3D applyThrustUncertainty(const math::Vector3D& thrust) const;
    
    /**
     * @brief Log thrust event if significant change
     * 
     * @param thrust Current thrust magnitude
     * @param flowRate Current mass flow rate
     */
    void logThrustEvent(double thrust, double flowRate) const;
};

} // namespace propulsion
} // namespace physics
} // namespace iloss