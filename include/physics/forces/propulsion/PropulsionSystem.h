#pragma once

#include "physics/dynamics/IMassProperties.h"
#include "physics/forces/propulsion/Engine.h"
#include "physics/forces/propulsion/PropellantTank.h"
#include "physics/forces/propulsion/ThrustVectorControl.h"
#include "core/math/Vector3D.h"
#include "core/math/Matrix3D.h"
#include <vector>
#include <memory>
#include <unordered_map>
#include <string>

namespace iloss {
namespace physics {
namespace propulsion {

/**
 * @brief Manages the complete propulsion system including engines, tanks, and mass properties
 * 
 * This class coordinates all propulsion components and provides time-varying mass
 * properties to the dynamics engine. It handles:
 * - Multiple engines with individual throttle control
 * - Multiple propellant tanks with automatic fuel distribution
 * - Center of mass shifts due to propellant consumption
 * - Inertia tensor updates as mass depletes
 * - Engine-tank assignments for different propellant types
 * 
 * The PropulsionSystem implements IMassProperties to provide accurate mass
 * properties that change as propellant is consumed during burns.
 * 
 * Thread-safety: This class is NOT thread-safe. External synchronization required.
 */
class PropulsionSystem : public dynamics::IMassProperties {
public:
    /**
     * @brief Configuration for the propulsion system
     */
    struct Configuration {
        double dryMass;                             ///< Dry mass of vehicle (kg)
        math::Vector3D dryCenterOfMass;            ///< Dry CoM in body frame (m)
        math::Matrix3D dryInertiaTensor;           ///< Dry inertia about CoM (kg·m²)
        double structuralDamping;                  ///< Structural damping coefficient
        bool autoDistributeFuel;                   ///< Automatically distribute fuel draw
        double fuelMargin;                         ///< Reserve fuel fraction (1%)
        
        /**
         * @brief Default constructor with standard values
         */
        Configuration()
            : dryMass(1000.0)
            , dryCenterOfMass()
            , dryInertiaTensor()
            , structuralDamping(0.01)
            , autoDistributeFuel(true)
            , fuelMargin(0.01) {}
    };
    
    /**
     * @brief Engine mounting information
     */
    struct EngineMounting {
        std::string engineId;                       ///< Engine identifier
        math::Vector3D position;                    ///< Mount position in body frame (m)
        math::Vector3D orientation;                 ///< Mount orientation (thrust direction)
        std::vector<std::string> assignedTanks;    ///< Tanks that feed this engine
        std::shared_ptr<ThrustVectorControl> tvc;  ///< Optional TVC system
    };
    
    /**
     * @brief Propulsion system state
     */
    struct State {
        double totalPropellantMass = 0.0;          ///< Current total propellant (kg)
        double totalMass = 0.0;                    ///< Current total mass (kg)
        math::Vector3D centerOfMass;               ///< Current CoM (m)
        math::Matrix3D inertiaTensor;              ///< Current inertia (kg·m²)
        double totalThrust = 0.0;                  ///< Current total thrust (N)
        double totalMassFlowRate = 0.0;           ///< Current flow rate (kg/s)
        size_t activeEngineCount = 0;              ///< Number of active engines
    };
    
    /**
     * @brief Construct propulsion system with configuration
     * 
     * @param config System configuration
     * @throws std::invalid_argument if configuration is invalid
     */
    explicit PropulsionSystem(const Configuration& config = Configuration());
    
    /**
     * @brief Destructor
     */
    virtual ~PropulsionSystem() = default;
    
    // Component management
    
    /**
     * @brief Add an engine to the system
     * 
     * @param engine Engine to add
     * @param mounting Mounting configuration
     * @return Engine ID for reference
     * @throws std::invalid_argument if engine already exists
     */
    std::string addEngine(std::shared_ptr<Engine> engine, 
                         const EngineMounting& mounting);
    
    /**
     * @brief Add a propellant tank to the system
     * 
     * @param tank Tank to add
     * @return Tank ID for reference
     * @throws std::invalid_argument if tank already exists
     */
    std::string addTank(std::shared_ptr<PropellantTank> tank);
    
    /**
     * @brief Assign tanks to an engine
     * 
     * @param engineId Engine identifier
     * @param tankIds List of tank identifiers
     * @throws std::runtime_error if engine or tanks not found
     */
    void assignTanksToEngine(const std::string& engineId,
                           const std::vector<std::string>& tankIds);
    
    /**
     * @brief Get engine by ID
     * @param engineId Engine identifier
     * @return Shared pointer to engine (nullptr if not found)
     */
    std::shared_ptr<Engine> getEngine(const std::string& engineId) const;
    
    /**
     * @brief Get tank by ID
     * @param tankId Tank identifier
     * @return Shared pointer to tank (nullptr if not found)
     */
    std::shared_ptr<PropellantTank> getTank(const std::string& tankId) const;
    
    /**
     * @brief Get all engines
     * @return Map of engine IDs to engines
     */
    const std::unordered_map<std::string, std::shared_ptr<Engine>>& 
        getEngines() const { return m_engines; }
    
    /**
     * @brief Get all tanks
     * @return Map of tank IDs to tanks
     */
    const std::unordered_map<std::string, std::shared_ptr<PropellantTank>>& 
        getTanks() const { return m_tanks; }
    
    /**
     * @brief Get engine mounting information
     * @param engineId Engine identifier
     * @return Pointer to mounting info (nullptr if not found)
     */
    const EngineMounting* getEngineMounting(const std::string& engineId) const;
    
    // System state and control
    
    /**
     * @brief Get current system state
     * @return Current state
     */
    const State& getSystemState() const { return m_state; }
    
    /**
     * @brief Update system for current timestep
     * 
     * This updates mass properties, consumes propellant, and tracks statistics.
     * Should be called once per integration step.
     * 
     * @param deltaTime Time step (s)
     * @param ambientPressure Current atmospheric pressure (Pa)
     */
    void update(double deltaTime, double ambientPressure = 0.0);
    
    /**
     * @brief Calculate total thrust vector in body frame
     * 
     * @param ambientPressure Atmospheric pressure (Pa)
     * @return Total thrust vector accounting for all active engines
     */
    math::Vector3D getTotalThrustVector(double ambientPressure = 0.0) const;
    
    /**
     * @brief Calculate total torque about center of mass
     * 
     * @param ambientPressure Atmospheric pressure (Pa)
     * @return Total torque from thrust misalignment
     */
    math::Vector3D getTotalTorque(double ambientPressure = 0.0) const;
    
    /**
     * @brief Check if propulsion system can operate
     * @return True if at least one engine has fuel
     */
    bool canOperate() const;
    
    /**
     * @brief Emergency shutdown all engines
     */
    void emergencyShutdown();
    
    // IMassProperties implementation
    
    double getMass() const override { return m_state.totalMass; }
    math::Vector3D getCenterOfMass() const override { return m_state.centerOfMass; }
    math::Matrix3D getInertiaTensor() const override { return m_state.inertiaTensor; }
    math::Matrix3D getInverseInertiaTensor() const override;
    bool isTimeVarying() const override { return true; }
    void updateTime(double time) override;
    
    // Utility methods
    
    /**
     * @brief Validate system configuration
     * @return True if configuration is valid
     */
    bool validate() const;
    
    /**
     * @brief Get string representation of system state
     * @return Human-readable status
     */
    std::string toString() const;
    
    /**
     * @brief Create a copy of this propulsion system
     * @return Unique pointer to cloned system
     */
    std::unique_ptr<PropulsionSystem> clone() const;
    
    /**
     * @brief Calculate specific impulse of entire system
     * @return Effective Isp based on current thrust and flow rate
     */
    double getEffectiveIsp() const;
    
    /**
     * @brief Calculate remaining delta-V capability
     * @return Estimated delta-V with current propellant (m/s)
     */
    double getRemainingDeltaV() const;

private:
    Configuration m_config;                                              ///< System configuration
    State m_state;                                                      ///< Current state
    std::unordered_map<std::string, std::shared_ptr<Engine>> m_engines;            ///< All engines
    std::unordered_map<std::string, std::shared_ptr<PropellantTank>> m_tanks;      ///< All tanks
    std::unordered_map<std::string, EngineMounting> m_engineMountings;             ///< Engine mount info
    mutable math::Matrix3D m_inverseInertiaTensor;                     ///< Cached inverse
    mutable bool m_inverseInertiaTensorValid = false;                  ///< Cache validity
    double m_currentTime = 0.0;                                         ///< Current simulation time
    
    /**
     * @brief Update mass properties based on current propellant
     */
    void updateMassProperties();
    
    /**
     * @brief Calculate center of mass
     * @return Current CoM position
     */
    math::Vector3D calculateCenterOfMass() const;
    
    /**
     * @brief Calculate inertia tensor about CoM
     * @return Current inertia tensor
     */
    math::Matrix3D calculateInertiaTensor() const;
    
    /**
     * @brief Distribute propellant consumption among tanks
     * 
     * @param engine Engine consuming propellant
     * @param requiredMass Mass to consume (kg)
     * @param deltaTime Time step (s)
     * @return Actual mass consumed
     */
    double distributePropellantConsumption(const Engine& engine,
                                          double requiredMass,
                                          double deltaTime);
    
    /**
     * @brief Apply parallel axis theorem
     * 
     * @param inertia Inertia about object CoM
     * @param mass Object mass
     * @param offset Offset from system CoM
     * @return Inertia about system CoM
     */
    static math::Matrix3D applyParallelAxis(const math::Matrix3D& inertia,
                                           double mass,
                                           const math::Vector3D& offset);
    
    /**
     * @brief Validate configuration
     * @throws std::invalid_argument if invalid
     */
    void validateConfiguration() const;
};

} // namespace propulsion
} // namespace physics
} // namespace iloss