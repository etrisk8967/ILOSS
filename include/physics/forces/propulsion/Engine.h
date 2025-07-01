#pragma once

#include "physics/forces/propulsion/PropulsionConstants.h"
#include <string>
#include <vector>
#include <memory>
#include <utility>
#include <functional>

namespace iloss {
namespace physics {
namespace propulsion {

/**
 * @brief Models a rocket engine with thrust and specific impulse characteristics
 * 
 * This class represents a single rocket engine with configurable thrust curves,
 * specific impulse variations, throttle capabilities, and gimbal limits. It supports
 * both simple constant-thrust models and complex thrust curves that vary with
 * atmospheric pressure and other conditions.
 * 
 * The engine model accounts for:
 * - Vacuum vs sea-level performance
 * - Throttle limits and response
 * - Gimbal angle constraints
 * - Mass flow rate based on thrust and Isp
 * - Optional thrust curves for detailed modeling
 * 
 * Thread-safety: This class is NOT thread-safe. External synchronization required.
 */
class Engine {
public:
    /**
     * @brief Simple engine configuration for constant thrust/Isp
     */
    struct Configuration {
        std::string name;                              ///< Engine identifier
        double vacuumThrust;                           ///< Vacuum thrust (N)
        double vacuumIsp;                              ///< Vacuum specific impulse (s)
        double seaLevelThrust = 0.0;                  ///< Sea level thrust (N), 0 = calculate from nozzle
        double seaLevelIsp = 0.0;                     ///< Sea level Isp (s), 0 = calculate
        double minThrottle = DEFAULT_MIN_THROTTLE;    ///< Minimum throttle (0-1)
        double maxThrottle = DEFAULT_MAX_THROTTLE;    ///< Maximum throttle (0-1)
        double nozzleExitArea = 0.0;                  ///< Nozzle exit area (m²)
        double nozzleExitPressure = 0.0;              ///< Nozzle exit pressure (Pa)
        double gimbalRange = DEFAULT_GIMBAL_RANGE;    ///< Max gimbal angle (rad)
        double gimbalRate = DEFAULT_GIMBAL_RATE;      ///< Max gimbal rate (rad/s)
        EngineCycle engineCycle = EngineCycle::GAS_GENERATOR;  ///< Engine cycle type
    };
    
    /**
     * @brief Thrust curve data point
     */
    struct ThrustPoint {
        double pressure;    ///< Ambient pressure (Pa)
        double thrust;      ///< Thrust at this pressure (N)
        double isp;         ///< Specific impulse at this pressure (s)
    };
    
    /**
     * @brief Engine state information
     */
    struct State {
        double throttleSetting = 0.0;   ///< Current throttle (0-1)
        double gimbalPitch = 0.0;       ///< Gimbal pitch angle (rad)
        double gimbalYaw = 0.0;         ///< Gimbal yaw angle (rad)
        bool isActive = false;          ///< Engine on/off state
        double totalBurnTime = 0.0;     ///< Accumulated burn time (s)
        double totalImpulse = 0.0;      ///< Total impulse delivered (N·s)
    };
    
    /**
     * @brief Construct engine with basic configuration
     * 
     * @param config Engine configuration parameters
     * @throws std::invalid_argument if configuration is invalid
     */
    explicit Engine(const Configuration& config);
    
    /**
     * @brief Construct engine with thrust curve
     * 
     * @param config Basic engine configuration
     * @param thrustCurve Thrust vs pressure data points
     * @param interpolation Interpolation method for curve
     * @throws std::invalid_argument if configuration or curve is invalid
     */
    Engine(const Configuration& config,
           const std::vector<ThrustPoint>& thrustCurve,
           ThrustCurveInterpolation interpolation = ThrustCurveInterpolation::LINEAR);
    
    /**
     * @brief Default destructor
     */
    ~Engine() = default;
    
    // Basic getters
    const std::string& getName() const { return m_config.name; }
    const Configuration& getConfiguration() const { return m_config; }
    const State& getState() const { return m_state; }
    EngineCycle getEngineCycle() const { return m_config.engineCycle; }
    
    // Performance queries
    
    /**
     * @brief Get current thrust
     * 
     * @param ambientPressure Ambient atmospheric pressure (Pa)
     * @return Current thrust in Newtons (accounting for throttle and pressure)
     */
    double getThrust(double ambientPressure = VACUUM_PRESSURE) const;
    
    /**
     * @brief Get current specific impulse
     * 
     * @param ambientPressure Ambient atmospheric pressure (Pa)
     * @return Specific impulse in seconds
     */
    double getIsp(double ambientPressure = VACUUM_PRESSURE) const;
    
    /**
     * @brief Get current mass flow rate
     * 
     * @param ambientPressure Ambient atmospheric pressure (Pa)
     * @return Mass flow rate in kg/s (negative value indicates consumption)
     */
    double getMassFlowRate(double ambientPressure = VACUUM_PRESSURE) const;
    
    /**
     * @brief Get exhaust velocity
     * 
     * @param ambientPressure Ambient atmospheric pressure (Pa)
     * @return Effective exhaust velocity in m/s
     */
    double getExhaustVelocity(double ambientPressure = VACUUM_PRESSURE) const;
    
    // State control
    
    /**
     * @brief Start the engine
     * 
     * @param initialThrottle Initial throttle setting (0-1)
     * @return True if engine started successfully
     */
    bool start(double initialThrottle = 1.0);
    
    /**
     * @brief Shutdown the engine
     */
    void shutdown();
    
    /**
     * @brief Set throttle
     * 
     * @param throttle Throttle setting (0-1)
     * @return Actual throttle set (clamped to limits)
     */
    double setThrottle(double throttle);
    
    /**
     * @brief Set gimbal angles
     * 
     * @param pitch Pitch angle (rad)
     * @param yaw Yaw angle (rad)
     * @return True if angles within limits
     */
    bool setGimbalAngles(double pitch, double yaw);
    
    /**
     * @brief Check if engine is active
     * @return True if engine is running
     */
    bool isActive() const { return m_state.isActive; }
    
    /**
     * @brief Check if engine can throttle
     * @return True if minThrottle < maxThrottle
     */
    bool canThrottle() const { 
        return m_config.minThrottle < m_config.maxThrottle; 
    }
    
    /**
     * @brief Check if engine has gimbal capability
     * @return True if gimbalRange > 0
     */
    bool hasGimbal() const { 
        return m_config.gimbalRange > 0.0; 
    }
    
    // Statistics and monitoring
    
    /**
     * @brief Update engine statistics
     * 
     * Call this each timestep to track burn time and total impulse
     * 
     * @param deltaTime Time step (s)
     * @param ambientPressure Current ambient pressure (Pa)
     */
    void updateStatistics(double deltaTime, double ambientPressure = VACUUM_PRESSURE);
    
    /**
     * @brief Get total burn time
     * @return Accumulated burn time in seconds
     */
    double getTotalBurnTime() const { return m_state.totalBurnTime; }
    
    /**
     * @brief Get total impulse delivered
     * @return Total impulse in N·s
     */
    double getTotalImpulse() const { return m_state.totalImpulse; }
    
    /**
     * @brief Reset engine statistics
     */
    void resetStatistics();
    
    // Utility methods
    
    /**
     * @brief Get string representation of engine state
     * @return Human-readable engine status
     */
    std::string toString() const;
    
    /**
     * @brief Create a copy of this engine
     * @return Unique pointer to cloned engine
     */
    std::unique_ptr<Engine> clone() const;
    
    /**
     * @brief Validate engine is in operational state
     * @return True if engine parameters are valid for operation
     */
    bool validate() const;
    
    /**
     * @brief Calculate thrust coefficient (for analysis)
     * 
     * @param ambientPressure Ambient pressure (Pa)
     * @return Thrust coefficient CF
     */
    double getThrustCoefficient(double ambientPressure = VACUUM_PRESSURE) const;

private:
    Configuration m_config;                        ///< Engine configuration
    State m_state;                                 ///< Current engine state
    std::vector<ThrustPoint> m_thrustCurve;       ///< Optional thrust curve
    ThrustCurveInterpolation m_interpolation;     ///< Curve interpolation method
    
    /**
     * @brief Validate configuration parameters
     * @throws std::invalid_argument if parameters invalid
     */
    void validateConfiguration() const;
    
    /**
     * @brief Validate thrust curve data
     * @throws std::invalid_argument if curve invalid
     */
    void validateThrustCurve() const;
    
    /**
     * @brief Calculate thrust from basic parameters
     * 
     * @param ambientPressure Ambient pressure (Pa)
     * @param throttle Throttle setting (0-1)
     * @return Thrust in Newtons
     */
    double calculateBasicThrust(double ambientPressure, double throttle) const;
    
    /**
     * @brief Calculate Isp from basic parameters
     * 
     * @param ambientPressure Ambient pressure (Pa)
     * @return Specific impulse in seconds
     */
    double calculateBasicIsp(double ambientPressure) const;
    
    /**
     * @brief Interpolate thrust curve
     * 
     * @param pressure Ambient pressure (Pa)
     * @param[out] thrust Interpolated thrust (N)
     * @param[out] isp Interpolated Isp (s)
     */
    void interpolateThrustCurve(double pressure, double& thrust, double& isp) const;
    
    /**
     * @brief Linear interpolation helper
     */
    static double lerp(double x0, double y0, double x1, double y1, double x);
};

} // namespace propulsion
} // namespace physics
} // namespace iloss