#pragma once

#include "core/math/Vector3D.h"
#include "core/math/Matrix3D.h"
#include <string>
#include <memory>
#include <cmath>

namespace iloss {
namespace physics {
namespace propulsion {

/**
 * @brief Models thrust vector control (TVC) system for engine gimbaling
 * 
 * This class handles the transformation of thrust vectors based on gimbal angles,
 * enforces gimbal limits, and models gimbal dynamics including rate limits.
 * It supports both single-axis and dual-axis gimbaling systems.
 * 
 * The gimbal angles follow aerospace conventions:
 * - Pitch: Rotation about the Y-axis (positive nose up)
 * - Yaw: Rotation about the Z-axis (positive nose right)
 * - Nominal thrust direction: Along negative Z-axis in body frame
 * 
 * Thread-safety: This class is NOT thread-safe. External synchronization required.
 */
class ThrustVectorControl {
public:
    /**
     * @brief TVC system configuration
     */
    struct Configuration {
        double maxPitchAngle;      ///< Maximum pitch angle (rad) ~5°
        double maxYawAngle;        ///< Maximum yaw angle (rad) ~5°
        double maxPitchRate;       ///< Maximum pitch rate (rad/s) ~10°/s
        double maxYawRate;         ///< Maximum yaw rate (rad/s) ~10°/s
        bool enablePitch;          ///< Enable pitch axis
        bool enableYaw;            ///< Enable yaw axis
        double actuatorDamping;    ///< Actuator damping ratio (0-1)
        double actuatorFrequency;  ///< Actuator natural frequency (Hz)
        
        /**
         * @brief Default constructor with standard values
         */
        Configuration()
            : maxPitchAngle(0.0872665)
            , maxYawAngle(0.0872665)
            , maxPitchRate(0.174533)
            , maxYawRate(0.174533)
            , enablePitch(true)
            , enableYaw(true)
            , actuatorDamping(0.7)
            , actuatorFrequency(10.0) {}
    };
    
    /**
     * @brief TVC system state
     */
    struct State {
        double currentPitch = 0.0;         ///< Current pitch angle (rad)
        double currentYaw = 0.0;           ///< Current yaw angle (rad)
        double pitchRate = 0.0;            ///< Current pitch rate (rad/s)
        double yawRate = 0.0;              ///< Current yaw rate (rad/s)
        double commandedPitch = 0.0;       ///< Commanded pitch angle (rad)
        double commandedYaw = 0.0;         ///< Commanded yaw angle (rad)
    };
    
    /**
     * @brief Construct TVC system with configuration
     * 
     * @param config TVC configuration parameters
     * @throws std::invalid_argument if configuration is invalid
     */
    explicit ThrustVectorControl(const Configuration& config = Configuration());
    
    /**
     * @brief Default destructor
     */
    ~ThrustVectorControl() = default;
    
    // Configuration access
    const Configuration& getConfiguration() const { return m_config; }
    const State& getState() const { return m_state; }
    
    /**
     * @brief Set commanded gimbal angles
     * 
     * The angles will be clamped to the configured limits. The actual
     * gimbal angles will slew to the commanded values based on rate limits.
     * 
     * @param pitch Commanded pitch angle (rad)
     * @param yaw Commanded yaw angle (rad)
     * @return True if commands were within limits (before clamping)
     */
    bool setCommandedAngles(double pitch, double yaw);
    
    /**
     * @brief Update gimbal dynamics
     * 
     * Call this each timestep to update the gimbal positions based on
     * commanded angles and rate limits.
     * 
     * @param deltaTime Time step (s)
     */
    void update(double deltaTime);
    
    /**
     * @brief Transform thrust vector by current gimbal angles
     * 
     * @param nominalThrust Thrust vector in engine frame (typically [0,0,-F])
     * @return Thrust vector in body frame after gimbal transformation
     */
    math::Vector3D transformThrustVector(const math::Vector3D& nominalThrust) const;
    
    /**
     * @brief Get the rotation matrix for current gimbal angles
     * 
     * @return 3x3 rotation matrix from engine to body frame
     */
    math::Matrix3D getGimbalRotationMatrix() const;
    
    /**
     * @brief Calculate gimbal angles for desired thrust direction
     * 
     * Given a desired thrust direction, calculate the required gimbal angles.
     * This is the inverse kinematics problem for the gimbal system.
     * 
     * @param desiredDirection Desired thrust direction (unit vector in body frame)
     * @param[out] pitch Required pitch angle (rad)
     * @param[out] yaw Required yaw angle (rad)
     * @return True if the desired direction is achievable within limits
     */
    bool calculateRequiredAngles(const math::Vector3D& desiredDirection,
                                double& pitch, double& yaw) const;
    
    /**
     * @brief Check if gimbal system is active
     * @return True if either pitch or yaw is enabled
     */
    bool isActive() const { return m_config.enablePitch || m_config.enableYaw; }
    
    /**
     * @brief Check if gimbal is at commanded position
     * 
     * @param tolerance Angular tolerance (rad)
     * @return True if current angles match commanded within tolerance
     */
    bool isAtCommandedPosition(double tolerance = 0.001) const;
    
    /**
     * @brief Get maximum deflection angle
     * 
     * @return Maximum combined gimbal angle possible (rad)
     */
    double getMaxDeflection() const;
    
    /**
     * @brief Reset gimbal to neutral position
     * 
     * Sets commanded angles to zero and optionally snaps current
     * angles to zero immediately.
     * 
     * @param immediate If true, bypass rate limits and snap to zero
     */
    void reset(bool immediate = false);
    
    /**
     * @brief Get gimbal authority
     * 
     * Calculate the maximum lateral acceleration that can be produced
     * by gimbaling, as a fraction of the axial acceleration.
     * 
     * @return Gimbal authority (typically 0.05-0.10 for 5-10% authority)
     */
    double getGimbalAuthority() const;
    
    /**
     * @brief Validate configuration
     * @return True if configuration is valid
     */
    bool validate() const;
    
    /**
     * @brief Get string representation of TVC state
     * @return Human-readable TVC status
     */
    std::string toString() const;
    
    /**
     * @brief Create a copy of this TVC system
     * @return Unique pointer to cloned TVC
     */
    std::unique_ptr<ThrustVectorControl> clone() const;

private:
    Configuration m_config;  ///< TVC configuration
    State m_state;          ///< Current TVC state
    
    /**
     * @brief Validate configuration parameters
     * @throws std::invalid_argument if parameters invalid
     */
    void validateConfiguration() const;
    
    /**
     * @brief Apply rate limiting to gimbal motion
     * 
     * @param current Current angle (rad)
     * @param commanded Commanded angle (rad)
     * @param maxRate Maximum rate (rad/s)
     * @param deltaTime Time step (s)
     * @return New angle after rate limiting
     */
    double applyRateLimit(double current, double commanded, 
                         double maxRate, double deltaTime) const;
    
    /**
     * @brief Model actuator dynamics (optional, for high-fidelity)
     * 
     * @param current Current position
     * @param commanded Commanded position
     * @param currentRate Current rate
     * @param deltaTime Time step
     * @return New position after actuator dynamics
     */
    double modelActuatorDynamics(double current, double commanded,
                                double& currentRate, double deltaTime) const;
    
    /**
     * @brief Clamp angle to limits
     * 
     * @param angle Input angle (rad)
     * @param limit Maximum allowed angle (rad)
     * @return Clamped angle
     */
    static double clampAngle(double angle, double limit);
};

} // namespace propulsion
} // namespace physics
} // namespace iloss