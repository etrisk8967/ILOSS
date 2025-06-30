#pragma once

#include "physics/forces/IAttitudeAwareForceModel.h"
#include "core/constants/AtmosphericModel.h"
#include "core/coordinates/CoordinateTransformer.h"
#include <memory>

namespace iloss {
namespace physics {
namespace forces {
namespace drag {

/**
 * @brief Atmospheric drag force model with attitude awareness
 * 
 * This class implements atmospheric drag force calculation for spacecraft
 * and other objects moving through the atmosphere. The drag force is computed
 * using the standard aerodynamic drag equation:
 * 
 * F_drag = -0.5 * Cd * A * ρ * v_rel² * v̂_rel
 * 
 * where:
 * - Cd is the drag coefficient
 * - A is the cross-sectional area (can be attitude-dependent)
 * - ρ is the atmospheric density
 * - v_rel is the relative velocity between object and atmosphere
 * - v̂_rel is the unit vector in the direction of relative velocity
 * 
 * The model supports multiple atmospheric density models and can account
 * for atmospheric rotation and wind. When attitude information is available,
 * it can calculate the actual cross-sectional area based on the spacecraft's
 * orientation relative to the velocity vector.
 */
class DragForceModel : public IAttitudeAwareForceModel {
public:
    /**
     * @brief Constructor
     * @param atmosphericModel Atmospheric model to use for density calculations
     */
    explicit DragForceModel(std::shared_ptr<constants::AtmosphericModel> atmosphericModel);

    /**
     * @brief Default constructor using exponential atmosphere
     */
    DragForceModel();

    /**
     * @brief Destructor
     */
    ~DragForceModel() override = default;

    /**
     * @brief Calculate the acceleration due to atmospheric drag
     * @param state Current state vector of the object
     * @param time Current time
     * @return Acceleration vector in m/s²
     * 
     * This method uses the reference area when attitude is not available.
     */
    math::Vector3D calculateAcceleration(
        const StateVector& state,
        const time::Time& time) const override;

    /**
     * @brief Calculate acceleration with attitude-dependent area
     * @param state Full dynamics state including attitude
     * @param time Current time
     * @return Acceleration vector in m/s²
     * 
     * When attitude is available, this method can calculate the actual
     * cross-sectional area based on the spacecraft's orientation relative
     * to the velocity vector. This provides more accurate drag calculations
     * for non-spherical spacecraft.
     */
    math::Vector3D calculateAccelerationWithAttitude(
        const dynamics::DynamicsState& state,
        const time::Time& time) const override;

    /**
     * @brief Initialize the force model with configuration
     * @param config Configuration parameters
     * @return True if initialization successful
     * 
     * Expected configuration parameters:
     * - "drag_coefficient" (double): Drag coefficient (dimensionless)
     * - "area" (double): Cross-sectional area in m²
     * - "ballistic_coefficient" (double): Optional, overrides Cd*A/m if provided (m²/kg)
     * - "enable_atmospheric_rotation" (bool): Include atmospheric rotation (default: true)
     * - "enable_wind" (bool): Include wind effects (default: false)
     * - "wind_velocity" (Vector3D): Wind velocity in ECEF frame (m/s)
     */
    bool initialize(const ForceModelConfig& config) override;

    /**
     * @brief Validate the force model configuration
     * @return True if configuration is valid
     */
    bool validate() const override;

    /**
     * @brief Clone the force model
     * @return Unique pointer to cloned model
     */
    std::unique_ptr<ForceModel> clone() const override;

    /**
     * @brief Get a string representation of the force model
     * @return String description
     */
    std::string toString() const override;

    // Getters
    /**
     * @brief Get the drag coefficient
     * @return Drag coefficient (dimensionless)
     */
    double getDragCoefficient() const { return m_dragCoefficient; }

    /**
     * @brief Get the cross-sectional area
     * @return Area in m²
     */
    double getCrossSectionalArea() const { return m_area; }

    /**
     * @brief Get the ballistic coefficient
     * @return Ballistic coefficient in m²/kg
     */
    double getBallisticCoefficient() const { return m_ballisticCoefficient; }

    /**
     * @brief Check if atmospheric rotation is enabled
     * @return True if atmospheric rotation is considered
     */
    bool isAtmosphericRotationEnabled() const { return m_enableAtmosphericRotation; }

    /**
     * @brief Check if wind effects are enabled
     * @return True if wind is considered
     */
    bool isWindEnabled() const { return m_enableWind; }

    /**
     * @brief Get the wind velocity
     * @return Wind velocity in ECEF frame (m/s)
     */
    const math::Vector3D& getWindVelocity() const { return m_windVelocity; }

    /**
     * @brief Get the atmospheric model
     * @return Pointer to atmospheric model
     */
    std::shared_ptr<constants::AtmosphericModel> getAtmosphericModel() const { 
        return m_atmosphericModel; 
    }

    // Setters
    /**
     * @brief Set the drag coefficient
     * @param cd Drag coefficient (dimensionless)
     * @throws std::invalid_argument if cd is negative
     */
    void setDragCoefficient(double cd);

    /**
     * @brief Set the cross-sectional area
     * @param area Area in m²
     * @throws std::invalid_argument if area is negative
     */
    void setCrossSectionalArea(double area);

    /**
     * @brief Set the ballistic coefficient
     * @param bc Ballistic coefficient in m²/kg
     * @throws std::invalid_argument if bc is negative
     * @note Setting this overrides the use of separate Cd and A values
     */
    void setBallisticCoefficient(double bc);

    /**
     * @brief Enable or disable atmospheric rotation
     * @param enable True to include atmospheric rotation
     */
    void setAtmosphericRotation(bool enable);

    /**
     * @brief Enable or disable wind effects
     * @param enable True to include wind
     */
    void setWind(bool enable);

    /**
     * @brief Set the wind velocity
     * @param wind Wind velocity in ECEF frame (m/s)
     */
    void setWindVelocity(const math::Vector3D& wind);

    /**
     * @brief Set the atmospheric model
     * @param model New atmospheric model
     */
    void setAtmosphericModel(std::shared_ptr<constants::AtmosphericModel> model);

    /**
     * @brief Get the reference area for drag calculation
     * @return Reference area in m²
     * 
     * This returns the fixed cross-sectional area used when attitude
     * information is not available.
     */
    double getReferenceArea() const override { return m_area; }

    /**
     * @brief Set the reference area for drag calculation
     * @param area Reference area in m²
     * 
     * This sets the fixed cross-sectional area used when attitude
     * information is not available.
     */
    void setReferenceArea(double area) override { setCrossSectionalArea(area); }

protected:
    /**
     * @brief Calculate the relative velocity between object and atmosphere
     * @param state State vector of the object
     * @return Relative velocity vector in appropriate frame (m/s)
     */
    math::Vector3D calculateRelativeVelocity(const StateVector& state) const;

    /**
     * @brief Calculate atmospheric velocity due to Earth's rotation
     * @param position Position in ECEF coordinates (m)
     * @return Atmospheric velocity in ECEF frame (m/s)
     */
    math::Vector3D calculateAtmosphericVelocity(const math::Vector3D& position) const;

    /**
     * @brief Get the effective drag area coefficient (Cd * A)
     * @param mass Object mass in kg
     * @return Cd * A in m²
     */
    double getEffectiveDragArea(double mass) const;

    /**
     * @brief Calculate attitude-dependent cross-sectional area
     * @param velocityDirection Unit vector in direction of relative velocity (body frame)
     * @param attitude Current attitude quaternion
     * @return Effective cross-sectional area in m²
     * 
     * This method can be overridden by derived classes to implement
     * specific area models for different spacecraft geometries.
     * The default implementation returns the reference area.
     */
    virtual double calculateAttitudeDependentArea(
        const math::Vector3D& velocityDirection,
        const math::Quaternion& attitude) const;

private:
    // Configuration parameters
    double m_dragCoefficient;               ///< Drag coefficient (dimensionless)
    double m_area;                          ///< Cross-sectional area (m²)
    double m_ballisticCoefficient;          ///< Ballistic coefficient (m²/kg)
    bool m_useBallisticCoefficient;         ///< Use BC instead of Cd*A
    bool m_enableAtmosphericRotation;       ///< Include atmospheric rotation
    bool m_enableWind;                      ///< Include wind effects
    math::Vector3D m_windVelocity;          ///< Wind velocity in ECEF (m/s)

    // Dependencies
    std::shared_ptr<constants::AtmosphericModel> m_atmosphericModel;
    std::unique_ptr<coordinates::CoordinateTransformer> m_coordinateTransformer;

    // Earth rotation rate (rad/s)
    static constexpr double EARTH_ROTATION_RATE = 7.2921158553e-5;
};

} // namespace drag
} // namespace forces
} // namespace physics
} // namespace iloss