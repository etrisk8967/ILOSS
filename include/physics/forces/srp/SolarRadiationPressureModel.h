#pragma once

#include "physics/forces/IAttitudeAwareForceModel.h"
#include "core/coordinates/CoordinateTransformer.h"
#include "core/math/Matrix3D.h"
#include <memory>
#include <vector>

namespace iloss {
namespace physics {
namespace forces {
namespace srp {

/**
 * @brief Shadow model types for solar radiation pressure calculations
 */
enum class ShadowModelType {
    None,           ///< No shadow (always in sunlight)
    Cylindrical,    ///< Simple cylindrical shadow model
    Conical,        ///< Conical shadow with umbra and penumbra
    Dual            ///< Both Earth and Moon shadows
};

/**
 * @brief Eclipse event information
 */
struct EclipseEvent {
    time::Time startTime;       ///< Start time of eclipse
    time::Time endTime;         ///< End time of eclipse
    time::Time maxTime;         ///< Time of maximum eclipse
    double maxObscuration;      ///< Maximum obscuration fraction (0-1)
    enum Type {
        None,       ///< No eclipse
        Partial,    ///< Partial eclipse (penumbra)
        Total,      ///< Total eclipse (umbra)
        Annular     ///< Annular eclipse (object behind Earth but visible)
    } type;
    
    /**
     * @brief Get duration of eclipse in seconds
     */
    double getDuration() const {
        return endTime.getTime() - startTime.getTime();
    }
};

/**
 * @brief Solar radiation pressure force model with attitude awareness
 * 
 * This class implements solar radiation pressure (SRP) force calculation for spacecraft
 * and other objects in space. The SRP force is computed using:
 * 
 * F_srp = -P * A * Cr * cos(θ) * shadow * ŝ
 * 
 * where:
 * - P is the solar radiation pressure at the object's distance (N/m²)
 * - A is the cross-sectional area exposed to the Sun (m²)
 * - Cr is the radiation pressure coefficient (1.0 for absorption, 2.0 for reflection)
 * - θ is the angle between surface normal and Sun direction
 * - shadow is the shadow function (0-1)
 * - ŝ is the unit vector from object to Sun
 * 
 * The model includes:
 * - Conical shadow model with umbra and penumbra
 * - Solar flux variations with distance
 * - Eclipse detection and prediction
 * - Support for various surface reflectivity models
 * - Attitude-dependent area calculations when available
 */
class SolarRadiationPressureModel : public IAttitudeAwareForceModel {
public:
    /**
     * @brief Constructor
     * @param shadowModel Type of shadow model to use
     */
    explicit SolarRadiationPressureModel(ShadowModelType shadowModel = ShadowModelType::Conical);

    /**
     * @brief Destructor
     */
    ~SolarRadiationPressureModel() override = default;

    /**
     * @brief Calculate the acceleration due to solar radiation pressure
     * @param state Current state vector of the object
     * @param time Current time
     * @return Acceleration vector in m/s²
     * 
     * This method uses the reference area and surface normal when attitude is not available.
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
     * When attitude is available, this method calculates the actual
     * cross-sectional area and surface orientation based on the spacecraft's
     * attitude relative to the Sun direction. This provides more accurate
     * SRP calculations for non-spherical spacecraft.
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
     * - "reflectivity_coefficient" (double): Cr coefficient (1.0-2.0)
     * - "area" (double): Cross-sectional area in m²
     * - "area_to_mass_ratio" (double): Optional, A/m ratio (m²/kg)
     * - "shadow_model" (string): "none", "cylindrical", "conical", "dual"
     * - "enable_flux_variation" (bool): Include solar distance variation (default: true)
     * - "solar_flux_au" (double): Solar flux at 1 AU (default: 1367 W/m²)
     * - "speed_of_light" (double): Speed of light (default: 299792458 m/s)
     * - "include_moon_shadow" (bool): Include lunar eclipses (default: false)
     * - "surface_normal" (Vector3D): Surface normal in body frame (default: [1,0,0])
     */
    bool initialize(const ForceModelConfig& config) override;

    /**
     * @brief Update time-dependent parameters
     * @param time Current time
     * 
     * Updates Sun position and solar flux based on current time
     */
    void update(const time::Time& time) override;

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

    // Shadow calculations
    /**
     * @brief Calculate shadow function at given position
     * @param objectPosition Position of object in inertial frame (m)
     * @param time Current time
     * @return Shadow function value (0=full shadow, 1=full sunlight)
     */
    double calculateShadowFunction(
        const math::Vector3D& objectPosition,
        const time::Time& time) const;

    /**
     * @brief Check if object is in eclipse
     * @param objectPosition Position of object in inertial frame (m)
     * @param time Current time
     * @return True if object is in Earth's or Moon's shadow
     */
    bool isInEclipse(
        const math::Vector3D& objectPosition,
        const time::Time& time) const;

    /**
     * @brief Predict next eclipse event
     * @param state Current state vector
     * @param startTime Start time for search
     * @param searchDuration Duration to search (seconds)
     * @param timeStep Time step for search (seconds)
     * @return Eclipse event information
     */
    EclipseEvent predictNextEclipse(
        const StateVector& state,
        const time::Time& startTime,
        double searchDuration = 86400.0,  // 1 day
        double timeStep = 60.0) const;     // 1 minute

    // Getters
    double getReflectivityCoefficient() const { return m_reflectivityCoefficient; }
    double getCrossSectionalArea() const { return m_area; }
    double getAreaToMassRatio() const { return m_areaToMassRatio; }
    ShadowModelType getShadowModel() const { return m_shadowModel; }
    bool isFluxVariationEnabled() const { return m_enableFluxVariation; }
    double getSolarFluxAU() const { return m_solarFluxAU; }
    bool isMoonShadowEnabled() const { return m_includeMoonShadow; }
    const math::Vector3D& getSurfaceNormal() const { return m_surfaceNormal; }

    // Setters
    void setReflectivityCoefficient(double cr);
    void setCrossSectionalArea(double area);
    void setAreaToMassRatio(double ratio);
    void setShadowModel(ShadowModelType model);
    void setFluxVariation(bool enable);
    void setSolarFluxAU(double flux);
    void setMoonShadow(bool enable);
    void setSurfaceNormal(const math::Vector3D& normal);

    /**
     * @brief Get the reference area for SRP calculation
     * @return Reference area in m²
     * 
     * This returns the fixed cross-sectional area used when attitude
     * information is not available.
     */
    double getReferenceArea() const override { return m_area; }

    /**
     * @brief Set the reference area for SRP calculation
     * @param area Reference area in m²
     * 
     * This sets the fixed cross-sectional area used when attitude
     * information is not available.
     */
    void setReferenceArea(double area) override { setCrossSectionalArea(area); }

    // Solar information getters (updated by update() method)
    const math::Vector3D& getSunPosition() const { return m_sunPosition; }
    double getSunDistance() const { return m_sunDistance; }
    double getCurrentSolarFlux() const { return m_currentSolarFlux; }

protected:
    /**
     * @brief Calculate Sun position in inertial frame
     * @param time Current time
     * @return Sun position vector (m)
     */
    math::Vector3D calculateSunPosition(const time::Time& time) const;

    /**
     * @brief Calculate Moon position in inertial frame
     * @param time Current time
     * @return Moon position vector (m)
     */
    math::Vector3D calculateMoonPosition(const time::Time& time) const;

    /**
     * @brief Calculate solar flux at given distance
     * @param distance Distance from Sun (m)
     * @return Solar flux (W/m²)
     */
    double calculateSolarFlux(double distance) const;

    /**
     * @brief Calculate cylindrical shadow function
     * @param objectPos Object position (m)
     * @param sunPos Sun position (m)
     * @param earthPos Earth position (m)
     * @return Shadow function (0-1)
     */
    double calculateCylindricalShadow(
        const math::Vector3D& objectPos,
        const math::Vector3D& sunPos,
        const math::Vector3D& earthPos = math::Vector3D()) const;

    /**
     * @brief Calculate conical shadow function with penumbra
     * @param objectPos Object position (m)
     * @param sunPos Sun position (m)
     * @param occludingBody Position of occulding body (m)
     * @param occludingRadius Radius of occluding body (m)
     * @return Shadow function (0-1)
     */
    double calculateConicalShadow(
        const math::Vector3D& objectPos,
        const math::Vector3D& sunPos,
        const math::Vector3D& occludingBody,
        double occludingRadius) const;

    /**
     * @brief Calculate effective area based on Sun angle
     * @param sunDirection Unit vector towards Sun
     * @param surfaceNormal Surface normal vector
     * @return Effective area (m²)
     */
    double calculateEffectiveArea(
        const math::Vector3D& sunDirection,
        const math::Vector3D& surfaceNormal) const;

    /**
     * @brief Calculate attitude-dependent cross-sectional area
     * @param sunDirection Unit vector towards Sun (body frame)
     * @param attitude Current attitude quaternion
     * @return Effective cross-sectional area in m²
     * 
     * This method can be overridden by derived classes to implement
     * specific area models for different spacecraft geometries.
     * The default implementation uses the surface normal and flat plate model.
     */
    virtual double calculateAttitudeDependentArea(
        const math::Vector3D& sunDirection,
        const math::Quaternion& attitude) const;

private:
    // Configuration parameters
    double m_reflectivityCoefficient;       ///< Radiation pressure coefficient (1.0-2.0)
    double m_area;                          ///< Cross-sectional area (m²)
    double m_areaToMassRatio;               ///< Area-to-mass ratio (m²/kg)
    bool m_useAreaToMassRatio;              ///< Use A/m ratio instead of A
    ShadowModelType m_shadowModel;          ///< Type of shadow model
    bool m_enableFluxVariation;             ///< Include solar distance variation
    double m_solarFluxAU;                   ///< Solar flux at 1 AU (W/m²)
    double m_speedOfLight;                  ///< Speed of light (m/s)
    bool m_includeMoonShadow;               ///< Include lunar eclipses
    math::Vector3D m_surfaceNormal;         ///< Surface normal in body frame

    // Time-dependent cached values
    mutable math::Vector3D m_sunPosition;   ///< Current Sun position (m)
    mutable double m_sunDistance;           ///< Current distance to Sun (m)
    mutable double m_currentSolarFlux;      ///< Current solar flux (W/m²)
    mutable time::Time m_lastUpdateTime;    ///< Last time update was called
    
    // Dependencies
    std::unique_ptr<coordinates::CoordinateTransformer> m_coordinateTransformer;

    // Physical constants
    static constexpr double AU = 1.495978707e11;           ///< Astronomical unit (m)
    static constexpr double SOLAR_RADIUS = 6.96e8;         ///< Solar radius (m)
    static constexpr double EARTH_RADIUS = 6.371e6;        ///< Earth radius (m)
    static constexpr double MOON_RADIUS = 1.737e6;         ///< Moon radius (m)
    static constexpr double DEFAULT_SOLAR_FLUX = 1367.0;   ///< Solar constant (W/m²)
    static constexpr double DEFAULT_SPEED_OF_LIGHT = 299792458.0; ///< Speed of light (m/s)
};

} // namespace srp
} // namespace forces
} // namespace physics
} // namespace iloss