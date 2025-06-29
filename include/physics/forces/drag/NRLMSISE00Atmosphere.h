#pragma once

#include "core/constants/AtmosphericModel.h"
#include <array>
#include <memory>

namespace iloss {
namespace physics {
namespace forces {
namespace drag {

/**
 * @brief Space weather parameters for NRLMSISE-00 model
 * 
 * This structure contains the solar and magnetic activity indices
 * required by the NRLMSISE-00 atmospheric model.
 */
struct SpaceWeatherData {
    double f107;           ///< Daily F10.7 solar flux (10^-22 W/m²/Hz)
    double f107a;          ///< 81-day average F10.7 solar flux
    double ap;             ///< Daily magnetic index
    std::array<double, 7> ap_array;  ///< AP history array
    
    /**
     * @brief Default constructor with typical values
     */
    SpaceWeatherData() 
        : f107(150.0)      // Typical solar flux
        , f107a(150.0)     // Typical average
        , ap(4.0)          // Quiet magnetic conditions
        , ap_array{{4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0}} {
    }
};

/**
 * @brief NRLMSISE-00 atmospheric model interface
 * 
 * The Naval Research Laboratory Mass Spectrometer and Incoherent Scatter
 * Radar Exosphere (NRLMSISE-00) model is an empirical, global model of
 * the Earth's atmosphere from ground to space. It models the temperatures
 * and densities of the atmosphere's components.
 * 
 * This class provides an interface to the NRLMSISE-00 model. The actual
 * implementation can use either a native implementation or wrap an
 * external library.
 * 
 * @note The full implementation of NRLMSISE-00 requires extensive data
 * tables and complex calculations. This interface allows for future
 * implementation or integration with existing libraries.
 */
class NRLMSISE00Atmosphere : public constants::AtmosphericModel {
public:
    /**
     * @brief Output structure for NRLMSISE-00 calculations
     */
    struct ModelOutput {
        double density_total;    ///< Total mass density (kg/m³)
        double temperature_neutral;  ///< Neutral temperature (K)
        double temperature_exospheric; ///< Exospheric temperature (K)
        
        // Number densities (particles/m³)
        double density_He;       ///< Helium
        double density_O;        ///< Atomic oxygen
        double density_N2;       ///< Molecular nitrogen
        double density_O2;       ///< Molecular oxygen
        double density_Ar;       ///< Argon
        double density_H;        ///< Hydrogen
        double density_N;        ///< Atomic nitrogen
        double density_anomalous_O;  ///< Anomalous oxygen
    };

    /**
     * @brief Constructor
     */
    NRLMSISE00Atmosphere();

    /**
     * @brief Destructor
     */
    ~NRLMSISE00Atmosphere() override = default;

    /**
     * @brief Get atmospheric density at a given position and time
     * @param position Position in ECEF coordinates (meters)
     * @param julianDate Julian date (days)
     * @return Atmospheric density (kg/m³)
     */
    double getDensity(const math::Vector3D& position, double julianDate) const override;

    /**
     * @brief Get atmospheric temperature at a given position and time
     * @param position Position in ECEF coordinates (meters)
     * @param julianDate Julian date (days)
     * @return Temperature (K)
     */
    double getTemperature(const math::Vector3D& position, double julianDate) const override;

    /**
     * @brief Get atmospheric pressure at a given position and time
     * @param position Position in ECEF coordinates (meters)
     * @param julianDate Julian date (days)
     * @return Pressure (Pa)
     */
    double getPressure(const math::Vector3D& position, double julianDate) const override;

    /**
     * @brief Get the name of the atmospheric model
     * @return Model name
     */
    std::string getName() const override { return "NRLMSISE-00"; }

    /**
     * @brief Check if the model is valid at the given altitude
     * @param altitude Altitude above reference ellipsoid (meters)
     * @return True if model is valid at this altitude
     */
    bool isValidAltitude(double altitude) const override;

    /**
     * @brief Get the minimum valid altitude for this model
     * @return Minimum altitude (meters)
     */
    double getMinAltitude() const override { return 0.0; }

    /**
     * @brief Get the maximum valid altitude for this model
     * @return Maximum altitude (meters)
     */
    double getMaxAltitude() const override { return 1000000.0; }  // 1000 km

    /**
     * @brief Set space weather data
     * @param data Space weather parameters
     */
    void setSpaceWeather(const SpaceWeatherData& data);

    /**
     * @brief Get current space weather data
     * @return Space weather parameters
     */
    const SpaceWeatherData& getSpaceWeather() const { return m_spaceWeather; }

    /**
     * @brief Get detailed model output
     * @param position Position in ECEF coordinates (meters)
     * @param julianDate Julian date (days)
     * @return Detailed atmospheric properties
     */
    ModelOutput getDetailedOutput(const math::Vector3D& position, double julianDate) const;

    /**
     * @brief Enable or disable specific atmospheric components
     * @param component Component index (0-8)
     * @param enable True to enable, false to disable
     */
    void setComponentEnabled(int component, bool enable);

private:
    /**
     * @brief Convert ECEF position to geodetic coordinates
     * @param position ECEF position (meters)
     * @param[out] lat Geodetic latitude (radians)
     * @param[out] lon Geodetic longitude (radians)
     * @param[out] alt Altitude above ellipsoid (meters)
     */
    void ecefToGeodetic(const math::Vector3D& position, 
                       double& lat, double& lon, double& alt) const;

    /**
     * @brief Calculate local solar time
     * @param lon Longitude (radians)
     * @param julianDate Julian date
     * @return Local solar time (hours)
     */
    double calculateLocalSolarTime(double lon, double julianDate) const;

    /**
     * @brief Calculate model parameters
     * 
     * This is where the actual NRLMSISE-00 calculations would be performed.
     * For now, this returns reasonable default values.
     * 
     * @param lat Geodetic latitude (radians)
     * @param lon Geodetic longitude (radians)
     * @param alt Altitude (meters)
     * @param julianDate Julian date
     * @return Model output structure
     */
    ModelOutput calculateModelOutput(double lat, double lon, double alt, 
                                   double julianDate) const;

private:
    SpaceWeatherData m_spaceWeather;     ///< Space weather parameters
    std::array<bool, 9> m_switches;      ///< Model component switches
    
    // Model constants
    static constexpr double AVOGADRO = 6.022140857e23;  ///< Avogadro's number
    static constexpr double GAS_CONSTANT = 8.3144598;   ///< Universal gas constant (J/mol/K)
    
    // Molecular weights (kg/mol)
    static constexpr double MW_HE = 4.0026e-3;
    static constexpr double MW_O = 15.9994e-3;
    static constexpr double MW_N2 = 28.0134e-3;
    static constexpr double MW_O2 = 31.9988e-3;
    static constexpr double MW_AR = 39.948e-3;
    static constexpr double MW_H = 1.00794e-3;
    static constexpr double MW_N = 14.0067e-3;
};

} // namespace drag
} // namespace forces
} // namespace physics
} // namespace iloss