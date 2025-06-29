#include "physics/forces/drag/NRLMSISE00Atmosphere.h"
#include "core/constants/EarthModel.h"
#include "core/logging/Logger.h"
#include <cmath>
#include <algorithm>

namespace iloss {
namespace physics {
namespace forces {
namespace drag {

NRLMSISE00Atmosphere::NRLMSISE00Atmosphere() 
    : m_spaceWeather()
    , m_switches{{true, true, true, true, true, true, true, true, true}} {
    LOG_INFO("NRLMSISE00Atmosphere", "Initialized NRLMSISE-00 atmospheric model");
}

double NRLMSISE00Atmosphere::getDensity(const math::Vector3D& position, double julianDate) const {
    ModelOutput output = getDetailedOutput(position, julianDate);
    return output.density_total;
}

double NRLMSISE00Atmosphere::getTemperature(const math::Vector3D& position, double julianDate) const {
    ModelOutput output = getDetailedOutput(position, julianDate);
    return output.temperature_neutral;
}

double NRLMSISE00Atmosphere::getPressure(const math::Vector3D& position, double julianDate) const {
    ModelOutput output = getDetailedOutput(position, julianDate);
    
    // Calculate pressure using ideal gas law for the mixture
    // P = n * k * T where n is number density and k is Boltzmann constant
    double totalNumberDensity = output.density_He + output.density_O + 
                               output.density_N2 + output.density_O2 + 
                               output.density_Ar + output.density_H + 
                               output.density_N;
    
    const double BOLTZMANN = 1.380649e-23;  // J/K
    return totalNumberDensity * BOLTZMANN * output.temperature_neutral;
}

bool NRLMSISE00Atmosphere::isValidAltitude(double altitude) const {
    return altitude >= getMinAltitude() && altitude <= getMaxAltitude();
}

void NRLMSISE00Atmosphere::setSpaceWeather(const SpaceWeatherData& data) {
    m_spaceWeather = data;
    LOG_DEBUG("NRLMSISE00Atmosphere", 
              "Updated space weather: F10.7=" + std::to_string(data.f107) + 
              ", Ap=" + std::to_string(data.ap));
}

void NRLMSISE00Atmosphere::setComponentEnabled(int component, bool enable) {
    if (component >= 0 && component < 9) {
        m_switches[component] = enable;
    }
}

NRLMSISE00Atmosphere::ModelOutput NRLMSISE00Atmosphere::getDetailedOutput(
    const math::Vector3D& position, double julianDate) const {
    
    // Convert position to geodetic coordinates
    double lat, lon, alt;
    ecefToGeodetic(position, lat, lon, alt);
    
    // Calculate model output
    return calculateModelOutput(lat, lon, alt, julianDate);
}

void NRLMSISE00Atmosphere::ecefToGeodetic(const math::Vector3D& position, 
                                          double& lat, double& lon, double& alt) const {
    // Use Earth model for conversion
    double x = position.x();
    double y = position.y();
    double z = position.z();
    
    // Calculate longitude
    lon = std::atan2(y, x);
    
    // Calculate latitude and altitude using iterative method
    double p = std::sqrt(x * x + y * y);
    double theta = std::atan2(z * constants::EarthModel::EQUATORIAL_RADIUS, 
                             p * constants::EarthModel::POLAR_RADIUS);
    
    double sin_theta = std::sin(theta);
    double cos_theta = std::cos(theta);
    
    lat = std::atan2(z + constants::EarthModel::ECCENTRICITY_SQUARED * 
                     constants::EarthModel::POLAR_RADIUS * sin_theta * sin_theta * sin_theta,
                     p - constants::EarthModel::ECCENTRICITY_SQUARED * 
                     constants::EarthModel::EQUATORIAL_RADIUS * cos_theta * cos_theta * cos_theta);
    
    double sin_lat = std::sin(lat);
    double N = constants::EarthModel::EQUATORIAL_RADIUS / 
               std::sqrt(1.0 - constants::EarthModel::ECCENTRICITY_SQUARED * sin_lat * sin_lat);
    
    if (std::abs(lat) < M_PI / 4) {
        alt = p / std::cos(lat) - N;
    } else {
        alt = z / sin_lat - N * (1.0 - constants::EarthModel::ECCENTRICITY_SQUARED);
    }
}

double NRLMSISE00Atmosphere::calculateLocalSolarTime(double lon, double julianDate) const {
    // Convert Julian date to hours since J2000
    double daysSinceJ2000 = julianDate - 2451545.0;
    double centuriesSinceJ2000 = daysSinceJ2000 / 36525.0;
    
    // Calculate Greenwich Mean Sidereal Time (GMST) in degrees
    double gmst = 280.46061837 + 360.98564736629 * daysSinceJ2000 +
                  0.000387933 * centuriesSinceJ2000 * centuriesSinceJ2000 -
                  centuriesSinceJ2000 * centuriesSinceJ2000 * centuriesSinceJ2000 / 38710000.0;
    
    // Normalize to [0, 360)
    gmst = std::fmod(gmst, 360.0);
    if (gmst < 0) gmst += 360.0;
    
    // Calculate local solar time in hours
    double lst = gmst / 15.0 + lon * 180.0 / M_PI / 15.0;
    
    // Normalize to [0, 24)
    lst = std::fmod(lst, 24.0);
    if (lst < 0) lst += 24.0;
    
    return lst;
}

NRLMSISE00Atmosphere::ModelOutput NRLMSISE00Atmosphere::calculateModelOutput(
    double lat, double lon, double alt, double julianDate) const {
    
    ModelOutput output;
    
    // This is a simplified implementation that provides reasonable atmospheric
    // density values based on altitude. The full NRLMSISE-00 model would include:
    // - Complex spherical harmonic expansions
    // - Solar activity dependencies
    // - Magnetic activity dependencies
    // - Seasonal and diurnal variations
    // - Latitude dependencies
    
    // For now, we'll use an enhanced exponential model with some realistic features
    
    // Convert altitude to km for calculations
    double alt_km = alt / 1000.0;
    
    // Base scale heights and densities for different altitude regions
    struct AtmosphereLayer {
        double alt_base;    // Base altitude (km)
        double alt_top;     // Top altitude (km)
        double density_base; // Base density (kg/m³)
        double scale_height; // Scale height (km)
        double temp_base;   // Base temperature (K)
        double temp_gradient; // Temperature gradient (K/km)
    };
    
    // Define atmospheric layers
    const AtmosphereLayer layers[] = {
        {0,    11,   1.225,     8.5,   288.15, -6.5},    // Troposphere
        {11,   20,   0.36391,   6.3,   216.65,  0.0},    // Lower Stratosphere
        {20,   32,   0.08803,   6.3,   216.65,  1.0},    // Upper Stratosphere
        {32,   47,   0.01322,   6.4,   228.65,  2.8},    // Lower Mesosphere
        {47,   51,   0.00143,   6.7,   270.65,  0.0},    // Upper Mesosphere
        {51,   71,   0.00086,   7.0,   270.65, -2.8},    // Lower Thermosphere
        {71,   84.852, 0.000064, 7.5,  214.65, -2.0},    // Upper Thermosphere base
        {84.852, 1000, 0.0000065, 50.0, 186.95, 0.0}     // Exosphere
    };
    
    // Find the appropriate layer
    const AtmosphereLayer* layer = nullptr;
    for (const auto& l : layers) {
        if (alt_km >= l.alt_base && alt_km < l.alt_top) {
            layer = &l;
            break;
        }
    }
    
    if (!layer) {
        // Above model range - use very thin exosphere
        double density, temp;
        if (alt_km <= 1000.0) {
            // Still within valid range, use exponential decay
            double density_at_last_layer = 0.0000065;
            double scale_height_km = 100.0;  // Very large scale height in exosphere
            double height_above_base = alt - 84.852 * 1000.0;
            density = density_at_last_layer * std::exp(-height_above_base / (scale_height_km * 1000.0));
            temp = 186.95 + 0.1 * (alt_km - 84.852);  // Slight temperature increase
        } else {
            // Above model range
            density = 0.0;
            temp = 1000.0;
        }
        output.density_total = density;
        output.temperature_neutral = temp;
        output.temperature_exospheric = 1000.0 + 800.0 * (m_spaceWeather.f107 - 70.0) / 180.0;
        return output;
    }
    
    // Calculate temperature
    double temp = layer->temp_base + layer->temp_gradient * (alt_km - layer->alt_base);
    output.temperature_neutral = temp;
    
    // Calculate density with exponential decay
    double density_at_base = layer->density_base;
    double scale_height_m = layer->scale_height * 1000.0;
    double height_above_base = alt - layer->alt_base * 1000.0;
    
    // Basic exponential atmosphere
    double density = density_at_base * std::exp(-height_above_base / scale_height_m);
    
    // Apply solar activity correction (simplified)
    double f107_correction = 1.0 + 0.5 * (m_spaceWeather.f107 - 150.0) / 150.0;
    if (alt_km > 100.0) {  // Only apply at high altitudes
        density *= f107_correction;
    }
    
    // Apply latitude variation (simplified)
    double lat_factor = 1.0 - 0.1 * std::cos(2.0 * lat);  // Bulge at equator
    if (alt_km > 200.0) {
        density *= lat_factor;
    }
    
    // Apply diurnal variation (simplified)
    double lst = calculateLocalSolarTime(lon, julianDate);
    double diurnal_factor = 1.0;
    if (alt_km > 150.0) {
        // Maximum density at 14:00 local solar time
        diurnal_factor = 1.0 + 0.3 * std::cos((lst - 14.0) * M_PI / 12.0);
        density *= diurnal_factor;
    }
    
    output.density_total = density;
    
    // Set exospheric temperature based on solar activity
    output.temperature_exospheric = 1000.0 + 800.0 * (m_spaceWeather.f107 - 70.0) / 180.0;
    
    // Calculate species densities (simplified - actual model is much more complex)
    // These are rough approximations of the composition at different altitudes
    if (alt_km < 100.0) {
        // Below 100 km: mostly N2 and O2
        double mean_molecular_weight = 0.78 * MW_N2 + 0.21 * MW_O2 + 0.01 * MW_AR;
        double total_number_density = density / mean_molecular_weight * AVOGADRO;
        
        output.density_N2 = 0.78 * total_number_density;
        output.density_O2 = 0.21 * total_number_density;
        output.density_Ar = 0.01 * total_number_density;
        output.density_O = 0.0;
        output.density_He = 0.0;
        output.density_H = 0.0;
        output.density_N = 0.0;
    } else if (alt_km < 200.0) {
        // 100-200 km: transition region
        double mean_molecular_weight = (MW_N2 + MW_O2 + MW_O) / 3.0;
        double total_number_density = density / mean_molecular_weight * AVOGADRO;
        
        double o_fraction = (alt_km - 100.0) / 100.0;
        output.density_O = o_fraction * 0.5 * total_number_density;
        output.density_N2 = (1.0 - o_fraction * 0.5) * 0.6 * total_number_density;
        output.density_O2 = (1.0 - o_fraction * 0.7) * 0.4 * total_number_density;
        output.density_Ar = 0.0;
        output.density_He = 0.0;
        output.density_H = 0.0;
        output.density_N = 0.0;
    } else {
        // Above 200 km: atomic oxygen dominates, with He and H at very high altitudes
        double mean_molecular_weight = MW_O;
        if (alt_km > 500.0) {
            mean_molecular_weight = (MW_O + MW_HE + MW_H) / 3.0;
        }
        
        double total_number_density = density / mean_molecular_weight * AVOGADRO;
        
        if (alt_km < 500.0) {
            output.density_O = 0.9 * total_number_density;
            output.density_N2 = 0.05 * total_number_density;
            output.density_He = 0.04 * total_number_density;
            output.density_O2 = 0.01 * total_number_density;
            output.density_H = 0.0;
        } else {
            // Very high altitudes
            output.density_He = 0.5 * total_number_density;
            output.density_H = 0.3 * total_number_density;
            output.density_O = 0.2 * total_number_density;
            output.density_N2 = 0.0;
            output.density_O2 = 0.0;
        }
        
        output.density_Ar = 0.0;
        output.density_N = 0.0;
    }
    
    output.density_anomalous_O = 0.0;  // Not modeled in this simplified version
    
    LOG_TRACE("NRLMSISE00Atmosphere", 
              "Calculated density at " + std::to_string(alt_km) + " km: " +
              std::to_string(density) + " kg/m³, T=" + std::to_string(temp) + " K");
    
    return output;
}

} // namespace drag
} // namespace forces
} // namespace physics
} // namespace iloss