#pragma once

#include "physics/aerodynamics/AerodynamicCoefficients.h"
#include <vector>
#include <string>
#include <memory>
#include <map>
#include <optional>
#include <mutex>

namespace iloss {
namespace physics {
namespace aerodynamics {

/**
 * @brief Container for a single data point in the aerodynamic database
 * 
 * Represents aerodynamic coefficients at a specific Mach number and angle of attack
 */
struct AerodynamicDataPoint {
    double mach;              ///< Mach number
    double angleOfAttack;     ///< Angle of attack in radians
    AerodynamicCoefficients coefficients;  ///< Coefficients at this condition
    
    /**
     * @brief Constructor
     * 
     * @param m Mach number
     * @param aoa Angle of attack in radians
     * @param coeffs Aerodynamic coefficients
     */
    AerodynamicDataPoint(double m, double aoa, const AerodynamicCoefficients& coeffs)
        : mach(m), angleOfAttack(aoa), coefficients(coeffs) {}
};

/**
 * @brief Aerodynamic database for loading and interpolating coefficient data
 * 
 * This class manages aerodynamic coefficient data loaded from CSV files and
 * provides efficient 2D interpolation based on Mach number and angle of attack.
 * The database supports multiple vehicle configurations and caching for performance.
 * 
 * CSV Format:
 * ```
 * # Comments start with #
 * # Mach, AoA_deg, CD, CL, CY, Cl, Cm, Cn
 * 0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 0.0, 0.0
 * 0.0, 5.0, 0.16, 0.25, 0.0, 0.0, -0.01, 0.0
 * ...
 * ```
 * 
 * Features:
 * - Bilinear interpolation for smooth coefficient variation
 * - Extrapolation with warning for out-of-range queries
 * - Caching of recently accessed values
 * - Thread-safe access for parallel simulations
 * - Support for sparse data with adaptive interpolation
 * 
 * @note Angles in CSV files are in degrees but stored internally in radians
 */
class AerodynamicDatabase {
public:
    /**
     * @brief Configuration options for the database
     */
    struct Config {
        bool enableCaching = true;          ///< Enable coefficient caching
        size_t cacheSize = 1000;           ///< Maximum cache entries
        bool allowExtrapolation = true;     ///< Allow extrapolation beyond data
        double extrapolationWarningThreshold = 0.1; ///< Warn if extrapolating > 10%
        bool validateMonotonicity = false;  ///< Check for monotonic Mach/AoA
    };

    /**
     * @brief Default constructor
     */
    AerodynamicDatabase();

    /**
     * @brief Constructor with configuration
     * 
     * @param config Database configuration options
     */
    explicit AerodynamicDatabase(const Config& config);

    /**
     * @brief Destructor
     */
    ~AerodynamicDatabase();

    /**
     * @brief Load aerodynamic data from a CSV file
     * 
     * The CSV file should have the format:
     * Mach, AoA_deg, CD, CL, CY, Cl, Cm, Cn
     * 
     * @param filename Path to the CSV file
     * @param configName Optional name for this configuration (e.g., "clean", "gear_down")
     * @throws std::runtime_error if file cannot be read or parsed
     */
    void loadFromCSV(const std::string& filename, 
                     const std::string& configName = "default");

    /**
     * @brief Load aerodynamic data from a string (for testing)
     * 
     * @param csvData CSV data as a string
     * @param configName Optional name for this configuration
     * @throws std::runtime_error if data cannot be parsed
     */
    void loadFromString(const std::string& csvData,
                       const std::string& configName = "default");

    /**
     * @brief Get interpolated coefficients for given conditions
     * 
     * Uses bilinear interpolation between the four nearest data points.
     * If caching is enabled, checks cache first.
     * 
     * @param mach Mach number
     * @param angleOfAttack Angle of attack in radians
     * @param configName Configuration to use (default: "default")
     * @return Interpolated aerodynamic coefficients
     * @throws std::runtime_error if configuration not found or data is empty
     */
    AerodynamicCoefficients getCoefficients(double mach, double angleOfAttack,
                                          const std::string& configName = "default") const;

    /**
     * @brief Check if a configuration exists
     * 
     * @param configName Name of the configuration
     * @return true if configuration exists
     */
    bool hasConfiguration(const std::string& configName) const;

    /**
     * @brief Get list of available configurations
     * 
     * @return Vector of configuration names
     */
    std::vector<std::string> getConfigurationNames() const;

    /**
     * @brief Get the data range for a configuration
     * 
     * @param configName Configuration name
     * @param minMach Output: minimum Mach number
     * @param maxMach Output: maximum Mach number
     * @param minAoA Output: minimum angle of attack (radians)
     * @param maxAoA Output: maximum angle of attack (radians)
     * @return true if configuration exists
     */
    bool getDataRange(const std::string& configName,
                     double& minMach, double& maxMach,
                     double& minAoA, double& maxAoA) const;

    /**
     * @brief Clear all cached values
     */
    void clearCache() const;

    /**
     * @brief Get cache statistics
     * 
     * @param hits Output: number of cache hits
     * @param misses Output: number of cache misses
     * @param size Output: current cache size
     */
    void getCacheStats(size_t& hits, size_t& misses, size_t& size) const;

    /**
     * @brief Validate the loaded data
     * 
     * Checks for:
     * - Duplicate data points
     * - Non-physical coefficient values
     * - Proper grid structure
     * 
     * @param configName Configuration to validate
     * @return true if data is valid
     */
    bool validateData(const std::string& configName = "default") const;

    /**
     * @brief Export configuration to CSV file
     * 
     * @param filename Output filename
     * @param configName Configuration to export
     * @throws std::runtime_error if configuration not found or write fails
     */
    void exportToCSV(const std::string& filename,
                    const std::string& configName = "default") const;

private:
    /**
     * @brief Internal structure for organizing data by configuration
     */
    struct ConfigurationData {
        std::vector<AerodynamicDataPoint> dataPoints;
        std::vector<double> uniqueMachNumbers;
        std::vector<double> uniqueAoA;
        
        // Pre-computed for faster lookups
        std::map<std::pair<size_t, size_t>, size_t> gridIndexMap;
    };

    Config m_config;
    std::map<std::string, std::unique_ptr<ConfigurationData>> m_configurations;
    
    // Caching
    mutable std::map<std::tuple<double, double, std::string>, AerodynamicCoefficients> m_cache;
    mutable std::mutex m_cacheMutex;
    mutable size_t m_cacheHits = 0;
    mutable size_t m_cacheMisses = 0;

    /**
     * @brief Parse a single line of CSV data
     * 
     * @param line The CSV line to parse
     * @return Parsed data point
     * @throws std::runtime_error if line cannot be parsed
     */
    AerodynamicDataPoint parseCSVLine(const std::string& line) const;

    /**
     * @brief Build grid index map for fast lookups
     * 
     * @param config Configuration data to process
     */
    void buildGridIndexMap(ConfigurationData& config) const;

    /**
     * @brief Perform bilinear interpolation
     * 
     * @param mach Query Mach number
     * @param aoa Query angle of attack
     * @param config Configuration data
     * @return Interpolated coefficients
     */
    AerodynamicCoefficients interpolate(double mach, double aoa,
                                      const ConfigurationData& config) const;

    /**
     * @brief Find bracketing indices for interpolation
     * 
     * @param value Query value
     * @param sortedValues Sorted array of values
     * @param lowIndex Output: lower bracketing index
     * @param highIndex Output: upper bracketing index
     * @param fraction Output: interpolation fraction
     */
    void findBracketingIndices(double value, 
                              const std::vector<double>& sortedValues,
                              size_t& lowIndex, size_t& highIndex,
                              double& fraction) const;

    /**
     * @brief Check if a value is significantly outside the data range
     * 
     * @param value Query value
     * @param min Minimum data value
     * @param max Maximum data value
     * @return true if extrapolation exceeds threshold
     */
    bool isSignificantExtrapolation(double value, double min, double max) const;

    /**
     * @brief Manage cache size by removing oldest entries
     */
    void trimCache() const;
};

} // namespace aerodynamics
} // namespace physics
} // namespace iloss