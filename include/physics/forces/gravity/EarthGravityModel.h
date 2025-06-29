#pragma once

#include "physics/forces/ForceModel.h"
#include "core/math/Vector3D.h"
#include "core/math/Matrix3D.h"
#include <vector>
#include <memory>
#include <unordered_map>
#include <atomic>
#include <mutex>

namespace iloss {
namespace physics {
namespace forces {

/**
 * @brief Structure to hold spherical harmonic coefficients
 */
struct SphericalHarmonicCoefficient {
    int n;      ///< Degree
    int m;      ///< Order
    double C;   ///< Cosine coefficient (normalized)
    double S;   ///< Sine coefficient (normalized)
};

/**
 * @brief Configuration for Earth gravity model
 */
struct EarthGravityConfig {
    int maxDegree = 8;          ///< Maximum degree to use in computation
    int maxOrder = 8;           ///< Maximum order to use in computation
    bool computeGradient = false; ///< Whether to compute gravity gradient tensor
    bool useCache = true;       ///< Enable position-based caching for performance
    double cacheResolution = 100.0; ///< Cache resolution in meters
    std::string coefficientFile = ""; ///< Path to coefficient file (optional)
};

/**
 * @brief High-fidelity Earth gravity model using spherical harmonics
 * 
 * This class implements a spherical harmonic gravity model capable of
 * representing Earth's complex gravity field up to arbitrary degree and order.
 * It supports loading coefficients from standard formats like EGM2008.
 * 
 * The gravitational potential is expressed as:
 * U = (μ/r) * Σ(n=0 to N) Σ(m=0 to n) (R/r)^n * P_nm(sin φ) * [C_nm cos(mλ) + S_nm sin(mλ)]
 * 
 * where:
 * - μ is the gravitational parameter
 * - r is the distance from Earth's center
 * - R is the reference radius
 * - φ is the geocentric latitude
 * - λ is the longitude
 * - P_nm are the normalized associated Legendre functions
 * - C_nm, S_nm are the normalized spherical harmonic coefficients
 */
class EarthGravityModel : public ForceModel {
public:
    /**
     * @brief Constructor with configuration
     * @param name Name of the force model
     * @param config Configuration parameters
     */
    EarthGravityModel(const std::string& name = "EarthGravityModel",
                      const EarthGravityConfig& config = EarthGravityConfig());

    /**
     * @brief Destructor
     */
    ~EarthGravityModel() override;

    /**
     * @brief Calculate gravitational acceleration
     * @param state Current state vector
     * @param time Current time (unused for gravity)
     * @return Acceleration vector in m/s²
     */
    math::Vector3D calculateAcceleration(const StateVector& state,
                                       const time::Time& time) const override;

    /**
     * @brief Initialize the force model
     * @param config Force model configuration
     * @return True if initialization successful
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
     * @brief Get string representation
     * @return Description string
     */
    std::string toString() const override;

    /**
     * @brief Load spherical harmonic coefficients from vector
     * @param coefficients Vector of coefficients
     * @return True if successful
     */
    bool loadCoefficients(const std::vector<SphericalHarmonicCoefficient>& coefficients);

    /**
     * @brief Load coefficients from file
     * @param filename Path to coefficient file
     * @return True if successful
     */
    bool loadCoefficientsFromFile(const std::string& filename);

    /**
     * @brief Set the degree and order to use in calculations
     * @param degree Maximum degree (must be <= loaded coefficients)
     * @param order Maximum order (must be <= degree)
     */
    void setDegreeOrder(int degree, int order);

    /**
     * @brief Get the current maximum degree
     * @return Maximum degree in use
     */
    int getMaxDegree() const { return m_config.maxDegree; }

    /**
     * @brief Get the current maximum order
     * @return Maximum order in use
     */
    int getMaxOrder() const { return m_config.maxOrder; }

    /**
     * @brief Calculate gravitational potential at a position
     * @param position Position in ECI coordinates
     * @return Gravitational potential in m²/s²
     */
    double calculatePotential(const math::Vector3D& position) const;

    /**
     * @brief Calculate gravity gradient tensor
     * @param position Position in ECI coordinates
     * @return 3x3 gravity gradient tensor in 1/s²
     */
    math::Matrix3D calculateGravityGradient(const math::Vector3D& position) const;

    /**
     * @brief Clear the acceleration cache
     */
    void clearCache() const;

    /**
     * @brief Get cache statistics
     * @param[out] hits Number of cache hits
     * @param[out] misses Number of cache misses
     * @param[out] size Current cache size
     */
    void getCacheStatistics(size_t& hits, size_t& misses, size_t& size) const;

private:
    /**
     * @brief Internal structure for caching Legendre polynomials
     */
    struct LegendreCache {
        std::vector<std::vector<double>> P;    ///< P[n][m] values
        std::vector<std::vector<double>> dP;   ///< dP/dθ values
        double sinPhi;                         ///< sin(latitude) for which cache is valid
        double cosPhi;                         ///< cos(latitude) for which cache is valid
        int maxDegree;                         ///< Maximum degree for which cache is valid
        int maxOrder;                          ///< Maximum order for which cache is valid
        bool valid;                            ///< Whether cache is valid
    };

    /**
     * @brief Internal structure for caching acceleration values
     */
    struct AccelerationCacheEntry {
        math::Vector3D acceleration;
        uint64_t accessCount;
        double timestamp;
    };

    /**
     * @brief Calculate normalized associated Legendre polynomials
     * @param sinPhi Sine of geocentric latitude
     * @param cosPhi Cosine of geocentric latitude
     * @param[out] cache Legendre cache to populate
     */
    void calculateLegendre(double sinPhi, double cosPhi, LegendreCache& cache) const;

    /**
     * @brief Calculate trigonometric functions for longitude
     * @param lambda Longitude in radians
     * @param maxOrder Maximum order needed
     * @param[out] cosMLambda cos(m*lambda) values
     * @param[out] sinMLambda sin(m*lambda) values
     */
    void calculateTrigonometric(double lambda, int maxOrder,
                               std::vector<double>& cosMLambda,
                               std::vector<double>& sinMLambda) const;

    /**
     * @brief Convert position to spherical coordinates
     * @param position Cartesian position
     * @param[out] r Radius
     * @param[out] phi Geocentric latitude
     * @param[out] lambda Longitude
     */
    void cartesianToSpherical(const math::Vector3D& position,
                             double& r, double& phi, double& lambda) const;

    /**
     * @brief Calculate acceleration without caching
     * @param position Position in ECI coordinates
     * @return Acceleration vector
     */
    math::Vector3D calculateAccelerationDirect(const math::Vector3D& position) const;

    /**
     * @brief Get cache key for position
     * @param position Position vector
     * @return Cache key
     */
    uint64_t getCacheKey(const math::Vector3D& position) const;

    /**
     * @brief Load EGM2008 coefficients
     * @param filename Path to EGM2008 file
     * @return True if successful
     */
    bool loadEGM2008(const std::string& filename);

    /**
     * @brief Load generic coefficient file
     * @param filename Path to coefficient file
     * @return True if successful
     */
    bool loadGenericCoefficients(const std::string& filename);

    // Configuration
    EarthGravityConfig m_config;
    
    // Model parameters
    double m_mu;                    ///< Gravitational parameter (m³/s²)
    double m_radius;                ///< Reference radius (m)
    int m_maxDegreeLoaded;          ///< Maximum degree in loaded coefficients
    int m_maxOrderLoaded;           ///< Maximum order in loaded coefficients
    
    // Coefficient storage
    std::vector<std::vector<double>> m_C;  ///< C[n][m] normalized coefficients
    std::vector<std::vector<double>> m_S;  ///< S[n][m] normalized coefficients
    
    // Performance optimization
    mutable std::unordered_map<uint64_t, AccelerationCacheEntry> m_accelerationCache;
    mutable std::mutex m_cacheMutex;
    mutable std::atomic<size_t> m_cacheHits{0};
    mutable std::atomic<size_t> m_cacheMisses{0};
    
    // Thread-local storage for Legendre polynomial computation
    static thread_local LegendreCache t_legendreCache;
    static thread_local std::vector<double> t_cosMLambda;
    static thread_local std::vector<double> t_sinMLambda;
    
    // Constants for numerical stability
    static constexpr double EPSILON = 1e-15;
    static constexpr double MIN_RADIUS = 1000.0;  // Minimum radius in meters
    static constexpr size_t MAX_CACHE_SIZE = 10000; // Maximum cache entries
};

/**
 * @brief Loader for EGM2008 gravity model coefficients
 */
class EGM2008CoefficientLoader {
public:
    /**
     * @brief Load coefficients from official EGM2008 file
     * @param filename Path to EGM2008 coefficient file
     * @param[out] coefficients Vector to store loaded coefficients
     * @param maxDegree Maximum degree to load (default: all)
     * @param maxOrder Maximum order to load (default: all)
     * @return True if successful
     */
    static bool loadCoefficients(const std::string& filename,
                                std::vector<SphericalHarmonicCoefficient>& coefficients,
                                int maxDegree = -1,
                                int maxOrder = -1);

    /**
     * @brief Get default EGM2008 data path
     * @return Path to EGM2008 data file
     */
    static std::string getDefaultDataPath();

    /**
     * @brief Validate coefficient file format
     * @param filename Path to coefficient file
     * @return True if file format is valid
     */
    static bool validateFile(const std::string& filename);
};

} // namespace forces
} // namespace physics
} // namespace iloss