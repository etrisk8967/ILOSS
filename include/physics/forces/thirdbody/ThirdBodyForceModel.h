#pragma once

#include "physics/forces/ForceModel.h"
#include "core/math/MathConstants.h"
#include "core/external/SPICEWrapper.h"
#include <map>
#include <string>
#include <vector>
#include <memory>
#include <mutex>

namespace iloss {
namespace physics {
namespace forces {
namespace thirdbody {

/**
 * @brief Configuration for third-body perturbations
 */
struct ThirdBodyConfig {
    std::string name;           ///< Name of the perturbing body (e.g., "Sun", "Moon")
    std::string spiceId;        ///< SPICE ID string (e.g., "10" for Sun, "301" for Moon)
    double mu;                  ///< Gravitational parameter in m³/s²
    bool enabled;               ///< Whether this body is enabled
    double minDistance;         ///< Minimum distance for perturbation calculation (m)
    
    ThirdBodyConfig() : mu(0.0), enabled(true), minDistance(1e6) {}
    
    ThirdBodyConfig(const std::string& n, const std::string& id, double m)
        : name(n), spiceId(id), mu(m), enabled(true), minDistance(1e6) {}
};

/**
 * @brief Cache entry for ephemeris data
 */
struct EphemerisCache {
    time::Time lastUpdateTime;
    math::Vector3D position;      ///< Position in inertial frame (m)
    math::Vector3D velocity;      ///< Velocity in inertial frame (m/s)
    bool valid;
    
    EphemerisCache() : valid(false) {}
};

/**
 * @brief Third-body gravitational perturbation force model
 * 
 * This model calculates the gravitational perturbations on a spacecraft
 * due to third bodies such as the Sun, Moon, and planets. The perturbation
 * acceleration is computed as the difference between the direct attraction
 * on the spacecraft and the attraction on the central body.
 * 
 * The perturbation acceleration is given by:
 * a = GM_p * [r_ps/|r_ps|³ - r_pc/|r_pc|³]
 * 
 * where:
 * - GM_p is the gravitational parameter of the perturbing body
 * - r_ps is the vector from perturbing body to spacecraft
 * - r_pc is the vector from perturbing body to central body
 * 
 * This model uses the SPICE toolkit for accurate ephemeris data
 * and implements caching to optimize performance.
 */
class ThirdBodyForceModel : public ForceModel {
public:
    /**
     * @brief Constructor with default configuration
     * @param name Name of the force model instance
     */
    explicit ThirdBodyForceModel(const std::string& name = "ThirdBody");

    /**
     * @brief Destructor
     */
    virtual ~ThirdBodyForceModel() = default;

    /**
     * @brief Calculate third-body perturbation accelerations
     * 
     * @param state Current state vector (must be in inertial frame)
     * @param time Current time
     * @return Total perturbation acceleration vector in m/s²
     */
    math::Vector3D calculateAcceleration(
        const StateVector& state,
        const time::Time& time) const override;

    /**
     * @brief Initialize the model with configuration
     * 
     * Configuration parameters:
     * - "bodies": Array of body names to include (e.g., ["Sun", "Moon"])
     * - "central_body": Central body name (default: "Earth")
     * - "cache_duration": Cache validity duration in seconds (default: 60.0)
     * - "enable_all": Enable all known bodies (default: false)
     * - "custom_bodies": Array of custom body configurations
     * 
     * @param config Configuration parameters
     * @return True if successful
     */
    bool initialize(const ForceModelConfig& config) override;

    /**
     * @brief Update ephemeris data for current time
     * @param time Current time
     */
    void update(const time::Time& time) override;

    /**
     * @brief Validate the model configuration
     * @return True if configuration is valid
     */
    bool validate() const override;

    /**
     * @brief Clone the force model
     * @return Cloned model
     */
    std::unique_ptr<ForceModel> clone() const override;

    /**
     * @brief Get string representation
     * @return Description string
     */
    std::string toString() const override;

    /**
     * @brief Add a perturbing body
     * @param config Body configuration
     */
    void addBody(const ThirdBodyConfig& config);

    /**
     * @brief Remove a perturbing body
     * @param name Name of the body to remove
     * @return True if body was found and removed
     */
    bool removeBody(const std::string& name);

    /**
     * @brief Enable or disable a specific body
     * @param name Name of the body
     * @param enabled True to enable, false to disable
     * @return True if body was found
     */
    bool setBodyEnabled(const std::string& name, bool enabled);

    /**
     * @brief Get list of configured bodies
     * @return Vector of body configurations
     */
    std::vector<ThirdBodyConfig> getBodies() const;

    /**
     * @brief Get the central body name
     * @return Central body name
     */
    const std::string& getCentralBody() const { return m_centralBody; }

    /**
     * @brief Set the central body
     * @param body Central body name
     */
    void setCentralBody(const std::string& body);

    /**
     * @brief Get cache duration
     * @return Cache duration in seconds
     */
    double getCacheDuration() const { return m_cacheDuration; }

    /**
     * @brief Set cache duration
     * @param duration Cache duration in seconds
     */
    void setCacheDuration(double duration);

    /**
     * @brief Clear ephemeris cache
     */
    void clearCache();

    /**
     * @brief Get number of cache hits (for performance monitoring)
     * @return Number of cache hits
     */
    size_t getCacheHits() const { return m_cacheHits; }

    /**
     * @brief Get number of cache misses (for performance monitoring)
     * @return Number of cache misses
     */
    size_t getCacheMisses() const { return m_cacheMisses; }

    /**
     * @brief Reset cache statistics
     */
    void resetCacheStats();

    /**
     * @brief Check if SPICE is available
     * @return True if SPICE toolkit is available
     */
    static bool isSpiceAvailable();

    /**
     * @brief Get default gravitational parameters for known bodies
     */
    static const std::map<std::string, double>& getDefaultGravitationalParameters();

    /**
     * @brief Get SPICE ID for known body
     * @param bodyName Name of the body
     * @return SPICE ID string, empty if not found
     */
    static std::string getSpiceId(const std::string& bodyName);

private:
    /**
     * @brief Calculate perturbation from a single body
     * @param bodyConfig Configuration of the perturbing body
     * @param spacecraftPosition Position of spacecraft in inertial frame
     * @param time Current time
     * @return Perturbation acceleration vector
     */
    math::Vector3D calculateBodyPerturbation(
        const ThirdBodyConfig& bodyConfig,
        const math::Vector3D& spacecraftPosition,
        const time::Time& time) const;

    /**
     * @brief Get body position from SPICE
     * @param spiceId SPICE ID of the body
     * @param time Current time
     * @return Position vector in J2000 frame
     */
    math::Vector3D getBodyPosition(
        const std::string& spiceId,
        const time::Time& time) const;

    /**
     * @brief Update cache for a specific body
     * @param bodyName Name of the body
     * @param time Current time
     */
    void updateBodyCache(const std::string& bodyName, const time::Time& time) const;

    /**
     * @brief Check if cache is valid for given time
     * @param cache Cache entry to check
     * @param time Current time
     * @return True if cache is still valid
     */
    bool isCacheValid(const EphemerisCache& cache, const time::Time& time) const;

    /**
     * @brief Convert J2000 Epoch time to SPICE ephemeris time
     * @param time Time object
     * @return Ephemeris time in seconds
     */
    double timeToEphemerisTime(const time::Time& time) const;

    // Member variables
    std::string m_centralBody;                                    ///< Central body name
    std::string m_centralBodySpiceId;                            ///< Central body SPICE ID
    std::map<std::string, ThirdBodyConfig> m_bodies;            ///< Configured bodies
    double m_cacheDuration;                                       ///< Cache validity duration (seconds)
    mutable std::map<std::string, EphemerisCache> m_cache;      ///< Ephemeris cache
    mutable std::mutex m_cacheMutex;                             ///< Mutex for thread-safe cache access
    mutable size_t m_cacheHits;                                  ///< Cache hit counter
    mutable size_t m_cacheMisses;                                ///< Cache miss counter
    bool m_spiceInitialized;                                     ///< Whether SPICE has been initialized

    // Static data
    static const std::map<std::string, double> DEFAULT_MU_VALUES;
    static const std::map<std::string, std::string> BODY_SPICE_IDS;
};

} // namespace thirdbody
} // namespace forces
} // namespace physics
} // namespace iloss