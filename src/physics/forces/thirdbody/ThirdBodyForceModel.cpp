#include "physics/forces/thirdbody/ThirdBodyForceModel.h"
#include "core/logging/Logger.h"
#include "core/time/Time.h"
#include <stdexcept>
#include <sstream>
#include <cmath>
#include <algorithm>

namespace iloss {
namespace physics {
namespace forces {
namespace thirdbody {

// Static member initialization
const std::map<std::string, double> ThirdBodyForceModel::DEFAULT_MU_VALUES = {
    {"Sun", math::constants::SUN_MU},
    {"Moon", math::constants::MOON_MU},
    {"Mercury", 2.2032e13},         // m³/s²
    {"Venus", math::constants::VENUS_MU},
    {"Mars", math::constants::MARS_MU},
    {"Jupiter", math::constants::JUPITER_MU},
    {"Saturn", 3.7931187e16},       // m³/s²
    {"Uranus", 5.793939e15},        // m³/s²
    {"Neptune", 6.836529e15},       // m³/s²
    {"Pluto", 8.71e11}              // m³/s²
};

const std::map<std::string, std::string> ThirdBodyForceModel::BODY_SPICE_IDS = {
    {"Sun", "10"},
    {"Mercury", "199"},
    {"Venus", "299"},
    {"Earth", "399"},
    {"Moon", "301"},
    {"Mars", "499"},
    {"Jupiter", "599"},
    {"Saturn", "699"},
    {"Uranus", "799"},
    {"Neptune", "899"},
    {"Pluto", "999"}
};

ThirdBodyForceModel::ThirdBodyForceModel(const std::string& name)
    : ForceModel(name, ForceModelType::ThirdBody),
      m_centralBody("Earth"),
      m_centralBodySpiceId("399"),
      m_cacheDuration(60.0),  // 60 seconds default
      m_cacheHits(0),
      m_cacheMisses(0),
      m_spiceInitialized(false) {
    
    LOG_INFO("ThirdBodyForceModel", "Created third-body force model '{}'", getName());
    
    // Initialize SPICE if available
    if (isSpiceAvailable()) {
        try {
            spice::SPICEUtils::initialize();
            m_spiceInitialized = true;
            LOG_INFO("ThirdBodyForceModel", "SPICE toolkit initialized successfully");
        } catch (const std::exception& e) {
            LOG_WARN("ThirdBodyForceModel", "Failed to initialize SPICE: {}", e.what());
            m_spiceInitialized = false;
        }
    } else {
        LOG_WARN("ThirdBodyForceModel", "SPICE toolkit not available");
    }
}

math::Vector3D ThirdBodyForceModel::calculateAcceleration(
    const StateVector& state,
    const time::Time& time) const {
    
    if (!m_spiceInitialized) {
        LOG_WARN("ThirdBodyForceModel", "SPICE not initialized, returning zero acceleration");
        return math::Vector3D(0.0, 0.0, 0.0);
    }
    
    // Get spacecraft position
    const math::Vector3D& spacecraftPosition = state.getPosition();
    
    // Total perturbation acceleration
    math::Vector3D totalAcceleration(0.0, 0.0, 0.0);
    
    // Calculate perturbation from each enabled body
    for (const auto& [bodyName, bodyConfig] : m_bodies) {
        if (!bodyConfig.enabled) {
            continue;
        }
        
        try {
            math::Vector3D perturbation = calculateBodyPerturbation(
                bodyConfig, spacecraftPosition, time);
            totalAcceleration += perturbation;
            
            LOG_TRACE("ThirdBodyForceModel", 
                     "Perturbation from {}: ({}, {}, {}) m/s² (magnitude: {} m/s²)",
                     bodyName, perturbation.x(), perturbation.y(), perturbation.z(), 
                     perturbation.magnitude());
                     
        } catch (const std::exception& e) {
            LOG_ERROR("ThirdBodyForceModel", 
                     "Failed to calculate perturbation from {}: {}", 
                     bodyName, e.what());
        }
    }
    
    LOG_TRACE("ThirdBodyForceModel", 
             "Total third-body acceleration: ({}, {}, {}) m/s² (magnitude: {} m/s²)",
             totalAcceleration.x(), totalAcceleration.y(), totalAcceleration.z(), 
             totalAcceleration.magnitude());
    
    return totalAcceleration;
}

bool ThirdBodyForceModel::initialize(const ForceModelConfig& config) {
    try {
        // Clear existing bodies
        m_bodies.clear();
        clearCache();
        
        // Set central body
        if (config.hasParameter("central_body")) {
            setCentralBody(config.getParameter<std::string>("central_body"));
        }
        
        // Set cache duration
        if (config.hasParameter("cache_duration")) {
            setCacheDuration(config.getParameter<double>("cache_duration"));
        }
        
        // Check if we should enable all known bodies
        if (config.getParameter<bool>("enable_all", false)) {
            // Add all default bodies except the central body
            for (const auto& [name, mu] : DEFAULT_MU_VALUES) {
                if (name != m_centralBody) {
                    auto spiceId = getSpiceId(name);
                    if (!spiceId.empty()) {
                        addBody(ThirdBodyConfig(name, spiceId, mu));
                    }
                }
            }
            LOG_INFO("ThirdBodyForceModel", "Enabled all known bodies");
        } else {
            // Add specific bodies from configuration
            if (config.hasParameter("bodies")) {
                auto bodies = config.getParameter<std::vector<std::string>>("bodies");
                for (const auto& bodyName : bodies) {
                    // Check if it's a known body
                    auto muIt = DEFAULT_MU_VALUES.find(bodyName);
                    auto spiceId = getSpiceId(bodyName);
                    
                    if (muIt != DEFAULT_MU_VALUES.end() && !spiceId.empty()) {
                        addBody(ThirdBodyConfig(bodyName, spiceId, muIt->second));
                        LOG_INFO("ThirdBodyForceModel", "Added body: {} (μ = {} m³/s²)", 
                                bodyName, muIt->second);
                    } else {
                        LOG_WARN("ThirdBodyForceModel", "Unknown body: {}", bodyName);
                    }
                }
            }
        }
        
        // Add custom bodies if provided
        if (config.hasParameter("custom_bodies")) {
            auto customBodies = config.getParameter<std::vector<ThirdBodyConfig>>("custom_bodies");
            for (const auto& customBody : customBodies) {
                addBody(customBody);
                LOG_INFO("ThirdBodyForceModel", "Added custom body: {} (μ = {} m³/s²)", 
                        customBody.name, customBody.mu);
            }
        }
        
        // Validate configuration
        if (!validate()) {
            LOG_ERROR("ThirdBodyForceModel", "Invalid configuration");
            return false;
        }
        
        LOG_INFO("ThirdBodyForceModel", "Initialized with {} bodies", m_bodies.size());
        return true;
        
    } catch (const std::exception& e) {
        LOG_ERROR("ThirdBodyForceModel", "Failed to initialize: {}", e.what());
        return false;
    }
}

void ThirdBodyForceModel::update(const time::Time& time) {
    if (!m_spiceInitialized) {
        return;
    }
    
    // Update cache for all enabled bodies
    for (const auto& [bodyName, bodyConfig] : m_bodies) {
        if (bodyConfig.enabled) {
            updateBodyCache(bodyName, time);
        }
    }
}

bool ThirdBodyForceModel::validate() const {
    if (m_bodies.empty()) {
        LOG_WARN("ThirdBodyForceModel", "No bodies configured");
        return true;  // Valid but will produce zero acceleration
    }
    
    // Check each body configuration
    for (const auto& [name, config] : m_bodies) {
        if (config.mu <= 0.0) {
            LOG_ERROR("ThirdBodyForceModel", 
                     "Invalid gravitational parameter for {}: {} m³/s²", 
                     name, config.mu);
            return false;
        }
        
        if (config.spiceId.empty()) {
            LOG_ERROR("ThirdBodyForceModel", "Empty SPICE ID for {}", name);
            return false;
        }
        
        if (config.minDistance < 0.0) {
            LOG_ERROR("ThirdBodyForceModel", 
                     "Invalid minimum distance for {}: {} m", 
                     name, config.minDistance);
            return false;
        }
    }
    
    return true;
}

std::unique_ptr<ForceModel> ThirdBodyForceModel::clone() const {
    auto cloned = std::make_unique<ThirdBodyForceModel>(getName());
    
    // Copy configuration
    cloned->m_centralBody = m_centralBody;
    cloned->m_centralBodySpiceId = m_centralBodySpiceId;
    cloned->m_bodies = m_bodies;
    cloned->m_cacheDuration = m_cacheDuration;
    cloned->setEnabled(isEnabled());
    
    // Don't copy cache or statistics
    
    return cloned;
}

std::string ThirdBodyForceModel::toString() const {
    std::ostringstream oss;
    oss << "ThirdBodyForceModel[name=" << getName()
        << ", central_body=" << m_centralBody
        << ", bodies={";
    
    bool first = true;
    for (const auto& [name, config] : m_bodies) {
        if (!first) oss << ", ";
        oss << name << (config.enabled ? "" : " (disabled)");
        first = false;
    }
    
    oss << "}, cache_duration=" << m_cacheDuration << "s"
        << ", enabled=" << (isEnabled() ? "true" : "false")
        << "]";
    
    return oss.str();
}

void ThirdBodyForceModel::addBody(const ThirdBodyConfig& config) {
    m_bodies[config.name] = config;
    // Clear cache for this body
    std::lock_guard<std::mutex> lock(m_cacheMutex);
    m_cache.erase(config.name);
}

bool ThirdBodyForceModel::removeBody(const std::string& name) {
    auto it = m_bodies.find(name);
    if (it != m_bodies.end()) {
        m_bodies.erase(it);
        // Clear cache for this body
        std::lock_guard<std::mutex> lock(m_cacheMutex);
        m_cache.erase(name);
        return true;
    }
    return false;
}

bool ThirdBodyForceModel::setBodyEnabled(const std::string& name, bool enabled) {
    auto it = m_bodies.find(name);
    if (it != m_bodies.end()) {
        it->second.enabled = enabled;
        return true;
    }
    return false;
}

std::vector<ThirdBodyConfig> ThirdBodyForceModel::getBodies() const {
    std::vector<ThirdBodyConfig> result;
    result.reserve(m_bodies.size());
    
    for (const auto& [name, config] : m_bodies) {
        result.push_back(config);
    }
    
    return result;
}

void ThirdBodyForceModel::setCentralBody(const std::string& body) {
    m_centralBody = body;
    auto it = BODY_SPICE_IDS.find(body);
    if (it != BODY_SPICE_IDS.end()) {
        m_centralBodySpiceId = it->second;
    } else {
        LOG_WARN("ThirdBodyForceModel", 
                "Unknown central body '{}', using default SPICE ID", body);
        m_centralBodySpiceId = "399";  // Earth
    }
}

void ThirdBodyForceModel::setCacheDuration(double duration) {
    if (duration < 0.0) {
        LOG_WARN("ThirdBodyForceModel", 
                "Invalid cache duration {}, using 0.0", duration);
        m_cacheDuration = 0.0;
    } else {
        m_cacheDuration = duration;
    }
}

void ThirdBodyForceModel::clearCache() {
    std::lock_guard<std::mutex> lock(m_cacheMutex);
    m_cache.clear();
    m_cacheHits = 0;
    m_cacheMisses = 0;
}

void ThirdBodyForceModel::resetCacheStats() {
    m_cacheHits = 0;
    m_cacheMisses = 0;
}

bool ThirdBodyForceModel::isSpiceAvailable() {
    return spice::SPICEUtils::isAvailable();
}

const std::map<std::string, double>& ThirdBodyForceModel::getDefaultGravitationalParameters() {
    return DEFAULT_MU_VALUES;
}

std::string ThirdBodyForceModel::getSpiceId(const std::string& bodyName) {
    auto it = BODY_SPICE_IDS.find(bodyName);
    if (it != BODY_SPICE_IDS.end()) {
        return it->second;
    }
    return "";
}

math::Vector3D ThirdBodyForceModel::calculateBodyPerturbation(
    const ThirdBodyConfig& bodyConfig,
    const math::Vector3D& spacecraftPosition,
    const time::Time& time) const {
    
    // Get position of perturbing body relative to central body
    math::Vector3D bodyPosition = getBodyPosition(bodyConfig.spiceId, time);
    
    // Vector from perturbing body to spacecraft
    math::Vector3D r_ps = spacecraftPosition - bodyPosition;
    double r_ps_mag = r_ps.magnitude();
    
    // Check minimum distance
    if (r_ps_mag < bodyConfig.minDistance) {
        LOG_WARN("ThirdBodyForceModel", 
                "Spacecraft too close to {}: {} m (min: {} m)",
                bodyConfig.name, r_ps_mag, bodyConfig.minDistance);
        return math::Vector3D(0.0, 0.0, 0.0);
    }
    
    // Vector from perturbing body to central body (opposite of bodyPosition)
    math::Vector3D r_pc = -bodyPosition;
    double r_pc_mag = r_pc.magnitude();
    
    // Calculate perturbation acceleration
    // a = GM_p * [r_ps/|r_ps|³ - r_pc/|r_pc|³]
    double factor_ps = bodyConfig.mu / (r_ps_mag * r_ps_mag * r_ps_mag);
    double factor_pc = bodyConfig.mu / (r_pc_mag * r_pc_mag * r_pc_mag);
    
    math::Vector3D acceleration = r_ps * factor_ps - r_pc * factor_pc;
    
    return acceleration;
}

math::Vector3D ThirdBodyForceModel::getBodyPosition(
    const std::string& spiceId,
    const time::Time& time) const {
    
    // Check cache first
    {
        std::lock_guard<std::mutex> lock(m_cacheMutex);
        
        // Find body name for this SPICE ID (for cache lookup)
        std::string bodyName;
        for (const auto& [name, config] : m_bodies) {
            if (config.spiceId == spiceId) {
                bodyName = name;
                break;
            }
        }
        
        if (!bodyName.empty()) {
            auto cacheIt = m_cache.find(bodyName);
            if (cacheIt != m_cache.end() && isCacheValid(cacheIt->second, time)) {
                ++m_cacheHits;
                return cacheIt->second.position;
            }
        }
    }
    
    // Cache miss - get from SPICE
    ++m_cacheMisses;
    
    try {
        // Convert time to ephemeris time
        double et = timeToEphemerisTime(time);
        
        // Get state of perturbing body relative to central body
        std::array<double, 6> state;
        double lt;
        spice::SPICEUtils::getState(
            spiceId,                  // Target body
            et,                       // Ephemeris time
            spice::Frames::J2000,     // Reference frame
            "NONE",                   // No aberration correction
            m_centralBodySpiceId,     // Observer (central body)
            state,
            lt
        );
        
        // Convert from km to m
        math::Vector3D position(
            state[0] * 1000.0,
            state[1] * 1000.0,
            state[2] * 1000.0
        );
        
        // Update cache
        {
            std::lock_guard<std::mutex> lock(m_cacheMutex);
            
            // Find body name for cache update
            for (const auto& [name, config] : m_bodies) {
                if (config.spiceId == spiceId) {
                    EphemerisCache& cache = m_cache[name];
                    cache.lastUpdateTime = time;
                    cache.position = position;
                    cache.velocity = math::Vector3D(
                        state[3] * 1000.0,
                        state[4] * 1000.0,
                        state[5] * 1000.0
                    );
                    cache.valid = true;
                    break;
                }
            }
        }
        
        return position;
        
    } catch (const std::exception& e) {
        LOG_ERROR("ThirdBodyForceModel", 
                 "Failed to get position for body {}: {}", 
                 spiceId, e.what());
        throw;
    }
}

void ThirdBodyForceModel::updateBodyCache(const std::string& bodyName, const time::Time& time) const {
    auto bodyIt = m_bodies.find(bodyName);
    if (bodyIt == m_bodies.end()) {
        return;
    }
    
    // Force cache update by calling getBodyPosition
    try {
        getBodyPosition(bodyIt->second.spiceId, time);
    } catch (const std::exception& e) {
        LOG_ERROR("ThirdBodyForceModel", 
                 "Failed to update cache for {}: {}", 
                 bodyName, e.what());
    }
}

bool ThirdBodyForceModel::isCacheValid(const EphemerisCache& cache, const time::Time& time) const {
    if (!cache.valid) {
        return false;
    }
    
    // Check if cache has expired
    double elapsedTime = time - cache.lastUpdateTime;
    return elapsedTime >= 0.0 && elapsedTime <= m_cacheDuration;
}

double ThirdBodyForceModel::timeToEphemerisTime(const time::Time& time) const {
    // Convert from J2000 epoch to SPICE ephemeris time
    // SPICE ET is seconds past J2000.0
    return time.getTime(time::TimeSystem::TDB);
}

} // namespace thirdbody
} // namespace forces
} // namespace physics
} // namespace iloss