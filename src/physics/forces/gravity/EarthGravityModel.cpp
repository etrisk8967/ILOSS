#include "physics/forces/gravity/EarthGravityModel.h"
#include "core/constants/EarthModel.h"
#include "core/logging/Logger.h"
#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <chrono>
#include <iostream>

namespace iloss {
namespace physics {
namespace forces {

// Thread-local storage definitions
thread_local EarthGravityModel::LegendreCache EarthGravityModel::t_legendreCache = {
    .P = {},
    .dP = {},
    .sinPhi = 0.0,
    .cosPhi = 0.0,
    .maxDegree = -1,
    .maxOrder = -1,
    .valid = false
};
thread_local std::vector<double> EarthGravityModel::t_cosMLambda;
thread_local std::vector<double> EarthGravityModel::t_sinMLambda;

EarthGravityModel::EarthGravityModel(const std::string& name, const EarthGravityConfig& config)
    : ForceModel(name, ForceModelType::GravityField)
    , m_config(config)
    , m_mu(constants::EarthModel::MU)
    , m_radius(constants::EarthModel::EQUATORIAL_RADIUS)
    , m_maxDegreeLoaded(0)
    , m_maxOrderLoaded(0) {
    
    // Reserve space for coefficients up to reasonable maximum
    const int maxSize = std::max(config.maxDegree, 70) + 1;
    m_C.resize(maxSize);
    m_S.resize(maxSize);
    
    for (int n = 0; n < maxSize; ++n) {
        m_C[n].resize(n + 1, 0.0);
        m_S[n].resize(n + 1, 0.0);
    }
    
    // Initialize C00 = 1 (normalized)
    m_C[0][0] = 1.0;
    
    // Note: Thread-local storage will be resized on first use
}

EarthGravityModel::~EarthGravityModel() = default;

math::Vector3D EarthGravityModel::calculateAcceleration(const StateVector& state,
                                                       const time::Time& /*time*/) const {
    const math::Vector3D& position = state.getPosition();
    
    // Use cache if enabled
    if (m_config.useCache) {
        uint64_t cacheKey = getCacheKey(position);
        
        {
            std::lock_guard<std::mutex> lock(m_cacheMutex);
            auto it = m_accelerationCache.find(cacheKey);
            if (it != m_accelerationCache.end()) {
                m_cacheHits++;
                it->second.accessCount++;
                return it->second.acceleration;
            }
        }
        
        // Cache miss - calculate acceleration
        m_cacheMisses++;
        math::Vector3D acceleration = calculateAccelerationDirect(position);
        
        // Store in cache
        {
            std::lock_guard<std::mutex> lock(m_cacheMutex);
            
            // Evict least recently used entry if cache is full
            if (m_accelerationCache.size() >= MAX_CACHE_SIZE) {
                auto oldest = std::min_element(m_accelerationCache.begin(), m_accelerationCache.end(),
                    [](const auto& a, const auto& b) {
                        return a.second.accessCount < b.second.accessCount;
                    });
                m_accelerationCache.erase(oldest);
            }
            
            AccelerationCacheEntry entry;
            entry.acceleration = acceleration;
            entry.accessCount = 1;
            entry.timestamp = 0.0; // Could use actual time if needed
            m_accelerationCache[cacheKey] = entry;
        }
        
        return acceleration;
    } else {
        return calculateAccelerationDirect(position);
    }
}

math::Vector3D EarthGravityModel::calculateAccelerationDirect(const math::Vector3D& position) const {
    // Convert to spherical coordinates
    double r, phi, lambda;
    cartesianToSpherical(position, r, phi, lambda);
    
    // Check minimum radius to avoid singularity
    if (r < MIN_RADIUS) {
        LOG_WARN("EarthGravityModel: Position too close to center, r = {}", r);
        return math::Vector3D::zero();
    }
    
    // Calculate sin and cos of latitude
    double sinPhi = std::sin(phi);
    double cosPhi = std::cos(phi);
    
    // Calculate Legendre polynomials
    if (!t_legendreCache.valid || 
        std::abs(t_legendreCache.sinPhi - sinPhi) > EPSILON ||
        std::abs(t_legendreCache.cosPhi - cosPhi) > EPSILON ||
        t_legendreCache.maxDegree < m_config.maxDegree ||
        t_legendreCache.maxOrder < m_config.maxOrder) {
        calculateLegendre(sinPhi, cosPhi, t_legendreCache);
    }
    
    // Calculate trigonometric functions
    calculateTrigonometric(lambda, m_config.maxOrder, t_cosMLambda, t_sinMLambda);
    
    // Initialize acceleration components in spherical coordinates
    double ar = 0.0;    // Radial component
    double aphi = 0.0;  // Latitude component
    double alambda = 0.0; // Longitude component
    
    // Ratio for recursion
    double rRatio = m_radius / r;
    double rPower = 1.0; // (R/r)^0 = 1
    
    // Debug output
    LOG_DEBUG("calculateAccelerationDirect: maxDegree={}, maxOrder={}", m_config.maxDegree, m_config.maxOrder);
    LOG_DEBUG("Position: r={}, phi={}, lambda={}", r, phi, lambda);
    
    // Sum over degrees and orders
    for (int n = 0; n <= m_config.maxDegree; ++n) {
        if (n > 0) {
            rPower *= rRatio; // (R/r)^n
        }
        
        for (int m = 0; m <= std::min(n, m_config.maxOrder); ++m) {
            double Cnm = m_C[n][m];
            double Snm = m_S[n][m];
            
            LOG_DEBUG("n={}, m={}: C={}, S={}", n, m, Cnm, Snm);
            
            // Skip if coefficients are zero
            if (std::abs(Cnm) < EPSILON && std::abs(Snm) < EPSILON) {
                LOG_DEBUG("Skipping zero coefficients");
                continue;
            }
            
            // Bounds check
            if (n >= static_cast<int>(t_legendreCache.P.size()) || 
                m >= static_cast<int>(t_legendreCache.P[n].size())) {
                LOG_WARN("Legendre polynomial not available for n={}, m={}", n, m);
                continue;
            }
            
            double Pnm = t_legendreCache.P[n][m];
            double dPnm = t_legendreCache.dP[n][m];
            
            double cosMLambda = t_cosMLambda[m];
            double sinMLambda = t_sinMLambda[m];
            
            // Potential contribution
            double potentialTerm = Cnm * cosMLambda + Snm * sinMLambda;
            
            // Radial acceleration: -(n+1) * (R/r)^n * P_nm * (C_nm*cos + S_nm*sin)
            ar -= (n + 1) * rPower * Pnm * potentialTerm;
            
            // Latitude acceleration: (R/r)^n * dP_nm/dθ * (C_nm*cos + S_nm*sin)
            aphi += rPower * dPnm * potentialTerm;
            
            // Longitude acceleration: m * (R/r)^n * P_nm * (-C_nm*sin + S_nm*cos)
            if (m > 0) {
                alambda += m * rPower * Pnm * (-Cnm * sinMLambda + Snm * cosMLambda);
            }
        }
    }
    
    // Scale accelerations
    double muOverR2 = m_mu / (r * r);
    ar *= muOverR2;
    aphi *= -muOverR2; // Negative because dP/dθ convention
    alambda *= -muOverR2 / cosPhi; // Scale by 1/cos(phi) for longitude component
    
    // Convert from spherical to Cartesian coordinates
    double cosLambda = t_cosMLambda[1];
    double sinLambda = t_sinMLambda[1];
    
    math::Vector3D acceleration;
    acceleration.setX(ar * cosPhi * cosLambda - aphi * sinPhi * cosLambda - alambda * sinLambda);
    acceleration.setY(ar * cosPhi * sinLambda - aphi * sinPhi * sinLambda + alambda * cosLambda);
    acceleration.setZ(ar * sinPhi + aphi * cosPhi);
    
    return acceleration;
}

void EarthGravityModel::calculateLegendre(double sinPhi, double cosPhi, LegendreCache& cache) const {
    const int maxN = m_config.maxDegree;
    const int maxM = m_config.maxOrder;
    
    // Ensure cache is properly sized
    cache.P.resize(maxN + 1);
    cache.dP.resize(maxN + 1);
    for (int n = 0; n <= maxN; ++n) {
        cache.P[n].resize(n + 1);
        cache.dP[n].resize(n + 1);
    }
    
    // Initialize P[0][0] = 1 (normalized)
    cache.P[0][0] = 1.0;
    cache.dP[0][0] = 0.0;
    
    // Calculate sectoral terms P[m][m]
    double sqrt3 = std::sqrt(3.0);
    for (int m = 1; m <= maxM && m <= maxN; ++m) {
        // P[m][m] = sqrt((2m+1)/(2m)) * cos(phi) * P[m-1][m-1]
        double factor = std::sqrt((2.0 * m + 1.0) / (2.0 * m));
        cache.P[m][m] = factor * cosPhi * cache.P[m-1][m-1];
        
        // Derivative
        if (m == 1) {
            cache.dP[1][1] = -sqrt3 * sinPhi;
        } else {
            cache.dP[m][m] = factor * (-sinPhi * cache.P[m-1][m-1] + cosPhi * cache.dP[m-1][m-1]);
        }
    }
    
    // Calculate P[m+1][m] terms
    for (int m = 0; m < maxM && m < maxN; ++m) {
        if (m + 1 <= maxN) {
            // P[m+1][m] = sqrt(2m+3) * sin(phi) * P[m][m]
            double factor = std::sqrt(2.0 * m + 3.0);
            cache.P[m+1][m] = factor * sinPhi * cache.P[m][m];
            
            // Derivative
            cache.dP[m+1][m] = factor * (cosPhi * cache.P[m][m] + sinPhi * cache.dP[m][m]);
        }
    }
    
    // Use recursion for remaining terms
    for (int m = 0; m <= maxM && m <= maxN; ++m) {
        for (int n = m + 2; n <= maxN; ++n) {
            // Recursion coefficients
            double anm = std::sqrt((2.0 * n + 1.0) * (2.0 * n - 1.0) / 
                                  ((n - m) * (n + m)));
            double bnm = std::sqrt((2.0 * n + 1.0) * (n + m - 1.0) * (n - m - 1.0) / 
                                  ((2.0 * n - 3.0) * (n - m) * (n + m)));
            
            // P[n][m] = anm * sin(phi) * P[n-1][m] - bnm * P[n-2][m]
            cache.P[n][m] = anm * sinPhi * cache.P[n-1][m] - bnm * cache.P[n-2][m];
            
            // Derivative
            cache.dP[n][m] = anm * (cosPhi * cache.P[n-1][m] + sinPhi * cache.dP[n-1][m]) 
                           - bnm * cache.dP[n-2][m];
        }
    }
    
    // Update cache validity
    cache.sinPhi = sinPhi;
    cache.cosPhi = cosPhi;
    cache.maxDegree = maxN;
    cache.maxOrder = maxM;
    cache.valid = true;
}

void EarthGravityModel::calculateTrigonometric(double lambda, int maxOrder,
                                              std::vector<double>& cosMLambda,
                                              std::vector<double>& sinMLambda) const {
    // Ensure vectors are properly sized
    // Always allocate at least 2 elements to store cos(λ) and sin(λ) for coordinate transformation
    int minSize = std::max(2, maxOrder + 1);
    if (cosMLambda.size() < static_cast<size_t>(minSize)) {
        cosMLambda.resize(minSize);
        sinMLambda.resize(minSize);
    }
    
    // Calculate cos(m*lambda) and sin(m*lambda) using recursion
    cosMLambda[0] = 1.0;
    sinMLambda[0] = 0.0;
    
    // Always calculate cos(λ) and sin(λ) as they're needed for coordinate transformation
    cosMLambda[1] = std::cos(lambda);
    sinMLambda[1] = std::sin(lambda);
    
    // Use angle addition formulas for efficiency
    // cos((m+1)λ) = cos(mλ)cos(λ) - sin(mλ)sin(λ)
    // sin((m+1)λ) = sin(mλ)cos(λ) + cos(mλ)sin(λ)
    for (int m = 2; m <= maxOrder; ++m) {
        cosMLambda[m] = cosMLambda[m-1] * cosMLambda[1] - sinMLambda[m-1] * sinMLambda[1];
        sinMLambda[m] = sinMLambda[m-1] * cosMLambda[1] + cosMLambda[m-1] * sinMLambda[1];
    }
}

void EarthGravityModel::cartesianToSpherical(const math::Vector3D& position,
                                            double& r, double& phi, double& lambda) const {
    r = position.magnitude();
    
    if (r < EPSILON) {
        phi = 0.0;
        lambda = 0.0;
        return;
    }
    
    // Geocentric latitude
    phi = std::asin(position.z() / r);
    
    // Longitude
    lambda = std::atan2(position.y(), position.x());
}

bool EarthGravityModel::initialize(const ForceModelConfig& config) {
    m_config.maxDegree = config.getParameter<int>("maxDegree", m_config.maxDegree);
    m_config.maxOrder = config.getParameter<int>("maxOrder", m_config.maxOrder);
    m_config.computeGradient = config.getParameter<bool>("computeGradient", m_config.computeGradient);
    m_config.useCache = config.getParameter<bool>("useCache", m_config.useCache);
    m_config.cacheResolution = config.getParameter<double>("cacheResolution", m_config.cacheResolution);
    m_config.coefficientFile = config.getParameter<std::string>("coefficientFile", m_config.coefficientFile);
    
    // Load coefficients if file specified
    if (!m_config.coefficientFile.empty()) {
        if (!loadCoefficientsFromFile(m_config.coefficientFile)) {
            LOG_ERROR("Failed to load coefficients from: {}", m_config.coefficientFile);
            return false;
        }
    } else {
        // For degree 0 model, don't load higher order coefficients
        if (m_config.maxDegree > 0) {
            // Load default coefficients (at least J2)
            m_C[2][0] = -math::constants::EARTH_J2;
            m_C[3][0] = -math::constants::EARTH_J3;
            m_C[4][0] = -math::constants::EARTH_J4;
            m_maxDegreeLoaded = 4;
            m_maxOrderLoaded = 0;
        } else {
            // Just point mass
            m_maxDegreeLoaded = 0;
            m_maxOrderLoaded = 0;
        }
    }
    
    // Ensure requested degree/order doesn't exceed loaded coefficients
    m_config.maxDegree = std::min(m_config.maxDegree, m_maxDegreeLoaded);
    m_config.maxOrder = std::min(m_config.maxOrder, m_maxOrderLoaded);
    
    return true;
}

bool EarthGravityModel::validate() const {
    if (m_config.maxDegree < 0 || m_config.maxOrder < 0) {
        return false;
    }
    
    if (m_config.maxOrder > m_config.maxDegree) {
        return false;
    }
    
    if (m_mu <= 0.0 || m_radius <= 0.0) {
        return false;
    }
    
    return true;
}

std::unique_ptr<ForceModel> EarthGravityModel::clone() const {
    auto cloned = std::make_unique<EarthGravityModel>(m_name, m_config);
    
    // Copy coefficients
    cloned->m_C = m_C;
    cloned->m_S = m_S;
    cloned->m_maxDegreeLoaded = m_maxDegreeLoaded;
    cloned->m_maxOrderLoaded = m_maxOrderLoaded;
    cloned->m_mu = m_mu;
    cloned->m_radius = m_radius;
    cloned->m_enabled = m_enabled;
    
    return cloned;
}

std::string EarthGravityModel::toString() const {
    std::stringstream ss;
    ss << ForceModel::toString();
    ss << " [degree=" << m_config.maxDegree << ", order=" << m_config.maxOrder;
    ss << ", loaded=" << m_maxDegreeLoaded << "x" << m_maxOrderLoaded;
    if (m_config.useCache) {
        ss << ", cached";
    }
    ss << "]";
    return ss.str();
}

bool EarthGravityModel::loadCoefficients(const std::vector<SphericalHarmonicCoefficient>& coefficients) {
    // Reset coefficients
    for (auto& row : m_C) {
        std::fill(row.begin(), row.end(), 0.0);
    }
    for (auto& row : m_S) {
        std::fill(row.begin(), row.end(), 0.0);
    }
    
    m_C[0][0] = 1.0; // Normalized C00
    
    // Load provided coefficients
    m_maxDegreeLoaded = 0;
    m_maxOrderLoaded = 0;
    
    for (const auto& coeff : coefficients) {
        if (coeff.n < 0 || coeff.m < 0 || coeff.m > coeff.n) {
            LOG_WARN("Invalid coefficient: n={}, m={}", coeff.n, coeff.m);
            continue;
        }
        
        if (coeff.n >= static_cast<int>(m_C.size())) {
            // Resize if necessary
            int newSize = coeff.n + 1;
            m_C.resize(newSize);
            m_S.resize(newSize);
            for (int i = m_maxDegreeLoaded + 1; i < newSize; ++i) {
                m_C[i].resize(i + 1, 0.0);
                m_S[i].resize(i + 1, 0.0);
            }
        }
        
        m_C[coeff.n][coeff.m] = coeff.C;
        m_S[coeff.n][coeff.m] = coeff.S;
        
        m_maxDegreeLoaded = std::max(m_maxDegreeLoaded, coeff.n);
        m_maxOrderLoaded = std::max(m_maxOrderLoaded, coeff.m);
    }
    
    LOG_INFO("Loaded spherical harmonic coefficients up to degree {} and order {}", 
             m_maxDegreeLoaded, m_maxOrderLoaded);
    
    return true;
}

bool EarthGravityModel::loadCoefficientsFromFile(const std::string& filename) {
    std::vector<SphericalHarmonicCoefficient> coefficients;
    
    if (EGM2008CoefficientLoader::loadCoefficients(filename, coefficients, 
                                                   m_config.maxDegree, m_config.maxOrder)) {
        return loadCoefficients(coefficients);
    }
    
    return false;
}

void EarthGravityModel::setDegreeOrder(int degree, int order) {
    m_config.maxDegree = std::min(degree, m_maxDegreeLoaded);
    m_config.maxOrder = std::min(order, std::min(m_maxOrderLoaded, m_config.maxDegree));
    
    // Clear cache when configuration changes
    clearCache();
}

double EarthGravityModel::calculatePotential(const math::Vector3D& position) const {
    double r, phi, lambda;
    cartesianToSpherical(position, r, phi, lambda);
    
    if (r < MIN_RADIUS) {
        return 0.0;
    }
    
    double sinPhi = std::sin(phi);
    double cosPhi = std::cos(phi);
    
    // Calculate Legendre polynomials
    if (!t_legendreCache.valid || 
        std::abs(t_legendreCache.sinPhi - sinPhi) > EPSILON ||
        std::abs(t_legendreCache.cosPhi - cosPhi) > EPSILON ||
        t_legendreCache.maxDegree < m_config.maxDegree ||
        t_legendreCache.maxOrder < m_config.maxOrder) {
        calculateLegendre(sinPhi, cosPhi, t_legendreCache);
    }
    
    // Calculate trigonometric functions
    calculateTrigonometric(lambda, m_config.maxOrder, t_cosMLambda, t_sinMLambda);
    
    double potential = 0.0;
    double rRatio = m_radius / r;
    double rPower = 1.0;
    
    for (int n = 0; n <= m_config.maxDegree; ++n) {
        if (n > 0) {
            rPower *= rRatio; // (R/r)^n
        }
        
        for (int m = 0; m <= std::min(n, m_config.maxOrder); ++m) {
            double Cnm = m_C[n][m];
            double Snm = m_S[n][m];
            
            if (std::abs(Cnm) < EPSILON && std::abs(Snm) < EPSILON) {
                continue;
            }
            
            double Pnm = t_legendreCache.P[n][m];
            double cosMLambda = t_cosMLambda[m];
            double sinMLambda = t_sinMLambda[m];
            
            potential += rPower * Pnm * (Cnm * cosMLambda + Snm * sinMLambda);
        }
    }
    
    return -m_mu * potential / r;
}

math::Matrix3D EarthGravityModel::calculateGravityGradient(const math::Vector3D& position) const {
    // Implementation of gravity gradient tensor calculation
    // This is computationally intensive and typically only needed for
    // high-precision applications or gravity gradient torque calculations
    
    math::Matrix3D gradient;
    
    // Use finite differences for now (can be optimized with analytical derivatives)
    const double delta = 1.0; // 1 meter perturbation
    
    for (int i = 0; i < 3; ++i) {
        math::Vector3D perturbedPos = position;
        perturbedPos[i] += delta;
        math::Vector3D accPlus = calculateAccelerationDirect(perturbedPos);
        
        perturbedPos[i] = position[i] - delta;
        math::Vector3D accMinus = calculateAccelerationDirect(perturbedPos);
        
        math::Vector3D gradient_i = (accPlus - accMinus) / (2.0 * delta);
        
        for (int j = 0; j < 3; ++j) {
            gradient(j, i) = gradient_i[j];
        }
    }
    
    return gradient;
}

void EarthGravityModel::clearCache() const {
    std::lock_guard<std::mutex> lock(m_cacheMutex);
    m_accelerationCache.clear();
    m_cacheHits = 0;
    m_cacheMisses = 0;
}

void EarthGravityModel::getCacheStatistics(size_t& hits, size_t& misses, size_t& size) const {
    hits = m_cacheHits.load();
    misses = m_cacheMisses.load();
    
    std::lock_guard<std::mutex> lock(m_cacheMutex);
    size = m_accelerationCache.size();
}

uint64_t EarthGravityModel::getCacheKey(const math::Vector3D& position) const {
    // Quantize position to cache resolution
    int64_t x = static_cast<int64_t>(position.x() / m_config.cacheResolution);
    int64_t y = static_cast<int64_t>(position.y() / m_config.cacheResolution);
    int64_t z = static_cast<int64_t>(position.z() / m_config.cacheResolution);
    
    // Combine into single key using bit operations
    uint64_t key = 0;
    key |= (static_cast<uint64_t>(x & 0x1FFFFF) << 42); // 21 bits for x
    key |= (static_cast<uint64_t>(y & 0x1FFFFF) << 21); // 21 bits for y
    key |= (static_cast<uint64_t>(z & 0x1FFFFF));       // 21 bits for z
    
    return key;
}

// EGM2008CoefficientLoader implementation

bool EGM2008CoefficientLoader::loadCoefficients(const std::string& filename,
                                               std::vector<SphericalHarmonicCoefficient>& coefficients,
                                               int maxDegree, int maxOrder) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        LOG_ERROR("Failed to open coefficient file: {}", filename);
        return false;
    }
    
    coefficients.clear();
    std::string line;
    
    // Try to detect file format
    std::getline(file, line);
    file.seekg(0); // Reset to beginning
    
    // Check if it's EGM2008 format or generic format
    if (line.find("EGM") != std::string::npos || line.find("2008") != std::string::npos) {
        // EGM2008 format
        // Skip header lines
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#' || line[0] == '%') {
                continue;
            }
            
            // Try to parse coefficient line
            std::istringstream iss(line);
            int n, m;
            double C, S;
            
            if (iss >> n >> m >> C >> S) {
                // Check degree/order limits
                if (maxDegree >= 0 && n > maxDegree) continue;
                if (maxOrder >= 0 && m > maxOrder) continue;
                
                SphericalHarmonicCoefficient coeff;
                coeff.n = n;
                coeff.m = m;
                coeff.C = C;
                coeff.S = S;
                
                coefficients.push_back(coeff);
            }
        }
    } else {
        // Generic format: n m C S
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') {
                continue;
            }
            
            std::istringstream iss(line);
            int n, m;
            double C, S;
            
            if (iss >> n >> m >> C >> S) {
                // Check degree/order limits
                if (maxDegree >= 0 && n > maxDegree) continue;
                if (maxOrder >= 0 && m > maxOrder) continue;
                
                SphericalHarmonicCoefficient coeff;
                coeff.n = n;
                coeff.m = m;
                coeff.C = C;
                coeff.S = S;
                
                coefficients.push_back(coeff);
            }
        }
    }
    
    file.close();
    
    LOG_INFO("Loaded {} spherical harmonic coefficients from {}", coefficients.size(), filename);
    
    return !coefficients.empty();
}

std::string EGM2008CoefficientLoader::getDefaultDataPath() {
    // Return path to default EGM2008 data file
    // This would typically be in the data directory
    return "data/gravity/EGM2008_to360.txt";
}

bool EGM2008CoefficientLoader::validateFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    // Try to read a few lines and validate format
    std::string line;
    int validLines = 0;
    
    while (std::getline(file, line) && validLines < 10) {
        if (line.empty() || line[0] == '#' || line[0] == '%') {
            continue;
        }
        
        std::istringstream iss(line);
        int n, m;
        double C, S;
        
        if (iss >> n >> m >> C >> S) {
            // Basic validation
            if (n >= 0 && m >= 0 && m <= n) {
                validLines++;
            }
        }
    }
    
    file.close();
    
    return validLines > 0;
}

} // namespace forces
} // namespace physics
} // namespace iloss