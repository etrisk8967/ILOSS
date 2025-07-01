#include "physics/aerodynamics/AerodynamicDatabase.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>
#include <iomanip>

namespace iloss {
namespace physics {
namespace aerodynamics {

// Constants
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double EPSILON = 1e-10;

AerodynamicDatabase::AerodynamicDatabase() 
    : m_config() {
}

AerodynamicDatabase::AerodynamicDatabase(const Config& config)
    : m_config(config) {
}

AerodynamicDatabase::~AerodynamicDatabase() = default;

void AerodynamicDatabase::loadFromCSV(const std::string& filename, 
                                     const std::string& configName) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open aerodynamic database file: " + filename);
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    loadFromString(buffer.str(), configName);
}

void AerodynamicDatabase::loadFromString(const std::string& csvData,
                                       const std::string& configName) {
    auto config = std::make_unique<ConfigurationData>();
    
    std::istringstream stream(csvData);
    std::string line;
    int lineNumber = 0;
    
    while (std::getline(stream, line)) {
        lineNumber++;
        
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        try {
            config->dataPoints.push_back(parseCSVLine(line));
        } catch (const std::exception& e) {
            throw std::runtime_error("Error parsing line " + std::to_string(lineNumber) + 
                                   ": " + e.what());
        }
    }
    
    if (config->dataPoints.empty()) {
        throw std::runtime_error("No valid data points found in aerodynamic database");
    }
    
    // Extract unique Mach numbers and angles of attack
    std::set<double> machSet, aoaSet;
    for (const auto& point : config->dataPoints) {
        machSet.insert(point.mach);
        aoaSet.insert(point.angleOfAttack);
    }
    
    config->uniqueMachNumbers.assign(machSet.begin(), machSet.end());
    config->uniqueAoA.assign(aoaSet.begin(), aoaSet.end());
    
    // Validate grid structure if requested
    if (m_config.validateMonotonicity) {
        if (!std::is_sorted(config->uniqueMachNumbers.begin(), 
                           config->uniqueMachNumbers.end())) {
            throw std::runtime_error("Mach numbers are not monotonically increasing");
        }
        if (!std::is_sorted(config->uniqueAoA.begin(), 
                           config->uniqueAoA.end())) {
            throw std::runtime_error("Angles of attack are not monotonically increasing");
        }
    }
    
    // Build grid index map
    buildGridIndexMap(*config);
    
    // Store configuration
    m_configurations[configName] = std::move(config);
    
    // Clear cache when new data is loaded
    clearCache();
}

AerodynamicDataPoint AerodynamicDatabase::parseCSVLine(const std::string& line) const {
    std::istringstream iss(line);
    std::vector<double> values;
    std::string value;
    
    while (std::getline(iss, value, ',')) {
        // Trim whitespace
        value.erase(0, value.find_first_not_of(" \t"));
        value.erase(value.find_last_not_of(" \t") + 1);
        
        try {
            values.push_back(std::stod(value));
        } catch (const std::exception&) {
            throw std::runtime_error("Invalid numeric value: " + value);
        }
    }
    
    if (values.size() < 3) {
        throw std::runtime_error("Insufficient data columns (need at least Mach, AoA, CD)");
    }
    
    double mach = values[0];
    double aoaDeg = values[1];
    double aoaRad = aoaDeg * DEG_TO_RAD;
    
    // Handle different CSV formats
    AerodynamicCoefficients coeffs;
    
    if (values.size() >= 3) coeffs.setDragCoefficient(values[2]);
    if (values.size() >= 4) coeffs.setLiftCoefficient(values[3]);
    if (values.size() >= 5) coeffs.setSideForceCoefficient(values[4]);
    if (values.size() >= 6) coeffs.setRollMomentCoefficient(values[5]);
    if (values.size() >= 7) coeffs.setPitchMomentCoefficient(values[6]);
    if (values.size() >= 8) coeffs.setYawMomentCoefficient(values[7]);
    
    return AerodynamicDataPoint(mach, aoaRad, coeffs);
}

void AerodynamicDatabase::buildGridIndexMap(ConfigurationData& config) const {
    // Build a map from (mach_index, aoa_index) to data point index
    for (size_t i = 0; i < config.dataPoints.size(); ++i) {
        const auto& point = config.dataPoints[i];
        
        // Find indices in unique arrays
        auto machIt = std::lower_bound(config.uniqueMachNumbers.begin(),
                                      config.uniqueMachNumbers.end(),
                                      point.mach);
        auto aoaIt = std::lower_bound(config.uniqueAoA.begin(),
                                     config.uniqueAoA.end(),
                                     point.angleOfAttack);
        
        if (machIt != config.uniqueMachNumbers.end() && 
            aoaIt != config.uniqueAoA.end() &&
            std::abs(*machIt - point.mach) < EPSILON &&
            std::abs(*aoaIt - point.angleOfAttack) < EPSILON) {
            
            size_t machIdx = std::distance(config.uniqueMachNumbers.begin(), machIt);
            size_t aoaIdx = std::distance(config.uniqueAoA.begin(), aoaIt);
            
            config.gridIndexMap[{machIdx, aoaIdx}] = i;
        }
    }
}

AerodynamicCoefficients AerodynamicDatabase::getCoefficients(
    double mach, double angleOfAttack, const std::string& configName) const {
    
    // Check cache first
    if (m_config.enableCaching) {
        std::lock_guard<std::mutex> lock(m_cacheMutex);
        auto cacheKey = std::make_tuple(mach, angleOfAttack, configName);
        auto it = m_cache.find(cacheKey);
        if (it != m_cache.end()) {
            m_cacheHits++;
            return it->second;
        }
        m_cacheMisses++;
    }
    
    // Find configuration
    auto configIt = m_configurations.find(configName);
    if (configIt == m_configurations.end()) {
        throw std::runtime_error("Aerodynamic configuration not found: " + configName);
    }
    
    const auto& config = *configIt->second;
    if (config.dataPoints.empty()) {
        throw std::runtime_error("No data points in configuration: " + configName);
    }
    
    // Check for extrapolation
    if (!m_config.allowExtrapolation) {
        if (mach < config.uniqueMachNumbers.front() || 
            mach > config.uniqueMachNumbers.back() ||
            angleOfAttack < config.uniqueAoA.front() || 
            angleOfAttack > config.uniqueAoA.back()) {
            throw std::runtime_error("Extrapolation not allowed for Mach=" + 
                                   std::to_string(mach) + ", AoA=" + 
                                   std::to_string(angleOfAttack));
        }
    } else if (m_config.extrapolationWarningThreshold > 0) {
        if (isSignificantExtrapolation(mach, config.uniqueMachNumbers.front(), 
                                      config.uniqueMachNumbers.back()) ||
            isSignificantExtrapolation(angleOfAttack, config.uniqueAoA.front(), 
                                      config.uniqueAoA.back())) {
            std::cerr << "Warning: Significant extrapolation for Mach=" << mach 
                     << ", AoA=" << angleOfAttack << " rad" << std::endl;
        }
    }
    
    // Perform interpolation
    AerodynamicCoefficients result = interpolate(mach, angleOfAttack, config);
    
    // Cache result
    if (m_config.enableCaching) {
        std::lock_guard<std::mutex> lock(m_cacheMutex);
        auto cacheKey = std::make_tuple(mach, angleOfAttack, configName);
        m_cache[cacheKey] = result;
        
        // Trim cache if needed
        if (m_cache.size() > m_config.cacheSize) {
            trimCache();
        }
    }
    
    return result;
}

AerodynamicCoefficients AerodynamicDatabase::interpolate(
    double mach, double aoa, const ConfigurationData& config) const {
    
    // Find bracketing indices
    size_t machLow, machHigh;
    double machFrac;
    findBracketingIndices(mach, config.uniqueMachNumbers, machLow, machHigh, machFrac);
    
    size_t aoaLow, aoaHigh;
    double aoaFrac;
    findBracketingIndices(aoa, config.uniqueAoA, aoaLow, aoaHigh, aoaFrac);
    
    // Get the four corner points
    auto getPoint = [&](size_t mi, size_t ai) -> const AerodynamicCoefficients* {
        auto it = config.gridIndexMap.find({mi, ai});
        if (it != config.gridIndexMap.end()) {
            return &config.dataPoints[it->second].coefficients;
        }
        return nullptr;
    };
    
    const auto* p00 = getPoint(machLow, aoaLow);
    const auto* p01 = getPoint(machLow, aoaHigh);
    const auto* p10 = getPoint(machHigh, aoaLow);
    const auto* p11 = getPoint(machHigh, aoaHigh);
    
    // Handle missing data points
    if (!p00 || !p01 || !p10 || !p11) {
        // Fall back to nearest neighbor
        double minDist = std::numeric_limits<double>::max();
        const AerodynamicCoefficients* nearest = nullptr;
        
        for (const auto& point : config.dataPoints) {
            double machDist = (point.mach - mach) / (config.uniqueMachNumbers.back() - 
                                                    config.uniqueMachNumbers.front());
            double aoaDist = (point.angleOfAttack - aoa) / (config.uniqueAoA.back() - 
                                                           config.uniqueAoA.front());
            double dist = std::sqrt(machDist * machDist + aoaDist * aoaDist);
            
            if (dist < minDist) {
                minDist = dist;
                nearest = &point.coefficients;
            }
        }
        
        if (nearest) {
            return *nearest;
        } else {
            throw std::runtime_error("No valid data points for interpolation");
        }
    }
    
    // Bilinear interpolation
    // First interpolate in Mach direction
    AerodynamicCoefficients lower = p00->interpolate(*p10, machFrac);
    AerodynamicCoefficients upper = p01->interpolate(*p11, machFrac);
    
    // Then interpolate in AoA direction
    return lower.interpolate(upper, aoaFrac);
}

void AerodynamicDatabase::findBracketingIndices(
    double value, const std::vector<double>& sortedValues,
    size_t& lowIndex, size_t& highIndex, double& fraction) const {
    
    if (sortedValues.size() < 2) {
        lowIndex = highIndex = 0;
        fraction = 0.0;
        return;
    }
    
    // Handle extrapolation cases
    if (value <= sortedValues.front()) {
        lowIndex = 0;
        highIndex = 1;
        fraction = 0.0;
        return;
    }
    
    if (value >= sortedValues.back()) {
        lowIndex = sortedValues.size() - 2;
        highIndex = sortedValues.size() - 1;
        fraction = 1.0;
        return;
    }
    
    // Binary search for bracketing indices
    auto it = std::lower_bound(sortedValues.begin(), sortedValues.end(), value);
    if (it == sortedValues.begin()) {
        lowIndex = 0;
        highIndex = 1;
    } else if (it == sortedValues.end()) {
        lowIndex = sortedValues.size() - 2;
        highIndex = sortedValues.size() - 1;
    } else {
        highIndex = std::distance(sortedValues.begin(), it);
        lowIndex = highIndex - 1;
    }
    
    // Calculate interpolation fraction
    double span = sortedValues[highIndex] - sortedValues[lowIndex];
    if (std::abs(span) > EPSILON) {
        fraction = (value - sortedValues[lowIndex]) / span;
        fraction = std::max(0.0, std::min(1.0, fraction));
    } else {
        fraction = 0.0;
    }
}

bool AerodynamicDatabase::isSignificantExtrapolation(
    double value, double min, double max) const {
    
    double range = max - min;
    if (range <= 0) return false;
    
    if (value < min) {
        return (min - value) / range > m_config.extrapolationWarningThreshold;
    } else if (value > max) {
        return (value - max) / range > m_config.extrapolationWarningThreshold;
    }
    
    return false;
}

bool AerodynamicDatabase::hasConfiguration(const std::string& configName) const {
    return m_configurations.find(configName) != m_configurations.end();
}

std::vector<std::string> AerodynamicDatabase::getConfigurationNames() const {
    std::vector<std::string> names;
    names.reserve(m_configurations.size());
    
    for (const auto& pair : m_configurations) {
        names.push_back(pair.first);
    }
    
    return names;
}

bool AerodynamicDatabase::getDataRange(const std::string& configName,
                                      double& minMach, double& maxMach,
                                      double& minAoA, double& maxAoA) const {
    auto it = m_configurations.find(configName);
    if (it == m_configurations.end()) {
        return false;
    }
    
    const auto& config = *it->second;
    if (config.uniqueMachNumbers.empty() || config.uniqueAoA.empty()) {
        return false;
    }
    
    minMach = config.uniqueMachNumbers.front();
    maxMach = config.uniqueMachNumbers.back();
    minAoA = config.uniqueAoA.front();
    maxAoA = config.uniqueAoA.back();
    
    return true;
}

void AerodynamicDatabase::clearCache() const {
    std::lock_guard<std::mutex> lock(m_cacheMutex);
    m_cache.clear();
    m_cacheHits = 0;
    m_cacheMisses = 0;
}

void AerodynamicDatabase::getCacheStats(size_t& hits, size_t& misses, size_t& size) const {
    std::lock_guard<std::mutex> lock(m_cacheMutex);
    hits = m_cacheHits;
    misses = m_cacheMisses;
    size = m_cache.size();
}

bool AerodynamicDatabase::validateData(const std::string& configName) const {
    auto it = m_configurations.find(configName);
    if (it == m_configurations.end()) {
        return false;
    }
    
    const auto& config = *it->second;
    
    // Check for duplicate points
    std::set<std::pair<double, double>> uniquePoints;
    for (const auto& point : config.dataPoints) {
        auto key = std::make_pair(point.mach, point.angleOfAttack);
        if (!uniquePoints.insert(key).second) {
            std::cerr << "Duplicate data point at Mach=" << point.mach 
                     << ", AoA=" << point.angleOfAttack << std::endl;
            return false;
        }
    }
    
    // Validate coefficients
    for (const auto& point : config.dataPoints) {
        if (!point.coefficients.isValid()) {
            std::cerr << "Invalid coefficients at Mach=" << point.mach 
                     << ", AoA=" << point.angleOfAttack << std::endl;
            return false;
        }
    }
    
    // Check grid completeness (optional - may have sparse data)
    size_t expectedPoints = config.uniqueMachNumbers.size() * config.uniqueAoA.size();
    if (config.dataPoints.size() != expectedPoints) {
        std::cerr << "Warning: Incomplete grid. Expected " << expectedPoints 
                 << " points, found " << config.dataPoints.size() << std::endl;
    }
    
    return true;
}

void AerodynamicDatabase::exportToCSV(const std::string& filename,
                                    const std::string& configName) const {
    auto it = m_configurations.find(configName);
    if (it == m_configurations.end()) {
        throw std::runtime_error("Configuration not found: " + configName);
    }
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file for writing: " + filename);
    }
    
    // Write header
    file << "# Aerodynamic Coefficient Database\n";
    file << "# Configuration: " << configName << "\n";
    file << "# Mach, AoA_deg, CD, CL, CY, Cl, Cm, Cn\n";
    
    // Sort data points for nice output
    auto points = it->second->dataPoints;
    std::sort(points.begin(), points.end(), 
              [](const auto& a, const auto& b) {
                  if (std::abs(a.mach - b.mach) > EPSILON) {
                      return a.mach < b.mach;
                  }
                  return a.angleOfAttack < b.angleOfAttack;
              });
    
    // Write data
    file << std::fixed << std::setprecision(6);
    for (const auto& point : points) {
        file << point.mach << ", "
             << point.angleOfAttack / DEG_TO_RAD << ", "
             << point.coefficients.getDragCoefficient() << ", "
             << point.coefficients.getLiftCoefficient() << ", "
             << point.coefficients.getSideForceCoefficient() << ", "
             << point.coefficients.getRollMomentCoefficient() << ", "
             << point.coefficients.getPitchMomentCoefficient() << ", "
             << point.coefficients.getYawMomentCoefficient() << "\n";
    }
}

void AerodynamicDatabase::trimCache() const {
    // Simple strategy: clear half the cache
    // More sophisticated strategies could use LRU, frequency, etc.
    size_t targetSize = m_config.cacheSize / 2;
    
    auto it = m_cache.begin();
    while (m_cache.size() > targetSize && it != m_cache.end()) {
        it = m_cache.erase(it);
    }
}

} // namespace aerodynamics
} // namespace physics
} // namespace iloss