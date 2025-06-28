#pragma once

#include "core/math/Vector3D.h"
#include "core/math/Matrix3D.h"
#include "core/math/Quaternion.h"
#include "core/time/Time.h"
#include "core/coordinates/CoordinateSystem.h"
#include <random>
#include <vector>
#include <memory>
#include <chrono>

namespace iloss {
namespace test {

/**
 * @brief Generates test data for various ILOSS data types
 * 
 * Provides deterministic and random data generation for testing
 */
class TestDataGenerator {
public:
    explicit TestDataGenerator(unsigned int seed = 12345)
        : m_rng(seed), m_seed(seed) {}
    
    void setSeed(unsigned int seed) {
        m_seed = seed;
        m_rng.seed(seed);
    }
    
    // Vector generation
    math::Vector3D randomVector(double minVal = -1.0, double maxVal = 1.0) {
        std::uniform_real_distribution<double> dist(minVal, maxVal);
        return math::Vector3D(dist(m_rng), dist(m_rng), dist(m_rng));
    }
    
    math::Vector3D randomUnitVector() {
        auto vec = randomVector();
        return vec.normalized();
    }
    
    math::Vector3D randomPosition(double minRadius = 6378137.0,  // Earth radius
                                 double maxRadius = 42164000.0) { // GEO radius
        double r = std::uniform_real_distribution<double>(minRadius, maxRadius)(m_rng);
        return randomUnitVector() * r;
    }
    
    math::Vector3D randomVelocity(double minSpeed = 1000.0,
                                 double maxSpeed = 11000.0) {
        double speed = std::uniform_real_distribution<double>(minSpeed, maxSpeed)(m_rng);
        return randomUnitVector() * speed;
    }
    
    std::vector<math::Vector3D> randomVectorArray(size_t count,
                                                 double minVal = -1.0,
                                                 double maxVal = 1.0) {
        std::vector<math::Vector3D> result;
        result.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            result.push_back(randomVector(minVal, maxVal));
        }
        return result;
    }
    
    // Matrix generation
    math::Matrix3D randomMatrix(double minVal = -1.0, double maxVal = 1.0) {
        std::uniform_real_distribution<double> dist(minVal, maxVal);
        double elements[9];
        for (int i = 0; i < 9; ++i) {
            elements[i] = dist(m_rng);
        }
        return math::Matrix3D(elements[0], elements[1], elements[2],
                             elements[3], elements[4], elements[5],
                             elements[6], elements[7], elements[8]);
    }
    
    math::Matrix3D randomRotationMatrix() {
        // Generate random rotation using quaternion
        auto q = randomQuaternion();
        return q.toMatrix();
    }
    
    math::Matrix3D randomSymmetricMatrix(double minVal = -1.0, double maxVal = 1.0) {
        std::uniform_real_distribution<double> dist(minVal, maxVal);
        double a = dist(m_rng), b = dist(m_rng), c = dist(m_rng);
        double d = dist(m_rng), e = dist(m_rng), f = dist(m_rng);
        return math::Matrix3D(a, b, c,
                             b, d, e,
                             c, e, f);
    }
    
    // Quaternion generation
    math::Quaternion randomQuaternion() {
        // Use algorithm from "Uniform Random Rotations" by Ken Shoemake
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        double u1 = dist(m_rng);
        double u2 = dist(m_rng) * 2.0 * M_PI;
        double u3 = dist(m_rng) * 2.0 * M_PI;
        
        double a = std::sqrt(1.0 - u1);
        double b = std::sqrt(u1);
        
        return math::Quaternion(
            a * std::sin(u2),
            a * std::cos(u2),
            b * std::sin(u3),
            b * std::cos(u3)
        ).normalized();
    }
    
    // Time generation
    time::Time randomTime(const time::Time& minTime, const time::Time& maxTime) {
        double minJD = minTime.getJulianDate();
        double maxJD = maxTime.getJulianDate();
        std::uniform_real_distribution<double> dist(minJD, maxJD);
        return time::Time::fromJulianDate(dist(m_rng));
    }
    
    time::Time randomTimeInYear(int year) {
        time::Time start(year, 1, 1, 0, 0, 0.0);
        time::Time end(year, 12, 31, 23, 59, 59.999);
        return randomTime(start, end);
    }
    
    std::vector<time::Time> randomTimeSequence(size_t count,
                                              const time::Time& start,
                                              double minDeltaSeconds = 1.0,
                                              double maxDeltaSeconds = 60.0) {
        std::vector<time::Time> result;
        result.reserve(count);
        
        time::Time current = start;
        result.push_back(current);
        
        std::uniform_real_distribution<double> dist(minDeltaSeconds, maxDeltaSeconds);
        for (size_t i = 1; i < count; ++i) {
            current = current + dist(m_rng);
            result.push_back(current);
        }
        
        return result;
    }
    
    // Orbital elements generation
    struct OrbitalElements {
        double semiMajorAxis;    // meters
        double eccentricity;     // dimensionless
        double inclination;      // radians
        double raan;            // radians (right ascension of ascending node)
        double argOfPerigee;    // radians
        double trueAnomaly;     // radians
    };
    
    OrbitalElements randomOrbitalElements(bool bounded = true) {
        OrbitalElements elements;
        
        if (bounded) {
            // Generate reasonable orbital elements
            elements.semiMajorAxis = std::uniform_real_distribution<double>(
                6578137.0, 50000000.0)(m_rng);  // LEO to beyond GEO
            elements.eccentricity = std::uniform_real_distribution<double>(
                0.0, 0.8)(m_rng);  // Bound to elliptical orbits
        } else {
            // Allow any values
            elements.semiMajorAxis = std::uniform_real_distribution<double>(
                1000000.0, 1e9)(m_rng);
            elements.eccentricity = std::uniform_real_distribution<double>(
                0.0, 2.0)(m_rng);  // Include hyperbolic
        }
        
        elements.inclination = std::uniform_real_distribution<double>(
            0.0, M_PI)(m_rng);
        elements.raan = std::uniform_real_distribution<double>(
            0.0, 2.0 * M_PI)(m_rng);
        elements.argOfPerigee = std::uniform_real_distribution<double>(
            0.0, 2.0 * M_PI)(m_rng);
        elements.trueAnomaly = std::uniform_real_distribution<double>(
            0.0, 2.0 * M_PI)(m_rng);
        
        return elements;
    }
    
    // State vector generation
    struct StateVector {
        math::Vector3D position;  // meters
        math::Vector3D velocity;  // m/s
        double mass;             // kg
        time::Time epoch;
    };
    
    StateVector randomStateVector(const time::Time& epoch = time::Time()) {
        StateVector state;
        state.position = randomPosition();
        state.velocity = randomVelocity();
        state.mass = std::uniform_real_distribution<double>(100.0, 10000.0)(m_rng);
        state.epoch = epoch.isValid() ? epoch : randomTimeInYear(2025);
        return state;
    }
    
    std::vector<StateVector> randomTrajectory(size_t count,
                                             const time::Time& startEpoch,
                                             double timestep = 10.0) {
        std::vector<StateVector> trajectory;
        trajectory.reserve(count);
        
        // Generate initial state
        auto state = randomStateVector(startEpoch);
        trajectory.push_back(state);
        
        // Simple propagation with random perturbations
        for (size_t i = 1; i < count; ++i) {
            state.epoch = state.epoch + timestep;
            
            // Simple forward Euler with small random perturbations
            state.position = state.position + state.velocity * timestep;
            state.velocity = state.velocity + randomVector(-0.01, 0.01);
            state.mass *= std::uniform_real_distribution<double>(0.999, 1.0)(m_rng);
            
            trajectory.push_back(state);
        }
        
        return trajectory;
    }
    
    // Configuration data generation
    nlohmann::json randomConfig(int depth = 3, int maxKeys = 5) {
        if (depth <= 0) {
            // Generate leaf value
            std::uniform_int_distribution<int> typeDist(0, 3);
            switch (typeDist(m_rng)) {
                case 0: // boolean
                    return std::uniform_int_distribution<int>(0, 1)(m_rng) == 1;
                case 1: // number
                    return std::uniform_real_distribution<double>(-100.0, 100.0)(m_rng);
                case 2: // string
                    return "test_string_" + std::to_string(
                        std::uniform_int_distribution<int>(0, 1000)(m_rng));
                default: // array
                    nlohmann::json arr = nlohmann::json::array();
                    int size = std::uniform_int_distribution<int>(1, 5)(m_rng);
                    for (int i = 0; i < size; ++i) {
                        arr.push_back(std::uniform_real_distribution<double>(
                            -10.0, 10.0)(m_rng));
                    }
                    return arr;
            }
        }
        
        // Generate object
        nlohmann::json obj = nlohmann::json::object();
        int numKeys = std::uniform_int_distribution<int>(1, maxKeys)(m_rng);
        
        for (int i = 0; i < numKeys; ++i) {
            std::string key = "key_" + std::to_string(i);
            obj[key] = randomConfig(depth - 1, maxKeys);
        }
        
        return obj;
    }
    
    // String generation
    std::string randomString(size_t minLength = 5, size_t maxLength = 20) {
        static const char charset[] = 
            "abcdefghijklmnopqrstuvwxyz"
            "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
            "0123456789";
        
        std::uniform_int_distribution<size_t> lengthDist(minLength, maxLength);
        std::uniform_int_distribution<size_t> charDist(0, sizeof(charset) - 2);
        
        size_t length = lengthDist(m_rng);
        std::string result;
        result.reserve(length);
        
        for (size_t i = 0; i < length; ++i) {
            result += charset[charDist(m_rng)];
        }
        
        return result;
    }
    
    std::string randomIdentifier() {
        return "id_" + std::to_string(
            std::uniform_int_distribution<int>(10000, 99999)(m_rng));
    }
    
    std::string randomFilename(const std::string& extension = ".dat") {
        return randomString(8, 12) + "_" + 
               std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
               extension;
    }
    
private:
    std::mt19937 m_rng;
    unsigned int m_seed;
};

/**
 * @brief Specialized generator for physics test data
 */
class PhysicsTestDataGenerator : public TestDataGenerator {
public:
    using TestDataGenerator::TestDataGenerator;
    
    // Generate state on a Keplerian orbit
    StateVector keplerianState(const OrbitalElements& elements,
                              double gravitationalParameter = 3.986004418e14) {
        // Convert orbital elements to state vector
        double a = elements.semiMajorAxis;
        double e = elements.eccentricity;
        double i = elements.inclination;
        double Omega = elements.raan;
        double omega = elements.argOfPerigee;
        double nu = elements.trueAnomaly;
        
        // Position in perifocal frame
        double r = a * (1 - e * e) / (1 + e * std::cos(nu));
        double x_pqw = r * std::cos(nu);
        double y_pqw = r * std::sin(nu);
        
        // Velocity in perifocal frame
        double h = std::sqrt(gravitationalParameter * a * (1 - e * e));
        double vx_pqw = -gravitationalParameter / h * std::sin(nu);
        double vy_pqw = gravitationalParameter / h * (e + std::cos(nu));
        
        // Rotation matrix from perifocal to inertial
        double cos_Omega = std::cos(Omega);
        double sin_Omega = std::sin(Omega);
        double cos_i = std::cos(i);
        double sin_i = std::sin(i);
        double cos_omega = std::cos(omega);
        double sin_omega = std::sin(omega);
        
        math::Matrix3D rot(
            cos_Omega * cos_omega - sin_Omega * sin_omega * cos_i,
            -cos_Omega * sin_omega - sin_Omega * cos_omega * cos_i,
            sin_Omega * sin_i,
            sin_Omega * cos_omega + cos_Omega * sin_omega * cos_i,
            -sin_Omega * sin_omega + cos_Omega * cos_omega * cos_i,
            -cos_Omega * sin_i,
            sin_omega * sin_i,
            cos_omega * sin_i,
            cos_i
        );
        
        // Transform to inertial frame
        math::Vector3D pos_pqw(x_pqw, y_pqw, 0.0);
        math::Vector3D vel_pqw(vx_pqw, vy_pqw, 0.0);
        
        StateVector state;
        state.position = rot * pos_pqw;
        state.velocity = rot * vel_pqw;
        state.mass = 1000.0;  // Default mass
        state.epoch = time::Time();  // Current time
        
        return state;
    }
    
    // Generate circular orbit state
    StateVector circularOrbitState(double radius, double inclination,
                                  double raan = 0.0) {
        OrbitalElements elements;
        elements.semiMajorAxis = radius;
        elements.eccentricity = 0.0;
        elements.inclination = inclination;
        elements.raan = raan;
        elements.argOfPerigee = 0.0;
        elements.trueAnomaly = randomAngle();
        
        return keplerianState(elements);
    }
    
    // Generate geostationary orbit state
    StateVector geostationaryState(double longitude) {
        const double GEO_RADIUS = 42164169.0;  // meters
        
        StateVector state;
        state.position = math::Vector3D(
            GEO_RADIUS * std::cos(longitude),
            GEO_RADIUS * std::sin(longitude),
            0.0
        );
        
        // Velocity for circular orbit
        const double GM = 3.986004418e14;
        double v = std::sqrt(GM / GEO_RADIUS);
        state.velocity = math::Vector3D(
            -v * std::sin(longitude),
            v * std::cos(longitude),
            0.0
        );
        
        state.mass = 2000.0;  // Typical GEO satellite
        state.epoch = time::Time();
        
        return state;
    }
    
private:
    double randomAngle() {
        return std::uniform_real_distribution<double>(0.0, 2.0 * M_PI)(m_rng);
    }
};

} // namespace test
} // namespace iloss