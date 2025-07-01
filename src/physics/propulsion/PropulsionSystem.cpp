#include "physics/forces/propulsion/PropulsionSystem.h"
#include "core/logging/Logger.h"
#include <algorithm>
#include <numeric>
#include <sstream>
#include <iomanip>

namespace iloss {
namespace physics {
namespace propulsion {

using namespace iloss::logging;
using namespace iloss::math;

PropulsionSystem::PropulsionSystem(const Configuration& config)
    : m_config(config)
    , m_state() {
    
    validateConfiguration();
    
    // Initialize state with dry mass properties
    m_state.totalMass = m_config.dryMass;
    m_state.centerOfMass = m_config.dryCenterOfMass;
    m_state.inertiaTensor = m_config.dryInertiaTensor;
    
    // Initialize inverse inertia tensor
    m_inverseInertiaTensor = m_config.dryInertiaTensor.inverse();
    m_inverseInertiaTensorValid = true;
    
    Logger::getInstance().logf(LogLevel::Info, LogCategory::Physics,
        "Created propulsion system with dry mass {:.1f} kg", m_config.dryMass);
}

void PropulsionSystem::validateConfiguration() const {
    if (m_config.dryMass <= 0.0) {
        throw std::invalid_argument("Dry mass must be positive");
    }
    
    // Check inertia tensor is positive definite using Sylvester's criterion
    // For a 3x3 symmetric matrix, check:
    // 1. All diagonal elements are positive
    // 2. All leading principal minors have positive determinant
    double I11 = m_config.dryInertiaTensor(0, 0);
    double I22 = m_config.dryInertiaTensor(1, 1);
    double I33 = m_config.dryInertiaTensor(2, 2);
    
    if (I11 <= 0.0 || I22 <= 0.0 || I33 <= 0.0) {
        throw std::invalid_argument("Inertia tensor diagonal elements must be positive");
    }
    
    // Check 2x2 principal minor
    double I12 = m_config.dryInertiaTensor(0, 1);
    double minor2x2 = I11 * I22 - I12 * I12;
    if (minor2x2 <= 0.0) {
        throw std::invalid_argument("Inertia tensor 2x2 principal minor must be positive");
    }
    
    // Check full determinant
    double det = m_config.dryInertiaTensor.determinant();
    if (det <= 0.0) {
        throw std::invalid_argument("Inertia tensor determinant must be positive");
    }
    
    // Check triangle inequality for inertia tensor
    double Ixx = m_config.dryInertiaTensor(0, 0);
    double Iyy = m_config.dryInertiaTensor(1, 1);
    double Izz = m_config.dryInertiaTensor(2, 2);
    
    if (Ixx + Iyy <= Izz || Iyy + Izz <= Ixx || Izz + Ixx <= Iyy) {
        throw std::invalid_argument("Inertia tensor violates triangle inequality");
    }
    
    if (m_config.structuralDamping < 0.0 || m_config.structuralDamping > 1.0) {
        throw std::invalid_argument("Structural damping must be in [0,1]");
    }
    
    if (m_config.fuelMargin < 0.0 || m_config.fuelMargin > 0.1) {
        throw std::invalid_argument("Fuel margin must be in [0,0.1]");
    }
}

std::string PropulsionSystem::addEngine(std::shared_ptr<Engine> engine,
                                       const EngineMounting& mounting) {
    if (!engine) {
        throw std::invalid_argument("Engine pointer cannot be null");
    }
    
    std::string engineId = mounting.engineId.empty() ? 
        engine->getName() : mounting.engineId;
    
    if (m_engines.find(engineId) != m_engines.end()) {
        throw std::invalid_argument("Engine with ID '" + engineId + "' already exists");
    }
    
    m_engines[engineId] = engine;
    m_engineMountings[engineId] = mounting;
    
    Logger::getInstance().logf(LogLevel::Info, LogCategory::Physics,
        "Added engine '{}' at position ({:.2f}, {:.2f}, {:.2f})",
        engineId, mounting.position.x(), mounting.position.y(), mounting.position.z());
    
    return engineId;
}

std::string PropulsionSystem::addTank(std::shared_ptr<PropellantTank> tank) {
    if (!tank) {
        throw std::invalid_argument("Tank pointer cannot be null");
    }
    
    std::string tankId = tank->getName();
    
    if (m_tanks.find(tankId) != m_tanks.end()) {
        throw std::invalid_argument("Tank with ID '" + tankId + "' already exists");
    }
    
    m_tanks[tankId] = tank;
    
    // Update total propellant mass
    m_state.totalPropellantMass += tank->getCurrentMass();
    updateMassProperties();
    
    Logger::getInstance().logf(LogLevel::Info, LogCategory::Physics,
        "Added tank '{}' with {:.1f} kg propellant",
        tankId, tank->getCurrentMass());
    
    return tankId;
}

void PropulsionSystem::assignTanksToEngine(const std::string& engineId,
                                          const std::vector<std::string>& tankIds) {
    auto engineIt = m_engineMountings.find(engineId);
    if (engineIt == m_engineMountings.end()) {
        throw std::runtime_error("Engine '" + engineId + "' not found");
    }
    
    // Verify all tanks exist
    for (const auto& tankId : tankIds) {
        if (m_tanks.find(tankId) == m_tanks.end()) {
            throw std::runtime_error("Tank '" + tankId + "' not found");
        }
    }
    
    engineIt->second.assignedTanks = tankIds;
    
    Logger::getInstance().logf(LogLevel::Info, LogCategory::Physics,
        "Assigned {} tanks to engine '{}'", tankIds.size(), engineId);
}

std::shared_ptr<Engine> PropulsionSystem::getEngine(const std::string& engineId) const {
    auto it = m_engines.find(engineId);
    return (it != m_engines.end()) ? it->second : nullptr;
}

std::shared_ptr<PropellantTank> PropulsionSystem::getTank(const std::string& tankId) const {
    auto it = m_tanks.find(tankId);
    return (it != m_tanks.end()) ? it->second : nullptr;
}

const PropulsionSystem::EngineMounting* PropulsionSystem::getEngineMounting(
    const std::string& engineId) const {
    auto it = m_engineMountings.find(engineId);
    return (it != m_engineMountings.end()) ? &it->second : nullptr;
}

void PropulsionSystem::update(double deltaTime, double ambientPressure) {
    if (deltaTime <= 0.0) {
        return;
    }
    
    m_currentTime += deltaTime;
    
    // Reset flow rate and thrust
    m_state.totalMassFlowRate = 0.0;
    m_state.totalThrust = 0.0;
    m_state.activeEngineCount = 0;
    
    // Process each engine
    for (auto& [engineId, engine] : m_engines) {
        if (!engine->isActive()) {
            continue;
        }
        
        m_state.activeEngineCount++;
        
        // Get mass flow rate for this engine
        double massFlowRate = -engine->getMassFlowRate(ambientPressure);  // Positive consumption
        if (massFlowRate <= 0.0) {
            continue;
        }
        
        // Distribute propellant consumption
        double requiredMass = massFlowRate * deltaTime;
        double actualMass = distributePropellantConsumption(*engine, requiredMass, deltaTime);
        
        if (actualMass < requiredMass * 0.95) {
            // Not enough propellant - shutdown engine
            Logger::getInstance().logf(LogLevel::Warning, LogCategory::Physics,
                "Engine '{}' starved of propellant, shutting down", engineId);
            engine->shutdown();
        } else {
            // Update engine statistics
            engine->updateStatistics(deltaTime, ambientPressure);
            
            // Update system totals
            m_state.totalMassFlowRate += massFlowRate;
            m_state.totalThrust += engine->getThrust(ambientPressure);
        }
        
        // Update TVC if present
        auto& mounting = m_engineMountings[engineId];
        if (mounting.tvc) {
            mounting.tvc->update(deltaTime);
        }
    }
    
    // Update mass properties
    updateMassProperties();
}

Vector3D PropulsionSystem::getTotalThrustVector(double ambientPressure) const {
    Vector3D totalThrust = Vector3D::zero();
    
    for (const auto& [engineId, engine] : m_engines) {
        if (!engine->isActive()) {
            continue;
        }
        
        // Get thrust magnitude
        double thrustMag = engine->getThrust(ambientPressure);
        if (thrustMag <= 0.0) {
            continue;
        }
        
        // Get mounting information
        const auto& mounting = m_engineMountings.at(engineId);
        
        // Nominal thrust direction (normalized)
        Vector3D thrustDir = mounting.orientation.normalized();
        
        // Apply TVC if present
        if (mounting.tvc) {
            thrustDir = mounting.tvc->transformThrustVector(thrustDir);
        }
        
        // Add to total
        totalThrust += thrustDir * thrustMag;
    }
    
    return totalThrust;
}

Vector3D PropulsionSystem::getTotalTorque(double ambientPressure) const {
    Vector3D totalTorque = Vector3D::zero();
    
    for (const auto& [engineId, engine] : m_engines) {
        if (!engine->isActive()) {
            continue;
        }
        
        // Get thrust vector
        double thrustMag = engine->getThrust(ambientPressure);
        if (thrustMag <= 0.0) {
            continue;
        }
        
        const auto& mounting = m_engineMountings.at(engineId);
        Vector3D thrustDir = mounting.orientation.normalized();
        
        // Apply TVC if present
        if (mounting.tvc) {
            thrustDir = mounting.tvc->transformThrustVector(thrustDir);
        }
        
        Vector3D thrustVector = thrustDir * thrustMag;
        
        // Calculate moment arm from CoM to engine position
        Vector3D momentArm = mounting.position - m_state.centerOfMass;
        
        // Torque = r × F
        totalTorque += momentArm.cross(thrustVector);
    }
    
    return totalTorque;
}

bool PropulsionSystem::canOperate() const {
    // Check if any engine has fuel available
    for (const auto& [engineId, engine] : m_engines) {
        const auto& mounting = m_engineMountings.at(engineId);
        
        // Check assigned tanks
        for (const auto& tankId : mounting.assignedTanks) {
            auto tank = m_tanks.at(tankId);
            if (!tank->isEmpty()) {
                return true;
            }
        }
        
        // If no specific tanks assigned, check all tanks
        if (mounting.assignedTanks.empty()) {
            for (const auto& [tankId, tank] : m_tanks) {
                if (!tank->isEmpty()) {
                    return true;
                }
            }
        }
    }
    
    return false;
}

void PropulsionSystem::emergencyShutdown() {
    Logger::getInstance().logf(LogLevel::Warning, LogCategory::Physics,
        "Emergency shutdown initiated");
    
    for (auto& [engineId, engine] : m_engines) {
        if (engine->isActive()) {
            engine->shutdown();
        }
    }
    
    m_state.totalThrust = 0.0;
    m_state.totalMassFlowRate = 0.0;
    m_state.activeEngineCount = 0;
}

Matrix3D PropulsionSystem::getInverseInertiaTensor() const {
    if (!m_inverseInertiaTensorValid) {
        m_inverseInertiaTensor = m_state.inertiaTensor.inverse();
        m_inverseInertiaTensorValid = true;
    }
    return m_inverseInertiaTensor;
}

void PropulsionSystem::updateTime(double time) {
    // This is called by the dynamics engine before querying mass properties
    // We don't need to do anything here as we update in our own update() method
    m_currentTime = time;
}

bool PropulsionSystem::validate() const {
    try {
        validateConfiguration();
        
        // Validate all engines
        for (const auto& [id, engine] : m_engines) {
            if (!engine->validate()) {
                return false;
            }
        }
        
        // Validate all tanks
        for (const auto& [id, tank] : m_tanks) {
            if (tank->getCurrentMass() < 0.0) {
                return false;
            }
        }
        
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

std::string PropulsionSystem::toString() const {
    std::ostringstream oss;
    oss << "PropulsionSystem: "
        << m_engines.size() << " engines (" 
        << m_state.activeEngineCount << " active), "
        << m_tanks.size() << " tanks, "
        << std::fixed << std::setprecision(1)
        << "Mass=" << m_state.totalMass << " kg ("
        << m_state.totalPropellantMass << " kg fuel), "
        << "Thrust=" << m_state.totalThrust / 1000.0 << " kN";
    
    if (m_state.totalMassFlowRate > 0.0) {
        oss << ", Flow=" << std::setprecision(2) 
            << m_state.totalMassFlowRate << " kg/s";
        
        double burnTime = m_state.totalPropellantMass / m_state.totalMassFlowRate;
        oss << " (" << std::setprecision(0) << burnTime << "s remaining)";
    }
    
    return oss.str();
}

std::unique_ptr<PropulsionSystem> PropulsionSystem::clone() const {
    auto clone = std::make_unique<PropulsionSystem>(m_config);
    
    // Clone all engines and tanks
    for (const auto& [id, engine] : m_engines) {
        auto engineClone = engine->clone();
        const auto& mounting = m_engineMountings.at(id);
        clone->m_engines[id] = std::move(engineClone);
        clone->m_engineMountings[id] = mounting;
    }
    
    for (const auto& [id, tank] : m_tanks) {
        clone->m_tanks[id] = tank->clone();
    }
    
    // Copy state
    clone->m_state = m_state;
    clone->m_currentTime = m_currentTime;
    
    return clone;
}

double PropulsionSystem::getEffectiveIsp() const {
    if (m_state.totalThrust <= 0.0 || m_state.totalMassFlowRate <= 0.0) {
        return 0.0;
    }
    
    // Isp = F / (ṁ * g₀)
    return m_state.totalThrust / (m_state.totalMassFlowRate * STANDARD_GRAVITY);
}

double PropulsionSystem::getRemainingDeltaV() const {
    if (m_state.totalMass <= m_config.dryMass || getEffectiveIsp() <= 0.0) {
        return 0.0;
    }
    
    // Tsiolkovsky rocket equation: Δv = Isp * g₀ * ln(m₀/mf)
    double massRatio = m_state.totalMass / m_config.dryMass;
    return getEffectiveIsp() * STANDARD_GRAVITY * std::log(massRatio);
}

void PropulsionSystem::updateMassProperties() {
    // Calculate total propellant mass
    m_state.totalPropellantMass = 0.0;
    for (const auto& [id, tank] : m_tanks) {
        m_state.totalPropellantMass += tank->getCurrentMass();
    }
    
    // Update total mass
    m_state.totalMass = m_config.dryMass + m_state.totalPropellantMass;
    
    // Update center of mass
    m_state.centerOfMass = calculateCenterOfMass();
    
    // Update inertia tensor
    m_state.inertiaTensor = calculateInertiaTensor();
    m_inverseInertiaTensorValid = false;
}

Vector3D PropulsionSystem::calculateCenterOfMass() const {
    // Weighted average of all mass components
    Vector3D totalMoment = m_config.dryCenterOfMass * m_config.dryMass;
    
    // Add contribution from each tank
    // For simplicity, assume propellant is at tank location
    // In reality, this would require tank geometry and fill level
    for (const auto& [id, tank] : m_tanks) {
        double tankMass = tank->getCurrentMass();
        if (tankMass > 0.0) {
            // TODO: Add tank position to configuration
            // For now, assume propellant doesn't shift CoM significantly
            // This is reasonable for well-balanced tank layouts
        }
    }
    
    return totalMoment / m_state.totalMass;
}

Matrix3D PropulsionSystem::calculateInertiaTensor() const {
    // Start with dry inertia about dry CoM
    Matrix3D totalInertia = m_config.dryInertiaTensor;
    
    // Apply parallel axis theorem if CoM has shifted
    Vector3D comShift = m_state.centerOfMass - m_config.dryCenterOfMass;
    if (comShift.magnitude() > 1e-6) {
        totalInertia = applyParallelAxis(totalInertia, m_config.dryMass, -comShift);
    }
    
    // Add contribution from propellant
    // For simplicity, assume propellant adds to diagonal terms
    // In reality, this would require tank geometry and fill level
    double propellantInertiaFactor = m_state.totalPropellantMass * 0.1;  // Rough estimate
    totalInertia(0, 0) += propellantInertiaFactor;
    totalInertia(1, 1) += propellantInertiaFactor;
    totalInertia(2, 2) += propellantInertiaFactor;
    
    return totalInertia;
}

double PropulsionSystem::distributePropellantConsumption(const Engine& engine,
                                                       double requiredMass,
                                                       double deltaTime) {
    const auto& mounting = m_engineMountings.at(engine.getName());
    std::vector<std::string> availableTanks;
    
    // Get list of available tanks
    if (!mounting.assignedTanks.empty()) {
        // Use only assigned tanks
        for (const auto& tankId : mounting.assignedTanks) {
            auto tank = m_tanks.at(tankId);
            if (!tank->isEmpty()) {
                availableTanks.push_back(tankId);
            }
        }
    } else if (m_config.autoDistributeFuel) {
        // Use all non-empty tanks
        for (const auto& [tankId, tank] : m_tanks) {
            if (!tank->isEmpty()) {
                availableTanks.push_back(tankId);
            }
        }
    }
    
    if (availableTanks.empty()) {
        return 0.0;
    }
    
    // Distribute consumption evenly among available tanks
    double massPerTank = requiredMass / availableTanks.size();
    double totalConsumed = 0.0;
    
    for (const auto& tankId : availableTanks) {
        auto tank = m_tanks.at(tankId);
        double consumed = tank->consumeMass(massPerTank);
        totalConsumed += consumed;
    }
    
    return totalConsumed;
}

Matrix3D PropulsionSystem::applyParallelAxis(const Matrix3D& inertia,
                                            double mass,
                                            const Vector3D& offset) {
    // I_new = I_cm + m * (|r|² * I - r ⊗ r)
    double r2 = offset.dot(offset);
    Matrix3D result = inertia;
    
    result(0, 0) += mass * (r2 - offset.x() * offset.x());
    result(0, 1) -= mass * offset.x() * offset.y();
    result(0, 2) -= mass * offset.x() * offset.z();
    
    result(1, 0) -= mass * offset.y() * offset.x();
    result(1, 1) += mass * (r2 - offset.y() * offset.y());
    result(1, 2) -= mass * offset.y() * offset.z();
    
    result(2, 0) -= mass * offset.z() * offset.x();
    result(2, 1) -= mass * offset.z() * offset.y();
    result(2, 2) += mass * (r2 - offset.z() * offset.z());
    
    return result;
}

} // namespace propulsion
} // namespace physics
} // namespace iloss