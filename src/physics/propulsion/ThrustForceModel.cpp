#include "physics/forces/propulsion/ThrustForceModel.h"
#include "core/logging/Logger.h"
#include "core/constants/EarthModel.h"
#include <random>
#include <sstream>
#include <iomanip>

namespace iloss {
namespace physics {
namespace propulsion {

using namespace iloss::logging;
using namespace iloss::math;
using namespace iloss::time;
using namespace iloss::physics::forces;
using namespace iloss::constants;

ThrustForceModel::ThrustForceModel(const std::string& name,
                                 std::shared_ptr<PropulsionSystem> propulsionSystem,
                                 std::shared_ptr<AtmosphericModel> atmosphereModel,
                                 const Configuration& config)
    : IAttitudeAwareForceModel(name, ForceModelType::Thrust)
    , m_propulsionSystem(propulsionSystem)
    , m_atmosphereModel(atmosphereModel)
    , m_configuration(config)
    , m_lastUpdateTime(0.0, TimeSystem::UTC) {
    
    if (!m_propulsionSystem) {
        throw std::invalid_argument("Propulsion system cannot be null");
    }
    
    Logger::getInstance().logf(LogLevel::Info, LogCategory::Physics,
        "Created thrust force model '{}' with {} engines and {} tanks",
        name, 
        m_propulsionSystem->getEngines().size(),
        m_propulsionSystem->getTanks().size());
}

Vector3D ThrustForceModel::calculateAccelerationWithAttitude(
    const dynamics::DynamicsState& state,
    const Time& time) const {
    
    if (!isEnabled()) {
        return Vector3D::zero();
    }
    
    // Calculate time step since last update
    double deltaTime = 0.0;
    if (m_lastUpdateTime.getTime() > 0.0) {
        deltaTime = time.getTime() - m_lastUpdateTime.getTime();
    }
    m_lastUpdateTime = time;
    
    // Get atmospheric pressure if model available
    double atmosphericPressure = 0.0;
    if (m_configuration.enableAtmosphericEffects && m_atmosphereModel) {
        atmosphericPressure = getAtmosphericPressure(state.getPosition(), time);
    }
    m_lastAtmosphericPressure = atmosphericPressure;
    
    // Update propulsion system
    if (deltaTime > 0.0) {
        updatePropulsionSystem(deltaTime, atmosphericPressure);
    }
    
    // Get thrust vector in body frame
    Vector3D thrustBody = m_propulsionSystem->getTotalThrustVector(atmosphericPressure);
    
    // Apply efficiency and uncertainty
    thrustBody *= m_configuration.thrustEfficiency;
    if (m_configuration.thrustUncertainty > 0.0) {
        thrustBody = applyThrustUncertainty(thrustBody);
    }
    
    // Store for telemetry
    m_lastTotalThrust = thrustBody.magnitude();
    m_lastMassFlowRate = m_propulsionSystem->getSystemState().totalMassFlowRate;
    
    // Log significant events
    if (m_configuration.logThrustEvents) {
        logThrustEvent(m_lastTotalThrust, m_lastMassFlowRate);
    }
    
    // Transform thrust from body to inertial frame
    Vector3D thrustInertial = state.getAttitude().rotate(thrustBody);
    
    // Calculate acceleration (F = ma, so a = F/m)
    double currentMass = state.getMass();
    if (currentMass <= 0.0) {
        Logger::getInstance().logf(LogLevel::Error, LogCategory::Physics,
            "Invalid mass {:.3f} kg", currentMass);
        return Vector3D::zero();
    }
    
    // Return acceleration
    return thrustInertial / currentMass;
}

bool ThrustForceModel::initialize(const ForceModelConfig& config) {
    m_config = config;
    
    // Extract configuration parameters
    m_configuration.enableAtmosphericEffects = 
        config.getParameter<bool>("enable_atmospheric_effects", true);
    m_configuration.enableMassUpdate = 
        config.getParameter<bool>("enable_mass_update", true);
    m_configuration.thrustEfficiency = 
        config.getParameter<double>("thrust_efficiency", 1.0);
    m_configuration.thrustUncertainty = 
        config.getParameter<double>("thrust_uncertainty", 0.0);
    m_configuration.logThrustEvents = 
        config.getParameter<bool>("log_thrust_events", true);
    
    // Handle engine throttle settings
    if (config.hasParameter("throttle")) {
        double globalThrottle = config.getParameter<double>("throttle", 1.0);
        // Apply to all engines
        for (const auto& [engineId, engine] : m_propulsionSystem->getEngines()) {
            engine->setThrottle(globalThrottle);
        }
    }
    
    // Handle engine-specific settings
    if (config.hasParameter("engine_settings")) {
        // TODO: Parse engine-specific throttle and gimbal settings
    }
    
    // Handle gimbal settings
    double gimbalPitch = config.getParameter<double>("gimbal_pitch", 0.0);
    double gimbalYaw = config.getParameter<double>("gimbal_yaw", 0.0);
    
    if (gimbalPitch != 0.0 || gimbalYaw != 0.0) {
        // Apply to all engines with TVC
        for (const auto& [engineId, engine] : m_propulsionSystem->getEngines()) {
            auto mounting = m_propulsionSystem->getEngineMounting(engineId);
            if (mounting && mounting->tvc) {
                mounting->tvc->setCommandedAngles(gimbalPitch, gimbalYaw);
            }
        }
    }
    
    Logger::getInstance().logf(LogLevel::Info, LogCategory::Physics,
        "Initialized with efficiency={:.2f}, uncertainty={:.1f}%",
        m_configuration.thrustEfficiency,
        m_configuration.thrustUncertainty * 100.0);
    
    return true;
}

void ThrustForceModel::update(const Time& time) {
    // Update simulation time
    m_simulationTime = time.getTime();
    
    // The actual propulsion system update happens in calculateAcceleration
    // when we know the current state and can calculate atmospheric pressure
}

bool ThrustForceModel::validate() const {
    if (!m_propulsionSystem) {
        return false;
    }
    
    if (!m_propulsionSystem->validate()) {
        return false;
    }
    
    if (m_configuration.thrustEfficiency <= 0.0 || 
        m_configuration.thrustEfficiency > 1.0) {
        return false;
    }
    
    if (m_configuration.thrustUncertainty < 0.0 || 
        m_configuration.thrustUncertainty > 0.1) {
        return false;
    }
    
    return true;
}

std::unique_ptr<ForceModel> ThrustForceModel::clone() const {
    // Clone the propulsion system
    auto propulsionClone = m_propulsionSystem->clone();
    
    // Create new thrust force model
    auto clone = std::make_unique<ThrustForceModel>(
        getName(), std::move(propulsionClone), m_atmosphereModel, m_configuration);
    
    // Copy state
    clone->setEnabled(isEnabled());
    clone->m_config = m_config;
    
    return clone;
}

double ThrustForceModel::getAtmosphericPressure(const Vector3D& position,
                                               const Time& time) const {
    if (!m_atmosphereModel) {
        return 0.0;
    }
    
    // Convert position to geodetic for altitude
    double lat, lon, alt;
    EarthModel::ecefToGeodetic(position, lat, lon, alt);
    
    // Get atmospheric density
    double density = m_atmosphereModel->getDensity(position, time.getJulianDate());
    
    // Convert density to pressure using ideal gas law approximation
    // P = ρ * R * T, where R is specific gas constant for air
    // This is a simplification - real atmosphere models would provide pressure directly
    const double R_air = 287.05;  // J/(kg·K)
    double temperature = 288.15 - 0.0065 * alt;  // Simple temperature model
    
    return density * R_air * temperature;
}

bool ThrustForceModel::setEngineThrottle(const std::string& engineId, double throttle) {
    auto engine = m_propulsionSystem->getEngine(engineId);
    if (!engine) {
        Logger::getInstance().logf(LogLevel::Warning, LogCategory::Physics,
            "Engine '{}' not found", engineId);
        return false;
    }
    
    double actualThrottle = engine->setThrottle(throttle);
    return std::abs(actualThrottle - throttle) < 1e-6;
}

bool ThrustForceModel::setEngineGimbal(const std::string& engineId, 
                                      double pitch, double yaw) {
    auto engine = m_propulsionSystem->getEngine(engineId);
    if (!engine) {
        Logger::getInstance().logf(LogLevel::Warning, LogCategory::Physics,
            "Engine '{}' not found", engineId);
        return false;
    }
    
    // Get TVC from mounting info
    auto mounting = m_propulsionSystem->getEngineMounting(engineId);
    if (mounting && mounting->tvc) {
        return mounting->tvc->setCommandedAngles(pitch, yaw);
    } else {
        // Fall back to engine's internal gimbal state
        return engine->setGimbalAngles(pitch, yaw);
    }
}

bool ThrustForceModel::startEngine(const std::string& engineId, double throttle) {
    auto engine = m_propulsionSystem->getEngine(engineId);
    if (!engine) {
        Logger::getInstance().logf(LogLevel::Warning, LogCategory::Physics,
            "Engine '{}' not found", engineId);
        return false;
    }
    
    return engine->start(throttle);
}

void ThrustForceModel::shutdownEngine(const std::string& engineId) {
    auto engine = m_propulsionSystem->getEngine(engineId);
    if (!engine) {
        Logger::getInstance().logf(LogLevel::Warning, LogCategory::Physics,
            "Engine '{}' not found", engineId);
        return;
    }
    
    engine->shutdown();
}

void ThrustForceModel::shutdownAllEngines() {
    m_propulsionSystem->emergencyShutdown();
}

bool ThrustForceModel::isActive() const {
    return m_propulsionSystem->getSystemState().activeEngineCount > 0;
}

std::string ThrustForceModel::toString() const {
    std::ostringstream oss;
    oss << "ThrustForceModel[" << getName() << "]: ";
    
    if (!isEnabled()) {
        oss << "DISABLED";
    } else if (!isActive()) {
        oss << "INACTIVE";
    } else {
        oss << std::fixed << std::setprecision(1)
            << "F=" << m_lastTotalThrust / 1000.0 << " kN, "
            << "ṁ=" << std::setprecision(2) << m_lastMassFlowRate << " kg/s, "
            << "Isp=" << std::setprecision(0) << getEffectiveIsp() << " s";
        
        if (m_lastAtmosphericPressure > 0.0) {
            oss << ", P=" << std::setprecision(0) 
                << m_lastAtmosphericPressure << " Pa";
        }
    }
    
    return oss.str();
}

void ThrustForceModel::updatePropulsionSystem(double deltaTime, double pressure) const {
    // Cast away const for mutable operation
    // This is safe because we're only updating internal state
    const_cast<PropulsionSystem*>(m_propulsionSystem.get())->update(deltaTime, pressure);
}

Vector3D ThrustForceModel::applyThrustUncertainty(const Vector3D& thrust) const {
    if (m_configuration.thrustUncertainty <= 0.0) {
        return thrust;
    }
    
    // Apply random variation to thrust magnitude
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::normal_distribution<double> dist(1.0, m_configuration.thrustUncertainty);
    
    double factor = dist(gen);
    factor = std::max(0.0, factor);  // Ensure non-negative
    
    return thrust * factor;
}

void ThrustForceModel::logThrustEvent(double thrust, double flowRate) const {
    static double lastLoggedThrust = 0.0;
    static double lastLoggedFlow = 0.0;
    
    // Log if thrust changes significantly (>1% or on/off)
    bool significantChange = false;
    
    if ((thrust > 0.0 && lastLoggedThrust == 0.0) ||
        (thrust == 0.0 && lastLoggedThrust > 0.0)) {
        // Engine start/stop
        significantChange = true;
    } else if (thrust > 0.0 && lastLoggedThrust > 0.0) {
        // Check for significant throttle change
        double changeRatio = std::abs(thrust - lastLoggedThrust) / lastLoggedThrust;
        if (changeRatio > 0.01) {
            significantChange = true;
        }
    }
    
    if (significantChange) {
        if (thrust > 0.0) {
            Logger::getInstance().logf(LogLevel::Info, LogCategory::Physics,
                "Thrust event: F={:.1f} kN, ṁ={:.2f} kg/s, Isp={:.0f} s",
                thrust / 1000.0, flowRate, getEffectiveIsp());
        } else {
            Logger::getInstance().logf(LogLevel::Info, LogCategory::Physics,
                "Engines shutdown");
        }
        
        lastLoggedThrust = thrust;
        lastLoggedFlow = flowRate;
    }
}

} // namespace propulsion
} // namespace physics
} // namespace iloss