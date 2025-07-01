#include "physics/forces/propulsion/PropellantTank.h"
#include "core/logging/Logger.h"
#include <cmath>
#include <sstream>
#include <iomanip>
#include <limits>

namespace iloss {
namespace physics {
namespace propulsion {

using namespace iloss::logging;

PropellantTank::PropellantTank(const std::string& name,
                             PropellantType propellantType,
                             double maxCapacity,
                             double currentMass)
    : m_name(name)
    , m_propellantType(propellantType)
    , m_customPropellantName("")
    , m_maxCapacity(maxCapacity)
    , m_currentMass(currentMass) {
    
    validateParameters();
    
    Logger::getInstance().logf(LogLevel::Info, LogCategory::Physics,
        "Created tank '{}' with {} propellant: {:.1f}/{:.1f} kg",
        m_name, propellantTypeToString(m_propellantType), 
        m_currentMass, m_maxCapacity);
}

PropellantTank::PropellantTank(const std::string& name,
                             const std::string& customPropellantName,
                             double maxCapacity,
                             double currentMass)
    : m_name(name)
    , m_propellantType(PropellantType::CUSTOM)
    , m_customPropellantName(customPropellantName)
    , m_maxCapacity(maxCapacity)
    , m_currentMass(currentMass) {
    
    if (customPropellantName.empty()) {
        throw std::invalid_argument("Custom propellant name cannot be empty");
    }
    
    validateParameters();
    
    Logger::getInstance().logf(LogLevel::Info, LogCategory::Physics,
        "Created tank '{}' with custom propellant '{}': {:.1f}/{:.1f} kg",
        m_name, m_customPropellantName, m_currentMass, m_maxCapacity);
}

void PropellantTank::validateParameters() const {
    if (m_name.empty()) {
        throw std::invalid_argument("Tank name cannot be empty");
    }
    
    if (m_maxCapacity <= 0.0) {
        throw std::invalid_argument("Tank capacity must be positive: " + 
            std::to_string(m_maxCapacity));
    }
    
    if (m_currentMass < 0.0) {
        throw std::invalid_argument("Current mass cannot be negative: " + 
            std::to_string(m_currentMass));
    }
    
    if (m_currentMass > m_maxCapacity) {
        throw std::invalid_argument("Current mass exceeds tank capacity: " + 
            std::to_string(m_currentMass) + " > " + std::to_string(m_maxCapacity));
    }
}

std::string PropellantTank::getPropellantTypeString() const {
    if (m_propellantType == PropellantType::CUSTOM) {
        return m_customPropellantName;
    }
    return propellantTypeToString(m_propellantType);
}

double PropellantTank::getAvailableMass() const {
    // Consider tank empty if below minimum fraction to avoid numerical issues
    double minMass = m_maxCapacity * MIN_PROPELLANT_FRACTION;
    if (m_currentMass <= minMass) {
        return 0.0;
    }
    return m_currentMass;
}

double PropellantTank::consumeMass(double mass) {
    if (mass < 0.0) {
        throw std::invalid_argument("Cannot consume negative mass: " + 
            std::to_string(mass));
    }
    
    if (mass == 0.0) {
        return 0.0;
    }
    
    double availableMass = getAvailableMass();
    double actualConsumption = std::min(mass, availableMass);
    m_currentMass -= actualConsumption;
    
    // Clamp to zero to handle floating-point errors
    if (m_currentMass < 0.0) {
        m_currentMass = 0.0;
    }
    
    if (actualConsumption < mass) {
        Logger::getInstance().logf(LogLevel::Warning, LogCategory::Physics,
            "Tank '{}' could only provide {:.3f} kg of requested {:.3f} kg",
            m_name, actualConsumption, mass);
    }
    
    return actualConsumption;
}

double PropellantTank::addMass(double mass) {
    if (mass < 0.0) {
        throw std::invalid_argument("Cannot add negative mass: " + 
            std::to_string(mass));
    }
    
    if (mass == 0.0) {
        return 0.0;
    }
    
    double availableSpace = m_maxCapacity - m_currentMass;
    double actualAddition = std::min(mass, availableSpace);
    m_currentMass += actualAddition;
    
    if (actualAddition < mass) {
        Logger::getInstance().logf(LogLevel::Warning, LogCategory::Physics,
            "Tank '{}' could only accept {:.3f} kg of {:.3f} kg",
            m_name, actualAddition, mass);
    }
    
    return actualAddition;
}

void PropellantTank::setCurrentMass(double mass) {
    if (mass < 0.0) {
        throw std::invalid_argument("Mass cannot be negative: " + 
            std::to_string(mass));
    }
    
    if (mass > m_maxCapacity) {
        throw std::invalid_argument("Mass exceeds tank capacity: " + 
            std::to_string(mass) + " > " + std::to_string(m_maxCapacity));
    }
    
    m_currentMass = mass;
}

bool PropellantTank::canSustainFlowRate(double massFlowRate, double duration) const {
    if (massFlowRate < 0.0 || duration < 0.0) {
        return false;
    }
    
    if (massFlowRate == 0.0 || duration == 0.0) {
        return true;
    }
    
    double requiredMass = massFlowRate * duration;
    return getAvailableMass() >= requiredMass;
}

double PropellantTank::getMaxBurnDuration(double massFlowRate) const {
    if (massFlowRate < 0.0) {
        throw std::invalid_argument("Mass flow rate cannot be negative: " + 
            std::to_string(massFlowRate));
    }
    
    if (massFlowRate == 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    
    return getAvailableMass() / massFlowRate;
}

std::string PropellantTank::toString() const {
    std::ostringstream oss;
    oss << "PropellantTank[" << m_name << "]: "
        << getPropellantTypeString() << " "
        << std::fixed << std::setprecision(1)
        << m_currentMass << "/" << m_maxCapacity << " kg "
        << "(" << std::setprecision(1) << (getFillFraction() * 100.0) << "%)";
    
    if (isEmpty()) {
        oss << " [EMPTY]";
    } else if (isFull()) {
        oss << " [FULL]";
    }
    
    return oss.str();
}

std::unique_ptr<PropellantTank> PropellantTank::clone() const {
    if (m_propellantType == PropellantType::CUSTOM) {
        return std::make_unique<PropellantTank>(
            m_name, m_customPropellantName, m_maxCapacity, m_currentMass);
    } else {
        return std::make_unique<PropellantTank>(
            m_name, m_propellantType, m_maxCapacity, m_currentMass);
    }
}

} // namespace propulsion
} // namespace physics
} // namespace iloss