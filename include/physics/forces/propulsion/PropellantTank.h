#pragma once

#include "physics/forces/propulsion/PropulsionConstants.h"
#include <string>
#include <stdexcept>
#include <memory>

namespace iloss {
namespace physics {
namespace propulsion {

/**
 * @brief Represents a propellant tank with mass tracking capabilities
 * 
 * This class models a propellant tank that can supply fuel to one or more engines.
 * It tracks the current propellant mass and handles depletion during burns.
 * The tank enforces physical constraints such as non-negative mass and maximum capacity.
 * 
 * Thread-safety: This class is NOT thread-safe. External synchronization is required
 * for concurrent access.
 */
class PropellantTank {
public:
    /**
     * @brief Construct a new propellant tank
     * 
     * @param name Unique identifier for the tank
     * @param propellantType Type of propellant stored
     * @param maxCapacity Maximum propellant mass the tank can hold (kg)
     * @param currentMass Initial propellant mass (kg)
     * @throws std::invalid_argument if maxCapacity <= 0 or currentMass < 0 or currentMass > maxCapacity
     */
    PropellantTank(const std::string& name,
                   PropellantType propellantType,
                   double maxCapacity,
                   double currentMass);
    
    /**
     * @brief Construct a tank with custom propellant type
     * 
     * @param name Unique identifier for the tank
     * @param customPropellantName Name of custom propellant
     * @param maxCapacity Maximum propellant mass the tank can hold (kg)
     * @param currentMass Initial propellant mass (kg)
     * @throws std::invalid_argument if maxCapacity <= 0 or currentMass < 0 or currentMass > maxCapacity
     */
    PropellantTank(const std::string& name,
                   const std::string& customPropellantName,
                   double maxCapacity,
                   double currentMass);
    
    /**
     * @brief Default destructor
     */
    ~PropellantTank() = default;
    
    /**
     * @brief Get the tank name
     * @return Tank identifier
     */
    const std::string& getName() const { return m_name; }
    
    /**
     * @brief Get the propellant type
     * @return Propellant type enum
     */
    PropellantType getPropellantType() const { return m_propellantType; }
    
    /**
     * @brief Get the propellant type as string
     * @return Human-readable propellant type
     */
    std::string getPropellantTypeString() const;
    
    /**
     * @brief Get current propellant mass
     * @return Current mass in kg
     */
    double getCurrentMass() const { return m_currentMass; }
    
    /**
     * @brief Get maximum tank capacity
     * @return Maximum capacity in kg
     */
    double getMaxCapacity() const { return m_maxCapacity; }
    
    /**
     * @brief Get available propellant mass
     * 
     * This accounts for unusable propellant residuals. When the mass fraction
     * falls below MIN_PROPELLANT_FRACTION, the tank is considered empty.
     * 
     * @return Available mass in kg
     */
    double getAvailableMass() const;
    
    /**
     * @brief Get propellant fill fraction
     * @return Current mass / max capacity (0.0 to 1.0)
     */
    double getFillFraction() const { return m_currentMass / m_maxCapacity; }
    
    /**
     * @brief Check if tank is empty
     * 
     * A tank is considered empty when the available mass is zero, which occurs
     * when the mass fraction falls below MIN_PROPELLANT_FRACTION.
     * 
     * @return True if no usable propellant remains
     */
    bool isEmpty() const { return getAvailableMass() <= 0.0; }
    
    /**
     * @brief Check if tank is full
     * @return True if current mass equals max capacity
     */
    bool isFull() const { return m_currentMass >= m_maxCapacity; }
    
    /**
     * @brief Consume propellant from the tank
     * 
     * @param mass Mass to consume (kg)
     * @return Actual mass consumed (may be less if tank runs empty)
     * @throws std::invalid_argument if mass < 0
     */
    double consumeMass(double mass);
    
    /**
     * @brief Add propellant to the tank (for refueling)
     * 
     * @param mass Mass to add (kg)
     * @return Actual mass added (may be less if tank fills up)
     * @throws std::invalid_argument if mass < 0
     */
    double addMass(double mass);
    
    /**
     * @brief Set current propellant mass (for initialization/reset)
     * 
     * @param mass New current mass (kg)
     * @throws std::invalid_argument if mass < 0 or mass > maxCapacity
     */
    void setCurrentMass(double mass);
    
    /**
     * @brief Reset tank to full capacity
     */
    void refill() { m_currentMass = m_maxCapacity; }
    
    /**
     * @brief Empty the tank completely
     */
    void empty() { m_currentMass = 0.0; }
    
    /**
     * @brief Get a string representation of the tank state
     * @return Human-readable tank status
     */
    std::string toString() const;
    
    /**
     * @brief Create a copy of this tank
     * @return Unique pointer to cloned tank
     */
    std::unique_ptr<PropellantTank> clone() const;
    
    /**
     * @brief Check if a given mass flow rate can be sustained
     * 
     * @param massFlowRate Desired flow rate (kg/s)
     * @param duration Time duration (s)
     * @return True if the tank has enough propellant
     */
    bool canSustainFlowRate(double massFlowRate, double duration) const;
    
    /**
     * @brief Calculate how long a flow rate can be sustained
     * 
     * @param massFlowRate Flow rate (kg/s)
     * @return Maximum duration in seconds (may be infinite for zero flow rate)
     * @throws std::invalid_argument if massFlowRate < 0
     */
    double getMaxBurnDuration(double massFlowRate) const;

private:
    std::string m_name;                    ///< Tank identifier
    PropellantType m_propellantType;      ///< Type of propellant
    std::string m_customPropellantName;   ///< Name for custom propellant type
    double m_maxCapacity;                  ///< Maximum capacity (kg)
    double m_currentMass;                  ///< Current propellant mass (kg)
    
    /**
     * @brief Validate tank parameters
     * @throws std::invalid_argument if parameters are invalid
     */
    void validateParameters() const;
};

} // namespace propulsion
} // namespace physics
} // namespace iloss