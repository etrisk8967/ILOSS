#pragma once

#include "core/math/Vector3D.h"
#include "physics/state/StateVector.h"
#include "core/time/Time.h"
#include <string>
#include <memory>
#include <unordered_map>
#include <any>

namespace iloss {
namespace physics {
namespace forces {

/**
 * @brief Enumeration of available force model types
 */
enum class ForceModelType {
    TwoBody,           ///< Central gravitational force (two-body dynamics)
    GravityField,      ///< Earth gravity field (spherical harmonics)
    ThirdBody,         ///< Third-body perturbations (Sun, Moon, planets)
    Drag,              ///< Atmospheric drag
    SolarRadiation,    ///< Solar radiation pressure
    EarthRadiation,    ///< Earth radiation pressure (albedo + IR)
    Relativistic,      ///< Relativistic corrections
    Thrust,            ///< Propulsive forces
    UserDefined        ///< Custom user-defined force model
};

/**
 * @brief Configuration parameters for force models
 * 
 * This class uses std::any to allow flexible configuration storage
 * for different force model types.
 */
class ForceModelConfig {
public:
    /**
     * @brief Set a configuration parameter
     * @param key Parameter name
     * @param value Parameter value (any type)
     */
    template<typename T>
    void setParameter(const std::string& key, T value) {
        m_parameters[key] = value;
    }

    /**
     * @brief Get a configuration parameter
     * @param key Parameter name
     * @param defaultValue Default value if parameter not found
     * @return Parameter value
     */
    template<typename T>
    T getParameter(const std::string& key, const T& defaultValue = T{}) const {
        auto it = m_parameters.find(key);
        if (it != m_parameters.end()) {
            try {
                return std::any_cast<T>(it->second);
            } catch (const std::bad_any_cast&) {
                return defaultValue;
            }
        }
        return defaultValue;
    }

    /**
     * @brief Check if a parameter exists
     * @param key Parameter name
     * @return True if parameter exists
     */
    bool hasParameter(const std::string& key) const {
        return m_parameters.find(key) != m_parameters.end();
    }

    /**
     * @brief Clear all parameters
     */
    void clear() {
        m_parameters.clear();
    }

private:
    std::unordered_map<std::string, std::any> m_parameters;
};

/**
 * @brief Abstract base class for all force models
 * 
 * This class defines the interface that all force models must implement.
 * Force models calculate accelerations acting on a spacecraft based on
 * various physical phenomena.
 */
class ForceModel {
public:
    /**
     * @brief Constructor
     * @param name Name of the force model
     * @param type Type of the force model
     */
    ForceModel(const std::string& name, ForceModelType type)
        : m_name(name), m_type(type), m_enabled(true) {}

    /**
     * @brief Virtual destructor
     */
    virtual ~ForceModel() = default;

    /**
     * @brief Calculate the acceleration due to this force
     * @param state Current state vector of the object
     * @param time Current time
     * @return Acceleration vector in m/s²
     */
    virtual math::Vector3D calculateAcceleration(
        const StateVector& state,
        const time::Time& time) const = 0;

    /**
     * @brief Initialize the force model with configuration
     * @param config Configuration parameters
     * @return True if initialization successful
     */
    virtual bool initialize(const ForceModelConfig& config) = 0;

    /**
     * @brief Update any time-dependent parameters
     * @param time Current time
     * 
     * This method is called before force calculation to allow
     * models to update time-dependent parameters (e.g., solar position)
     */
    virtual void update(const time::Time& /*time*/) {}

    /**
     * @brief Validate the force model configuration
     * @return True if configuration is valid
     */
    virtual bool validate() const = 0;

    /**
     * @brief Get the name of this force model
     * @return Model name
     */
    const std::string& getName() const { return m_name; }

    /**
     * @brief Get the type of this force model
     * @return Model type
     */
    ForceModelType getType() const { return m_type; }

    /**
     * @brief Check if the force model is enabled
     * @return True if enabled
     */
    bool isEnabled() const { return m_enabled; }

    /**
     * @brief Enable or disable the force model
     * @param enabled True to enable, false to disable
     */
    void setEnabled(bool enabled) { m_enabled = enabled; }

    /**
     * @brief Get a string representation of the force model
     * @return String description
     */
    virtual std::string toString() const {
        return m_name + " (" + forceModelTypeToString(m_type) + ")";
    }

    /**
     * @brief Convert force model type to string
     * @param type Force model type
     * @return String representation
     */
    static std::string forceModelTypeToString(ForceModelType type) {
        switch (type) {
            case ForceModelType::TwoBody: return "TwoBody";
            case ForceModelType::GravityField: return "GravityField";
            case ForceModelType::ThirdBody: return "ThirdBody";
            case ForceModelType::Drag: return "Drag";
            case ForceModelType::SolarRadiation: return "SolarRadiation";
            case ForceModelType::EarthRadiation: return "EarthRadiation";
            case ForceModelType::Relativistic: return "Relativistic";
            case ForceModelType::Thrust: return "Thrust";
            case ForceModelType::UserDefined: return "UserDefined";
            default: return "Unknown";
        }
    }

    /**
     * @brief Clone the force model
     * @return Unique pointer to cloned model
     * 
     * This is used for creating copies of force models,
     * particularly useful for parallel processing or checkpointing.
     */
    virtual std::unique_ptr<ForceModel> clone() const = 0;

protected:
    std::string m_name;      ///< Name of the force model
    ForceModelType m_type;   ///< Type of the force model
    bool m_enabled;          ///< Whether the model is enabled

    /**
     * @brief Current configuration
     * 
     * Derived classes can store their configuration here
     * for easy access and cloning.
     */
    ForceModelConfig m_config;
};

} // namespace forces
} // namespace physics
} // namespace iloss