#pragma once

#include "physics/forces/ForceModel.h"
#include "core/logging/Logger.h"
#include <memory>
#include <functional>
#include <unordered_map>
#include <vector>
#include <string>
#include <mutex>

namespace iloss {
namespace physics {
namespace forces {

/**
 * @brief Factory function type for creating force models
 */
using ForceModelFactory = std::function<std::unique_ptr<ForceModel>(const std::string& name)>;

/**
 * @brief Information about a registered force model type
 */
struct ForceModelInfo {
    ForceModelType type;                    ///< Type of the force model
    std::string typeName;                   ///< String name of the type
    std::string description;                ///< Human-readable description
    ForceModelFactory factory;              ///< Factory function to create instances
    std::vector<std::string> requiredParams; ///< List of required configuration parameters
    std::vector<std::string> optionalParams; ///< List of optional configuration parameters
};

/**
 * @brief Registry for force model types
 * 
 * The ForceModelRegistry maintains a catalog of available force model types
 * and provides factory methods for creating instances. It follows the singleton
 * pattern and is thread-safe.
 */
class ForceModelRegistry {
public:
    /**
     * @brief Get the singleton instance
     * @return Reference to the registry instance
     */
    static ForceModelRegistry& getInstance();

    /**
     * @brief Register a force model type
     * @param type Type identifier
     * @param info Force model information including factory
     * @return True if successfully registered
     */
    bool registerForceModel(ForceModelType type, const ForceModelInfo& info);

    /**
     * @brief Register a force model type with simplified parameters
     * @param type Type identifier
     * @param typeName String name of the type
     * @param description Human-readable description
     * @param factory Factory function
     * @return True if successfully registered
     */
    bool registerForceModel(ForceModelType type,
                           const std::string& typeName,
                           const std::string& description,
                           ForceModelFactory factory);

    /**
     * @brief Unregister a force model type
     * @param type Type to unregister
     * @return True if successfully unregistered
     */
    bool unregisterForceModel(ForceModelType type);

    /**
     * @brief Create a force model instance
     * @param type Type of model to create
     * @param name Name for the new instance
     * @return Unique pointer to the created model or nullptr on failure
     */
    std::unique_ptr<ForceModel> createForceModel(ForceModelType type,
                                                 const std::string& name) const;

    /**
     * @brief Create a force model instance by type name
     * @param typeName String name of the type
     * @param name Name for the new instance
     * @return Unique pointer to the created model or nullptr on failure
     */
    std::unique_ptr<ForceModel> createForceModel(const std::string& typeName,
                                                 const std::string& name) const;

    /**
     * @brief Check if a force model type is registered
     * @param type Type to check
     * @return True if registered
     */
    bool isRegistered(ForceModelType type) const;

    /**
     * @brief Check if a force model type name is registered
     * @param typeName Type name to check
     * @return True if registered
     */
    bool isRegistered(const std::string& typeName) const;

    /**
     * @brief Get information about a force model type
     * @param type Type to query
     * @return Pointer to info or nullptr if not found
     */
    const ForceModelInfo* getForceModelInfo(ForceModelType type) const;

    /**
     * @brief Get information about a force model type by name
     * @param typeName Type name to query
     * @return Pointer to info or nullptr if not found
     */
    const ForceModelInfo* getForceModelInfo(const std::string& typeName) const;

    /**
     * @brief Get all registered force model types
     * @return Vector of registered types
     */
    std::vector<ForceModelType> getRegisteredTypes() const;

    /**
     * @brief Get all registered force model type names
     * @return Vector of type names
     */
    std::vector<std::string> getRegisteredTypeNames() const;

    /**
     * @brief Get a summary of all registered force models
     * @return String summary
     */
    std::string getSummary() const;

    /**
     * @brief Clear all registrations
     * 
     * This should only be used in testing or shutdown scenarios
     */
    void clear();

    /**
     * @brief Register standard force models
     * 
     * This method registers the built-in force models that come
     * with the system (e.g., two-body, drag, etc.)
     */
    void registerStandardModels();

private:
    /**
     * @brief Private constructor for singleton
     */
    ForceModelRegistry();

    /**
     * @brief Private destructor
     */
    ~ForceModelRegistry() = default;

    /**
     * @brief Delete copy constructor
     */
    ForceModelRegistry(const ForceModelRegistry&) = delete;

    /**
     * @brief Delete copy assignment
     */
    ForceModelRegistry& operator=(const ForceModelRegistry&) = delete;

    /**
     * @brief Registry of force model types
     */
    std::unordered_map<ForceModelType, ForceModelInfo> m_registry;

    /**
     * @brief Map from type names to types for string-based lookup
     */
    std::unordered_map<std::string, ForceModelType> m_typeNameMap;

    /**
     * @brief Mutex for thread safety
     */
    mutable std::mutex m_mutex;

    /**
     * @brief Logger instance
     */
    iloss::logging::Logger& m_logger = iloss::logging::Logger::getInstance();
};

/**
 * @brief Helper class for automatic force model registration
 * 
 * This class can be used to automatically register force models
 * when a translation unit is loaded.
 */
class ForceModelRegistrar {
public:
    /**
     * @brief Constructor that registers a force model
     * @param type Type identifier
     * @param info Force model information
     */
    ForceModelRegistrar(ForceModelType type, const ForceModelInfo& info) {
        ForceModelRegistry::getInstance().registerForceModel(type, info);
    }
};

/**
 * @brief Macro to simplify force model registration
 * 
 * Usage: REGISTER_FORCE_MODEL(ForceModelType::TwoBody, info);
 */
#define REGISTER_FORCE_MODEL(type, info) \
    static ForceModelRegistrar _registrar_##type(type, info)

} // namespace forces
} // namespace physics
} // namespace iloss