#include "physics/forces/ForceModelRegistry.h"
#include <sstream>
#include <algorithm>
#include <fmt/format.h>

namespace iloss {
namespace physics {
namespace forces {

ForceModelRegistry::ForceModelRegistry() {
    // Don't log during construction to avoid potential deadlock during static initialization
}

ForceModelRegistry& ForceModelRegistry::getInstance() {
    static ForceModelRegistry instance;
    return instance;
}

bool ForceModelRegistry::registerForceModel(ForceModelType type, 
                                           const ForceModelInfo& info) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    // Check if already registered
    if (m_registry.find(type) != m_registry.end()) {
        m_logger.warning(logging::LogCategory::Physics, 
            fmt::format("Force model type {} is already registered", 
            ForceModel::forceModelTypeToString(type)));
        return false;
    }
    
    // Validate the factory function
    if (!info.factory) {
        m_logger.error(logging::LogCategory::Physics, 
            fmt::format("Cannot register force model type {} without factory function", 
            ForceModel::forceModelTypeToString(type)));
        return false;
    }
    
    // Register the model
    m_registry[type] = info;
    m_typeNameMap[info.typeName] = type;
    
    m_logger.info(logging::LogCategory::Physics, 
        fmt::format("Registered force model type: {} ({})", 
        info.typeName, info.description));
    
    return true;
}

bool ForceModelRegistry::registerForceModel(ForceModelType type,
                                           const std::string& typeName,
                                           const std::string& description,
                                           ForceModelFactory factory) {
    ForceModelInfo info;
    info.type = type;
    info.typeName = typeName;
    info.description = description;
    info.factory = factory;
    
    return registerForceModel(type, info);
}

bool ForceModelRegistry::unregisterForceModel(ForceModelType type) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    auto it = m_registry.find(type);
    if (it == m_registry.end()) {
        m_logger.warning(logging::LogCategory::Physics, 
            fmt::format("Cannot unregister unknown force model type {}", 
            ForceModel::forceModelTypeToString(type)));
        return false;
    }
    
    // Remove from type name map
    m_typeNameMap.erase(it->second.typeName);
    
    // Remove from registry
    m_registry.erase(it);
    
    m_logger.info(logging::LogCategory::Physics, 
        fmt::format("Unregistered force model type {}", 
        ForceModel::forceModelTypeToString(type)));
    
    return true;
}

std::unique_ptr<ForceModel> ForceModelRegistry::createForceModel(
    ForceModelType type, const std::string& name) const {
    
    std::lock_guard<std::mutex> lock(m_mutex);
    
    auto it = m_registry.find(type);
    if (it == m_registry.end()) {
        m_logger.error(logging::LogCategory::Physics, 
            fmt::format("Cannot create unregistered force model type {}", 
            ForceModel::forceModelTypeToString(type)));
        return nullptr;
    }
    
    try {
        auto model = it->second.factory(name);
        if (model) {
            m_logger.debug(logging::LogCategory::Physics, 
                fmt::format("Created force model '{}' of type {}", 
                name, it->second.typeName));
        }
        return model;
    } catch (const std::exception& e) {
        m_logger.error(logging::LogCategory::Physics, 
            fmt::format("Error creating force model '{}' of type {}: {}", 
            name, it->second.typeName, e.what()));
        return nullptr;
    }
}

std::unique_ptr<ForceModel> ForceModelRegistry::createForceModel(
    const std::string& typeName, const std::string& name) const {
    
    // Find the type first while holding the lock
    ForceModelType type;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        auto it = m_typeNameMap.find(typeName);
        if (it == m_typeNameMap.end()) {
            m_logger.error(logging::LogCategory::Physics, 
                fmt::format("Cannot create force model with unknown type name '{}'", typeName));
            return nullptr;
        }
        type = it->second;
    }
    
    // Now call the type-based creation method without holding the lock
    return createForceModel(type, name);
}

bool ForceModelRegistry::isRegistered(ForceModelType type) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_registry.find(type) != m_registry.end();
}

bool ForceModelRegistry::isRegistered(const std::string& typeName) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_typeNameMap.find(typeName) != m_typeNameMap.end();
}

const ForceModelInfo* ForceModelRegistry::getForceModelInfo(ForceModelType type) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    auto it = m_registry.find(type);
    if (it != m_registry.end()) {
        return &it->second;
    }
    return nullptr;
}

const ForceModelInfo* ForceModelRegistry::getForceModelInfo(const std::string& typeName) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    auto typeIt = m_typeNameMap.find(typeName);
    if (typeIt == m_typeNameMap.end()) {
        return nullptr;
    }
    
    auto infoIt = m_registry.find(typeIt->second);
    if (infoIt != m_registry.end()) {
        return &infoIt->second;
    }
    return nullptr;
}

std::vector<ForceModelType> ForceModelRegistry::getRegisteredTypes() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    std::vector<ForceModelType> types;
    types.reserve(m_registry.size());
    
    for (const auto& pair : m_registry) {
        types.push_back(pair.first);
    }
    
    return types;
}

std::vector<std::string> ForceModelRegistry::getRegisteredTypeNames() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    std::vector<std::string> names;
    names.reserve(m_typeNameMap.size());
    
    for (const auto& pair : m_typeNameMap) {
        names.push_back(pair.first);
    }
    
    // Sort alphabetically
    std::sort(names.begin(), names.end());
    
    return names;
}

std::string ForceModelRegistry::getSummary() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    std::stringstream ss;
    ss << "Force Model Registry Summary:\n";
    ss << "Registered models: " << m_registry.size() << "\n\n";
    
    for (const auto& pair : m_registry) {
        const ForceModelInfo& info = pair.second;
        ss << "Type: " << info.typeName << "\n";
        ss << "  Description: " << info.description << "\n";
        
        if (!info.requiredParams.empty()) {
            ss << "  Required parameters: ";
            for (size_t i = 0; i < info.requiredParams.size(); ++i) {
                if (i > 0) ss << ", ";
                ss << info.requiredParams[i];
            }
            ss << "\n";
        }
        
        if (!info.optionalParams.empty()) {
            ss << "  Optional parameters: ";
            for (size_t i = 0; i < info.optionalParams.size(); ++i) {
                if (i > 0) ss << ", ";
                ss << info.optionalParams[i];
            }
            ss << "\n";
        }
        
        ss << "\n";
    }
    
    return ss.str();
}

void ForceModelRegistry::clear() {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    m_registry.clear();
    m_typeNameMap.clear();
    
    m_logger.info(logging::LogCategory::Physics, "Cleared all force model registrations");
}

void ForceModelRegistry::registerStandardModels() {
    m_logger.info(logging::LogCategory::Physics, "Registering standard force models");
    
    // Note: The actual implementations of these models will be added in subsequent tasks
    // For now, we're just setting up the registry structure
    
    // Two-body gravity will be implemented in Task 18
    // Earth gravity field will be implemented in Task 19
    // Third-body perturbations will be implemented in Task 20
    // Atmospheric drag will be implemented in Task 21
    // Solar radiation pressure will be implemented in Task 22
    
    // The registry is ready to accept these registrations when the models are implemented
}

} // namespace forces
} // namespace physics
} // namespace iloss