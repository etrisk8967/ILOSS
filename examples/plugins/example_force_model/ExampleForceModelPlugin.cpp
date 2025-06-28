#include "ExampleForceModelPlugin.h"
#include "core/plugins/PluginVersion.h"
#include <sstream>

namespace example {

ExampleForceModelPlugin::ExampleForceModelPlugin()
    : m_state(iloss::plugins::PluginState::UNLOADED)
    , m_accelerationX(0.0)
    , m_accelerationY(0.0)
    , m_accelerationZ(0.0) {
    m_state = iloss::plugins::PluginState::LOADED;
}

iloss::plugins::PluginMetadata ExampleForceModelPlugin::getMetadata() const {
    iloss::plugins::PluginMetadata metadata;
    metadata.name = "Example Force Model";
    metadata.description = "Example plugin demonstrating a simple constant acceleration force model";
    metadata.author = "ILOSS Development Team";
    metadata.version = "1.0.0";
    metadata.minApiVersion = "1.0.0";
    metadata.maxApiVersion = "1.0.0";
    // No dependencies for this example
    
    return metadata;
}

iloss::plugins::PluginCapability ExampleForceModelPlugin::getCapabilities() const {
    return iloss::plugins::PluginCapability::FORCE_MODEL;
}

bool ExampleForceModelPlugin::initialize() {
    if (m_state != iloss::plugins::PluginState::LOADED) {
        setError("Cannot initialize: plugin is not in LOADED state");
        return false;
    }
    
    // Perform initialization
    // In a real plugin, this might load configuration files,
    // allocate resources, etc.
    
    // Set default configuration
    m_configuration["acceleration_x"] = "0.0";
    m_configuration["acceleration_y"] = "0.0";
    m_configuration["acceleration_z"] = "0.0";
    
    m_state = iloss::plugins::PluginState::INITIALIZED;
    return true;
}

bool ExampleForceModelPlugin::activate() {
    if (m_state != iloss::plugins::PluginState::INITIALIZED) {
        setError("Cannot activate: plugin is not in INITIALIZED state");
        return false;
    }
    
    // Parse configuration values
    try {
        m_accelerationX = std::stod(m_configuration["acceleration_x"]);
        m_accelerationY = std::stod(m_configuration["acceleration_y"]);
        m_accelerationZ = std::stod(m_configuration["acceleration_z"]);
    } catch (const std::exception& e) {
        setError("Failed to parse configuration: " + std::string(e.what()));
        return false;
    }
    
    // In a real plugin, this would:
    // - Register the force model with the physics engine
    // - Subscribe to relevant events
    // - Start any background threads
    
    m_state = iloss::plugins::PluginState::ACTIVE;
    return true;
}

bool ExampleForceModelPlugin::deactivate() {
    if (m_state != iloss::plugins::PluginState::ACTIVE) {
        setError("Cannot deactivate: plugin is not in ACTIVE state");
        return false;
    }
    
    // In a real plugin, this would:
    // - Unregister the force model
    // - Unsubscribe from events
    // - Stop background threads
    
    m_state = iloss::plugins::PluginState::INITIALIZED;
    return true;
}

bool ExampleForceModelPlugin::shutdown() {
    if (m_state != iloss::plugins::PluginState::INITIALIZED) {
        setError("Cannot shutdown: plugin is not in INITIALIZED state");
        return false;
    }
    
    // Release resources
    m_configuration.clear();
    
    m_state = iloss::plugins::PluginState::LOADED;
    return true;
}

iloss::plugins::PluginState ExampleForceModelPlugin::getState() const {
    return m_state;
}

std::string ExampleForceModelPlugin::getLastError() const {
    return m_lastError;
}

bool ExampleForceModelPlugin::configure(const std::string& key, const std::string& value) {
    // Validate configuration keys
    if (key != "acceleration_x" && key != "acceleration_y" && key != "acceleration_z") {
        setError("Unknown configuration key: " + key);
        return false;
    }
    
    // Validate value is a number
    try {
        std::stod(value);
    } catch (const std::exception&) {
        setError("Configuration value must be a number");
        return false;
    }
    
    m_configuration[key] = value;
    
    // If active, update the values
    if (m_state == iloss::plugins::PluginState::ACTIVE) {
        try {
            if (key == "acceleration_x") {
                m_accelerationX = std::stod(value);
            } else if (key == "acceleration_y") {
                m_accelerationY = std::stod(value);
            } else if (key == "acceleration_z") {
                m_accelerationZ = std::stod(value);
            }
        } catch (const std::exception& e) {
            setError("Failed to update configuration: " + std::string(e.what()));
            return false;
        }
    }
    
    return true;
}

std::string ExampleForceModelPlugin::getConfiguration(const std::string& key) const {
    auto it = m_configuration.find(key);
    if (it != m_configuration.end()) {
        return it->second;
    }
    return "";
}

void ExampleForceModelPlugin::setError(const std::string& error) {
    m_lastError = error;
}

} // namespace example

// Export functions required by the plugin system
ILOSS_PLUGIN_EXPORT_FUNCTIONS(example::ExampleForceModelPlugin)