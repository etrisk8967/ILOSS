#pragma once

#include "core/plugins/IPlugin.h"
#include <map>

namespace example {

/**
 * @brief Example force model plugin
 * 
 * This is a template showing how to create a plugin for ILOSS.
 * This example implements a simple constant acceleration force model.
 */
class ExampleForceModelPlugin : public iloss::plugins::IPlugin {
public:
    ExampleForceModelPlugin();
    ~ExampleForceModelPlugin() override = default;

    // IPlugin interface implementation
    iloss::plugins::PluginMetadata getMetadata() const override;
    iloss::plugins::PluginCapability getCapabilities() const override;
    bool initialize() override;
    bool activate() override;
    bool deactivate() override;
    bool shutdown() override;
    iloss::plugins::PluginState getState() const override;
    std::string getLastError() const override;
    bool configure(const std::string& key, const std::string& value) override;
    std::string getConfiguration(const std::string& key) const override;

private:
    iloss::plugins::PluginState m_state;
    std::string m_lastError;
    std::map<std::string, std::string> m_configuration;
    
    // Plugin-specific data
    double m_accelerationX;
    double m_accelerationY;
    double m_accelerationZ;
    
    void setError(const std::string& error);
};

} // namespace example