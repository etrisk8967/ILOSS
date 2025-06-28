#pragma once

#include "core/external/TestWrapper.h"
#include "core/plugins/IPlugin.h"
#include <functional>

namespace iloss {
namespace test {

/**
 * @brief Mock plugin for testing plugin system
 */
class MockPlugin : public plugin::IPlugin {
public:
    MockPlugin(const std::string& name = "MockPlugin",
               const std::string& version = "1.0.0")
        : m_metadata{name, version, "Mock plugin for testing", "Test Author"} {
        m_metadata.apiVersion = plugin::IPlugin::getAPIVersion();
    }
    
    ~MockPlugin() override = default;
    
    const plugin::PluginMetadata& getMetadata() const override {
        return m_metadata;
    }
    
    bool initialize(const nlohmann::json& config) override {
        m_initializeCount++;
        m_lastConfig = config;
        
        if (m_initializeCallback) {
            return m_initializeCallback(config);
        }
        
        return m_shouldInitializeSucceed;
    }
    
    void shutdown() override {
        m_shutdownCount++;
        
        if (m_shutdownCallback) {
            m_shutdownCallback();
        }
    }
    
    bool activate() override {
        m_activateCount++;
        m_isActive = m_shouldActivateSucceed;
        
        if (m_activateCallback) {
            return m_activateCallback();
        }
        
        return m_shouldActivateSucceed;
    }
    
    void deactivate() override {
        m_deactivateCount++;
        m_isActive = false;
        
        if (m_deactivateCallback) {
            m_deactivateCallback();
        }
    }
    
    bool isActive() const override {
        return m_isActive;
    }
    
    nlohmann::json getStatus() const override {
        return {
            {"name", m_metadata.name},
            {"active", m_isActive},
            {"initializeCount", m_initializeCount},
            {"activateCount", m_activateCount},
            {"customStatus", m_customStatus}
        };
    }
    
    // Test utilities
    void setShouldInitializeSucceed(bool succeed) { m_shouldInitializeSucceed = succeed; }
    void setShouldActivateSucceed(bool succeed) { m_shouldActivateSucceed = succeed; }
    void setCustomStatus(const nlohmann::json& status) { m_customStatus = status; }
    
    int getInitializeCount() const { return m_initializeCount; }
    int getShutdownCount() const { return m_shutdownCount; }
    int getActivateCount() const { return m_activateCount; }
    int getDeactivateCount() const { return m_deactivateCount; }
    const nlohmann::json& getLastConfig() const { return m_lastConfig; }
    
    // Set custom callbacks
    void setInitializeCallback(std::function<bool(const nlohmann::json&)> callback) {
        m_initializeCallback = callback;
    }
    
    void setShutdownCallback(std::function<void()> callback) {
        m_shutdownCallback = callback;
    }
    
    void setActivateCallback(std::function<bool()> callback) {
        m_activateCallback = callback;
    }
    
    void setDeactivateCallback(std::function<void()> callback) {
        m_deactivateCallback = callback;
    }
    
    // Modify metadata for testing
    plugin::PluginMetadata& getMutableMetadata() { return m_metadata; }
    
private:
    plugin::PluginMetadata m_metadata;
    bool m_isActive = false;
    bool m_shouldInitializeSucceed = true;
    bool m_shouldActivateSucceed = true;
    
    int m_initializeCount = 0;
    int m_shutdownCount = 0;
    int m_activateCount = 0;
    int m_deactivateCount = 0;
    
    nlohmann::json m_lastConfig;
    nlohmann::json m_customStatus;
    
    std::function<bool(const nlohmann::json&)> m_initializeCallback;
    std::function<void()> m_shutdownCallback;
    std::function<bool()> m_activateCallback;
    std::function<void()> m_deactivateCallback;
};

/**
 * @brief Factory function for creating mock plugins
 */
inline std::unique_ptr<plugin::IPlugin> createMockPlugin(
    const std::string& name = "MockPlugin",
    const std::string& version = "1.0.0") {
    return std::make_unique<MockPlugin>(name, version);
}

} // namespace test
} // namespace iloss