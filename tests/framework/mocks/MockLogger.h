#pragma once

#include "core/external/TestWrapper.h"
#include "core/logging/Logger.h"
#include <vector>
#include <string>
#include <tuple>

namespace iloss {
namespace test {

/**
 * @brief Mock logger for testing logging functionality
 */
class MockLogger : public logging::ILogSink {
public:
    struct LogEntry {
        logging::LogLevel level;
        std::string category;
        std::string message;
        std::chrono::system_clock::time_point timestamp;
        std::string file;
        int line;
        std::string function;
    };
    
    void log(logging::LogLevel level,
             const std::string& category,
             const std::string& message,
             const std::chrono::system_clock::time_point& timestamp,
             const std::string& file,
             int line,
             const std::string& function) override {
        LogEntry entry{level, category, message, timestamp, file, line, function};
        m_entries.push_back(entry);
        
        // Call registered callback if any
        if (m_callback) {
            m_callback(entry);
        }
    }
    
    void flush() override {
        m_flushCount++;
    }
    
    // Test utilities
    void clear() {
        m_entries.clear();
        m_flushCount = 0;
    }
    
    const std::vector<LogEntry>& getEntries() const { return m_entries; }
    
    size_t getEntryCount() const { return m_entries.size(); }
    
    size_t getEntryCount(logging::LogLevel level) const {
        return std::count_if(m_entries.begin(), m_entries.end(),
            [level](const LogEntry& e) { return e.level == level; });
    }
    
    bool hasMessage(const std::string& message) const {
        return std::any_of(m_entries.begin(), m_entries.end(),
            [&message](const LogEntry& e) { 
                return e.message.find(message) != std::string::npos; 
            });
    }
    
    bool hasCategory(const std::string& category) const {
        return std::any_of(m_entries.begin(), m_entries.end(),
            [&category](const LogEntry& e) { return e.category == category; });
    }
    
    const LogEntry* getLastEntry() const {
        return m_entries.empty() ? nullptr : &m_entries.back();
    }
    
    int getFlushCount() const { return m_flushCount; }
    
    // Set callback for real-time log monitoring
    using LogCallback = std::function<void(const LogEntry&)>;
    void setCallback(LogCallback callback) { m_callback = callback; }
    
private:
    std::vector<LogEntry> m_entries;
    int m_flushCount = 0;
    LogCallback m_callback;
};

/**
 * @brief RAII helper to temporarily replace logger sinks for testing
 */
class LoggerTestScope {
public:
    explicit LoggerTestScope(std::shared_ptr<MockLogger> mockLogger)
        : m_mockLogger(mockLogger) {
        auto& logger = logging::Logger::getInstance();
        
        // Save current sinks
        // Note: This assumes Logger has methods to get/set sinks
        // which may need to be added to the Logger interface
        
        // Clear existing sinks and add mock
        logger.clearSinks();
        logger.addSink(m_mockLogger);
    }
    
    ~LoggerTestScope() {
        // Restore original sinks
        auto& logger = logging::Logger::getInstance();
        logger.clearSinks();
        // Note: Would need to restore original sinks here
    }
    
    MockLogger& getMockLogger() { return *m_mockLogger; }
    
private:
    std::shared_ptr<MockLogger> m_mockLogger;
};

} // namespace test
} // namespace iloss