#pragma once

#include "core/external/TestWrapper.h"
#include <filesystem>
#include <fstream>
#include <chrono>
#include <thread>
#include <functional>

namespace iloss {
namespace test {

/**
 * @brief Collection of utility functions for testing
 */
class TestUtilities {
public:
    // File system utilities
    static bool createFile(const std::filesystem::path& path, 
                          const std::string& content = "") {
        try {
            std::filesystem::create_directories(path.parent_path());
            std::ofstream file(path);
            if (file.is_open()) {
                file << content;
                return true;
            }
        } catch (...) {
            // Ignore exceptions
        }
        return false;
    }
    
    static std::string readFile(const std::filesystem::path& path) {
        std::ifstream file(path);
        if (!file.is_open()) {
            return "";
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }
    
    static bool fileExists(const std::filesystem::path& path) {
        return std::filesystem::exists(path);
    }
    
    static bool deleteFile(const std::filesystem::path& path) {
        try {
            return std::filesystem::remove(path);
        } catch (...) {
            return false;
        }
    }
    
    static bool createDirectory(const std::filesystem::path& path) {
        try {
            return std::filesystem::create_directories(path);
        } catch (...) {
            return false;
        }
    }
    
    // Timing utilities
    template<typename Func>
    static double measureExecutionTime(Func&& func) {
        auto start = std::chrono::high_resolution_clock::now();
        func();
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        return duration.count() / 1000.0;  // Return milliseconds
    }
    
    template<typename Func>
    static double measureAverageExecutionTime(Func&& func, int iterations = 100) {
        double totalTime = 0.0;
        for (int i = 0; i < iterations; ++i) {
            totalTime += measureExecutionTime(func);
        }
        return totalTime / iterations;
    }
    
    // Thread utilities
    static void waitFor(std::function<bool()> condition, 
                       std::chrono::milliseconds timeout = std::chrono::milliseconds(1000),
                       std::chrono::milliseconds pollInterval = std::chrono::milliseconds(10)) {
        auto start = std::chrono::steady_clock::now();
        
        while (!condition()) {
            if (std::chrono::steady_clock::now() - start > timeout) {
                FAIL() << "Timeout waiting for condition";
            }
            std::this_thread::sleep_for(pollInterval);
        }
    }
    
    template<typename T>
    static void waitForValue(const T& value, 
                            std::function<T()> getter,
                            std::chrono::milliseconds timeout = std::chrono::milliseconds(1000)) {
        waitFor([&]() { return getter() == value; }, timeout);
    }
    
    // Comparison utilities with tolerance
    static bool nearlyEqual(double a, double b, double tolerance = 1e-9) {
        return std::abs(a - b) <= tolerance;
    }
    
    static bool nearlyEqual(const std::vector<double>& a, 
                           const std::vector<double>& b, 
                           double tolerance = 1e-9) {
        if (a.size() != b.size()) return false;
        
        for (size_t i = 0; i < a.size(); ++i) {
            if (!nearlyEqual(a[i], b[i], tolerance)) {
                return false;
            }
        }
        return true;
    }
    
    // String utilities
    static bool contains(const std::string& str, const std::string& substr) {
        return str.find(substr) != std::string::npos;
    }
    
    static std::vector<std::string> split(const std::string& str, char delimiter) {
        std::vector<std::string> tokens;
        std::stringstream ss(str);
        std::string token;
        
        while (std::getline(ss, token, delimiter)) {
            tokens.push_back(token);
        }
        
        return tokens;
    }
    
    static std::string join(const std::vector<std::string>& strings, 
                           const std::string& delimiter) {
        if (strings.empty()) return "";
        
        std::stringstream ss;
        ss << strings[0];
        for (size_t i = 1; i < strings.size(); ++i) {
            ss << delimiter << strings[i];
        }
        return ss.str();
    }
    
    // Random utilities
    static std::string randomString(size_t length) {
        static const char charset[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
        static std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());
        static std::uniform_int_distribution<> dist(0, sizeof(charset) - 2);
        
        std::string result;
        result.reserve(length);
        for (size_t i = 0; i < length; ++i) {
            result += charset[dist(rng)];
        }
        return result;
    }
    
    // Memory utilities
    template<typename T>
    static size_t getObjectSize(const T& obj) {
        return sizeof(obj);
    }
    
    // Exception testing utilities
    template<typename Exception, typename Func>
    static void expectException(Func&& func, const std::string& expectedMessage = "") {
        try {
            func();
            FAIL() << "Expected exception of type " << typeid(Exception).name() 
                   << " but no exception was thrown";
        } catch (const Exception& e) {
            if (!expectedMessage.empty()) {
                EXPECT_THAT(e.what(), ::testing::HasSubstr(expectedMessage));
            }
        } catch (...) {
            FAIL() << "Expected exception of type " << typeid(Exception).name() 
                   << " but different exception was thrown";
        }
    }
    
    template<typename Func>
    static void expectNoException(Func&& func) {
        try {
            func();
        } catch (const std::exception& e) {
            FAIL() << "Unexpected exception: " << e.what();
        } catch (...) {
            FAIL() << "Unexpected exception of unknown type";
        }
    }
};

/**
 * @brief RAII class for temporary environment variable changes
 */
class EnvironmentVariableScope {
public:
    EnvironmentVariableScope(const std::string& name, const std::string& value)
        : m_name(name) {
        // Save original value
        const char* original = std::getenv(name.c_str());
        if (original) {
            m_hadOriginal = true;
            m_originalValue = original;
        }
        
        // Set new value
#ifdef _WIN32
        _putenv_s(name.c_str(), value.c_str());
#else
        setenv(name.c_str(), value.c_str(), 1);
#endif
    }
    
    ~EnvironmentVariableScope() {
        // Restore original value
        if (m_hadOriginal) {
#ifdef _WIN32
            _putenv_s(m_name.c_str(), m_originalValue.c_str());
#else
            setenv(m_name.c_str(), m_originalValue.c_str(), 1);
#endif
        } else {
#ifdef _WIN32
            _putenv_s(m_name.c_str(), "");
#else
            unsetenv(m_name.c_str());
#endif
        }
    }
    
private:
    std::string m_name;
    std::string m_originalValue;
    bool m_hadOriginal = false;
};

/**
 * @brief RAII class for temporary working directory changes
 */
class WorkingDirectoryScope {
public:
    explicit WorkingDirectoryScope(const std::filesystem::path& newPath) {
        m_originalPath = std::filesystem::current_path();
        std::filesystem::current_path(newPath);
    }
    
    ~WorkingDirectoryScope() {
        try {
            std::filesystem::current_path(m_originalPath);
        } catch (...) {
            // Ignore errors on restoration
        }
    }
    
private:
    std::filesystem::path m_originalPath;
};

/**
 * @brief Test data file manager for accessing test resources
 */
class TestDataManager {
public:
    static TestDataManager& getInstance() {
        static TestDataManager instance;
        return instance;
    }
    
    void setTestDataRoot(const std::filesystem::path& root) {
        m_testDataRoot = root;
    }
    
    std::filesystem::path getTestDataPath(const std::string& relativePath) const {
        return m_testDataRoot / relativePath;
    }
    
    std::string loadTestData(const std::string& relativePath) const {
        auto fullPath = getTestDataPath(relativePath);
        return TestUtilities::readFile(fullPath);
    }
    
    bool testDataExists(const std::string& relativePath) const {
        return std::filesystem::exists(getTestDataPath(relativePath));
    }
    
private:
    TestDataManager() {
        // Default to data directory relative to binary
        m_testDataRoot = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "data";
    }
    
    std::filesystem::path m_testDataRoot;
};

} // namespace test
} // namespace iloss