/**
 * @file LoggerTestFixture.h
 * @brief Test fixture that ensures proper Logger initialization for tests
 * @author ILOSS Development Team
 * @date 2025
 */

#pragma once

#include <gtest/gtest.h>
#include "core/logging/Logger.h"

namespace iloss {
namespace test {

/**
 * @brief Base test fixture that initializes the Logger singleton
 * 
 * This fixture ensures the Logger is properly initialized before any tests run,
 * preventing hangs or crashes when code tries to log during test execution.
 * 
 * Usage:
 * @code
 * class MyTest : public iloss::test::LoggerTestFixture {
 * protected:
 *     void SetUp() override {
 *         LoggerTestFixture::SetUp();
 *         // Your test-specific setup
 *     }
 * };
 * @endcode
 */
class LoggerTestFixture : public ::testing::Test {
public:
    /**
     * @brief Set up the test environment
     * 
     * Called once before all tests in the test suite
     */
    static void SetUpTestSuite() {
        // Initialize the logger once for all tests
        auto& logger = iloss::logging::Logger::getInstance();
        
        // Initialize with test-friendly settings:
        // - Console output only (no file output in tests)
        // - Warning level by default to reduce test output noise
        logger.initialize("ILOSS_TEST", "test.log", true, false, 
                         iloss::logging::LogLevel::Warning);
        
        // Optionally set specific categories to different levels for debugging
        // logger.setCategoryLogLevel(iloss::logging::LogCategory::Physics, 
        //                            iloss::logging::LogLevel::Debug);
    }
    
    /**
     * @brief Tear down the test environment
     * 
     * Called once after all tests in the test suite
     */
    static void TearDownTestSuite() {
        // Flush any pending log messages
        iloss::logging::Logger::getInstance().flush();
    }
    
protected:
    /**
     * @brief Set up individual test
     */
    void SetUp() override {
        // Ensure logger is initialized (in case SetUpTestSuite wasn't called)
        auto& logger = iloss::logging::Logger::getInstance();
        if (!logger.isCategoryEnabled(iloss::logging::LogCategory::Test)) {
            // Logger not initialized, do it now
            SetUpTestSuite();
        }
    }
    
    /**
     * @brief Helper to temporarily change log level for a category
     */
    void setTestLogLevel(iloss::logging::LogCategory category, 
                        iloss::logging::LogLevel level) {
        iloss::logging::Logger::getInstance().setCategoryLogLevel(category, level);
    }
    
    /**
     * @brief Helper to temporarily enable/disable a category
     */
    void setTestCategoryEnabled(iloss::logging::LogCategory category, bool enabled) {
        iloss::logging::Logger::getInstance().setCategoryEnabled(category, enabled);
    }
};

} // namespace test
} // namespace iloss