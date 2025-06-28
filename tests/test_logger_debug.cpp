#include "core/logging/Logger.h"
#include <iostream>
#include <chrono>
#include <thread>

using namespace iloss::logging;

int main() {
    std::cout << "Starting logger test..." << std::endl;
    
    // Get logger instance
    Logger& logger = Logger::getInstance();
    std::cout << "Got logger instance" << std::endl;
    
    // Initialize with custom settings - no console output to avoid potential issues
    logger.initialize("TestApp", "debug_test.log", false, true, LogLevel::Debug);
    std::cout << "Logger initialized" << std::endl;
    
    // Test logging
    logger.info(LogCategory::Core, "Test message");
    std::cout << "Logged test message" << std::endl;
    
    // Flush
    logger.flush();
    std::cout << "Flushed logger" << std::endl;
    
    // Shutdown
    logger.shutdown();
    std::cout << "Shutdown logger" << std::endl;
    
    return 0;
}