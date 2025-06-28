/**
 * @file test_DatabaseManagerMinimal.cpp
 * @brief Minimal test to debug DatabaseManager
 * @author ILOSS Development Team
 * @date 2025
 */

#include <iostream>
#include <filesystem>
#include "data/database/DatabaseManager.h"

int main() {
    std::cout << "Starting minimal database test...\n";
    
    std::string testDbPath = "test_minimal.db";
    
    try {
        std::cout << "Getting instance...\n";
        auto& dbManager = iloss::data::database::DatabaseManager::getInstance();
        
        std::cout << "Before initialize...\n";
        dbManager.initialize(testDbPath, true);
        std::cout << "After initialize...\n";
        
        std::cout << "Database initialized successfully!\n";
        std::cout << "Schema version: " << dbManager.getSchemaVersion() << "\n";
        
        dbManager.close();
        std::filesystem::remove(testDbPath);
        
        std::cout << "Test completed successfully!\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}