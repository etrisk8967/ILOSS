/**
 * @file DatabaseManager.cpp
 * @brief Implementation of the database connection manager
 * @author ILOSS Development Team
 * @date 2025
 */

#include "data/database/DatabaseManager.h"
#include "data/database/DatabaseSchema.h"
// #include "core/logging/Logger.h"  // TODO: Re-enable logging after fixing initialization
#include <sstream>
#include <filesystem>
#include <chrono>

namespace iloss {
namespace data {
namespace database {

DatabaseManager& DatabaseManager::getInstance() {
    static DatabaseManager instance;
    return instance;
}

DatabaseManager::~DatabaseManager() {
    close();
}

void DatabaseManager::initialize(const std::string& dbPath, bool createIfNotExists) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (m_database && m_database->get()) {
        throw std::runtime_error("Database is already initialized");
    }

    m_dbPath = dbPath;

    // Check if database exists
    bool dbExists = std::filesystem::exists(dbPath);
    if (!dbExists && !createIfNotExists) {
        throw std::runtime_error("Database file does not exist: " + dbPath);
    }

    // Create directory if needed
    if (!dbExists) {
        std::filesystem::path dbDir = std::filesystem::path(dbPath).parent_path();
        if (!dbDir.empty() && !std::filesystem::exists(dbDir)) {
            std::filesystem::create_directories(dbDir);
        }
    }

    try {
        // Open database
        m_database = std::make_unique<db::SQLiteDatabase>(dbPath);
        
        // Initialize pragmas
        initializePragmas();

        // Create schema if database is new
        if (!dbExists) {
            // LOG_INFO("Creating new database schema");
            createSchema();
            setSchemaVersion(SCHEMA_VERSION);
            // loadDefaultData();  // TODO: Re-enable after fixing any issues
        } else {
            // Check schema version for existing database
            int currentVersion = getSchemaVersion();
            if (currentVersion < SCHEMA_VERSION) {
                // LOG_INFO("Database schema upgrade required: {} -> {}", 
                //         currentVersion, SCHEMA_VERSION);
                // Future: Implement migration logic here
            }
        }

        // LOG_INFO("Database initialized successfully: {}", dbPath);
    } catch (const std::exception& e) {
        m_database.reset();
        throw std::runtime_error("Failed to initialize database: " + std::string(e.what()));
    }
}

void DatabaseManager::close() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_database) {
        m_database->close();
        m_database.reset();
        // LOG_INFO("Database closed");
    }
}

bool DatabaseManager::isOpen() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_database && *m_database;
}

db::SQLiteDatabase& DatabaseManager::getDatabase() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_database || !*m_database) {
        throw std::runtime_error("Database is not initialized");
    }
    return *m_database;
}

void DatabaseManager::execute(const std::string& sql) {
    std::lock_guard<std::mutex> lock(m_mutex);
    executeInternal(sql);
}

void DatabaseManager::executeInternal(const std::string& sql) {
    if (!m_database || !*m_database) {
        throw std::runtime_error("Database is not initialized");
    }
    m_database->execute(sql);
}

void DatabaseManager::beginTransaction() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_database || !*m_database) {
        throw std::runtime_error("Database is not initialized");
    }
    if (m_inTransaction) {
        throw std::runtime_error("Transaction already in progress");
    }
    m_database->execute("BEGIN TRANSACTION");
    m_inTransaction = true;
}

void DatabaseManager::commit() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_database || !*m_database) {
        throw std::runtime_error("Database is not initialized");
    }
    if (!m_inTransaction) {
        throw std::runtime_error("No transaction in progress");
    }
    m_database->execute("COMMIT");
    m_inTransaction = false;
}

void DatabaseManager::rollback() {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_database || !*m_database) {
        throw std::runtime_error("Database is not initialized");
    }
    if (!m_inTransaction) {
        throw std::runtime_error("No transaction in progress");
    }
    m_database->execute("ROLLBACK");
    m_inTransaction = false;
}

bool DatabaseManager::executeInTransaction(std::function<void()> func) {
    try {
        beginTransaction();
        func();
        commit();
        return true;
    } catch (const std::exception& e) {
        // LOG_ERROR("Transaction failed: {}", e.what());
        try {
            rollback();
        } catch (...) {
            // Ignore rollback errors
        }
        return false;
    }
}

bool DatabaseManager::executeInTransactionInternal(std::function<void()> func) {
    try {
        executeInternal("BEGIN TRANSACTION");
        m_inTransaction = true;
        func();
        executeInternal("COMMIT");
        m_inTransaction = false;
        return true;
    } catch (const std::exception& e) {
        // LOG_ERROR("Transaction failed: {}", e.what());
        try {
            executeInternal("ROLLBACK");
            m_inTransaction = false;
        } catch (...) {
            // Ignore rollback errors
        }
        return false;
    }
}

int DatabaseManager::getSchemaVersion() {
    try {
        // NOTE: Do not use getDatabase() here as it may cause deadlock
        // if called from initialize() which already holds the mutex
        if (!m_database || !*m_database) {
            return 0;
        }
        
        db::SQLiteStatement stmt(*m_database, 
            "SELECT value FROM metadata WHERE key = 'schema_version'");
        
        if (stmt.step()) {
            return std::stoi(stmt.getString(0));
        }
    } catch (...) {
        // Table might not exist yet
    }
    return 0;
}

void DatabaseManager::setSchemaVersion(int version) {
    // Use internal execute to avoid deadlock when called from initialize()
    executeInternal("INSERT OR REPLACE INTO metadata (key, value) VALUES ('schema_version', '" + 
            std::to_string(version) + "')");
}

void DatabaseManager::setForeignKeyConstraints(bool enable) {
    executeInternal(enable ? "PRAGMA foreign_keys = ON" : "PRAGMA foreign_keys = OFF");
}

void DatabaseManager::setJournalMode(const std::string& mode) {
    try {
        executeInternal("PRAGMA journal_mode = " + mode);
    } catch (const std::exception& e) {
        // SQLite doesn't allow changing out of WAL mode in certain conditions
        // This is a known limitation, so we'll ignore the error
        // LOG_WARNING("Failed to change journal mode: {}", e.what());
    }
}

void DatabaseManager::optimize() {
    executeInternal("VACUUM");
    executeInternal("ANALYZE");
}

std::string DatabaseManager::getDatabaseStats() {
    std::stringstream stats;
    
    // Get page count and size
    db::SQLiteStatement pageCountStmt(getDatabase(), "PRAGMA page_count");
    db::SQLiteStatement pageSizeStmt(getDatabase(), "PRAGMA page_size");
    
    if (pageCountStmt.step() && pageSizeStmt.step()) {
        int pageCount = pageCountStmt.getInt(0);
        int pageSize = pageSizeStmt.getInt(0);
        int totalSize = pageCount * pageSize;
        
        stats << "Database size: " << (totalSize / 1024.0 / 1024.0) << " MB\n";
        stats << "Page count: " << pageCount << "\n";
        stats << "Page size: " << pageSize << " bytes\n";
    }
    
    // Get table count
    db::SQLiteStatement tableStmt(getDatabase(), 
        "SELECT COUNT(*) FROM sqlite_master WHERE type='table'");
    if (tableStmt.step()) {
        stats << "Tables: " << tableStmt.getInt(0) << "\n";
    }
    
    // Get index count
    db::SQLiteStatement indexStmt(getDatabase(), 
        "SELECT COUNT(*) FROM sqlite_master WHERE type='index'");
    if (indexStmt.step()) {
        stats << "Indexes: " << indexStmt.getInt(0) << "\n";
    }
    
    return stats.str();
}

void DatabaseManager::createSchema() {
    // Create all tables
    for (const auto& createTable : Schema::ALL_TABLES) {
        executeInternal(createTable);
    }
    
    // Create indexes
    for (const auto& createIndex : Schema::CREATE_INDEXES) {
        executeInternal(createIndex);
    }
    
    // Create triggers
    for (const auto& createTrigger : Schema::CREATE_TRIGGERS) {
        executeInternal(createTrigger);
    }
}

void DatabaseManager::initializePragmas() {
    // Enable foreign keys
    setForeignKeyConstraints(true);
    
    // Set journal mode to WAL for better concurrency
    setJournalMode("WAL");
    
    // Set synchronous mode
    executeInternal("PRAGMA synchronous = NORMAL");
    
    // Set temp store to memory
    executeInternal("PRAGMA temp_store = MEMORY");
    
    // Set cache size (negative for KB)
    executeInternal("PRAGMA cache_size = -8000"); // 8MB cache
    
    // Enable query optimizer
    executeInternal("PRAGMA optimize");
}

bool DatabaseManager::tableExists(const std::string& tableName) {
    db::SQLiteStatement stmt(getDatabase(), 
        "SELECT COUNT(*) FROM sqlite_master WHERE type='table' AND name=?");
    stmt.bind(1, tableName);
    
    if (stmt.step()) {
        return stmt.getInt(0) > 0;
    }
    return false;
}

void DatabaseManager::loadDefaultData() {
    executeInTransactionInternal([this]() {
        // Insert default launch sites
        execute(R"(
            INSERT INTO launch_sites (name, code, latitude, longitude, elevation, azimuth_min, azimuth_max, is_builtin, description)
            VALUES 
            ('Cape Canaveral SFS', 'CCAFS', 28.488889, -80.577778, 3.0, 35.0, 120.0, 1, 'Cape Canaveral Space Force Station, Florida, USA'),
            ('Kennedy Space Center', 'KSC', 28.524058, -80.650849, 3.0, 35.0, 120.0, 1, 'Kennedy Space Center LC-39, Florida, USA'),
            ('Vandenberg SFB', 'VAFB', 34.581163, -120.626789, 174.0, 140.0, 220.0, 1, 'Vandenberg Space Force Base, California, USA'),
            ('Baikonur Cosmodrome', 'TYSC', 45.920278, 63.342222, 90.0, 0.0, 360.0, 1, 'Baikonur Cosmodrome, Kazakhstan'),
            ('Guiana Space Centre', 'CSG', 5.239380, -52.768487, 14.0, 90.0, 120.0, 1, 'Centre Spatial Guyanais, French Guiana')
        )");

        // Insert default vehicle template (Falcon 9)
        execute(R"(
            INSERT INTO vehicles (name, manufacturer, type, is_builtin)
            VALUES ('Falcon 9 v1.2', 'SpaceX', 'Medium-lift launch vehicle', 1)
        )");

        // Get the vehicle ID
        int64_t vehicleId = getDatabase().lastInsertRowId();

        // Insert Falcon 9 first stage
        execute("INSERT INTO vehicle_stages (vehicle_id, stage_number, name, gross_mass, dry_mass, propellant_mass, length, diameter) "
                "VALUES (" + std::to_string(vehicleId) + ", 1, 'First Stage', 433100, 25600, 407500, 42.6, 3.7)");
        
        int64_t stage1Id = getDatabase().lastInsertRowId();

        // Insert first stage engines
        execute("INSERT INTO engines (stage_id, name, count, thrust_sl, thrust_vac, isp_sl, isp_vac, mass_flow_rate, burn_time) "
                "VALUES (" + std::to_string(stage1Id) + ", 'Merlin 1D', 9, 845, 914, 282, 311, 293.5, 162)");

        // Insert Falcon 9 second stage
        execute("INSERT INTO vehicle_stages (vehicle_id, stage_number, name, gross_mass, dry_mass, propellant_mass, length, diameter) "
                "VALUES (" + std::to_string(vehicleId) + ", 2, 'Second Stage', 116000, 4000, 112000, 14.9, 3.7)");
        
        int64_t stage2Id = getDatabase().lastInsertRowId();

        // Insert second stage engine
        execute("INSERT INTO engines (stage_id, name, count, thrust_sl, thrust_vac, isp_sl, isp_vac, mass_flow_rate, burn_time) "
                "VALUES (" + std::to_string(stage2Id) + ", 'Merlin 1D Vacuum', 1, 0, 981, 0, 348, 282, 397)");

        // Insert default user preferences
        execute(R"(
            INSERT INTO user_preferences (key, value, category)
            VALUES 
            ('theme', 'dark', 'ui'),
            ('units', 'metric', 'display'),
            ('timestep', '0.1', 'simulation'),
            ('integrator', 'RK4', 'simulation'),
            ('autosave_interval', '300', 'general')
        )");

        // Insert mission template
        execute(R"(
            INSERT INTO templates (name, type, description, template_json, is_builtin)
            VALUES (
                'LEO Mission Template',
                'mission',
                'Standard template for Low Earth Orbit missions',
                '{"target_altitude": 400000, "target_inclination": 51.6, "target_eccentricity": 0.0, "force_models": ["gravity", "drag", "thirdbody"]}',
                1
            )
        )");
    });
}

} // namespace database
} // namespace data
} // namespace iloss