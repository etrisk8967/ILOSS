/**
 * @file DatabaseManager.h
 * @brief Database connection manager for ILOSS
 * @author ILOSS Development Team
 * @date 2025
 * 
 * This class manages the database connection, initialization, and provides
 * thread-safe access to the database.
 */

#pragma once

#include "core/external/SQLiteWrapper.h"
#include <string>
#include <memory>
#include <mutex>
#include <functional>

namespace iloss {
namespace data {
namespace database {

/**
 * @brief Manages database connections and operations for ILOSS
 * 
 * This class provides a singleton interface to the SQLite database,
 * handles database initialization, schema creation, and migrations.
 */
class DatabaseManager {
public:
    /**
     * @brief Get the singleton instance of DatabaseManager
     * @return Reference to the DatabaseManager instance
     */
    static DatabaseManager& getInstance();

    /**
     * @brief Initialize the database with the given file path
     * @param dbPath Path to the database file
     * @param createIfNotExists Create the database if it doesn't exist
     * @throws std::runtime_error if initialization fails
     */
    void initialize(const std::string& dbPath, bool createIfNotExists = true);

    /**
     * @brief Close the database connection
     */
    void close();

    /**
     * @brief Check if the database is open and ready
     * @return True if database is open, false otherwise
     */
    bool isOpen() const;

    /**
     * @brief Get the database connection
     * @return Pointer to the SQLite database connection
     * @throws std::runtime_error if database is not open
     */
    db::SQLiteDatabase& getDatabase();

    /**
     * @brief Execute a SQL statement
     * @param sql SQL statement to execute
     * @throws std::runtime_error if execution fails
     */
    void execute(const std::string& sql);

    /**
     * @brief Begin a database transaction
     * @throws std::runtime_error if transaction start fails
     */
    void beginTransaction();

    /**
     * @brief Commit the current transaction
     * @throws std::runtime_error if commit fails
     */
    void commit();

    /**
     * @brief Rollback the current transaction
     * @throws std::runtime_error if rollback fails
     */
    void rollback();

    /**
     * @brief Execute a function within a transaction
     * @param func Function to execute
     * @return True if transaction succeeded, false if rolled back
     */
    bool executeInTransaction(std::function<void()> func);

    /**
     * @brief Get the current schema version
     * @return Current schema version, or 0 if not initialized
     */
    int getSchemaVersion();

    /**
     * @brief Set the schema version
     * @param version New schema version
     */
    void setSchemaVersion(int version);

    /**
     * @brief Get the database file path
     * @return Path to the database file
     */
    const std::string& getDatabasePath() const { return m_dbPath; }

    /**
     * @brief Enable or disable foreign key constraints
     * @param enable True to enable, false to disable
     */
    void setForeignKeyConstraints(bool enable);

    /**
     * @brief Set journal mode for the database
     * @param mode Journal mode (e.g., "WAL", "DELETE", "MEMORY")
     */
    void setJournalMode(const std::string& mode);

    /**
     * @brief Optimize the database (VACUUM)
     */
    void optimize();

    /**
     * @brief Get database statistics
     * @return String containing database statistics
     */
    std::string getDatabaseStats();

    // Delete copy constructor and assignment operator
    DatabaseManager(const DatabaseManager&) = delete;
    DatabaseManager& operator=(const DatabaseManager&) = delete;

private:
    DatabaseManager() = default;
    ~DatabaseManager();

    /**
     * @brief Create the database schema
     */
    void createSchema();

    /**
     * @brief Initialize database pragmas and settings
     */
    void initializePragmas();

    /**
     * @brief Check if a table exists
     * @param tableName Name of the table to check
     * @return True if table exists, false otherwise
     */
    bool tableExists(const std::string& tableName);

    /**
     * @brief Load default data into the database
     */
    void loadDefaultData();

private:
    /**
     * @brief Internal execute method that doesn't acquire mutex
     * @param sql SQL statement to execute
     */
    void executeInternal(const std::string& sql);
    
    /**
     * @brief Internal transaction method that doesn't acquire mutex
     * @param func Function to execute in transaction
     * @return True if transaction succeeded, false if rolled back
     */
    bool executeInTransactionInternal(std::function<void()> func);

    mutable std::mutex m_mutex;
    std::unique_ptr<db::SQLiteDatabase> m_database;
    std::string m_dbPath;
    bool m_inTransaction = false;
};

/**
 * @brief RAII class for automatic transaction management
 */
class DatabaseTransaction {
public:
    explicit DatabaseTransaction(DatabaseManager& manager)
        : m_manager(manager), m_committed(false) {
        m_manager.beginTransaction();
    }

    ~DatabaseTransaction() {
        if (!m_committed) {
            try {
                m_manager.rollback();
            } catch (...) {
                // Suppress exceptions in destructor
            }
        }
    }

    void commit() {
        m_manager.commit();
        m_committed = true;
    }

    void rollback() {
        m_manager.rollback();
        m_committed = true;
    }

private:
    DatabaseManager& m_manager;
    bool m_committed;
};

} // namespace database
} // namespace data
} // namespace iloss