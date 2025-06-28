/**
 * @file MigrationManager.h
 * @brief Database migration management system
 * @author ILOSS Development Team
 * @date 2025
 * 
 * This class handles database schema migrations, allowing for seamless
 * upgrades between different versions of the database schema.
 */

#pragma once

#include <string>
#include <vector>
#include <functional>
#include <map>

namespace iloss {
namespace data {
namespace database {

// Forward declaration
class DatabaseManager;

/**
 * @brief Represents a single database migration
 */
struct Migration {
    int fromVersion;
    int toVersion;
    std::string description;
    std::function<void(DatabaseManager&)> apply;
    std::function<void(DatabaseManager&)> rollback;
};

/**
 * @brief Manages database schema migrations
 * 
 * This class provides a framework for managing database schema changes
 * over time, ensuring smooth upgrades and potential rollbacks.
 */
class MigrationManager {
public:
    explicit MigrationManager(DatabaseManager& dbManager);
    ~MigrationManager() = default;

    /**
     * @brief Register a migration
     * @param migration The migration to register
     */
    void registerMigration(const Migration& migration);

    /**
     * @brief Check if migrations are needed
     * @return True if the database needs migration
     */
    bool needsMigration();

    /**
     * @brief Get current database version
     * @return Current schema version
     */
    int getCurrentVersion();

    /**
     * @brief Get target database version
     * @return Target schema version
     */
    int getTargetVersion() const { return m_targetVersion; }

    /**
     * @brief Set target database version
     * @param version Target schema version
     */
    void setTargetVersion(int version) { m_targetVersion = version; }

    /**
     * @brief Migrate database to target version
     * @param targetVersion Version to migrate to (default: latest)
     * @throws std::runtime_error if migration fails
     */
    void migrate(int targetVersion = -1);

    /**
     * @brief Rollback to a previous version
     * @param targetVersion Version to rollback to
     * @throws std::runtime_error if rollback fails
     */
    void rollback(int targetVersion);

    /**
     * @brief Get list of pending migrations
     * @return Vector of migrations that need to be applied
     */
    std::vector<const Migration*> getPendingMigrations();

    /**
     * @brief Get migration history
     * @return Vector of applied migration descriptions
     */
    std::vector<std::string> getMigrationHistory();

    /**
     * @brief Create migration history table if it doesn't exist
     */
    void createMigrationTable();

    /**
     * @brief Record a migration in the history
     * @param migration The migration that was applied
     * @param success Whether the migration succeeded
     */
    void recordMigration(const Migration& migration, bool success);

    /**
     * @brief Initialize default migrations
     */
    void initializeDefaultMigrations();

private:
    DatabaseManager& m_dbManager;
    std::map<int, std::vector<Migration>> m_migrations; // Indexed by fromVersion
    int m_targetVersion;

    /**
     * @brief Find migration path from current to target version
     * @param fromVersion Starting version
     * @param toVersion Target version
     * @return Vector of migrations to apply in order
     */
    std::vector<const Migration*> findMigrationPath(int fromVersion, int toVersion);

    /**
     * @brief Apply a single migration
     * @param migration Migration to apply
     */
    void applyMigration(const Migration& migration);

    /**
     * @brief Apply a single rollback
     * @param migration Migration to rollback
     */
    void applyRollback(const Migration& migration);
};

/**
 * @brief Helper class for building migrations
 */
class MigrationBuilder {
public:
    MigrationBuilder& fromVersion(int version) {
        m_migration.fromVersion = version;
        return *this;
    }

    MigrationBuilder& toVersion(int version) {
        m_migration.toVersion = version;
        return *this;
    }

    MigrationBuilder& description(const std::string& desc) {
        m_migration.description = desc;
        return *this;
    }

    MigrationBuilder& apply(std::function<void(DatabaseManager&)> func) {
        m_migration.apply = func;
        return *this;
    }

    MigrationBuilder& rollback(std::function<void(DatabaseManager&)> func) {
        m_migration.rollback = func;
        return *this;
    }

    Migration build() {
        return m_migration;
    }

private:
    Migration m_migration;
};

} // namespace database
} // namespace data
} // namespace iloss