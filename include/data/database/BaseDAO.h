/**
 * @file BaseDAO.h
 * @brief Base Data Access Object class for database operations
 * @author ILOSS Development Team
 * @date 2025
 */

#pragma once

#include "data/database/DatabaseManager.h"
#include "core/external/SQLiteWrapper.h"
#include <string>
#include <vector>
#include <optional>
#include <memory>
#include <chrono>

namespace iloss {
namespace data {
namespace database {

/**
 * @brief Base class for all Data Access Objects
 * 
 * Provides common database operations and utilities for derived DAO classes.
 * @tparam T The entity type managed by this DAO
 */
template<typename T>
class BaseDAO {
public:
    using EntityPtr = std::shared_ptr<T>;
    using EntityList = std::vector<EntityPtr>;

    /**
     * @brief Constructor
     * @param tableName Name of the database table
     */
    explicit BaseDAO(const std::string& tableName) 
        : m_tableName(tableName), m_dbManager(DatabaseManager::getInstance()) {}

    virtual ~BaseDAO() = default;

    /**
     * @brief Find an entity by ID
     * @param id The entity ID
     * @return Optional containing the entity if found
     */
    virtual std::optional<EntityPtr> findById(int64_t id) = 0;

    /**
     * @brief Find all entities
     * @return Vector of all entities
     */
    virtual EntityList findAll() = 0;

    /**
     * @brief Save an entity (insert or update)
     * @param entity The entity to save
     * @return The saved entity with updated ID if inserted
     */
    virtual EntityPtr save(EntityPtr entity) = 0;

    /**
     * @brief Delete an entity by ID
     * @param id The entity ID
     * @return True if deleted, false if not found
     */
    virtual bool deleteById(int64_t id) {
        std::string sql = "DELETE FROM " + m_tableName + " WHERE id = ?";
        db::SQLiteStatement stmt(getDatabase(), sql);
        stmt.bind(1, id);
        stmt.execute();
        return getDatabase().changesCount() > 0;
    }

    /**
     * @brief Count total entities
     * @return Total count
     */
    virtual int64_t count() {
        std::string sql = "SELECT COUNT(*) FROM " + m_tableName;
        db::SQLiteStatement stmt(getDatabase(), sql);
        if (stmt.step()) {
            return stmt.getInt64(0);
        }
        return 0;
    }

    /**
     * @brief Check if an entity exists by ID
     * @param id The entity ID
     * @return True if exists, false otherwise
     */
    virtual bool exists(int64_t id) {
        std::string sql = "SELECT 1 FROM " + m_tableName + " WHERE id = ? LIMIT 1";
        db::SQLiteStatement stmt(getDatabase(), sql);
        stmt.bind(1, id);
        return stmt.step();
    }

protected:
    /**
     * @brief Get the database connection
     * @return Reference to the database
     */
    db::SQLiteDatabase& getDatabase() {
        return m_dbManager.getDatabase();
    }

    /**
     * @brief Get the database manager
     * @return Reference to the database manager
     */
    DatabaseManager& getDatabaseManager() {
        return m_dbManager;
    }

    /**
     * @brief Convert from entity to database representation
     * @param entity The entity to convert
     * @param stmt The prepared statement to bind values to
     */
    virtual void bindEntity(const T& entity, db::SQLiteStatement& stmt) = 0;

    /**
     * @brief Convert from database representation to entity
     * @param stmt The statement with current row data
     * @return The constructed entity
     */
    virtual EntityPtr extractEntity(db::SQLiteStatement& stmt) = 0;

    /**
     * @brief Get the current timestamp as string
     * @return ISO 8601 timestamp string
     */
    std::string getCurrentTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        char buffer[30];
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", std::gmtime(&time_t));
        return std::string(buffer);
    }

    /**
     * @brief Execute a query and return entities
     * @param sql The SQL query
     * @param bindFunc Optional function to bind parameters
     * @return Vector of entities
     */
    EntityList executeQuery(const std::string& sql, 
                          std::function<void(db::SQLiteStatement&)> bindFunc = nullptr) {
        db::SQLiteStatement stmt(getDatabase(), sql);
        if (bindFunc) {
            bindFunc(stmt);
        }
        
        EntityList results;
        while (stmt.step()) {
            results.push_back(extractEntity(stmt));
        }
        return results;
    }

    /**
     * @brief Execute a query and return single entity
     * @param sql The SQL query
     * @param bindFunc Optional function to bind parameters
     * @return Optional containing the entity if found
     */
    std::optional<EntityPtr> executeQuerySingle(const std::string& sql,
                                               std::function<void(db::SQLiteStatement&)> bindFunc = nullptr) {
        auto results = executeQuery(sql, bindFunc);
        if (!results.empty()) {
            return results[0];
        }
        return std::nullopt;
    }

protected:
    std::string m_tableName;
    DatabaseManager& m_dbManager;
};

} // namespace database
} // namespace data
} // namespace iloss