/**
 * @file test_DatabaseManager.cpp
 * @brief Unit tests for DatabaseManager class
 * @author ILOSS Development Team
 * @date 2025
 */

#include <gtest/gtest.h>
#include "data/database/DatabaseManager.h"
#include "data/database/DatabaseSchema.h"
#include "core/external/SQLiteWrapper.h"
#include <filesystem>
#include <thread>

using namespace iloss::data::database;
using namespace iloss::db;

class DatabaseManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a temporary database file for testing
        m_testDbPath = "test_database_" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".db";
    }

    void TearDown() override {
        // Clean up test database
        DatabaseManager::getInstance().close();
        std::filesystem::remove(m_testDbPath);
    }

    std::string m_testDbPath;
};

TEST_F(DatabaseManagerTest, SingletonInstance) {
    // Test that getInstance returns the same instance
    auto& instance1 = DatabaseManager::getInstance();
    auto& instance2 = DatabaseManager::getInstance();
    
    EXPECT_EQ(&instance1, &instance2);
}

TEST_F(DatabaseManagerTest, InitializeNewDatabase) {
    auto& dbManager = DatabaseManager::getInstance();
    
    EXPECT_NO_THROW(dbManager.initialize(m_testDbPath, true));
    EXPECT_TRUE(dbManager.isOpen());
    EXPECT_TRUE(std::filesystem::exists(m_testDbPath));
    
    // Verify schema version
    EXPECT_EQ(dbManager.getSchemaVersion(), SCHEMA_VERSION);
}

TEST_F(DatabaseManagerTest, InitializeExistingDatabase) {
    auto& dbManager = DatabaseManager::getInstance();
    
    // Create database first
    dbManager.initialize(m_testDbPath, true);
    dbManager.close();
    
    // Re-open existing database
    EXPECT_NO_THROW(dbManager.initialize(m_testDbPath, false));
    EXPECT_TRUE(dbManager.isOpen());
}

TEST_F(DatabaseManagerTest, InitializeNonExistentDatabaseNoCreate) {
    auto& dbManager = DatabaseManager::getInstance();
    
    EXPECT_THROW(
        dbManager.initialize("nonexistent.db", false),
        std::runtime_error
    );
}

TEST_F(DatabaseManagerTest, CloseDatabase) {
    auto& dbManager = DatabaseManager::getInstance();
    
    dbManager.initialize(m_testDbPath, true);
    EXPECT_TRUE(dbManager.isOpen());
    
    dbManager.close();
    EXPECT_FALSE(dbManager.isOpen());
}

TEST_F(DatabaseManagerTest, DoubleInitializeThrows) {
    auto& dbManager = DatabaseManager::getInstance();
    
    dbManager.initialize(m_testDbPath, true);
    
    EXPECT_THROW(
        dbManager.initialize(m_testDbPath, true),
        std::runtime_error
    );
}

TEST_F(DatabaseManagerTest, ExecuteSQL) {
    auto& dbManager = DatabaseManager::getInstance();
    dbManager.initialize(m_testDbPath, true);
    
    // Test simple SQL execution
    EXPECT_NO_THROW(
        dbManager.execute("CREATE TABLE test_table (id INTEGER PRIMARY KEY, name TEXT)")
    );
    
    EXPECT_NO_THROW(
        dbManager.execute("INSERT INTO test_table (name) VALUES ('test')")
    );
}

TEST_F(DatabaseManagerTest, ExecuteInvalidSQL) {
    auto& dbManager = DatabaseManager::getInstance();
    dbManager.initialize(m_testDbPath, true);
    
    EXPECT_THROW(
        dbManager.execute("INVALID SQL STATEMENT"),
        std::runtime_error
    );
}

TEST_F(DatabaseManagerTest, TransactionCommit) {
    auto& dbManager = DatabaseManager::getInstance();
    dbManager.initialize(m_testDbPath, true);
    
    // Create test table
    dbManager.execute("CREATE TABLE test_table (id INTEGER PRIMARY KEY, value INTEGER)");
    
    // Test transaction
    dbManager.beginTransaction();
    dbManager.execute("INSERT INTO test_table (value) VALUES (42)");
    dbManager.commit();
    
    // Verify data was committed
    SQLiteStatement stmt(dbManager.getDatabase(), "SELECT value FROM test_table");
    EXPECT_TRUE(stmt.step());
    EXPECT_EQ(stmt.getInt(0), 42);
}

TEST_F(DatabaseManagerTest, TransactionRollback) {
    auto& dbManager = DatabaseManager::getInstance();
    dbManager.initialize(m_testDbPath, true);
    
    // Create test table
    dbManager.execute("CREATE TABLE test_table (id INTEGER PRIMARY KEY, value INTEGER)");
    
    // Test rollback
    dbManager.beginTransaction();
    dbManager.execute("INSERT INTO test_table (value) VALUES (42)");
    dbManager.rollback();
    
    // Verify data was not committed
    SQLiteStatement stmt(dbManager.getDatabase(), "SELECT COUNT(*) FROM test_table");
    EXPECT_TRUE(stmt.step());
    EXPECT_EQ(stmt.getInt(0), 0);
}

TEST_F(DatabaseManagerTest, ExecuteInTransaction) {
    auto& dbManager = DatabaseManager::getInstance();
    dbManager.initialize(m_testDbPath, true);
    
    // Create test table
    dbManager.execute("CREATE TABLE test_table (id INTEGER PRIMARY KEY, value INTEGER)");
    
    // Test successful transaction
    bool result = dbManager.executeInTransaction([&]() {
        dbManager.execute("INSERT INTO test_table (value) VALUES (100)");
        dbManager.execute("INSERT INTO test_table (value) VALUES (200)");
    });
    
    EXPECT_TRUE(result);
    
    // Verify data
    SQLiteStatement stmt(dbManager.getDatabase(), "SELECT COUNT(*) FROM test_table");
    EXPECT_TRUE(stmt.step());
    EXPECT_EQ(stmt.getInt(0), 2);
}

TEST_F(DatabaseManagerTest, ExecuteInTransactionRollbackOnError) {
    auto& dbManager = DatabaseManager::getInstance();
    dbManager.initialize(m_testDbPath, true);
    
    // Create test table
    dbManager.execute("CREATE TABLE test_table (id INTEGER PRIMARY KEY, value INTEGER)");
    
    // Test failed transaction
    bool result = dbManager.executeInTransaction([&]() {
        dbManager.execute("INSERT INTO test_table (value) VALUES (100)");
        // This should fail
        dbManager.execute("INVALID SQL");
    });
    
    EXPECT_FALSE(result);
    
    // Verify no data was committed
    SQLiteStatement stmt(dbManager.getDatabase(), "SELECT COUNT(*) FROM test_table");
    EXPECT_TRUE(stmt.step());
    EXPECT_EQ(stmt.getInt(0), 0);
}

TEST_F(DatabaseManagerTest, SetAndGetSchemaVersion) {
    auto& dbManager = DatabaseManager::getInstance();
    dbManager.initialize(m_testDbPath, true);
    
    int newVersion = 42;
    dbManager.setSchemaVersion(newVersion);
    
    EXPECT_EQ(dbManager.getSchemaVersion(), newVersion);
}

TEST_F(DatabaseManagerTest, ForeignKeyConstraints) {
    auto& dbManager = DatabaseManager::getInstance();
    dbManager.initialize(m_testDbPath, true);
    
    // Foreign keys should be enabled by default
    SQLiteStatement stmt(dbManager.getDatabase(), "PRAGMA foreign_keys");
    EXPECT_TRUE(stmt.step());
    EXPECT_EQ(stmt.getInt(0), 1);
    
    // Test disabling
    dbManager.setForeignKeyConstraints(false);
    SQLiteStatement stmt2(dbManager.getDatabase(), "PRAGMA foreign_keys");
    EXPECT_TRUE(stmt2.step());
    EXPECT_EQ(stmt2.getInt(0), 0);
}

TEST_F(DatabaseManagerTest, JournalMode) {
    auto& dbManager = DatabaseManager::getInstance();
    dbManager.initialize(m_testDbPath, true);
    
    // Journal mode should be WAL by default
    SQLiteStatement stmt(dbManager.getDatabase(), "PRAGMA journal_mode");
    EXPECT_TRUE(stmt.step());
    EXPECT_EQ(stmt.getString(0), "wal");
    
    // Note: SQLite doesn't allow changing out of WAL mode within a transaction
    // or when there are active statements. This is a SQLite limitation.
    // We can test that the method doesn't throw, but the mode won't actually change.
    EXPECT_NO_THROW(dbManager.setJournalMode("DELETE"));
}

TEST_F(DatabaseManagerTest, DatabaseStats) {
    auto& dbManager = DatabaseManager::getInstance();
    dbManager.initialize(m_testDbPath, true);
    
    std::string stats = dbManager.getDatabaseStats();
    
    EXPECT_FALSE(stats.empty());
    EXPECT_NE(stats.find("Database size:"), std::string::npos);
    EXPECT_NE(stats.find("Tables:"), std::string::npos);
    EXPECT_NE(stats.find("Indexes:"), std::string::npos);
}

TEST_F(DatabaseManagerTest, Optimize) {
    auto& dbManager = DatabaseManager::getInstance();
    dbManager.initialize(m_testDbPath, true);
    
    // Add some data
    dbManager.execute("CREATE TABLE test_table (id INTEGER PRIMARY KEY, data TEXT)");
    for (int i = 0; i < 100; ++i) {
        dbManager.execute("INSERT INTO test_table (data) VALUES ('test data " + std::to_string(i) + "')");
    }
    
    // Should not throw
    EXPECT_NO_THROW(dbManager.optimize());
}

TEST_F(DatabaseManagerTest, DatabaseTransaction) {
    auto& dbManager = DatabaseManager::getInstance();
    dbManager.initialize(m_testDbPath, true);
    
    // Create test table
    dbManager.execute("CREATE TABLE test_table (id INTEGER PRIMARY KEY, value INTEGER)");
    
    // Test RAII transaction commit
    {
        DatabaseTransaction transaction(dbManager);
        dbManager.execute("INSERT INTO test_table (value) VALUES (123)");
        transaction.commit();
    }
    
    // Verify data was committed
    SQLiteStatement stmt(dbManager.getDatabase(), "SELECT value FROM test_table");
    EXPECT_TRUE(stmt.step());
    EXPECT_EQ(stmt.getInt(0), 123);
}

TEST_F(DatabaseManagerTest, DatabaseTransactionAutoRollback) {
    auto& dbManager = DatabaseManager::getInstance();
    dbManager.initialize(m_testDbPath, true);
    
    // Create test table
    dbManager.execute("CREATE TABLE test_table (id INTEGER PRIMARY KEY, value INTEGER)");
    
    // Test RAII transaction auto-rollback
    {
        DatabaseTransaction transaction(dbManager);
        dbManager.execute("INSERT INTO test_table (value) VALUES (456)");
        // No commit - should auto-rollback
    }
    
    // Verify data was not committed
    SQLiteStatement stmt(dbManager.getDatabase(), "SELECT COUNT(*) FROM test_table");
    EXPECT_TRUE(stmt.step());
    EXPECT_EQ(stmt.getInt(0), 0);
}

TEST_F(DatabaseManagerTest, ConcurrentAccess) {
    auto& dbManager = DatabaseManager::getInstance();
    dbManager.initialize(m_testDbPath, true);
    
    // Create test table
    dbManager.execute("CREATE TABLE test_table (id INTEGER PRIMARY KEY, value INTEGER)");
    
    const int numThreads = 5;
    const int insertsPerThread = 20;
    std::vector<std::thread> threads;
    
    // Launch multiple threads to insert data
    for (int t = 0; t < numThreads; ++t) {
        threads.emplace_back([&, t]() {
            for (int i = 0; i < insertsPerThread; ++i) {
                dbManager.execute("INSERT INTO test_table (value) VALUES (" + 
                                std::to_string(t * 100 + i) + ")");
            }
        });
    }
    
    // Wait for all threads
    for (auto& thread : threads) {
        thread.join();
    }
    
    // Verify all data was inserted
    SQLiteStatement stmt(dbManager.getDatabase(), "SELECT COUNT(*) FROM test_table");
    EXPECT_TRUE(stmt.step());
    EXPECT_EQ(stmt.getInt(0), numThreads * insertsPerThread);
}

TEST_F(DatabaseManagerTest, SchemaCreation) {
    auto& dbManager = DatabaseManager::getInstance();
    dbManager.initialize(m_testDbPath, true);
    
    // Verify all tables were created
    const std::vector<std::string> expectedTables = {
        "metadata", "launch_sites", "vehicles", "vehicle_stages",
        "engines", "missions", "mission_configs", "simulation_results",
        "trajectory_data", "events", "user_preferences", "templates"
    };
    
    for (const auto& tableName : expectedTables) {
        SQLiteStatement stmt(dbManager.getDatabase(), 
            "SELECT COUNT(*) FROM sqlite_master WHERE type='table' AND name=?");
        stmt.bind(1, tableName);
        EXPECT_TRUE(stmt.step());
        EXPECT_EQ(stmt.getInt(0), 1) << "Table " << tableName << " not found";
    }
}