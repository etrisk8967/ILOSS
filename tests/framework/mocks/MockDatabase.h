#pragma once

#include "core/external/TestWrapper.h"
#include "data/database/Database.h"
#include <unordered_map>
#include <queue>

namespace iloss {
namespace test {

/**
 * @brief Mock database connection for testing
 */
class MockDatabaseConnection : public data::IConnection {
public:
    MockDatabaseConnection() = default;
    
    void execute(const std::string& sql) override {
        m_executedQueries.push_back(sql);
        
        if (m_shouldThrowOnExecute) {
            throw std::runtime_error("Mock execute error: " + m_executeError);
        }
        
        if (m_executeCallback) {
            m_executeCallback(sql);
        }
    }
    
    std::unique_ptr<data::IResultSet> query(const std::string& sql) override {
        m_executedQueries.push_back(sql);
        
        if (m_shouldThrowOnQuery) {
            throw std::runtime_error("Mock query error: " + m_queryError);
        }
        
        if (m_queryCallback) {
            return m_queryCallback(sql);
        }
        
        // Return configured result or empty result set
        auto it = m_queryResults.find(sql);
        if (it != m_queryResults.end() && !it->second.empty()) {
            auto result = std::move(it->second.front());
            it->second.pop();
            return result;
        }
        
        return std::make_unique<MockResultSet>();
    }
    
    std::unique_ptr<data::IStatement> prepare(const std::string& sql) override {
        m_preparedStatements.push_back(sql);
        
        if (m_shouldThrowOnPrepare) {
            throw std::runtime_error("Mock prepare error: " + m_prepareError);
        }
        
        return std::make_unique<MockStatement>(this, sql);
    }
    
    void beginTransaction() override {
        m_transactionCount++;
        m_inTransaction = true;
    }
    
    void commit() override {
        if (!m_inTransaction) {
            throw std::runtime_error("No active transaction");
        }
        m_commitCount++;
        m_inTransaction = false;
    }
    
    void rollback() override {
        if (!m_inTransaction) {
            throw std::runtime_error("No active transaction");
        }
        m_rollbackCount++;
        m_inTransaction = false;
    }
    
    bool isValid() const override {
        return m_isValid;
    }
    
    // Test utilities
    void setValid(bool valid) { m_isValid = valid; }
    void setShouldThrowOnExecute(bool shouldThrow, const std::string& error = "Execute failed") {
        m_shouldThrowOnExecute = shouldThrow;
        m_executeError = error;
    }
    void setShouldThrowOnQuery(bool shouldThrow, const std::string& error = "Query failed") {
        m_shouldThrowOnQuery = shouldThrow;
        m_queryError = error;
    }
    void setShouldThrowOnPrepare(bool shouldThrow, const std::string& error = "Prepare failed") {
        m_shouldThrowOnPrepare = shouldThrow;
        m_prepareError = error;
    }
    
    void addQueryResult(const std::string& sql, std::unique_ptr<data::IResultSet> result) {
        m_queryResults[sql].push(std::move(result));
    }
    
    const std::vector<std::string>& getExecutedQueries() const { return m_executedQueries; }
    const std::vector<std::string>& getPreparedStatements() const { return m_preparedStatements; }
    
    int getTransactionCount() const { return m_transactionCount; }
    int getCommitCount() const { return m_commitCount; }
    int getRollbackCount() const { return m_rollbackCount; }
    bool isInTransaction() const { return m_inTransaction; }
    
    void clearHistory() {
        m_executedQueries.clear();
        m_preparedStatements.clear();
    }
    
    // Set callbacks
    void setExecuteCallback(std::function<void(const std::string&)> callback) {
        m_executeCallback = callback;
    }
    
    void setQueryCallback(std::function<std::unique_ptr<data::IResultSet>(const std::string&)> callback) {
        m_queryCallback = callback;
    }
    
private:
    // Mock result set implementation
    class MockResultSet : public data::IResultSet {
    public:
        bool next() override {
            if (m_currentRow < m_rows.size() - 1) {
                m_currentRow++;
                return true;
            }
            return false;
        }
        
        int getInt(int column) const override {
            if (m_currentRow >= 0 && m_currentRow < m_rows.size() &&
                column >= 0 && column < m_rows[m_currentRow].size()) {
                return std::get<int>(m_rows[m_currentRow][column]);
            }
            throw std::out_of_range("Invalid row/column");
        }
        
        double getDouble(int column) const override {
            if (m_currentRow >= 0 && m_currentRow < m_rows.size() &&
                column >= 0 && column < m_rows[m_currentRow].size()) {
                return std::get<double>(m_rows[m_currentRow][column]);
            }
            throw std::out_of_range("Invalid row/column");
        }
        
        std::string getString(int column) const override {
            if (m_currentRow >= 0 && m_currentRow < m_rows.size() &&
                column >= 0 && column < m_rows[m_currentRow].size()) {
                return std::get<std::string>(m_rows[m_currentRow][column]);
            }
            throw std::out_of_range("Invalid row/column");
        }
        
        bool isNull(int column) const override {
            if (m_currentRow >= 0 && m_currentRow < m_rows.size() &&
                column >= 0 && column < m_rows[m_currentRow].size()) {
                return std::holds_alternative<std::monostate>(m_rows[m_currentRow][column]);
            }
            return true;
        }
        
        // Test utilities
        void addRow(std::vector<std::variant<std::monostate, int, double, std::string>> row) {
            m_rows.push_back(std::move(row));
        }
        
    private:
        std::vector<std::vector<std::variant<std::monostate, int, double, std::string>>> m_rows;
        int m_currentRow = -1;
    };
    
    // Mock statement implementation
    class MockStatement : public data::IStatement {
    public:
        MockStatement(MockDatabaseConnection* conn, const std::string& sql)
            : m_connection(conn), m_sql(sql) {}
        
        void bind(int param, int value) override {
            m_bindings[param] = value;
        }
        
        void bind(int param, double value) override {
            m_bindings[param] = value;
        }
        
        void bind(int param, const std::string& value) override {
            m_bindings[param] = value;
        }
        
        void bindNull(int param) override {
            m_bindings[param] = std::monostate{};
        }
        
        void execute() override {
            m_connection->execute(m_sql + " [with bindings]");
        }
        
        std::unique_ptr<data::IResultSet> query() override {
            return m_connection->query(m_sql + " [with bindings]");
        }
        
        void reset() override {
            m_bindings.clear();
        }
        
    private:
        MockDatabaseConnection* m_connection;
        std::string m_sql;
        std::unordered_map<int, std::variant<std::monostate, int, double, std::string>> m_bindings;
    };
    
    bool m_isValid = true;
    bool m_shouldThrowOnExecute = false;
    bool m_shouldThrowOnQuery = false;
    bool m_shouldThrowOnPrepare = false;
    std::string m_executeError;
    std::string m_queryError;
    std::string m_prepareError;
    
    std::vector<std::string> m_executedQueries;
    std::vector<std::string> m_preparedStatements;
    
    int m_transactionCount = 0;
    int m_commitCount = 0;
    int m_rollbackCount = 0;
    bool m_inTransaction = false;
    
    std::unordered_map<std::string, std::queue<std::unique_ptr<data::IResultSet>>> m_queryResults;
    
    std::function<void(const std::string&)> m_executeCallback;
    std::function<std::unique_ptr<data::IResultSet>(const std::string&)> m_queryCallback;
};

} // namespace test
} // namespace iloss