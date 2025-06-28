/**
 * @file QueryBuilder.h
 * @brief SQL query builder for type-safe query construction
 * @author ILOSS Development Team
 * @date 2025
 * 
 * This class provides a fluent interface for building SQL queries
 * in a type-safe manner, avoiding SQL injection vulnerabilities.
 */

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <sstream>
#include <variant>

namespace iloss {
namespace data {
namespace database {

/**
 * @brief Value type that can be bound to a query
 */
using QueryValue = std::variant<int, int64_t, double, std::string, std::nullptr_t>;

/**
 * @brief Represents a condition in a WHERE clause
 */
struct Condition {
    std::string column;
    std::string op;
    QueryValue value;
    
    Condition(const std::string& col, const std::string& operation, const QueryValue& val)
        : column(col), op(operation), value(val) {}
};

/**
 * @brief Represents a JOIN clause
 */
struct JoinClause {
    enum Type { INNER, LEFT, RIGHT, FULL };
    Type type;
    std::string table;
    std::string on;
    
    JoinClause(Type t, const std::string& tbl, const std::string& condition)
        : type(t), table(tbl), on(condition) {}
};

/**
 * @brief SQL query builder class
 * 
 * Provides a fluent interface for building SELECT, INSERT, UPDATE, and DELETE queries.
 */
class QueryBuilder {
public:
    enum class QueryType { SELECT, INSERT, UPDATE, DELETE };

    QueryBuilder() = default;
    ~QueryBuilder() = default;

    // SELECT operations
    static QueryBuilder select(const std::vector<std::string>& columns = {});
    QueryBuilder& from(const std::string& table);
    QueryBuilder& distinct();
    
    // INSERT operations
    static QueryBuilder insertInto(const std::string& table);
    QueryBuilder& columns(const std::vector<std::string>& cols);
    QueryBuilder& values(const std::vector<QueryValue>& vals);
    QueryBuilder& valuesMultiple(const std::vector<std::vector<QueryValue>>& rows);
    
    // UPDATE operations
    static QueryBuilder update(const std::string& table);
    QueryBuilder& set(const std::string& column, const QueryValue& value);
    QueryBuilder& set(const std::vector<std::pair<std::string, QueryValue>>& assignments);
    
    // DELETE operations
    static QueryBuilder deleteFrom(const std::string& table);
    
    // WHERE conditions
    QueryBuilder& where(const std::string& column, const std::string& op, const QueryValue& value);
    QueryBuilder& where(const Condition& condition);
    QueryBuilder& andWhere(const std::string& column, const std::string& op, const QueryValue& value);
    QueryBuilder& orWhere(const std::string& column, const std::string& op, const QueryValue& value);
    QueryBuilder& whereIn(const std::string& column, const std::vector<QueryValue>& values);
    QueryBuilder& whereNotIn(const std::string& column, const std::vector<QueryValue>& values);
    QueryBuilder& whereBetween(const std::string& column, const QueryValue& min, const QueryValue& max);
    QueryBuilder& whereNull(const std::string& column);
    QueryBuilder& whereNotNull(const std::string& column);
    QueryBuilder& whereLike(const std::string& column, const std::string& pattern);
    
    // JOIN operations
    QueryBuilder& join(const std::string& table, const std::string& on);
    QueryBuilder& leftJoin(const std::string& table, const std::string& on);
    QueryBuilder& rightJoin(const std::string& table, const std::string& on);
    QueryBuilder& fullJoin(const std::string& table, const std::string& on);
    
    // GROUP BY and HAVING
    QueryBuilder& groupBy(const std::vector<std::string>& columns);
    QueryBuilder& having(const std::string& column, const std::string& op, const QueryValue& value);
    
    // ORDER BY
    QueryBuilder& orderBy(const std::string& column, bool ascending = true);
    QueryBuilder& orderBy(const std::vector<std::pair<std::string, bool>>& orders);
    
    // LIMIT and OFFSET
    QueryBuilder& limit(int count);
    QueryBuilder& offset(int count);
    
    // Build the query
    std::string build() const;
    std::vector<QueryValue> getBindValues() const { return m_bindValues; }
    
    // Utility methods
    QueryBuilder& raw(const std::string& rawSql);
    void reset();

    // Static helper methods
    static std::string escape(const std::string& str);
    static std::string quote(const std::string& identifier);

private:
    QueryType m_type = QueryType::SELECT;
    std::vector<std::string> m_selectColumns;
    std::string m_table;
    bool m_distinct = false;
    
    // For INSERT
    std::vector<std::string> m_insertColumns;
    std::vector<std::vector<QueryValue>> m_insertValues;
    
    // For UPDATE
    std::vector<std::pair<std::string, QueryValue>> m_updateSets;
    
    // WHERE conditions
    std::vector<std::tuple<std::string, Condition>> m_whereConditions; // prefix, condition
    
    // JOINs
    std::vector<JoinClause> m_joins;
    
    // GROUP BY and HAVING
    std::vector<std::string> m_groupByColumns;
    std::vector<Condition> m_havingConditions;
    
    // ORDER BY
    std::vector<std::pair<std::string, bool>> m_orderByColumns;
    
    // LIMIT and OFFSET
    int m_limit = -1;
    int m_offset = -1;
    
    // Raw SQL
    std::string m_rawSql;
    
    // Bind values for prepared statements
    mutable std::vector<QueryValue> m_bindValues;
    
    // Helper methods
    std::string buildSelect() const;
    std::string buildInsert() const;
    std::string buildUpdate() const;
    std::string buildDelete() const;
    std::string buildWhere() const;
    std::string buildJoins() const;
    std::string buildGroupBy() const;
    std::string buildOrderBy() const;
    std::string buildLimit() const;
    
    std::string formatValue(const QueryValue& value) const;
    void addBindValue(const QueryValue& value) const;
};

/**
 * @brief Helper class for building complex expressions
 */
class Expression {
public:
    static std::string count(const std::string& column = "*");
    static std::string sum(const std::string& column);
    static std::string avg(const std::string& column);
    static std::string min(const std::string& column);
    static std::string max(const std::string& column);
    static std::string concat(const std::vector<std::string>& parts);
    static std::string coalesce(const std::vector<std::string>& expressions);
    static std::string cast(const std::string& expression, const std::string& type);
};

} // namespace database
} // namespace data
} // namespace iloss