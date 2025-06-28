#include "TestFixtureBase.h"

namespace iloss {
namespace test {

void DatabaseTestFixture::executeSQL(const std::string& sql) {
    // TODO: Implement when DatabaseManager is available
    (void)sql;
    (void)m_dbPathStr;
    ADD_FAILURE() << "DatabaseTestFixture::executeSQL not implemented yet";
}

bool DatabaseTestFixture::tableExists(const std::string& tableName) {
    // TODO: Implement when DatabaseManager is available
    (void)tableName;
    return false;
}

int DatabaseTestFixture::getRowCount(const std::string& tableName) {
    // TODO: Implement when DatabaseManager is available
    (void)tableName;
    return -1;
}

} // namespace test
} // namespace iloss