/**
 * @file test_MissionDAO.cpp
 * @brief Unit tests for MissionDAO class
 * @author ILOSS Development Team
 * @date 2025
 */

#include <gtest/gtest.h>
#include "data/database/MissionDAO.h"
#include "data/database/DatabaseManager.h"
#include <filesystem>
#include <chrono>

using namespace iloss::data::database;
using namespace iloss::data::database::entities;

class MissionDAOTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a temporary database file for testing
        m_testDbPath = "test_mission_dao_" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".db";
        
        // Initialize database
        auto& dbManager = DatabaseManager::getInstance();
        dbManager.initialize(m_testDbPath, true);
        
        // Create test data
        createTestData();
        
        m_dao = std::make_unique<MissionDAO>();
    }

    void TearDown() override {
        m_dao.reset();
        DatabaseManager::getInstance().close();
        std::filesystem::remove(m_testDbPath);
    }
    
    void createTestData() {
        auto& dbManager = DatabaseManager::getInstance();
        
        // Insert test launch sites
        dbManager.execute(
            "INSERT INTO launch_sites (id, name, code, latitude, longitude, elevation) "
            "VALUES (1, 'Test Site 1', 'TS1', 28.5, -80.5, 10)"
        );
        dbManager.execute(
            "INSERT INTO launch_sites (id, name, code, latitude, longitude, elevation) "
            "VALUES (2, 'Test Site 2', 'TS2', 34.5, -120.5, 100)"
        );
        
        // Insert test vehicles
        dbManager.execute(
            "INSERT INTO vehicles (id, name, manufacturer, type) "
            "VALUES (1, 'Test Vehicle 1', 'Test Corp', 'Medium')"
        );
        dbManager.execute(
            "INSERT INTO vehicles (id, name, manufacturer, type) "
            "VALUES (2, 'Test Vehicle 2', 'Test Corp', 'Heavy')"
        );
    }
    
    std::shared_ptr<Mission> createTestMission(const std::string& name) {
        auto mission = std::make_shared<Mission>();
        mission->setName(name);
        mission->setDescription("Test mission description");
        mission->setLaunchSiteId(1);
        mission->setVehicleId(1);
        mission->setLaunchTime("2025-01-01T12:00:00Z");
        mission->setMissionType("LEO");
        mission->setStatus(Mission::Status::Planned);
        return mission;
    }

    std::string m_testDbPath;
    std::unique_ptr<MissionDAO> m_dao;
};

TEST_F(MissionDAOTest, SaveNewMission) {
    auto mission = createTestMission("Test Mission 1");
    
    EXPECT_EQ(mission->getId(), 0);
    
    auto savedMission = m_dao->save(mission);
    
    EXPECT_NE(savedMission->getId(), 0);
    EXPECT_EQ(savedMission->getName(), "Test Mission 1");
    EXPECT_FALSE(savedMission->getCreatedAt().empty());
    EXPECT_FALSE(savedMission->getUpdatedAt().empty());
}

TEST_F(MissionDAOTest, UpdateExistingMission) {
    auto mission = createTestMission("Original Name");
    auto savedMission = m_dao->save(mission);
    
    savedMission->setName("Updated Name");
    savedMission->setStatus(Mission::Status::Completed);
    
    auto updatedMission = m_dao->save(savedMission);
    
    EXPECT_EQ(updatedMission->getId(), savedMission->getId());
    EXPECT_EQ(updatedMission->getName(), "Updated Name");
    EXPECT_EQ(updatedMission->getStatus(), Mission::Status::Completed);
}

TEST_F(MissionDAOTest, FindById) {
    auto mission = createTestMission("Find By ID Test");
    auto savedMission = m_dao->save(mission);
    
    auto foundMission = m_dao->findById(savedMission->getId());
    
    ASSERT_TRUE(foundMission.has_value());
    EXPECT_EQ((*foundMission)->getId(), savedMission->getId());
    EXPECT_EQ((*foundMission)->getName(), "Find By ID Test");
}

TEST_F(MissionDAOTest, FindByIdNotFound) {
    auto foundMission = m_dao->findById(99999);
    
    EXPECT_FALSE(foundMission.has_value());
}

TEST_F(MissionDAOTest, FindAll) {
    m_dao->save(createTestMission("Mission 1"));
    m_dao->save(createTestMission("Mission 2"));
    m_dao->save(createTestMission("Mission 3"));
    
    auto missions = m_dao->findAll();
    
    EXPECT_EQ(missions.size(), 3);
}

TEST_F(MissionDAOTest, FindByStatus) {
    auto mission1 = createTestMission("Planned Mission");
    mission1->setStatus(Mission::Status::Planned);
    m_dao->save(mission1);
    
    auto mission2 = createTestMission("Completed Mission");
    mission2->setStatus(Mission::Status::Completed);
    m_dao->save(mission2);
    
    auto mission3 = createTestMission("Another Planned");
    mission3->setStatus(Mission::Status::Planned);
    m_dao->save(mission3);
    
    auto plannedMissions = m_dao->findByStatus(Mission::Status::Planned);
    auto completedMissions = m_dao->findByStatus(Mission::Status::Completed);
    
    EXPECT_EQ(plannedMissions.size(), 2);
    EXPECT_EQ(completedMissions.size(), 1);
}

TEST_F(MissionDAOTest, FindByLaunchSite) {
    auto mission1 = createTestMission("Site 1 Mission");
    mission1->setLaunchSiteId(1);
    m_dao->save(mission1);
    
    auto mission2 = createTestMission("Site 2 Mission");
    mission2->setLaunchSiteId(2);
    m_dao->save(mission2);
    
    auto mission3 = createTestMission("Another Site 1");
    mission3->setLaunchSiteId(1);
    m_dao->save(mission3);
    
    auto site1Missions = m_dao->findByLaunchSite(1);
    auto site2Missions = m_dao->findByLaunchSite(2);
    
    EXPECT_EQ(site1Missions.size(), 2);
    EXPECT_EQ(site2Missions.size(), 1);
}

TEST_F(MissionDAOTest, FindByVehicle) {
    auto mission1 = createTestMission("Vehicle 1 Mission");
    mission1->setVehicleId(1);
    m_dao->save(mission1);
    
    auto mission2 = createTestMission("Vehicle 2 Mission");
    mission2->setVehicleId(2);
    m_dao->save(mission2);
    
    auto vehicle1Missions = m_dao->findByVehicle(1);
    auto vehicle2Missions = m_dao->findByVehicle(2);
    
    EXPECT_EQ(vehicle1Missions.size(), 1);
    EXPECT_EQ(vehicle2Missions.size(), 1);
}

TEST_F(MissionDAOTest, FindByTimeRange) {
    auto mission1 = createTestMission("Early Mission");
    mission1->setLaunchTime("2025-01-01T10:00:00Z");
    m_dao->save(mission1);
    
    auto mission2 = createTestMission("Mid Mission");
    mission2->setLaunchTime("2025-06-15T12:00:00Z");
    m_dao->save(mission2);
    
    auto mission3 = createTestMission("Late Mission");
    mission3->setLaunchTime("2025-12-31T23:00:00Z");
    m_dao->save(mission3);
    
    auto q1Missions = m_dao->findByTimeRange("2025-01-01T00:00:00Z", "2025-03-31T23:59:59Z");
    auto yearMissions = m_dao->findByTimeRange("2025-01-01T00:00:00Z", "2025-12-31T23:59:59Z");
    
    EXPECT_EQ(q1Missions.size(), 1);
    EXPECT_EQ(yearMissions.size(), 3);
}

TEST_F(MissionDAOTest, FindByNamePattern) {
    m_dao->save(createTestMission("Alpha Mission"));
    m_dao->save(createTestMission("Beta Test"));
    m_dao->save(createTestMission("Gamma Mission"));
    
    auto missionPattern = m_dao->findByNamePattern("%Mission%");
    auto testPattern = m_dao->findByNamePattern("%Test%");
    auto alphaPattern = m_dao->findByNamePattern("Alpha%");
    
    EXPECT_EQ(missionPattern.size(), 2);
    EXPECT_EQ(testPattern.size(), 1);
    EXPECT_EQ(alphaPattern.size(), 1);
}

TEST_F(MissionDAOTest, DeleteById) {
    auto mission = m_dao->save(createTestMission("To Delete"));
    
    bool deleted = m_dao->deleteById(mission->getId());
    EXPECT_TRUE(deleted);
    
    auto found = m_dao->findById(mission->getId());
    EXPECT_FALSE(found.has_value());
    
    // Try deleting non-existent
    bool deletedAgain = m_dao->deleteById(mission->getId());
    EXPECT_FALSE(deletedAgain);
}

TEST_F(MissionDAOTest, Count) {
    EXPECT_EQ(m_dao->count(), 0);
    
    m_dao->save(createTestMission("Mission 1"));
    m_dao->save(createTestMission("Mission 2"));
    
    EXPECT_EQ(m_dao->count(), 2);
}

TEST_F(MissionDAOTest, Exists) {
    auto mission = m_dao->save(createTestMission("Exists Test"));
    
    EXPECT_TRUE(m_dao->exists(mission->getId()));
    EXPECT_FALSE(m_dao->exists(99999));
}

TEST_F(MissionDAOTest, UpdateStatus) {
    auto mission = m_dao->save(createTestMission("Status Update"));
    
    bool updated = m_dao->updateStatus(mission->getId(), Mission::Status::Simulating);
    EXPECT_TRUE(updated);
    
    auto updatedMission = m_dao->findById(mission->getId());
    ASSERT_TRUE(updatedMission.has_value());
    EXPECT_EQ((*updatedMission)->getStatus(), Mission::Status::Simulating);
}

TEST_F(MissionDAOTest, GetRecentMissions) {
    // Create missions with unique IDs to track order
    std::vector<int64_t> missionIds;
    for (int i = 1; i <= 5; ++i) {
        auto mission = m_dao->save(createTestMission("Mission " + std::to_string(i)));
        missionIds.push_back(mission->getId());
        // Small delay to ensure different timestamps
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    auto recent3 = m_dao->getRecentMissions(3);
    auto recentAll = m_dao->getRecentMissions(10);
    
    EXPECT_EQ(recent3.size(), 3);
    EXPECT_EQ(recentAll.size(), 5);
    
    // Verify that results are sorted by ID descending (most recent first)
    // Since IDs are autoincrementing, higher ID = more recent
    for (size_t i = 1; i < recent3.size(); ++i) {
        EXPECT_GT(recent3[i-1]->getId(), recent3[i]->getId());
    }
    
    // Verify we got the most recent 3 missions
    EXPECT_EQ(recent3[0]->getId(), missionIds[4]); // Mission 5
    EXPECT_EQ(recent3[1]->getId(), missionIds[3]); // Mission 4
    EXPECT_EQ(recent3[2]->getId(), missionIds[2]); // Mission 3
}

TEST_F(MissionDAOTest, CloneMission) {
    auto original = createTestMission("Original Mission");
    original->setDescription("Original description");
    original = m_dao->save(original);
    
    auto cloned = m_dao->cloneMission(original->getId(), "Cloned Mission");
    
    EXPECT_NE(cloned->getId(), original->getId());
    EXPECT_EQ(cloned->getName(), "Cloned Mission");
    EXPECT_EQ(cloned->getDescription(), original->getDescription());
    EXPECT_EQ(cloned->getStatus(), Mission::Status::Planned);
}

TEST_F(MissionDAOTest, MissionConfigSaveAndLoad) {
    auto mission = m_dao->save(createTestMission("Config Test"));
    
    auto config = std::make_shared<MissionConfig>();
    config->setMissionId(mission->getId());
    config->setLaunchAzimuth(90.0);
    config->setTargetAltitude(400000.0);
    config->setTargetInclination(51.6);
    config->setTargetEccentricity(0.0);
    config->setPayloadMass(1000.0);
    config->setSimulationDuration(86400.0);
    config->setTimestep(0.1);
    config->setIntegratorType("RK4");
    config->setForceModels("gravity,drag,thirdbody");
    config->setConfigJson("{\"test\": true}");
    
    auto savedConfig = m_dao->saveMissionConfig(config);
    
    EXPECT_NE(savedConfig->getId(), 0);
    
    auto loadedConfig = m_dao->getMissionConfig(mission->getId());
    
    ASSERT_TRUE(loadedConfig.has_value());
    EXPECT_EQ((*loadedConfig)->getMissionId(), mission->getId());
    EXPECT_EQ((*loadedConfig)->getLaunchAzimuth(), 90.0);
    EXPECT_EQ((*loadedConfig)->getTargetAltitude(), 400000.0);
    EXPECT_EQ((*loadedConfig)->getConfigJson(), "{\"test\": true}");
}

TEST_F(MissionDAOTest, MissionConfigUpdate) {
    auto mission = m_dao->save(createTestMission("Config Update Test"));
    
    auto config = std::make_shared<MissionConfig>();
    config->setMissionId(mission->getId());
    config->setLaunchAzimuth(90.0);
    
    auto savedConfig = m_dao->saveMissionConfig(config);
    
    savedConfig->setLaunchAzimuth(120.0);
    savedConfig->setTargetAltitude(500000.0);
    
    auto updatedConfig = m_dao->saveMissionConfig(savedConfig);
    
    EXPECT_EQ(updatedConfig->getId(), savedConfig->getId());
    EXPECT_EQ(updatedConfig->getLaunchAzimuth(), 120.0);
    EXPECT_EQ(updatedConfig->getTargetAltitude(), 500000.0);
}

TEST_F(MissionDAOTest, DeleteMissionConfig) {
    auto mission = m_dao->save(createTestMission("Config Delete Test"));
    
    auto config = std::make_shared<MissionConfig>();
    config->setMissionId(mission->getId());
    m_dao->saveMissionConfig(config);
    
    bool deleted = m_dao->deleteMissionConfig(mission->getId());
    EXPECT_TRUE(deleted);
    
    auto loadedConfig = m_dao->getMissionConfig(mission->getId());
    EXPECT_FALSE(loadedConfig.has_value());
}

TEST_F(MissionDAOTest, CloneMissionWithConfig) {
    auto original = m_dao->save(createTestMission("Original with Config"));
    
    auto config = std::make_shared<MissionConfig>();
    config->setMissionId(original->getId());
    config->setLaunchAzimuth(90.0);
    config->setTargetAltitude(400000.0);
    m_dao->saveMissionConfig(config);
    
    auto cloned = m_dao->cloneMission(original->getId(), "Cloned with Config");
    
    auto clonedConfig = m_dao->getMissionConfig(cloned->getId());
    
    ASSERT_TRUE(clonedConfig.has_value());
    EXPECT_EQ((*clonedConfig)->getLaunchAzimuth(), 90.0);
    EXPECT_EQ((*clonedConfig)->getTargetAltitude(), 400000.0);
}

TEST_F(MissionDAOTest, NullHandling) {
    auto mission = createTestMission("Null Test");
    mission->setLaunchSiteId(std::nullopt);
    mission->setVehicleId(std::nullopt);
    
    auto saved = m_dao->save(mission);
    
    auto loaded = m_dao->findById(saved->getId());
    
    ASSERT_TRUE(loaded.has_value());
    EXPECT_FALSE((*loaded)->getLaunchSiteId().has_value());
    EXPECT_FALSE((*loaded)->getVehicleId().has_value());
}