/**
 * @file MissionDAO.h
 * @brief Data Access Object for Mission entities
 * @author ILOSS Development Team
 * @date 2025
 */

#pragma once

#include "data/database/BaseDAO.h"
#include "data/database/entities/Mission.h"
#include <vector>

namespace iloss {
namespace data {
namespace database {

/**
 * @brief Data Access Object for Mission entities
 */
class MissionDAO : public BaseDAO<entities::Mission> {
public:
    MissionDAO();
    ~MissionDAO() override = default;

    // Override base methods
    std::optional<EntityPtr> findById(int64_t id) override;
    EntityList findAll() override;
    EntityPtr save(EntityPtr entity) override;

    // Mission-specific queries
    /**
     * @brief Find missions by status
     * @param status Mission status
     * @return Vector of missions with the given status
     */
    EntityList findByStatus(entities::Mission::Status status);

    /**
     * @brief Find missions by launch site
     * @param launchSiteId Launch site ID
     * @return Vector of missions for the given launch site
     */
    EntityList findByLaunchSite(int64_t launchSiteId);

    /**
     * @brief Find missions by vehicle
     * @param vehicleId Vehicle ID
     * @return Vector of missions using the given vehicle
     */
    EntityList findByVehicle(int64_t vehicleId);

    /**
     * @brief Find missions within a time range
     * @param startTime Start time (ISO 8601)
     * @param endTime End time (ISO 8601)
     * @return Vector of missions within the time range
     */
    EntityList findByTimeRange(const std::string& startTime, const std::string& endTime);

    /**
     * @brief Find missions by name pattern
     * @param pattern Name pattern (supports % wildcard)
     * @return Vector of matching missions
     */
    EntityList findByNamePattern(const std::string& pattern);

    /**
     * @brief Get mission configuration
     * @param missionId Mission ID
     * @return Mission configuration if exists
     */
    std::optional<std::shared_ptr<entities::MissionConfig>> getMissionConfig(int64_t missionId);

    /**
     * @brief Save mission configuration
     * @param config Mission configuration to save
     * @return Saved configuration with updated ID
     */
    std::shared_ptr<entities::MissionConfig> saveMissionConfig(std::shared_ptr<entities::MissionConfig> config);

    /**
     * @brief Delete mission configuration
     * @param missionId Mission ID
     * @return True if deleted, false if not found
     */
    bool deleteMissionConfig(int64_t missionId);

    /**
     * @brief Update mission status
     * @param missionId Mission ID
     * @param status New status
     * @return True if updated, false if not found
     */
    bool updateStatus(int64_t missionId, entities::Mission::Status status);

    /**
     * @brief Get recent missions
     * @param limit Number of missions to return
     * @return Vector of recent missions ordered by creation date
     */
    EntityList getRecentMissions(int limit = 10);

    /**
     * @brief Clone a mission
     * @param missionId Mission ID to clone
     * @param newName Name for the cloned mission
     * @return Cloned mission
     */
    EntityPtr cloneMission(int64_t missionId, const std::string& newName);

protected:
    void bindEntity(const entities::Mission& entity, db::SQLiteStatement& stmt) override;
    EntityPtr extractEntity(db::SQLiteStatement& stmt) override;

private:
    /**
     * @brief Bind mission configuration to statement
     * @param config Mission configuration
     * @param stmt SQL statement
     */
    void bindMissionConfig(const entities::MissionConfig& config, db::SQLiteStatement& stmt);

    /**
     * @brief Extract mission configuration from statement
     * @param stmt SQL statement
     * @return Mission configuration
     */
    std::shared_ptr<entities::MissionConfig> extractMissionConfig(db::SQLiteStatement& stmt);
};

} // namespace database
} // namespace data
} // namespace iloss