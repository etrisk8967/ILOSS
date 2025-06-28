/**
 * @file LaunchSiteDAO.h
 * @brief Data Access Object for LaunchSite entities
 * @author ILOSS Development Team
 * @date 2025
 */

#pragma once

#include "data/database/BaseDAO.h"
#include "data/database/entities/LaunchSite.h"

namespace iloss {
namespace data {
namespace database {

/**
 * @brief Data Access Object for LaunchSite entities
 */
class LaunchSiteDAO : public BaseDAO<entities::LaunchSite> {
public:
    LaunchSiteDAO();
    ~LaunchSiteDAO() override = default;

    // Override base methods
    std::optional<EntityPtr> findById(int64_t id) override;
    EntityList findAll() override;
    EntityPtr save(EntityPtr entity) override;

    // LaunchSite-specific queries
    /**
     * @brief Find launch site by code
     * @param code Launch site code (e.g., "KSC", "CCAFS")
     * @return Launch site if found
     */
    std::optional<EntityPtr> findByCode(const std::string& code);

    /**
     * @brief Find launch site by name
     * @param name Launch site name
     * @return Launch site if found
     */
    std::optional<EntityPtr> findByName(const std::string& name);

    /**
     * @brief Find builtin launch sites
     * @return Vector of builtin launch sites
     */
    EntityList findBuiltin();

    /**
     * @brief Find launch sites by country/region
     * @param pattern Pattern to match in description (supports % wildcard)
     * @return Vector of matching launch sites
     */
    EntityList findByRegion(const std::string& pattern);

    /**
     * @brief Find launch sites within a geographic area
     * @param minLat Minimum latitude
     * @param maxLat Maximum latitude
     * @param minLon Minimum longitude
     * @param maxLon Maximum longitude
     * @return Vector of launch sites within the area
     */
    EntityList findByArea(double minLat, double maxLat, double minLon, double maxLon);

    /**
     * @brief Find launch sites that can support a specific azimuth
     * @param azimuth Launch azimuth in degrees
     * @return Vector of compatible launch sites
     */
    EntityList findByAzimuthCapability(double azimuth);

    /**
     * @brief Check if a launch site code exists
     * @param code Launch site code
     * @return True if exists, false otherwise
     */
    bool codeExists(const std::string& code);

    /**
     * @brief Clone a launch site
     * @param siteId Launch site ID to clone
     * @param newName Name for the cloned site
     * @param newCode Code for the cloned site
     * @return Cloned launch site
     */
    EntityPtr cloneLaunchSite(int64_t siteId, const std::string& newName, const std::string& newCode);

protected:
    void bindEntity(const entities::LaunchSite& entity, db::SQLiteStatement& stmt) override;
    EntityPtr extractEntity(db::SQLiteStatement& stmt) override;
};

} // namespace database
} // namespace data
} // namespace iloss