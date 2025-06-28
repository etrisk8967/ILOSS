/**
 * @file MissionDAO.cpp
 * @brief Implementation of Mission Data Access Object
 * @author ILOSS Development Team
 * @date 2025
 */

#include "data/database/MissionDAO.h"
// #include "core/logging/Logger.h"  // TODO: Re-enable logging after fixing initialization

namespace iloss {
namespace data {
namespace database {

MissionDAO::MissionDAO() : BaseDAO<entities::Mission>("missions") {
}

std::optional<MissionDAO::EntityPtr> MissionDAO::findById(int64_t id) {
    const std::string sql = 
        "SELECT id, name, description, launch_site_id, vehicle_id, launch_time, "
        "mission_type, status, created_at, updated_at "
        "FROM missions WHERE id = ?";
    
    return executeQuerySingle(sql, [id](db::SQLiteStatement& stmt) {
        stmt.bind(1, id);
    });
}

MissionDAO::EntityList MissionDAO::findAll() {
    const std::string sql = 
        "SELECT id, name, description, launch_site_id, vehicle_id, launch_time, "
        "mission_type, status, created_at, updated_at "
        "FROM missions ORDER BY created_at DESC";
    
    return executeQuery(sql);
}

MissionDAO::EntityPtr MissionDAO::save(EntityPtr entity) {
    if (!entity) {
        throw std::invalid_argument("Cannot save null mission");
    }

    DatabaseTransaction transaction(getDatabaseManager());

    try {
        if (entity->getId() == 0) {
            // Insert new mission
            const std::string sql = 
                "INSERT INTO missions (name, description, launch_site_id, vehicle_id, "
                "launch_time, mission_type, status, created_at, updated_at) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)";
            
            db::SQLiteStatement stmt(getDatabase(), sql);
            bindEntity(*entity, stmt);
            stmt.execute();
            
            entity->setId(getDatabase().lastInsertRowId());
            entity->setCreatedAt(getCurrentTimestamp());
            entity->setUpdatedAt(getCurrentTimestamp());
        } else {
            // Update existing mission
            const std::string sql = 
                "UPDATE missions SET name = ?, description = ?, launch_site_id = ?, "
                "vehicle_id = ?, launch_time = ?, mission_type = ?, status = ?, "
                "updated_at = ? WHERE id = ?";
            
            db::SQLiteStatement stmt(getDatabase(), sql);
            bindEntity(*entity, stmt);
            stmt.bind(9, entity->getId());
            stmt.execute();
            
            entity->setUpdatedAt(getCurrentTimestamp());
        }

        transaction.commit();
        return entity;
    } catch (const std::exception& e) {
        // LOG_ERROR("Failed to save mission: {}", e.what());
        throw;
    }
}

MissionDAO::EntityList MissionDAO::findByStatus(entities::Mission::Status status) {
    const std::string sql = 
        "SELECT id, name, description, launch_site_id, vehicle_id, launch_time, "
        "mission_type, status, created_at, updated_at "
        "FROM missions WHERE status = ? ORDER BY launch_time";
    
    return executeQuery(sql, [status](db::SQLiteStatement& stmt) {
        stmt.bind(1, entities::Mission::statusToString(status));
    });
}

MissionDAO::EntityList MissionDAO::findByLaunchSite(int64_t launchSiteId) {
    const std::string sql = 
        "SELECT id, name, description, launch_site_id, vehicle_id, launch_time, "
        "mission_type, status, created_at, updated_at "
        "FROM missions WHERE launch_site_id = ? ORDER BY launch_time";
    
    return executeQuery(sql, [launchSiteId](db::SQLiteStatement& stmt) {
        stmt.bind(1, launchSiteId);
    });
}

MissionDAO::EntityList MissionDAO::findByVehicle(int64_t vehicleId) {
    const std::string sql = 
        "SELECT id, name, description, launch_site_id, vehicle_id, launch_time, "
        "mission_type, status, created_at, updated_at "
        "FROM missions WHERE vehicle_id = ? ORDER BY launch_time";
    
    return executeQuery(sql, [vehicleId](db::SQLiteStatement& stmt) {
        stmt.bind(1, vehicleId);
    });
}

MissionDAO::EntityList MissionDAO::findByTimeRange(const std::string& startTime, const std::string& endTime) {
    const std::string sql = 
        "SELECT id, name, description, launch_site_id, vehicle_id, launch_time, "
        "mission_type, status, created_at, updated_at "
        "FROM missions WHERE launch_time BETWEEN ? AND ? ORDER BY launch_time";
    
    return executeQuery(sql, [&startTime, &endTime](db::SQLiteStatement& stmt) {
        stmt.bind(1, startTime);
        stmt.bind(2, endTime);
    });
}

MissionDAO::EntityList MissionDAO::findByNamePattern(const std::string& pattern) {
    const std::string sql = 
        "SELECT id, name, description, launch_site_id, vehicle_id, launch_time, "
        "mission_type, status, created_at, updated_at "
        "FROM missions WHERE name LIKE ? ORDER BY name";
    
    return executeQuery(sql, [&pattern](db::SQLiteStatement& stmt) {
        stmt.bind(1, pattern);
    });
}

std::optional<std::shared_ptr<entities::MissionConfig>> MissionDAO::getMissionConfig(int64_t missionId) {
    const std::string sql = 
        "SELECT id, mission_id, launch_azimuth, target_altitude, target_inclination, "
        "target_eccentricity, payload_mass, simulation_duration, timestep, "
        "integrator_type, force_models, config_json, created_at, updated_at "
        "FROM mission_configs WHERE mission_id = ?";
    
    db::SQLiteStatement stmt(getDatabase(), sql);
    stmt.bind(1, missionId);
    
    if (stmt.step()) {
        return extractMissionConfig(stmt);
    }
    return std::nullopt;
}

std::shared_ptr<entities::MissionConfig> MissionDAO::saveMissionConfig(std::shared_ptr<entities::MissionConfig> config) {
    if (!config) {
        throw std::invalid_argument("Cannot save null mission config");
    }

    DatabaseTransaction transaction(getDatabaseManager());

    try {
        if (config->getId() == 0) {
            // Insert new config
            const std::string sql = 
                "INSERT INTO mission_configs (mission_id, launch_azimuth, target_altitude, "
                "target_inclination, target_eccentricity, payload_mass, simulation_duration, "
                "timestep, integrator_type, force_models, config_json, created_at, updated_at) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)";
            
            db::SQLiteStatement stmt(getDatabase(), sql);
            bindMissionConfig(*config, stmt);
            stmt.execute();
            
            config->setId(getDatabase().lastInsertRowId());
            config->setCreatedAt(getCurrentTimestamp());
            config->setUpdatedAt(getCurrentTimestamp());
        } else {
            // Update existing config
            const std::string sql = 
                "UPDATE mission_configs SET mission_id = ?, launch_azimuth = ?, "
                "target_altitude = ?, target_inclination = ?, target_eccentricity = ?, "
                "payload_mass = ?, simulation_duration = ?, timestep = ?, "
                "integrator_type = ?, force_models = ?, config_json = ?, updated_at = ? "
                "WHERE id = ?";
            
            db::SQLiteStatement stmt(getDatabase(), sql);
            bindMissionConfig(*config, stmt);
            stmt.bind(13, config->getId());
            stmt.execute();
            
            config->setUpdatedAt(getCurrentTimestamp());
        }

        transaction.commit();
        return config;
    } catch (const std::exception& e) {
        // LOG_ERROR("Failed to save mission config: {}", e.what());
        throw;
    }
}

bool MissionDAO::deleteMissionConfig(int64_t missionId) {
    const std::string sql = "DELETE FROM mission_configs WHERE mission_id = ?";
    db::SQLiteStatement stmt(getDatabase(), sql);
    stmt.bind(1, missionId);
    stmt.execute();
    return getDatabase().changesCount() > 0;
}

bool MissionDAO::updateStatus(int64_t missionId, entities::Mission::Status status) {
    const std::string sql = "UPDATE missions SET status = ?, updated_at = ? WHERE id = ?";
    db::SQLiteStatement stmt(getDatabase(), sql);
    stmt.bind(1, entities::Mission::statusToString(status));
    stmt.bind(2, getCurrentTimestamp());
    stmt.bind(3, missionId);
    stmt.execute();
    return getDatabase().changesCount() > 0;
}

MissionDAO::EntityList MissionDAO::getRecentMissions(int limit) {
    const std::string sql = 
        "SELECT id, name, description, launch_site_id, vehicle_id, launch_time, "
        "mission_type, status, created_at, updated_at "
        "FROM missions ORDER BY id DESC LIMIT ?";
    
    return executeQuery(sql, [limit](db::SQLiteStatement& stmt) {
        stmt.bind(1, limit);
    });
}

MissionDAO::EntityPtr MissionDAO::cloneMission(int64_t missionId, const std::string& newName) {
    auto originalMission = findById(missionId);
    if (!originalMission) {
        throw std::runtime_error("Mission not found: " + std::to_string(missionId));
    }

    auto clonedMission = std::make_shared<entities::Mission>(**originalMission);
    clonedMission->setId(0); // Reset ID for new insertion
    clonedMission->setName(newName);
    clonedMission->setStatus(entities::Mission::Status::Planned);
    
    // Save the cloned mission
    clonedMission = save(clonedMission);

    // Clone mission config if exists
    auto originalConfig = getMissionConfig(missionId);
    if (originalConfig) {
        auto clonedConfig = std::make_shared<entities::MissionConfig>(**originalConfig);
        clonedConfig->setId(0);
        clonedConfig->setMissionId(clonedMission->getId());
        saveMissionConfig(clonedConfig);
    }

    return clonedMission;
}

void MissionDAO::bindEntity(const entities::Mission& entity, db::SQLiteStatement& stmt) {
    int index = 1;
    stmt.bind(index++, entity.getName());
    stmt.bind(index++, entity.getDescription());
    
    if (entity.getLaunchSiteId()) {
        stmt.bind(index++, *entity.getLaunchSiteId());
    } else {
        stmt.bindNull(index++);
    }
    
    if (entity.getVehicleId()) {
        stmt.bind(index++, *entity.getVehicleId());
    } else {
        stmt.bindNull(index++);
    }
    
    stmt.bind(index++, entity.getLaunchTime());
    stmt.bind(index++, entity.getMissionType());
    stmt.bind(index++, entity.getStatusString());
    
    if (entity.getId() == 0) {
        // For insert, bind created_at and updated_at
        std::string timestamp = getCurrentTimestamp();
        stmt.bind(index++, timestamp);
        stmt.bind(index++, timestamp);
    } else {
        // For update, only bind updated_at
        stmt.bind(index++, getCurrentTimestamp());
    }
}

MissionDAO::EntityPtr MissionDAO::extractEntity(db::SQLiteStatement& stmt) {
    auto mission = std::make_shared<entities::Mission>();
    
    mission->setId(stmt.getInt64(0));
    mission->setName(stmt.getString(1));
    mission->setDescription(stmt.getString(2));
    
    if (!stmt.isNull(3)) {
        mission->setLaunchSiteId(stmt.getInt64(3));
    }
    
    if (!stmt.isNull(4)) {
        mission->setVehicleId(stmt.getInt64(4));
    }
    
    mission->setLaunchTime(stmt.getString(5));
    mission->setMissionType(stmt.getString(6));
    mission->setStatus(entities::Mission::statusFromString(stmt.getString(7)));
    mission->setCreatedAt(stmt.getString(8));
    mission->setUpdatedAt(stmt.getString(9));
    
    return mission;
}

void MissionDAO::bindMissionConfig(const entities::MissionConfig& config, db::SQLiteStatement& stmt) {
    int index = 1;
    stmt.bind(index++, config.getMissionId());
    
    if (config.getLaunchAzimuth()) {
        stmt.bind(index++, *config.getLaunchAzimuth());
    } else {
        stmt.bindNull(index++);
    }
    
    if (config.getTargetAltitude()) {
        stmt.bind(index++, *config.getTargetAltitude());
    } else {
        stmt.bindNull(index++);
    }
    
    if (config.getTargetInclination()) {
        stmt.bind(index++, *config.getTargetInclination());
    } else {
        stmt.bindNull(index++);
    }
    
    if (config.getTargetEccentricity()) {
        stmt.bind(index++, *config.getTargetEccentricity());
    } else {
        stmt.bindNull(index++);
    }
    
    if (config.getPayloadMass()) {
        stmt.bind(index++, *config.getPayloadMass());
    } else {
        stmt.bindNull(index++);
    }
    
    if (config.getSimulationDuration()) {
        stmt.bind(index++, *config.getSimulationDuration());
    } else {
        stmt.bindNull(index++);
    }
    
    stmt.bind(index++, config.getTimestep());
    stmt.bind(index++, config.getIntegratorType());
    stmt.bind(index++, config.getForceModels());
    stmt.bind(index++, config.getConfigJson());
    
    if (config.getId() == 0) {
        // For insert
        std::string timestamp = getCurrentTimestamp();
        stmt.bind(index++, timestamp);
        stmt.bind(index++, timestamp);
    } else {
        // For update
        stmt.bind(index++, getCurrentTimestamp());
    }
}

std::shared_ptr<entities::MissionConfig> MissionDAO::extractMissionConfig(db::SQLiteStatement& stmt) {
    auto config = std::make_shared<entities::MissionConfig>();
    
    config->setId(stmt.getInt64(0));
    config->setMissionId(stmt.getInt64(1));
    
    if (!stmt.isNull(2)) {
        config->setLaunchAzimuth(stmt.getDouble(2));
    }
    
    if (!stmt.isNull(3)) {
        config->setTargetAltitude(stmt.getDouble(3));
    }
    
    if (!stmt.isNull(4)) {
        config->setTargetInclination(stmt.getDouble(4));
    }
    
    if (!stmt.isNull(5)) {
        config->setTargetEccentricity(stmt.getDouble(5));
    }
    
    if (!stmt.isNull(6)) {
        config->setPayloadMass(stmt.getDouble(6));
    }
    
    if (!stmt.isNull(7)) {
        config->setSimulationDuration(stmt.getDouble(7));
    }
    
    config->setTimestep(stmt.getDouble(8));
    config->setIntegratorType(stmt.getString(9));
    config->setForceModels(stmt.getString(10));
    config->setConfigJson(stmt.getString(11));
    config->setCreatedAt(stmt.getString(12));
    config->setUpdatedAt(stmt.getString(13));
    
    return config;
}

} // namespace database
} // namespace data
} // namespace iloss