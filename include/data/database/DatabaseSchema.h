/**
 * @file DatabaseSchema.h
 * @brief Database schema definitions for ILOSS
 * @author ILOSS Development Team
 * @date 2025
 * 
 * This file defines the database schema for the ILOSS system, including
 * table structures, relationships, and SQL statements for creation.
 */

#pragma once

#include <string>
#include <vector>

namespace iloss {
namespace data {
namespace database {

/**
 * @brief Database schema version for migration tracking
 */
constexpr int SCHEMA_VERSION = 1;

/**
 * @brief SQL statements for creating database tables
 */
namespace Schema {

// Metadata table for tracking schema version and migrations
const std::string CREATE_METADATA_TABLE = R"(
CREATE TABLE IF NOT EXISTS metadata (
    key TEXT PRIMARY KEY,
    value TEXT NOT NULL,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
)";

// Launch sites table
const std::string CREATE_LAUNCH_SITES_TABLE = R"(
CREATE TABLE IF NOT EXISTS launch_sites (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL UNIQUE,
    code TEXT NOT NULL UNIQUE,
    latitude REAL NOT NULL,
    longitude REAL NOT NULL,
    elevation REAL NOT NULL,
    azimuth_min REAL DEFAULT 0,
    azimuth_max REAL DEFAULT 360,
    is_builtin BOOLEAN DEFAULT 0,
    description TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    CHECK (latitude >= -90 AND latitude <= 90),
    CHECK (longitude >= -180 AND longitude <= 180),
    CHECK (azimuth_min >= 0 AND azimuth_min <= 360),
    CHECK (azimuth_max >= 0 AND azimuth_max <= 360)
);
)";

// Launch vehicles table
const std::string CREATE_VEHICLES_TABLE = R"(
CREATE TABLE IF NOT EXISTS vehicles (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL UNIQUE,
    manufacturer TEXT,
    type TEXT NOT NULL,
    is_builtin BOOLEAN DEFAULT 0,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
)";

// Vehicle stages table
const std::string CREATE_VEHICLE_STAGES_TABLE = R"(
CREATE TABLE IF NOT EXISTS vehicle_stages (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    vehicle_id INTEGER NOT NULL,
    stage_number INTEGER NOT NULL,
    name TEXT NOT NULL,
    gross_mass REAL NOT NULL,
    dry_mass REAL NOT NULL,
    propellant_mass REAL NOT NULL,
    length REAL NOT NULL,
    diameter REAL NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (vehicle_id) REFERENCES vehicles(id) ON DELETE CASCADE,
    UNIQUE(vehicle_id, stage_number),
    CHECK (gross_mass > 0),
    CHECK (dry_mass > 0),
    CHECK (propellant_mass >= 0),
    CHECK (length > 0),
    CHECK (diameter > 0)
);
)";

// Engines table
const std::string CREATE_ENGINES_TABLE = R"(
CREATE TABLE IF NOT EXISTS engines (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    stage_id INTEGER NOT NULL,
    name TEXT NOT NULL,
    count INTEGER DEFAULT 1,
    thrust_sl REAL,
    thrust_vac REAL NOT NULL,
    isp_sl REAL,
    isp_vac REAL NOT NULL,
    mass_flow_rate REAL NOT NULL,
    burn_time REAL,
    throttle_min REAL DEFAULT 1.0,
    throttle_max REAL DEFAULT 1.0,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (stage_id) REFERENCES vehicle_stages(id) ON DELETE CASCADE,
    CHECK (count > 0),
    CHECK (thrust_vac > 0),
    CHECK (isp_vac > 0),
    CHECK (mass_flow_rate > 0),
    CHECK (throttle_min > 0 AND throttle_min <= 1),
    CHECK (throttle_max >= throttle_min AND throttle_max <= 1)
);
)";

// Missions table
const std::string CREATE_MISSIONS_TABLE = R"(
CREATE TABLE IF NOT EXISTS missions (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL,
    description TEXT,
    launch_site_id INTEGER,
    vehicle_id INTEGER,
    launch_time TIMESTAMP,
    mission_type TEXT,
    status TEXT DEFAULT 'planned',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (launch_site_id) REFERENCES launch_sites(id),
    FOREIGN KEY (vehicle_id) REFERENCES vehicles(id),
    CHECK (status IN ('planned', 'simulating', 'completed', 'failed'))
);
)";

// Mission configurations table
const std::string CREATE_MISSION_CONFIGS_TABLE = R"(
CREATE TABLE IF NOT EXISTS mission_configs (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    mission_id INTEGER NOT NULL UNIQUE,
    launch_azimuth REAL,
    target_altitude REAL,
    target_inclination REAL,
    target_eccentricity REAL,
    payload_mass REAL,
    simulation_duration REAL,
    timestep REAL DEFAULT 0.1,
    integrator_type TEXT DEFAULT 'RK4',
    force_models TEXT,
    config_json TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (mission_id) REFERENCES missions(id) ON DELETE CASCADE,
    CHECK (timestep > 0),
    CHECK (simulation_duration > 0)
);
)";

// Simulation results table
const std::string CREATE_SIMULATION_RESULTS_TABLE = R"(
CREATE TABLE IF NOT EXISTS simulation_results (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    mission_id INTEGER NOT NULL,
    run_number INTEGER NOT NULL,
    start_time TIMESTAMP NOT NULL,
    end_time TIMESTAMP NOT NULL,
    status TEXT NOT NULL,
    final_altitude REAL,
    final_velocity REAL,
    max_altitude REAL,
    max_velocity REAL,
    max_acceleration REAL,
    total_deltav REAL,
    fuel_consumed REAL,
    error_message TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (mission_id) REFERENCES missions(id) ON DELETE CASCADE,
    UNIQUE(mission_id, run_number),
    CHECK (status IN ('success', 'failed', 'aborted'))
);
)";

// Trajectory data table (stores sampled trajectory points)
const std::string CREATE_TRAJECTORY_DATA_TABLE = R"(
CREATE TABLE IF NOT EXISTS trajectory_data (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    result_id INTEGER NOT NULL,
    time REAL NOT NULL,
    position_x REAL NOT NULL,
    position_y REAL NOT NULL,
    position_z REAL NOT NULL,
    velocity_x REAL NOT NULL,
    velocity_y REAL NOT NULL,
    velocity_z REAL NOT NULL,
    mass REAL NOT NULL,
    altitude REAL,
    latitude REAL,
    longitude REAL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (result_id) REFERENCES simulation_results(id) ON DELETE CASCADE
);
)";

// Events table
const std::string CREATE_EVENTS_TABLE = R"(
CREATE TABLE IF NOT EXISTS events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    result_id INTEGER NOT NULL,
    time REAL NOT NULL,
    event_type TEXT NOT NULL,
    description TEXT,
    data_json TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (result_id) REFERENCES simulation_results(id) ON DELETE CASCADE
);
)";

// User preferences table
const std::string CREATE_USER_PREFERENCES_TABLE = R"(
CREATE TABLE IF NOT EXISTS user_preferences (
    key TEXT PRIMARY KEY,
    value TEXT NOT NULL,
    category TEXT,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
)";

// Templates table
const std::string CREATE_TEMPLATES_TABLE = R"(
CREATE TABLE IF NOT EXISTS templates (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL UNIQUE,
    type TEXT NOT NULL,
    description TEXT,
    template_json TEXT NOT NULL,
    is_builtin BOOLEAN DEFAULT 0,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    CHECK (type IN ('mission', 'vehicle', 'report'))
);
)";

// Create indexes for better query performance
const std::vector<std::string> CREATE_INDEXES = {
    "CREATE INDEX IF NOT EXISTS idx_missions_launch_time ON missions(launch_time);",
    "CREATE INDEX IF NOT EXISTS idx_missions_status ON missions(status);",
    "CREATE INDEX IF NOT EXISTS idx_simulation_results_mission ON simulation_results(mission_id);",
    "CREATE INDEX IF NOT EXISTS idx_trajectory_time ON trajectory_data(result_id, time);",
    "CREATE INDEX IF NOT EXISTS idx_events_time ON events(result_id, time);",
    "CREATE INDEX IF NOT EXISTS idx_events_type ON events(event_type);",
    "CREATE INDEX IF NOT EXISTS idx_user_preferences_category ON user_preferences(category);"
};

// Create triggers for automatic timestamp updates
const std::vector<std::string> CREATE_TRIGGERS = {
    R"(
    CREATE TRIGGER IF NOT EXISTS update_launch_sites_timestamp 
    AFTER UPDATE ON launch_sites
    BEGIN
        UPDATE launch_sites SET updated_at = CURRENT_TIMESTAMP WHERE id = NEW.id;
    END;
    )",
    R"(
    CREATE TRIGGER IF NOT EXISTS update_vehicles_timestamp 
    AFTER UPDATE ON vehicles
    BEGIN
        UPDATE vehicles SET updated_at = CURRENT_TIMESTAMP WHERE id = NEW.id;
    END;
    )",
    R"(
    CREATE TRIGGER IF NOT EXISTS update_missions_timestamp 
    AFTER UPDATE ON missions
    BEGIN
        UPDATE missions SET updated_at = CURRENT_TIMESTAMP WHERE id = NEW.id;
    END;
    )",
    R"(
    CREATE TRIGGER IF NOT EXISTS update_mission_configs_timestamp 
    AFTER UPDATE ON mission_configs
    BEGIN
        UPDATE mission_configs SET updated_at = CURRENT_TIMESTAMP WHERE id = NEW.id;
    END;
    )",
    R"(
    CREATE TRIGGER IF NOT EXISTS update_templates_timestamp 
    AFTER UPDATE ON templates
    BEGIN
        UPDATE templates SET updated_at = CURRENT_TIMESTAMP WHERE id = NEW.id;
    END;
    )",
    R"(
    CREATE TRIGGER IF NOT EXISTS update_user_preferences_timestamp 
    AFTER UPDATE ON user_preferences
    BEGIN
        UPDATE user_preferences SET updated_at = CURRENT_TIMESTAMP WHERE key = NEW.key;
    END;
    )"
};

// All tables in creation order (respecting foreign key dependencies)
const std::vector<std::string> ALL_TABLES = {
    CREATE_METADATA_TABLE,
    CREATE_LAUNCH_SITES_TABLE,
    CREATE_VEHICLES_TABLE,
    CREATE_VEHICLE_STAGES_TABLE,
    CREATE_ENGINES_TABLE,
    CREATE_MISSIONS_TABLE,
    CREATE_MISSION_CONFIGS_TABLE,
    CREATE_SIMULATION_RESULTS_TABLE,
    CREATE_TRAJECTORY_DATA_TABLE,
    CREATE_EVENTS_TABLE,
    CREATE_USER_PREFERENCES_TABLE,
    CREATE_TEMPLATES_TABLE
};

} // namespace Schema

} // namespace database
} // namespace data
} // namespace iloss