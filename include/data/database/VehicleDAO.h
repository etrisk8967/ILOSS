/**
 * @file VehicleDAO.h
 * @brief Data Access Object for Vehicle entities
 * @author ILOSS Development Team
 * @date 2025
 */

#pragma once

#include "data/database/BaseDAO.h"
#include "data/database/entities/Vehicle.h"
#include <optional>

namespace iloss {
namespace data {
namespace database {

/**
 * @brief Data Access Object for Vehicle entities
 */
class VehicleDAO : public BaseDAO<entities::Vehicle> {
public:
    VehicleDAO();
    ~VehicleDAO() override = default;

    // Override base methods
    std::optional<EntityPtr> findById(int64_t id) override;
    EntityList findAll() override;
    EntityPtr save(EntityPtr entity) override;

    // Vehicle-specific queries
    /**
     * @brief Find vehicles by manufacturer
     * @param manufacturer Manufacturer name
     * @return Vector of vehicles from the given manufacturer
     */
    EntityList findByManufacturer(const std::string& manufacturer);

    /**
     * @brief Find vehicles by type
     * @param type Vehicle type
     * @return Vector of vehicles of the given type
     */
    EntityList findByType(const std::string& type);

    /**
     * @brief Find builtin vehicles
     * @return Vector of builtin vehicles
     */
    EntityList findBuiltin();

    /**
     * @brief Find vehicles by name pattern
     * @param pattern Name pattern (supports % wildcard)
     * @return Vector of matching vehicles
     */
    EntityList findByNamePattern(const std::string& pattern);

    /**
     * @brief Clone a vehicle
     * @param vehicleId Vehicle ID to clone
     * @param newName Name for the cloned vehicle
     * @return Cloned vehicle
     */
    EntityPtr cloneVehicle(int64_t vehicleId, const std::string& newName);

    /**
     * @brief Load vehicle with all stages and engines
     * @param vehicleId Vehicle ID
     * @return Vehicle with fully loaded stages and engines
     */
    std::optional<EntityPtr> loadComplete(int64_t vehicleId);

    // Stage operations
    /**
     * @brief Save vehicle stage
     * @param stage Stage to save
     * @return Saved stage with updated ID
     */
    std::shared_ptr<entities::VehicleStage> saveStage(std::shared_ptr<entities::VehicleStage> stage);

    /**
     * @brief Delete vehicle stage
     * @param stageId Stage ID
     * @return True if deleted, false if not found
     */
    bool deleteStage(int64_t stageId);

    /**
     * @brief Get stages for a vehicle
     * @param vehicleId Vehicle ID
     * @return Vector of stages ordered by stage number
     */
    std::vector<std::shared_ptr<entities::VehicleStage>> getStages(int64_t vehicleId);

    // Engine operations
    /**
     * @brief Save engine
     * @param engine Engine to save
     * @return Saved engine with updated ID
     */
    std::shared_ptr<entities::Engine> saveEngine(std::shared_ptr<entities::Engine> engine);

    /**
     * @brief Delete engine
     * @param engineId Engine ID
     * @return True if deleted, false if not found
     */
    bool deleteEngine(int64_t engineId);

    /**
     * @brief Get engines for a stage
     * @param stageId Stage ID
     * @return Vector of engines for the stage
     */
    std::vector<std::shared_ptr<entities::Engine>> getEngines(int64_t stageId);

protected:
    void bindEntity(const entities::Vehicle& entity, db::SQLiteStatement& stmt) override;
    EntityPtr extractEntity(db::SQLiteStatement& stmt) override;

private:
    /**
     * @brief Bind vehicle stage to statement
     * @param stage Vehicle stage
     * @param stmt SQL statement
     */
    void bindStage(const entities::VehicleStage& stage, db::SQLiteStatement& stmt);

    /**
     * @brief Extract vehicle stage from statement
     * @param stmt SQL statement
     * @return Vehicle stage
     */
    std::shared_ptr<entities::VehicleStage> extractStage(db::SQLiteStatement& stmt);

    /**
     * @brief Bind engine to statement
     * @param engine Engine
     * @param stmt SQL statement
     */
    void bindEngine(const entities::Engine& engine, db::SQLiteStatement& stmt);

    /**
     * @brief Extract engine from statement
     * @param stmt SQL statement
     * @return Engine
     */
    std::shared_ptr<entities::Engine> extractEngine(db::SQLiteStatement& stmt);

    /**
     * @brief Load all stages for a vehicle
     * @param vehicle Vehicle to load stages for
     */
    void loadStages(EntityPtr vehicle);

    /**
     * @brief Load all engines for a stage
     * @param stage Stage to load engines for
     */
    void loadEngines(std::shared_ptr<entities::VehicleStage> stage);
};

} // namespace database
} // namespace data
} // namespace iloss