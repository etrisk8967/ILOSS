#pragma once

#include "physics/forces/ForceModel.h"
#include "core/math/Vector3D.h"
#include "physics/state/StateVector.h"
#include "core/time/Time.h"
#include "core/logging/Logger.h"
#include <vector>
#include <memory>
#include <algorithm>
#include <unordered_map>

namespace iloss {
namespace physics {
namespace forces {

/**
 * @brief Aggregates multiple force models to compute total acceleration
 * 
 * The ForceAggregator manages a collection of force models and combines
 * their contributions to calculate the total acceleration acting on an object.
 * It supports enabling/disabling individual models and provides detailed
 * acceleration breakdowns for analysis.
 */
class ForceAggregator {
public:
    /**
     * @brief Structure to hold acceleration contribution from each force model
     */
    struct AccelerationContribution {
        std::string modelName;              ///< Name of the contributing model
        ForceModelType modelType;           ///< Type of the contributing model
        math::Vector3D acceleration;        ///< Acceleration contribution
        double magnitude;                   ///< Magnitude of acceleration
        double percentageContribution;      ///< Percentage of total acceleration
    };

    /**
     * @brief Default constructor
     */
    ForceAggregator() = default;

    /**
     * @brief Add a force model to the aggregator
     * @param model Unique pointer to the force model
     * @return True if successfully added
     */
    bool addForceModel(std::unique_ptr<ForceModel> model);

    /**
     * @brief Remove a force model by name
     * @param name Name of the model to remove
     * @return True if successfully removed
     */
    bool removeForceModel(const std::string& name);

    /**
     * @brief Remove all force models of a specific type
     * @param type Type of models to remove
     * @return Number of models removed
     */
    size_t removeForceModelsByType(ForceModelType type);

    /**
     * @brief Get a force model by name
     * @param name Name of the model
     * @return Pointer to the model or nullptr if not found
     */
    ForceModel* getForceModel(const std::string& name);

    /**
     * @brief Get a const force model by name
     * @param name Name of the model
     * @return Const pointer to the model or nullptr if not found
     */
    const ForceModel* getForceModel(const std::string& name) const;

    /**
     * @brief Get all force models of a specific type
     * @param type Type of models to retrieve
     * @return Vector of pointers to matching models
     */
    std::vector<ForceModel*> getForceModelsByType(ForceModelType type);

    /**
     * @brief Enable or disable a force model by name
     * @param name Name of the model
     * @param enabled True to enable, false to disable
     * @return True if model found and state changed
     */
    bool setForceModelEnabled(const std::string& name, bool enabled);

    /**
     * @brief Enable or disable all force models of a specific type
     * @param type Type of models to affect
     * @param enabled True to enable, false to disable
     * @return Number of models affected
     */
    size_t setForceModelsEnabledByType(ForceModelType type, bool enabled);

    /**
     * @brief Calculate total acceleration from all enabled force models
     * @param state Current state vector
     * @param time Current time
     * @return Total acceleration vector in m/s²
     */
    math::Vector3D calculateTotalAcceleration(const StateVector& state, 
                                             const time::Time& time);

    /**
     * @brief Calculate total acceleration with detailed breakdown
     * @param state Current state vector
     * @param time Current time
     * @param contributions Output vector of individual contributions
     * @return Total acceleration vector in m/s²
     */
    math::Vector3D calculateTotalAccelerationWithBreakdown(
        const StateVector& state,
        const time::Time& time,
        std::vector<AccelerationContribution>& contributions);

    /**
     * @brief Update all force models for the given time
     * @param time Current time
     * 
     * This calls the update() method on all force models to allow
     * them to update time-dependent parameters.
     */
    void updateAllModels(const time::Time& time);

    /**
     * @brief Get the number of force models
     * @return Total number of models (enabled and disabled)
     */
    size_t getModelCount() const { return m_forceModels.size(); }

    /**
     * @brief Get the number of enabled force models
     * @return Number of enabled models
     */
    size_t getEnabledModelCount() const;

    /**
     * @brief Clear all force models
     */
    void clear();

    /**
     * @brief Validate all force models
     * @return True if all models are valid
     */
    bool validateAll() const;

    /**
     * @brief Get a list of all force model names
     * @return Vector of model names
     */
    std::vector<std::string> getModelNames() const;

    /**
     * @brief Clone the entire force aggregator
     * @return Unique pointer to cloned aggregator
     */
    std::unique_ptr<ForceAggregator> clone() const;

    /**
     * @brief Get a summary string of the force aggregator state
     * @return String summary
     */
    std::string toString() const;

private:
    /**
     * @brief Collection of force models
     */
    std::vector<std::unique_ptr<ForceModel>> m_forceModels;

    /**
     * @brief Map for fast lookup by name
     */
    std::unordered_map<std::string, size_t> m_modelIndexMap;

    /**
     * @brief Logger instance
     */
    iloss::logging::Logger& m_logger = iloss::logging::Logger::getInstance();

    /**
     * @brief Rebuild the index map after model changes
     */
    void rebuildIndexMap();
};

} // namespace forces
} // namespace physics
} // namespace iloss