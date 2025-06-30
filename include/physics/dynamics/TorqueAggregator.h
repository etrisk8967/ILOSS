#pragma once

#include "physics/dynamics/ITorqueModel.h"
#include <vector>
#include <map>
#include <memory>
#include <algorithm>

namespace iloss {
namespace physics {
namespace dynamics {

/**
 * @brief Aggregates multiple torque models to compute total torque
 * 
 * This class manages a collection of torque models and combines their
 * contributions to calculate the total torque acting on a rigid body.
 * Similar to ForceAggregator but for rotational dynamics.
 */
class TorqueAggregator {
public:
    /**
     * @brief Structure to hold torque contribution details
     */
    struct TorqueContribution {
        std::string modelName;          ///< Name of the contributing model
        TorqueModelType modelType;      ///< Type of the model
        math::Vector3D torque;          ///< Torque contribution in body frame
        double magnitude;               ///< Magnitude of torque for analysis
    };

    /**
     * @brief Default constructor
     */
    TorqueAggregator() = default;

    /**
     * @brief Add a torque model to the aggregator
     * @param model Torque model to add (ownership transferred)
     */
    void addModel(std::unique_ptr<ITorqueModel> model) {
        if (model) {
            m_models.push_back(std::move(model));
        }
    }

    /**
     * @brief Remove all torque models
     */
    void clearModels() {
        m_models.clear();
        m_lastContributions.clear();
    }

    /**
     * @brief Get the number of torque models
     * @return Number of models
     */
    size_t getModelCount() const {
        return m_models.size();
    }

    /**
     * @brief Enable or disable a specific model type
     * @param type Model type to enable/disable
     * @param enabled True to enable, false to disable
     * @return Number of models affected
     */
    size_t setModelEnabled(TorqueModelType type, bool enabled) {
        size_t count = 0;
        for (auto& model : m_models) {
            if (model->getType() == type) {
                model->setEnabled(enabled);
                ++count;
            }
        }
        return count;
    }

    /**
     * @brief Check if any model of a specific type is enabled
     * @param type Model type to check
     * @return True if at least one model of this type is enabled
     */
    bool isModelTypeEnabled(TorqueModelType type) const {
        return std::any_of(m_models.begin(), m_models.end(),
            [type](const auto& model) {
                return model->getType() == type && model->isEnabled();
            });
    }

    /**
     * @brief Calculate total torque from all enabled models
     * @param state Current dynamics state
     * @param time Current simulation time
     * @return Total torque vector in body frame (N⋅m)
     */
    math::Vector3D calculateTotalTorque(const DynamicsState& state, double time) {
        math::Vector3D totalTorque;
        m_lastContributions.clear();

        for (const auto& model : m_models) {
            if (model->isEnabled()) {
                math::Vector3D torque = model->calculateTorque(state, time);
                totalTorque = totalTorque + torque;

                // Store contribution for analysis
                TorqueContribution contrib;
                contrib.modelName = model->getName();
                contrib.modelType = model->getType();
                contrib.torque = torque;
                contrib.magnitude = torque.magnitude();
                m_lastContributions.push_back(contrib);
            }
        }

        return totalTorque;
    }

    /**
     * @brief Get detailed breakdown of last torque calculation
     * @return Vector of torque contributions
     */
    const std::vector<TorqueContribution>& getLastContributions() const {
        return m_lastContributions;
    }

    /**
     * @brief Get torque contribution by model type
     * @param type Model type to query
     * @return Total torque from all models of this type
     */
    math::Vector3D getTorqueByType(TorqueModelType type) const {
        math::Vector3D torque;
        for (const auto& contrib : m_lastContributions) {
            if (contrib.modelType == type) {
                torque = torque + contrib.torque;
            }
        }
        return torque;
    }

    /**
     * @brief Get percentage contribution of each model type
     * @return Map of model type to percentage contribution
     */
    std::map<TorqueModelType, double> getContributionPercentages() const {
        std::map<TorqueModelType, double> percentages;
        
        // Calculate total magnitude
        double totalMagnitude = 0.0;
        for (const auto& contrib : m_lastContributions) {
            totalMagnitude += contrib.magnitude;
        }

        if (totalMagnitude > 0.0) {
            // Calculate percentages by type
            std::map<TorqueModelType, double> typeMagnitudes;
            for (const auto& contrib : m_lastContributions) {
                typeMagnitudes[contrib.modelType] += contrib.magnitude;
            }

            for (const auto& [type, magnitude] : typeMagnitudes) {
                percentages[type] = (magnitude / totalMagnitude) * 100.0;
            }
        }

        return percentages;
    }

    /**
     * @brief Clone the aggregator with all its models
     * @return Deep copy of the aggregator
     */
    std::unique_ptr<TorqueAggregator> clone() const {
        auto cloned = std::make_unique<TorqueAggregator>();
        for (const auto& model : m_models) {
            cloned->addModel(model->clone());
        }
        return cloned;
    }

private:
    std::vector<std::unique_ptr<ITorqueModel>> m_models;   ///< Collection of torque models
    std::vector<TorqueContribution> m_lastContributions;   ///< Last calculated contributions
};

} // namespace dynamics
} // namespace physics
} // namespace iloss