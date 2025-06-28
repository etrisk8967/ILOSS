#include "physics/forces/ForceAggregator.h"
#include <sstream>
#include <iomanip>
#include <fmt/format.h>

namespace iloss {
namespace physics {
namespace forces {

bool ForceAggregator::addForceModel(std::unique_ptr<ForceModel> model) {
    if (!model) {
        m_logger.error(logging::LogCategory::Physics, "Cannot add null force model");
        return false;
    }

    const std::string& name = model->getName();
    
    // Check if model with same name already exists
    if (m_modelIndexMap.find(name) != m_modelIndexMap.end()) {
        m_logger.warning(logging::LogCategory::Physics, 
            fmt::format("Force model with name '{}' already exists", name));
        return false;
    }

    // Add the model
    m_forceModels.push_back(std::move(model));
    m_modelIndexMap[name] = m_forceModels.size() - 1;

    m_logger.info(logging::LogCategory::Physics, 
        fmt::format("Added force model '{}' of type {}", 
        name, ForceModel::forceModelTypeToString(m_forceModels.back()->getType())));

    return true;
}

bool ForceAggregator::removeForceModel(const std::string& name) {
    auto it = m_modelIndexMap.find(name);
    if (it == m_modelIndexMap.end()) {
        m_logger.warning(logging::LogCategory::Physics, 
            fmt::format("Force model '{}' not found for removal", name));
        return false;
    }

    size_t index = it->second;
    
    // Remove from vector
    m_forceModels.erase(m_forceModels.begin() + index);
    
    // Rebuild index map
    rebuildIndexMap();

    m_logger.info(logging::LogCategory::Physics, 
        fmt::format("Removed force model '{}'", name));
    return true;
}

size_t ForceAggregator::removeForceModelsByType(ForceModelType type) {
    size_t removedCount = 0;
    
    // Remove all models of the specified type
    auto it = m_forceModels.begin();
    while (it != m_forceModels.end()) {
        if ((*it)->getType() == type) {
            it = m_forceModels.erase(it);
            removedCount++;
        } else {
            ++it;
        }
    }

    if (removedCount > 0) {
        rebuildIndexMap();
        m_logger.info(logging::LogCategory::Physics, 
            fmt::format("Removed {} force models of type {}", 
            removedCount, ForceModel::forceModelTypeToString(type)));
    }

    return removedCount;
}

ForceModel* ForceAggregator::getForceModel(const std::string& name) {
    auto it = m_modelIndexMap.find(name);
    if (it != m_modelIndexMap.end()) {
        return m_forceModels[it->second].get();
    }
    return nullptr;
}

const ForceModel* ForceAggregator::getForceModel(const std::string& name) const {
    auto it = m_modelIndexMap.find(name);
    if (it != m_modelIndexMap.end()) {
        return m_forceModels[it->second].get();
    }
    return nullptr;
}

std::vector<ForceModel*> ForceAggregator::getForceModelsByType(ForceModelType type) {
    std::vector<ForceModel*> models;
    for (const auto& model : m_forceModels) {
        if (model->getType() == type) {
            models.push_back(model.get());
        }
    }
    return models;
}

bool ForceAggregator::setForceModelEnabled(const std::string& name, bool enabled) {
    ForceModel* model = getForceModel(name);
    if (model) {
        model->setEnabled(enabled);
        m_logger.debug(logging::LogCategory::Physics, 
            fmt::format("Force model '{}' {}", name, enabled ? "enabled" : "disabled"));
        return true;
    }
    return false;
}

size_t ForceAggregator::setForceModelsEnabledByType(ForceModelType type, bool enabled) {
    size_t count = 0;
    for (auto& model : m_forceModels) {
        if (model->getType() == type) {
            model->setEnabled(enabled);
            count++;
        }
    }
    
    if (count > 0) {
        m_logger.debug(logging::LogCategory::Physics, 
            fmt::format("{} {} force models of type {}", 
            enabled ? "Enabled" : "Disabled", count,
            ForceModel::forceModelTypeToString(type)));
    }
    
    return count;
}

math::Vector3D ForceAggregator::calculateTotalAcceleration(
    const StateVector& state, 
    const time::Time& time) {
    
    math::Vector3D totalAcceleration(0.0, 0.0, 0.0);
    
    for (const auto& model : m_forceModels) {
        if (model->isEnabled()) {
            try {
                math::Vector3D acceleration = model->calculateAcceleration(state, time);
                totalAcceleration += acceleration;
            } catch (const std::exception& e) {
                m_logger.error(logging::LogCategory::Physics, 
                    fmt::format("Error calculating acceleration for model '{}': {}", 
                    model->getName(), e.what()));
            }
        }
    }
    
    return totalAcceleration;
}

math::Vector3D ForceAggregator::calculateTotalAccelerationWithBreakdown(
    const StateVector& state,
    const time::Time& time,
    std::vector<AccelerationContribution>& contributions) {
    
    contributions.clear();
    math::Vector3D totalAcceleration(0.0, 0.0, 0.0);
    
    // Calculate individual contributions
    for (const auto& model : m_forceModels) {
        if (model->isEnabled()) {
            try {
                math::Vector3D acceleration = model->calculateAcceleration(state, time);
                totalAcceleration += acceleration;
                
                AccelerationContribution contrib;
                contrib.modelName = model->getName();
                contrib.modelType = model->getType();
                contrib.acceleration = acceleration;
                contrib.magnitude = acceleration.magnitude();
                contrib.percentageContribution = 0.0; // Will be calculated after
                
                contributions.push_back(contrib);
            } catch (const std::exception& e) {
                m_logger.error(logging::LogCategory::Physics, 
                    fmt::format("Error calculating acceleration for model '{}': {}", 
                    model->getName(), e.what()));
            }
        }
    }
    
    // Calculate percentage contributions
    double totalMagnitude = totalAcceleration.magnitude();
    if (totalMagnitude > 1e-15) {
        for (auto& contrib : contributions) {
            contrib.percentageContribution = 
                (contrib.magnitude / totalMagnitude) * 100.0;
        }
    }
    
    // Sort by magnitude (largest first)
    std::sort(contributions.begin(), contributions.end(),
        [](const AccelerationContribution& a, const AccelerationContribution& b) {
            return a.magnitude > b.magnitude;
        });
    
    return totalAcceleration;
}

void ForceAggregator::updateAllModels(const time::Time& time) {
    for (auto& model : m_forceModels) {
        if (model->isEnabled()) {
            try {
                model->update(time);
            } catch (const std::exception& e) {
                m_logger.error(logging::LogCategory::Physics, 
                    fmt::format("Error updating model '{}': {}", 
                    model->getName(), e.what()));
            }
        }
    }
}

size_t ForceAggregator::getEnabledModelCount() const {
    size_t count = 0;
    for (const auto& model : m_forceModels) {
        if (model->isEnabled()) {
            count++;
        }
    }
    return count;
}

void ForceAggregator::clear() {
    m_forceModels.clear();
    m_modelIndexMap.clear();
    m_logger.info(logging::LogCategory::Physics, "Cleared all force models");
}

bool ForceAggregator::validateAll() const {
    bool allValid = true;
    for (const auto& model : m_forceModels) {
        if (!model->validate()) {
            m_logger.error(logging::LogCategory::Physics, 
                fmt::format("Force model '{}' failed validation", model->getName()));
            allValid = false;
        }
    }
    return allValid;
}

std::vector<std::string> ForceAggregator::getModelNames() const {
    std::vector<std::string> names;
    names.reserve(m_forceModels.size());
    for (const auto& model : m_forceModels) {
        names.push_back(model->getName());
    }
    return names;
}

std::unique_ptr<ForceAggregator> ForceAggregator::clone() const {
    auto clone = std::make_unique<ForceAggregator>();
    
    for (const auto& model : m_forceModels) {
        clone->addForceModel(model->clone());
    }
    
    return clone;
}

std::string ForceAggregator::toString() const {
    std::stringstream ss;
    ss << "ForceAggregator: " << m_forceModels.size() << " models, "
       << getEnabledModelCount() << " enabled\n";
    
    for (const auto& model : m_forceModels) {
        ss << "  - " << model->getName() 
           << " (" << ForceModel::forceModelTypeToString(model->getType()) << ")"
           << " [" << (model->isEnabled() ? "enabled" : "disabled") << "]\n";
    }
    
    return ss.str();
}

void ForceAggregator::rebuildIndexMap() {
    m_modelIndexMap.clear();
    for (size_t i = 0; i < m_forceModels.size(); ++i) {
        m_modelIndexMap[m_forceModels[i]->getName()] = i;
    }
}

} // namespace forces
} // namespace physics
} // namespace iloss