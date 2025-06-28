/**
 * @file Mission.h
 * @brief Mission entity class
 * @author ILOSS Development Team
 * @date 2025
 */

#pragma once

#include <string>
#include <chrono>
#include <optional>

namespace iloss {
namespace data {
namespace database {
namespace entities {

/**
 * @brief Represents a mission in the database
 */
class Mission {
public:
    enum class Status {
        Planned,
        Simulating,
        Completed,
        Failed
    };

    Mission() = default;
    ~Mission() = default;

    // Getters
    int64_t getId() const { return m_id; }
    const std::string& getName() const { return m_name; }
    const std::string& getDescription() const { return m_description; }
    std::optional<int64_t> getLaunchSiteId() const { return m_launchSiteId; }
    std::optional<int64_t> getVehicleId() const { return m_vehicleId; }
    const std::string& getLaunchTime() const { return m_launchTime; }
    const std::string& getMissionType() const { return m_missionType; }
    Status getStatus() const { return m_status; }
    const std::string& getCreatedAt() const { return m_createdAt; }
    const std::string& getUpdatedAt() const { return m_updatedAt; }

    // Setters
    void setId(int64_t id) { m_id = id; }
    void setName(const std::string& name) { m_name = name; }
    void setDescription(const std::string& description) { m_description = description; }
    void setLaunchSiteId(std::optional<int64_t> launchSiteId) { m_launchSiteId = launchSiteId; }
    void setVehicleId(std::optional<int64_t> vehicleId) { m_vehicleId = vehicleId; }
    void setLaunchTime(const std::string& launchTime) { m_launchTime = launchTime; }
    void setMissionType(const std::string& missionType) { m_missionType = missionType; }
    void setStatus(Status status) { m_status = status; }
    void setCreatedAt(const std::string& createdAt) { m_createdAt = createdAt; }
    void setUpdatedAt(const std::string& updatedAt) { m_updatedAt = updatedAt; }

    // Status conversion helpers
    static std::string statusToString(Status status) {
        switch (status) {
            case Status::Planned: return "planned";
            case Status::Simulating: return "simulating";
            case Status::Completed: return "completed";
            case Status::Failed: return "failed";
            default: return "planned";
        }
    }

    static Status statusFromString(const std::string& str) {
        if (str == "planned") return Status::Planned;
        if (str == "simulating") return Status::Simulating;
        if (str == "completed") return Status::Completed;
        if (str == "failed") return Status::Failed;
        return Status::Planned;
    }

    std::string getStatusString() const {
        return statusToString(m_status);
    }

private:
    int64_t m_id = 0;
    std::string m_name;
    std::string m_description;
    std::optional<int64_t> m_launchSiteId;
    std::optional<int64_t> m_vehicleId;
    std::string m_launchTime;
    std::string m_missionType;
    Status m_status = Status::Planned;
    std::string m_createdAt;
    std::string m_updatedAt;
};

/**
 * @brief Mission configuration associated with a mission
 */
class MissionConfig {
public:
    MissionConfig() = default;
    ~MissionConfig() = default;

    // Getters
    int64_t getId() const { return m_id; }
    int64_t getMissionId() const { return m_missionId; }
    std::optional<double> getLaunchAzimuth() const { return m_launchAzimuth; }
    std::optional<double> getTargetAltitude() const { return m_targetAltitude; }
    std::optional<double> getTargetInclination() const { return m_targetInclination; }
    std::optional<double> getTargetEccentricity() const { return m_targetEccentricity; }
    std::optional<double> getPayloadMass() const { return m_payloadMass; }
    std::optional<double> getSimulationDuration() const { return m_simulationDuration; }
    double getTimestep() const { return m_timestep; }
    const std::string& getIntegratorType() const { return m_integratorType; }
    const std::string& getForceModels() const { return m_forceModels; }
    const std::string& getConfigJson() const { return m_configJson; }
    const std::string& getCreatedAt() const { return m_createdAt; }
    const std::string& getUpdatedAt() const { return m_updatedAt; }

    // Setters
    void setId(int64_t id) { m_id = id; }
    void setMissionId(int64_t missionId) { m_missionId = missionId; }
    void setLaunchAzimuth(std::optional<double> azimuth) { m_launchAzimuth = azimuth; }
    void setTargetAltitude(std::optional<double> altitude) { m_targetAltitude = altitude; }
    void setTargetInclination(std::optional<double> inclination) { m_targetInclination = inclination; }
    void setTargetEccentricity(std::optional<double> eccentricity) { m_targetEccentricity = eccentricity; }
    void setPayloadMass(std::optional<double> mass) { m_payloadMass = mass; }
    void setSimulationDuration(std::optional<double> duration) { m_simulationDuration = duration; }
    void setTimestep(double timestep) { m_timestep = timestep; }
    void setIntegratorType(const std::string& type) { m_integratorType = type; }
    void setForceModels(const std::string& models) { m_forceModels = models; }
    void setConfigJson(const std::string& json) { m_configJson = json; }
    void setCreatedAt(const std::string& createdAt) { m_createdAt = createdAt; }
    void setUpdatedAt(const std::string& updatedAt) { m_updatedAt = updatedAt; }

private:
    int64_t m_id = 0;
    int64_t m_missionId = 0;
    std::optional<double> m_launchAzimuth;
    std::optional<double> m_targetAltitude;
    std::optional<double> m_targetInclination;
    std::optional<double> m_targetEccentricity;
    std::optional<double> m_payloadMass;
    std::optional<double> m_simulationDuration;
    double m_timestep = 0.1;
    std::string m_integratorType = "RK4";
    std::string m_forceModels;
    std::string m_configJson;
    std::string m_createdAt;
    std::string m_updatedAt;
};

} // namespace entities
} // namespace database
} // namespace data
} // namespace iloss