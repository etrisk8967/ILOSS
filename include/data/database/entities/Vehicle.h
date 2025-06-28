/**
 * @file Vehicle.h
 * @brief Vehicle and related entity classes
 * @author ILOSS Development Team
 * @date 2025
 */

#pragma once

#include <string>
#include <vector>
#include <memory>

namespace iloss {
namespace data {
namespace database {
namespace entities {

/**
 * @brief Represents an engine in the database
 */
class Engine {
public:
    Engine() = default;
    ~Engine() = default;

    // Getters
    int64_t getId() const { return m_id; }
    int64_t getStageId() const { return m_stageId; }
    const std::string& getName() const { return m_name; }
    int getCount() const { return m_count; }
    std::optional<double> getThrustSl() const { return m_thrustSl; }
    double getThrustVac() const { return m_thrustVac; }
    std::optional<double> getIspSl() const { return m_ispSl; }
    double getIspVac() const { return m_ispVac; }
    double getMassFlowRate() const { return m_massFlowRate; }
    std::optional<double> getBurnTime() const { return m_burnTime; }
    double getThrottleMin() const { return m_throttleMin; }
    double getThrottleMax() const { return m_throttleMax; }
    const std::string& getCreatedAt() const { return m_createdAt; }

    // Setters
    void setId(int64_t id) { m_id = id; }
    void setStageId(int64_t stageId) { m_stageId = stageId; }
    void setName(const std::string& name) { m_name = name; }
    void setCount(int count) { m_count = count; }
    void setThrustSl(std::optional<double> thrust) { m_thrustSl = thrust; }
    void setThrustVac(double thrust) { m_thrustVac = thrust; }
    void setIspSl(std::optional<double> isp) { m_ispSl = isp; }
    void setIspVac(double isp) { m_ispVac = isp; }
    void setMassFlowRate(double rate) { m_massFlowRate = rate; }
    void setBurnTime(std::optional<double> time) { m_burnTime = time; }
    void setThrottleMin(double throttle) { m_throttleMin = throttle; }
    void setThrottleMax(double throttle) { m_throttleMax = throttle; }
    void setCreatedAt(const std::string& createdAt) { m_createdAt = createdAt; }

private:
    int64_t m_id = 0;
    int64_t m_stageId = 0;
    std::string m_name;
    int m_count = 1;
    std::optional<double> m_thrustSl;      // kN
    double m_thrustVac = 0;                // kN
    std::optional<double> m_ispSl;         // seconds
    double m_ispVac = 0;                   // seconds
    double m_massFlowRate = 0;             // kg/s
    std::optional<double> m_burnTime;      // seconds
    double m_throttleMin = 1.0;            // 0-1
    double m_throttleMax = 1.0;            // 0-1
    std::string m_createdAt;
};

/**
 * @brief Represents a vehicle stage in the database
 */
class VehicleStage {
public:
    VehicleStage() = default;
    ~VehicleStage() = default;

    // Getters
    int64_t getId() const { return m_id; }
    int64_t getVehicleId() const { return m_vehicleId; }
    int getStageNumber() const { return m_stageNumber; }
    const std::string& getName() const { return m_name; }
    double getGrossMass() const { return m_grossMass; }
    double getDryMass() const { return m_dryMass; }
    double getPropellantMass() const { return m_propellantMass; }
    double getLength() const { return m_length; }
    double getDiameter() const { return m_diameter; }
    const std::string& getCreatedAt() const { return m_createdAt; }
    const std::vector<std::shared_ptr<Engine>>& getEngines() const { return m_engines; }

    // Setters
    void setId(int64_t id) { m_id = id; }
    void setVehicleId(int64_t vehicleId) { m_vehicleId = vehicleId; }
    void setStageNumber(int number) { m_stageNumber = number; }
    void setName(const std::string& name) { m_name = name; }
    void setGrossMass(double mass) { m_grossMass = mass; }
    void setDryMass(double mass) { m_dryMass = mass; }
    void setPropellantMass(double mass) { m_propellantMass = mass; }
    void setLength(double length) { m_length = length; }
    void setDiameter(double diameter) { m_diameter = diameter; }
    void setCreatedAt(const std::string& createdAt) { m_createdAt = createdAt; }
    void setEngines(const std::vector<std::shared_ptr<Engine>>& engines) { m_engines = engines; }

    // Utility methods
    void addEngine(std::shared_ptr<Engine> engine) { m_engines.push_back(engine); }
    void clearEngines() { m_engines.clear(); }

private:
    int64_t m_id = 0;
    int64_t m_vehicleId = 0;
    int m_stageNumber = 0;
    std::string m_name;
    double m_grossMass = 0;        // kg
    double m_dryMass = 0;          // kg
    double m_propellantMass = 0;   // kg
    double m_length = 0;           // meters
    double m_diameter = 0;         // meters
    std::string m_createdAt;
    std::vector<std::shared_ptr<Engine>> m_engines;
};

/**
 * @brief Represents a launch vehicle in the database
 */
class Vehicle {
public:
    Vehicle() = default;
    ~Vehicle() = default;

    // Getters
    int64_t getId() const { return m_id; }
    const std::string& getName() const { return m_name; }
    const std::string& getManufacturer() const { return m_manufacturer; }
    const std::string& getType() const { return m_type; }
    bool isBuiltin() const { return m_isBuiltin; }
    const std::string& getCreatedAt() const { return m_createdAt; }
    const std::string& getUpdatedAt() const { return m_updatedAt; }
    const std::vector<std::shared_ptr<VehicleStage>>& getStages() const { return m_stages; }

    // Setters
    void setId(int64_t id) { m_id = id; }
    void setName(const std::string& name) { m_name = name; }
    void setManufacturer(const std::string& manufacturer) { m_manufacturer = manufacturer; }
    void setType(const std::string& type) { m_type = type; }
    void setBuiltin(bool builtin) { m_isBuiltin = builtin; }
    void setCreatedAt(const std::string& createdAt) { m_createdAt = createdAt; }
    void setUpdatedAt(const std::string& updatedAt) { m_updatedAt = updatedAt; }
    void setStages(const std::vector<std::shared_ptr<VehicleStage>>& stages) { m_stages = stages; }

    // Utility methods
    void addStage(std::shared_ptr<VehicleStage> stage) { m_stages.push_back(stage); }
    void clearStages() { m_stages.clear(); }
    
    /**
     * @brief Calculate total vehicle mass
     * @return Total mass in kg
     */
    double getTotalMass() const {
        double totalMass = 0;
        for (const auto& stage : m_stages) {
            totalMass += stage->getGrossMass();
        }
        return totalMass;
    }

    /**
     * @brief Calculate total vehicle length
     * @return Total length in meters
     */
    double getTotalLength() const {
        double totalLength = 0;
        for (const auto& stage : m_stages) {
            totalLength += stage->getLength();
        }
        return totalLength;
    }

private:
    int64_t m_id = 0;
    std::string m_name;
    std::string m_manufacturer;
    std::string m_type;
    bool m_isBuiltin = false;
    std::string m_createdAt;
    std::string m_updatedAt;
    std::vector<std::shared_ptr<VehicleStage>> m_stages;
};

} // namespace entities
} // namespace database
} // namespace data
} // namespace iloss