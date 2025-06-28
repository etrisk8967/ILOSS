/**
 * @file LaunchSite.h
 * @brief Launch site entity class
 * @author ILOSS Development Team
 * @date 2025
 */

#pragma once

#include <string>

namespace iloss {
namespace data {
namespace database {
namespace entities {

/**
 * @brief Represents a launch site in the database
 */
class LaunchSite {
public:
    LaunchSite() = default;
    ~LaunchSite() = default;

    // Getters
    int64_t getId() const { return m_id; }
    const std::string& getName() const { return m_name; }
    const std::string& getCode() const { return m_code; }
    double getLatitude() const { return m_latitude; }
    double getLongitude() const { return m_longitude; }
    double getElevation() const { return m_elevation; }
    double getAzimuthMin() const { return m_azimuthMin; }
    double getAzimuthMax() const { return m_azimuthMax; }
    bool isBuiltin() const { return m_isBuiltin; }
    const std::string& getDescription() const { return m_description; }
    const std::string& getCreatedAt() const { return m_createdAt; }
    const std::string& getUpdatedAt() const { return m_updatedAt; }

    // Setters
    void setId(int64_t id) { m_id = id; }
    void setName(const std::string& name) { m_name = name; }
    void setCode(const std::string& code) { m_code = code; }
    void setLatitude(double latitude) { m_latitude = latitude; }
    void setLongitude(double longitude) { m_longitude = longitude; }
    void setElevation(double elevation) { m_elevation = elevation; }
    void setAzimuthMin(double azimuth) { m_azimuthMin = azimuth; }
    void setAzimuthMax(double azimuth) { m_azimuthMax = azimuth; }
    void setBuiltin(bool builtin) { m_isBuiltin = builtin; }
    void setDescription(const std::string& description) { m_description = description; }
    void setCreatedAt(const std::string& createdAt) { m_createdAt = createdAt; }
    void setUpdatedAt(const std::string& updatedAt) { m_updatedAt = updatedAt; }

    // Utility methods
    /**
     * @brief Check if a launch azimuth is within allowed range
     * @param azimuth Launch azimuth in degrees
     * @return True if azimuth is allowed, false otherwise
     */
    bool isAzimuthAllowed(double azimuth) const {
        if (m_azimuthMin <= m_azimuthMax) {
            return azimuth >= m_azimuthMin && azimuth <= m_azimuthMax;
        } else {
            // Handle wraparound case (e.g., 350-10 degrees)
            return azimuth >= m_azimuthMin || azimuth <= m_azimuthMax;
        }
    }

    /**
     * @brief Get the range of allowed azimuths
     * @return Azimuth range in degrees
     */
    double getAzimuthRange() const {
        if (m_azimuthMin <= m_azimuthMax) {
            return m_azimuthMax - m_azimuthMin;
        } else {
            return (360.0 - m_azimuthMin) + m_azimuthMax;
        }
    }

private:
    int64_t m_id = 0;
    std::string m_name;
    std::string m_code;
    double m_latitude = 0;     // degrees
    double m_longitude = 0;    // degrees
    double m_elevation = 0;    // meters
    double m_azimuthMin = 0;   // degrees
    double m_azimuthMax = 360; // degrees
    bool m_isBuiltin = false;
    std::string m_description;
    std::string m_createdAt;
    std::string m_updatedAt;
};

} // namespace entities
} // namespace database
} // namespace data
} // namespace iloss