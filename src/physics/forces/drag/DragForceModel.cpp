#include "physics/forces/drag/DragForceModel.h"
#include "core/constants/EarthModel.h"
#include "core/constants/AtmosphericModel.h"
#include "core/logging/Logger.h"
#include <cmath>
#include <stdexcept>

namespace iloss {
namespace physics {
namespace forces {
namespace drag {

DragForceModel::DragForceModel(std::shared_ptr<constants::AtmosphericModel> atmosphericModel)
    : ForceModel("AtmosphericDrag", ForceModelType::Drag)
    , m_dragCoefficient(2.2)  // Default drag coefficient for typical spacecraft
    , m_area(10.0)           // Default area 10 m²
    , m_ballisticCoefficient(0.0)
    , m_useBallisticCoefficient(false)
    , m_enableAtmosphericRotation(true)
    , m_enableWind(false)
    , m_windVelocity(0.0, 0.0, 0.0)
    , m_atmosphericModel(atmosphericModel)
    , m_coordinateTransformer(std::make_unique<coordinates::CoordinateTransformer>()) {
    if (!m_atmosphericModel) {
        throw std::invalid_argument("Atmospheric model cannot be null");
    }
    LOG_DEBUG("DragForceModel", "Created drag force model with " + atmosphericModel->getName());
}

DragForceModel::DragForceModel()
    : DragForceModel(std::make_shared<constants::ExponentialAtmosphere>()) {
    // Delegate to main constructor with default exponential atmosphere
}

math::Vector3D DragForceModel::calculateAcceleration(
    const StateVector& state,
    const time::Time& time) const {
    
    if (!m_enabled) {
        return math::Vector3D(0.0, 0.0, 0.0);
    }

    // Get position and velocity from state
    math::Vector3D position = state.getPosition();
    math::Vector3D velocity = state.getVelocity();
    double mass = state.getMass();

    // Transform to ECEF if needed for atmospheric model
    math::Vector3D ecefPosition = position;
    math::Vector3D ecefVelocity = velocity;
    
    if (state.getCoordinateSystem() != coordinates::CoordinateSystemType::ECEF_WGS84) {
        // Create transformation parameters
        auto params = coordinates::CoordinateTransformer::createParameters(time);
        
        // Transform position
        ecefPosition = m_coordinateTransformer->transformPosition(
            position, 
            state.getCoordinateSystem(),
            coordinates::CoordinateSystemType::ECEF_WGS84,
            params
        );
        
        // Transform velocity by creating a state vector
        coordinates::StateVector coordState;
        coordState.position = position;
        coordState.velocity = velocity;
        
        coordinates::StateVector ecefState = m_coordinateTransformer->transform(
            coordState,
            state.getCoordinateSystem(),
            coordinates::CoordinateSystemType::ECEF_WGS84,
            params
        );
        
        ecefVelocity = ecefState.velocity;
    }

    // Calculate altitude for validation
    double altitude = constants::EarthModel::getAltitude(ecefPosition);
    
    // Check if we're within valid atmosphere range
    if (!m_atmosphericModel->isValidAltitude(altitude)) {
        // Outside atmosphere - no drag
        LOG_TRACE("DragForceModel", 
            "Outside atmosphere range: altitude=" + std::to_string(altitude/1000.0) + " km");
        return math::Vector3D(0.0, 0.0, 0.0);
    }

    // Get atmospheric density
    double julianDate = time.getJulianDate();
    double density = m_atmosphericModel->getDensity(ecefPosition, julianDate);
    
    LOG_TRACE("DragForceModel", 
        "Density at altitude " + std::to_string(altitude/1000.0) + " km: " + 
        std::to_string(density) + " kg/m³");
    
    // Calculate relative velocity in ECEF frame
    math::Vector3D atmosphericVelocity(0.0, 0.0, 0.0);
    
    if (m_enableAtmosphericRotation) {
        atmosphericVelocity = calculateAtmosphericVelocity(ecefPosition);
    }
    
    if (m_enableWind) {
        atmosphericVelocity = atmosphericVelocity + m_windVelocity;
    }
    
    math::Vector3D relativeVelocity = ecefVelocity - atmosphericVelocity;
    double relativeSpeed = relativeVelocity.magnitude();
    
    // If relative velocity is near zero, no drag
    if (relativeSpeed < 1e-6) {
        return math::Vector3D(0.0, 0.0, 0.0);
    }

    // Calculate drag acceleration
    // F_drag = -0.5 * Cd * A * ρ * v² * v̂
    // a_drag = F_drag / m = -0.5 * (Cd * A / m) * ρ * v² * v̂
    
    double effectiveDragArea = getEffectiveDragArea(mass);
    double dragMagnitude = 0.5 * effectiveDragArea * density * relativeSpeed * relativeSpeed / mass;
    
    // Direction is opposite to relative velocity
    math::Vector3D dragAcceleration = relativeVelocity.normalized() * (-dragMagnitude);
    
    // Transform back to original coordinate system if needed
    if (state.getCoordinateSystem() != coordinates::CoordinateSystemType::ECEF_WGS84) {
        // Create transformation parameters
        auto params = coordinates::CoordinateTransformer::createParameters(time);
        
        // Transform acceleration as a velocity vector in a state
        coordinates::StateVector ecefAccelState;
        ecefAccelState.position = ecefPosition;
        ecefAccelState.velocity = dragAcceleration;  // Treat acceleration as velocity for transformation
        
        coordinates::StateVector origAccelState = m_coordinateTransformer->transform(
            ecefAccelState,
            coordinates::CoordinateSystemType::ECEF_WGS84,
            state.getCoordinateSystem(),
            params
        );
        
        dragAcceleration = origAccelState.velocity;  // Extract transformed acceleration
    }
    
    // Log detailed information if in debug mode
    LOG_TRACE("DragForceModel", 
        "Drag calculation: altitude=" + std::to_string(altitude/1000.0) + " km, " +
        "density=" + std::to_string(density) + " kg/m³, " +
        "rel_speed=" + std::to_string(relativeSpeed) + " m/s, " +
        "drag_accel=" + std::to_string(dragAcceleration.magnitude()) + " m/s²"
    );
    
    return dragAcceleration;
}

bool DragForceModel::initialize(const ForceModelConfig& config) {
    m_config = config;
    
    try {
        // Load drag coefficient
        if (config.hasParameter("drag_coefficient")) {
            setDragCoefficient(config.getParameter<double>("drag_coefficient"));
        }
        
        // Load cross-sectional area
        if (config.hasParameter("area")) {
            setCrossSectionalArea(config.getParameter<double>("area"));
        }
        
        // Load ballistic coefficient (overrides Cd and A if provided)
        if (config.hasParameter("ballistic_coefficient")) {
            setBallisticCoefficient(config.getParameter<double>("ballistic_coefficient"));
        }
        
        // Load atmospheric rotation flag
        if (config.hasParameter("enable_atmospheric_rotation")) {
            setAtmosphericRotation(config.getParameter<bool>("enable_atmospheric_rotation"));
        }
        
        // Load wind parameters
        if (config.hasParameter("enable_wind")) {
            setWind(config.getParameter<bool>("enable_wind"));
        }
        
        if (config.hasParameter("wind_velocity")) {
            setWindVelocity(config.getParameter<math::Vector3D>("wind_velocity"));
        }
        
        LOG_INFO("DragForceModel", 
            "Initialized with Cd=" + std::to_string(m_dragCoefficient) + 
            ", A=" + std::to_string(m_area) + " m²" +
            (m_useBallisticCoefficient ? ", BC=" + std::to_string(m_ballisticCoefficient) + " m²/kg" : "")
        );
        
        return validate();
        
    } catch (const std::exception& e) {
        LOG_ERROR("DragForceModel", "Initialization failed: " + std::string(e.what()));
        return false;
    }
}

bool DragForceModel::validate() const {
    if (m_dragCoefficient < 0.0) {
        LOG_ERROR("DragForceModel", "Invalid drag coefficient: " + std::to_string(m_dragCoefficient));
        return false;
    }
    
    if (m_area < 0.0) {
        LOG_ERROR("DragForceModel", "Invalid cross-sectional area: " + std::to_string(m_area));
        return false;
    }
    
    if (m_useBallisticCoefficient && m_ballisticCoefficient < 0.0) {
        LOG_ERROR("DragForceModel", "Invalid ballistic coefficient: " + std::to_string(m_ballisticCoefficient));
        return false;
    }
    
    if (!m_atmosphericModel) {
        LOG_ERROR("DragForceModel", "No atmospheric model set");
        return false;
    }
    
    return true;
}

std::unique_ptr<ForceModel> DragForceModel::clone() const {
    auto cloned = std::make_unique<DragForceModel>(m_atmosphericModel);
    
    // Copy all parameters
    cloned->m_dragCoefficient = m_dragCoefficient;
    cloned->m_area = m_area;
    cloned->m_ballisticCoefficient = m_ballisticCoefficient;
    cloned->m_useBallisticCoefficient = m_useBallisticCoefficient;
    cloned->m_enableAtmosphericRotation = m_enableAtmosphericRotation;
    cloned->m_enableWind = m_enableWind;
    cloned->m_windVelocity = m_windVelocity;
    cloned->m_enabled = m_enabled;
    cloned->m_config = m_config;
    
    return cloned;
}

std::string DragForceModel::toString() const {
    std::stringstream ss;
    ss << ForceModel::toString() << " [";
    
    if (m_useBallisticCoefficient) {
        ss << "BC=" << m_ballisticCoefficient << " m²/kg";
    } else {
        ss << "Cd=" << m_dragCoefficient << ", A=" << m_area << " m²";
    }
    
    ss << ", Atmosphere=" << m_atmosphericModel->getName();
    
    if (m_enableAtmosphericRotation) {
        ss << ", RotatingAtm";
    }
    
    if (m_enableWind) {
        ss << ", Wind=" << m_windVelocity.magnitude() << " m/s";
    }
    
    ss << "]";
    
    return ss.str();
}

void DragForceModel::setDragCoefficient(double cd) {
    if (cd < 0.0) {
        throw std::invalid_argument("Drag coefficient must be non-negative");
    }
    m_dragCoefficient = cd;
    m_useBallisticCoefficient = false;  // Switch to Cd/A mode
}

void DragForceModel::setCrossSectionalArea(double area) {
    if (area < 0.0) {
        throw std::invalid_argument("Cross-sectional area must be non-negative");
    }
    m_area = area;
    m_useBallisticCoefficient = false;  // Switch to Cd/A mode
}

void DragForceModel::setBallisticCoefficient(double bc) {
    if (bc < 0.0) {
        throw std::invalid_argument("Ballistic coefficient must be non-negative");
    }
    m_ballisticCoefficient = bc;
    m_useBallisticCoefficient = true;  // Switch to BC mode
}

void DragForceModel::setAtmosphericRotation(bool enable) {
    m_enableAtmosphericRotation = enable;
}

void DragForceModel::setWind(bool enable) {
    m_enableWind = enable;
}

void DragForceModel::setWindVelocity(const math::Vector3D& wind) {
    m_windVelocity = wind;
}

void DragForceModel::setAtmosphericModel(std::shared_ptr<constants::AtmosphericModel> model) {
    if (!model) {
        throw std::invalid_argument("Atmospheric model cannot be null");
    }
    m_atmosphericModel = model;
}

math::Vector3D DragForceModel::calculateRelativeVelocity(const StateVector& state) const {
    // This method is kept for potential future use but the main calculation
    // is done in calculateAcceleration for better control over coordinate transforms
    return state.getVelocity();
}

math::Vector3D DragForceModel::calculateAtmosphericVelocity(const math::Vector3D& position) const {
    // Calculate atmospheric velocity due to Earth's rotation
    // v_atm = ω × r
    // where ω is Earth's rotation vector (pointing through North pole)
    // and r is the position vector
    
    math::Vector3D omega(0.0, 0.0, EARTH_ROTATION_RATE);  // Earth rotation vector in ECEF
    return omega.cross(position);
}

double DragForceModel::getEffectiveDragArea(double mass) const {
    if (m_useBallisticCoefficient) {
        // BC = m / (Cd * A), so Cd * A = m / BC
        return mass / m_ballisticCoefficient;
    } else {
        return m_dragCoefficient * m_area;
    }
}

} // namespace drag
} // namespace forces
} // namespace physics
} // namespace iloss