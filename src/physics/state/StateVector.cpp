#include "physics/state/StateVector.h"
#include "core/math/MathConstants.h"
#include <cmath>
#include <iomanip>

namespace iloss {
namespace physics {

using namespace math;
using namespace math::constants;

StateVector::StateVector()
    : m_position(Vector3D::zero()),
      m_velocity(Vector3D::zero()),
      m_mass(0.0),
      m_time(time::TimeConstants::J2000_EPOCH, time::TimeSystem::UTC),
      m_coordinateSystem(coordinates::CoordinateSystemType::ECI_J2000),
      m_isValid(false),
      m_validationError("State vector not initialized")
{
}

StateVector::StateVector(const Vector3D& position,
                        const Vector3D& velocity,
                        double mass,
                        const time::Time& time,
                        coordinates::CoordinateSystemType coordinateSystem)
    : m_position(position),
      m_velocity(velocity),
      m_mass(mass),
      m_time(time),
      m_coordinateSystem(coordinateSystem),
      m_isValid(false)
{
    updateValidity();
}

void StateVector::setPosition(const Vector3D& position)
{
    m_position = position;
    updateValidity();
}

void StateVector::setVelocity(const Vector3D& velocity)
{
    m_velocity = velocity;
    updateValidity();
}

void StateVector::setMass(double mass)
{
    if (mass <= 0.0) {
        throw std::invalid_argument("Mass must be positive");
    }
    m_mass = mass;
    updateValidity();
}

void StateVector::setTime(const time::Time& time)
{
    m_time = time;
    updateValidity();
}

void StateVector::setCoordinateSystem(coordinates::CoordinateSystemType coordinateSystem)
{
    m_coordinateSystem = coordinateSystem;
    updateValidity();
}

double StateVector::getRadius() const
{
    return m_position.magnitude();
}

double StateVector::getSpeed() const
{
    return m_velocity.magnitude();
}

double StateVector::getSpecificEnergy(double mu) const
{
    double r = getRadius();
    if (r < POSITION_TOLERANCE) {
        throw std::runtime_error("Cannot compute specific energy at origin");
    }
    
    double v_squared = m_velocity.magnitudeSquared();
    return v_squared / 2.0 - mu / r;
}

Vector3D StateVector::getSpecificAngularMomentum() const
{
    return m_position.cross(m_velocity);
}

double StateVector::getFlightPathAngle() const
{
    double r_dot_v = m_position.dot(m_velocity);
    double r_mag = getRadius();
    double v_mag = getSpeed();
    
    if (r_mag < POSITION_TOLERANCE || v_mag < VELOCITY_TOLERANCE) {
        return 0.0;
    }
    
    // Flight path angle is angle between velocity and local horizontal
    // sin(gamma) = r_dot_v / (r * v)
    double sin_gamma = r_dot_v / (r_mag * v_mag);
    
    // Clamp to avoid numerical errors
    sin_gamma = std::max(-1.0, std::min(1.0, sin_gamma));
    
    return std::asin(sin_gamma);
}

StateVector StateVector::applyDeltaV(const Vector3D& deltaV, double deltaMass) const
{
    if (!m_isValid) {
        throw std::runtime_error("Cannot apply delta-v to invalid state");
    }
    
    double newMass = m_mass + deltaMass;
    if (newMass <= 0.0) {
        throw std::invalid_argument("Resulting mass would be non-positive");
    }
    
    StateVector newState(*this);
    newState.m_velocity += deltaV;
    newState.m_mass = newMass;
    newState.updateValidity();
    
    return newState;
}

StateVector StateVector::interpolateLinear(const StateVector& other, double t) const
{
    if (!m_isValid || !other.m_isValid) {
        throw std::runtime_error("Cannot interpolate invalid states");
    }
    
    if (m_coordinateSystem != other.m_coordinateSystem) {
        throw std::invalid_argument("Cannot interpolate between different coordinate systems");
    }
    
    if (t < 0.0 || t > 1.0) {
        throw std::invalid_argument("Interpolation parameter must be in [0, 1]");
    }
    
    // Linear interpolation for position and velocity
    Vector3D interpPos = m_position * (1.0 - t) + other.m_position * t;
    Vector3D interpVel = m_velocity * (1.0 - t) + other.m_velocity * t;
    
    // Linear interpolation for mass
    double interpMass = m_mass * (1.0 - t) + other.m_mass * t;
    
    // Linear interpolation for time
    double t1 = m_time.getTime(time::TimeSystem::UTC);
    double t2 = other.m_time.getTime(time::TimeSystem::UTC);
    double interpTime = t1 * (1.0 - t) + t2 * t;
    
    StateVector result(interpPos, interpVel, interpMass, 
                      time::Time(interpTime, time::TimeSystem::UTC),
                      m_coordinateSystem);
    
    return result;
}

StateVector StateVector::fromOrbitalElements(double a, double e, double i,
                                           double omega, double Omega, double nu,
                                           double mass, const time::Time& time,
                                           double mu)
{
    // Validate orbital elements
    if (a <= 0.0) {
        throw std::invalid_argument("Semi-major axis must be positive");
    }
    if (e < 0.0) {
        throw std::invalid_argument("Eccentricity cannot be negative");
    }
    if (e >= 1.0 && a > 0.0) {
        throw std::invalid_argument("For parabolic/hyperbolic orbits, use negative semi-major axis");
    }
    
    // Calculate position and velocity in perifocal coordinate system
    double p = a * (1.0 - e * e);  // Semi-latus rectum
    double r = p / (1.0 + e * std::cos(nu));  // Radius
    
    // Position in perifocal frame
    double x_pf = r * std::cos(nu);
    double y_pf = r * std::sin(nu);
    double z_pf = 0.0;
    
    // Velocity in perifocal frame
    double sqrt_mu_p = std::sqrt(mu / p);
    double vx_pf = -sqrt_mu_p * std::sin(nu);
    double vy_pf = sqrt_mu_p * (e + std::cos(nu));
    double vz_pf = 0.0;
    
    // Rotation matrices
    double cos_omega = std::cos(omega);
    double sin_omega = std::sin(omega);
    double cos_Omega = std::cos(Omega);
    double sin_Omega = std::sin(Omega);
    double cos_i = std::cos(i);
    double sin_i = std::sin(i);
    
    // Transform to ECI frame
    // R = R3(-Omega) * R1(-i) * R3(-omega)
    double x_eci = (cos_omega * cos_Omega - sin_omega * cos_i * sin_Omega) * x_pf +
                   (-sin_omega * cos_Omega - cos_omega * cos_i * sin_Omega) * y_pf +
                   (sin_i * sin_Omega) * z_pf;
                   
    double y_eci = (cos_omega * sin_Omega + sin_omega * cos_i * cos_Omega) * x_pf +
                   (-sin_omega * sin_Omega + cos_omega * cos_i * cos_Omega) * y_pf +
                   (-sin_i * cos_Omega) * z_pf;
                   
    double z_eci = (sin_omega * sin_i) * x_pf +
                   (cos_omega * sin_i) * y_pf +
                   (cos_i) * z_pf;
    
    double vx_eci = (cos_omega * cos_Omega - sin_omega * cos_i * sin_Omega) * vx_pf +
                    (-sin_omega * cos_Omega - cos_omega * cos_i * sin_Omega) * vy_pf +
                    (sin_i * sin_Omega) * vz_pf;
                    
    double vy_eci = (cos_omega * sin_Omega + sin_omega * cos_i * cos_Omega) * vx_pf +
                    (-sin_omega * sin_Omega + cos_omega * cos_i * cos_Omega) * vy_pf +
                    (-sin_i * cos_Omega) * vz_pf;
                    
    double vz_eci = (sin_omega * sin_i) * vx_pf +
                    (cos_omega * sin_i) * vy_pf +
                    (cos_i) * vz_pf;
    
    Vector3D position(x_eci, y_eci, z_eci);
    Vector3D velocity(vx_eci, vy_eci, vz_eci);
    
    return StateVector(position, velocity, mass, time, 
                      coordinates::CoordinateSystemType::ECI_J2000);
}

bool StateVector::validate() const
{
    m_validationError.clear();
    
    // Check for NaN or infinity in position
    if (std::isnan(m_position.x()) || std::isnan(m_position.y()) || std::isnan(m_position.z()) ||
        std::isinf(m_position.x()) || std::isinf(m_position.y()) || std::isinf(m_position.z())) {
        m_validationError = "Position contains NaN or infinity";
        return false;
    }
    
    // Check for NaN or infinity in velocity
    if (std::isnan(m_velocity.x()) || std::isnan(m_velocity.y()) || std::isnan(m_velocity.z()) ||
        std::isinf(m_velocity.x()) || std::isinf(m_velocity.y()) || std::isinf(m_velocity.z())) {
        m_validationError = "Velocity contains NaN or infinity";
        return false;
    }
    
    // Check mass
    if (std::isnan(m_mass) || std::isinf(m_mass)) {
        m_validationError = "Mass is NaN or infinity";
        return false;
    }
    if (m_mass <= 0.0) {
        m_validationError = "Mass must be positive";
        return false;
    }
    
    // Check position magnitude (basic sanity check)
    double r = getRadius();
    if (r < 100.0) {  // Less than 100m from Earth center is unrealistic
        m_validationError = "Position too close to Earth center";
        return false;
    }
    if (r > 1e9) {  // More than 1 million km is beyond typical Earth orbit
        m_validationError = "Position unrealistically far from Earth";
        return false;
    }
    
    // Check velocity magnitude
    double v = getSpeed();
    if (v > 100000.0) {  // More than 100 km/s is unrealistic for Earth orbit
        m_validationError = "Velocity unrealistically high";
        return false;
    }
    
    return true;
}

std::string StateVector::getValidationError() const
{
    return m_validationError;
}

bool StateVector::operator==(const StateVector& other) const
{
    if (m_coordinateSystem != other.m_coordinateSystem) {
        return false;
    }
    
    // Use appropriate tolerances for comparison
    bool posEqual = (m_position - other.m_position).magnitude() < POSITION_TOLERANCE;
    bool velEqual = (m_velocity - other.m_velocity).magnitude() < VELOCITY_TOLERANCE;
    bool massEqual = std::abs(m_mass - other.m_mass) < 1e-9;  // kg tolerance
    bool timeEqual = std::abs(m_time.getTime() - other.m_time.getTime()) < TIME_TOLERANCE;
    
    return posEqual && velEqual && massEqual && timeEqual;
}

bool StateVector::operator!=(const StateVector& other) const
{
    return !(*this == other);
}

std::string StateVector::toString() const
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << "StateVector[";
    
    if (!m_isValid) {
        ss << "INVALID: " << m_validationError;
    } else {
        ss << "Pos=(" << m_position.x() / 1000.0 << ", " 
           << m_position.y() / 1000.0 << ", " 
           << m_position.z() / 1000.0 << ") km, ";
        ss << "Vel=(" << m_velocity.x() << ", " 
           << m_velocity.y() << ", " 
           << m_velocity.z() << ") m/s, ";
        ss << "Mass=" << m_mass << " kg, ";
        ss << "Time=" << m_time.toCalendarString() << ", ";
        
        // Coordinate system name
        switch (m_coordinateSystem) {
            case coordinates::CoordinateSystemType::ECI_J2000:
                ss << "Frame=ECI_J2000";
                break;
            case coordinates::CoordinateSystemType::ECEF_WGS84:
                ss << "Frame=ECEF_WGS84";
                break;
            case coordinates::CoordinateSystemType::RTN:
                ss << "Frame=RTN";
                break;
            case coordinates::CoordinateSystemType::LVLH:
                ss << "Frame=LVLH";
                break;
            default:
                ss << "Frame=Other";
                break;
        }
    }
    
    ss << "]";
    return ss.str();
}

void StateVector::updateValidity()
{
    m_isValid = validate();
}

StateVector StateVector::operator+(const StateVector& other) const
{
    if (m_coordinateSystem != other.m_coordinateSystem) {
        throw std::invalid_argument("Cannot add state vectors in different coordinate systems");
    }
    
    // Add positions and velocities
    Vector3D newPosition = m_position + other.m_position;
    Vector3D newVelocity = m_velocity + other.m_velocity;
    
    // For mass, we'll use the average (this is a design decision for integration)
    double newMass = (m_mass + other.m_mass) / 2.0;
    
    // For time, use the average
    double t1 = m_time.getTime(time::TimeSystem::UTC);
    double t2 = other.m_time.getTime(time::TimeSystem::UTC);
    double avgTime = (t1 + t2) / 2.0;
    
    return StateVector(newPosition, newVelocity, newMass,
                       time::Time(avgTime, time::TimeSystem::UTC),
                       m_coordinateSystem);
}

StateVector StateVector::operator*(double scalar) const
{
    // Scale position and velocity
    Vector3D newPosition = m_position * scalar;
    Vector3D newVelocity = m_velocity * scalar;
    
    // Mass and time remain unchanged
    return StateVector(newPosition, newVelocity, m_mass, m_time, m_coordinateSystem);
}

StateVector operator*(double scalar, const StateVector& state)
{
    return state * scalar;
}


double StateVector::getTimeAsDouble() const
{
    // Convert to seconds since J2000 in UTC
    return m_time.getJ2000(time::TimeSystem::UTC);
}

void StateVector::setTime(double timeJ2000)
{
    // Convert J2000 seconds to Unix seconds
    // J2000 epoch is 946728000.0 seconds after Unix epoch
    double unixSeconds = timeJ2000 + time::TimeConstants::J2000_EPOCH;
    m_time = time::Time(unixSeconds, time::TimeSystem::UTC);
    updateValidity();
}

Eigen::VectorXd StateVector::toVector() const
{
    Eigen::VectorXd vec(7);
    vec(0) = m_position.x();
    vec(1) = m_position.y();
    vec(2) = m_position.z();
    vec(3) = m_velocity.x();
    vec(4) = m_velocity.y();
    vec(5) = m_velocity.z();
    vec(6) = m_mass;
    return vec;
}

StateVector StateVector::fromVector(const Eigen::VectorXd& vec, 
                                    double timeJ2000,
                                    coordinates::CoordinateSystemType coordinateSystem)
{
    if (vec.size() != 7) {
        throw std::invalid_argument("Vector must have dimension 7 for StateVector");
    }
    
    Vector3D position(vec(0), vec(1), vec(2));
    Vector3D velocity(vec(3), vec(4), vec(5));
    double mass = vec(6);
    
    // Convert J2000 seconds to Unix seconds
    double unixSeconds = timeJ2000 + time::TimeConstants::J2000_EPOCH;
    
    return StateVector(position, velocity, mass,
                       time::Time(unixSeconds, time::TimeSystem::UTC),
                       coordinateSystem);
}

double StateVector::errorNorm(const StateVector& other) const
{
    if (m_coordinateSystem != other.m_coordinateSystem) {
        throw std::invalid_argument("Cannot compute error norm between different coordinate systems");
    }
    
    // Position error in meters
    double posError = (m_position - other.m_position).magnitude();
    
    // Velocity error in m/s
    double velError = (m_velocity - other.m_velocity).magnitude();
    
    // Mass error in kg
    double massError = std::abs(m_mass - other.m_mass);
    
    // Weighted error norm suitable for adaptive integration
    // Scale factors chosen to balance position (km), velocity (km/s), and mass (tons)
    const double posScale = 1.0e3;    // 1 km
    const double velScale = 1.0;      // 1 m/s
    const double massScale = 1.0e3;   // 1 ton
    
    double scaledPosError = posError / posScale;
    double scaledVelError = velError / velScale;
    double scaledMassError = massError / massScale;
    
    // RMS error
    return std::sqrt((scaledPosError * scaledPosError + 
                      scaledVelError * scaledVelError + 
                      scaledMassError * scaledMassError) / 3.0);
}

} // namespace physics
} // namespace iloss