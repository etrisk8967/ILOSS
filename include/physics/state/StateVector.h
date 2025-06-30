#pragma once

#include "core/math/Vector3D.h"
#include "core/time/Time.h"
#include "core/coordinates/CoordinateSystem.h"
#include <Eigen/Core>
#include <sstream>
#include <memory>
#include <vector>
#include <optional>

namespace iloss {
namespace physics {

/**
 * @brief Represents the state of a spacecraft or vehicle at a specific time
 * 
 * The StateVector encapsulates position, velocity, and mass of an object,
 * along with the time and coordinate system in which it is defined.
 * This is the fundamental data structure for orbital mechanics calculations.
 */
class StateVector {
public:
    /**
     * @brief Default constructor - creates an invalid state
     */
    StateVector();

    /**
     * @brief Construct a state vector with position and velocity
     * @param position Position vector in meters
     * @param velocity Velocity vector in meters/second
     * @param mass Mass in kilograms
     * @param time Time of the state
     * @param coordinateSystem Coordinate system (defaults to ECI J2000)
     */
    StateVector(const math::Vector3D& position,
                const math::Vector3D& velocity,
                double mass,
                const time::Time& time,
                coordinates::CoordinateSystemType coordinateSystem = coordinates::CoordinateSystemType::ECI_J2000);

    /**
     * @brief Copy constructor
     */
    StateVector(const StateVector& other) = default;

    /**
     * @brief Move constructor
     */
    StateVector(StateVector&& other) = default;

    /**
     * @brief Copy assignment operator
     */
    StateVector& operator=(const StateVector& other) = default;

    /**
     * @brief Move assignment operator
     */
    StateVector& operator=(StateVector&& other) = default;

    /**
     * @brief Destructor
     */
    ~StateVector() = default;

    // Getters
    /**
     * @brief Get the position vector
     * @return Position in meters
     */
    const math::Vector3D& getPosition() const { return m_position; }

    /**
     * @brief Get the velocity vector
     * @return Velocity in meters/second
     */
    const math::Vector3D& getVelocity() const { return m_velocity; }

    /**
     * @brief Get the mass
     * @return Mass in kilograms
     */
    double getMass() const { return m_mass; }

    /**
     * @brief Get the time of this state
     * @return Time object
     */
    const time::Time& getTime() const { return m_time; }

    /**
     * @brief Get the coordinate system
     * @return Coordinate system enum
     */
    coordinates::CoordinateSystemType getCoordinateSystem() const { return m_coordinateSystem; }

    /**
     * @brief Check if the state vector is valid
     * @return True if all components are valid
     */
    bool isValid() const { return m_isValid; }

    // Setters
    /**
     * @brief Set the position vector
     * @param position New position in meters
     */
    void setPosition(const math::Vector3D& position);

    /**
     * @brief Set the velocity vector
     * @param velocity New velocity in meters/second
     */
    void setVelocity(const math::Vector3D& velocity);

    /**
     * @brief Set the mass
     * @param mass New mass in kilograms
     * @throws std::invalid_argument if mass is negative or zero
     */
    void setMass(double mass);

    /**
     * @brief Set the time
     * @param time New time
     */
    void setTime(const time::Time& time);

    /**
     * @brief Set the coordinate system
     * @param coordinateSystem New coordinate system
     */
    void setCoordinateSystem(coordinates::CoordinateSystemType coordinateSystem);

    // Computed properties
    /**
     * @brief Calculate the orbital radius (distance from origin)
     * @return Radius in meters
     */
    double getRadius() const;

    /**
     * @brief Calculate the speed (magnitude of velocity)
     * @return Speed in meters/second
     */
    double getSpeed() const;

    /**
     * @brief Calculate the specific orbital energy
     * @param mu Gravitational parameter (default: Earth)
     * @return Specific energy in J/kg
     */
    double getSpecificEnergy(double mu = 3.986004418e14) const;

    /**
     * @brief Calculate the specific angular momentum vector
     * @return Angular momentum vector in m²/s
     */
    math::Vector3D getSpecificAngularMomentum() const;

    /**
     * @brief Calculate the flight path angle
     * @return Angle between velocity and local horizontal in radians
     */
    double getFlightPathAngle() const;

    // State operations
    /**
     * @brief Add a velocity change (delta-v) to the state
     * @param deltaV Velocity change in m/s
     * @param deltaMass Mass change in kg (negative for fuel consumption)
     * @return New state vector with updated velocity and mass
     */
    StateVector applyDeltaV(const math::Vector3D& deltaV, double deltaMass = 0.0) const;

    /**
     * @brief Interpolate between two states
     * @param other The other state vector
     * @param t Interpolation parameter [0, 1]
     * @return Interpolated state vector
     * @throws std::invalid_argument if states have different coordinate systems
     */
    StateVector interpolateLinear(const StateVector& other, double t) const;

    /**
     * @brief Create a state vector from classical orbital elements
     * @param a Semi-major axis in meters
     * @param e Eccentricity
     * @param i Inclination in radians
     * @param omega Argument of periapsis in radians
     * @param Omega Right ascension of ascending node in radians
     * @param nu True anomaly in radians
     * @param mass Mass in kilograms
     * @param time Time of the state
     * @param mu Gravitational parameter (default: Earth)
     * @return State vector in ECI J2000
     */
    static StateVector fromOrbitalElements(double a, double e, double i, 
                                          double omega, double Omega, double nu,
                                          double mass, const time::Time& time,
                                          double mu = 3.986004418e14);

    // Validation
    /**
     * @brief Validate the state vector components
     * @return True if all validations pass
     */
    bool validate() const;

    /**
     * @brief Get validation error message if state is invalid
     * @return Error message or empty string if valid
     */
    std::string getValidationError() const;

    // Operators
    /**
     * @brief Equality operator
     * @param other Other state vector to compare
     * @return True if states are equal within tolerance
     */
    bool operator==(const StateVector& other) const;

    /**
     * @brief Inequality operator
     * @param other Other state vector to compare
     * @return True if states are not equal
     */
    bool operator!=(const StateVector& other) const;

    // Arithmetic operators for integration
    /**
     * @brief Addition operator for state vectors
     * @param other Other state vector to add
     * @return Sum of the two state vectors
     * @throws std::invalid_argument if coordinate systems don't match
     */
    StateVector operator+(const StateVector& other) const;

    /**
     * @brief Scalar multiplication operator
     * @param scalar Scalar value to multiply by
     * @return Scaled state vector
     */
    StateVector operator*(double scalar) const;

    /**
     * @brief Friend scalar multiplication operator
     * @param scalar Scalar value to multiply by
     * @param state State vector to scale
     * @return Scaled state vector
     */
    friend StateVector operator*(double scalar, const StateVector& state);

    // Integration support methods
    /**
     * @brief Get the dimension of the state vector
     * @return Dimension (7: 3 position + 3 velocity + 1 mass)
     */
    std::size_t getDimension() const { return 7; }


    /**
     * @brief Get time as a double (seconds since J2000)
     * @return Time in seconds since J2000
     */
    double getTimeAsDouble() const;
    
    /**
     * @brief Set time from a double (seconds since J2000)
     * @param timeJ2000 Time in seconds since J2000
     */
    void setTime(double timeJ2000);

    /**
     * @brief Convert state to Eigen vector representation
     * @return Vector containing [position(3), velocity(3), mass(1)]
     */
    Eigen::VectorXd toVector() const;

    /**
     * @brief Create state from Eigen vector representation
     * @param vec Vector containing [position(3), velocity(3), mass(1)]
     * @param timeJ2000 Time in seconds since J2000
     * @param coordinateSystem Coordinate system (default: ECI J2000)
     * @return New StateVector instance
     */
    static StateVector fromVector(const Eigen::VectorXd& vec, 
                                  double timeJ2000,
                                  coordinates::CoordinateSystemType coordinateSystem = 
                                      coordinates::CoordinateSystemType::ECI_J2000);

    /**
     * @brief Calculate error norm between two states
     * @param other Other state to compare with
     * @return Weighted error norm suitable for adaptive integration
     */
    double errorNorm(const StateVector& other) const;

    // String representation
    /**
     * @brief Get a string representation of the state
     * @return Human-readable string
     */
    std::string toString() const;

private:
    math::Vector3D m_position;                    ///< Position vector in meters
    math::Vector3D m_velocity;                    ///< Velocity vector in m/s
    double m_mass;                                ///< Mass in kilograms
    time::Time m_time;                            ///< Time of the state
    coordinates::CoordinateSystemType m_coordinateSystem; ///< Reference frame
    bool m_isValid;                               ///< Validity flag
    mutable std::string m_validationError;        ///< Cached validation error

    /**
     * @brief Internal validation helper
     */
    void updateValidity();
};

/**
 * @brief State propagation interface
 * 
 * Abstract base class for state propagation algorithms
 */
class IStatePropagator {
public:
    virtual ~IStatePropagator() = default;

    /**
     * @brief Propagate a state forward in time
     * @param initialState Initial state vector
     * @param targetTime Target time for propagation
     * @return Propagated state vector
     */
    virtual StateVector propagate(const StateVector& initialState, 
                                 const time::Time& targetTime) = 0;

    /**
     * @brief Propagate a state by a time duration
     * @param initialState Initial state vector
     * @param dt Time step in seconds
     * @return Propagated state vector
     */
    virtual StateVector propagateByDuration(const StateVector& initialState, 
                                           double dt) = 0;
};

} // namespace physics
} // namespace iloss