/**
 * @file two_body_example.cpp
 * @brief Example demonstrating two-body orbital dynamics
 * 
 * This example shows how to:
 * 1. Create and propagate orbits using Kepler's laws
 * 2. Convert between state vectors and orbital elements
 * 3. Calculate orbital maneuvers
 * 4. Analyze conic sections
 */

#include "physics/forces/twobody/TwoBodyAnalyticalPropagator.h"
#include "physics/forces/twobody/ConicSectionUtilities.h"
#include "core/logging/Logger.h"
#include <iostream>
#include <iomanip>

using namespace iloss;
using namespace iloss::physics::forces::twobody;

void printOrbitalElements(const OrbitalElements& elements) {
    std::cout << "\nOrbital Elements:\n";
    std::cout << "  Semi-major axis: " << elements.a / 1000.0 << " km\n";
    std::cout << "  Eccentricity: " << elements.e << "\n";
    std::cout << "  Inclination: " << math::constants::radiansToDegrees(elements.i) << " deg\n";
    std::cout << "  Arg of periapsis: " << math::constants::radiansToDegrees(elements.omega) << " deg\n";
    std::cout << "  RAAN: " << math::constants::radiansToDegrees(elements.Omega) << " deg\n";
    std::cout << "  True anomaly: " << math::constants::radiansToDegrees(elements.nu) << " deg\n";
    std::cout << "  Period: " << elements.T / 3600.0 << " hours\n";
    std::cout << "  Orbit type: " << elements.getOrbitType() << "\n";
}

void printState(const StateVector& state, const std::string& label) {
    std::cout << "\n" << label << ":\n";
    std::cout << "  Position: (" 
              << state.getPosition().x() / 1000.0 << ", "
              << state.getPosition().y() / 1000.0 << ", "
              << state.getPosition().z() / 1000.0 << ") km\n";
    std::cout << "  Velocity: ("
              << state.getVelocity().x() << ", "
              << state.getVelocity().y() << ", "
              << state.getVelocity().z() << ") m/s\n";
    std::cout << "  Time: " << state.getTime().toISO8601() << "\n";
}

int main() {
    // Initialize logging
    Logger::getInstance().setMinLevel(LogLevel::INFO);
    
    std::cout << "=== Two-Body Dynamics Example ===\n";
    
    // Create propagator
    TwoBodyAnalyticalPropagator propagator;
    
    // Example 1: ISS-like orbit
    std::cout << "\n--- Example 1: ISS-like Orbit ---\n";
    
    // Create initial state (circular orbit at 408 km altitude, 51.6° inclination)
    double altitude = 408000.0; // 408 km
    double radius = math::constants::EARTH_RADIUS_EQUATORIAL + altitude;
    double inclination = 51.6 * math::constants::DEG_TO_RAD;
    
    // Position at ascending node
    math::Vector3D position(radius, 0.0, 0.0);
    
    // Velocity for circular orbit
    double v_circular = std::sqrt(math::constants::EARTH_MU / radius);
    math::Vector3D velocity(0.0, 
                           v_circular * std::cos(inclination),
                           v_circular * std::sin(inclination));
    
    time::Time epoch(2025, 6, 27, 12, 0, 0.0);
    StateVector issState(position, velocity, 420000.0, epoch); // ISS mass ~420 tons
    
    printState(issState, "Initial State");
    
    // Convert to orbital elements
    OrbitalElements issElements = propagator.determineOrbit(
        issState.getPosition(), issState.getVelocity(), issState.getTime());
    printOrbitalElements(issElements);
    
    // Propagate for one orbit
    std::cout << "\nPropagating for one complete orbit...\n";
    StateVector finalState = propagator.propagateByDuration(issState, issElements.T);
    printState(finalState, "After One Orbit");
    
    // Verify we're back at the starting position
    double positionError = (finalState.getPosition() - issState.getPosition()).magnitude();
    std::cout << "Position error after one orbit: " << positionError << " m\n";
    
    // Example 2: Hohmann Transfer
    std::cout << "\n--- Example 2: Hohmann Transfer from LEO to GEO ---\n";
    
    double r_leo = math::constants::EARTH_RADIUS_EQUATORIAL + 300000.0; // 300 km altitude
    double r_geo = 42164000.0; // GEO radius
    
    auto [dv1, dv2] = propagator.calculateHohmannTransfer(r_leo, r_geo);
    
    std::cout << "LEO altitude: " << (r_leo - math::constants::EARTH_RADIUS_EQUATORIAL) / 1000.0 << " km\n";
    std::cout << "GEO altitude: " << (r_geo - math::constants::EARTH_RADIUS_EQUATORIAL) / 1000.0 << " km\n";
    std::cout << "First burn (at LEO): " << dv1 << " m/s (" << dv1 / 1000.0 << " km/s)\n";
    std::cout << "Second burn (at GEO): " << dv2 << " m/s (" << dv2 / 1000.0 << " km/s)\n";
    std::cout << "Total delta-v: " << (dv1 + dv2) << " m/s (" << (dv1 + dv2) / 1000.0 << " km/s)\n";
    
    // Example 3: Elliptical Orbit Analysis
    std::cout << "\n--- Example 3: Elliptical Orbit Analysis ---\n";
    
    // Create a Molniya-like orbit
    double periapsis = math::constants::EARTH_RADIUS_EQUATORIAL + 600000.0; // 600 km
    double apoapsis = math::constants::EARTH_RADIUS_EQUATORIAL + 39400000.0; // 39,400 km
    
    position = math::Vector3D(periapsis, 0.0, 0.0);
    double a = (periapsis + apoapsis) / 2.0;
    double v_periapsis = std::sqrt(math::constants::EARTH_MU * (2.0 / periapsis - 1.0 / a));
    velocity = math::Vector3D(0.0, v_periapsis, 0.0);
    
    StateVector molniyaState(position, velocity, 1000.0, epoch);
    OrbitalElements molniyaElements = propagator.determineOrbit(
        molniyaState.getPosition(), molniyaState.getVelocity(), molniyaState.getTime());
    
    printOrbitalElements(molniyaElements);
    
    // Calculate time to apoapsis
    time::Time apoapsisTime = propagator.nextApoapsisTime(molniyaState);
    double timeToApoapsis = apoapsisTime.getTime() - epoch.getTime();
    std::cout << "\nTime to reach apoapsis: " << timeToApoapsis / 3600.0 << " hours\n";
    
    // Propagate to apoapsis
    StateVector atApoapsis = propagator.propagateToTime(molniyaState, apoapsisTime);
    printState(atApoapsis, "State at Apoapsis");
    
    // Example 4: Conic Section Utilities
    std::cout << "\n--- Example 4: Conic Section Analysis ---\n";
    
    // Analyze the Molniya orbit
    double energy = ConicSectionUtilities::calculateSpecificEnergy(
        molniyaState.getPosition(), molniyaState.getVelocity(), math::constants::EARTH_MU);
    
    math::Vector3D h = ConicSectionUtilities::calculateSpecificAngularMomentum(
        molniyaState.getPosition(), molniyaState.getVelocity());
    
    std::cout << "Specific energy: " << energy / 1000.0 << " km²/s²\n";
    std::cout << "Specific angular momentum: " << h.magnitude() / 1000.0 << " km²/s\n";
    std::cout << "Is bound orbit: " << (ConicSectionUtilities::isBoundOrbit(energy) ? "Yes" : "No") << "\n";
    
    // Calculate velocities at apsides
    auto [v_peri, v_apo] = ConicSectionUtilities::calculateApsidalVelocities(
        math::constants::EARTH_MU, molniyaElements.a, molniyaElements.e);
    
    std::cout << "Velocity at periapsis: " << v_peri << " m/s\n";
    std::cout << "Velocity at apoapsis: " << v_apo << " m/s\n";
    
    // Example 5: Ground Track
    std::cout << "\n--- Example 5: Ground Track Generation ---\n";
    
    // Generate ground track for ISS orbit
    double trackDuration = issElements.T; // One complete orbit
    double timeStep = 60.0; // 1 minute
    
    auto groundTrack = propagator.calculateGroundTrack(issState, trackDuration, timeStep);
    
    std::cout << "Generated " << groundTrack.size() << " ground track points\n";
    std::cout << "First 5 points (lat, lon in degrees):\n";
    
    for (size_t i = 0; i < std::min(size_t(5), groundTrack.size()); ++i) {
        auto [lat, lon] = groundTrack[i];
        std::cout << "  Point " << i << ": ("
                  << math::constants::radiansToDegrees(lat) << ", "
                  << math::constants::radiansToDegrees(lon) << ")\n";
    }
    
    // Example 6: Lambert's Problem (simplified)
    std::cout << "\n--- Example 6: Lambert's Problem ---\n";
    
    // Two points on a circular orbit
    double lambert_radius = 8000000.0;
    double angle_separation = 60.0 * math::constants::DEG_TO_RAD;
    
    math::Vector3D r1(lambert_radius, 0.0, 0.0);
    math::Vector3D r2(lambert_radius * std::cos(angle_separation),
                     lambert_radius * std::sin(angle_separation), 0.0);
    
    // Time of flight for minimum energy transfer
    double n = std::sqrt(math::constants::EARTH_MU / (lambert_radius * lambert_radius * lambert_radius));
    double tof = angle_separation / n;
    
    std::cout << "Initial position: (" << r1.x() / 1000.0 << ", " 
              << r1.y() / 1000.0 << ", " << r1.z() / 1000.0 << ") km\n";
    std::cout << "Final position: (" << r2.x() / 1000.0 << ", " 
              << r2.y() / 1000.0 << ", " << r2.z() / 1000.0 << ") km\n";
    std::cout << "Time of flight: " << tof / 60.0 << " minutes\n";
    
    auto solutions = propagator.solveLambertProblem(r1, r2, tof);
    
    if (!solutions.empty() && solutions[0].isValid()) {
        std::cout << "Lambert solution found:\n";
        std::cout << "  Initial velocity: (" << solutions[0].v1.x() << ", "
                  << solutions[0].v1.y() << ", " << solutions[0].v1.z() << ") m/s\n";
        std::cout << "  Transfer orbit semi-major axis: " 
                  << solutions[0].a / 1000.0 << " km\n";
        std::cout << "  Transfer orbit eccentricity: " << solutions[0].e << "\n";
    }
    
    std::cout << "\n=== End of Two-Body Dynamics Example ===\n";
    
    return 0;
}