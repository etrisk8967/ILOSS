#include <iostream>
#include <cmath>

int main() {
    // Constants
    const double EARTH_RADIUS_EQUATORIAL = 6378137.0;  // m
    const double EARTH_MU = 3.986004418e14;  // m³/s²
    
    // Create circular orbit at 300 km
    double altitude = 300000.0;  // m
    double radius = EARTH_RADIUS_EQUATORIAL + altitude;
    double orbitalSpeed = std::sqrt(EARTH_MU / radius);
    
    std::cout << "Initial circular orbit state:\n";
    std::cout << "  Altitude: " << altitude/1000 << " km\n";
    std::cout << "  Radius: " << radius/1000 << " km\n";
    std::cout << "  Orbital speed: " << orbitalSpeed << " m/s\n";
    
    // Position at ascending node
    double position_x = radius;
    double position_y = 0.0;
    double position_z = 0.0;
    
    // Velocity in orbital plane (inclination = 0)
    double velocity_x = 0.0;
    double velocity_y = orbitalSpeed;
    double velocity_z = 0.0;
    
    std::cout << "\nInitial state vectors:\n";
    std::cout << "  Position: (" << position_x << ", " << position_y << ", " << position_z << ") m\n";
    std::cout << "  Velocity: (" << velocity_x << ", " << velocity_y << ", " << velocity_z << ") m/s\n";
    
    // Check orbital elements
    double r = std::sqrt(position_x*position_x + position_y*position_y + position_z*position_z);
    double v = std::sqrt(velocity_x*velocity_x + velocity_y*velocity_y + velocity_z*velocity_z);
    
    // Specific energy
    double energy = v*v/2 - EARTH_MU/r;
    
    // Semi-major axis from vis-viva equation
    double a = -EARTH_MU / (2 * energy);
    
    std::cout << "\nOrbital verification:\n";
    std::cout << "  Position magnitude: " << r/1000 << " km\n";
    std::cout << "  Velocity magnitude: " << v << " m/s\n";
    std::cout << "  Specific energy: " << energy << " m²/s²\n";
    std::cout << "  Semi-major axis: " << a/1000 << " km\n";
    std::cout << "  Expected SMA for circular: " << radius/1000 << " km\n";
    
    // Check if velocity is perpendicular to position (circular orbit)
    double dot_product = position_x*velocity_x + position_y*velocity_y + position_z*velocity_z;
    std::cout << "  r·v (should be ~0 for circular): " << dot_product << "\n";
    
    return 0;
}