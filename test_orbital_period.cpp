#include <iostream>
#include <cmath>

int main() {
    double EARTH_MU = 3.986004418e14;  // m³/s²
    double EARTH_RADIUS = 6378137.0;   // m
    
    double altitude = 500000.0; // meters
    double radius = EARTH_RADIUS + altitude;
    double period = 2.0 * M_PI * std::sqrt(std::pow(radius, 3) / EARTH_MU);
    
    std::cout << "Orbital period at 500km: " << period << " seconds\n";
    std::cout << "That's " << period / 60.0 << " minutes\n";
    std::cout << "With 10s steps, that's " << period / 10.0 << " steps\n";
    
    return 0;
}
