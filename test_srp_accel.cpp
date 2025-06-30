#include <iostream>
#include <cmath>

int main() {
    // Test parameters
    double solarFlux = 1367.0;  // W/m²
    double speedOfLight = 299792458.0;  // m/s
    double cr = 1.0;  // Absorption only
    double area = 10.0;  // m²
    double mass = 1000.0;  // kg
    
    // Calculate solar pressure
    double solarPressure = solarFlux / speedOfLight;
    std::cout << "Solar flux: " << solarFlux << " W/m²\n";
    std::cout << "Speed of light: " << speedOfLight << " m/s\n";
    std::cout << "Solar pressure: " << solarPressure << " N/m²\n";
    std::cout << "Solar pressure: " << solarPressure << " Pa\n";
    
    // Calculate acceleration
    double force = solarPressure * cr * area;
    double acceleration = force / mass;
    
    std::cout << "\nForce calculation:\n";
    std::cout << "P * Cr * A = " << solarPressure << " * " << cr << " * " << area << " = " << force << " N\n";
    std::cout << "Acceleration = F/m = " << force << " / " << mass << " = " << acceleration << " m/s²\n";
    std::cout << "Expected: 4.56e-8 m/s²\n";
    std::cout << "Ratio: " << acceleration / 4.56e-8 << "\n";
    
    return 0;
}