#include <iostream>
#include <cmath>

int main() {
    // Constants from MathConstants.h
    const double SEA_LEVEL_DENSITY = 1.225;              // kg/m³
    const double ATMOSPHERIC_SCALE_HEIGHT = 8500.0;      // m
    
    // Test altitudes
    double altitudes[] = {100000, 200000, 300000, 400000, 500000}; // meters
    
    std::cout << "Exponential Atmosphere Density Calculations:\n";
    std::cout << "Scale Height: " << ATMOSPHERIC_SCALE_HEIGHT << " m\n";
    std::cout << "Sea Level Density: " << SEA_LEVEL_DENSITY << " kg/m³\n\n";
    
    for (double altitude : altitudes) {
        // Exponential atmosphere: ρ = ρ₀ * exp(-h/H)
        double density = SEA_LEVEL_DENSITY * std::exp(-altitude / ATMOSPHERIC_SCALE_HEIGHT);
        
        std::cout << "Altitude: " << altitude/1000.0 << " km\n";
        std::cout << "  Density: " << density << " kg/m³\n";
        std::cout << "  Scientific: " << std::scientific << density << std::defaultfloat << "\n\n";
    }
    
    // Check drag force for typical values at 300km
    double altitude = 300000.0;
    double density = SEA_LEVEL_DENSITY * std::exp(-altitude / ATMOSPHERIC_SCALE_HEIGHT);
    double velocity = 7700.0; // typical orbital velocity at 300km
    double Cd = 2.2;
    double area = 1.0; // m²
    double mass = 100.0; // kg satellite
    
    // F_drag = 0.5 * Cd * A * ρ * v²
    double drag_force = 0.5 * Cd * area * density * velocity * velocity;
    double drag_accel = drag_force / mass;
    
    std::cout << "\nDrag calculation at 300 km:\n";
    std::cout << "  Velocity: " << velocity << " m/s\n";
    std::cout << "  Cd * A: " << Cd * area << " m²\n";
    std::cout << "  Mass: " << mass << " kg\n";
    std::cout << "  Drag Force: " << drag_force << " N\n";
    std::cout << "  Drag Acceleration: " << drag_accel << " m/s²\n";
    std::cout << "  Drag Acceleration (scientific): " << std::scientific << drag_accel << std::defaultfloat << " m/s²\n";
    
    return 0;
}