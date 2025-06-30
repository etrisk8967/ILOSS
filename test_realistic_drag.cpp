#include <iostream>
#include <iomanip>
#include <cmath>

// More realistic atmospheric density model for LEO
double getRealisticDensity(double altitude_m) {
    double h = altitude_m / 1000.0;  // Convert to km
    
    // Based on approximate values from NRLMSISE-00 for solar minimum conditions
    if (h < 100) {
        // Below 100 km, use exponential with 8.5 km scale height
        return 1.225 * exp(-altitude_m / 8500.0);
    } else if (h < 150) {
        // 100-150 km transition region
        double rho_100 = 5.0e-7;
        double H = 20000;  // 20 km scale height
        return rho_100 * exp(-(altitude_m - 100000) / H);
    } else if (h < 200) {
        // 150-200 km
        double rho_150 = 2.0e-9;
        double H = 40000;  // 40 km scale height
        return rho_150 * exp(-(altitude_m - 150000) / H);
    } else if (h < 300) {
        // 200-300 km
        double rho_200 = 2.5e-10;
        double H = 60000;  // 60 km scale height
        return rho_200 * exp(-(altitude_m - 200000) / H);
    } else if (h < 400) {
        // 300-400 km
        double rho_300 = 2.0e-11;
        double H = 70000;  // 70 km scale height
        return rho_300 * exp(-(altitude_m - 300000) / H);
    } else if (h < 500) {
        // 400-500 km
        double rho_400 = 3.7e-12;
        double H = 80000;  // 80 km scale height
        return rho_400 * exp(-(altitude_m - 400000) / H);
    } else {
        // Above 500 km
        double rho_500 = 1.0e-12;
        double H = 100000;  // 100 km scale height
        return rho_500 * exp(-(altitude_m - 500000) / H);
    }
}

int main() {
    std::cout << "Realistic Atmospheric Density Model (approximating NRLMSISE-00):\n\n";
    
    // Test altitudes
    double altitudes[] = {100, 150, 200, 250, 300, 350, 400, 450, 500}; // km
    
    std::cout << "Altitude (km) | Density (kg/m³) | Scale Height (km)\n";
    std::cout << "------------- | --------------- | -----------------\n";
    
    for (double h_km : altitudes) {
        double h_m = h_km * 1000;
        double density = getRealisticDensity(h_m);
        
        // Calculate local scale height
        double h2_m = h_m + 1000;  // 1 km higher
        double density2 = getRealisticDensity(h2_m);
        double scale_height = -1000 / log(density2 / density);
        
        std::cout << std::fixed << std::setw(13) << h_km << " | " 
                  << std::scientific << std::setw(15) << density << " | "
                  << std::fixed << std::setw(17) << scale_height/1000 << "\n";
    }
    
    // Check drag at 300 km with realistic density
    std::cout << "\n\nDrag calculation at 300 km with realistic density:\n";
    double altitude = 300000.0;  // 300 km in meters
    double density = getRealisticDensity(altitude);
    double velocity = 7700.0;    // typical orbital velocity at 300km
    double Cd = 2.2;
    double area = 1.0;           // m²
    double mass = 100.0;         // kg satellite
    
    // F_drag = 0.5 * Cd * A * ρ * v²
    double drag_force = 0.5 * Cd * area * density * velocity * velocity;
    double drag_accel = drag_force / mass;
    
    std::cout << "  Altitude: " << altitude/1000 << " km\n";
    std::cout << "  Density: " << std::scientific << density << " kg/m³\n";
    std::cout << "  Velocity: " << std::fixed << velocity << " m/s\n";
    std::cout << "  Cd * A: " << Cd * area << " m²\n";
    std::cout << "  Mass: " << mass << " kg\n";
    std::cout << "  Drag Force: " << std::scientific << drag_force << " N\n";
    std::cout << "  Drag Acceleration: " << drag_accel << " m/s²\n";
    
    // Compare with exponential model
    double exp_density = 1.225 * exp(-altitude / 8500.0);
    double exp_drag_force = 0.5 * Cd * area * exp_density * velocity * velocity;
    double exp_drag_accel = exp_drag_force / mass;
    
    std::cout << "\nComparison with simple exponential model (H=8.5km):\n";
    std::cout << "  Density: " << std::scientific << exp_density << " kg/m³\n";
    std::cout << "  Drag Acceleration: " << exp_drag_accel << " m/s²\n";
    std::cout << "  Ratio (realistic/exponential): " << density/exp_density << "\n";
    
    return 0;
}