#include <iostream>
#include <cmath>

int main() {
    const double AU = 1.495978707e11;  // meters
    
    // Test satellite position at GEO
    double satX = 42164000.0;  // 42,164 km
    double satY = 0.0;
    double satZ = 0.0;
    
    // Sun position
    double sunX = AU;
    double sunY = 0.0;
    double sunZ = 0.0;
    
    // Calculate distance from satellite to Sun
    double dx = sunX - satX;
    double dy = sunY - satY;
    double dz = sunZ - satZ;
    
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    std::cout << "Satellite position: (" << satX << ", " << satY << ", " << satZ << ") m\n";
    std::cout << "Sun position: (" << sunX << ", " << sunY << ", " << sunZ << ") m\n";
    std::cout << "Distance satellite to Sun: " << distance << " m\n";
    std::cout << "Distance in AU: " << distance / AU << "\n";
    std::cout << "Ratio to 1 AU: " << distance / AU << "\n";
    
    // The distance should be very close to 1 AU since GEO radius is tiny compared to AU
    std::cout << "\nGEO radius / AU = " << satX / AU << " = " << satX / AU * 100 << "%\n";
    
    return 0;
}