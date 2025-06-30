#include <iostream>
#include <cmath>
#include <iomanip>

int main() {
    const double AU = 1.495978707e11;  // meters
    
    // Test case 1: At 1 AU
    std::cout << "=== Test at 1 AU ===" << std::endl;
    
    // Satellite position along Y axis at 1 AU
    double satY = 1.496e11;  // 1 AU
    double satX = 0.0;
    double satZ = 0.0;
    
    // Sun position along +X axis at 1 AU
    double sunX = AU;
    double sunY = 0.0;
    double sunZ = 0.0;
    
    // Vector from satellite to Sun
    double toSunX = sunX - satX;
    double toSunY = sunY - satY;
    double toSunZ = sunZ - satZ;
    
    double sunDistance = std::sqrt(toSunX*toSunX + toSunY*toSunY + toSunZ*toSunZ);
    
    std::cout << "Satellite position: (" << satX << ", " << satY << ", " << satZ << ")" << std::endl;
    std::cout << "Sun position: (" << sunX << ", " << sunY << ", " << sunZ << ")" << std::endl;
    std::cout << "Sun distance: " << sunDistance << " m = " << sunDistance/AU << " AU" << std::endl;
    
    // Sun direction (unit vector)
    double sunDirX = toSunX / sunDistance;
    double sunDirY = toSunY / sunDistance;
    double sunDirZ = toSunZ / sunDistance;
    
    std::cout << "Sun direction: (" << sunDirX << ", " << sunDirY << ", " << sunDirZ << ")" << std::endl;
    
    // Solar flux at this distance
    double solarFluxAU = 1367.0;  // W/m² at 1 AU
    double ratio = AU / sunDistance;
    double solarFlux = solarFluxAU * ratio * ratio;
    
    std::cout << "Distance ratio (AU/r): " << ratio << std::endl;
    std::cout << "Solar flux: " << solarFlux << " W/m²" << std::endl;
    
    // Solar pressure
    double speedOfLight = 299792458.0;
    double solarPressure = solarFlux / speedOfLight;
    
    std::cout << "Solar pressure: " << std::scientific << solarPressure << " N/m²" << std::endl;
    
    // Surface normal (should be set to face Sun)
    double surfNormX = sunDirX;
    double surfNormY = sunDirY;
    double surfNormZ = sunDirZ;
    
    // Effective area
    double area = 10.0;  // m²
    double cosTheta = sunDirX*surfNormX + sunDirY*surfNormY + sunDirZ*surfNormZ;
    double effectiveArea = area * cosTheta;
    
    std::cout << "Cos(theta): " << cosTheta << std::endl;
    std::cout << "Effective area: " << effectiveArea << " m²" << std::endl;
    
    // Area to mass ratio
    double mass = 1000.0;  // kg
    double areaToMass = effectiveArea / mass;
    
    std::cout << "Area/mass: " << areaToMass << " m²/kg" << std::endl;
    
    // Acceleration magnitude
    double cr = 1.0;  // reflectivity coefficient
    double shadowFactor = 1.0;  // no shadow
    double accelMag = solarPressure * cr * areaToMass * shadowFactor;
    
    std::cout << "Acceleration magnitude: " << std::scientific << accelMag << " m/s²" << std::endl;
    std::cout << "Acceleration magnitude: " << std::fixed << std::setprecision(10) << accelMag << " m/s²" << std::endl;
    
    // Acceleration vector (opposite to Sun direction)
    double accelX = -sunDirX * accelMag;
    double accelY = -sunDirY * accelMag;
    double accelZ = -sunDirZ * accelMag;
    
    std::cout << "Acceleration vector: (" << std::scientific 
              << accelX << ", " << accelY << ", " << accelZ << ")" << std::endl;
    
    double accelTotal = std::sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
    std::cout << "Acceleration total magnitude: " << accelTotal << " m/s²" << std::endl;
    
    return 0;
}