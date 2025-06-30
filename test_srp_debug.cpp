#include <iostream>
#include <cmath>

struct Vector3D {
    double x, y, z;
    Vector3D(double x_ = 0, double y_ = 0, double z_ = 0) : x(x_), y(y_), z(z_) {}
    Vector3D operator-(const Vector3D& v) const { return Vector3D(x-v.x, y-v.y, z-v.z); }
    Vector3D operator/(double s) const { return Vector3D(x/s, y/s, z/s); }
    double dot(const Vector3D& v) const { return x*v.x + y*v.y + z*v.z; }
    double magnitude() const { return std::sqrt(x*x + y*y + z*z); }
};

double calculateCylindricalShadow(
    const Vector3D& objectPos,
    const Vector3D& sunPos,
    const Vector3D& earthPos)
{
    const double EARTH_RADIUS = 6.371e6;
    
    // Vector from Earth to object
    Vector3D earthToObject = objectPos - earthPos;
    
    // Vector from Earth to Sun
    Vector3D earthToSun = sunPos - earthPos;
    double sunDistance = earthToSun.magnitude();
    Vector3D sunDirection = earthToSun / sunDistance;
    
    // Project object position onto Sun-Earth line
    double projection = earthToObject.dot(sunDirection);
    
    std::cout << "Earth to object: (" << earthToObject.x << ", " << earthToObject.y << ", " << earthToObject.z << ")\n";
    std::cout << "Earth to Sun: (" << earthToSun.x << ", " << earthToSun.y << ", " << earthToSun.z << ")\n";
    std::cout << "Sun direction: (" << sunDirection.x << ", " << sunDirection.y << ", " << sunDirection.z << ")\n";
    std::cout << "Projection: " << projection << "\n";
    
    // Object is in shadow if behind Earth (relative to Sun)
    if (projection > 0) {
        std::cout << "Object is on Sun side (projection > 0)\n";
        return 1.0;  // Sunlit
    }
    
    // Calculate perpendicular distance to Sun-Earth line
    Vector3D perpVector = earthToObject - Vector3D(sunDirection.x * projection, sunDirection.y * projection, sunDirection.z * projection);
    double perpDistance = perpVector.magnitude();
    
    std::cout << "Perpendicular distance: " << perpDistance << " m\n";
    std::cout << "Earth radius: " << EARTH_RADIUS << " m\n";
    
    // Check if within Earth's cylindrical shadow
    if (perpDistance < EARTH_RADIUS) {
        std::cout << "Within Earth's shadow cylinder\n";
        return 0.0;  // Total eclipse
    }
    
    return 1.0;  // No eclipse
}

int main() {
    const double AU = 1.495978707e11;
    
    Vector3D earthPos(0, 0, 0);
    Vector3D sunPos(AU, 0, 0);
    
    std::cout << "Test 1: Sunlit position (42164000, 0, 0)\n";
    Vector3D sunlitPos(42164000.0, 0.0, 0.0);
    double shadow1 = calculateCylindricalShadow(sunlitPos, sunPos, earthPos);
    std::cout << "Shadow factor: " << shadow1 << " (expected: 1.0)\n\n";
    
    std::cout << "Test 2: Shadow position (-7000000, 0, 0)\n";
    Vector3D shadowPos(-7000000.0, 0.0, 0.0);
    double shadow2 = calculateCylindricalShadow(shadowPos, sunPos, earthPos);
    std::cout << "Shadow factor: " << shadow2 << " (expected: 0.0)\n\n";
    
    return 0;
}