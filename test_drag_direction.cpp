#include <iostream>
#include <cmath>
#include <iomanip>

// Simple Vector3D class for testing
struct Vector3D {
    double x, y, z;
    
    Vector3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    
    double magnitude() const {
        return std::sqrt(x*x + y*y + z*z);
    }
    
    Vector3D normalized() const {
        double mag = magnitude();
        if (mag < 1e-15) return Vector3D(0, 0, 0);
        return Vector3D(x/mag, y/mag, z/mag);
    }
    
    Vector3D operator*(double scalar) const {
        return Vector3D(x*scalar, y*scalar, z*scalar);
    }
    
    Vector3D operator-(const Vector3D& other) const {
        return Vector3D(x-other.x, y-other.y, z-other.z);
    }
    
    double dot(const Vector3D& other) const {
        return x*other.x + y*other.y + z*other.z;
    }
    
    Vector3D cross(const Vector3D& other) const {
        return Vector3D(
            y*other.z - z*other.y,
            z*other.x - x*other.z,
            x*other.y - y*other.x
        );
    }
};

std::ostream& operator<<(std::ostream& os, const Vector3D& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

int main() {
    std::cout << "Testing Drag Force Direction\n";
    std::cout << "============================\n\n";
    
    // Earth parameters
    const double EARTH_RADIUS = 6378137.0;  // m
    const double EARTH_ROTATION_RATE = 7.2921158553e-5;  // rad/s
    
    // Satellite in circular orbit at 300 km
    double altitude = 300000.0;  // m
    double radius = EARTH_RADIUS + altitude;
    
    // Position: on equator, at prime meridian
    Vector3D position(radius, 0, 0);
    
    // Circular velocity
    double EARTH_MU = 3.986004418e14;  // m³/s²
    double orbital_speed = std::sqrt(EARTH_MU / radius);
    
    // Velocity: circular orbit in equatorial plane
    Vector3D velocity(0, orbital_speed, 0);
    
    std::cout << "Satellite state:\n";
    std::cout << "  Position: " << position << " m\n";
    std::cout << "  Radius: " << radius/1000 << " km\n";
    std::cout << "  Altitude: " << altitude/1000 << " km\n";
    std::cout << "  Velocity: " << velocity << " m/s\n";
    std::cout << "  Speed: " << velocity.magnitude() << " m/s\n\n";
    
    // Calculate atmospheric velocity due to Earth rotation
    Vector3D omega(0, 0, EARTH_ROTATION_RATE);
    Vector3D atmo_velocity = omega.cross(position);
    
    std::cout << "Atmospheric rotation:\n";
    std::cout << "  Omega: " << omega << " rad/s\n";
    std::cout << "  Atmospheric velocity: " << atmo_velocity << " m/s\n";
    std::cout << "  Atmospheric speed: " << atmo_velocity.magnitude() << " m/s\n\n";
    
    // Relative velocity
    Vector3D relative_velocity = velocity - atmo_velocity;
    double relative_speed = relative_velocity.magnitude();
    
    std::cout << "Relative velocity:\n";
    std::cout << "  Relative velocity: " << relative_velocity << " m/s\n";
    std::cout << "  Relative speed: " << relative_speed << " m/s\n\n";
    
    // Drag calculation
    double density = 2.0e-11;  // Realistic value at 300 km
    double Cd = 2.2;
    double area = 1.0;  // m²
    double mass = 100.0;  // kg
    
    double drag_magnitude = 0.5 * Cd * area * density * relative_speed * relative_speed / mass;
    Vector3D drag_acceleration = relative_velocity.normalized() * (-drag_magnitude);
    
    std::cout << "Drag calculation:\n";
    std::cout << "  Density: " << std::scientific << density << " kg/m³\n";
    std::cout << "  Cd * A: " << std::fixed << Cd * area << " m²\n";
    std::cout << "  Mass: " << mass << " kg\n";
    std::cout << "  Drag magnitude: " << std::scientific << drag_magnitude << " m/s²\n";
    std::cout << "  Drag acceleration: " << drag_acceleration << " m/s²\n\n";
    
    // Check direction
    double angle_rad = std::acos(drag_acceleration.dot(velocity) / 
                                 (drag_acceleration.magnitude() * velocity.magnitude()));
    double angle_deg = angle_rad * 180.0 / M_PI;
    
    std::cout << "Direction check:\n";
    std::cout << "  Angle between drag and velocity: " << std::fixed << angle_deg << " degrees\n";
    std::cout << "  (Should be ~180 degrees if drag opposes velocity)\n\n";
    
    // Test with unrealistic exponential density
    double exp_density = 5.755666e-16;  // Value from exponential model
    double exp_drag_magnitude = 0.5 * Cd * area * exp_density * relative_speed * relative_speed / mass;
    Vector3D exp_drag_acceleration = relative_velocity.normalized() * (-exp_drag_magnitude);
    
    std::cout << "With exponential atmosphere density:\n";
    std::cout << "  Density: " << std::scientific << exp_density << " kg/m³\n";
    std::cout << "  Drag magnitude: " << exp_drag_magnitude << " m/s²\n";
    std::cout << "  Drag acceleration: " << exp_drag_acceleration << " m/s²\n";
    
    // Check if numerical issues
    if (exp_drag_magnitude < 1e-15) {
        std::cout << "\n  WARNING: Drag magnitude is below machine epsilon!\n";
        std::cout << "  This could cause numerical instabilities.\n";
    }
    
    return 0;
}