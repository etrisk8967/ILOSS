/**
 * @file main.cpp
 * @brief Main entry point for the Integrated Launch and Orbit Simulation System
 * @author ILOSS Development Team
 * @date 2025
 */

#include <iostream>
#include <iomanip>
#include <string>
#include "core/math/Vector3D.h"
#include "core/math/Matrix3D.h"
#include "core/math/Quaternion.h"
#include "core/math/CoordinateTransforms.h"
#include "core/math/MathConstants.h"

using namespace iloss::math;

int main(int argc, char* argv[])
{
    std::cout << "Integrated Launch and Orbit Simulation System (ILOSS) v1.0.0\n";
    std::cout << "Copyright (c) 2025 - MIT License\n\n";
    
    if (argc > 1) {
        std::string arg(argv[1]);
        if (arg == "--version" || arg == "-v") {
            std::cout << "Version: 1.0.0\n";
            return 0;
        }
        if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: iloss [options]\n";
            std::cout << "Options:\n";
            std::cout << "  -h, --help     Show this help message\n";
            std::cout << "  -v, --version  Show version information\n";
            std::cout << "  --demo         Run math library demonstration\n";
            return 0;
        }
        if (arg == "--demo") {
            std::cout << "Math Library Demonstration\n";
            std::cout << "=========================\n\n";
            
            // Demonstrate Vector3D
            std::cout << "Vector3D Operations:\n";
            Vector3D v1(1.0, 2.0, 3.0);
            Vector3D v2(4.0, 5.0, 6.0);
            std::cout << "  v1 = " << v1 << "\n";
            std::cout << "  v2 = " << v2 << "\n";
            std::cout << "  v1 · v2 = " << v1.dot(v2) << "\n";
            std::cout << "  v1 × v2 = " << v1.cross(v2) << "\n";
            std::cout << "  |v1| = " << std::fixed << std::setprecision(3) << v1.magnitude() << "\n\n";
            
            // Demonstrate Matrix3D
            std::cout << "Matrix3D Operations:\n";
            Matrix3D rotZ = Matrix3D::rotationZ(constants::HALF_PI);
            std::cout << "  90° rotation around Z:\n" << rotZ << "\n";
            Vector3D rotated = rotZ * Vector3D::unitX();
            std::cout << "  Rotating unit X: " << rotated << " (should be unit Y)\n\n";
            
            // Demonstrate Quaternion
            std::cout << "Quaternion Operations:\n";
            Quaternion q = Quaternion::fromAxisAngle(Vector3D::unitZ(), constants::HALF_PI);
            std::cout << "  90° rotation quaternion: " << q << "\n";
            Vector3D qRotated = q.rotate(Vector3D::unitX());
            std::cout << "  Rotating unit X: " << qRotated << " (should be unit Y)\n\n";
            
            // Demonstrate Coordinate Transforms
            std::cout << "Coordinate Transformations:\n";
            double lat = constants::degreesToRadians(28.5);   // Cape Canaveral latitude
            double lon = constants::degreesToRadians(-80.6);  // Cape Canaveral longitude
            double alt = 0.0;                                  // Sea level
            
            Vector3D ecef = CoordinateTransforms::geodeticToECEF(lat, lon, alt);
            std::cout << "  Cape Canaveral ECEF: " << ecef << " meters\n";
            std::cout << "  Distance from center: " << ecef.magnitude() / 1000.0 << " km\n\n";
            
            // Physical Constants
            std::cout << "Physical Constants:\n";
            std::cout << "  Earth radius (equatorial): " << constants::EARTH_RADIUS_EQUATORIAL / 1000.0 << " km\n";
            std::cout << "  Earth μ (GM): " << std::scientific << constants::EARTH_MU << " m³/s²\n";
            std::cout << "  Earth rotation rate: " << std::fixed << std::setprecision(2) 
                      << constants::EARTH_ROTATION_RATE * constants::RAD_TO_DEG * 3600 << " deg/hr\n";
            std::cout << "  Standard gravity: " << constants::STANDARD_GRAVITY << " m/s²\n\n";
            
            return 0;
        }
    }
    
    std::cout << "ILOSS is under development. Core math library completed (Task 5).\n";
    std::cout << "Try 'iloss --demo' to see math library demonstration.\n";
    
    return 0;
}