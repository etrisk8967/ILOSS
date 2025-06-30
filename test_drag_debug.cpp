#include <iostream>
#include <cmath>
#include <iomanip>

int main() {
    // Test if extremely small drag values could cause numerical issues
    
    const double EARTH_MU = 3.986004418e14;  // m³/s²
    const double EARTH_RADIUS = 6378137.0;    // m
    
    // Initial state
    double r = 6.67814e6;  // m
    double v = 7725.76;    // m/s
    
    // Gravity acceleration
    double a_grav = EARTH_MU / (r * r);  // magnitude
    
    // Drag parameters
    double density = 5.755666e-16;  // kg/m³
    double Cd_A_over_m = 2.2 * 1.0 / 100.0;  // Cd*A/m
    double a_drag = 0.5 * density * v * v * Cd_A_over_m;
    
    std::cout << "Acceleration magnitudes:\n";
    std::cout << std::scientific;
    std::cout << "  Gravity: " << a_grav << " m/s²\n";
    std::cout << "  Drag: " << a_drag << " m/s²\n";
    std::cout << "  Ratio (drag/gravity): " << a_drag/a_grav << "\n\n";
    
    // Check for numerical issues with adding tiny values
    double large = 1.0e10;
    double tiny = 1.0e-15;
    
    std::cout << "Numerical precision test:\n";
    std::cout << "  large = " << large << "\n";
    std::cout << "  tiny = " << tiny << "\n";
    std::cout << "  large + tiny = " << (large + tiny) << "\n";
    std::cout << "  (large + tiny) - large = " << ((large + tiny) - large) << "\n\n";
    
    // Check what happens if we accidentally flip the sign
    std::cout << "Sign error test:\n";
    std::cout << "  If drag accelerates instead of decelerates:\n";
    
    double dt = 1.0;  // 1 second
    double v_new_correct = v - a_drag * dt;  // Correct: drag reduces velocity
    double v_new_wrong = v + a_drag * dt;    // Wrong: drag increases velocity
    
    std::cout << "  Original velocity: " << v << " m/s\n";
    std::cout << "  After 1s (correct): " << v_new_correct << " m/s\n";
    std::cout << "  After 1s (wrong): " << v_new_wrong << " m/s\n";
    std::cout << "  Difference: " << (v_new_wrong - v) << " m/s\n\n";
    
    // Exponential growth calculation
    std::cout << "Exponential growth test:\n";
    std::cout << "  If there's positive feedback causing exponential growth:\n";
    
    double growth_rate = 0.1;  // 10% per second
    double r0 = 6.67814e6;     // Initial radius
    double t = 300;            // 300 seconds
    double r_exp = r0 * std::exp(growth_rate * t);
    
    std::cout << "  Initial radius: " << r0/1000 << " km\n";
    std::cout << "  Growth rate: " << growth_rate << " /s\n";
    std::cout << "  Time: " << t << " s\n";
    std::cout << "  Final radius (exponential): " << r_exp/1000 << " km\n";
    std::cout << "  Final radius (exponential): " << r_exp << " m\n";
    std::cout << "  Growth factor: " << r_exp/r0 << "\n\n";
    
    // To get to 7.2e10 km from 6678 km, we need:
    double r_final = 7.2e13;  // m
    double factor = r_final / r0;
    double implied_rate = std::log(factor) / t;
    
    std::cout << "Actual test result analysis:\n";
    std::cout << "  Initial radius: " << r0/1000 << " km\n";
    std::cout << "  Final radius: " << r_final/1000 << " km\n";
    std::cout << "  Growth factor: " << factor << "\n";
    std::cout << "  Implied growth rate: " << implied_rate << " /s\n";
    std::cout << "  This is " << implied_rate/growth_rate << "x the 10%/s rate\n";
    
    return 0;
}