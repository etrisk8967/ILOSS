#include <iostream>
#include <cmath>

const double DEG_TO_RAD = M_PI / 180.0;
const double AU = 1.495978707e11;

int main() {
    // J2000 epoch is JD 2451545.0
    double jd = 2451545.0;
    double T = (jd - 2451545.0) / 36525.0;  // Julian centuries from J2000
    
    std::cout << "T = " << T << " (should be 0 for J2000)\n";
    
    // Mean longitude of Sun
    double L = (280.46646 + 36000.76983 * T) * DEG_TO_RAD;
    std::cout << "L = " << L << " rad = " << L / DEG_TO_RAD << " deg\n";
    
    // Mean anomaly
    double M = (357.52911 + 35999.05029 * T) * DEG_TO_RAD;
    std::cout << "M = " << M << " rad = " << M / DEG_TO_RAD << " deg\n";
    
    // Equation of center
    double C = (1.914602 - 0.004817 * T) * std::sin(M) * DEG_TO_RAD;
    std::cout << "C = " << C << " rad = " << C / DEG_TO_RAD << " deg\n";
    
    // True longitude
    double lambda = L + C;
    std::cout << "lambda = " << lambda << " rad = " << lambda / DEG_TO_RAD << " deg\n";
    
    // Distance (simplified)
    double r = AU * (1.000001018 * (1 - 0.01671 * std::cos(M + C)));
    std::cout << "r = " << r << " m = " << r/AU << " AU\n";
    
    // Ecliptic to equatorial (simplified, ignoring nutation)
    double eps = 23.439291 * DEG_TO_RAD;  // Obliquity
    
    double x = r * std::cos(lambda);
    double y = r * std::sin(lambda) * std::cos(eps);
    double z = r * std::sin(lambda) * std::sin(eps);
    
    std::cout << "\nSun position at J2000:\n";
    std::cout << "X = " << x << " m\n";
    std::cout << "Y = " << y << " m\n";
    std::cout << "Z = " << z << " m\n";
    
    double mag = std::sqrt(x*x + y*y + z*z);
    std::cout << "\nMagnitude = " << mag << " m = " << mag/AU << " AU\n";
    
    // Unit vector
    std::cout << "\nSun unit vector: (" << x/mag << ", " << y/mag << ", " << z/mag << ")\n";
    
    return 0;
}