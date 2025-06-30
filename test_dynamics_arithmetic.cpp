#include <iostream>
#include <cmath>

// Simulate the issue with state arithmetic

struct SimpleState {
    double x, y, vx, vy;
    double time;
    
    SimpleState(double x_, double y_, double vx_, double vy_, double t_)
        : x(x_), y(y_), vx(vx_), vy(vy_), time(t_) {}
    
    // Addition operator that averages time (like StateVector)
    SimpleState operator+(const SimpleState& other) const {
        return SimpleState(
            x + other.x,
            y + other.y,
            vx + other.vx,
            vy + other.vy,
            (time + other.time) / 2.0  // AVERAGING TIME!
        );
    }
    
    // Scalar multiplication
    SimpleState operator*(double scalar) const {
        return SimpleState(
            x * scalar,
            y * scalar,
            vx * scalar,
            vy * scalar,
            time * scalar  // SCALING TIME!
        );
    }
    
    void setTime(double t) { time = t; }
    double getTime() const { return time; }
};

int main() {
    std::cout << "Testing state arithmetic with time averaging\n";
    std::cout << "==========================================\n\n";
    
    // Initial state at t=0
    SimpleState state(6.67814e6, 0, 0, 7725.76, 0.0);
    
    // Derivative (velocity and acceleration)
    SimpleState derivative(0, 7725.76, -7.52e-3, 0, 0.0);  // typical gravity at 300km
    
    double dt = 1.0;  // 1 second timestep
    
    std::cout << "Initial state: x=" << state.x << ", y=" << state.y 
              << ", vx=" << state.vx << ", vy=" << state.vy 
              << ", t=" << state.time << "\n";
    
    std::cout << "Derivative: dx=" << derivative.x << ", dy=" << derivative.y 
              << ", dvx=" << derivative.vx << ", dvy=" << derivative.vy 
              << ", dt=" << derivative.time << "\n\n";
    
    // RK4 step 1: k1 = dt * derivative
    SimpleState k1 = derivative * dt;
    std::cout << "k1 = derivative * dt:\n";
    std::cout << "  k1.x=" << k1.x << ", k1.y=" << k1.y 
              << ", k1.vx=" << k1.vx << ", k1.vy=" << k1.vy 
              << ", k1.time=" << k1.time << "\n\n";
    
    // RK4 step 2: temp = state + k1/2
    SimpleState temp = state + k1 * 0.5;
    std::cout << "temp = state + k1 * 0.5:\n";
    std::cout << "  temp.x=" << temp.x << ", temp.y=" << temp.y 
              << ", temp.vx=" << temp.vx << ", temp.vy=" << temp.vy 
              << ", temp.time=" << temp.time << " (SHOULD BE 0.5!)\n\n";
    
    // Now set time correctly
    temp.setTime(state.time + dt * 0.5);
    std::cout << "After setTime(0.5): temp.time=" << temp.time << "\n\n";
    
    // Check what happens with multiple additions
    SimpleState state2 = state;
    for (int i = 0; i < 5; i++) {
        SimpleState k = derivative * dt;
        state2 = state2 + k;
        std::cout << "After step " << i+1 << ": time=" << state2.time 
                  << " (expected: " << (i+1)*dt << ")\n";
    }
    
    return 0;
}