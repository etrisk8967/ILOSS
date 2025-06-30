#include <iostream>
#include <cmath>
#include <vector>

struct State {
    double x, y, vx, vy;
    double time;
};

// Simple RK4 integrator
State rk4_step(const State& state, double dt, 
               double ax_func(const State&), double ay_func(const State&)) {
    // k1
    double k1_vx = ax_func(state);
    double k1_vy = ay_func(state);
    double k1_x = state.vx;
    double k1_y = state.vy;
    
    // k2
    State mid1 = {state.x + k1_x*dt/2, state.y + k1_y*dt/2,
                  state.vx + k1_vx*dt/2, state.vy + k1_vy*dt/2, state.time + dt/2};
    double k2_vx = ax_func(mid1);
    double k2_vy = ay_func(mid1);
    double k2_x = mid1.vx;
    double k2_y = mid1.vy;
    
    // k3
    State mid2 = {state.x + k2_x*dt/2, state.y + k2_y*dt/2,
                  state.vx + k2_vx*dt/2, state.vy + k2_vy*dt/2, state.time + dt/2};
    double k3_vx = ax_func(mid2);
    double k3_vy = ay_func(mid2);
    double k3_x = mid2.vx;
    double k3_y = mid2.vy;
    
    // k4
    State end = {state.x + k3_x*dt, state.y + k3_y*dt,
                 state.vx + k3_vx*dt, state.vy + k3_vy*dt, state.time + dt};
    double k4_vx = ax_func(end);
    double k4_vy = ay_func(end);
    double k4_x = end.vx;
    double k4_y = end.vy;
    
    // Combine
    State result;
    result.x = state.x + dt/6 * (k1_x + 2*k2_x + 2*k3_x + k4_x);
    result.y = state.y + dt/6 * (k1_y + 2*k2_y + 2*k3_y + k4_y);
    result.vx = state.vx + dt/6 * (k1_vx + 2*k2_vx + 2*k3_vx + k4_vx);
    result.vy = state.vy + dt/6 * (k1_vy + 2*k2_vy + 2*k3_vy + k4_vy);
    result.time = state.time + dt;
    
    return result;
}

const double EARTH_MU = 3.986004418e14;  // m³/s²
const double EARTH_RADIUS = 6378137.0;    // m

// Acceleration functions including gravity and drag
double ax_gravity_drag(const State& state) {
    double r = std::sqrt(state.x*state.x + state.y*state.y);
    
    // Gravity
    double ax_grav = -EARTH_MU * state.x / (r*r*r);
    
    // Drag (very small)
    double density = 5.755666e-16;  // Exponential model at 300km
    double v = std::sqrt(state.vx*state.vx + state.vy*state.vy);
    double Cd_A_over_m = 2.2 * 1.0 / 100.0;  // Cd*A/m
    
    double ax_drag = 0.0;
    if (v > 1e-6) {
        ax_drag = -0.5 * density * v * state.vx * Cd_A_over_m;
    }
    
    return ax_grav + ax_drag;
}

double ay_gravity_drag(const State& state) {
    double r = std::sqrt(state.x*state.x + state.y*state.y);
    
    // Gravity
    double ay_grav = -EARTH_MU * state.y / (r*r*r);
    
    // Drag (very small)
    double density = 5.755666e-16;  // Exponential model at 300km
    double v = std::sqrt(state.vx*state.vx + state.vy*state.vy);
    double Cd_A_over_m = 2.2 * 1.0 / 100.0;  // Cd*A/m
    
    double ay_drag = 0.0;
    if (v > 1e-6) {
        ay_drag = -0.5 * density * v * state.vy * Cd_A_over_m;
    }
    
    return ay_grav + ay_drag;
}

int main() {
    // Initial state: circular orbit at 300 km
    State state;
    state.x = 6.67814e6;  // m
    state.y = 0.0;
    state.vx = 0.0;
    state.vy = 7725.76;  // m/s
    state.time = 0.0;
    
    std::cout << "Testing minimal drag integration\n";
    std::cout << "================================\n\n";
    
    double dt = 1.0;  // 1 second timestep
    int steps = 300;  // 300 seconds total
    
    // Print header
    std::cout << "Time(s)  |  X(km)      |  Y(km)      |  Vx(m/s)   |  Vy(m/s)   |  R(km)     |  V(m/s)\n";
    std::cout << "---------|-------------|-------------|------------|------------|------------|------------\n";
    
    // Print initial state
    double r = std::sqrt(state.x*state.x + state.y*state.y);
    double v = std::sqrt(state.vx*state.vx + state.vy*state.vy);
    printf("%8.1f | %11.3f | %11.3f | %10.3f | %10.3f | %10.3f | %10.3f\n",
           state.time, state.x/1000, state.y/1000, state.vx, state.vy, r/1000, v);
    
    // Integrate
    for (int i = 0; i < steps; i++) {
        state = rk4_step(state, dt, ax_gravity_drag, ay_gravity_drag);
        
        // Print every 50 seconds
        if ((i+1) % 50 == 0 || i == steps-1) {
            r = std::sqrt(state.x*state.x + state.y*state.y);
            v = std::sqrt(state.vx*state.vx + state.vy*state.vy);
            
            // Check for explosion
            if (r > 1e10 || std::isnan(r) || std::isinf(r)) {
                printf("\nERROR: Integration diverged at t=%g s! r=%g km\n", state.time, r/1000);
                
                // Calculate accelerations at this point
                double ax = ax_gravity_drag(state);
                double ay = ay_gravity_drag(state);
                double a = std::sqrt(ax*ax + ay*ay);
                
                printf("Final accelerations: ax=%g, ay=%g, |a|=%g m/s²\n", ax, ay, a);
                break;
            }
            
            printf("%8.1f | %11.3f | %11.3f | %10.3f | %10.3f | %10.3f | %10.3f\n",
                   state.time, state.x/1000, state.y/1000, state.vx, state.vy, r/1000, v);
        }
    }
    
    // Calculate energy change
    double r_final = std::sqrt(state.x*state.x + state.y*state.y);
    double v_final = std::sqrt(state.vx*state.vx + state.vy*state.vy);
    double E_initial = 7725.76*7725.76/2 - EARTH_MU/6.67814e6;
    double E_final = v_final*v_final/2 - EARTH_MU/r_final;
    
    std::cout << "\nEnergy check:\n";
    std::cout << "  Initial specific energy: " << E_initial << " m²/s²\n";
    std::cout << "  Final specific energy: " << E_final << " m²/s²\n";
    std::cout << "  Energy change: " << E_final - E_initial << " m²/s²\n";
    
    return 0;
}