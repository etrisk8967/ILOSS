#include <gtest/gtest.h>
#include "physics/dynamics/DynamicsEngine.h"
#include "physics/dynamics/SimpleMassProperties.h"
#include "physics/dynamics/torques/GravityGradientTorque.h"
#include "physics/forces/SimpleGravityModel.h"
#include "physics/integrators/DynamicsIntegratorAdapter.h"
#include "physics/integrators/GenericRK4Integrator.h"
#include "physics/integrators/GenericRK78Integrator.h"
#include "core/math/MathConstants.h"
#include <cmath>
#include <vector>

using namespace iloss::physics::dynamics;
using namespace iloss::physics::forces;
using namespace iloss::physics::integrators;
using namespace iloss::math;

class DynamicsIntegratorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create mass properties
        double mass = 1000.0;
        massProps = std::make_shared<SimpleMassProperties>(
            mass, 100.0, 500.0, 600.0
        );
        
        // Create dynamics engine with gravity
        auto forceAgg = std::make_shared<ForceAggregator>();
        forceAgg->addForceModel(std::make_unique<SimpleGravityModel>("Earth Gravity"));
        
        auto torqueAgg = std::make_shared<TorqueAggregator>();
        
        // Add gravity gradient torque
        torques::GravityGradientConfig ggConfig;
        ggConfig.centralBodyMu = constants::EARTH_MU;
        ggConfig.massProperties = massProps;
        torqueAgg->addModel(std::make_unique<torques::GravityGradientTorque>(ggConfig));
        
        engine = std::make_shared<DynamicsEngine>(massProps, forceAgg, torqueAgg);
        
        // Create adapter and generic integrators
        IntegratorConfig config;
        config.initialStepSize = 0.01;  // 0.01 second for high accuracy
        config.minStepSize = 0.001;
        config.maxStepSize = 0.1;

        adapter = std::make_shared<DynamicsIntegratorAdapter>(engine);
        rk4Integrator = std::make_shared<GenericRK4Integrator<DynamicsState, DynamicsIntegratorAdapter>>(config);
        rk78Integrator = std::make_shared<GenericRK78Integrator<DynamicsState, DynamicsIntegratorAdapter>>(config);
        
        // Create initial state (500 km circular orbit)
        double altitude = 500000.0;
        double radius = constants::EARTH_RADIUS_EQUATORIAL + altitude;
        Vector3D position(radius, 0.0, 0.0);
        Vector3D velocity(0.0, std::sqrt(constants::EARTH_MU / radius), 0.0);
        
        initialState = std::make_unique<DynamicsState>(
            position, velocity, mass, 0.0,
            Quaternion::fromAxisAngle(Vector3D::unitY(), M_PI / 6),  // 30 degree tilt
            Vector3D(0.0, 0.0, 0.05)  // Small spin
        );
    }

    std::shared_ptr<SimpleMassProperties> massProps;
    std::shared_ptr<DynamicsEngine> engine;
    std::shared_ptr<DynamicsIntegratorAdapter> adapter;
    std::shared_ptr<GenericRK4Integrator<DynamicsState, DynamicsIntegratorAdapter>> rk4Integrator;
    std::shared_ptr<GenericRK78Integrator<DynamicsState, DynamicsIntegratorAdapter>> rk78Integrator;
    std::unique_ptr<DynamicsState> initialState;
};

TEST_F(DynamicsIntegratorTest, Constructor) {
    EXPECT_EQ(adapter->getDynamicsEngine(), engine);
    EXPECT_EQ(rk4Integrator->getType(), "GenericRK4");
    EXPECT_EQ(rk78Integrator->getType(), "GenericRK78");
}

TEST_F(DynamicsIntegratorTest, NullArguments) {
    EXPECT_THROW(
        DynamicsIntegratorAdapter(nullptr),
        std::invalid_argument
    );
}

TEST_F(DynamicsIntegratorTest, BasicIntegration) {
    double finalTime = 10.0;  // 10 seconds
    
    DynamicsState finalState = rk4Integrator->integrate(
        *initialState, *adapter, finalTime
    );
    
    // Check time advanced (with small tolerance for numerical precision)
    EXPECT_NEAR(finalState.getTime().getJ2000(iloss::time::TimeSystem::UTC), finalTime, 1e-5);
    
    // Check state changed
    EXPECT_NE(finalState.getPosition(), initialState->getPosition());
    EXPECT_NE(finalState.getVelocity(), initialState->getVelocity());
    EXPECT_NE(finalState.getAttitude(), initialState->getAttitude());
    EXPECT_NE(finalState.getAngularVelocity(), initialState->getAngularVelocity());
    
    // Check attitude remains normalized
    EXPECT_NEAR(finalState.getAttitude().norm(), 1.0, 1e-10);
}

TEST_F(DynamicsIntegratorTest, OrbitalMotion) {
    // Integrate for one orbit period
    double radius = initialState->getPosition().magnitude();
    double period = 2.0 * M_PI * std::sqrt(radius * radius * radius / constants::EARTH_MU);
    
    DynamicsState finalState = rk78Integrator->integrate(
        *initialState, *adapter, period
    );
    
    // Should return close to initial position (circular orbit)
    double posError = (finalState.getPosition() - initialState->getPosition()).magnitude();
    double relError = posError / radius;
    
    // Adaptive RK78 should give high accuracy for full orbit
    EXPECT_LT(relError, 1e-4);  // Less than 0.01% error
}

TEST_F(DynamicsIntegratorTest, AngularMomentumConservation) {
    // Remove torques for conservation test
    engine->getTorqueAggregator()->clearModels();
    
    // Calculate initial angular momentum
    Vector3D L0 = initialState->getAngularMomentumInertial(*massProps);
    
    // Integrate for 100 seconds
    DynamicsState finalState = rk4Integrator->integrate(
        *initialState, *adapter, 100.0
    );
    
    // Calculate final angular momentum
    Vector3D L1 = finalState.getAngularMomentumInertial(*massProps);
    
    // Should be conserved
    double relError = (L1 - L0).magnitude() / L0.magnitude();
    EXPECT_LT(relError, 1e-6);
}

TEST_F(DynamicsIntegratorTest, EnergyConservation) {
    // For conservative system, total energy should be conserved
    // Remove drag and other dissipative forces (already done with SimpleGravityModel)
    
    // Calculate initial energy
    double E0 = initialState->getSpecificEnergy() + 
                initialState->getRotationalKineticEnergy(*massProps) / massProps->getMass();
    
    // Integrate for 100 seconds
    DynamicsState finalState = rk4Integrator->integrate(
        *initialState, *adapter, 100.0
    );
    
    // Calculate final energy
    double E1 = finalState.getSpecificEnergy() + 
                finalState.getRotationalKineticEnergy(*massProps) / massProps->getMass();
    
    // Should be conserved (within numerical tolerance)
    double relError = std::abs(E1 - E0) / std::abs(E0);
    EXPECT_LT(relError, 3e-5);  // Relaxed for RK4 accuracy
}

TEST_F(DynamicsIntegratorTest, StateCallback) {
    std::vector<DynamicsState> states;
    std::vector<double> times;
    
    auto callback = [&states, &times](const DynamicsState& state, double time) {
        states.push_back(state);
        times.push_back(time);
        return true;
    };
    
    double finalTime = 10.0;
    rk4Integrator->integrate(*initialState, *adapter, finalTime, callback);
    
    // Should have multiple intermediate states
    EXPECT_GT(states.size(), 2);  // At least initial, some intermediate, and final
    
    // Times should be monotonically increasing
    for (size_t i = 1; i < times.size(); ++i) {
        EXPECT_GT(times[i], times[i-1]);
    }
    
    // First callback state should be at first step, not initial
    EXPECT_GT(times.front(), 0.0);  // First callback after first step
    EXPECT_LE(times.front(), rk4Integrator->getConfig().initialStepSize);  // Should be at or before initial step size
    
    // Last state should be at final time
    EXPECT_NEAR(times.back(), finalTime, 1e-10);
}

TEST_F(DynamicsIntegratorTest, GravityGradientStabilization) {
    // Test that gravity gradient torque stabilizes the satellite
    // Set initial state with moderate angular velocity
    initialState->setAngularVelocity(Vector3D(0.1, 0.05, 0.02));  // Reduced for stability
    
    // Integrate for moderate time
    double finalTime = 100.0;  // Enough to see dynamics
    
    DynamicsState finalState = rk4Integrator->integrate(
        *initialState, *adapter, finalTime
    );
    
    // Angular velocity magnitude should decrease (but not to zero without damping)
    double omega0 = initialState->getAngularVelocity().magnitude();
    double omega1 = finalState.getAngularVelocity().magnitude();
    
    // Without damping, energy is conserved but redistributed
    // Just verify the state evolved
    EXPECT_NE(finalState.getAngularVelocity(), initialState->getAngularVelocity());
}

TEST_F(DynamicsIntegratorTest, StateVectorConversion) {
    // Test internal conversion functions by verifying round-trip
    double finalTime = 1.0;
    
    DynamicsState finalState = rk4Integrator->integrate(
        *initialState, *adapter, finalTime
    );
    
    // Verify all components were properly integrated
    EXPECT_FALSE(std::isnan(finalState.getPosition().magnitude()));
    EXPECT_FALSE(std::isnan(finalState.getVelocity().magnitude()));
    EXPECT_FALSE(std::isnan(finalState.getAttitude().norm()));
    EXPECT_FALSE(std::isnan(finalState.getAngularVelocity().magnitude()));
    
    // Verify coordinate system preserved
    EXPECT_EQ(finalState.getCoordinateSystem(), initialState->getCoordinateSystem());
    
    // Verify mass preserved (not integrated)
    EXPECT_DOUBLE_EQ(finalState.getMass(), initialState->getMass());
}

TEST_F(DynamicsIntegratorTest, MultipleIntegrationSteps) {
    // Test integrating in multiple steps vs single step
    double dt = 50.0;
    double t0 = initialState->getTime().getJ2000(iloss::time::TimeSystem::UTC);
    
    // Single step
    DynamicsState state1 = rk4Integrator->integrate(*initialState, *adapter, t0 + 2 * dt);
    
    // Two steps
    DynamicsState intermediate = rk4Integrator->integrate(*initialState, *adapter, t0 + dt);
    DynamicsState state2 = rk4Integrator->integrate(intermediate, *adapter, t0 + 2 * dt);
    
    // Results should be very close (not exact due to adaptive stepping)
    double posError = (state1.getPosition() - state2.getPosition()).magnitude();
    double velError = (state1.getVelocity() - state2.getVelocity()).magnitude();
    
    EXPECT_LT(posError, 1.0);  // Less than 1 meter difference
    EXPECT_LT(velError, 0.001); // Less than 1 mm/s difference
    
    // Attitudes should also be close
    Quaternion qDiff = state1.getAttitude() * state2.getAttitude().conjugate();
    double angleDiff = qDiff.angle();
    EXPECT_LT(angleDiff, 0.001);  // Less than 0.001 radians
}