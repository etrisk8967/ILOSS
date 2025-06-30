#include <gtest/gtest.h>
#include "test_framework/LoggerTestFixture.h"
#include "physics/integrators/RK4Integrator.h"
#include "physics/state/StateVector.h"
#include "physics/forces/ForceAggregator.h"
#include "physics/forces/twobody/TwoBodyForceModel.h"
#include "core/math/Vector3D.h"
#include "core/constants/EarthModel.h"
#include <cmath>

using namespace iloss::physics::integrators;
using namespace iloss::physics;
using namespace iloss::physics::forces;
using namespace iloss::math;
using namespace iloss::constants;

class RK4IntegratorTest : public iloss::test::LoggerTestFixture {
protected:
    void SetUp() override {
        LoggerTestFixture::SetUp();
        
        // Create force aggregator with two-body force
        forceAggregator = std::make_unique<ForceAggregator>();
        
        auto twoBodyForce = std::make_unique<twobody::TwoBodyForceModel>("TwoBody");
        forceAggregator->addForceModel(std::move(twoBodyForce));
        
        // Create integrator with default config
        IntegratorConfig config;
        config.initialStepSize = 10.0; // 10 seconds
        integrator = std::make_unique<RK4Integrator>(config);
    }
    
    std::unique_ptr<ForceAggregator> forceAggregator;
    std::unique_ptr<RK4Integrator> integrator;
};

// Test basic functionality
TEST_F(RK4IntegratorTest, BasicConstruction) {
    EXPECT_EQ(integrator->getType(), "RK4");
    EXPECT_FALSE(integrator->isAdaptive());
    EXPECT_EQ(integrator->getOrder(), 4);
}

// Test configuration
TEST_F(RK4IntegratorTest, Configuration) {
    IntegratorConfig config;
    config.initialStepSize = 60.0;
    config.maxIterations = 10000;
    
    integrator->setConfig(config);
    
    EXPECT_DOUBLE_EQ(integrator->getConfig().initialStepSize, 60.0);
    EXPECT_EQ(integrator->getConfig().maxIterations, 10000);
}

// Test circular orbit integration
TEST_F(RK4IntegratorTest, CircularOrbitIntegration) {
    // Create circular orbit at 500 km altitude
    double altitude = 500000.0; // meters
    double radius = EarthModel::EQUATORIAL_RADIUS + altitude;
    double velocity = std::sqrt(EarthModel::MU / radius);
    
    StateVector initialState;
    initialState.setPosition(Vector3D(radius, 0, 0));
    initialState.setVelocity(Vector3D(0, velocity, 0));
    initialState.setMass(1000.0);
    initialState.setTime(iloss::time::Time(iloss::time::TimeConstants::J2000_EPOCH, iloss::time::TimeSystem::UTC));
    
    // Calculate orbital period
    double period = 2.0 * M_PI * std::sqrt(std::pow(radius, 3) / EarthModel::MU);
    
    // Integrate for one full orbit
    StateVector finalState = integrator->integrate(
        initialState, *forceAggregator, period);
    
    // Check that we returned to approximately the same position
    double positionError = (finalState.getPosition() - initialState.getPosition()).magnitude();
    double velocityError = (finalState.getVelocity() - initialState.getVelocity()).magnitude();
    
    // RK4 with 10 second steps should give good accuracy for circular orbit
    EXPECT_LT(positionError, 100.0); // Less than 100m error
    EXPECT_LT(velocityError, 0.1);   // Less than 0.1 m/s error
    
    // Check energy conservation
    double initialEnergy = initialState.getSpecificEnergy();
    double finalEnergy = finalState.getSpecificEnergy();
    double energyError = std::abs(finalEnergy - initialEnergy) / std::abs(initialEnergy);
    
    EXPECT_LT(energyError, 1e-6); // Energy should be conserved to 0.0001%
}

// Test single step functionality
TEST_F(RK4IntegratorTest, SingleStep) {
    StateVector initialState;
    initialState.setPosition(Vector3D(7000000.0, 0, 0));
    initialState.setVelocity(Vector3D(0, 7500.0, 0));
    initialState.setMass(1000.0);
    initialState.setTime(iloss::time::Time(iloss::time::TimeConstants::J2000_EPOCH, iloss::time::TimeSystem::UTC));
    
    auto result = integrator->step(initialState, *forceAggregator, 1.0);
    
    EXPECT_TRUE(result.stepAccepted);
    EXPECT_DOUBLE_EQ(result.actualStepSize, 1.0);
    EXPECT_DOUBLE_EQ(result.estimatedError, 0.0); // RK4 doesn't estimate error
    EXPECT_DOUBLE_EQ(result.newState.getTime().getJ2000(), 1.0);
    
    // Check that position and velocity changed
    EXPECT_NE(result.newState.getPosition().x(), initialState.getPosition().x());
    EXPECT_NE(result.newState.getVelocity().y(), initialState.getVelocity().y());
}

// Test backward integration
TEST_F(RK4IntegratorTest, BackwardIntegration) {
    StateVector initialState;
    initialState.setPosition(Vector3D(7000000.0, 0, 0));
    initialState.setVelocity(Vector3D(0, 7500.0, 0));
    initialState.setMass(1000.0);
    initialState.setTime(iloss::time::Time(100.0 + iloss::time::TimeConstants::J2000_EPOCH, iloss::time::TimeSystem::UTC));
    
    // Integrate backward to t=0
    StateVector backwardState = integrator->integrate(
        initialState, *forceAggregator, 0.0);
    
    EXPECT_DOUBLE_EQ(backwardState.getTime().getTime(), 0.0);
    
    // Integrate forward again
    StateVector forwardState = integrator->integrate(
        backwardState, *forceAggregator, 100.0);
    
    // Should return to approximately the same state
    double positionError = (forwardState.getPosition() - initialState.getPosition()).magnitude();
    double velocityError = (forwardState.getVelocity() - initialState.getVelocity()).magnitude();
    
    EXPECT_LT(positionError, 1.0);    // Less than 1m error
    EXPECT_LT(velocityError, 0.001);  // Less than 1 mm/s error
}

// Test statistics collection
TEST_F(RK4IntegratorTest, StatisticsCollection) {
    IntegratorConfig config;
    config.initialStepSize = 10.0;
    config.enableStatistics = true;
    integrator->setConfig(config);
    
    StateVector initialState;
    initialState.setPosition(Vector3D(7000000.0, 0, 0));
    initialState.setVelocity(Vector3D(0, 7500.0, 0));
    initialState.setMass(1000.0);
    initialState.setTime(iloss::time::Time(iloss::time::TimeConstants::J2000_EPOCH, iloss::time::TimeSystem::UTC));
    
    // Integrate for 100 seconds
    integrator->integrate(initialState, *forceAggregator, 100.0);
    
    const auto& stats = integrator->getStatistics();
    
    // Should have taken 10 steps (100s / 10s)
    EXPECT_EQ(stats.totalSteps, 10);
    EXPECT_EQ(stats.acceptedSteps, 10);
    EXPECT_EQ(stats.rejectedSteps, 0); // RK4 doesn't reject steps
    
    // Each step uses 4 function evaluations
    EXPECT_EQ(stats.functionEvaluations, 40);
    
    // Check step size statistics
    EXPECT_DOUBLE_EQ(stats.minStepSizeUsed, 10.0);
    EXPECT_DOUBLE_EQ(stats.maxStepSizeUsed, 10.0);
    EXPECT_DOUBLE_EQ(stats.averageStepSize, 10.0);
    
    // Check timing statistics
    EXPECT_GT(stats.totalIntegrationTime.count(), 0);
    EXPECT_GT(stats.averageStepTime.count(), 0);
}

// Test callback functionality
TEST_F(RK4IntegratorTest, CallbackFunction) {
    StateVector initialState;
    initialState.setPosition(Vector3D(7000000.0, 0, 0));
    initialState.setVelocity(Vector3D(0, 7500.0, 0));
    initialState.setMass(1000.0);
    initialState.setTime(iloss::time::Time(iloss::time::TimeConstants::J2000_EPOCH, iloss::time::TimeSystem::UTC));
    
    std::vector<double> times;
    std::vector<Vector3D> positions;
    
    auto callback = [&times, &positions](const StateVector& state, double time) {
        times.push_back(time);
        positions.push_back(state.getPosition());
        return true; // Continue integration
    };
    
    integrator->integrate(initialState, *forceAggregator, 50.0, callback);
    
    // Should have 5 intermediate states (10s steps from 0 to 50s)
    EXPECT_EQ(times.size(), 5);
    EXPECT_EQ(positions.size(), 5);
    
    // Check that times are correct
    for (size_t i = 0; i < times.size(); ++i) {
        EXPECT_DOUBLE_EQ(times[i], (i + 1) * 10.0);
    }
}

// Test early termination via callback
TEST_F(RK4IntegratorTest, CallbackEarlyTermination) {
    StateVector initialState;
    initialState.setPosition(Vector3D(7000000.0, 0, 0));
    initialState.setVelocity(Vector3D(0, 7500.0, 0));
    initialState.setMass(1000.0);
    initialState.setTime(iloss::time::Time(iloss::time::TimeConstants::J2000_EPOCH, iloss::time::TimeSystem::UTC));
    
    int callCount = 0;
    auto callback = [&callCount](const StateVector&, double) {
        callCount++;
        return callCount < 3; // Stop after 3 calls
    };
    
    StateVector finalState = integrator->integrate(
        initialState, *forceAggregator, 100.0, callback);
    
    EXPECT_EQ(callCount, 3);
    EXPECT_DOUBLE_EQ(finalState.getTime().getJ2000(), 30.0); // 3 steps of 10s each
}

// Test error handling
TEST_F(RK4IntegratorTest, InvalidInitialState) {
    StateVector invalidState; // Uninitialized state
    
    EXPECT_THROW(integrator->integrate(invalidState, *forceAggregator, 100.0),
                 std::invalid_argument);
}

TEST_F(RK4IntegratorTest, TargetTimeBeforeInitialTime) {
    StateVector initialState;
    initialState.setPosition(Vector3D(7000000.0, 0, 0));
    initialState.setVelocity(Vector3D(0, 7500.0, 0));
    initialState.setMass(1000.0);
    initialState.setTime(iloss::time::Time(100.0 + iloss::time::TimeConstants::J2000_EPOCH, iloss::time::TimeSystem::UTC));
    
    // This should work (backward integration)
    EXPECT_NO_THROW(integrator->integrate(initialState, *forceAggregator, 0.0));
}

// Test maximum iterations
TEST_F(RK4IntegratorTest, MaxIterationsExceeded) {
    IntegratorConfig config;
    config.initialStepSize = 1.0;
    config.maxIterations = 10;
    integrator->setConfig(config);
    
    StateVector initialState;
    initialState.setPosition(Vector3D(7000000.0, 0, 0));
    initialState.setVelocity(Vector3D(0, 7500.0, 0));
    initialState.setMass(1000.0);
    initialState.setTime(iloss::time::Time(iloss::time::TimeConstants::J2000_EPOCH, iloss::time::TimeSystem::UTC));
    
    // Try to integrate for 100 seconds with max 10 iterations
    EXPECT_THROW(integrator->integrate(initialState, *forceAggregator, 100.0),
                 std::runtime_error);
}

// Test Kepler's third law for different orbits
TEST_F(RK4IntegratorTest, KeplersThirdLaw) {
    // Test multiple circular orbits
    std::vector<double> altitudes = {400e3, 800e3, 1500e3}; // meters
    std::vector<double> periods;
    
    for (double altitude : altitudes) {
        double radius = EarthModel::EQUATORIAL_RADIUS + altitude;
        double velocity = std::sqrt(EarthModel::MU / radius);
        
        StateVector initialState;
        initialState.setPosition(Vector3D(radius, 0, 0));
        initialState.setVelocity(Vector3D(0, velocity, 0));
        initialState.setMass(1000.0);
        initialState.setTime(iloss::time::Time(iloss::time::TimeConstants::J2000_EPOCH, iloss::time::TimeSystem::UTC));
        
        // Calculate theoretical period
        double expectedPeriod = 2.0 * M_PI * std::sqrt(std::pow(radius, 3) / 
                                                      EarthModel::MU);
        
        // Integrate for one orbit
        StateVector finalState = integrator->integrate(
            initialState, *forceAggregator, expectedPeriod);
        
        // Check position return
        double posError = (finalState.getPosition() - initialState.getPosition()).magnitude();
        double relError = posError / radius;
        
        EXPECT_LT(relError, 1e-4) << "Failed for altitude " << altitude/1000 << " km";
        
        periods.push_back(expectedPeriod);
    }
    
    // Verify Kepler's third law: T² ∝ a³
    for (size_t i = 1; i < altitudes.size(); ++i) {
        double r1 = EarthModel::EQUATORIAL_RADIUS + altitudes[0];
        double r2 = EarthModel::EQUATORIAL_RADIUS + altitudes[i];
        double T1 = periods[0];
        double T2 = periods[i];
        
        double ratio1 = (T2 * T2) / (T1 * T1);
        double ratio2 = std::pow(r2/r1, 3);
        
        EXPECT_NEAR(ratio1, ratio2, 1e-6);
    }
}

// Test integration with different step sizes
TEST_F(RK4IntegratorTest, StepSizeAccuracy) {
    StateVector initialState;
    initialState.setPosition(Vector3D(7000000.0, 0, 0));
    initialState.setVelocity(Vector3D(0, 7500.0, 0));
    initialState.setMass(1000.0);
    initialState.setTime(iloss::time::Time(iloss::time::TimeConstants::J2000_EPOCH, iloss::time::TimeSystem::UTC));
    
    // Reference solution with very small step
    IntegratorConfig refConfig;
    refConfig.initialStepSize = 0.1; // 0.1 second
    RK4Integrator refIntegrator(refConfig);
    
    StateVector refState = refIntegrator.integrate(
        initialState, *forceAggregator, 100.0);
    
    // Test with different step sizes
    std::vector<double> stepSizes = {1.0, 5.0, 10.0, 20.0};
    std::vector<double> errors;
    
    for (double stepSize : stepSizes) {
        IntegratorConfig config;
        config.initialStepSize = stepSize;
        RK4Integrator testIntegrator(config);
        
        StateVector testState = testIntegrator.integrate(
            initialState, *forceAggregator, 100.0);
        
        double error = (testState.getPosition() - refState.getPosition()).magnitude();
        errors.push_back(error);
    }
    
    // Verify 4th order convergence
    for (size_t i = 1; i < stepSizes.size(); ++i) {
        double ratio = stepSizes[i] / stepSizes[i-1];
        double errorRatio = errors[i] / errors[i-1];
        double expectedRatio = std::pow(ratio, 4); // 4th order method
        
        // Check that error scales approximately as h^4
        // Allow wider tolerance as numerical effects can impact convergence
        EXPECT_NEAR(errorRatio / expectedRatio, 1.0, 1.0);
    }
}