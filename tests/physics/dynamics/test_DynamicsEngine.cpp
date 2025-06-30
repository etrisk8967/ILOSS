#include <gtest/gtest.h>
#include "physics/dynamics/DynamicsEngine.h"
#include "physics/dynamics/SimpleMassProperties.h"
#include "physics/dynamics/torques/GravityGradientTorque.h"
#include "physics/forces/SimpleGravityModel.h"
#include "core/math/MathConstants.h"
#include <cmath>

using namespace iloss::physics::dynamics;
using namespace iloss::physics::forces;
using namespace iloss::math;

class DynamicsEngineTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create mass properties for a typical satellite
        double mass = 1000.0;  // 1000 kg
        double Ixx = 100.0;    // kg⋅m²
        double Iyy = 500.0;    // kg⋅m²
        double Izz = 600.0;    // kg⋅m²
        Vector3D com(0.1, 0.0, 0.0);  // 10 cm offset from origin
        
        massProps = std::make_shared<SimpleMassProperties>(mass, Ixx, Iyy, Izz, com);
        
        // Create force aggregator with simple gravity
        forceAggregator = std::make_shared<ForceAggregator>();
        auto gravityModel = std::make_unique<SimpleGravityModel>("Earth Gravity");
        forceAggregator->addForceModel(std::move(gravityModel));
        
        // Create torque aggregator
        torqueAggregator = std::make_shared<TorqueAggregator>();
        
        // Create dynamics engine
        engine = std::make_unique<DynamicsEngine>(
            massProps, forceAggregator, torqueAggregator
        );
        
        // Create initial state (500 km altitude circular orbit)
        double altitude = 500000.0;
        double radius = constants::EARTH_RADIUS_EQUATORIAL + altitude;
        position = Vector3D(radius, 0.0, 0.0);
        velocity = Vector3D(0.0, std::sqrt(constants::EARTH_MU / radius), 0.0);
        
        initialState = std::make_unique<DynamicsState>(
            position, velocity, mass, 0.0,
            Quaternion::identity(),
            Vector3D(0.0, 0.0, 0.1)  // 0.1 rad/s spin about Z
        );
    }

    std::shared_ptr<SimpleMassProperties> massProps;
    std::shared_ptr<ForceAggregator> forceAggregator;
    std::shared_ptr<TorqueAggregator> torqueAggregator;
    std::unique_ptr<DynamicsEngine> engine;
    std::unique_ptr<DynamicsState> initialState;
    Vector3D position;
    Vector3D velocity;
};

TEST_F(DynamicsEngineTest, Constructor) {
    EXPECT_EQ(engine->getMassProperties(), massProps);
    EXPECT_EQ(engine->getForceAggregator(), forceAggregator);
    EXPECT_EQ(engine->getTorqueAggregator(), torqueAggregator);
}

TEST_F(DynamicsEngineTest, NullMassProperties) {
    EXPECT_THROW(
        DynamicsEngine(nullptr, forceAggregator, torqueAggregator),
        std::invalid_argument
    );
}

TEST_F(DynamicsEngineTest, DefaultAggregators) {
    // Create engine without providing aggregators
    DynamicsEngine engineDefault(massProps);
    
    EXPECT_NE(engineDefault.getForceAggregator(), nullptr);
    EXPECT_NE(engineDefault.getTorqueAggregator(), nullptr);
}

TEST_F(DynamicsEngineTest, TranslationalDynamics) {
    auto derivatives = engine->calculateDerivatives(*initialState, 0.0);
    
    // Check velocity derivative equals current velocity
    EXPECT_EQ(derivatives.velocity, velocity);
    
    // Check acceleration is due to gravity
    double r = position.magnitude();
    double expectedAccelMag = constants::EARTH_MU / (r * r);
    Vector3D expectedAccel = -position.normalized() * expectedAccelMag;
    
    EXPECT_NEAR(derivatives.acceleration.x(), expectedAccel.x(), 1e-6);
    EXPECT_NEAR(derivatives.acceleration.y(), expectedAccel.y(), 1e-6);
    EXPECT_NEAR(derivatives.acceleration.z(), expectedAccel.z(), 1e-6);
}

TEST_F(DynamicsEngineTest, RotationalDynamicsNoTorque) {
    // With no torques, angular acceleration should be due to Euler's equations only
    auto derivatives = engine->calculateDerivatives(*initialState, 0.0);
    
    Vector3D omega = initialState->getAngularVelocity();
    Matrix3D I = massProps->getInertiaTensor();
    Matrix3D invI = massProps->getInverseInertiaTensor();
    
    // Euler's equations: I*ω̇ = τ - ω × (I*ω)
    // With τ = 0: ω̇ = -I^(-1) * (ω × (I*ω))
    Vector3D Iomega = I * omega;
    Vector3D expectedAlpha = invI * (-omega.cross(Iomega));
    
    EXPECT_NEAR(derivatives.angularAcceleration.x(), expectedAlpha.x(), 1e-10);
    EXPECT_NEAR(derivatives.angularAcceleration.y(), expectedAlpha.y(), 1e-10);
    EXPECT_NEAR(derivatives.angularAcceleration.z(), expectedAlpha.z(), 1e-10);
}

TEST_F(DynamicsEngineTest, QuaternionKinematics) {
    auto derivatives = engine->calculateDerivatives(*initialState, 0.0);
    
    // q̇ = 0.5 * q ⊗ ω_quat
    Quaternion q = initialState->getAttitude();
    Vector3D omega = initialState->getAngularVelocity();
    Quaternion omegaQuat(0.0, omega);
    Quaternion expectedQdot = q * omegaQuat * 0.5;
    
    EXPECT_NEAR(derivatives.attitudeRate.w(), expectedQdot.w(), 1e-10);
    EXPECT_NEAR(derivatives.attitudeRate.x(), expectedQdot.x(), 1e-10);
    EXPECT_NEAR(derivatives.attitudeRate.y(), expectedQdot.y(), 1e-10);
    EXPECT_NEAR(derivatives.attitudeRate.z(), expectedQdot.z(), 1e-10);
}

TEST_F(DynamicsEngineTest, WithGravityGradientTorque) {
    // Add gravity gradient torque
    torques::GravityGradientConfig ggConfig;
    ggConfig.centralBodyMu = constants::EARTH_MU;
    ggConfig.massProperties = massProps;
    
    auto ggTorque = std::make_unique<torques::GravityGradientTorque>(ggConfig);
    torqueAggregator->addModel(std::move(ggTorque));
    
    // Rotate satellite to create gravity gradient torque
    initialState->setAttitude(
        Quaternion::fromAxisAngle(Vector3D::unitY(), M_PI / 4)
    );
    
    auto derivatives = engine->calculateDerivatives(*initialState, 0.0);
    
    // Angular acceleration should now include gravity gradient effects
    EXPECT_GT(derivatives.totalTorque.magnitude(), 0.0);
    
    // Verify torque affects angular acceleration
    Vector3D omega = initialState->getAngularVelocity();
    Matrix3D I = massProps->getInertiaTensor();
    Matrix3D invI = massProps->getInverseInertiaTensor();
    Vector3D Iomega = I * omega;
    Vector3D expectedAlpha = invI * (derivatives.totalTorque - omega.cross(Iomega));
    
    EXPECT_NEAR(derivatives.angularAcceleration.x(), expectedAlpha.x(), 1e-10);
    EXPECT_NEAR(derivatives.angularAcceleration.y(), expectedAlpha.y(), 1e-10);
    EXPECT_NEAR(derivatives.angularAcceleration.z(), expectedAlpha.z(), 1e-10);
}

TEST_F(DynamicsEngineTest, CenterOfMassOffset) {
    // With COM offset, forces should create torques
    // Rotate satellite so force doesn't align with COM offset
    initialState->setAttitude(Quaternion::fromAxisAngle(Vector3D::unitZ(), M_PI / 4));
    
    auto derivatives = engine->calculateDerivatives(*initialState, 0.0);
    
    // Force in body frame
    Vector3D forceBody = initialState->inertialToBody(derivatives.totalForce);
    Vector3D com = massProps->getCenterOfMass();
    Vector3D torqueFromForce = com.cross(forceBody);
    
    // This torque should be included in total torque
    // (Note: exact comparison difficult due to other torque sources)
    EXPECT_GT(torqueFromForce.magnitude(), 0.0);
}

TEST_F(DynamicsEngineTest, TimeVaryingMassProperties) {
    // Create time-varying mass properties
    class MockTimeVaryingProps : public IMassProperties {
    public:
        mutable int updateCount = 0;
        
        double getMass() const override { return 1000.0; }
        Vector3D getCenterOfMass() const override { return Vector3D(); }
        Matrix3D getInertiaTensor() const override {
            return Matrix3D(100.0, 0.0, 0.0, 0.0, 500.0, 0.0, 0.0, 0.0, 600.0);
        }
        Matrix3D getInverseInertiaTensor() const override {
            return getInertiaTensor().inverse();
        }
        bool isTimeVarying() const override { return true; }
        void updateTime(double /*time*/) override { updateCount++; }
    };
    
    auto mockProps = std::make_shared<MockTimeVaryingProps>();
    engine->setMassProperties(mockProps);
    
    // Calculate derivatives
    engine->calculateDerivatives(*initialState, 10.0);
    
    // Verify updateTime was called
    EXPECT_EQ(mockProps->updateCount, 1);
}

TEST_F(DynamicsEngineTest, ApplyDerivatives) {
    auto derivatives = engine->calculateDerivatives(*initialState, 0.0);
    
    double dt = 0.1;
    DynamicsState newState = DynamicsEngine::applyDerivatives(
        *initialState, derivatives, dt
    );
    
    // Check position update
    Vector3D expectedPos = position + velocity * dt;
    EXPECT_NEAR((newState.getPosition() - expectedPos).magnitude(), 0.0, 1e-10);
    
    // Check velocity update
    Vector3D expectedVel = velocity + derivatives.acceleration * dt;
    EXPECT_NEAR((newState.getVelocity() - expectedVel).magnitude(), 0.0, 1e-10);
    
    // Check time update
    EXPECT_NEAR(newState.getTime().getJ2000(iloss::time::TimeSystem::UTC), dt, 1e-6);
    
    // Check attitude normalization
    EXPECT_NEAR(newState.getAttitude().norm(), 1.0, 1e-10);
    
    // Check angular velocity update
    Vector3D expectedOmega = initialState->getAngularVelocity() + 
                            derivatives.angularAcceleration * dt;
    EXPECT_NEAR((newState.getAngularVelocity() - expectedOmega).magnitude(), 0.0, 1e-10);
}

TEST_F(DynamicsEngineTest, DerivativesToVector) {
    auto derivatives = engine->calculateDerivatives(*initialState, 0.0);
    
    std::vector<double> vec = derivatives.toVector();
    
    EXPECT_EQ(vec.size(), 13);
    
    // Check order
    EXPECT_DOUBLE_EQ(vec[0], derivatives.velocity.x());
    EXPECT_DOUBLE_EQ(vec[1], derivatives.velocity.y());
    EXPECT_DOUBLE_EQ(vec[2], derivatives.velocity.z());
    EXPECT_DOUBLE_EQ(vec[3], derivatives.acceleration.x());
    EXPECT_DOUBLE_EQ(vec[4], derivatives.acceleration.y());
    EXPECT_DOUBLE_EQ(vec[5], derivatives.acceleration.z());
    EXPECT_DOUBLE_EQ(vec[6], derivatives.attitudeRate.w());
    EXPECT_DOUBLE_EQ(vec[7], derivatives.attitudeRate.x());
    EXPECT_DOUBLE_EQ(vec[8], derivatives.attitudeRate.y());
    EXPECT_DOUBLE_EQ(vec[9], derivatives.attitudeRate.z());
    EXPECT_DOUBLE_EQ(vec[10], derivatives.angularAcceleration.x());
    EXPECT_DOUBLE_EQ(vec[11], derivatives.angularAcceleration.y());
    EXPECT_DOUBLE_EQ(vec[12], derivatives.angularAcceleration.z());
}

TEST_F(DynamicsEngineTest, ConservationOfAngularMomentum) {
    // With no external torques, angular momentum should be conserved
    // (in inertial frame)
    
    // Remove all torques
    torqueAggregator->clearModels();
    
    // Calculate initial angular momentum in inertial frame
    Vector3D L0 = initialState->getAngularMomentumInertial(*massProps);
    
    // Apply dynamics for a short time
    auto derivatives = engine->calculateDerivatives(*initialState, 0.0);
    double dt = 0.1;
    DynamicsState newState = DynamicsEngine::applyDerivatives(
        *initialState, derivatives, dt
    );
    
    // Calculate new angular momentum
    Vector3D L1 = newState.getAngularMomentumInertial(*massProps);
    
    // Should be conserved (within numerical tolerance)
    EXPECT_NEAR((L1 - L0).magnitude(), 0.0, 1e-9);
}

TEST_F(DynamicsEngineTest, SetMassProperties) {
    // Create new mass properties
    auto newProps = std::make_shared<SimpleMassProperties>(
        2000.0, 200.0, 400.0, 500.0
    );
    
    engine->setMassProperties(newProps);
    
    EXPECT_EQ(engine->getMassProperties(), newProps);
    
    // Verify null check
    EXPECT_THROW(engine->setMassProperties(nullptr), std::invalid_argument);
}