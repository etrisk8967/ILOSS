#include <gtest/gtest.h>
#include "physics/dynamics/DynamicsState.h"
#include "physics/dynamics/SimpleMassProperties.h"
#include "core/coordinates/CoordinateSystem.h"
#include "core/math/MathConstants.h"
#include <cmath>

using namespace iloss::physics::dynamics;
using namespace iloss::physics;
using namespace iloss::math;
using namespace iloss::coordinates;
using namespace iloss::time;

class DynamicsStateTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a default dynamics state
        position = Vector3D(7000000.0, 0.0, 0.0);  // 7000 km radius
        velocity = Vector3D(0.0, 7500.0, 0.0);     // Circular orbit velocity
        mass = 1000.0;                             // 1000 kg
        time = 0.0;
        attitude = Quaternion::identity();         // No rotation
        angularVelocity = Vector3D(0.0, 0.0, 0.1); // 0.1 rad/s about Z
        
        defaultState = std::make_unique<DynamicsState>(
            position, velocity, mass, time, attitude, angularVelocity
        );
        
        // Create mass properties for energy calculations
        massProps = std::make_unique<SimpleMassProperties>(
            mass, 100.0, 200.0, 150.0  // Principal moments of inertia
        );
    }

    Vector3D position;
    Vector3D velocity;
    double mass;
    double time;
    Quaternion attitude;
    Vector3D angularVelocity;
    std::unique_ptr<DynamicsState> defaultState;
    std::unique_ptr<SimpleMassProperties> massProps;
};

TEST_F(DynamicsStateTest, DefaultConstructor) {
    DynamicsState state;
    
    EXPECT_EQ(state.getPosition(), Vector3D());
    EXPECT_EQ(state.getVelocity(), Vector3D());
    EXPECT_DOUBLE_EQ(state.getMass(), 0.0);
    EXPECT_DOUBLE_EQ(state.getTime().getJ2000(iloss::time::TimeSystem::UTC), 0.0);
    EXPECT_EQ(state.getAttitude(), Quaternion::identity());
    EXPECT_EQ(state.getAngularVelocity(), Vector3D());
    EXPECT_EQ(state.getCoordinateSystem(), CoordinateSystemType::ECI_J2000);
}

TEST_F(DynamicsStateTest, FullConstructor) {
    EXPECT_EQ(defaultState->getPosition(), position);
    EXPECT_EQ(defaultState->getVelocity(), velocity);
    EXPECT_DOUBLE_EQ(defaultState->getMass(), mass);
    EXPECT_DOUBLE_EQ(defaultState->getTime().getJ2000(iloss::time::TimeSystem::UTC), time);
    EXPECT_EQ(defaultState->getAttitude(), attitude);
    EXPECT_EQ(defaultState->getAngularVelocity(), angularVelocity);
}

TEST_F(DynamicsStateTest, ConstructFromStateVector) {
    iloss::physics::StateVector baseState(position, velocity, mass, Time(time + TimeConstants::J2000_EPOCH, TimeSystem::UTC), CoordinateSystemType::ECI_J2000);
    Quaternion att = Quaternion::fromAxisAngle(Vector3D::unitZ(), M_PI / 4);
    Vector3D omega(0.1, 0.2, 0.3);
    
    DynamicsState dynState(baseState, att, omega);
    
    EXPECT_EQ(dynState.getPosition(), position);
    EXPECT_EQ(dynState.getVelocity(), velocity);
    EXPECT_DOUBLE_EQ(dynState.getMass(), mass);
    EXPECT_DOUBLE_EQ(dynState.getTime().getJ2000(iloss::time::TimeSystem::UTC), time);
    EXPECT_EQ(dynState.getCoordinateSystem(), CoordinateSystemType::ECI_J2000);
    EXPECT_EQ(dynState.getAttitude(), att.normalized());
    EXPECT_EQ(dynState.getAngularVelocity(), omega);
}

TEST_F(DynamicsStateTest, AttitudeNormalization) {
    // Create unnormalized quaternion
    Quaternion unnormalized(2.0, 0.0, 0.0, 0.0);
    
    DynamicsState state(position, velocity, mass, time, unnormalized, angularVelocity);
    
    // Should be normalized
    EXPECT_NEAR(state.getAttitude().norm(), 1.0, 1e-10);
    EXPECT_NEAR(state.getAttitude().w(), 1.0, 1e-10);
}

TEST_F(DynamicsStateTest, SettersAndGetters) {
    DynamicsState state;
    
    Quaternion newAtt = Quaternion::fromAxisAngle(Vector3D::unitX(), M_PI / 6);
    state.setAttitude(newAtt);
    EXPECT_EQ(state.getAttitude(), newAtt.normalized());
    
    Vector3D newOmega(0.5, -0.3, 0.2);
    state.setAngularVelocity(newOmega);
    EXPECT_EQ(state.getAngularVelocity(), newOmega);
}

TEST_F(DynamicsStateTest, RotationMatrix) {
    Quaternion att = Quaternion::fromAxisAngle(Vector3D::unitZ(), M_PI / 2);
    defaultState->setAttitude(att);
    
    Matrix3D rotMat = defaultState->getRotationMatrix();
    
    // For 90 degree rotation about Z:
    // [0 -1 0]
    // [1  0 0]
    // [0  0 1]
    EXPECT_NEAR(rotMat(0, 0), 0.0, 1e-10);
    EXPECT_NEAR(rotMat(0, 1), -1.0, 1e-10);
    EXPECT_NEAR(rotMat(1, 0), 1.0, 1e-10);
    EXPECT_NEAR(rotMat(1, 1), 0.0, 1e-10);
    EXPECT_NEAR(rotMat(2, 2), 1.0, 1e-10);
}

TEST_F(DynamicsStateTest, FrameTransformations) {
    // Set attitude to 90 degree rotation about Z
    Quaternion att = Quaternion::fromAxisAngle(Vector3D::unitZ(), M_PI / 2);
    defaultState->setAttitude(att);
    
    // Test body to inertial transformation
    Vector3D bodyVec(1.0, 0.0, 0.0);  // X in body frame
    Vector3D inertialVec = defaultState->bodyToInertial(bodyVec);
    
    // Should be Y in inertial frame
    EXPECT_NEAR(inertialVec.x(), 0.0, 1e-10);
    EXPECT_NEAR(inertialVec.y(), 1.0, 1e-10);
    EXPECT_NEAR(inertialVec.z(), 0.0, 1e-10);
    
    // Test inertial to body transformation
    Vector3D inertialVec2(0.0, 1.0, 0.0);  // Y in inertial frame
    Vector3D bodyVec2 = defaultState->inertialToBody(inertialVec2);
    
    // Should be X in body frame
    EXPECT_NEAR(bodyVec2.x(), 1.0, 1e-10);
    EXPECT_NEAR(bodyVec2.y(), 0.0, 1e-10);
    EXPECT_NEAR(bodyVec2.z(), 0.0, 1e-10);
}

TEST_F(DynamicsStateTest, AngularMomentumBody) {
    Vector3D omega(0.1, 0.2, 0.3);
    defaultState->setAngularVelocity(omega);
    
    Vector3D L_body = defaultState->getAngularMomentumBody(*massProps);
    
    // L = I * omega
    Matrix3D I = massProps->getInertiaTensor();
    Vector3D expected = I * omega;
    
    EXPECT_NEAR(L_body.x(), expected.x(), 1e-10);
    EXPECT_NEAR(L_body.y(), expected.y(), 1e-10);
    EXPECT_NEAR(L_body.z(), expected.z(), 1e-10);
}

TEST_F(DynamicsStateTest, AngularMomentumInertial) {
    // Set non-trivial attitude
    Quaternion att = Quaternion::fromAxisAngle(Vector3D::unitY(), M_PI / 3);
    defaultState->setAttitude(att);
    
    Vector3D L_body = defaultState->getAngularMomentumBody(*massProps);
    Vector3D L_inertial = defaultState->getAngularMomentumInertial(*massProps);
    
    // L_inertial should be L_body transformed to inertial frame
    Vector3D expected = defaultState->bodyToInertial(L_body);
    
    EXPECT_NEAR(L_inertial.x(), expected.x(), 1e-10);
    EXPECT_NEAR(L_inertial.y(), expected.y(), 1e-10);
    EXPECT_NEAR(L_inertial.z(), expected.z(), 1e-10);
}

TEST_F(DynamicsStateTest, RotationalKineticEnergy) {
    Vector3D omega(0.1, 0.2, 0.3);
    defaultState->setAngularVelocity(omega);
    
    double T_rot = defaultState->getRotationalKineticEnergy(*massProps);
    
    // T = 0.5 * omega^T * I * omega
    Matrix3D I = massProps->getInertiaTensor();
    Vector3D Iomega = I * omega;
    double expected = 0.5 * omega.dot(Iomega);
    
    EXPECT_NEAR(T_rot, expected, 1e-10);
}

TEST_F(DynamicsStateTest, TotalKineticEnergy) {
    double T_total = defaultState->getTotalKineticEnergy(*massProps);
    
    // Translational KE
    double T_trans = 0.5 * mass * velocity.magnitudeSquared();
    
    // Rotational KE
    double T_rot = defaultState->getRotationalKineticEnergy(*massProps);
    
    EXPECT_NEAR(T_total, T_trans + T_rot, 1e-10);
}

TEST_F(DynamicsStateTest, StateValidation) {
    EXPECT_TRUE(defaultState->isValid());
    
    // Test with unnormalized quaternion (should still be valid after normalization)
    DynamicsState state1(position, velocity, mass, time, 
                        Quaternion(2.0, 0.0, 0.0, 0.0), angularVelocity);
    EXPECT_TRUE(state1.isValid());
    
    // Test with NaN in angular velocity
    DynamicsState state2;
    state2.setAngularVelocity(Vector3D(std::nan(""), 0.0, 0.0));
    EXPECT_FALSE(state2.isValid());
}

TEST_F(DynamicsStateTest, Clone) {
    auto cloned = defaultState->clone();
    
    EXPECT_EQ(cloned->getPosition(), defaultState->getPosition());
    EXPECT_EQ(cloned->getVelocity(), defaultState->getVelocity());
    EXPECT_DOUBLE_EQ(cloned->getMass(), defaultState->getMass());
    EXPECT_DOUBLE_EQ(cloned->getTime().getTime(), defaultState->getTime().getTime());
    EXPECT_EQ(cloned->getAttitude(), defaultState->getAttitude());
    EXPECT_EQ(cloned->getAngularVelocity(), defaultState->getAngularVelocity());
    
    // Verify it's a deep copy
    cloned->setAngularVelocity(Vector3D(1.0, 2.0, 3.0));
    EXPECT_NE(cloned->getAngularVelocity(), defaultState->getAngularVelocity());
}

TEST_F(DynamicsStateTest, Interpolation) {
    // Create two states with different attitudes
    DynamicsState state1(position, velocity, mass, 0.0,
                        Quaternion::identity(), Vector3D(0.0, 0.0, 0.1));
    
    Vector3D pos2 = position + velocity * 10.0;
    Vector3D vel2 = velocity + Vector3D(10.0, 0.0, 0.0);
    Quaternion att2 = Quaternion::fromAxisAngle(Vector3D::unitZ(), M_PI / 2);
    Vector3D omega2(0.0, 0.0, 0.5);
    
    DynamicsState state2(pos2, vel2, mass, 10.0, att2, omega2);
    
    // Interpolate at t = 0.5
    DynamicsState interp = state1.interpolate(state2, 0.5);
    
    // Check position and velocity (linear interpolation)
    Vector3D expectedPos = position * 0.5 + pos2 * 0.5;
    Vector3D expectedVel = velocity * 0.5 + vel2 * 0.5;
    EXPECT_NEAR((interp.getPosition() - expectedPos).magnitude(), 0.0, 1e-10);
    EXPECT_NEAR((interp.getVelocity() - expectedVel).magnitude(), 0.0, 1e-10);
    
    // Check time
    EXPECT_NEAR(interp.getTime().getJ2000(iloss::time::TimeSystem::UTC), 5.0, 1e-10);
    
    // Check angular velocity (linear interpolation)
    Vector3D expectedOmega = state1.getAngularVelocity() * 0.5 + omega2 * 0.5;
    EXPECT_NEAR((interp.getAngularVelocity() - expectedOmega).magnitude(), 0.0, 1e-10);
    
    // Check attitude (should use SLERP)
    // For 90 degree rotation, halfway should be 45 degrees
    double angle = interp.getAttitude().angle();
    EXPECT_NEAR(angle, M_PI / 4, 1e-6);
}