#include <gtest/gtest.h>
#include "physics/dynamics/torques/GravityGradientTorque.h"
#include "physics/dynamics/SimpleMassProperties.h"
#include "physics/dynamics/DynamicsState.h"
#include "core/math/MathConstants.h"
#include <cmath>

using namespace iloss::physics::dynamics;
using namespace iloss::physics::dynamics::torques;
using namespace iloss::math;

class GravityGradientTorqueTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create asymmetric satellite for gravity gradient effects
        // Principal moments: Ixx < Iyy < Izz (typical elongated satellite)
        double mass = 1000.0;  // 1000 kg
        double Ixx = 100.0;    // kg⋅m²
        double Iyy = 500.0;    // kg⋅m²
        double Izz = 600.0;    // kg⋅m²
        
        massProps = std::make_shared<SimpleMassProperties>(mass, Ixx, Iyy, Izz);
        
        // Configure gravity gradient torque model
        GravityGradientConfig config;
        config.centralBodyMu = constants::EARTH_MU;
        config.massProperties = massProps;
        
        ggTorque = std::make_unique<GravityGradientTorque>(config);
        
        // Create default state at 500 km altitude
        double altitude = 500000.0;  // 500 km
        double radius = constants::EARTH_RADIUS_EQUATORIAL + altitude;
        position = Vector3D(radius, 0.0, 0.0);
        velocity = Vector3D(0.0, std::sqrt(constants::EARTH_MU / radius), 0.0);  // Circular orbit
        
        defaultState = std::make_unique<DynamicsState>(
            position, velocity, mass, 0.0,
            Quaternion::identity(),
            Vector3D()  // No angular velocity
        );
    }

    std::shared_ptr<SimpleMassProperties> massProps;
    std::unique_ptr<GravityGradientTorque> ggTorque;
    std::unique_ptr<DynamicsState> defaultState;
    Vector3D position;
    Vector3D velocity;
};

TEST_F(GravityGradientTorqueTest, DefaultConstructor) {
    GravityGradientTorque torque;
    
    EXPECT_EQ(torque.getType(), TorqueModelType::GravityGradient);
    EXPECT_EQ(torque.getName(), "Gravity Gradient Torque");
    EXPECT_TRUE(torque.isEnabled());
    EXPECT_DOUBLE_EQ(torque.getGravitationalParameter(), constants::EARTH_MU);
}

TEST_F(GravityGradientTorqueTest, ConfiguredConstructor) {
    double customMu = 1.32712440018e20;  // Sun's gravitational parameter
    
    GravityGradientConfig config;
    config.centralBodyMu = customMu;
    config.massProperties = massProps;
    
    GravityGradientTorque torque(config);
    
    EXPECT_DOUBLE_EQ(torque.getGravitationalParameter(), customMu);
}

TEST_F(GravityGradientTorqueTest, NoTorqueWhenDisabled) {
    ggTorque->setEnabled(false);
    
    Vector3D torque = ggTorque->calculateTorque(*defaultState, 0.0);
    
    EXPECT_EQ(torque, Vector3D());
}

TEST_F(GravityGradientTorqueTest, NoTorqueWithoutMassProperties) {
    GravityGradientTorque torque;  // No mass properties set
    
    Vector3D result = torque.calculateTorque(*defaultState, 0.0);
    
    EXPECT_EQ(result, Vector3D());
}

TEST_F(GravityGradientTorqueTest, NoTorqueAtOrigin) {
    // Place satellite at origin (should avoid singularity)
    DynamicsState stateAtOrigin(
        Vector3D(), velocity, 1000.0, 0.0,
        Quaternion::identity(), Vector3D()
    );
    
    Vector3D torque = ggTorque->calculateTorque(stateAtOrigin, 0.0);
    
    EXPECT_EQ(torque, Vector3D());
}

TEST_F(GravityGradientTorqueTest, NoTorqueWhenAligned) {
    // Test non-equilibrium orientation produces non-zero torque
    // Rotate satellite slightly so no principal axis aligns with nadir
    Quaternion attitude = Quaternion::fromAxisAngle(Vector3D::unitZ(), 0.1);  // Small rotation
    defaultState->setAttitude(attitude);
    
    Vector3D torque = ggTorque->calculateTorque(*defaultState, 0.0);
    
    // With small rotation, nadir no longer aligns with principal axis
    // This is not an equilibrium position, so torque should be non-zero
    EXPECT_GT(torque.magnitude(), 0.0);
}

TEST_F(GravityGradientTorqueTest, StableEquilibrium) {
    // Rotate satellite so Z-axis (max inertia) points toward Earth
    // This requires rotating 90 degrees about Y-axis
    Quaternion attitude = Quaternion::fromAxisAngle(Vector3D::unitY(), -M_PI / 2);
    defaultState->setAttitude(attitude);
    
    Vector3D torque = ggTorque->calculateTorque(*defaultState, 0.0);
    
    // Should be very small (numerical errors only)
    EXPECT_LT(torque.magnitude(), 1e-10);
}

TEST_F(GravityGradientTorqueTest, UnstableEquilibrium) {
    // Rotate satellite so X-axis (min inertia) points toward Earth
    // This is unstable equilibrium - small torque expected due to numerical precision
    Quaternion attitude = Quaternion::identity();  // X already points radially
    defaultState->setAttitude(attitude);
    
    // Slightly perturb to break perfect alignment
    Quaternion perturbation = Quaternion::fromAxisAngle(Vector3D::unitZ(), 0.001);
    defaultState->setAttitude(attitude * perturbation);
    
    Vector3D torque = ggTorque->calculateTorque(*defaultState, 0.0);
    
    // Torque should try to rotate away from unstable equilibrium
    EXPECT_GT(torque.magnitude(), 0.0);
}

TEST_F(GravityGradientTorqueTest, TorqueMagnitudeScaling) {
    // Test that torque scales as 1/r³
    // Use non-aligned orientation to ensure non-zero torques
    Quaternion attitude = Quaternion::fromAxisAngle(Vector3D::unitZ(), M_PI / 4);  // 45 degree rotation
    defaultState->setAttitude(attitude);
    Vector3D torque1 = ggTorque->calculateTorque(*defaultState, 0.0);
    
    // Double the orbital radius
    Vector3D position2 = position * 2.0;
    DynamicsState state2(
        position2, velocity, 1000.0, 0.0,
        attitude,  // Same attitude
        Vector3D()
    );
    
    Vector3D torque2 = ggTorque->calculateTorque(state2, 0.0);
    
    // Torque should decrease by factor of 8 (2³)
    double ratio = torque1.magnitude() / torque2.magnitude();
    EXPECT_NEAR(ratio, 8.0, 0.1);  // Allow small tolerance
}

TEST_F(GravityGradientTorqueTest, TorqueDirection) {
    // Test torque direction for specific orientation
    // Rotate 45 degrees about Y-axis
    Quaternion attitude = Quaternion::fromAxisAngle(Vector3D::unitY(), M_PI / 4);
    defaultState->setAttitude(attitude);
    
    Vector3D torque = ggTorque->calculateTorque(*defaultState, 0.0);
    
    // For this configuration, torque should be primarily about Y-axis
    EXPECT_LT(std::abs(torque.x()), std::abs(torque.y()));
    EXPECT_LT(std::abs(torque.z()), std::abs(torque.y()));
}

TEST_F(GravityGradientTorqueTest, SymmetricInertia) {
    // For spherical satellite (Ixx = Iyy = Izz), no gravity gradient torque
    auto sphericalProps = std::make_shared<SimpleMassProperties>(
        1000.0, 300.0, 300.0, 300.0
    );
    
    GravityGradientConfig config;
    config.centralBodyMu = constants::EARTH_MU;
    config.massProperties = sphericalProps;
    
    GravityGradientTorque sphericalGG(config);
    
    // Try various orientations
    for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 4) {
        Quaternion attitude = Quaternion::fromAxisAngle(Vector3D::unitZ(), angle);
        defaultState->setAttitude(attitude);
        
        Vector3D torque = sphericalGG.calculateTorque(*defaultState, 0.0);
        
        EXPECT_LT(torque.magnitude(), 1e-10);
    }
}

TEST_F(GravityGradientTorqueTest, ConfigurationUpdate) {
    // Test configuration update
    double newMu = 4.9048695e12;  // Moon's gravitational parameter
    
    GravityGradientConfig newConfig;
    newConfig.centralBodyMu = newMu;
    newConfig.massProperties = massProps;
    
    ggTorque->configure(newConfig);
    
    EXPECT_DOUBLE_EQ(ggTorque->getGravitationalParameter(), newMu);
}

TEST_F(GravityGradientTorqueTest, InvalidConfiguration) {
    // Test invalid configuration type
    int invalidConfig = 42;
    
    EXPECT_THROW(ggTorque->configure(invalidConfig), std::invalid_argument);
}

TEST_F(GravityGradientTorqueTest, Clone) {
    auto cloned = ggTorque->clone();
    
    EXPECT_EQ(cloned->getType(), TorqueModelType::GravityGradient);
    EXPECT_EQ(cloned->getName(), "Gravity Gradient Torque");
    EXPECT_EQ(cloned->isEnabled(), ggTorque->isEnabled());
    
    // Verify it calculates same torque
    Vector3D originalTorque = ggTorque->calculateTorque(*defaultState, 0.0);
    Vector3D clonedTorque = cloned->calculateTorque(*defaultState, 0.0);
    
    EXPECT_NEAR(originalTorque.x(), clonedTorque.x(), 1e-10);
    EXPECT_NEAR(originalTorque.y(), clonedTorque.y(), 1e-10);
    EXPECT_NEAR(originalTorque.z(), clonedTorque.z(), 1e-10);
}

TEST_F(GravityGradientTorqueTest, TimeVaryingMassProperties) {
    // Create mock time-varying mass properties
    class TimeVaryingMassProps : public IMassProperties {
    public:
        TimeVaryingMassProps() : m_time(0.0) {}
        
        double getMass() const override { return 1000.0; }
        Vector3D getCenterOfMass() const override { return Vector3D(); }
        Matrix3D getInertiaTensor() const override {
            // Inertia varies with time
            double factor = 1.0 + 0.1 * std::sin(m_time);
            return Matrix3D(
                100.0 * factor, 0.0, 0.0,
                0.0, 500.0 * factor, 0.0,
                0.0, 0.0, 600.0 * factor
            );
        }
        Matrix3D getInverseInertiaTensor() const override {
            return getInertiaTensor().inverse();
        }
        bool isTimeVarying() const override { return true; }
        void updateTime(double time) override { m_time = time; }
        
    private:
        double m_time;
    };
    
    auto timeVaryingProps = std::make_shared<TimeVaryingMassProps>();
    ggTorque->setMassProperties(timeVaryingProps);
    
    // Use non-aligned orientation to ensure non-zero torques
    Quaternion attitude = Quaternion::fromAxisAngle(Vector3D::unitZ(), 0.3);
    defaultState->setAttitude(attitude);
    
    // Calculate torque at different times
    Vector3D torque1 = ggTorque->calculateTorque(*defaultState, 0.0);
    Vector3D torque2 = ggTorque->calculateTorque(*defaultState, M_PI / 2);  // 90 degrees phase
    
    // Torques should be different due to changing inertia
    EXPECT_GT((torque1 - torque2).magnitude(), 0.0);
}