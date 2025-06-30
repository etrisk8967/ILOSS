#include <gtest/gtest.h>
#include "physics/dynamics/SimpleMassProperties.h"
#include "core/math/MathConstants.h"
#include <cmath>

using namespace iloss::physics::dynamics;
using namespace iloss::math;

class SimpleMassPropertiesTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a simple cube satellite mass properties
        // 100 kg mass, 1m x 1m x 1m cube
        double mass = 100.0;
        double sideLength = 1.0;
        double I = mass * sideLength * sideLength / 6.0;  // For a cube
        
        cubeSat = std::make_unique<SimpleMassProperties>(mass, I, I, I);
    }

    std::unique_ptr<SimpleMassProperties> cubeSat;
};

TEST_F(SimpleMassPropertiesTest, ConstructorWithDiagonalInertia) {
    double mass = 50.0;
    SimpleMassProperties props(mass, 10.0, 20.0, 30.0);
    
    EXPECT_DOUBLE_EQ(props.getMass(), mass);
    EXPECT_EQ(props.getCenterOfMass(), Vector3D());  // Default at origin
    
    auto inertia = props.getInertiaTensor();
    EXPECT_DOUBLE_EQ(inertia(0, 0), 10.0);
    EXPECT_DOUBLE_EQ(inertia(1, 1), 20.0);
    EXPECT_DOUBLE_EQ(inertia(2, 2), 30.0);
    
    // Check off-diagonal elements are zero
    EXPECT_DOUBLE_EQ(inertia(0, 1), 0.0);
    EXPECT_DOUBLE_EQ(inertia(0, 2), 0.0);
    EXPECT_DOUBLE_EQ(inertia(1, 2), 0.0);
}

TEST_F(SimpleMassPropertiesTest, ConstructorWithFullTensor) {
    double mass = 75.0;
    Matrix3D inertia(
        100.0, -5.0,  -3.0,
        -5.0,  200.0, -2.0,
        -3.0,  -2.0,  150.0
    );
    Vector3D com(0.1, -0.05, 0.02);
    
    SimpleMassProperties props(mass, inertia, com);
    
    EXPECT_DOUBLE_EQ(props.getMass(), mass);
    EXPECT_EQ(props.getCenterOfMass(), com);
    EXPECT_EQ(props.getInertiaTensor(), inertia);
}

TEST_F(SimpleMassPropertiesTest, InvalidMass) {
    EXPECT_THROW(SimpleMassProperties(0.0, 10.0, 10.0, 10.0), std::invalid_argument);
    EXPECT_THROW(SimpleMassProperties(-10.0, 10.0, 10.0, 10.0), std::invalid_argument);
}

TEST_F(SimpleMassPropertiesTest, InvalidInertia) {
    // Negative diagonal elements
    EXPECT_THROW(SimpleMassProperties(10.0, -1.0, 10.0, 10.0), std::invalid_argument);
    EXPECT_THROW(SimpleMassProperties(10.0, 10.0, -1.0, 10.0), std::invalid_argument);
    EXPECT_THROW(SimpleMassProperties(10.0, 10.0, 10.0, -1.0), std::invalid_argument);
    
    // Violates triangle inequality
    EXPECT_THROW(SimpleMassProperties(10.0, 1.0, 1.0, 10.0), std::invalid_argument);
}

TEST_F(SimpleMassPropertiesTest, NonSymmetricTensor) {
    Matrix3D nonSymmetric(
        10.0, 1.0, 2.0,
        3.0, 20.0, 4.0,
        5.0, 6.0, 30.0
    );
    
    EXPECT_THROW(SimpleMassProperties(50.0, nonSymmetric), std::invalid_argument);
}

TEST_F(SimpleMassPropertiesTest, InverseInertiaTensor) {
    auto inverse = cubeSat->getInverseInertiaTensor();
    auto inertia = cubeSat->getInertiaTensor();
    
    // I * I^-1 should equal identity
    auto product = inertia * inverse;
    auto identity = Matrix3D::identity();
    
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(product(i, j), identity(i, j), 1e-10);
        }
    }
}

TEST_F(SimpleMassPropertiesTest, TimeVarying) {
    // SimpleMassProperties should not be time-varying
    EXPECT_FALSE(cubeSat->isTimeVarying());
    
    // updateTime should be a no-op
    double time = 100.0;
    cubeSat->updateTime(time);  // Should not throw or change anything
    
    EXPECT_DOUBLE_EQ(cubeSat->getMass(), 100.0);
}

TEST_F(SimpleMassPropertiesTest, Setters) {
    // Test mass setter
    cubeSat->setMass(150.0);
    EXPECT_DOUBLE_EQ(cubeSat->getMass(), 150.0);
    
    // Test invalid mass
    EXPECT_THROW(cubeSat->setMass(0.0), std::invalid_argument);
    EXPECT_THROW(cubeSat->setMass(-10.0), std::invalid_argument);
    
    // Test center of mass setter
    Vector3D newCom(0.1, 0.2, 0.3);
    cubeSat->setCenterOfMass(newCom);
    EXPECT_EQ(cubeSat->getCenterOfMass(), newCom);
    
    // Test inertia tensor setter
    Matrix3D newInertia(
        50.0, 0.0, 0.0,
        0.0, 60.0, 0.0,
        0.0, 0.0, 70.0
    );
    cubeSat->setInertiaTensor(newInertia);
    EXPECT_EQ(cubeSat->getInertiaTensor(), newInertia);
    
    // Verify inverse is updated
    auto inverse = cubeSat->getInverseInertiaTensor();
    EXPECT_NEAR(inverse(0, 0), 1.0/50.0, 1e-10);
    EXPECT_NEAR(inverse(1, 1), 1.0/60.0, 1e-10);
    EXPECT_NEAR(inverse(2, 2), 1.0/70.0, 1e-10);
}

TEST_F(SimpleMassPropertiesTest, RealisticSatelliteProperties) {
    // Test with realistic satellite properties
    // 500 kg satellite, roughly 2m x 1m x 1m
    double mass = 500.0;
    double Ixx = 250.0;  // About X axis (smaller dimension)
    double Iyy = 550.0;  // About Y axis (larger dimension)
    double Izz = 550.0;  // About Z axis (larger dimension)
    Vector3D com(0.05, -0.02, 0.01);  // Slight offset from geometric center
    
    SimpleMassProperties satellite(mass, Ixx, Iyy, Izz, com);
    
    EXPECT_DOUBLE_EQ(satellite.getMass(), mass);
    EXPECT_EQ(satellite.getCenterOfMass(), com);
    
    // Verify principal moments satisfy physical constraints
    auto inertia = satellite.getInertiaTensor();
    double I1 = inertia(0, 0);
    double I2 = inertia(1, 1);
    double I3 = inertia(2, 2);
    
    // Triangle inequality
    EXPECT_GT(I1 + I2, I3);
    EXPECT_GT(I2 + I3, I1);
    EXPECT_GT(I3 + I1, I2);
    
    // All positive
    EXPECT_GT(I1, 0.0);
    EXPECT_GT(I2, 0.0);
    EXPECT_GT(I3, 0.0);
}