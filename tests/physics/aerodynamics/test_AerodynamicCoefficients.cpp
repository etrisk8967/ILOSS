#include <gtest/gtest.h>
#include "physics/aerodynamics/AerodynamicCoefficients.h"
#include <cmath>
#include <sstream>

using namespace iloss::physics::aerodynamics;

class AerodynamicCoefficientsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Default test values
        testCD = 0.3;
        testCL = 0.15;
        testCY = 0.05;
        testCroll = 0.01;
        testCpitch = -0.02;
        testCyaw = 0.03;
    }

    double testCD, testCL, testCY, testCroll, testCpitch, testCyaw;
};

TEST_F(AerodynamicCoefficientsTest, DefaultConstructor) {
    AerodynamicCoefficients coeffs;
    
    EXPECT_DOUBLE_EQ(coeffs.getDragCoefficient(), 0.0);
    EXPECT_DOUBLE_EQ(coeffs.getLiftCoefficient(), 0.0);
    EXPECT_DOUBLE_EQ(coeffs.getSideForceCoefficient(), 0.0);
    EXPECT_DOUBLE_EQ(coeffs.getRollMomentCoefficient(), 0.0);
    EXPECT_DOUBLE_EQ(coeffs.getPitchMomentCoefficient(), 0.0);
    EXPECT_DOUBLE_EQ(coeffs.getYawMomentCoefficient(), 0.0);
}

TEST_F(AerodynamicCoefficientsTest, FullConstructor) {
    AerodynamicCoefficients coeffs(testCD, testCL, testCY, 
                                  testCroll, testCpitch, testCyaw);
    
    EXPECT_DOUBLE_EQ(coeffs.getDragCoefficient(), testCD);
    EXPECT_DOUBLE_EQ(coeffs.getLiftCoefficient(), testCL);
    EXPECT_DOUBLE_EQ(coeffs.getSideForceCoefficient(), testCY);
    EXPECT_DOUBLE_EQ(coeffs.getRollMomentCoefficient(), testCroll);
    EXPECT_DOUBLE_EQ(coeffs.getPitchMomentCoefficient(), testCpitch);
    EXPECT_DOUBLE_EQ(coeffs.getYawMomentCoefficient(), testCyaw);
}

TEST_F(AerodynamicCoefficientsTest, ForceOnlyConstructor) {
    AerodynamicCoefficients coeffs(testCD, testCL, testCY);
    
    EXPECT_DOUBLE_EQ(coeffs.getDragCoefficient(), testCD);
    EXPECT_DOUBLE_EQ(coeffs.getLiftCoefficient(), testCL);
    EXPECT_DOUBLE_EQ(coeffs.getSideForceCoefficient(), testCY);
    EXPECT_DOUBLE_EQ(coeffs.getRollMomentCoefficient(), 0.0);
    EXPECT_DOUBLE_EQ(coeffs.getPitchMomentCoefficient(), 0.0);
    EXPECT_DOUBLE_EQ(coeffs.getYawMomentCoefficient(), 0.0);
}

TEST_F(AerodynamicCoefficientsTest, SettersAndGetters) {
    AerodynamicCoefficients coeffs;
    
    coeffs.setDragCoefficient(testCD);
    coeffs.setLiftCoefficient(testCL);
    coeffs.setSideForceCoefficient(testCY);
    coeffs.setRollMomentCoefficient(testCroll);
    coeffs.setPitchMomentCoefficient(testCpitch);
    coeffs.setYawMomentCoefficient(testCyaw);
    
    EXPECT_DOUBLE_EQ(coeffs.getDragCoefficient(), testCD);
    EXPECT_DOUBLE_EQ(coeffs.getLiftCoefficient(), testCL);
    EXPECT_DOUBLE_EQ(coeffs.getSideForceCoefficient(), testCY);
    EXPECT_DOUBLE_EQ(coeffs.getRollMomentCoefficient(), testCroll);
    EXPECT_DOUBLE_EQ(coeffs.getPitchMomentCoefficient(), testCpitch);
    EXPECT_DOUBLE_EQ(coeffs.getYawMomentCoefficient(), testCyaw);
}

TEST_F(AerodynamicCoefficientsTest, SetForceCoefficients) {
    AerodynamicCoefficients coeffs;
    coeffs.setForceCoefficients(testCD, testCL, testCY);
    
    EXPECT_DOUBLE_EQ(coeffs.getDragCoefficient(), testCD);
    EXPECT_DOUBLE_EQ(coeffs.getLiftCoefficient(), testCL);
    EXPECT_DOUBLE_EQ(coeffs.getSideForceCoefficient(), testCY);
}

TEST_F(AerodynamicCoefficientsTest, SetMomentCoefficients) {
    AerodynamicCoefficients coeffs;
    coeffs.setMomentCoefficients(testCroll, testCpitch, testCyaw);
    
    EXPECT_DOUBLE_EQ(coeffs.getRollMomentCoefficient(), testCroll);
    EXPECT_DOUBLE_EQ(coeffs.getPitchMomentCoefficient(), testCpitch);
    EXPECT_DOUBLE_EQ(coeffs.getYawMomentCoefficient(), testCyaw);
}

TEST_F(AerodynamicCoefficientsTest, InvalidCoefficients) {
    // Test NaN
    EXPECT_THROW(AerodynamicCoefficients(std::nan(""), 0, 0), std::invalid_argument);
    
    // Test infinity
    EXPECT_THROW(AerodynamicCoefficients(std::numeric_limits<double>::infinity(), 0, 0), 
                 std::invalid_argument);
    
    // Test negative infinity
    EXPECT_THROW(AerodynamicCoefficients(-std::numeric_limits<double>::infinity(), 0, 0), 
                 std::invalid_argument);
}

TEST_F(AerodynamicCoefficientsTest, IsValid) {
    // Valid coefficients
    AerodynamicCoefficients validCoeffs(0.3, 0.1, 0.0);
    EXPECT_TRUE(validCoeffs.isValid());
    
    // Create with default constructor (all zeros) - should be valid
    AerodynamicCoefficients zeroCoeffs;
    EXPECT_TRUE(zeroCoeffs.isValid());
}

TEST_F(AerodynamicCoefficientsTest, ToString) {
    AerodynamicCoefficients coeffs(testCD, testCL, testCY, 
                                  testCroll, testCpitch, testCyaw);
    std::string str = coeffs.toString();
    
    EXPECT_TRUE(str.find("CD=0.300000") != std::string::npos);
    EXPECT_TRUE(str.find("CL=0.150000") != std::string::npos);
    EXPECT_TRUE(str.find("CY=0.050000") != std::string::npos);
    EXPECT_TRUE(str.find("Cl=0.010000") != std::string::npos);
    EXPECT_TRUE(str.find("Cm=-0.020000") != std::string::npos);
    EXPECT_TRUE(str.find("Cn=0.030000") != std::string::npos);
}

TEST_F(AerodynamicCoefficientsTest, Scale) {
    AerodynamicCoefficients coeffs(testCD, testCL, testCY, 
                                  testCroll, testCpitch, testCyaw);
    double scaleFactor = 2.0;
    
    AerodynamicCoefficients scaled = coeffs.scale(scaleFactor);
    
    EXPECT_DOUBLE_EQ(scaled.getDragCoefficient(), testCD * scaleFactor);
    EXPECT_DOUBLE_EQ(scaled.getLiftCoefficient(), testCL * scaleFactor);
    EXPECT_DOUBLE_EQ(scaled.getSideForceCoefficient(), testCY * scaleFactor);
    EXPECT_DOUBLE_EQ(scaled.getRollMomentCoefficient(), testCroll * scaleFactor);
    EXPECT_DOUBLE_EQ(scaled.getPitchMomentCoefficient(), testCpitch * scaleFactor);
    EXPECT_DOUBLE_EQ(scaled.getYawMomentCoefficient(), testCyaw * scaleFactor);
}

TEST_F(AerodynamicCoefficientsTest, Interpolate) {
    AerodynamicCoefficients coeffs1(0.3, 0.0, 0.0, 0.0, 0.0, 0.0);
    AerodynamicCoefficients coeffs2(0.4, 0.2, 0.0, 0.0, -0.02, 0.0);
    
    // Test interpolation at factor = 0 (should equal coeffs1)
    AerodynamicCoefficients interp0 = coeffs1.interpolate(coeffs2, 0.0);
    EXPECT_DOUBLE_EQ(interp0.getDragCoefficient(), coeffs1.getDragCoefficient());
    EXPECT_DOUBLE_EQ(interp0.getLiftCoefficient(), coeffs1.getLiftCoefficient());
    
    // Test interpolation at factor = 1 (should equal coeffs2)
    AerodynamicCoefficients interp1 = coeffs1.interpolate(coeffs2, 1.0);
    EXPECT_DOUBLE_EQ(interp1.getDragCoefficient(), coeffs2.getDragCoefficient());
    EXPECT_DOUBLE_EQ(interp1.getLiftCoefficient(), coeffs2.getLiftCoefficient());
    
    // Test interpolation at factor = 0.5 (halfway)
    AerodynamicCoefficients interp05 = coeffs1.interpolate(coeffs2, 0.5);
    EXPECT_DOUBLE_EQ(interp05.getDragCoefficient(), 0.35);
    EXPECT_DOUBLE_EQ(interp05.getLiftCoefficient(), 0.1);
    EXPECT_DOUBLE_EQ(interp05.getPitchMomentCoefficient(), -0.01);
    
    // Test invalid interpolation factors
    EXPECT_THROW(coeffs1.interpolate(coeffs2, -0.1), std::invalid_argument);
    EXPECT_THROW(coeffs1.interpolate(coeffs2, 1.1), std::invalid_argument);
}

TEST_F(AerodynamicCoefficientsTest, EqualityOperator) {
    AerodynamicCoefficients coeffs1(testCD, testCL, testCY, 
                                   testCroll, testCpitch, testCyaw);
    AerodynamicCoefficients coeffs2(testCD, testCL, testCY, 
                                   testCroll, testCpitch, testCyaw);
    AerodynamicCoefficients coeffs3(testCD * 2, testCL, testCY, 
                                   testCroll, testCpitch, testCyaw);
    
    EXPECT_TRUE(coeffs1 == coeffs2);
    EXPECT_FALSE(coeffs1 == coeffs3);
    EXPECT_FALSE(coeffs1 != coeffs2);
    EXPECT_TRUE(coeffs1 != coeffs3);
}

TEST_F(AerodynamicCoefficientsTest, AdditionOperator) {
    AerodynamicCoefficients coeffs1(0.3, 0.1, 0.0, 0.01, -0.02, 0.03);
    AerodynamicCoefficients coeffs2(0.1, 0.05, 0.02, -0.005, 0.01, -0.01);
    
    AerodynamicCoefficients sum = coeffs1 + coeffs2;
    
    EXPECT_NEAR(sum.getDragCoefficient(), 0.4, 1e-10);
    EXPECT_NEAR(sum.getLiftCoefficient(), 0.15, 1e-10);
    EXPECT_NEAR(sum.getSideForceCoefficient(), 0.02, 1e-10);
    EXPECT_NEAR(sum.getRollMomentCoefficient(), 0.005, 1e-10);
    EXPECT_NEAR(sum.getPitchMomentCoefficient(), -0.01, 1e-10);
    EXPECT_NEAR(sum.getYawMomentCoefficient(), 0.02, 1e-10);
}

TEST_F(AerodynamicCoefficientsTest, AdditionAssignmentOperator) {
    AerodynamicCoefficients coeffs1(0.3, 0.1, 0.0, 0.01, -0.02, 0.03);
    AerodynamicCoefficients coeffs2(0.1, 0.05, 0.02, -0.005, 0.01, -0.01);
    
    coeffs1 += coeffs2;
    
    EXPECT_NEAR(coeffs1.getDragCoefficient(), 0.4, 1e-10);
    EXPECT_NEAR(coeffs1.getLiftCoefficient(), 0.15, 1e-10);
    EXPECT_NEAR(coeffs1.getSideForceCoefficient(), 0.02, 1e-10);
    EXPECT_NEAR(coeffs1.getRollMomentCoefficient(), 0.005, 1e-10);
    EXPECT_NEAR(coeffs1.getPitchMomentCoefficient(), -0.01, 1e-10);
    EXPECT_NEAR(coeffs1.getYawMomentCoefficient(), 0.02, 1e-10);
}

// ExtendedAerodynamicCoefficients Tests

TEST(ExtendedAerodynamicCoefficientsTest, DefaultConstructor) {
    ExtendedAerodynamicCoefficients coeffs;
    
    // Check base coefficients are zero
    EXPECT_DOUBLE_EQ(coeffs.getDragCoefficient(), 0.0);
    EXPECT_DOUBLE_EQ(coeffs.getLiftCoefficient(), 0.0);
    
    // Check stability derivatives are zero
    EXPECT_DOUBLE_EQ(coeffs.getCLalpha(), 0.0);
    EXPECT_DOUBLE_EQ(coeffs.getCDalpha(), 0.0);
    EXPECT_DOUBLE_EQ(coeffs.getCMalpha(), 0.0);
    EXPECT_DOUBLE_EQ(coeffs.getCYbeta(), 0.0);
    EXPECT_DOUBLE_EQ(coeffs.getCLbeta(), 0.0);
    EXPECT_DOUBLE_EQ(coeffs.getCNbeta(), 0.0);
}

TEST(ExtendedAerodynamicCoefficientsTest, ConstructFromBase) {
    AerodynamicCoefficients base(0.3, 0.15, 0.05, 0.01, -0.02, 0.03);
    ExtendedAerodynamicCoefficients extended(base);
    
    // Check base coefficients are copied
    EXPECT_DOUBLE_EQ(extended.getDragCoefficient(), 0.3);
    EXPECT_DOUBLE_EQ(extended.getLiftCoefficient(), 0.15);
    EXPECT_DOUBLE_EQ(extended.getSideForceCoefficient(), 0.05);
    
    // Check stability derivatives are still zero
    EXPECT_DOUBLE_EQ(extended.getCLalpha(), 0.0);
}

TEST(ExtendedAerodynamicCoefficientsTest, SettersAndGetters) {
    ExtendedAerodynamicCoefficients coeffs;
    
    // Set stability derivatives
    coeffs.setCLalpha(5.73);  // ~0.1/deg
    coeffs.setCDalpha(0.5);
    coeffs.setCMalpha(-1.2);
    coeffs.setCYbeta(-0.8);
    coeffs.setCLbeta(-0.1);
    coeffs.setCNbeta(0.15);
    coeffs.setCLq(3.5);
    coeffs.setCMq(-12.0);
    coeffs.setCLp(-0.5);
    coeffs.setCNp(-0.05);
    coeffs.setCLr(0.2);
    coeffs.setCNr(-0.25);
    
    // Verify all values
    EXPECT_DOUBLE_EQ(coeffs.getCLalpha(), 5.73);
    EXPECT_DOUBLE_EQ(coeffs.getCDalpha(), 0.5);
    EXPECT_DOUBLE_EQ(coeffs.getCMalpha(), -1.2);
    EXPECT_DOUBLE_EQ(coeffs.getCYbeta(), -0.8);
    EXPECT_DOUBLE_EQ(coeffs.getCLbeta(), -0.1);
    EXPECT_DOUBLE_EQ(coeffs.getCNbeta(), 0.15);
    EXPECT_DOUBLE_EQ(coeffs.getCLq(), 3.5);
    EXPECT_DOUBLE_EQ(coeffs.getCMq(), -12.0);
    EXPECT_DOUBLE_EQ(coeffs.getCLp(), -0.5);
    EXPECT_DOUBLE_EQ(coeffs.getCNp(), -0.05);
    EXPECT_DOUBLE_EQ(coeffs.getCLr(), 0.2);
    EXPECT_DOUBLE_EQ(coeffs.getCNr(), -0.25);
}

TEST(ExtendedAerodynamicCoefficientsTest, ApplyIncrements) {
    ExtendedAerodynamicCoefficients coeffs;
    
    // Set base coefficients
    coeffs.setDragCoefficient(0.3);
    coeffs.setLiftCoefficient(0.0);
    coeffs.setPitchMomentCoefficient(0.0);
    
    // Set stability derivatives
    coeffs.setCLalpha(5.73);   // 0.1/deg in rad
    coeffs.setCDalpha(0.5);
    coeffs.setCMalpha(-1.2);
    coeffs.setCMq(-12.0);
    
    // Apply increments
    double alpha = 0.0873;  // 5 degrees
    double beta = 0.0;
    double p_norm = 0.0;
    double q_norm = 0.1;
    double r_norm = 0.0;
    
    AerodynamicCoefficients modified = coeffs.applyIncrements(
        alpha, beta, p_norm, q_norm, r_norm);
    
    // Check increments were applied correctly
    EXPECT_NEAR(modified.getLiftCoefficient(), 0.0 + 5.73 * 0.0873, 1e-6);
    EXPECT_NEAR(modified.getDragCoefficient(), 0.3 + 0.5 * 0.0873, 1e-6);
    EXPECT_NEAR(modified.getPitchMomentCoefficient(), 0.0 + (-1.2) * 0.0873 + (-12.0) * 0.1, 1e-6);
}

TEST(ExtendedAerodynamicCoefficientsTest, HasStabilityDerivatives) {
    ExtendedAerodynamicCoefficients coeffs1;
    EXPECT_FALSE(coeffs1.hasStabilityDerivatives());
    
    ExtendedAerodynamicCoefficients coeffs2;
    coeffs2.setCLalpha(5.73);
    EXPECT_TRUE(coeffs2.hasStabilityDerivatives());
}