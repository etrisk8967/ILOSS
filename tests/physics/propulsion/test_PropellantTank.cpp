#include <gtest/gtest.h>
#include "physics/forces/propulsion/PropellantTank.h"
#include <limits>

using namespace iloss::physics::propulsion;

/**
 * @brief Test fixture for PropellantTank tests
 */
class PropellantTankTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test tanks
        m_tank1 = std::make_unique<PropellantTank>(
            "MainTank", PropellantType::RP1_LOX, 1000.0, 800.0);
        
        m_tank2 = std::make_unique<PropellantTank>(
            "CustomTank", "MyFuel", 500.0, 500.0);
    }
    
    std::unique_ptr<PropellantTank> m_tank1;
    std::unique_ptr<PropellantTank> m_tank2;
};

/**
 * @brief Test basic construction and getters
 */
TEST_F(PropellantTankTest, Construction) {
    // Test standard propellant type
    EXPECT_EQ(m_tank1->getName(), "MainTank");
    EXPECT_EQ(m_tank1->getPropellantType(), PropellantType::RP1_LOX);
    EXPECT_EQ(m_tank1->getPropellantTypeString(), "RP-1/LOX");
    EXPECT_DOUBLE_EQ(m_tank1->getMaxCapacity(), 1000.0);
    EXPECT_DOUBLE_EQ(m_tank1->getCurrentMass(), 800.0);
    EXPECT_DOUBLE_EQ(m_tank1->getFillFraction(), 0.8);
    EXPECT_FALSE(m_tank1->isEmpty());
    EXPECT_FALSE(m_tank1->isFull());
    
    // Test custom propellant type
    EXPECT_EQ(m_tank2->getName(), "CustomTank");
    EXPECT_EQ(m_tank2->getPropellantType(), PropellantType::CUSTOM);
    EXPECT_EQ(m_tank2->getPropellantTypeString(), "MyFuel");
    EXPECT_DOUBLE_EQ(m_tank2->getMaxCapacity(), 500.0);
    EXPECT_DOUBLE_EQ(m_tank2->getCurrentMass(), 500.0);
    EXPECT_DOUBLE_EQ(m_tank2->getFillFraction(), 1.0);
    EXPECT_FALSE(m_tank2->isEmpty());
    EXPECT_TRUE(m_tank2->isFull());
}

/**
 * @brief Test invalid construction parameters
 */
TEST(PropellantTankConstructionTest, InvalidParameters) {
    // Invalid capacity
    EXPECT_THROW(PropellantTank("Tank", PropellantType::RP1_LOX, 0.0, 0.0),
                 std::invalid_argument);
    EXPECT_THROW(PropellantTank("Tank", PropellantType::RP1_LOX, -100.0, 0.0),
                 std::invalid_argument);
    
    // Negative mass
    EXPECT_THROW(PropellantTank("Tank", PropellantType::RP1_LOX, 1000.0, -10.0),
                 std::invalid_argument);
    
    // Mass exceeds capacity
    EXPECT_THROW(PropellantTank("Tank", PropellantType::RP1_LOX, 1000.0, 1001.0),
                 std::invalid_argument);
    
    // Empty tank name
    EXPECT_THROW(PropellantTank("", PropellantType::RP1_LOX, 1000.0, 500.0),
                 std::invalid_argument);
    
    // Empty custom propellant name
    EXPECT_THROW(PropellantTank("Tank", "", 1000.0, 500.0),
                 std::invalid_argument);
}

/**
 * @brief Test mass consumption
 */
TEST_F(PropellantTankTest, ConsumeMass) {
    // Normal consumption
    double consumed = m_tank1->consumeMass(100.0);
    EXPECT_DOUBLE_EQ(consumed, 100.0);
    EXPECT_DOUBLE_EQ(m_tank1->getCurrentMass(), 700.0);
    
    // Consume more than available
    consumed = m_tank1->consumeMass(800.0);
    EXPECT_DOUBLE_EQ(consumed, 700.0);  // Only what was available
    EXPECT_DOUBLE_EQ(m_tank1->getCurrentMass(), 0.0);
    EXPECT_TRUE(m_tank1->isEmpty());
    
    // Try to consume from empty tank
    consumed = m_tank1->consumeMass(100.0);
    EXPECT_DOUBLE_EQ(consumed, 0.0);
    
    // Negative mass consumption
    EXPECT_THROW(m_tank1->consumeMass(-10.0), std::invalid_argument);
}

/**
 * @brief Test available mass with minimum fraction
 */
TEST_F(PropellantTankTest, AvailableMass) {
    // Full tank has all mass available
    EXPECT_DOUBLE_EQ(m_tank1->getAvailableMass(), 800.0);
    
    // Consume most of the mass (leaving slightly more than MIN_PROPELLANT_FRACTION)
    m_tank1->consumeMass(799.998);  // Leaves 0.002 kg (0.0002% of 1000 kg capacity)
    
    // Should still have a tiny amount available
    EXPECT_GT(m_tank1->getAvailableMass(), 0.0);
    EXPECT_FALSE(m_tank1->isEmpty());
    
    // Consume to below minimum fraction
    m_tank1->consumeMass(0.0015);  // Leaves 0.0005 kg (below 0.001 kg threshold)
    
    // Now should be considered empty
    EXPECT_DOUBLE_EQ(m_tank1->getAvailableMass(), 0.0);
    EXPECT_TRUE(m_tank1->isEmpty());
}

/**
 * @brief Test mass addition (refueling)
 */
TEST_F(PropellantTankTest, AddMass) {
    // Start with partially full tank
    m_tank1->setCurrentMass(500.0);
    
    // Normal addition
    double added = m_tank1->addMass(200.0);
    EXPECT_DOUBLE_EQ(added, 200.0);
    EXPECT_DOUBLE_EQ(m_tank1->getCurrentMass(), 700.0);
    
    // Try to overfill
    added = m_tank1->addMass(400.0);
    EXPECT_DOUBLE_EQ(added, 300.0);  // Only fills to capacity
    EXPECT_DOUBLE_EQ(m_tank1->getCurrentMass(), 1000.0);
    EXPECT_TRUE(m_tank1->isFull());
    
    // Try to add to full tank
    added = m_tank1->addMass(100.0);
    EXPECT_DOUBLE_EQ(added, 0.0);
    
    // Negative mass addition
    EXPECT_THROW(m_tank1->addMass(-10.0), std::invalid_argument);
}

/**
 * @brief Test flow rate calculations
 */
TEST_F(PropellantTankTest, FlowRateCalculations) {
    // Can sustain flow rate
    EXPECT_TRUE(m_tank1->canSustainFlowRate(10.0, 60.0));  // 10 kg/s for 60s = 600 kg
    EXPECT_FALSE(m_tank1->canSustainFlowRate(10.0, 100.0)); // 10 kg/s for 100s = 1000 kg > 800 kg
    
    // Max burn duration
    double duration = m_tank1->getMaxBurnDuration(10.0);
    EXPECT_DOUBLE_EQ(duration, 80.0);  // 800 kg / 10 kg/s = 80s
    
    // Zero flow rate
    duration = m_tank1->getMaxBurnDuration(0.0);
    EXPECT_EQ(duration, std::numeric_limits<double>::infinity());
    
    // Negative flow rate
    EXPECT_THROW(m_tank1->getMaxBurnDuration(-10.0), std::invalid_argument);
}

/**
 * @brief Test tank operations
 */
TEST_F(PropellantTankTest, TankOperations) {
    // Refill
    m_tank1->setCurrentMass(100.0);
    m_tank1->refill();
    EXPECT_DOUBLE_EQ(m_tank1->getCurrentMass(), 1000.0);
    EXPECT_TRUE(m_tank1->isFull());
    
    // Empty
    m_tank1->empty();
    EXPECT_DOUBLE_EQ(m_tank1->getCurrentMass(), 0.0);
    EXPECT_TRUE(m_tank1->isEmpty());
    
    // Set current mass
    m_tank1->setCurrentMass(750.0);
    EXPECT_DOUBLE_EQ(m_tank1->getCurrentMass(), 750.0);
    
    // Invalid set current mass
    EXPECT_THROW(m_tank1->setCurrentMass(-100.0), std::invalid_argument);
    EXPECT_THROW(m_tank1->setCurrentMass(1100.0), std::invalid_argument);
}

/**
 * @brief Test cloning
 */
TEST_F(PropellantTankTest, Cloning) {
    // Clone standard tank
    auto clone1 = m_tank1->clone();
    EXPECT_EQ(clone1->getName(), m_tank1->getName());
    EXPECT_EQ(clone1->getPropellantType(), m_tank1->getPropellantType());
    EXPECT_DOUBLE_EQ(clone1->getMaxCapacity(), m_tank1->getMaxCapacity());
    EXPECT_DOUBLE_EQ(clone1->getCurrentMass(), m_tank1->getCurrentMass());
    
    // Modify original shouldn't affect clone
    m_tank1->consumeMass(100.0);
    EXPECT_DOUBLE_EQ(m_tank1->getCurrentMass(), 700.0);
    EXPECT_DOUBLE_EQ(clone1->getCurrentMass(), 800.0);
    
    // Clone custom tank
    auto clone2 = m_tank2->clone();
    EXPECT_EQ(clone2->getPropellantTypeString(), "MyFuel");
}

/**
 * @brief Test string representation
 */
TEST_F(PropellantTankTest, ToString) {
    std::string str = m_tank1->toString();
    EXPECT_NE(str.find("MainTank"), std::string::npos);
    EXPECT_NE(str.find("RP-1/LOX"), std::string::npos);
    EXPECT_NE(str.find("800.0/1000.0"), std::string::npos);
    EXPECT_NE(str.find("80.0%"), std::string::npos);
    
    // Check empty tank
    m_tank1->empty();
    str = m_tank1->toString();
    EXPECT_NE(str.find("[EMPTY]"), std::string::npos);
    
    // Check full tank
    m_tank2->refill();
    str = m_tank2->toString();
    EXPECT_NE(str.find("[FULL]"), std::string::npos);
}

/**
 * @brief Test edge cases
 */
TEST_F(PropellantTankTest, EdgeCases) {
    // Very small tank
    PropellantTank smallTank("Small", PropellantType::HYDRAZINE, 0.001, 0.001);
    EXPECT_DOUBLE_EQ(smallTank.getMaxCapacity(), 0.001);
    
    // Very large tank
    PropellantTank largeTank("Large", PropellantType::LH2_LOX, 1e6, 5e5);
    EXPECT_DOUBLE_EQ(largeTank.getMaxCapacity(), 1e6);
    
    // Zero consumption/addition
    double consumed = m_tank1->consumeMass(0.0);
    EXPECT_DOUBLE_EQ(consumed, 0.0);
    EXPECT_DOUBLE_EQ(m_tank1->getCurrentMass(), 800.0);
    
    double added = m_tank1->addMass(0.0);
    EXPECT_DOUBLE_EQ(added, 0.0);
    EXPECT_DOUBLE_EQ(m_tank1->getCurrentMass(), 800.0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}