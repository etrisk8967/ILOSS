#include <gtest/gtest.h>
#include "physics/forces/ForceAggregator.h"
#include "physics/forces/SimpleGravityModel.h"
#include "physics/state/StateVector.h"
#include "core/time/Time.h"
#include "core/math/Vector3D.h"
#include "core/logging/Logger.h"
#include <memory>

using namespace iloss::physics::forces;
using namespace iloss::time;
using namespace iloss::math;
using iloss::physics::StateVector;  // Explicitly use physics::StateVector

/**
 * @brief Mock force model for testing
 */
class MockForceModel : public ForceModel {
public:
    MockForceModel(const std::string& name, ForceModelType type, 
                   const Vector3D& acceleration)
        : ForceModel(name, type)
        , m_acceleration(acceleration)
        , m_updateCalled(false)
        , m_calculateCalled(false) {}

    Vector3D calculateAcceleration(const StateVector& state, 
                                  const Time& time) const override {
        m_calculateCalled = true;
        return m_acceleration;
    }

    bool initialize(const ForceModelConfig& config) override {
        m_config = config;
        return true;
    }

    void update(const Time& time) override {
        m_updateCalled = true;
        m_lastUpdateTime = time;
    }

    bool validate() const override {
        return m_isValid;
    }

    std::unique_ptr<ForceModel> clone() const override {
        auto cloned = std::make_unique<MockForceModel>(m_name, m_type, m_acceleration);
        cloned->m_enabled = m_enabled;
        cloned->m_isValid = m_isValid;
        return cloned;
    }

    // Test helpers
    bool wasUpdateCalled() const { return m_updateCalled; }
    bool wasCalculateCalled() const { return m_calculateCalled; }
    void resetCallFlags() { m_updateCalled = false; m_calculateCalled = false; }
    void setValid(bool valid) { m_isValid = valid; }

private:
    Vector3D m_acceleration;
    mutable bool m_updateCalled;
    mutable bool m_calculateCalled;
    bool m_isValid = true;
    Time m_lastUpdateTime;
};

/**
 * @brief Test fixture for ForceAggregator tests
 */
class ForceAggregatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize the logger before creating ForceAggregator to avoid deadlock
        iloss::logging::Logger::getInstance().initialize(
            "ForceAggregatorTest", 
            "test.log", 
            false,  // No console output for tests
            false,  // No file output for tests
            iloss::logging::LogLevel::Warning  // Only log warnings and above
        );
        
        // Create a default state vector
        Vector3D position(6778137.0, 0.0, 0.0);
        Vector3D velocity(0.0, 7668.0, 0.0);
        m_defaultState = StateVector(position, velocity, 1000.0, 
                                    Time(2025, 6, 28, 12, 0, 0.0));
        
        m_aggregator = std::make_unique<ForceAggregator>();
    }

    StateVector m_defaultState;
    std::unique_ptr<ForceAggregator> m_aggregator;
};

/**
 * @brief Test adding force models
 */
TEST_F(ForceAggregatorTest, AddForceModelTest) {
    // Test adding valid models
    EXPECT_TRUE(m_aggregator->addForceModel(
        std::make_unique<SimpleGravityModel>("Gravity1")));
    EXPECT_EQ(m_aggregator->getModelCount(), 1);
    
    EXPECT_TRUE(m_aggregator->addForceModel(
        std::make_unique<MockForceModel>("Drag1", ForceModelType::Drag, 
                                        Vector3D(0.0, -0.001, 0.0))));
    EXPECT_EQ(m_aggregator->getModelCount(), 2);
    
    // Test adding null model
    EXPECT_FALSE(m_aggregator->addForceModel(nullptr));
    EXPECT_EQ(m_aggregator->getModelCount(), 2);
    
    // Test adding model with duplicate name
    EXPECT_FALSE(m_aggregator->addForceModel(
        std::make_unique<SimpleGravityModel>("Gravity1")));
    EXPECT_EQ(m_aggregator->getModelCount(), 2);
}

/**
 * @brief Test removing force models
 */
TEST_F(ForceAggregatorTest, RemoveForceModelTest) {
    // Add some models
    m_aggregator->addForceModel(std::make_unique<SimpleGravityModel>("Gravity1"));
    m_aggregator->addForceModel(std::make_unique<SimpleGravityModel>("Gravity2"));
    m_aggregator->addForceModel(
        std::make_unique<MockForceModel>("Drag1", ForceModelType::Drag, 
                                        Vector3D::zero()));
    EXPECT_EQ(m_aggregator->getModelCount(), 3);
    
    // Test removing by name
    EXPECT_TRUE(m_aggregator->removeForceModel("Gravity1"));
    EXPECT_EQ(m_aggregator->getModelCount(), 2);
    EXPECT_EQ(m_aggregator->getForceModel("Gravity1"), nullptr);
    
    // Test removing non-existent model
    EXPECT_FALSE(m_aggregator->removeForceModel("NonExistent"));
    EXPECT_EQ(m_aggregator->getModelCount(), 2);
    
    // Test removing by type
    size_t removed = m_aggregator->removeForceModelsByType(ForceModelType::TwoBody);
    EXPECT_EQ(removed, 1);  // Only Gravity2 should be removed
    EXPECT_EQ(m_aggregator->getModelCount(), 1);
}

/**
 * @brief Test getting force models
 */
TEST_F(ForceAggregatorTest, GetForceModelTest) {
    // Add models
    m_aggregator->addForceModel(std::make_unique<SimpleGravityModel>("Gravity1"));
    m_aggregator->addForceModel(
        std::make_unique<MockForceModel>("Drag1", ForceModelType::Drag, 
                                        Vector3D::zero()));
    m_aggregator->addForceModel(
        std::make_unique<MockForceModel>("SRP1", ForceModelType::SolarRadiation, 
                                        Vector3D::zero()));
    
    // Test getting by name
    ForceModel* gravity = m_aggregator->getForceModel("Gravity1");
    ASSERT_NE(gravity, nullptr);
    EXPECT_EQ(gravity->getName(), "Gravity1");
    EXPECT_EQ(gravity->getType(), ForceModelType::TwoBody);
    
    // Test const version
    const ForceAggregator& constAgg = *m_aggregator;
    const ForceModel* constGravity = constAgg.getForceModel("Gravity1");
    ASSERT_NE(constGravity, nullptr);
    EXPECT_EQ(constGravity->getName(), "Gravity1");
    
    // Test getting non-existent model
    EXPECT_EQ(m_aggregator->getForceModel("NonExistent"), nullptr);
    
    // Test getting by type
    auto dragModels = m_aggregator->getForceModelsByType(ForceModelType::Drag);
    EXPECT_EQ(dragModels.size(), 1);
    EXPECT_EQ(dragModels[0]->getName(), "Drag1");
}

/**
 * @brief Test enabling/disabling force models
 */
TEST_F(ForceAggregatorTest, EnableDisableTest) {
    // Add models
    m_aggregator->addForceModel(std::make_unique<SimpleGravityModel>("Gravity1"));
    m_aggregator->addForceModel(std::make_unique<SimpleGravityModel>("Gravity2"));
    m_aggregator->addForceModel(
        std::make_unique<MockForceModel>("Drag1", ForceModelType::Drag, 
                                        Vector3D::zero()));
    
    // Test initial state (all should be enabled)
    EXPECT_EQ(m_aggregator->getEnabledModelCount(), 3);
    
    // Test disabling by name
    EXPECT_TRUE(m_aggregator->setForceModelEnabled("Gravity1", false));
    EXPECT_EQ(m_aggregator->getEnabledModelCount(), 2);
    EXPECT_FALSE(m_aggregator->getForceModel("Gravity1")->isEnabled());
    
    // Test enabling by name
    EXPECT_TRUE(m_aggregator->setForceModelEnabled("Gravity1", true));
    EXPECT_EQ(m_aggregator->getEnabledModelCount(), 3);
    EXPECT_TRUE(m_aggregator->getForceModel("Gravity1")->isEnabled());
    
    // Test disabling non-existent model
    EXPECT_FALSE(m_aggregator->setForceModelEnabled("NonExistent", false));
    
    // Test disabling by type
    size_t affected = m_aggregator->setForceModelsEnabledByType(
        ForceModelType::TwoBody, false);
    EXPECT_EQ(affected, 2);
    EXPECT_EQ(m_aggregator->getEnabledModelCount(), 1);
}

/**
 * @brief Test acceleration calculation
 */
TEST_F(ForceAggregatorTest, CalculateTotalAccelerationTest) {
    // Add models with known accelerations
    m_aggregator->addForceModel(
        std::make_unique<MockForceModel>("Force1", ForceModelType::UserDefined,
                                        Vector3D(1.0, 0.0, 0.0)));
    m_aggregator->addForceModel(
        std::make_unique<MockForceModel>("Force2", ForceModelType::UserDefined,
                                        Vector3D(0.0, 2.0, 0.0)));
    m_aggregator->addForceModel(
        std::make_unique<MockForceModel>("Force3", ForceModelType::UserDefined,
                                        Vector3D(0.0, 0.0, 3.0)));
    
    // Calculate total acceleration
    Time time(2025, 6, 28, 12, 0, 0.0);
    Vector3D totalAccel = m_aggregator->calculateTotalAcceleration(
        m_defaultState, time);
    
    // Check result (should be sum of all accelerations)
    EXPECT_DOUBLE_EQ(totalAccel.x(), 1.0);
    EXPECT_DOUBLE_EQ(totalAccel.y(), 2.0);
    EXPECT_DOUBLE_EQ(totalAccel.z(), 3.0);
    
    // Disable one model and recalculate
    m_aggregator->setForceModelEnabled("Force2", false);
    totalAccel = m_aggregator->calculateTotalAcceleration(m_defaultState, time);
    
    EXPECT_DOUBLE_EQ(totalAccel.x(), 1.0);
    EXPECT_DOUBLE_EQ(totalAccel.y(), 0.0);
    EXPECT_DOUBLE_EQ(totalAccel.z(), 3.0);
}

/**
 * @brief Test acceleration calculation with breakdown
 */
TEST_F(ForceAggregatorTest, CalculateWithBreakdownTest) {
    // Add models
    m_aggregator->addForceModel(
        std::make_unique<MockForceModel>("Gravity", ForceModelType::TwoBody,
                                        Vector3D(-10.0, 0.0, 0.0)));
    m_aggregator->addForceModel(
        std::make_unique<MockForceModel>("Drag", ForceModelType::Drag,
                                        Vector3D(0.0, -0.1, 0.0)));
    m_aggregator->addForceModel(
        std::make_unique<MockForceModel>("SRP", ForceModelType::SolarRadiation,
                                        Vector3D(0.0, 0.0, 0.001)));
    
    // Calculate with breakdown
    Time time(2025, 6, 28, 12, 0, 0.0);
    std::vector<ForceAggregator::AccelerationContribution> contributions;
    Vector3D totalAccel = m_aggregator->calculateTotalAccelerationWithBreakdown(
        m_defaultState, time, contributions);
    
    // Check total
    EXPECT_DOUBLE_EQ(totalAccel.x(), -10.0);
    EXPECT_DOUBLE_EQ(totalAccel.y(), -0.1);
    EXPECT_DOUBLE_EQ(totalAccel.z(), 0.001);
    
    // Check contributions
    EXPECT_EQ(contributions.size(), 3);
    
    // Should be sorted by magnitude (gravity first)
    EXPECT_EQ(contributions[0].modelName, "Gravity");
    EXPECT_DOUBLE_EQ(contributions[0].magnitude, 10.0);
    EXPECT_GT(contributions[0].percentageContribution, 99.0);
    
    EXPECT_EQ(contributions[1].modelName, "Drag");
    EXPECT_DOUBLE_EQ(contributions[1].magnitude, 0.1);
    
    EXPECT_EQ(contributions[2].modelName, "SRP");
    EXPECT_DOUBLE_EQ(contributions[2].magnitude, 0.001);
}

/**
 * @brief Test update functionality
 */
TEST_F(ForceAggregatorTest, UpdateAllModelsTest) {
    // Add mock models
    auto* model1 = new MockForceModel("Model1", ForceModelType::UserDefined,
                                     Vector3D::zero());
    auto* model2 = new MockForceModel("Model2", ForceModelType::UserDefined,
                                     Vector3D::zero());
    
    m_aggregator->addForceModel(std::unique_ptr<ForceModel>(model1));
    m_aggregator->addForceModel(std::unique_ptr<ForceModel>(model2));
    
    // Disable one model
    m_aggregator->setForceModelEnabled("Model2", false);
    
    // Update all models
    Time updateTime(2025, 6, 28, 15, 30, 0.0);
    m_aggregator->updateAllModels(updateTime);
    
    // Check that only enabled model was updated
    EXPECT_TRUE(model1->wasUpdateCalled());
    EXPECT_FALSE(model2->wasUpdateCalled());
}

/**
 * @brief Test validation
 */
TEST_F(ForceAggregatorTest, ValidateAllTest) {
    // Add models
    auto* validModel = new MockForceModel("Valid", ForceModelType::UserDefined,
                                         Vector3D::zero());
    auto* invalidModel = new MockForceModel("Invalid", ForceModelType::UserDefined,
                                           Vector3D::zero());
    invalidModel->setValid(false);
    
    m_aggregator->addForceModel(std::unique_ptr<ForceModel>(validModel));
    m_aggregator->addForceModel(std::unique_ptr<ForceModel>(invalidModel));
    
    // Validate all should fail
    EXPECT_FALSE(m_aggregator->validateAll());
    
    // Remove invalid model
    m_aggregator->removeForceModel("Invalid");
    
    // Now should pass
    EXPECT_TRUE(m_aggregator->validateAll());
}

/**
 * @brief Test cloning
 */
TEST_F(ForceAggregatorTest, CloneTest) {
    // Set up original aggregator
    m_aggregator->addForceModel(std::make_unique<SimpleGravityModel>("Gravity"));
    m_aggregator->addForceModel(
        std::make_unique<MockForceModel>("Drag", ForceModelType::Drag,
                                        Vector3D(0.0, -0.1, 0.0)));
    m_aggregator->setForceModelEnabled("Drag", false);
    
    // Clone
    auto cloned = m_aggregator->clone();
    
    // Verify clone
    EXPECT_EQ(cloned->getModelCount(), 2);
    EXPECT_EQ(cloned->getEnabledModelCount(), 1);
    
    // Verify models were cloned
    EXPECT_NE(cloned->getForceModel("Gravity"), 
              m_aggregator->getForceModel("Gravity"));
    EXPECT_EQ(cloned->getForceModel("Gravity")->getName(), "Gravity");
    
    // Verify independence
    m_aggregator->setForceModelEnabled("Drag", true);
    EXPECT_FALSE(cloned->getForceModel("Drag")->isEnabled());
}

/**
 * @brief Test utility functions
 */
TEST_F(ForceAggregatorTest, UtilityFunctionsTest) {
    // Add models
    m_aggregator->addForceModel(std::make_unique<SimpleGravityModel>("Gravity"));
    m_aggregator->addForceModel(
        std::make_unique<MockForceModel>("Drag", ForceModelType::Drag,
                                        Vector3D::zero()));
    
    // Test getting model names
    auto names = m_aggregator->getModelNames();
    EXPECT_EQ(names.size(), 2);
    EXPECT_TRUE(std::find(names.begin(), names.end(), "Gravity") != names.end());
    EXPECT_TRUE(std::find(names.begin(), names.end(), "Drag") != names.end());
    
    // Test toString
    std::string summary = m_aggregator->toString();
    EXPECT_TRUE(summary.find("2 models") != std::string::npos);
    EXPECT_TRUE(summary.find("2 enabled") != std::string::npos);
    EXPECT_TRUE(summary.find("Gravity") != std::string::npos);
    EXPECT_TRUE(summary.find("Drag") != std::string::npos);
    
    // Test clear
    m_aggregator->clear();
    EXPECT_EQ(m_aggregator->getModelCount(), 0);
}