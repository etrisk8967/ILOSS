#include <gtest/gtest.h>
#include "physics/dynamics/TorqueAggregator.h"
#include "physics/dynamics/DynamicsState.h"
#include <memory>

using namespace iloss::physics::dynamics;
using namespace iloss::math;

// Mock torque model for testing
class MockTorqueModel : public ITorqueModel {
public:
    MockTorqueModel(const std::string& name, TorqueModelType type, const Vector3D& torque)
        : m_name(name), m_type(type), m_torque(torque), m_enabled(true) {}

    Vector3D calculateTorque(const DynamicsState& /*state*/, double /*time*/) const override {
        return m_enabled ? m_torque : Vector3D();
    }

    TorqueModelType getType() const override { return m_type; }
    std::string getName() const override { return m_name; }
    bool isEnabled() const override { return m_enabled; }
    void setEnabled(bool enabled) override { m_enabled = enabled; }
    
    void configure(const TorqueModelConfig& /*config*/) override {
        // Mock implementation
    }
    
    std::unique_ptr<ITorqueModel> clone() const override {
        return std::make_unique<MockTorqueModel>(m_name, m_type, m_torque);
    }

    void setTorque(const Vector3D& torque) { m_torque = torque; }

private:
    std::string m_name;
    TorqueModelType m_type;
    Vector3D m_torque;
    bool m_enabled;
};

class TorqueAggregatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        aggregator = std::make_unique<TorqueAggregator>();
        
        // Create test state
        testState = std::make_unique<DynamicsState>(
            Vector3D(7000000.0, 0.0, 0.0),
            Vector3D(0.0, 7500.0, 0.0),
            1000.0,
            0.0,
            Quaternion::identity(),
            Vector3D(0.0, 0.0, 0.1)
        );
    }

    std::unique_ptr<TorqueAggregator> aggregator;
    std::unique_ptr<DynamicsState> testState;
};

TEST_F(TorqueAggregatorTest, EmptyAggregator) {
    EXPECT_EQ(aggregator->getModelCount(), 0);
    
    Vector3D totalTorque = aggregator->calculateTotalTorque(*testState, 0.0);
    EXPECT_EQ(totalTorque, Vector3D());
    
    EXPECT_TRUE(aggregator->getLastContributions().empty());
}

TEST_F(TorqueAggregatorTest, AddModels) {
    auto model1 = std::make_unique<MockTorqueModel>(
        "GravityGradient", TorqueModelType::GravityGradient, Vector3D(1.0, 0.0, 0.0));
    auto model2 = std::make_unique<MockTorqueModel>(
        "Aerodynamic", TorqueModelType::Aerodynamic, Vector3D(0.0, 2.0, 0.0));
    
    aggregator->addModel(std::move(model1));
    aggregator->addModel(std::move(model2));
    
    EXPECT_EQ(aggregator->getModelCount(), 2);
}

TEST_F(TorqueAggregatorTest, CalculateTotalTorque) {
    Vector3D torque1(1.0, 2.0, 3.0);
    Vector3D torque2(0.5, -1.0, 0.5);
    Vector3D torque3(-0.5, 0.5, 1.0);
    
    aggregator->addModel(std::make_unique<MockTorqueModel>(
        "Model1", TorqueModelType::GravityGradient, torque1));
    aggregator->addModel(std::make_unique<MockTorqueModel>(
        "Model2", TorqueModelType::Aerodynamic, torque2));
    aggregator->addModel(std::make_unique<MockTorqueModel>(
        "Model3", TorqueModelType::SolarRadiation, torque3));
    
    Vector3D totalTorque = aggregator->calculateTotalTorque(*testState, 0.0);
    Vector3D expectedTotal = torque1 + torque2 + torque3;
    
    EXPECT_NEAR(totalTorque.x(), expectedTotal.x(), 1e-10);
    EXPECT_NEAR(totalTorque.y(), expectedTotal.y(), 1e-10);
    EXPECT_NEAR(totalTorque.z(), expectedTotal.z(), 1e-10);
}

TEST_F(TorqueAggregatorTest, DisabledModels) {
    auto model1 = std::make_unique<MockTorqueModel>(
        "Model1", TorqueModelType::GravityGradient, Vector3D(1.0, 0.0, 0.0));
    auto model2 = std::make_unique<MockTorqueModel>(
        "Model2", TorqueModelType::Aerodynamic, Vector3D(0.0, 2.0, 0.0));
    
    model2->setEnabled(false);
    
    aggregator->addModel(std::move(model1));
    aggregator->addModel(std::move(model2));
    
    Vector3D totalTorque = aggregator->calculateTotalTorque(*testState, 0.0);
    
    // Only model1's torque should contribute
    EXPECT_EQ(totalTorque, Vector3D(1.0, 0.0, 0.0));
}

TEST_F(TorqueAggregatorTest, SetModelEnabled) {
    aggregator->addModel(std::make_unique<MockTorqueModel>(
        "Model1", TorqueModelType::GravityGradient, Vector3D(1.0, 0.0, 0.0)));
    aggregator->addModel(std::make_unique<MockTorqueModel>(
        "Model2", TorqueModelType::GravityGradient, Vector3D(2.0, 0.0, 0.0)));
    aggregator->addModel(std::make_unique<MockTorqueModel>(
        "Model3", TorqueModelType::Aerodynamic, Vector3D(3.0, 0.0, 0.0)));
    
    // Disable all gravity gradient models
    size_t affected = aggregator->setModelEnabled(TorqueModelType::GravityGradient, false);
    EXPECT_EQ(affected, 2);
    
    Vector3D totalTorque = aggregator->calculateTotalTorque(*testState, 0.0);
    EXPECT_EQ(totalTorque, Vector3D(3.0, 0.0, 0.0));  // Only aerodynamic
    
    // Re-enable gravity gradient models
    affected = aggregator->setModelEnabled(TorqueModelType::GravityGradient, true);
    EXPECT_EQ(affected, 2);
    
    totalTorque = aggregator->calculateTotalTorque(*testState, 0.0);
    EXPECT_EQ(totalTorque, Vector3D(6.0, 0.0, 0.0));  // All models
}

TEST_F(TorqueAggregatorTest, IsModelTypeEnabled) {
    aggregator->addModel(std::make_unique<MockTorqueModel>(
        "Model1", TorqueModelType::GravityGradient, Vector3D(1.0, 0.0, 0.0)));
    
    EXPECT_TRUE(aggregator->isModelTypeEnabled(TorqueModelType::GravityGradient));
    EXPECT_FALSE(aggregator->isModelTypeEnabled(TorqueModelType::Aerodynamic));
    
    aggregator->setModelEnabled(TorqueModelType::GravityGradient, false);
    EXPECT_FALSE(aggregator->isModelTypeEnabled(TorqueModelType::GravityGradient));
}

TEST_F(TorqueAggregatorTest, LastContributions) {
    Vector3D torque1(1.0, 0.0, 0.0);
    Vector3D torque2(0.0, 2.0, 0.0);
    Vector3D torque3(0.0, 0.0, 3.0);
    
    aggregator->addModel(std::make_unique<MockTorqueModel>(
        "GravGrad", TorqueModelType::GravityGradient, torque1));
    aggregator->addModel(std::make_unique<MockTorqueModel>(
        "Aero", TorqueModelType::Aerodynamic, torque2));
    aggregator->addModel(std::make_unique<MockTorqueModel>(
        "SRP", TorqueModelType::SolarRadiation, torque3));
    
    aggregator->calculateTotalTorque(*testState, 0.0);
    
    const auto& contributions = aggregator->getLastContributions();
    EXPECT_EQ(contributions.size(), 3);
    
    // Check each contribution
    EXPECT_EQ(contributions[0].modelName, "GravGrad");
    EXPECT_EQ(contributions[0].modelType, TorqueModelType::GravityGradient);
    EXPECT_EQ(contributions[0].torque, torque1);
    EXPECT_DOUBLE_EQ(contributions[0].magnitude, 1.0);
    
    EXPECT_EQ(contributions[1].modelName, "Aero");
    EXPECT_EQ(contributions[1].modelType, TorqueModelType::Aerodynamic);
    EXPECT_EQ(contributions[1].torque, torque2);
    EXPECT_DOUBLE_EQ(contributions[1].magnitude, 2.0);
    
    EXPECT_EQ(contributions[2].modelName, "SRP");
    EXPECT_EQ(contributions[2].modelType, TorqueModelType::SolarRadiation);
    EXPECT_EQ(contributions[2].torque, torque3);
    EXPECT_DOUBLE_EQ(contributions[2].magnitude, 3.0);
}

TEST_F(TorqueAggregatorTest, GetTorqueByType) {
    aggregator->addModel(std::make_unique<MockTorqueModel>(
        "GG1", TorqueModelType::GravityGradient, Vector3D(1.0, 0.0, 0.0)));
    aggregator->addModel(std::make_unique<MockTorqueModel>(
        "GG2", TorqueModelType::GravityGradient, Vector3D(2.0, 0.0, 0.0)));
    aggregator->addModel(std::make_unique<MockTorqueModel>(
        "Aero", TorqueModelType::Aerodynamic, Vector3D(0.0, 3.0, 0.0)));
    
    aggregator->calculateTotalTorque(*testState, 0.0);
    
    Vector3D ggTorque = aggregator->getTorqueByType(TorqueModelType::GravityGradient);
    EXPECT_EQ(ggTorque, Vector3D(3.0, 0.0, 0.0));  // Sum of both GG models
    
    Vector3D aeroTorque = aggregator->getTorqueByType(TorqueModelType::Aerodynamic);
    EXPECT_EQ(aeroTorque, Vector3D(0.0, 3.0, 0.0));
    
    Vector3D srpTorque = aggregator->getTorqueByType(TorqueModelType::SolarRadiation);
    EXPECT_EQ(srpTorque, Vector3D());  // No SRP models
}

TEST_F(TorqueAggregatorTest, ContributionPercentages) {
    aggregator->addModel(std::make_unique<MockTorqueModel>(
        "Model1", TorqueModelType::GravityGradient, Vector3D(3.0, 0.0, 0.0)));
    aggregator->addModel(std::make_unique<MockTorqueModel>(
        "Model2", TorqueModelType::Aerodynamic, Vector3D(0.0, 4.0, 0.0)));
    aggregator->addModel(std::make_unique<MockTorqueModel>(
        "Model3", TorqueModelType::GravityGradient, Vector3D(0.0, 0.0, 5.0)));
    
    aggregator->calculateTotalTorque(*testState, 0.0);
    
    auto percentages = aggregator->getContributionPercentages();
    
    // Total magnitude: 3 + 4 + 5 = 12
    // GravityGradient: 3 + 5 = 8 -> 66.67%
    // Aerodynamic: 4 -> 33.33%
    
    EXPECT_NEAR(percentages[TorqueModelType::GravityGradient], 66.67, 0.01);
    EXPECT_NEAR(percentages[TorqueModelType::Aerodynamic], 33.33, 0.01);
}

TEST_F(TorqueAggregatorTest, ClearModels) {
    aggregator->addModel(std::make_unique<MockTorqueModel>(
        "Model1", TorqueModelType::GravityGradient, Vector3D(1.0, 0.0, 0.0)));
    aggregator->addModel(std::make_unique<MockTorqueModel>(
        "Model2", TorqueModelType::Aerodynamic, Vector3D(0.0, 2.0, 0.0)));
    
    EXPECT_EQ(aggregator->getModelCount(), 2);
    
    aggregator->clearModels();
    
    EXPECT_EQ(aggregator->getModelCount(), 0);
    EXPECT_TRUE(aggregator->getLastContributions().empty());
}

TEST_F(TorqueAggregatorTest, Clone) {
    aggregator->addModel(std::make_unique<MockTorqueModel>(
        "Model1", TorqueModelType::GravityGradient, Vector3D(1.0, 0.0, 0.0)));
    aggregator->addModel(std::make_unique<MockTorqueModel>(
        "Model2", TorqueModelType::Aerodynamic, Vector3D(0.0, 2.0, 0.0)));
    
    auto cloned = aggregator->clone();
    
    EXPECT_EQ(cloned->getModelCount(), 2);
    
    // Verify cloned aggregator produces same results
    Vector3D originalTorque = aggregator->calculateTotalTorque(*testState, 0.0);
    Vector3D clonedTorque = cloned->calculateTotalTorque(*testState, 0.0);
    
    EXPECT_EQ(originalTorque, clonedTorque);
    
    // Verify independence
    cloned->clearModels();
    EXPECT_EQ(cloned->getModelCount(), 0);
    EXPECT_EQ(aggregator->getModelCount(), 2);
}