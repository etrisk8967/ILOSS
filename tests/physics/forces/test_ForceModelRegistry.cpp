#include <gtest/gtest.h>
#include "physics/forces/ForceModelRegistry.h"
#include "physics/forces/SimpleGravityModel.h"
#include "core/logging/Logger.h"
#include <thread>
#include <vector>

using namespace iloss::physics::forces;

/**
 * @brief Test fixture for ForceModelRegistry tests
 */
class ForceModelRegistryTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize the logger before using ForceModelRegistry to avoid deadlock
        iloss::logging::Logger::getInstance().initialize(
            "ForceModelRegistryTest", 
            "test.log", 
            false,  // No console output for tests
            false,  // No file output for tests
            iloss::logging::LogLevel::Warning  // Only log warnings and above
        );
        
        // Clear the registry before each test
        ForceModelRegistry::getInstance().clear();
    }
    
    void TearDown() override {
        // Clear the registry after each test
        ForceModelRegistry::getInstance().clear();
    }
};

/**
 * @brief Test singleton behavior
 */
TEST_F(ForceModelRegistryTest, SingletonTest) {
    // Get multiple references to ensure they're the same instance
    auto& registry1 = ForceModelRegistry::getInstance();
    auto& registry2 = ForceModelRegistry::getInstance();
    
    // They should be the same instance
    EXPECT_EQ(&registry1, &registry2);
}

/**
 * @brief Test basic registration
 */
TEST_F(ForceModelRegistryTest, BasicRegistrationTest) {
    auto& registry = ForceModelRegistry::getInstance();
    
    // Create a factory function for SimpleGravityModel
    auto gravityFactory = [](const std::string& name) {
        return std::make_unique<SimpleGravityModel>(name);
    };
    
    // Register the model
    EXPECT_TRUE(registry.registerForceModel(
        ForceModelType::TwoBody,
        "TwoBody",
        "Two-body gravitational force model",
        gravityFactory));
    
    // Verify registration
    EXPECT_TRUE(registry.isRegistered(ForceModelType::TwoBody));
    EXPECT_TRUE(registry.isRegistered("TwoBody"));
    
    // Try to register again (should fail)
    EXPECT_FALSE(registry.registerForceModel(
        ForceModelType::TwoBody,
        "TwoBody",
        "Duplicate registration",
        gravityFactory));
}

/**
 * @brief Test registration with ForceModelInfo
 */
TEST_F(ForceModelRegistryTest, InfoRegistrationTest) {
    auto& registry = ForceModelRegistry::getInstance();
    
    // Create force model info
    ForceModelInfo info;
    info.type = ForceModelType::Drag;
    info.typeName = "AtmosphericDrag";
    info.description = "Atmospheric drag force model";
    info.factory = [](const std::string& name) {
        return std::make_unique<SimpleGravityModel>(name);  // Using SimpleGravity as placeholder
    };
    info.requiredParams = {"drag_coefficient", "area", "mass"};
    info.optionalParams = {"atmosphere_model", "rotation_rate"};
    
    // Register
    EXPECT_TRUE(registry.registerForceModel(ForceModelType::Drag, info));
    
    // Verify info retrieval
    const ForceModelInfo* retrievedInfo = registry.getForceModelInfo(ForceModelType::Drag);
    ASSERT_NE(retrievedInfo, nullptr);
    EXPECT_EQ(retrievedInfo->typeName, "AtmosphericDrag");
    EXPECT_EQ(retrievedInfo->description, "Atmospheric drag force model");
    EXPECT_EQ(retrievedInfo->requiredParams.size(), 3);
    EXPECT_EQ(retrievedInfo->optionalParams.size(), 2);
    
    // Test retrieval by type name
    const ForceModelInfo* infoByName = registry.getForceModelInfo("AtmosphericDrag");
    ASSERT_NE(infoByName, nullptr);
    EXPECT_EQ(infoByName->type, ForceModelType::Drag);
}

/**
 * @brief Test unregistration
 */
TEST_F(ForceModelRegistryTest, UnregistrationTest) {
    auto& registry = ForceModelRegistry::getInstance();
    
    // Register a model
    auto factory = [](const std::string& name) {
        return std::make_unique<SimpleGravityModel>(name);
    };
    registry.registerForceModel(ForceModelType::TwoBody, "TwoBody", 
                               "Test model", factory);
    
    // Verify it's registered
    EXPECT_TRUE(registry.isRegistered(ForceModelType::TwoBody));
    
    // Unregister
    EXPECT_TRUE(registry.unregisterForceModel(ForceModelType::TwoBody));
    
    // Verify it's no longer registered
    EXPECT_FALSE(registry.isRegistered(ForceModelType::TwoBody));
    EXPECT_FALSE(registry.isRegistered("TwoBody"));
    
    // Try to unregister again (should fail)
    EXPECT_FALSE(registry.unregisterForceModel(ForceModelType::TwoBody));
}

/**
 * @brief Test force model creation
 */
TEST_F(ForceModelRegistryTest, CreateForceModelTest) {
    auto& registry = ForceModelRegistry::getInstance();
    
    // Register models
    registry.registerForceModel(
        ForceModelType::TwoBody,
        "TwoBody",
        "Two-body gravity",
        [](const std::string& name) {
            return std::make_unique<SimpleGravityModel>(name);
        });
    
    // Create by type
    auto model1 = registry.createForceModel(ForceModelType::TwoBody, "EarthGravity");
    ASSERT_NE(model1, nullptr);
    EXPECT_EQ(model1->getName(), "EarthGravity");
    EXPECT_EQ(model1->getType(), ForceModelType::TwoBody);
    
    // Create by type name
    auto model2 = registry.createForceModel("TwoBody", "MoonGravity");
    ASSERT_NE(model2, nullptr);
    EXPECT_EQ(model2->getName(), "MoonGravity");
    EXPECT_EQ(model2->getType(), ForceModelType::TwoBody);
    
    // Try to create unregistered type
    auto model3 = registry.createForceModel(ForceModelType::Thrust, "TestThrust");
    EXPECT_EQ(model3, nullptr);
    
    auto model4 = registry.createForceModel("UnknownType", "Test");
    EXPECT_EQ(model4, nullptr);
}

/**
 * @brief Test getting registered types
 */
TEST_F(ForceModelRegistryTest, GetRegisteredTypesTest) {
    auto& registry = ForceModelRegistry::getInstance();
    
    // Initially empty
    EXPECT_EQ(registry.getRegisteredTypes().size(), 0);
    EXPECT_EQ(registry.getRegisteredTypeNames().size(), 0);
    
    // Register some models
    auto factory = [](const std::string& name) {
        return std::make_unique<SimpleGravityModel>(name);
    };
    
    registry.registerForceModel(ForceModelType::TwoBody, "TwoBody", 
                               "Two-body", factory);
    registry.registerForceModel(ForceModelType::Drag, "Drag", 
                               "Atmospheric drag", factory);
    registry.registerForceModel(ForceModelType::SolarRadiation, "SolarRadiation",
                               "Solar radiation pressure", factory);
    
    // Get registered types
    auto types = registry.getRegisteredTypes();
    EXPECT_EQ(types.size(), 3);
    EXPECT_TRUE(std::find(types.begin(), types.end(), 
                         ForceModelType::TwoBody) != types.end());
    EXPECT_TRUE(std::find(types.begin(), types.end(), 
                         ForceModelType::Drag) != types.end());
    EXPECT_TRUE(std::find(types.begin(), types.end(), 
                         ForceModelType::SolarRadiation) != types.end());
    
    // Get type names (should be sorted)
    auto names = registry.getRegisteredTypeNames();
    EXPECT_EQ(names.size(), 3);
    EXPECT_EQ(names[0], "Drag");  // Alphabetically first
    EXPECT_EQ(names[1], "SolarRadiation");
    EXPECT_EQ(names[2], "TwoBody");
}

/**
 * @brief Test summary generation
 */
TEST_F(ForceModelRegistryTest, GetSummaryTest) {
    auto& registry = ForceModelRegistry::getInstance();
    
    // Register models with full info
    ForceModelInfo gravityInfo;
    gravityInfo.type = ForceModelType::TwoBody;
    gravityInfo.typeName = "TwoBody";
    gravityInfo.description = "Central body gravitational force";
    gravityInfo.factory = [](const std::string& name) {
        return std::make_unique<SimpleGravityModel>(name);
    };
    gravityInfo.requiredParams = {"mu"};
    gravityInfo.optionalParams = {"central_body"};
    
    ForceModelInfo dragInfo;
    dragInfo.type = ForceModelType::Drag;
    dragInfo.typeName = "Drag";
    dragInfo.description = "Atmospheric drag force";
    dragInfo.factory = [](const std::string& name) {
        return std::make_unique<SimpleGravityModel>(name);
    };
    dragInfo.requiredParams = {"Cd", "area"};
    dragInfo.optionalParams = {"atmosphere_model"};
    
    registry.registerForceModel(ForceModelType::TwoBody, gravityInfo);
    registry.registerForceModel(ForceModelType::Drag, dragInfo);
    
    // Get summary
    std::string summary = registry.getSummary();
    
    // Verify content
    EXPECT_TRUE(summary.find("Force Model Registry Summary") != std::string::npos);
    EXPECT_TRUE(summary.find("Registered models: 2") != std::string::npos);
    EXPECT_TRUE(summary.find("TwoBody") != std::string::npos);
    EXPECT_TRUE(summary.find("Central body gravitational force") != std::string::npos);
    EXPECT_TRUE(summary.find("Required parameters: mu") != std::string::npos);
    EXPECT_TRUE(summary.find("Optional parameters: central_body") != std::string::npos);
}

/**
 * @brief Test thread safety
 */
TEST_F(ForceModelRegistryTest, ThreadSafetyTest) {
    auto& registry = ForceModelRegistry::getInstance();
    
    const int numThreads = 10;
    const int modelsPerThread = 5;
    std::vector<std::thread> threads;
    
    // Function to register models
    auto registerModels = [&](int threadId) {
        for (int i = 0; i < modelsPerThread; ++i) {
            ForceModelType type = static_cast<ForceModelType>(threadId);
            std::string typeName = "Type_" + std::to_string(threadId) + 
                                  "_" + std::to_string(i);
            
            registry.registerForceModel(
                type,
                typeName,
                "Test model",
                [](const std::string& name) {
                    return std::make_unique<SimpleGravityModel>(name);
                });
            
            // Also try to create models
            auto model = registry.createForceModel(type, 
                                                  "Model_" + std::to_string(threadId));
        }
    };
    
    // Launch threads
    for (int i = 0; i < numThreads; ++i) {
        threads.emplace_back(registerModels, i);
    }
    
    // Wait for all threads
    for (auto& thread : threads) {
        thread.join();
    }
    
    // Verify registry state (should have some models registered)
    auto types = registry.getRegisteredTypes();
    EXPECT_GT(types.size(), 0);
}

/**
 * @brief Test the ForceModelRegistrar helper
 */
TEST_F(ForceModelRegistryTest, ForceModelRegistrarTest) {
    auto& registry = ForceModelRegistry::getInstance();
    
    // Use the registrar helper
    {
        ForceModelInfo info;
        info.type = ForceModelType::Thrust;
        info.typeName = "Thrust";
        info.description = "Propulsive thrust force";
        info.factory = [](const std::string& name) {
            return std::make_unique<SimpleGravityModel>(name);
        };
        
        ForceModelRegistrar registrar(ForceModelType::Thrust, info);
    }
    
    // Verify registration
    EXPECT_TRUE(registry.isRegistered(ForceModelType::Thrust));
    EXPECT_TRUE(registry.isRegistered("Thrust"));
}

/**
 * @brief Test error handling in factory functions
 */
TEST_F(ForceModelRegistryTest, FactoryErrorHandlingTest) {
    auto& registry = ForceModelRegistry::getInstance();
    
    // Register a factory that throws
    registry.registerForceModel(
        ForceModelType::UserDefined,
        "Faulty",
        "Model with faulty factory",
        [](const std::string& name) -> std::unique_ptr<ForceModel> {
            throw std::runtime_error("Factory error");
        });
    
    // Try to create (should return nullptr, not throw)
    auto model = registry.createForceModel(ForceModelType::UserDefined, "Test");
    EXPECT_EQ(model, nullptr);
}

/**
 * @brief Test registering without factory
 */
TEST_F(ForceModelRegistryTest, NoFactoryTest) {
    auto& registry = ForceModelRegistry::getInstance();
    
    ForceModelInfo info;
    info.type = ForceModelType::Relativistic;
    info.typeName = "Relativistic";
    info.description = "Relativistic corrections";
    // info.factory is not set (nullptr)
    
    // Should fail to register
    EXPECT_FALSE(registry.registerForceModel(ForceModelType::Relativistic, info));
    EXPECT_FALSE(registry.isRegistered(ForceModelType::Relativistic));
}