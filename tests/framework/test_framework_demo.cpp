#include "fixtures/TestFixtureBase.h"
#include "generators/TestDataGenerator.h"
#include "mocks/MockLogger.h"
#include "mocks/MockEventListener.h"  
#include "mocks/MockPlugin.h"
#include "utilities/TestUtilities.h"
#include "benchmarks/BenchmarkBase.h"

using namespace iloss::test;

/**
 * @brief Demonstrate test fixture base functionality
 */
class TestFrameworkDemo : public TestFixtureBase {
protected:
    void SetUp() override {
        TestFixtureBase::SetUp();
        setVerboseLogging(true);
    }
};

TEST_F(TestFrameworkDemo, TestDirectoryManagement) {
    // Test directory is automatically created
    EXPECT_TRUE(std::filesystem::exists(getTestDir()));
    
    // Create test file
    writeTestFile("test_data.txt", "Hello, ILOSS!");
    EXPECT_TRUE(std::filesystem::exists(getTestDataPath("test_data.txt")));
    
    // Read test file
    auto content = readTestFile("test_data.txt");
    EXPECT_EQ(content, "Hello, ILOSS!");
}

TEST_F(TestFrameworkDemo, RandomNumberGeneration) {
    // Generate random values
    auto intVal = randomUniform(1, 100);
    EXPECT_GE(intVal, 1);
    EXPECT_LE(intVal, 100);
    
    auto doubleVal = randomUniform(0.0, 1.0);
    EXPECT_GE(doubleVal, 0.0);
    EXPECT_LE(doubleVal, 1.0);
    
    // Normal distribution
    auto normalVal = randomNormal(0.0, 1.0);
    EXPECT_NE(normalVal, 0.0); // Very unlikely to be exactly 0
}

TEST_F(TestFrameworkDemo, PerformanceTiming) {
    startTimer("test_operation");
    
    // Simulate some work
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    double elapsed = stopTimer("test_operation");
    EXPECT_GE(elapsed, 10.0);
    EXPECT_LE(elapsed, 20.0); // Allow some overhead
}

/**
 * @brief Demonstrate physics test fixture
 */
class PhysicsFrameworkDemo : public PhysicsTestFixture {
protected:
    void SetUp() override {
        PhysicsTestFixture::SetUp();
    }
};

TEST_F(PhysicsFrameworkDemo, PhysicsComparisons) {
    // Test position comparison
    expectPositionNear(1.000000001, 1.0);
    
    // Test velocity comparison
    expectVelocityNear(100.0 + 1e-13, 100.0);
    
    // Test angle comparison with wrapping
    expectAngleNear(M_PI, -M_PI); // Should be equal (wrapped)
    expectAngleNear(0.0, 2 * M_PI); // Should be equal (wrapped)
}

TEST_F(PhysicsFrameworkDemo, OrbitGeneration) {
    auto orbit = generateRandomOrbit();
    
    EXPECT_GE(orbit.semiMajorAxis, 6500000.0);
    EXPECT_LE(orbit.semiMajorAxis, 50000000.0);
    EXPECT_GE(orbit.eccentricity, 0.0);
    EXPECT_LT(orbit.eccentricity, 1.0);
    EXPECT_GE(orbit.inclination, 0.0);
    EXPECT_LE(orbit.inclination, M_PI);
}

/**
 * @brief Demonstrate test data generator
 */
TEST(TestDataGeneratorDemo, VectorGeneration) {
    TestDataGenerator gen(42); // Fixed seed for reproducibility
    
    // Generate random vectors
    auto vec = gen.randomVector(-10.0, 10.0);
    EXPECT_GE(vec.x(), -10.0);
    EXPECT_LE(vec.x(), 10.0);
    
    // Generate unit vector
    auto unitVec = gen.randomUnitVector();
    EXPECT_NEAR(unitVec.magnitude(), 1.0, 1e-9);
    
    // Generate position (Earth orbit)
    auto pos = gen.randomPosition();
    EXPECT_GE(pos.magnitude(), 6378137.0); // >= Earth radius
    
    // Generate velocity
    auto vel = gen.randomVelocity();
    EXPECT_GE(vel.magnitude(), 1000.0);
    EXPECT_LE(vel.magnitude(), 11000.0);
}

TEST(TestDataGeneratorDemo, TimeGeneration) {
    TestDataGenerator gen;
    
    // Generate random time in 2025
    auto time2025 = gen.randomTimeInYear(2025);
    EXPECT_EQ(time2025.getYear(), 2025);
    
    // Generate time sequence
    time::Time start(2025, 1, 1, 0, 0, 0.0);
    auto sequence = gen.randomTimeSequence(10, start, 10.0, 60.0);
    
    EXPECT_EQ(sequence.size(), 10);
    for (size_t i = 1; i < sequence.size(); ++i) {
        double delta = sequence[i].getTime() - sequence[i-1].getTime();
        EXPECT_GE(delta, 10.0);
        EXPECT_LE(delta, 60.0);
    }
}

TEST(TestDataGeneratorDemo, ConfigGeneration) {
    TestDataGenerator gen;
    
    auto config = gen.randomConfig(2, 3);
    EXPECT_TRUE(config.is_object());
    EXPECT_GT(config.size(), 0);
}

/**
 * @brief Demonstrate mock logger
 */
TEST(MockLoggerDemo, LogCapture) {
    auto mockLogger = std::make_shared<MockLogger>();
    
    // Simulate logging
    mockLogger->log(logging::LogLevel::Info, "Test", "Test message", 
                   std::chrono::system_clock::now(), __FILE__, __LINE__, __func__);
    
    EXPECT_EQ(mockLogger->getEntryCount(), 1);
    EXPECT_TRUE(mockLogger->hasMessage("Test message"));
    EXPECT_TRUE(mockLogger->hasCategory("Test"));
    
    auto lastEntry = mockLogger->getLastEntry();
    ASSERT_NE(lastEntry, nullptr);
    EXPECT_EQ(lastEntry->level, logging::LogLevel::Info);
}

/**
 * @brief Demonstrate mock event listener
 */
TEST(MockEventListenerDemo, EventHandling) {
    MockEventListener listener;
    
    // Create test event
    class TestEvent : public events::Event {
    public:
        TestEvent() : Event(events::EventCategory::System, 
                           events::EventPriority::Normal) {}
        std::string getType() const override { return "TestEvent"; }
        std::unique_ptr<Event> clone() const override {
            return std::make_unique<TestEvent>(*this);
        }
    };
    
    TestEvent event;
    listener.onEvent(event);
    
    EXPECT_EQ(listener.getEventCount(), 1);
    EXPECT_TRUE(listener.hasReceivedEvent("TestEvent"));
    listener.expectEventType("TestEvent");
    listener.expectLastEventIs<TestEvent>();
}

/**
 * @brief Demonstrate mock plugin
 */
TEST(MockPluginDemo, PluginLifecycle) {
    MockPlugin plugin("TestPlugin", "1.0.0");
    
    // Test initialization
    nlohmann::json config = {{"setting", "value"}};
    EXPECT_TRUE(plugin.initialize(config));
    EXPECT_EQ(plugin.getInitializeCount(), 1);
    EXPECT_EQ(plugin.getLastConfig(), config);
    
    // Test activation
    EXPECT_TRUE(plugin.activate());
    EXPECT_TRUE(plugin.isActive());
    EXPECT_EQ(plugin.getActivateCount(), 1);
    
    // Test deactivation
    plugin.deactivate();
    EXPECT_FALSE(plugin.isActive());
    EXPECT_EQ(plugin.getDeactivateCount(), 1);
    
    // Test shutdown
    plugin.shutdown();
    EXPECT_EQ(plugin.getShutdownCount(), 1);
}

/**
 * @brief Demonstrate test utilities
 */
TEST(TestUtilitiesDemo, FileOperations) {
    auto tempDir = std::filesystem::temp_directory_path() / "iloss_test_utils";
    TestUtilities::createDirectory(tempDir);
    
    auto testFile = tempDir / "test.txt";
    EXPECT_TRUE(TestUtilities::createFile(testFile, "Test content"));
    EXPECT_TRUE(TestUtilities::fileExists(testFile));
    
    auto content = TestUtilities::readFile(testFile);
    EXPECT_EQ(content, "Test content");
    
    EXPECT_TRUE(TestUtilities::deleteFile(testFile));
    std::filesystem::remove(tempDir);
}

TEST(TestUtilitiesDemo, TimingUtilities) {
    auto elapsed = TestUtilities::measureExecutionTime([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    });
    
    EXPECT_GE(elapsed, 10.0);
    EXPECT_LE(elapsed, 20.0);
}

TEST(TestUtilitiesDemo, StringUtilities) {
    EXPECT_TRUE(TestUtilities::contains("Hello, World!", "World"));
    EXPECT_FALSE(TestUtilities::contains("Hello", "World"));
    
    auto tokens = TestUtilities::split("a,b,c", ',');
    EXPECT_EQ(tokens.size(), 3);
    EXPECT_EQ(tokens[0], "a");
    EXPECT_EQ(tokens[1], "b");
    EXPECT_EQ(tokens[2], "c");
    
    auto joined = TestUtilities::join(tokens, "|");
    EXPECT_EQ(joined, "a|b|c");
}

/**
 * @brief Demonstrate benchmark framework
 */
class BenchmarkDemo : public BenchmarkBase {
protected:
    void SetUp() override {
        BenchmarkBase::SetUp();
        
        // Configure benchmark
        Config config;
        config.warmupIterations = 5;
        config.minIterations = 100;
        config.verbose = true;
        setConfig(config);
    }
};

BENCHMARK_F(BenchmarkDemo, VectorOperations) {
    TestDataGenerator gen;
    
    // Benchmark vector addition
    auto v1 = gen.randomVector();
    auto v2 = gen.randomVector();
    
    BENCHMARK_TIME("Vector Addition", {
        auto result = v1 + v2;
        (void)result; // Prevent optimization
    });
    
    // Benchmark vector normalization
    BENCHMARK_TIME("Vector Normalization", {
        auto result = v1.normalized();
        (void)result;
    });
    
    // Compare implementations
    compareBenchmarks(
        "Direct Magnitude", [&]() { 
            double mag = std::sqrt(v1.x() * v1.x() + v1.y() * v1.y() + v1.z() * v1.z());
            (void)mag;
        },
        "Method Magnitude", [&]() {
            double mag = v1.magnitude();
            (void)mag;
        }
    );
}

BENCHMARK_F(BenchmarkDemo, ScalingBehavior) {
    TestDataGenerator gen;
    
    // Benchmark with different sizes
    std::vector<size_t> sizes = {10, 100, 1000, 10000};
    
    benchmarkScaling("Vector Array Sum", [&](size_t size) {
        auto vectors = gen.randomVectorArray(size);
        math::Vector3D sum(0, 0, 0);
        for (const auto& v : vectors) {
            sum = sum + v;
        }
    }, sizes);
}