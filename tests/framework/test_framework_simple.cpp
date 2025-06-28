#include "fixtures/TestFixtureBase.h"
#include "utilities/TestUtilities.h"

using namespace iloss::test;

/**
 * @brief Simple demonstration of test framework functionality
 */
class SimpleTestDemo : public TestFixtureBase {
protected:
    void SetUp() override {
        TestFixtureBase::SetUp();
        std::cout << "Test framework is working!\n";
    }
};

TEST_F(SimpleTestDemo, BasicTestDirectoryFunctionality) {
    // Test directory is automatically created
    EXPECT_TRUE(std::filesystem::exists(getTestDir()));
    
    // Create and read test file
    writeTestFile("test.txt", "Hello from ILOSS test framework!");
    auto content = readTestFile("test.txt");
    EXPECT_EQ(content, "Hello from ILOSS test framework!");
}

TEST_F(SimpleTestDemo, RandomNumberGeneration) {
    // Test integer random
    for (int i = 0; i < 10; ++i) {
        auto val = randomUniform(1, 100);
        EXPECT_GE(val, 1);
        EXPECT_LE(val, 100);
    }
    
    // Test double random
    for (int i = 0; i < 10; ++i) {
        auto val = randomUniform(0.0, 1.0);
        EXPECT_GE(val, 0.0);
        EXPECT_LE(val, 1.0);
    }
}

TEST_F(SimpleTestDemo, TimingFunctionality) {
    startTimer("test");
    
    // Do some work
    std::vector<int> data;
    for (int i = 0; i < 1000; ++i) {
        data.push_back(i * i);
    }
    
    double elapsed = stopTimer("test");
    EXPECT_GT(elapsed, 0.0);
    std::cout << "Operation took: " << elapsed << " ms\n";
}

TEST(TestUtilitiesDemo, BasicUtilities) {
    // Test string utilities
    EXPECT_TRUE(TestUtilities::contains("Hello World", "World"));
    EXPECT_FALSE(TestUtilities::contains("Hello", "xyz"));
    
    // Test split
    auto parts = TestUtilities::split("a,b,c,d", ',');
    EXPECT_EQ(parts.size(), 4);
    EXPECT_EQ(parts[0], "a");
    EXPECT_EQ(parts[3], "d");
    
    // Test join
    std::vector<std::string> items = {"one", "two", "three"};
    auto joined = TestUtilities::join(items, " - ");
    EXPECT_EQ(joined, "one - two - three");
}

TEST(TestUtilitiesDemo, FileOperations) {
    auto tempFile = std::filesystem::temp_directory_path() / "iloss_test_file.txt";
    
    // Create file
    EXPECT_TRUE(TestUtilities::createFile(tempFile, "Test content"));
    EXPECT_TRUE(TestUtilities::fileExists(tempFile));
    
    // Read file
    auto content = TestUtilities::readFile(tempFile);
    EXPECT_EQ(content, "Test content");
    
    // Delete file
    EXPECT_TRUE(TestUtilities::deleteFile(tempFile));
    EXPECT_FALSE(TestUtilities::fileExists(tempFile));
}

TEST(TestUtilitiesDemo, ExecutionTiming) {
    auto elapsed = TestUtilities::measureExecutionTime([]() {
        // Simulate some work
        int sum = 0;
        for (int i = 0; i < 1000000; ++i) {
            sum += i;
        }
    });
    
    EXPECT_GT(elapsed, 0.0);
    std::cout << "Execution took: " << elapsed << " ms\n";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    std::cout << "\n=================================\n";
    std::cout << "ILOSS Test Framework Demo\n";
    std::cout << "=================================\n\n";
    
    return RUN_ALL_TESTS();
}