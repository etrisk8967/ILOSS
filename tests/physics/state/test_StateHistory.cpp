#include <gtest/gtest.h>
#include "physics/state/StateHistory.h"
#include "core/math/MathConstants.h"
#include <cmath>
#include <thread>
#include <chrono>

using namespace iloss::math;
using namespace iloss::time;
using iloss::coordinates::CoordinateSystemType;

class StateHistoryTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a series of test states representing a circular orbit
        double radius = 7000000.0;  // 7000 km
        double speed = std::sqrt(constants::EARTH_MU / radius);
        double period = 2.0 * constants::PI * radius / speed;
        double dt = period / 100.0;  // 100 points per orbit
        
        for (int i = 0; i < 10; ++i) {
            double t = i * dt;
            double angle = 2.0 * constants::PI * t / period;
            
            Vector3D pos(radius * std::cos(angle), radius * std::sin(angle), 0.0);
            Vector3D vel(-speed * std::sin(angle), speed * std::cos(angle), 0.0);
            
            testStates.emplace_back(pos, vel, 1000.0 - i * 0.1, Time(t));
        }
    }

    std::vector<iloss::physics::StateVector> testStates;
};

// Test construction and basic operations
TEST_F(StateHistoryTest, ConstructionAndBasics) {
    iloss::physics::StateHistory history;
    EXPECT_TRUE(history.empty());
    EXPECT_EQ(history.size(), 0);
    EXPECT_EQ(history.getMaxSize(), 0);  // Unlimited by default
    
    iloss::physics::StateHistory limitedHistory(5);
    EXPECT_EQ(limitedHistory.getMaxSize(), 5);
}

// Test adding states
TEST_F(StateHistoryTest, AddStates) {
    iloss::physics::StateHistory history;
    
    // Add single state
    history.addState(testStates[0]);
    EXPECT_EQ(history.size(), 1);
    EXPECT_FALSE(history.empty());
    
    // Add multiple states
    history.addStates(testStates);
    EXPECT_EQ(history.size(), testStates.size() + 1);  // +1 for duplicate
    
    // Test adding invalid state
    iloss::physics::StateVector invalidState;
    EXPECT_THROW(history.addState(invalidState), std::invalid_argument);
}

// Test max size enforcement
TEST_F(StateHistoryTest, MaxSizeEnforcement) {
    iloss::physics::StateHistory history(5);
    
    // Add more states than max size
    history.addStates(testStates);
    EXPECT_EQ(history.size(), 5);
    
    // Verify oldest states were removed
    auto earliest = history.getEarliestState();
    EXPECT_EQ(earliest.getTime(), testStates[5].getTime());
    
    // Change max size
    history.setMaxSize(3);
    EXPECT_EQ(history.size(), 3);
    
    // Set unlimited
    history.setMaxSize(0);
    history.addStates(testStates);
    EXPECT_GT(history.size(), 5);
}

// Test state retrieval
TEST_F(StateHistoryTest, StateRetrieval) {
    iloss::physics::StateHistory history;
    history.addStates(testStates);
    
    // Get state at exact time
    auto state = history.getStateAtTime(testStates[3].getTime());
    EXPECT_TRUE(state.has_value());
    EXPECT_EQ(state.value(), testStates[3]);
    
    // Get state at non-existent time
    state = history.getStateAtTime(Time(999999.0));
    EXPECT_FALSE(state.has_value());
    
    // Get earliest and latest states
    EXPECT_EQ(history.getEarliestState(), testStates[0]);
    EXPECT_EQ(history.getLatestState(), testStates.back());
    
    // Test on empty history
    iloss::physics::StateHistory emptyHistory;
    EXPECT_THROW(emptyHistory.getEarliestState(), std::runtime_error);
    EXPECT_THROW(emptyHistory.getLatestState(), std::runtime_error);
}

// Test nearest state
TEST_F(StateHistoryTest, NearestState) {
    iloss::physics::StateHistory history;
    history.addStates(testStates);
    
    // Get nearest to exact time
    auto nearest = history.getNearestState(testStates[3].getTime());
    EXPECT_EQ(nearest, testStates[3]);
    
    // Get nearest between two states
    double t1 = testStates[3].getTime().getTime();
    double t2 = testStates[4].getTime().getTime();
    Time midTime((t1 + t2) / 2.0);
    
    nearest = history.getNearestState(midTime);
    // Should return either state 3 or 4
    EXPECT_TRUE(nearest == testStates[3] || nearest == testStates[4]);
    
    // Test before earliest
    nearest = history.getNearestState(Time(-1000.0));
    EXPECT_EQ(nearest, testStates[0]);
    
    // Test after latest
    nearest = history.getNearestState(Time(1e10));
    EXPECT_EQ(nearest, testStates.back());
}

// Test linear interpolation
TEST_F(StateHistoryTest, LinearInterpolation) {
    iloss::physics::StateHistory history;
    history.addStates(testStates);
    
    // Interpolate between two states
    double t1 = testStates[3].getTime().getTime();
    double t2 = testStates[4].getTime().getTime();
    Time interpTime(t1 + 0.3 * (t2 - t1));
    
    iloss::physics::StateVector interp = history.getInterpolatedState(interpTime, "linear");
    
    // Check that interpolated state has reasonable radius
    double r1 = testStates[3].getRadius();
    double r2 = testStates[4].getRadius();
    double rInterp = interp.getRadius();
    // For circular orbit, interpolated radius should be close to original
    EXPECT_NEAR(rInterp, (r1 + r2) / 2.0, 10000.0);  // Within 10km
    
    // Test exact time returns exact state
    interp = history.getInterpolatedState(testStates[3].getTime(), "linear");
    EXPECT_EQ(interp, testStates[3]);
    
    // Test out of bounds
    EXPECT_THROW(history.getInterpolatedState(Time(-1000.0), "linear"), 
                 std::runtime_error);
    EXPECT_THROW(history.getInterpolatedState(Time(1e10), "linear"), 
                 std::runtime_error);
}

// Test cubic interpolation
TEST_F(StateHistoryTest, CubicInterpolation) {
    iloss::physics::StateHistory history;
    history.addStates(testStates);
    
    // Need at least 4 points for cubic
    if (testStates.size() >= 4) {
        double t1 = testStates[2].getTime().getTime();
        double t2 = testStates[3].getTime().getTime();
        Time interpTime(t1 + 0.5 * (t2 - t1));
        
        iloss::physics::StateVector cubic = history.getInterpolatedState(interpTime, "cubic");
        iloss::physics::StateVector linear = history.getInterpolatedState(interpTime, "linear");
        
        // Cubic and linear should be different but close
        double posDiff = (cubic.getPosition() - linear.getPosition()).magnitude();
        EXPECT_GT(posDiff, 0.0);
        EXPECT_LT(posDiff, 10000.0);  // Within 10km difference
    }
    
    // Test with insufficient points (should fall back to linear)
    iloss::physics::StateHistory smallHistory;
    smallHistory.addState(testStates[0]);
    smallHistory.addState(testStates[1]);
    
    Time midTime((testStates[0].getTime().getTime() + 
                  testStates[1].getTime().getTime()) / 2.0);
    EXPECT_NO_THROW(smallHistory.getInterpolatedState(midTime, "cubic"));
}

// Test states in range
TEST_F(StateHistoryTest, StatesInRange) {
    iloss::physics::StateHistory history;
    history.addStates(testStates);
    
    // Get subset of states
    Time startTime = testStates[2].getTime();
    Time endTime = testStates[6].getTime();
    
    auto rangeStates = history.getStatesInRange(startTime, endTime);
    EXPECT_EQ(rangeStates.size(), 5);  // States 2, 3, 4, 5, 6
    EXPECT_EQ(rangeStates.front(), testStates[2]);
    EXPECT_EQ(rangeStates.back(), testStates[6]);
    
    // Empty range
    rangeStates = history.getStatesInRange(Time(1000.0), Time(2000.0));
    EXPECT_TRUE(rangeStates.empty());
}

// Test state removal
TEST_F(StateHistoryTest, StateRemoval) {
    iloss::physics::StateHistory history;
    history.addStates(testStates);
    size_t originalSize = history.size();
    
    // Remove states before cutoff
    Time cutoff = testStates[3].getTime();
    size_t removed = history.removeStatesBefore(cutoff);
    EXPECT_EQ(removed, 3);  // States 0, 1, 2
    EXPECT_EQ(history.size(), originalSize - 3);
    EXPECT_EQ(history.getEarliestState(), testStates[3]);
    
    // Remove states after cutoff
    cutoff = testStates[6].getTime();
    removed = history.removeStatesAfter(cutoff);
    EXPECT_GT(removed, 0);
    EXPECT_EQ(history.getLatestState().getTime(), testStates[6].getTime());
    
    // Clear all
    history.clear();
    EXPECT_TRUE(history.empty());
}

// Test statistics
TEST_F(StateHistoryTest, Statistics) {
    iloss::physics::StateHistory history;
    history.addStates(testStates);
    
    auto stats = history.computeStatistics();
    
    EXPECT_EQ(stats["count"], static_cast<double>(history.size()));
    EXPECT_GT(stats["timespan_seconds"], 0.0);
    EXPECT_GT(stats["min_altitude_m"], 0.0);
    EXPECT_GT(stats["max_altitude_m"], 0.0);
    EXPECT_GT(stats["mean_altitude_m"], 0.0);
    EXPECT_GT(stats["min_speed_m/s"], 0.0);
    EXPECT_GT(stats["max_speed_m/s"], 0.0);
    EXPECT_LT(stats["mass_change_kg"], 0.0);  // Mass decreases in test data
    
    // Empty history statistics
    iloss::physics::StateHistory emptyHistory;
    stats = emptyHistory.computeStatistics();
    EXPECT_TRUE(stats.empty());
}

// Test resampling
TEST_F(StateHistoryTest, Resampling) {
    iloss::physics::StateHistory history;
    history.addStates(testStates);
    
    double originalTimespan = history.getTimeSpan();
    double dt = originalTimespan / 20.0;  // Resample to 20 points
    
    iloss::physics::StateHistory resampled = history.resample(dt, "linear");
    
    // Should have approximately 20 points
    EXPECT_GE(resampled.size(), 18);
    EXPECT_LE(resampled.size(), 22);
    
    // Time span should be similar
    EXPECT_NEAR(resampled.getTimeSpan(), originalTimespan, dt);
}

// Test discontinuity detection
TEST_F(StateHistoryTest, DiscontinuityDetection) {
    iloss::physics::StateHistory history;
    
    // Add normal states
    for (int i = 0; i < 5; ++i) {
        history.addState(testStates[i]);
    }
    
    // Add a discontinuous state (large position jump)
    Vector3D jumpPos = testStates[5].getPosition() + Vector3D(10000.0, 0.0, 0.0);
    iloss::physics::StateVector jumpState(jumpPos, testStates[5].getVelocity(), 
                         testStates[5].getMass(), testStates[5].getTime());
    history.addState(jumpState);
    
    // Add more normal states
    for (int i = 6; i < testStates.size(); ++i) {
        history.addState(testStates[i]);
    }
    
    // Detect discontinuities
    auto discontinuities = history.findDiscontinuities(5000.0, 100.0);
    EXPECT_GE(discontinuities.size(), 1);
    
    // The discontinuity should be at the jump time
    bool foundJump = false;
    for (const auto& discTime : discontinuities) {
        if (std::abs(discTime.getTime() - jumpState.getTime().getTime()) < 1e-9) {
            foundJump = true;
            break;
        }
    }
    EXPECT_TRUE(foundJump);
}

// Test getAllStates
TEST_F(StateHistoryTest, GetAllStates) {
    iloss::physics::StateHistory history;
    history.addStates(testStates);
    
    auto allStates = history.getAllStates();
    EXPECT_EQ(allStates.size(), history.size());
    
    // Verify chronological order
    for (size_t i = 1; i < allStates.size(); ++i) {
        EXPECT_LE(allStates[i-1].getTime().getTime(), 
                  allStates[i].getTime().getTime());
    }
}

// Test edge cases
TEST_F(StateHistoryTest, EdgeCases) {
    iloss::physics::StateHistory history;
    
    // Test interpolation with unknown method
    history.addStates(testStates);
    // Use a time between states to force interpolation
    double t1 = testStates[1].getTime().getTime();
    double t2 = testStates[2].getTime().getTime();
    Time betweenTime((t1 + t2) / 2.0);
    EXPECT_THROW(history.getInterpolatedState(betweenTime, "unknown"),
                 std::invalid_argument);
    
    // Test time span on single state
    iloss::physics::StateHistory singleState;
    singleState.addState(testStates[0]);
    EXPECT_EQ(singleState.getTimeSpan(), 0.0);
    
    // Test adding states with same time (multimap should handle this)
    iloss::physics::StateVector dupState = testStates[0];
    dupState.setPosition(testStates[0].getPosition() + Vector3D(1.0, 0.0, 0.0));
    history.addState(dupState);
    // Should not throw, multimap allows duplicate keys
}

// Test copy and move operations
TEST_F(StateHistoryTest, CopyAndMove) {
    iloss::physics::StateHistory history(5);
    history.addStates(testStates);
    
    // Copy constructor
    iloss::physics::StateHistory copied(history);
    EXPECT_EQ(copied.size(), history.size());
    EXPECT_EQ(copied.getMaxSize(), history.getMaxSize());
    
    // Copy assignment
    iloss::physics::StateHistory assigned;
    assigned = history;
    EXPECT_EQ(assigned.size(), history.size());
    
    // Move constructor
    size_t originalSize = history.size();
    iloss::physics::StateHistory moved(std::move(history));
    EXPECT_EQ(moved.size(), originalSize);
    
    // Move assignment
    iloss::physics::StateHistory moveAssigned;
    moveAssigned = std::move(moved);
    EXPECT_EQ(moveAssigned.size(), originalSize);
}