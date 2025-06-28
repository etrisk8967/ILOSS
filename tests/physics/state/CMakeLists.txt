# Physics State Tests

# StateVector tests
add_executable(test_StateVector test_StateVector.cpp)
target_link_libraries(test_StateVector
    PRIVATE
    iloss_physics_state
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME StateVectorTest COMMAND test_StateVector)

# StateHistory tests
add_executable(test_StateHistory test_StateHistory.cpp)
target_link_libraries(test_StateHistory
    PRIVATE
    iloss_physics_state
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME StateHistoryTest COMMAND test_StateHistory)