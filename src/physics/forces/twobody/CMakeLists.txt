# Two-body dynamics library
add_library(iloss_twobody STATIC
    TwoBodyForceModel.cpp
    KeplerPropagator.cpp
    ConicSectionUtilities.cpp
    TwoBodyAnalyticalPropagator.cpp
)

target_include_directories(iloss_twobody PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(iloss_twobody PUBLIC
    iloss_physics_state
    iloss_math
    iloss_time
    iloss_logging
    iloss_coordinates
    iloss_constants
)

# Set C++ standard
target_compile_features(iloss_twobody PUBLIC cxx_std_20)

# Add to physics forces group
set_target_properties(iloss_twobody PROPERTIES FOLDER "Physics/Forces")