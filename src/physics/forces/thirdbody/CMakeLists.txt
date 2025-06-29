# Third-body perturbation force model library
add_library(iloss_thirdbody STATIC
    ThirdBodyForceModel.cpp
)

target_include_directories(iloss_thirdbody PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(iloss_thirdbody PUBLIC
    iloss_physics_state
    iloss_math
    iloss_time
    iloss_logging
)

# Link SPICE if available
if(SPICE_FOUND)
    target_link_libraries(iloss_thirdbody PUBLIC ${SPICE_LIBRARIES} m)
    target_include_directories(iloss_thirdbody PUBLIC ${SPICE_INCLUDE_DIR})
    target_compile_definitions(iloss_thirdbody PUBLIC SPICE_FOUND)
endif()

# Set C++ standard
target_compile_features(iloss_thirdbody PUBLIC cxx_std_20)

# Add to physics forces group
set_target_properties(iloss_thirdbody PROPERTIES FOLDER "Physics/Forces")