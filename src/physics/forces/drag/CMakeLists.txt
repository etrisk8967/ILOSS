# Drag force model library
add_library(iloss_drag_force
    DragForceModel.cpp
    NRLMSISE00Atmosphere.cpp
)

target_include_directories(iloss_drag_force PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(iloss_drag_force PUBLIC
    iloss_force_model
    iloss_physics_state
    iloss_math
    iloss_time
    iloss_constants
    iloss_coordinates
    iloss_logging
)

# Set compile features
target_compile_features(iloss_drag_force PUBLIC cxx_std_20)

# Export the library
set_target_properties(iloss_drag_force PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)