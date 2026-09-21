# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target robot_localization::robot_localization
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${robot_localization_TARGETS}.
if(robot_localization_TARGETS AND NOT TARGET robot_localization::robot_localization)
  add_library(robot_localization::robot_localization INTERFACE IMPORTED)
  set_target_properties(robot_localization::robot_localization PROPERTIES
    INTERFACE_LINK_LIBRARIES "${robot_localization_TARGETS}")
endif()
