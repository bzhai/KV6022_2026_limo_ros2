# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target limo_msgs::limo_msgs
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${limo_msgs_TARGETS}.
if(limo_msgs_TARGETS AND NOT TARGET limo_msgs::limo_msgs)
  add_library(limo_msgs::limo_msgs INTERFACE IMPORTED)
  set_target_properties(limo_msgs::limo_msgs PROPERTIES
    INTERFACE_LINK_LIBRARIES "${limo_msgs_TARGETS}")
endif()
