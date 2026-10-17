cmake_minimum_required(VERSION 3.8)
project(fast_calib)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if (CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(livox_ros_driver2 QUIET)

find_package(PCL 1.10 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(FAST_CALIB_ROS_DEPS rclcpp sensor_msgs pcl_conversions rosbag2_cpp rosbag2_storage)
if(livox_ros_driver2_FOUND)
  message(STATUS "fast_calib: livox_ros_driver2 found, CustomMsg bags supported")
  list(APPEND FAST_CALIB_ROS_DEPS livox_ros_driver2)
endif()

function(fast_calib_target target)
  ament_target_dependencies(${target} ${FAST_CALIB_ROS_DEPS})
  target_link_libraries(${target} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
  if(livox_ros_driver2_FOUND)
    target_compile_definitions(${target} PRIVATE FAST_CALIB_HAVE_LIVOX)
  endif()
endfunction()

add_executable(fast_calib src/main.cpp)
fast_calib_target(fast_calib)

add_executable(lidar_center_test test/lidar_center_test.cpp)
target_include_directories(lidar_center_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
fast_calib_target(lidar_center_test)

add_executable(multi_fast_calib src/multi_scene.cpp)
fast_calib_target(multi_fast_calib)

install(TARGETS fast_calib lidar_center_test multi_fast_calib
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz_cfg
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/capture_scene.py scripts/distance_filter_tool.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(common_lib_test test/common_lib_test.cpp)
  if(TARGET common_lib_test)
    fast_calib_target(common_lib_test)
  endif()
endif()

ament_package()
