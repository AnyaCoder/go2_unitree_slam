cmake_minimum_required(VERSION 3.5)
project(nav_dog_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(cloud_to_map_filter src/cloud_to_map_filter.cpp)
target_include_directories(cloud_to_map_filter PRIVATE
  include ${EIGEN3_INCLUDE_DIR})
target_link_libraries(cloud_to_map_filter Eigen3::Eigen)
ament_target_dependencies(cloud_to_map_filter
  nav_msgs rclcpp sensor_msgs)
target_compile_options(cloud_to_map_filter PRIVATE
  -O3 -DNDEBUG -fno-math-errno -funroll-loops -Wall -Wextra -Wpedantic)

install(TARGETS cloud_to_map_filter
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cloud_filter_core test/test_cloud_filter_core.cpp)
  target_include_directories(test_cloud_filter_core PRIVATE include ${EIGEN3_INCLUDE_DIR})
  target_link_libraries(test_cloud_filter_core Eigen3::Eigen)
  target_compile_options(test_cloud_filter_core PRIVATE -O2 -Wall -Wextra -Wpedantic)
endif()

ament_package()
