cmake_minimum_required(VERSION 3.16)
project(witmotion_imu_driver)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(witmotion_imu_node src/witmotion_imu_node.cpp)
target_compile_features(witmotion_imu_node PRIVATE cxx_std_17)
target_compile_options(witmotion_imu_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(witmotion_imu_node rclcpp sensor_msgs std_srvs tf2)
install(TARGETS witmotion_imu_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(NAME serial_driver_check
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/check_driver.py
      $<TARGET_FILE:witmotion_imu_node>)
  set_tests_properties(serial_driver_check PROPERTIES TIMEOUT 35)
endif()

ament_package()
