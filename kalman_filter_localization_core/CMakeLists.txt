cmake_minimum_required(VERSION 3.5)
project(kalman_filter_localization_core)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::core ALIAS ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES EXPORT_NAME core)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE
  Eigen3::Eigen
)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ekf_core test/test_ekf_core.cpp)
  ament_add_gtest(test_eskf_replay test/test_eskf_replay.cpp)
  ament_add_gtest(test_imu_initializer test/test_imu_initializer.cpp)
  ament_add_gtest(test_odometry_core test/test_odometry_core.cpp)
  ament_add_gtest(test_vehicle_observability test/test_vehicle_observability.cpp)
  ament_add_gtest(test_sensor_fault_monitor test/test_sensor_fault_monitor.cpp)
  # ament_add_gtest uses the plain target_link_libraries() signature internally,
  # so keep using the plain signature here too (CMake forbids mixing styles).
  target_link_libraries(test_ekf_core ${PROJECT_NAME})
  target_link_libraries(test_eskf_replay ${PROJECT_NAME})
  target_link_libraries(test_imu_initializer ${PROJECT_NAME})
  target_link_libraries(test_odometry_core ${PROJECT_NAME})
  target_link_libraries(test_vehicle_observability ${PROJECT_NAME})
  target_link_libraries(test_sensor_fault_monitor ${PROJECT_NAME})
endif()

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
