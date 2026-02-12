cmake_minimum_required(VERSION 3.5)
project(kalman_filter_localization)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Header-only, ROS2-independent core target. This keeps non-ROS logic reusable
# without pulling in ROS2 deps.
add_library(kalman_filter_localization_core INTERFACE)
add_library(kalman_filter_localization::core ALIAS kalman_filter_localization_core)
set_target_properties(kalman_filter_localization_core PROPERTIES EXPORT_NAME core)
target_include_directories(kalman_filter_localization_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(kalman_filter_localization_core INTERFACE
  Eigen3::Eigen
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ekf_core
    test/test_ekf_core.cpp
  )
  # ament_add_gtest uses the plain target_link_libraries() signature internally,
  # so keep using the plain signature here too (CMake forbids mixing styles).
  target_link_libraries(test_ekf_core kalman_filter_localization::core)
endif()

add_library(ekf_localization_component SHARED
src/ekf_localization_component.cpp
)

target_compile_definitions(ekf_localization_component PRIVATE "KFL_EKFL_BUILDING_DLL")
# ament_target_dependencies uses the plain target_link_libraries() signature
# internally, so keep using the plain signature here too.
target_link_libraries(ekf_localization_component kalman_filter_localization::core)
ament_target_dependencies(ekf_localization_component
  rclcpp rclcpp_components nav_msgs sensor_msgs tf2 tf2_eigen tf2_geometry_msgs tf2_ros)

add_executable(ekf_localization_node
src/ekf_localization_node.cpp
)

target_link_libraries(ekf_localization_node
ekf_localization_component)

ament_target_dependencies(ekf_localization_node
  rclcpp rclcpp_components nav_msgs sensor_msgs tf2 tf2_eigen tf2_geometry_msgs tf2_ros)

rclcpp_components_register_nodes(ekf_localization_component
  "kalman_filter_localization::EkfLocalizationComponent")

# install executables/libs
install(
  DIRECTORY "include/"
  DESTINATION include
)

install(
  TARGETS kalman_filter_localization_core
  EXPORT export_${PROJECT_NAME}
)

install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ekf_localization_node
  DESTINATION lib/kalman_filter_localization
)

install(TARGETS
  ekf_localization_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


# export
ament_export_dependencies(rclcpp)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
