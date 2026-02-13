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
find_package(kalman_filter_localization_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# NOTE: Eigen is pulled in transitively via kalman_filter_localization_core.
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(ekf_localization_component SHARED
src/ekf_localization_component.cpp
)

target_compile_definitions(ekf_localization_component PRIVATE "KFL_EKFL_BUILDING_DLL")
target_include_directories(ekf_localization_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Exported library: use keyword signature so transitive deps propagate.
target_link_libraries(ekf_localization_component PUBLIC kalman_filter_localization_core::core)
ament_target_dependencies(ekf_localization_component PUBLIC
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
  DIRECTORY launch param rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ekf_localization_node
  DESTINATION lib/kalman_filter_localization
)

install(TARGETS
  ekf_localization_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


# export
ament_export_dependencies(kalman_filter_localization_core)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_eigen)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
