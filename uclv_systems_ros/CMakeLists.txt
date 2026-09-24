cmake_minimum_required(VERSION 3.20)
project(uclv_systems_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uclv_eigen_ros_conversions REQUIRED)
find_package(uclv_systems_interfaces REQUIRED)
find_package(uclv_systems_lib REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${uclv_eigen_ros_conversions_INCLUDE_DIRS}
    ${uclv_systems_lib_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  INTERFACE
    rclcpp::rclcpp
    rclcpp_components::component
    std_srvs::std_srvs
    geometry_msgs::geometry_msgs
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
    uclv_systems_interfaces::uclv_systems_interfaces
)

############### Add filter_geometry_msgs_component nodes as a components ###############
add_library(filter_geometry_msgs_component  SHARED src/${PROJECT_NAME}/filter_geometry_msgs_component.cpp)
target_include_directories(filter_geometry_msgs_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${uclv_eigen_ros_conversions_INCLUDE_DIRS}
    ${uclv_systems_lib_INCLUDE_DIRS}
)
target_link_libraries(filter_geometry_msgs_component
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    geometry_msgs::geometry_msgs
    Eigen3::Eigen
)
rclcpp_components_register_node(filter_geometry_msgs_component 
PLUGIN "uclv_systems_ros::FilterTwistNode"
EXECUTABLE filter_twist
)
rclcpp_components_register_node(filter_geometry_msgs_component 
PLUGIN "uclv_systems_ros::FilterTwistStampedNode"
EXECUTABLE filter_twist_stamped
)
rclcpp_components_register_node(filter_geometry_msgs_component 
PLUGIN "uclv_systems_ros::FilterWrenchNode"
EXECUTABLE filter_wrench
)
rclcpp_components_register_node(filter_geometry_msgs_component 
PLUGIN "uclv_systems_ros::FilterWrenchStampedNode"
EXECUTABLE filter_wrench_stamped
)

############### Add sum_geometry_msgs_component nodes as a components ###############
add_library(sum_geometry_msgs_component  SHARED src/${PROJECT_NAME}/sum_geometry_msgs_component.cpp)
target_include_directories(sum_geometry_msgs_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(sum_geometry_msgs_component
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    geometry_msgs::geometry_msgs
)
rclcpp_components_register_node(sum_geometry_msgs_component 
PLUGIN "uclv_systems_ros::SumTwistNode"
EXECUTABLE sum_twist
)
rclcpp_components_register_node(sum_geometry_msgs_component 
PLUGIN "uclv_systems_ros::SumTwistStampedNode"
EXECUTABLE sum_twist_stamped
)

############### Add twist_stamped_sine_publisher as a component ###############
add_library(twist_stamped_sine_publisher_component SHARED src/${PROJECT_NAME}/twist_stamped_sine_publisher.cpp)
target_include_directories(twist_stamped_sine_publisher_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(twist_stamped_sine_publisher_component
  PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    geometry_msgs::geometry_msgs
    yaml-cpp::yaml-cpp
)
rclcpp_components_register_node(twist_stamped_sine_publisher_component
  PLUGIN "uclv_systems_ros::TwistStampedSinePublisher"
  EXECUTABLE twist_stamped_sine_publisher
)

# install
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_srvs
  geometry_msgs
  uclv_eigen_ros_conversions
  uclv_systems_interfaces
  uclv_systems_lib
  Eigen3
  yaml-cpp
)

install(
  TARGETS
  ${PROJECT_NAME}
  filter_geometry_msgs_component
  sum_geometry_msgs_component
  twist_stamped_sine_publisher_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  DIRECTORY include/
  DESTINATION include)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(
    test_measurement_covariance_filter
    test/test_measurement_covariance_filter.cpp)
  target_link_libraries(
    test_measurement_covariance_filter
    ${PROJECT_NAME})
endif()

ament_package()
