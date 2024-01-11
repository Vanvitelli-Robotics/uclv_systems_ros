cmake_minimum_required(VERSION 3.8)
project(uclv_systems_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uclv_eigen_ros_conversions REQUIRED)
find_package(uclv_systems_lib REQUIRED)
find_package (Eigen3 REQUIRED NO_MODULE)

include_directories(include)

############### Add filter_geometry_msgs_component nodes as a components ###############
add_library(filter_geometry_msgs_component  SHARED src/${PROJECT_NAME}/filter_geometry_msgs_component.cpp)
ament_target_dependencies(filter_geometry_msgs_component
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "uclv_eigen_ros_conversions"
  "uclv_systems_lib"
)
target_link_libraries(filter_geometry_msgs_component Eigen3::Eigen)
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
ament_target_dependencies(sum_geometry_msgs_component
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
)
# target_link_libraries(sum_geometry_msgs_component Eigen3::Eigen)
rclcpp_components_register_node(sum_geometry_msgs_component 
PLUGIN "uclv_systems_ros::SumTwistNode"
EXECUTABLE sum_twist
)
rclcpp_components_register_node(sum_geometry_msgs_component 
PLUGIN "uclv_systems_ros::SumTwistStampedNode"
EXECUTABLE sum_twist_stamped
)

# install
install(
  TARGETS
  filter_geometry_msgs_component
  sum_geometry_msgs_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
