cmake_minimum_required(VERSION 3.8)
project(m8_vehicle_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# for the m8 interface:
find_package(rclcpp REQUIRED)
find_package(zgw_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(tier4_control_msgs REQUIRED)
find_package(tier4_vehicle_msgs REQUIRED)
# ADD MORE PACKAGE DEPENDENCIES HERE

include_directories(include/${PROJECT_NAME})

set(INTERFACE_EXEC_NAME ${PROJECT_NAME})
add_executable(${INTERFACE_EXEC_NAME} src/m8_vehicle_interface.cpp src/interface_helpers.cpp)
ament_target_dependencies(${INTERFACE_EXEC_NAME} 
  rclcpp 
  zgw_interfaces 
  geometry_msgs 
  autoware_control_msgs
  autoware_vehicle_msgs
  tier4_control_msgs
  tier4_vehicle_msgs
  # add the dependencies also here
)

install(TARGETS
  ${INTERFACE_EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

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
