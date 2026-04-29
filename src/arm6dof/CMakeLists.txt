cmake_minimum_required(VERSION 3.8)
project(arm6dof)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MITFeedback.msg"
  "msg/MITFeedbackArray.msg"
)


add_executable(armnode src/arm/armnode.cpp)
ament_target_dependencies(armnode rclcpp sensor_msgs diagnostic_msgs std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(armnode ${cpp_typesupport_target})

add_library(arm6dof_hardware SHARED
  src/arm/CanBridge.cpp
  src/arm/ArmController.cpp
  src/arm/ArmHardwareInterface.cpp
)
ament_target_dependencies(arm6dof_hardware rclcpp hardware_interface pluginlib)
pluginlib_export_plugin_description_file(hardware_interface arm6dof_plugin.xml)

install(TARGETS armnode arm6dof_hardware DESTINATION lib/${PROJECT_NAME})







install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})

install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
