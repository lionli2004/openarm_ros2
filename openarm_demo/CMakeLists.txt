cmake_minimum_required(VERSION 3.8)
project(openarm_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# Custom service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Replay.srv"
  "srv/Status.srv"
  DEPENDENCIES std_msgs
)

# --- RViz panel plugin -----------------------------------------------------
qt5_wrap_cpp(MOC_FILES include/openarm_demo/demo_panel.hpp)
add_library(${PROJECT_NAME}_panel SHARED src/demo_panel.cpp ${MOC_FILES})
target_include_directories(${PROJECT_NAME}_panel PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}_panel
  rclcpp
  rviz_common
  std_msgs
  std_srvs
  pluginlib
)
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME}
  "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_panel ${typesupport_target} Qt5::Widgets)
# dlopen'd by RViz: without RPATH the rosidl typesupport libraries in
# ${prefix}/lib cannot be found at load time
set_target_properties(${PROJECT_NAME}_panel PROPERTIES
  INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
  BUILD_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
)
pluginlib_export_plugin_description_file(rviz_common openarm_demo_panel_plugin.xml)
install(TARGETS ${PROJECT_NAME}_panel DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# --- Python demo controller ------------------------------------------------
install(PROGRAMS src/demo_controller.py DESTINATION lib/${PROJECT_NAME})

ament_package()
