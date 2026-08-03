# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22)
project(openarm_hardware)

# Pinocchio 4.0 requires C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pinocchio REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(urdf REQUIRED)

# Find openarm_can library
find_package(OpenArmCAN REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/v10_simple_hardware.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

target_link_libraries(${PROJECT_NAME}
  OpenArmCAN::openarm_can
  pinocchio::pinocchio
)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  hardware_interface
  pinocchio
  pluginlib
  rclcpp
  rclcpp_lifecycle
  urdf
)

pluginlib_export_plugin_description_file(hardware_interface openarm_hardware.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
  )

install(DIRECTORY include/
DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pinocchio
  pluginlib
  rclcpp
)

ament_package()

# Generate URDF for Pinocchio gravity compensation at build time
set(XACRO_INPUT "${CMAKE_CURRENT_SOURCE_DIR}/../../openarm_description/urdf/robot/v10.urdf.xacro")
get_filename_component(XACRO_INPUT "${XACRO_INPUT}" ABSOLUTE)
set(URDF_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/openarm.urdf")

if(EXISTS "${XACRO_INPUT}")
  find_program(XACRO_EXECUTABLE xacro
    PATHS /opt/ros/humble/bin
    NO_DEFAULT_PATH)
  if(XACRO_EXECUTABLE)
    execute_process(
      COMMAND ${XACRO_EXECUTABLE} ${XACRO_INPUT}
              "bimanual:=true"
              "ros2_control:=false"
      OUTPUT_FILE ${URDF_OUTPUT}
      RESULT_VARIABLE _xacro_result
      ERROR_VARIABLE _xacro_error
    )
    if(NOT _xacro_result EQUAL 0)
      message(WARNING "xacro generation failed: ${_xacro_error}")
    else()
      message(STATUS "Generated URDF for gravity compensation: ${URDF_OUTPUT}")
      install(FILES ${URDF_OUTPUT}
        DESTINATION share/${PROJECT_NAME})
    endif()
  endif()
endif()
