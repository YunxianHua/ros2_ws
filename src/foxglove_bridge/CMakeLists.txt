cmake_minimum_required(VERSION 3.10.2)

if(POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0048 NEW)
endif()
if(POLICY CMP0024)
  cmake_policy(SET CMP0024 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0024 NEW)
endif()

project(foxglove_bridge LANGUAGES CXX VERSION 0.7.3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

macro(enable_strict_compiler_warnings target)
  if (MSVC)
    target_compile_options(${target} PRIVATE /WX /W4)
  elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror -Wold-style-cast -Wfloat-equal -Wmost -Wunused-exception-parameter)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror -Wold-style-cast -Wfloat-equal)
  endif()
endmacro()

find_package(nlohmann_json QUIET)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
find_package(websocketpp REQUIRED)
find_package(ZLIB REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Determine wheter to use standalone or boost asio
option(USE_ASIO_STANDALONE "Build with standalone ASIO" ON)
if(USE_ASIO_STANDALONE)
  message(STATUS "Using standalone ASIO")
  add_definitions(-DASIO_STANDALONE)
else()
  message(STATUS "Using Boost ASIO")
  find_package(Boost REQUIRED)
endif(USE_ASIO_STANDALONE)

# Detect big-endian architectures
include(TestBigEndian)
TEST_BIG_ENDIAN(ENDIAN)
if (ENDIAN)
  add_compile_definitions(ARCH_IS_BIG_ENDIAN=1)
endif()

# Get git commit hash and replace variables in version.cpp.in
find_program(GIT_SCM git DOC "Git version control")
if (GIT_SCM)
  execute_process(
          COMMAND ${GIT_SCM} describe --always --dirty --exclude="*"
          WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
          OUTPUT_VARIABLE FOXGLOVE_BRIDGE_GIT_HASH
          OUTPUT_STRIP_TRAILING_WHITESPACE
  )
endif()
set(FOXGLOVE_BRIDGE_VERSION "${CMAKE_PROJECT_VERSION}")
configure_file(foxglove_bridge_base/src/version.cpp.in
        foxglove_bridge_base/src/version.cpp @ONLY)

# Build the foxglove_bridge_base library
add_library(foxglove_bridge_base SHARED
        foxglove_bridge_base/src/base64.cpp
        foxglove_bridge_base/src/foxglove_bridge.cpp
        foxglove_bridge_base/src/parameter.cpp
        foxglove_bridge_base/src/serialization.cpp
        foxglove_bridge_base/src/server_factory.cpp
        foxglove_bridge_base/src/test/test_client.cpp
        # Generated:
        ${CMAKE_CURRENT_BINARY_DIR}/foxglove_bridge_base/src/version.cpp
)
target_include_directories(foxglove_bridge_base
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/foxglove_bridge_base/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(foxglove_bridge_base
        OpenSSL::Crypto
        OpenSSL::SSL
        ZLIB::ZLIB
        ${CMAKE_THREAD_LIBS_INIT}
)
if(nlohmann_json_FOUND)
  target_link_libraries(foxglove_bridge_base nlohmann_json::nlohmann_json)
else()
  message(STATUS "nlohmann_json not found, will search at compile time")
endif()
enable_strict_compiler_warnings(foxglove_bridge_base)

if(DEFINED ENV{AMENT_PREFIX_PATH})
  message(STATUS "Building with ament_cmake")
  set(ROS_BUILD_TYPE "ament_cmake")

  find_package(ament_cmake REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(resource_retriever REQUIRED)

  add_library(foxglove_bridge_component SHARED
          ros2_foxglove_bridge/src/message_definition_cache.cpp
          ros2_foxglove_bridge/src/param_utils.cpp
          ros2_foxglove_bridge/src/ros2_foxglove_bridge.cpp
          ros2_foxglove_bridge/src/parameter_interface.cpp
          ros2_foxglove_bridge/src/generic_client.cpp
          ros2_foxglove_bridge/src/generic_subscription.cpp
  )
  target_include_directories(foxglove_bridge_component
          PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/foxglove_bridge_base/include>
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ros2_foxglove_bridge/include>
          $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(foxglove_bridge_component rclcpp rclcpp_components resource_retriever rosgraph_msgs)
  target_link_libraries(foxglove_bridge_component foxglove_bridge_base)
  rclcpp_components_register_nodes(foxglove_bridge_component "foxglove_bridge::FoxgloveBridge")
  enable_strict_compiler_warnings(foxglove_bridge_component)
  add_executable(foxglove_bridge
          ros2_foxglove_bridge/src/ros2_foxglove_bridge_node.cpp
  )
  target_include_directories(foxglove_bridge SYSTEM PRIVATE ${rclcpp_INCLUDE_DIRS})
  ament_target_dependencies(foxglove_bridge rclcpp rclcpp_components)
  enable_strict_compiler_warnings(foxglove_bridge)
else()
  message(FATAL_ERROR "Could not find ament_cmake")
endif()


#### INSTALL ###################################################################

if(ROS_BUILD_TYPE STREQUAL "catkin")
  install(TARGETS foxglove_bridge
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(TARGETS foxglove_bridge_base foxglove_bridge_nodelet
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )
  install(FILES nodelets.xml
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
  install(DIRECTORY ros1_foxglove_bridge/launch/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
elseif(ROS_BUILD_TYPE STREQUAL "ament_cmake")
  install(FILES ros2_foxglove_bridge/include/foxglove_bridge/ros2_foxglove_bridge.hpp
          DESTINATION include/${PROJECT_NAME}/
  )
  install(TARGETS foxglove_bridge
          DESTINATION lib/${PROJECT_NAME}
  )
  install(TARGETS foxglove_bridge_base foxglove_bridge_component
          ARCHIVE DESTINATION lib
          LIBRARY DESTINATION lib
          RUNTIME DESTINATION bin
  )
  install(DIRECTORY ros2_foxglove_bridge/launch
          DESTINATION share/${PROJECT_NAME}/
  )
  install(FILES ros2_foxglove_bridge/include/foxglove_bridge/utils.hpp
          DESTINATION include/${PROJECT_NAME}/
  )
  ament_export_libraries(foxglove_bridge_base foxglove_bridge_component)
  ament_package()
endif()
