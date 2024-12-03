cmake_minimum_required(VERSION 3.5)
project(ros_babel_fish VERSION 0.9.1)
set(CMAKE_CXX_STANDARD 17)

# If the value doesn't fit, an exception is thrown in any case because that could result in unexpected behavior and can not be ignored lightly
option(WARN_ON_INCOMPATIBLE_TYPE "If ON a warning is printed if a value message is set or accessed with a type that does not allow casting without loss of information" ON)

if (WARN_ON_INCOMPATIBLE_TYPE)
  add_definitions(-DRBF_WARN_ON_INCOMPATIBLE_TYPE)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

#set(CMAKE_BUILD_TYPE "Debug")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

###########
## Build ##
###########

set(SOURCES
  src/detail/babel_fish_action_client.cpp
  src/detail/babel_fish_action_server.cpp
  src/detail/babel_fish_publisher.cpp
  src/detail/babel_fish_service.cpp
  src/detail/babel_fish_service_client.cpp
  src/detail/babel_fish_subscription.cpp
  src/idl/providers/local_type_support_provider.cpp
  src/idl/serialization.cpp
  src/idl/type_support_provider.cpp
  src/messages/array_message.cpp
  src/messages/compound_message.cpp
  src/messages/message.cpp
  src/babel_fish.cpp
  src/detail/topic.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_compile_definitions(${PROJECT_NAME} PUBLIC RCLCPP_VERSION_MAJOR=${rclcpp_VERSION_MAJOR})
target_link_libraries(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
ament_target_dependencies(${PROJECT_NAME} ament_index_cpp rclcpp rclcpp_action rcpputils rosidl_runtime_cpp rosidl_typesupport_cpp rosidl_typesupport_introspection_cpp)


# Examples
add_executable(any_publisher examples/any_publisher.cpp)
target_link_libraries(any_publisher ${PROJECT_NAME})
target_include_directories(any_publisher PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(any_subscriber examples/any_subscriber.cpp)
target_link_libraries(any_subscriber ${PROJECT_NAME})
target_include_directories(any_subscriber PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(troll_node examples/troll_node.cpp)
target_link_libraries(troll_node ${PROJECT_NAME})
target_include_directories(troll_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(service_server examples/service_server.cpp)
target_link_libraries(service_server ${PROJECT_NAME})
target_include_directories(service_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(service_client examples/service_client.cpp)
target_link_libraries(service_client ${PROJECT_NAME})
target_include_directories(service_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(action_client examples/action_client.cpp)
target_link_libraries(action_client ${PROJECT_NAME})
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

#############
## Install ##
#############

## Mark libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

# Mark examples for installation
install(
  TARGETS action_client any_publisher any_subscriber service_client service_server troll_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)


# Export old style CMake variables
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action rosidl_runtime_cpp rosidl_typesupport_cpp rosidl_typesupport_introspection_cpp)

#############
## Testing ##
#############

if (BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/../.clang-format")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(action_tutorials_interfaces REQUIRED)
  find_package(example_interfaces REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(ros_babel_fish_test_msgs REQUIRED)
  include_directories(include)

  ament_add_gtest(test_message test/message.cpp)
  ament_target_dependencies(test_message geometry_msgs ros_babel_fish_test_msgs)
  target_link_libraries(test_message ${PROJECT_NAME})

  ament_add_gtest(test_message_decoding test/message_decoding.cpp)
  ament_target_dependencies(test_message_decoding geometry_msgs ros_babel_fish_test_msgs)
  target_link_libraries(test_message_decoding ${PROJECT_NAME})

  ament_add_gtest(test_message_encoding test/message_encoding.cpp)
  ament_target_dependencies(test_message_encoding geometry_msgs ros_babel_fish_test_msgs)
  target_link_libraries(test_message_encoding ${PROJECT_NAME})

  ament_add_gtest(test_service test/service.cpp)
  ament_target_dependencies(test_service example_interfaces ros_babel_fish_test_msgs)
  target_link_libraries(test_service ${PROJECT_NAME})

  ament_add_gtest(test_action_client test/action_client.cpp)
  ament_target_dependencies(test_action_client action_tutorials_interfaces example_interfaces ros_babel_fish_test_msgs)
  target_link_libraries(test_action_client ${PROJECT_NAME})

  ament_add_gtest(test_action_server test/action_server.cpp)
  ament_target_dependencies(test_action_server example_interfaces ros_babel_fish_test_msgs)
  target_link_libraries(test_action_server ${PROJECT_NAME})
endif ()

# to run: catkin build ros_babel_fish --no-deps -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug -v --catkin-make-args ros_babel_fish_coverage
# Path to results overview will be printed in the build process
# Big thanks to the moveit people from whose docs I've obtained the information and code to get the coverage
# Note: Sometimes this command has to be run twice. Don't know why. If you do, please tell me!
#if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
#  find_package(code_coverage REQUIRED)   # catkin package ros-*-code-coverage
#  include(CodeCoverage)
#  append_coverage_compiler_flags()
#  set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*" "*/${PROJECT_NAME}/examples*" "*/${PROJECT_NAME}/benchmarks*")
#  add_code_coverage(NAME ${PROJECT_NAME}_coverage)
#endif ()

find_package(benchmark QUIET)
if (${benchmark_FOUND})
  find_package(geometry_msgs REQUIRED)
  add_executable(benchmark_message benchmark/message.cpp)
  ament_target_dependencies(benchmark_message geometry_msgs)
  target_link_libraries(benchmark_message ${PROJECT_NAME} benchmark)
  target_include_directories(benchmark_message PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  install(
    TARGETS benchmark_message
    DESTINATION lib/${PROJECT_NAME}
  )
endif ()

ament_package()
