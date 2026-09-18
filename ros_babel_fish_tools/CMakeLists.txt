cmake_minimum_required(VERSION 3.15)
project(ros_babel_fish_tools VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_babel_fish REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# lz4 and zstd are used only by the stats tool's --compress option.
find_package(PkgConfig REQUIRED)
pkg_check_modules(LZ4 REQUIRED IMPORTED_TARGET liblz4)
pkg_check_modules(ZSTD REQUIRED IMPORTED_TARGET libzstd)

###########
## Build ##
###########

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE
  ros_babel_fish::ros_babel_fish
  yaml-cpp
)

add_executable(echo src/echo.cpp)
target_link_libraries(echo PRIVATE ${PROJECT_NAME} rclcpp::rclcpp)

add_executable(topic_stats src/topic_stats.cpp)
target_link_libraries(topic_stats PRIVATE ${PROJECT_NAME} rclcpp::rclcpp PkgConfig::LZ4 PkgConfig::ZSTD)

add_executable(service_stats src/service_stats.cpp)
target_link_libraries(service_stats PRIVATE ${PROJECT_NAME} rclcpp::rclcpp)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

install(
  TARGETS echo topic_stats service_stats
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp ros_babel_fish yaml_cpp_vendor)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(example_interfaces REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(ros_babel_fish_test_msgs REQUIRED)

  ament_add_gtest(test_nlohmann_json_serialization test/nlohmann_json_serialization.cpp)
  target_link_libraries(test_nlohmann_json_serialization ${PROJECT_NAME} ${geometry_msgs_TARGETS} ${std_msgs_TARGETS} ${ros_babel_fish_test_msgs_TARGETS})

  ament_add_gtest(test_yaml_cpp_serialization test/yaml_cpp_serialization.cpp)
  target_link_libraries(test_yaml_cpp_serialization ${PROJECT_NAME} ${geometry_msgs_TARGETS} ${std_msgs_TARGETS} ${ros_babel_fish_test_msgs_TARGETS})

  # Unit tests for the helpers shared by the stats tools (headers live next to the tool sources).
  ament_add_gtest(test_stats_common test/stats_common.cpp)
  target_include_directories(test_stats_common PRIVATE src)
  target_link_libraries(test_stats_common ${PROJECT_NAME})

  ament_add_gtest(test_compression test/compression.cpp)
  target_include_directories(test_compression PRIVATE src)
  target_link_libraries(test_compression ${PROJECT_NAME} PkgConfig::LZ4 PkgConfig::ZSTD)

  # End-to-end tests that run the built tool executables against publishers/servers in the test.
  # Each test binary gets its own ROS domain so concurrently running tests (colcon runs packages in
  # parallel) and nodes on the developer's machine do not show up in its graph.
  set(TOOL_TEST_ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
  ament_add_gtest(test_echo_tool test/echo.cpp TIMEOUT 120 ENV ${TOOL_TEST_ENV} ROS_DOMAIN_ID=61)
  add_dependencies(test_echo_tool echo)
  target_compile_definitions(test_echo_tool PRIVATE ECHO_EXECUTABLE="$<TARGET_FILE:echo>")
  target_link_libraries(test_echo_tool ${PROJECT_NAME} rclcpp::rclcpp ${geometry_msgs_TARGETS} ${std_msgs_TARGETS})

  ament_add_gtest(test_topic_stats_tool test/topic_stats.cpp TIMEOUT 180 ENV ${TOOL_TEST_ENV} ROS_DOMAIN_ID=62)
  add_dependencies(test_topic_stats_tool topic_stats)
  target_compile_definitions(test_topic_stats_tool PRIVATE TOPIC_STATS_EXECUTABLE="$<TARGET_FILE:topic_stats>")
  target_link_libraries(test_topic_stats_tool ${PROJECT_NAME} rclcpp::rclcpp ${geometry_msgs_TARGETS} ${std_msgs_TARGETS})

  ament_add_gtest(test_service_stats_tool test/service_stats.cpp TIMEOUT 180 ENV ${TOOL_TEST_ENV} ROS_DOMAIN_ID=63)
  add_dependencies(test_service_stats_tool service_stats)
  target_compile_definitions(test_service_stats_tool PRIVATE SERVICE_STATS_EXECUTABLE="$<TARGET_FILE:service_stats>")
  target_link_libraries(test_service_stats_tool ${PROJECT_NAME} rclcpp::rclcpp ${example_interfaces_TARGETS})
endif()

ament_package()
