cmake_minimum_required(VERSION 3.5)
project(ros_babel_fish_tools VERSION 4.26.43)
# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_babel_fish REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

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

add_executable(ros_babel_fish_echo src/ros_babel_fish_echo.cpp)
target_link_libraries(ros_babel_fish_echo PRIVATE ${PROJECT_NAME} rclcpp::rclcpp)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

install(
  TARGETS ros_babel_fish_echo
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
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(ros_babel_fish_test_msgs REQUIRED)

  ament_add_gtest(test_nlohmann_json_serialization test/nlohmann_json_serialization.cpp)
  target_link_libraries(test_nlohmann_json_serialization ${PROJECT_NAME} ${geometry_msgs_TARGETS} ${std_msgs_TARGETS} ${ros_babel_fish_test_msgs_TARGETS})

  ament_add_gtest(test_yaml_cpp_serialization test/yaml_cpp_serialization.cpp)
  target_link_libraries(test_yaml_cpp_serialization ${PROJECT_NAME} ${geometry_msgs_TARGETS} ${std_msgs_TARGETS} ${ros_babel_fish_test_msgs_TARGETS})
endif()

ament_package()
