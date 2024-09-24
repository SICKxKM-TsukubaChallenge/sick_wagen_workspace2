cmake_minimum_required(VERSION 3.8)
project(path_follower)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/path_follower.cpp
  src/mppi/mppi.cpp
)

ament_auto_add_executable(path_follower_node
  src/path_follower.cpp
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

add_backward(path_follower_node)
