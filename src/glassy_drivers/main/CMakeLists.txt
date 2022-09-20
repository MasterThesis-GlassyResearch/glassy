cmake_minimum_required(VERSION 3.5)
project(main)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17 
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#GLOBING
file(GLOB_RECURSE SOURCE_FILES src/mavsdk_node/*.cpp src/ros_node/*.cpp)           #------------ important to work
# file(GLOB_RECURSE SOURCE_FILES )


add_executable(main src/main.cpp ${SOURCE_FILES})
target_compile_features(main PRIVATE cxx_std_20)
target_include_directories(main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/mavsdk_node   ${CMAKE_CURRENT_SOURCE_DIR}/src/ros_node)

ament_target_dependencies(main rclcpp std_msgs)

install(TARGETS main
  DESTINATION lib/${PROJECT_NAME})

  target_link_libraries(main
  MAVSDK::mavsdk
  # MAVSDK::mavsdk_action
  # MAVSDK::mavsdk_telemetry
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
