cmake_minimum_required(VERSION 3.5)
project(glassy_outerloop)

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
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(glassy_msgs REQUIRED)
find_package (Eigen3 REQUIRED)



file(GLOB_RECURSE SOURCE_FILES src/PathFollowingNode/*.cpp ../control_lib/*.cpp) 

add_executable(glassy_outerloop src/glassy_outerloop_main.cpp ${SOURCE_FILES} )

target_compile_features(glassy_outerloop PRIVATE cxx_std_20)
target_include_directories(glassy_outerloop PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/PathFollowingNode ${CMAKE_CURRENT_SOURCE_DIR}/../control_lib ${EIGEN3_INCLUDE_DIR})

ament_target_dependencies(glassy_outerloop rclcpp sensor_msgs glassy_msgs std_srvs)

install(TARGETS glassy_outerloop
  DESTINATION lib/${PROJECT_NAME})

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

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
