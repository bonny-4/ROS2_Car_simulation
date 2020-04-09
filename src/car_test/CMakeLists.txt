cmake_minimum_required(VERSION 3.5)
project(car_test)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(car_test_interfaces REQUIRED)

# Configuration for Node 1 : Speed regulation
add_executable(speed_regulation src/speed_regulation.cpp)
ament_target_dependencies(speed_regulation rclcpp std_msgs car_test_interfaces)

# Configuration for Node 2 : Speed detection
add_executable(speed_detection src/speed_detection.cpp)
ament_target_dependencies(speed_detection rclcpp std_msgs car_test_interfaces)

# Configuration for Node 3 : Controling car (Throttle and brake)
add_executable(car_controls src/car_controls.cpp)
ament_target_dependencies(car_controls rclcpp std_msgs)

# Configuration for Node 4 : Car dashboard
add_executable(car_dashboard src/car_dashboard.cpp)
ament_target_dependencies(car_dashboard rclcpp std_msgs car_test_interfaces)
    
install(TARGETS
  speed_regulation
  speed_detection
  car_controls
  car_dashboard
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

ament_package()
