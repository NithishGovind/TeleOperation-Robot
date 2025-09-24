cmake_minimum_required(VERSION 3.8)
# Minimum required version of CMake to build this package

project(zenorak_serial)
# Name of your package/project

# --- Compiler Warnings (Optional) ---
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # -Wall: show all warnings
  # -Wextra: show extra warnings
  # -Wpedantic: enforce strict ISO C++ standard
endif()

# --- Find ROS 2 dependencies ---
find_package(ament_cmake REQUIRED)  # Required to build ROS 2 packages
find_package(rclpy REQUIRED)        # Needed because this node uses ROS 2 Python client
find_package(std_msgs REQUIRED)     # Needed because we subscribe to std_msgs/String

# --- Install Python Node ---
install(
  PROGRAMS
    scripts/zenorak_serial.py  # Path to your subscriber Python script
  DESTINATION lib/${PROJECT_NAME} # ROS 2 expects Python nodes here
)


# --- Testing and Linting (Optional) ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright check temporarily
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint check temporarily
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --- Declare This Directory as a ROS 2 Package ---
ament_package()
