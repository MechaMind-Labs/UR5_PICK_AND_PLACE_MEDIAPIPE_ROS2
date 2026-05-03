cmake_minimum_required(VERSION 3.8)
project(ur5_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)

# Install launch files and config
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python package (ur5_controller/__init__.py etc.)
ament_python_install_package(${PROJECT_NAME})

# Install Python entry-point scripts.
# The script is installed WITHOUT the .py extension so that
# ros2 run ur5_controller slider_controller works.
install(PROGRAMS
  ${PROJECT_NAME}/slider_controller.py
  RENAME slider_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
