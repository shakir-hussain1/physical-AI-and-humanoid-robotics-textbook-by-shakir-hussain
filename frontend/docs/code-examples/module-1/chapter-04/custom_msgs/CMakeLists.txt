# Physical AI & Humanoid Robotics Textbook
# Module 1, Chapter 4: Custom Messages CMakeLists.txt
# Description: Build configuration for AI-robot interface definitions
# Build: colcon build --packages-select humanoid_ai_msgs

cmake_minimum_required(VERSION 3.8)
project(humanoid_ai_msgs)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ============================================================
# Message, Service, and Action Generation
# ============================================================

# Declare all interface files
set(msg_files
  "msg/AICommand.msg"
  "msg/AICommandResult.msg"
  "msg/RobotState.msg"
)

set(srv_files
  "srv/InterpretCommand.srv"
  "srv/ValidateCommand.srv"
)

set(action_files
  "action/ExecuteBehavior.action"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    builtin_interfaces
)

# ============================================================
# Testing (optional)
# ============================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ============================================================
# Package Installation
# ============================================================

ament_package()
