cmake_minimum_required(VERSION 3.5)
project(chessaton_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(chessaton_description REQUIRED)
find_package(chessaton_moveit_config REQUIRED)
find_package(chessaton_chessaton_arm_ikfast_plugin REQUIRED)

find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)

# Convert xacros of description and moveit_config packages to URDF, SDF and SRDF
# get_filename_component(${PROJECT_NAME}_REPO_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
# set(${PROJECT_NAME}_DESCRIPTION_SCRIPTS_DIR ${${PROJECT_NAME}_REPO_DIR}/chessaton_description/scripts)
# set(${PROJECT_NAME}_MOVEIT_CONFIG_SCRIPTS_DIR ${${PROJECT_NAME}_REPO_DIR}/chessaton_moveit_config/scripts)
# execute_process(
#   COMMAND ${${PROJECT_NAME}_DESCRIPTION_SCRIPTS_DIR}/xacro2urdf.bash
# )
# execute_process(
#   COMMAND ${${PROJECT_NAME}_DESCRIPTION_SCRIPTS_DIR}/xacro2sdf.bash
# )
# execute_process(
#   COMMAND ${${PROJECT_NAME}_MOVEIT_CONFIG_SCRIPTS_DIR}/xacro2srdf.bash
# )

add_executable(move_group_demo src/move_group_demo.cpp)
# target_include_directories(move_group_demo PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
target_compile_features(move_group_demo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  move_group_demo
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

install(TARGETS move_group_demo
  DESTINATION lib/${PROJECT_NAME})

add_executable(pick_place_demo src/pick_place_demo.cpp)
# target_include_directories(pick_place_demo PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
target_compile_features(pick_place_demo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  pick_place_demo
  "moveit_task_constructor_core"
  "rclcpp"
)

install(TARGETS pick_place_demo
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# add_executable(chessaton_pick_place_demo src/pick_place_demo.cpp)
# ament_target_dependencies(chessaton_pick_place_demo moveit_task_constructor_core rclcpp)
# set_target_properties(chessaton_pick_place_demo PROPERTIES OUTPUT_NAME pick_place_demo PREFIX "")
# target_include_directories(chessaton_pick_place_demo PUBLIC
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>)
# target_compile_features(chessaton_pick_place_demo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# install(TARGETS chessaton_pick_place_demo
#     DESTINATION lib/${PROJECT_NAME}
# )

# Install directories
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
