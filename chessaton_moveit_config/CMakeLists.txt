cmake_minimum_required(VERSION 3.5)
project(chessaton_moveit_config)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(chessaton_description REQUIRED)
find_package(chessaton_chessaton_arm_ikfast_plugin REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(movep src/movep.cpp)
# target_include_directories(movep PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
target_compile_features(movep PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  movep
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

install(TARGETS movep
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

add_executable(chessaton_moveit_config_pick_place_demo src/pick_place_memo.cpp)
ament_target_dependencies(chessaton_moveit_config_pick_place_demo moveit_task_constructor_core rclcpp)
set_target_properties(chessaton_moveit_config_pick_place_demo PROPERTIES OUTPUT_NAME pick_place_memo PREFIX "")
# target_include_directories(chessaton_moveit_config_pick_place_demo PUBLIC
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>)
# target_compile_features(chessaton_moveit_config_pick_place_demo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS chessaton_moveit_config_pick_place_demo
    DESTINATION lib/${PROJECT_NAME}
)
# Install directories
install(DIRECTORY config launch rviz srdf DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()