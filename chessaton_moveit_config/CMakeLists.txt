cmake_minimum_required(VERSION 3.5)
project(chessaton_moveit_config)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(chessaton_description REQUIRED)
find_package(chessaton_chessaton_arm_ikfast_plugin REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)

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