
cmake_minimum_required(VERSION 3.8)
project(chessaton_chess_manager)

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
find_package(chessaton_interfaces REQUIRED)
find_package(chessaton_chessaton_arm_ikfast_plugin REQUIRED)
find_package(chessaton_control REQUIRED)

find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

add_library(chess_handler SHARED src/chess_handler.cpp src/position.cpp) 
ament_target_dependencies(chess_handler
    "rclcpp"
    "moveit_ros_planning_interface"
    "moveit_visual_tools"
    "chessaton_interfaces"
    chessaton_control
)
ament_export_libraries(chess_handler)
ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS chess_handler
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(chess_demo src/chess_demo.cpp src/position.cpp)
target_include_directories(chess_demo PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(chess_demo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(chess_demo
    "moveit_ros_planning_interface"
    "moveit_visual_tools"
    "rclcpp"
    chessaton_control
)
install(TARGETS chess_demo
    DESTINATION lib/${PROJECT_NAME})

add_executable(chess_engine src/chess_engine.cpp)
target_include_directories(chess_engine PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(chess_engine PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(chess_engine
    "rclcpp"
    chessaton_interfaces
)
install(TARGETS chess_engine
    DESTINATION lib/${PROJECT_NAME})

add_executable(chess_robot src/chess_robot.cpp src/position.cpp src/chess_handler.cpp)
target_include_directories(chess_robot PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(chess_robot PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(chess_robot
    "moveit_ros_planning_interface"
    "moveit_visual_tools"
    "rclcpp"
    "chessaton_interfaces"
    chessaton_control
)
install(TARGETS chess_robot
    DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch worlds models DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()