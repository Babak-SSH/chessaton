
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
find_package(chessaton_chessaton_arm_ikfast_plugin)

install(DIRECTORY launch worlds models DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()