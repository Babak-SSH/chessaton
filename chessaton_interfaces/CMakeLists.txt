cmake_minimum_required(VERSION 3.8)
project(chessaton_interfaces)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)


set(action_files

)

set(msg_files

)

set(srv_files
    "srv/BoxPositions.srv"
    "srv/GetBestMove.srv"
    "srv/SetState.srv"
    DEPENDENCIES geometry_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${action_files}
    ${msg_files}
    ${srv_files}
)

install(FILES DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()