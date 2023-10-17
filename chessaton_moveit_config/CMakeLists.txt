cmake_minimum_required(VERSION 3.5)
project(chessaton_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(chessaton_description REQUIRED)
find_package(chessaton_chessaton_arm_ikfast_plugin REQUIRED)

# Install directories
install(DIRECTORY config launch rviz srdf DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()