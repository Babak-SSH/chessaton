cmake_minimum_required(VERSION 3.5)
project(chessaton_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(chessaton_description REQUIRED)
find_package(chessaton_moveit_config REQUIRED)
find_package(chessaton_chessaton_arm_ikfast_plugin)

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

# Install directories
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
