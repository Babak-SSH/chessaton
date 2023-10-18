# needs change won't work in ros2
search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=chessaton.srdf
robot_name_in_srdf=chessaton
moveit_config_pkg=chessaton_moveit_config
robot_name=chessaton
planning_group_name=chessaton_arm
ikfast_plugin_pkg=chessaton_chessaton_arm_ikfast_plugin
base_link_name=chessaton_link0
eef_link_name=chessaton_gripper
ikfast_output_path=/home/babak-ssh/playground/test_ik/chessaton_chessaton_arm_ikfast_plugin/src/chessaton_chessaton_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
