# CHESSATON
chess playing 4dof robotic arm!
## Description
This ROS package contains the motion planning stack and chess interface (which uses my own chess engine [Kojiro](https://github.com/Babak-SSH/Kojiro/blob/main/)) for a semi-custom 4 DOF robotic arm.
The associated simulation environment in Gazebo includes chess-board and it's pieces with same size as their real life counter parts. 
The robotic manipulator hardware is controlled using micro-ros + esp32 + pca9685.

### packages
- [chessaton_description](./chessaton_description/README.md) : robots urdfs and base configurations
- [chessaton_moveit_config](./chessaton_moveit_config/README.md) : chessaton moveit2 stack
- [chessaton_control](./chessaton_control/README.md) : group of interfaces to control the arm and some demos of pick & place
- [chessaton_chess_manager](./chessaton_chess_manager/README.md) : chess related interfaces which help the robotic arm to play chess
- [micro_ros_esp](./micro_ros_esp/README.md) : bridge between main pc and esp32 to control servos
- [chessaton_interfaces](./chessaton_interfaces/README.md) : custom data types
- [chessaton_chessaton_arm_ikfast_plugin](./chessaton_chessaton_arm_ikfast_plugin/README.md) : 4dof kinematic solver made with ikfast


## Building

### Docker

## Running nodes
## moveit
## Pick & Place
## Chess