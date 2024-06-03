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
- [CAD](./CAD/) : assemblies and design of parts
<div align="center">
    <img src="./assets/chessaton_ass.gif" width=50% align="center" >
</div>

## Building

### Docker
```bash
cd chessaton
docker compose build
docker compose up -d chessaton-nvidia
docker exec -it ros_humble_nvidia /bin/bash
```
to enable nvidia gpu :
#### linux:
add this config to daemon.json which is usually located at /etc/docker/daemon.json:
```json
"default-runtime": "nvidia",
    "runtimes": {
        "nvidia": {
            "args": [],
            "path": "nvidia-container-runtime"
        }
    }
```
#### windows
add above config to docker hub settings.

---
### Manual

core dependencies:

- ros-humble
- gazebo
- moveit2

rest of the dependencies could be found in [Dockerfile](./Dockerfile)

## Running nodes
### Chess
for more refer to [chessaton_chess_manager](./chessaton_chess_manager/README.md)

```bash
ros2 launch chessaton_chess_manager chess_robot.launch.py engine:=kojiro
```

|![](./assets/openning_chess_robot.gif)<br>e2e4-e7e5|![](./assets/castling_chess_robot.gif)<br>e1g1 (castling)|
|:-:|:-:|

### Pick & Place
for more refer to [chessaton_control](./chessaton_control/README.md)
#### pick and place with predefined poses
```
ros2 launch chessaton_control pick_place_demo.launch.py demo_program:="pick_place_demo"
```

<div>
    <div align="center">
        <img src="./assets/pick_place_demo.gif" width=80%>
    </div>
</div>

#### pick and place using depth map to avoid collision
```
ros2 launch chessaton_control pick_place_camera_demo.launch.py demo_program:="pick_place_depth_demo" world:=/home/ros/workspace/src/chessaton/chessaton_control/world/simple_pick_place_obstacle
```

<div>
    <div align="center">
        <img src="./assets/pick_place_depth.gif" width=80%>
    </div>
</div>

### moveit
```bash
ros2 launch chessaton_moveit_config chessaton.launch.py use_camera:=false
```

<div>
    <div align="center">
        <img src="./assets/chessaton_moveit.gif" width=80%>
    </div>
</div>