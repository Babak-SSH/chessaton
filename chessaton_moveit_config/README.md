# chessaton_moveit_config

```bash
.
├── CMakeLists.txt
├── config
│   ├── chessaton_controllers.yaml          #controller management configuration
│   ├── joint_limits.yaml                   #speed limits
│   ├── kinematics_4dof.yaml                #ikfast produced kinematics (there are some limits with this kinematic for now)
│   ├── kinematics.yaml                     #normal default kinematics
│   ├── ompl_planning.yaml                  #ompl configurations
│   ├── sensor_3d.yaml                      #octomap settings
│   └── servo.yaml
├── launch
│   └── chessaton.launch.py
├── package.xml
├── README.md
├── rviz
│   └── moveit.rviz                         #rviz2 stup for moveit2 stack
├── scripts
│   └── xacro2srdf.bash
└── srdf
    ├── chessaton_macro.srdf.xacro         #semantic information about the robot structure
    ├── chessaton.srdf
    └── chessaton.srdf.xacro
```

- kinematics_4dof.yaml is the kinematic solver produced by ikfast which have some problems when used with moveit2, because moveit2 is made for arms with 6 and higher dof. to overcome this issue for now, two virtual (fake) joints have been added to hand section (yaw and roll) to be able to use 6dof kinematic solvers.

TODO: fix 4dof kinematic solver
## nodes

```bash
ros2 launch chessaton_moveit_config chessaton.launch.py use_camera:=false
```