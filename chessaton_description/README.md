# chessaton_description

chessaton's models, urdfs, sdfs and joint configurations.

```bash
.
├── chessaton
│   └── model.sdf
├── CMakeLists.txt
├── config                                      #joints and controllers configuration 
│   ├── controllers_effort.yaml
│   ├── controllers_position,velocity.yaml
│   ├── controllers_position.yaml
│   ├── controllers_velocity.yaml
│   └── initial_joint_positions.yaml
├── launch
│   ├── view_gz.launch.py                       #robotic arm sim in gazebo
│   └── view.launch.py                          #robotc arm in rviz2
├── meshes
│   ├── chessaton_gripper.STL
│   ├── chessaton_link0.STL
│   ├── chessaton_link1.STL
│   ├── chessaton_link2.STL
│   ├── chessaton_link3.STL
│   ├── chessaton_link4.STL
│   ├── left_finger.STL
│   ├── right_finger.STL
│   └── sensors
├── package.xml
├── README.md
├── rviz                                        #rviz2 setups
│   ├── view_robot.rviz
│   └── view_with_camera.rviz
├── scripts                                     #conversion scripts
│   ├── xacro2sdf.bash
│   ├── xacro2sdf_direct.bash
│   └── xacro2urdf.bash
├── urdf
│   ├── chessaton.csv                           #metadata and details of parts
│   ├── chessaton_gazebo.xacro                  #config related to gazebo sim, ros2 controllers and gazebo_grasp plugin
│   ├── chessaton_hand.xacro                    #xacro of sections related to gripper
│   ├── chessaton_include.xacro                 #colors ...
│   ├── chessaton.ros2_control                  #macros to add ros2_control
│   ├── chessaton.urdf                          #output of xacro2urdf.bash
│   ├── chessaton.urdf.xacro                    #main urdf file
│   └── sensors
│       ├── camera_gazebo.xacro
│       └── camera.urdf.xacro
└── world
    └── chessaton.world                         #empty world with robotic arm
```

## nodes

### robot in rviz
```bash
ros2 launch chess_description view.launch.py
```

### robot in gazebo
```bash
ros2 launch chess_description view_gz.launch.py
```

## configuring

after modifying chessaton.urdf.xacro run these two scripts:

you can also choose the controller type to use inside these scripts.
```bash
chessaton_description/scripts/xacro2urdf.bash
chessaton_description/scripts/xacro2sdf.bash
```