- moveit task constructor example:
```
ros2 launch chessaton_control mtc_demo.launch.py
```
- move to given position example:
terminal 1
```
ros2 launch ros2 launch chessaton_moveit_config chessaton.launch.py rviz_config:='./src/chessaton/chessaton_control/rviz/movep_demo.rviz'
```
terminal 2
```
ros2 run chessaton_control movep_demo
```