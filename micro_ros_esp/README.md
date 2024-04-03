this program provides means of communication between main PC's ROS2 nodes and esp32 using micro-ros.

:diagram:

we subscribe to /joint_states topic and convert them to commands that we can feed to servo motors using esp32.