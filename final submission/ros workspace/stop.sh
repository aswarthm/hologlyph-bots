ros2 topic pub --once /hb_bot_1/cmd_vell geometry_msgs/Vector3 "{x: 90.0, y: 90.0, z: 90.0}"
ros2 topic pub --once /hb_bot_2/cmd_vell geometry_msgs/Vector3 "{x: 90.0, y: 90.0, z: 90.0}"
ros2 topic pub --once /hb_bot_3/cmd_vell geometry_msgs/Vector3 "{x: 90.0, y: 90.0, z: 90.0}"

ros2 topic pub --once /pen1_down std_msgs/Bool "data: 0"
ros2 topic pub --once /pen2_down std_msgs/Bool "data: 0"
ros2 topic pub --once /pen3_down std_msgs/Bool "data: 0"
