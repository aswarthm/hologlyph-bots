rm build/ install/ log/ -r && colcon build --cmake-args -Wno-dev && source install/setup.bash && ros2 launch hb_task_1b hb_task_1b.launch.py 
