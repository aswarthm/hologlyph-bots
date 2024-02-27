rm build/ install/ log/ -r
colcon build  --packages-skip hb_task_2b --cmake-args -Wno-dev && 
source install/setup.bash &&
ros2 launch hb_task2a task2a.launch.py
