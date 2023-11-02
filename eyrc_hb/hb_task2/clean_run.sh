rm build/ install/ log/ -r
colcon build --packages-skip hb_task2a --cmake-args -Wno-dev && 
source install/setup.bash &&
ros2 launch hb_task2b task2b.launch.py
