rm build/ install/ log/ -r
colcon build --cmake-args -Wno-dev &&
source install/setup.bash &&
ros2 launch hb_task4b task4b.launch.py
