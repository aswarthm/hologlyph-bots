from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    service_node = Node(
        package="hb_task_1b",           # Enter the name of your ROS2 package
        executable="service_node.py",    # Enter the name of your executable
    )
    controller_node = Node(
        package="hb_task_1b",           # Enter the name of your ROS2 package
        executable="controller.py",    # Enter the name of your executable
    )

    return LaunchDescription([
        controller_node,
        service_node,
    ])