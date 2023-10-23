''' 
*****************************************************************************************
*
*        =============================================
*                  HB Theme (eYRC 2023-24)
*        =============================================
*
*
*  Filename:			Spawn_bot.launch.py
*  Description:         Use this file to spawn bot.
*  Created:				16/07/2023
*  Last Modified:	    16/09/2023
*  Modified by:         Srivenkateshwar
*  Author:				e-Yantra Team
*  
*****************************************************************************************
'''

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription ,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution,LaunchConfiguration, PythonExpression
import os
from ament_index_python.packages import get_package_share_directory,get_package_prefix


def generate_launch_description():
    pkg_name = 'hb_task2a'
    share_dir = get_package_share_directory(pkg_name)
    pkg_sim_world = get_package_share_directory('hb_world')
    pkg_sim_bot = get_package_share_directory('hb_bot')


     
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim_world, 'launch', 'world.launch.py'),
        )
    )
    spawn_bot=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim_bot, 'launch', 'Spawn_bot.launch.py'),
        )
    )
    service=Node(
        package=pkg_name,           # Enter the name of your ROS2 package
        executable="service_node.py",    # Enter the name of your executable
    )
    feedback=Node(
        package=pkg_name,           # Enter the name of your ROS2 package
        executable="feedback.py",    # Enter the name of your executable
    )
    controller=Node(
        package=pkg_name,           # Enter the name of your ROS2 package
        executable="controller.py",    # Enter the name of your executable
    )
    return LaunchDescription([
        service,
        world,
        spawn_bot,
        feedback,
        controller,        
        ])
