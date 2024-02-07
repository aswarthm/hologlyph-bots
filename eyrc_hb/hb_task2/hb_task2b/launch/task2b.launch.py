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

isSimulator = False

def generate_launch_description():
    share_dir = get_package_share_directory('hb_task2b')
    pkg_sim_world = get_package_share_directory('hb_world')
    pkg_sim_bot = get_package_share_directory('hb_bot')


     
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim_world, 'launch', 'world.launch.py'),
        )
    )
    spawn_bot=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim_bot, 'launch', 'multi_bot_spawn.launch.py'),
        )
    )
    goalPub = Node(
            package='hb_task2b',
            executable='nextGoalPub.py',
        )
    feedback = Node(
            package='hb_task2b',
            executable='feedback.py',
        )        
    controller = Node(
            package='hb_task2b',
            executable='bot_controller.py',
        )
    controller1 = Node(
            package='hb_task2b',
            executable='bot_controller_1.py',
        )
    controller2 = Node(
            package='hb_task2b',
            executable='bot_controller_2.py',
        )
    controller3 = Node(
            package='hb_task2b',
            executable='bot_controller_3.py',
        )
    
    if(isSimulator):
        return LaunchDescription([
            world,
            spawn_bot,
            goalPub,
            feedback,
            controller,
            # controller1,
            # controller2,
            # controller3,
            ])
    else:
        return LaunchDescription([
            # world,
            # spawn_bot,
            goalPub,
            feedback,
            controller,
            # controller1,
            # controller2,
            # controller3,
            ])
