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
from launch import LaunchDescription


def generate_launch_description():

    feedback = Node(
            package='hb_task4b',
            executable='feedback.py',
        )        

    return LaunchDescription([
        feedback,
        ])
