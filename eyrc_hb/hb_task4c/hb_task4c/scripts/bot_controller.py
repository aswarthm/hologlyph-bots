#!/usr/bin/env python3
'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2B of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''


# Team ID:		    hb_1036
# Author List:		[ M Aswartha Reddy, D K Bharath Reddy, Pulkit Dhamija, Sangeeta Prasad ]
# Filename:		    bot_controller.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		
#                   Subs: [ 'hb_bot_{self.bot_id}/goal', '/detected_aruco_{self.bot_id}' ]
#                   Pubs: [ "/hb_bot_{self.bot_id}/rear_wheel_force", "/hb_bot_{self.bot_id}/left_wheel_force", "/hb_bot_{self.bot_id}/right_wheel_force" ]


################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.msg import Goal             

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D    
import numpy as np  


from std_msgs.msg import Int32


# bot_id = 2

class HBController(Node):
    def __init__(self, bot_id):
        self.bot_id = bot_id
        super().__init__(f'hb_controller{self.bot_id}')
        self.get_logger().info(f"{self.bot_id} id controller start")

        # self.create_timer(0.1, self.timerCb)
        
        self.rear_wheel_publisher = self.create_publisher(Wrench,
                                                          f"/hb_bot_{self.bot_id}/rear_wheel_force",
                                                          10)
        self.left_wheel_publisher = self.create_publisher(Wrench,
                                                          f"/hb_bot_{self.bot_id}/left_wheel_force",
                                                          10)        
        self.right_wheel_publisher = self.create_publisher(Wrench,
                                                          f"/hb_bot_{self.bot_id}/right_wheel_force",
                                                          10)
        self.cmd_vel_publisher = self.create_publisher(Twist,
                                                       f"/cmd_vel/bot{self.bot_id}", 10)
        
        self.test = self.create_publisher(Int32, '/Integer', 10)
        self.index = 0
        self.steps = 0
        self.totalSteps = 50

        if(bot_id == 1):
            # self.create_timer(0.1, self.doSquare)
            self.create_timer(0.1, self.doTriangle)
        elif(bot_id == 2):
            self.create_timer(0.1, self.doTriangle)
        elif(bot_id == 3):
            self.create_timer(0.1, self.doCircle)

    def map(self, value, leftMin, leftMax, rightMin, rightMax):
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return 180 - (rightMin + (valueScaled * rightSpan))

    def publish_force_vectors(self, force):
        '''
        Purpose:
        ---
        Publishes the forces to appropriate wheels, i.e. [rear, left, right]

        Input Arguments:
        ---
        self:HBController
        force:numpy.ndarray in the format [rear, left, right]

        Returns:
        ---
        None

        Example call:
        ---
        hb_controller.publish_force_vectors(force)
        '''
        mult = 50.0
        self.get_logger().info("publish force" + str(self.bot_id) + str(force))
        force_rear = Wrench()
        force_left = Wrench()
        force_right = Wrench()

        force_rear.force.y = self.map(force[0], -1.0, 1.0, 0.0, 180.0)
        force_left.force.y = self.map(force[1], -1.0, 1.0, 0.0, 180.0)
        force_right.force.y = self.map(force[2], -1.0, 1.0, 0.0, 180.0)
        
        cmd_vel = Twist()
        cmd_vel.linear.x = force[0]
        msg = Int32()
        msg.data = 45
        self.test.publish(msg)
        self.rear_wheel_publisher.publish(force_rear)
        self.left_wheel_publisher.publish(force_left)
        self.right_wheel_publisher.publish(force_right)
        self.cmd_vel_publisher.publish(cmd_vel)
    
    def doSquare(self):
                    #rear, left, right
        moveRight = [1.0, -0.5, -0.5] 
        moveUp = [0.0, -math.sin(math.pi/3), math.sin(math.pi/3)]
        moveLeft = [-1.0, 0.5, 0.5] 
        moveDown = [0.0, math.sin(math.pi/3), -math.sin(math.pi/3)]
        moveStop = [0.0, 0.0, 0.0]

        moves = [moveRight, moveUp, moveLeft, moveDown, moveStop]

        move = moves[self.index]
        self.steps += 1
        if(self.steps > self.totalSteps):
            self.publish_force_vectors(move)
            self.index += 1
            self.steps = 0
        
        if(self.index >= len(moves)):
            self.index = 0
    
    def doTriangle(self):
                    #rear, left, right
        moveRight = [1.0, -0.5, -0.5] 
        moveUp = [-0.5, -0.5, 1.0]
        moveDown = [-0.5, 1.0, -0.5]
        moveStop = [0.0, 0.0, 0.0]

        moves = [moveRight, moveUp, moveDown, moveStop]

        move = moves[self.index]
        self.steps += 1
        if(self.steps > self.totalSteps):
            self.publish_force_vectors(move)
            self.index += 1
            self.steps = 0
        
        if(self.index >= len(moves)):
            self.index = 0
    
    def doCircle(self):
                    #rear, left, right
        moveRight = [1.0, -0.5, -0.5] 
        moveUp = [-0.5, 1.0, -0.5]
        moveDown = [-0.5, -0.5, 1.0]
        moveCircle = [1.0, 0.0, 0.0]
        moveStop = [0.0, 0.0, 0.0]

        moves = [moveRight, moveUp, moveDown, moveStop]

        move = moves[self.index]
        self.steps += 1
        if(self.steps >=self.totalSteps):
            self.publish_force_vectors(moveCircle)
            self.index += 1
            self.steps = 0
        
        if(self.index >= len(moves)):
            self.index = 0

def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()

    hb_controller_1 = HBController(bot_id=1)
    # hb_controller_2 = HBController(bot_id=2)
    # hb_controller_3 = HBController(bot_id=3)


    executor.add_node(hb_controller_1)
    # executor.add_node(hb_controller_2)
    # executor.add_node(hb_controller_3)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        hb_controller_1.destroy_node()
        # hb_controller_2.destroy_node()
        # hb_controller_3.destroy_node()
        rclpy.shutdown()
       
    # # Main loop
    # while rclpy.ok():
    #     # Spin once to process callbacks
    #     rclpy.spin_once(hb_controller)
    # # Destroy the node and shut down ROS
    # hb_controller.destroy_node()
    # rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()