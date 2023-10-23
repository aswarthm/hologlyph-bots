#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2A of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''


# Team ID:		[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		feedback.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal     
from geometry_msgs.msg import Pose2D        

# You can add more if required
##############################################################


# Initialize Global variables


################# ADD UTILITY FUNCTIONS HERE #################

##############################################################


# Define the HBController class, which is a ROS node
class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')
        self.get_logger().info("controller start")
        
        # Initialze Publisher and Subscriber
        # NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	    #	Use the below given topics to generate motion for the robot.
	    #   /hb_bot_1/left_wheel_force,
	    #   /hb_bot_1/right_wheel_force,
	    #   /hb_bot_1/left_wheel_force
        self.subscription = self.create_subscription(Pose2D, 
                                                    '/detectedAruco',
                                                    self.odometryCb,
                                                    10)
        
        self.left_wheel_publisher = self.create_publisher(Wrench,
                                                          "/hb_bot_1/left_wheel_force",
                                                          10)
        
        self.right_wheel_publisher = self.create_publisher(Wrench,
                                                          "/hb_bot_1/right_wheel_force",
                                                          10)
        
        self.rear_wheel_publisher = self.create_publisher(Wrench,
                                                          "/hb_bot_1/rear_wheel_force",
                                                          10)




        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

        self.hb_x = 0.0
        self.hb_y = 0.0
        self.hb_theta = 0.0

        self.kp = 1.2

        self.left_force = 0.0
        self.right_force = 0.0
        self.rear_force = 0.0



        # client for the "next_goal" service
        self.cli = self.create_client(NextGoal, 'next_goal')      
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = NextGoal.Request() 
        self.index = 0

    def odometryCb(self, msg):
        # self.get_logger().info(str(msg))

        self.hb_x = msg.x
        self.hb_y = msg.y
        self.hb_theta = msg.theta


    # Method to create a request to the "next_goal" service
    def send_request(self, request_goal):
        self.req.request_goal = request_goal
        self.future = self.cli.call_async(self.req)
        

    def inverse_kinematics():
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #	-> Use the target velocity you calculated for the robot in previous task, and
        #	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        #	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
        ############################################
        pass

    def publish_force_vectors(self):
        force_left = Wrench()
        force_left.force.y = 0.0
        self.left_wheel_publisher.publish(force_left)
        self.right_wheel_publisher.publish(force_left)
        self.rear_wheel_publisher.publish(force_left)


def main(args=None):
    rclpy.init(args=args)
    
    # Create an instance of the HBController class
    hb_controller = HBController()
   
    # Send an initial request with the index from ebot_controller.index
    hb_controller.send_request(hb_controller.index)
    
    # Main loop
    while rclpy.ok():
        

        # Check if the service call is done
        if hb_controller.future.done():
            try:
                # response from the service call
                response = hb_controller.future.result()
            except Exception as e:
                hb_controller.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                #########           GOAL POSE             #########
                x_goal      = response.x_goal
                y_goal      = response.y_goal
                theta_goal  = response.theta_goal
                hb_controller.flag = response.end_of_list
                ####################################################

                hb_controller.get_logger().info(f'{x_goal} {y_goal} {theta_goal}')
                
                # Calculate Error from feedback

                error_x = x_goal - hb_controller.hb_x
                error_y = y_goal - hb_controller.hb_y
                error_theta = theta_goal - hb_controller.hb_theta

                # Change the frame by using Rotation Matrix (If you find it required)

                # Calculate the required velocity of bot for the next iteration(s)
                
                # Find the required force vectors for individual wheels from it.(Inverse Kinematics)

                # Apply appropriate force vectors
                hb_controller.publish_force_vectors()

                # Modify the condition to Switch to Next goal (given position in pixels instead of meters)
                        
                ############     DO NOT MODIFY THIS       #########
                hb_controller.index += 1
                if hb_controller.flag == 1 :
                    hb_controller.index = 0
                hb_controller.send_request(hb_controller.index)
                ####################################################

        # Spin once to process callbacks
        rclpy.spin_once(hb_controller)
    
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
