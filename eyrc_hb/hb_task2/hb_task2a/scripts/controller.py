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
import numpy as np    

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
        
        self.rear_wheel_publisher = self.create_publisher(Wrench,
                                                          "/hb_bot_1/rear_wheel_force",
                                                          10)
        self.left_wheel_publisher = self.create_publisher(Wrench,
                                                          "/hb_bot_1/left_wheel_force",
                                                          10)        
        self.right_wheel_publisher = self.create_publisher(Wrench,
                                                          "/hb_bot_1/right_wheel_force",
                                                          10)
        
        self.cmd_vel_publisher = self.create_publisher(Twist,
                                                          "/cmd_vel",
                                                          10)




        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

        self.hb_x = 0.0
        self.hb_y = 0.0
        self.hb_theta = 0.0

        self.kp = 3.5 #1.5
        self.ka = 2.8 #1.8

        self.linear_tolerance = 5 # linear tolerance
        self.angular_tolerance = math.radians(12) # degree tolerance

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
        

    def inverse_kinematics(self, velocity):
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #	-> Use the target velocity you calculated for the robot in previous task, and
        #	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        #	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
        ############################################

        ik_matrix = np.array([
                                [-1,1,0],
                                [-1,-(math.cos(math.pi/3)),-math.sin(math.pi/3)],
                                [-1,-(math.cos(math.pi/3)),(math.sin(math.pi/3))]
                            ])
        # velocity is in the format [theta, x, y]
        # force is in the format [rear_wheel, left_wheel, right_wheel]
        force = np.dot(ik_matrix, velocity).flatten()

        return force

    def publish_force_vectors(self, force):
        force_rear = Wrench()
        force_left = Wrench()
        force_right = Wrench()

        force_rear.force.y = force[0]
        force_left.force.y = force[1]
        force_right.force.y = force[2]

        self.rear_wheel_publisher.publish(force_rear)
        self.left_wheel_publisher.publish(force_left)
        self.right_wheel_publisher.publish(force_right)
    
    def publish_cmd_vel(self, velocity):
        msg = Twist()
        msg.angular.z = velocity[0]
        msg.linear.x = velocity[1]
        msg.linear.y = velocity[2]
        self.cmd_vel_publisher.publish(msg)
    
    def goal_reached(self, frame):
        error_theta = frame[0]

        error_linear = math.sqrt(math.pow(frame[1], 2) + math.pow(frame[2], 2))
        self.get_logger().info(str(error_linear))

        if(abs(error_theta) < self.angular_tolerance and abs(error_linear) < self.linear_tolerance):
            return True
        
        return False

    def stop_bot(self):
        self.publish_force_vectors(np.array([0.0, 0.0, 0.0]))



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

                hb_controller.get_logger().info(f'{x_goal} {y_goal} {math.degrees(theta_goal)}')
                hb_controller.get_logger().info(f'cur {hb_controller.hb_x} {hb_controller.hb_y} {math.degrees(hb_controller.hb_theta)}')
                
                # Calculate Error from feedback

                error_x = x_goal - hb_controller.hb_x
                error_y = y_goal - hb_controller.hb_y
                error_theta = theta_goal - hb_controller.hb_theta

                # Change the frame by using Rotation Matrix (If you find it required)
                frame = np.array([-error_theta, error_x, error_y])

                # frame = np.array([
                #                     [error_x, error_y, error_theta]
                #                  ])
                # rot_matrix = np.array([
                #                         [math.cos(hb_controller.hb_theta), -math.sin(hb_controller.hb_theta), 0],
                #                         [math.sin(hb_controller.hb_theta), math.cos(hb_controller.hb_theta), 0],
                #                         [0, 0, 1],
                #                       ])
                # body_error_x =  error_x*math.cos(hb_controller.hb_theta) + error_y*math.sin(hb_controller.hb_theta)
                # body_error_y = (-error_x*math.sin(hb_controller.hb_theta) + error_y*math.cos(hb_controller.hb_theta))
                # body_error_theta = error_theta

                

                rot_matrix = np.array([
                                        [1, 0, 0],
                                        [0, math.cos(hb_controller.hb_theta), -math.sin(hb_controller.hb_theta)],
                                        [0, math.sin(hb_controller.hb_theta), math.cos(hb_controller.hb_theta)],
                                      ])
                global_error = np.dot(frame, rot_matrix).flatten()
            
                # Calculate the required velocity of bot for the next iteration(s)
                
                k = np.array([hb_controller.ka, hb_controller.kp, hb_controller.kp])
                velocity = np.multiply(global_error, k)
                hb_controller.get_logger().info(str(velocity))
                # hb_controller.publish_cmd_vel(velocity)
                
                # Find the required force vectors for individual wheels from it.(Inverse Kinematics)
                
                force = hb_controller.inverse_kinematics(velocity)
                # force = hb_controller.inverse_kinematics(np.array([0, 0, 10]))
                # hb_controller.get_logger().info(str(force))

                # Apply appropriate force vectors
                hb_controller.publish_force_vectors(force)

                # Modify the condition to Switch to Next goal (given position in pixels instead of meters)
                
                if(hb_controller.goal_reached(frame)):
                    # hb_controller.stop_bot()

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
