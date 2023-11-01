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


# Team ID:		    hb_1036
# Author List:		[ M Aswartha Reddy, D K Bharath Reddy, Pulkit Dhamija, Sangeeta Prasad ]
# Filename:		    controller.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		
#                   Subs: [/detectedAruco]
#                   Pubs: [ /hb_bot_1/rear_wheel_force, /hb_bot_1/left_wheel_force, /hb_bot_1/right_wheel_force ]


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
        '''
            Purpose:
            ---
            Called when the HBController Node is initialised

            Input Arguments:
            ---
            self:HBController

            Returns:
            ---
            None

            Example call:
            ---
            -
        '''
        super().__init__('hb_controller')
        self.get_logger().info("controller start")
        
        # Initialze Publisher and Subscriber
        # NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	    #	Use the below given topics to generate motion for the robot.
	    #   /hb_bot_1/left_wheel_force,
	    #   /hb_bot_1/right_wheel_force,
	    #   /hb_bot_1/left_wheel_force
        self.subscription = self.create_subscription(Pose2D, 
                                                    '/detected_aruco',
                                                    self.arucoCb,
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

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

        self.hb_x = 250.0
        self.hb_y = 250.0
        self.hb_theta = 0.0

        self.kp = 4.5 #1.5 # 5.5 gives 78
        self.ka = 18.0 #2.8 #1.8

        self.linear_tolerance = 3.5 # linear tolerance
        self.angular_tolerance = math.radians(8) # degree tolerance

        self.left_force = 0.0
        self.right_force = 0.0
        self.rear_force = 0.0



        # client for the "next_goal" service
        self.cli = self.create_client(NextGoal, 'next_goal')      
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = NextGoal.Request() 
        self.index = 0

    def arucoCb(self, msg):
        '''
        Purpose:
        ---
        Callback function when robot's position changes. Updates local variables with current x,y,theta

        Input Arguments:
        ---
        self:HBController
        odom:Pose2D updated odometry of robot

        Returns:
        ---
        None

        Example call:
        ---
        -
        '''
        # self.get_logger().info(str(msg))

        self.hb_x = msg.x
        self.hb_y = msg.y
        self.hb_theta = msg.theta


    # Method to create a request to the "next_goal" service
    def send_request(self, request_goal):
        '''
        Purpose:
        ---
        Helper function to make a request to the service node to get the next goal

        Input Arguments:
        ---
        self:HBController
        request_goal:Get the coordinates of "request_goal's" index

        Returns:
        ---
        None

        Example call:
        ---
        hb_controller.send_request(hb_controller.index)
        '''
        self.req.request_goal = request_goal
        self.future = self.cli.call_async(self.req)
        

    def inverse_kinematics(self, velocity):
        '''
        Purpose:
        ---
        Calculate inverse kinematics for the desired velocity

        Input Arguments:
        ---
        self:HBController
        velocity:numpy.ndarray in the format [theta, x, y]

        Returns:
        ---
        force:numpy.ndarray in the format [rear_wheel, left_wheel, right_wheel]

        Example call:
        ---
        force = hb_controller.inverse_kinematics(velocity)
        '''
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #	-> Use the target velocity you calculated for the robot in previous task, and
        #	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        #	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
        ############################################

        ik_matrix = np.array([
                                [-1.0,1.0,0.0],
                                [-1.0,-math.cos(math.pi/3.0),-math.sin(math.pi/3.0)],
                                [-1.0,-math.cos(math.pi/3.0),math.sin(math.pi/3.0)]
                            ])
        # velocity is in the format [theta, x, y]
        # force is in the format [rear_wheel, left_wheel, right_wheel]
        force = np.dot(ik_matrix, velocity).flatten()

        return force

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
        force_rear = Wrench()
        force_left = Wrench()
        force_right = Wrench()

        force_rear.force.y = force[0]
        force_left.force.y = force[1]
        force_right.force.y = force[2]

        self.rear_wheel_publisher.publish(force_rear)
        self.left_wheel_publisher.publish(force_left)
        self.right_wheel_publisher.publish(force_right)
    
    def goal_reached(self, frame):
        '''
        Purpose:
        ---
        Checks if the bot is within acceptable limits, which tells if the bot has reached its goal

        Input Arguments:
        ---
        self:HBController
        frame:numpy.ndarray in the format [error_theta, error_x, error_y]

        Returns:
        ---
        True if bot has reached goal
        False otherwise

        Example call:
        ---
        if(hb_controller.goal_reached(frame)):
            # do something
        '''
        error_theta = frame[0]

        error_linear = math.sqrt(math.pow(frame[1], 2) + math.pow(frame[2], 2))
        self.get_logger().info(str(error_linear))

        if(abs(error_theta) < self.angular_tolerance and abs(error_linear) < self.linear_tolerance):
        # if(abs(error_theta) < self.angular_tolerance and abs(frame[1]) < self.linear_tolerance and abs(frame[2]) < self.linear_tolerance):
            return True
        
        return False

    def stop_bot(self):
        '''
        Purpose:
        ---
        Stops the bot after reaching goal, or in case of an emergency

        Input Arguments:
        ---
        self:HBController

        Returns:
        ---
        None

        Example call:
        ---
        hb_controller.stop_bot()
        '''
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
                x_goal      = response.x_goal + 250.0
                y_goal      = response.y_goal + 250.0
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
                
                # Find the required force vectors for individual wheels from it.(Inverse Kinematics)
                force = hb_controller.inverse_kinematics(velocity)

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
