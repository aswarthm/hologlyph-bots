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
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.msg import Goal             

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D    
import numpy as np  



# bot_id = 2

class HBController(Node):
    def __init__(self, bot_id):
        self.bot_id = bot_id
        super().__init__(f'hb_controller{self.bot_id}')
        ##disable#self.get_logger().info(f"{self.bot_id} id controller start")

        self.create_timer(0.1, self.timerCb)

        self.goalsReceived = False        
        self.index = 0
        self.flag = 0
        self.locationReceived = False

        

        # Initialise the required variables
        self.bot_x_goals = []
        self.bot_y_goals = []
        self.bot_theta_goal = 0.0

        # Initialze Publisher and Subscriber
        # NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	    #	Use the below given topics to generate motion for the robot.
	    #   /hb_bot_1/left_wheel_force,
	    #   /hb_bot_1/right_wheel_force,
	    #   /hb_bot_1/left_wheel_force

        #Similar to this you can create subscribers for hb_bot_2 and hb_bot_3
        self.subscription_goal = self.create_subscription(Goal,  
                                                     f'hb_bot_{self.bot_id}/goal',  
                                                     self.goalCallBack,  # Callback function to handle received messages
                                                     10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        )  
        self.subscription_aruco = self.create_subscription(Pose2D, 
                                                    f'/detected_aruco_{self.bot_id}',
                                                    self.arucoCb,
                                                    10)
        
        self.rear_wheel_publisher = self.create_publisher(Wrench,
                                                          f"/hb_bot_{self.bot_id}/rear_wheel_force",
                                                          10)
        self.left_wheel_publisher = self.create_publisher(Wrench,
                                                          f"/hb_bot_{self.bot_id}/left_wheel_force",
                                                          10)        
        self.right_wheel_publisher = self.create_publisher(Wrench,
                                                          f"/hb_bot_{self.bot_id}/right_wheel_force",
                                                          10)


        self.hb_x = 250.0
        self.hb_y = 250.0
        self.hb_theta = 0.0

        self.k_mult = 40.0
        self.kp = 0.1*self.k_mult #1.5 # 5.5 gives 78
        self.ka = 0.4*self.k_mult #2.8 #1.8

        self.linear_tolerance = 4.5 # linear tolerance
        self.angular_tolerance = math.radians(8) # degree tolerance

        self.left_force = 0.0
        self.right_force = 0.0
        self.rear_force = 0.0

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

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
        # ##disable#self.get_logger().info(str(msg))
        if(self.locationReceived == False):
            self.locationReceived = True

        self.hb_x = msg.x
        self.hb_y = msg.y
        self.hb_theta = msg.theta

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
                                [1.0,1.0,0.0],
                                [1.0,-math.cos(math.pi/3.0),-math.sin(math.pi/3.0)],
                                [1.0,-math.cos(math.pi/3.0),math.sin(math.pi/3.0)]
                            ])
        # velocity is in the format [theta, x, y]
        # force is in the format [rear_wheel, left_wheel, right_wheel]
        force = np.dot(ik_matrix, velocity).flatten()

        return force

    def goalCallBack(self, msg):
        if(self.goalsReceived == False):
            self.bot_x_goals = msg.x
            self.bot_y_goals = msg.y
            self.bot_theta_goal = msg.theta

            self.goalsReceived = True

            time.sleep(self.bot_id*3) #add some delay before starting to prevent collision

            # for i in range(5*self.bot_id):
            #     time.sleep(0.4)
    
    def get_goal(self):

        goal_x = self.bot_x_goals[self.index]
        goal_y = self.bot_y_goals[self.index]
        goal_theta = self.bot_theta_goal

        end_of_list = int((1 + self.index) >= len(self.bot_x_goals))

        return [goal_x, goal_y, goal_theta, end_of_list]

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
        ##disable#self.get_logger().info(str(error_linear))

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

    def normalize_velocity(self, velocity):
        max_vel = 60.0

        velocity[1] = max(-max_vel, min(velocity[1], max_vel))
        velocity[2] = max(-max_vel, min(velocity[2], max_vel))
        return velocity
    
    def timerCb(self):
        # Check if the goals have been received
        if self.goalsReceived == True and self.locationReceived == True:
            try:
                # response from the service call
                response = self.get_goal()
            except Exception as e:
                ##disable###disable#self.get_logger().info('Goal call failed %r' % (e,))
                pass
            else:
                #########           GOAL POSE             #########
                x_goal      = response[0] #+ 250.0
                y_goal      = response[1] #+ 250.0
                theta_goal  = response[2]
                self.flag = response[3]
                ####################################################

                ##disable###disable#self.get_logger().info(f'{x_goal} {y_goal} {math.degrees(theta_goal)}')
                ##disable###disable#self.get_logger().info(f'cur {self.hb_x} {self.hb_y} {math.degrees(self.hb_theta)}')
                
                # Calculate Error from feedback
                error_x = x_goal - self.hb_x
                error_y = y_goal - self.hb_y
                error_theta = theta_goal - self.hb_theta

                # Change the frame by using Rotation Matrix (If you find it required)
                frame = np.array([error_theta, error_x, error_y])

                rot_matrix = np.array([
                                        [1, 0, 0],
                                        [0, math.cos(self.hb_theta), -math.sin(self.hb_theta)],
                                        [0, -math.sin(self.hb_theta), -math.cos(self.hb_theta)],
                                      ])
                global_error = np.dot(frame, rot_matrix).flatten()
                # ##disable###disable#self.get_logger().info(str(global_error))
            
                # Calculate the required velocity of bot for the next iteration(s)
                k = np.array([self.ka, self.kp, self.kp])
                velocity = np.multiply(global_error, k)
                velocity = self.normalize_velocity(velocity)
                ##disable###disable#self.get_logger().info(str(velocity))
                
                # Find the required force vectors for individual wheels from it.(Inverse Kinematics)
                force = self.inverse_kinematics(velocity)

                # Apply appropriate force vectors
                self.publish_force_vectors(force)

                # Modify the condition to Switch to Next goal (given position in pixels instead of meters)
                if(self.goal_reached(frame)):
                    # self.stop_bot()

                    ############     DO NOT MODIFY THIS       #########
                    self.index += 1
                    if self.flag == 1 :
                        self.index = 0
                    ####################################################



def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()

    hb_controller_1 = HBController(bot_id=1)
    hb_controller_2 = HBController(bot_id=2)
    hb_controller_3 = HBController(bot_id=3)


    executor.add_node(hb_controller_1)
    executor.add_node(hb_controller_2)
    executor.add_node(hb_controller_3)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        hb_controller_1.destroy_node()
        hb_controller_2.destroy_node()
        hb_controller_3.destroy_node()
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