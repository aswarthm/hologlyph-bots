#!/usr/bin/env python3
########################################################################################################################
########################################## eYRC 23-24 Hologlyph Bots Task 1B ###########################################
# Team ID: 1036
# Team Leader Name: M Aswartha Reddy
# Team Members Name: M Aswartha Reddy, D K Bharath Reddy, Pulkit Dhamija, Sangeeta Prasad
# College: R. V. College of Engineering
########################################################################################################################

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
from math import sin, cos
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal

class HBTask1BController(Node):

    def __init__(self):
        '''
        Purpose:
        ---
        Called when the controller is initialized

        Input Arguments:
        ---
        self:HBTask1BController

        Returns:
        ---
        None

        Example call:
        ---
        -
        '''
        super().__init__('hb_task1b_controller')
        
        # Initialze Publisher and Subscriber
        # We'll leave this for you to figure out the syntax for
        # initialising publisher and subscriber of cmd_vel and odom respectively

        self.subscription = self.create_subscription(Odometry, 
                                                        '/odom',
                                                        self.odometryCb,
                                                        10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Global position of bot
        self.hb_x = 0
        self.hb_y = 0
        self.hb_theta = 0


        # Declare a Twist message
        self.vel = Twist()
        # Initialise the required variables to 0

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)
        # Initialise variables that may be needed for the control loop
        # For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
        # and also Kp values for the P Controller

        # New velocity/pose of the bot, to be calculated
        self.x_d = 0.0
        self.y_d = 0.0
        self.theta_d = 0.0

        self.Kp = 1.5 # p value
        self.linear_tolerance = 0.2 # linear tolerance
        self.angular_tolerance = math.pi*(10/180) # degree tolerance

        # Max and min values to clamp velocity
        self.max_linear_vel = 999.0
        self.min_linear_vel = -999.0
        self.max_angular_vel = 2*math.pi
        self.min_angular_vel = -2*math.pi


        # client for the "next_goal" service
        self.cli = self.create_client(NextGoal, 'next_goal')      
        self.req = NextGoal.Request() 
        self.index = 0

    def odometryCb(self, odom: Odometry):
        '''
        Purpose:
        ---
        Callback function when robot's odometry is updated, updates local variables with current x,y,theta
        The odometry contains a quaternion which is converted to x, y, theta usi

        Input Arguments:
        ---
        self:HBTask1BController
        odom:Odometry updated odometry of robot

        Returns:
        ---
        None

        Example call:
        ---
        -
        '''
        # global hb_x, hb_y, hb_theta
        # Write your code to take the msg and update the three variables

        (self.hb_x, self.hb_y, self.hb_theta) = self.getPose(odom)
        msg =  str(self.hb_x) + "\t" + str(self.hb_x) + "\t" + str(self.hb_theta)
        # self.get_logger().info(msg)
    
    def getPose(self, odom: Odometry):
        '''
        Purpose:
        ---
        The odometry contains a quaternion which is converted to x, y, theta using the euler_from_quaternion function from the transformations library

        Input Arguments:
        ---
        self:HBTask1BController
        odom:Odometry updated odometry of robot

        Returns:
        ---
        (x:float, y:float, theta:float)

        Example call:
        ---
        self.getPose(odom)
        '''
        orientation_q = odom.pose.pose.orientation
        position = odom.pose.pose.position
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return position.x, position.y, yaw

    
    def send_request(self, index):
        '''
        Purpose:
        ---
        Helper function to make requests to the service node

        Input Arguments:
        ---
        self:HBTask1BController
        index:Get the coordinates of 'index'

        Returns:
        ---
        None

        Example call:
        ---
        ebot_controller.send_request(ebot_controller.index) 
        '''
        self.req.request_goal = index
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the EbotController class
    ebot_controller = HBTask1BController()

    # Send an initial request with the index from ebot_controller.index
    ebot_controller.send_request(ebot_controller.index) 

    # Main loop
    while rclpy.ok():

        # Check if the service call is done
        if ebot_controller.future.done():
            try:
                # response from the service call
                response = ebot_controller.future.result()
            except Exception as e:
                ebot_controller.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                #########           GOAL POSE             #########
                x_goal      = response.x_goal
                y_goal      = response.y_goal
                theta_goal  = response.theta_goal
                ebot_controller.flag = response.end_of_list
                ####################################################
                msg = f"x = {x_goal} y = {y_goal} theta = {theta_goal}"
                ebot_controller.get_logger().info(msg)
                # print(x_goal, y_goal, theta_goal)

                # Find error (in x, y and theta) in global frame
                # the /odom topic is giving pose of the robot in global frame
                # the desired pose is declared above and defined by you in global frame
                # therefore calculate error in global frame

                global_error_x = x_goal - ebot_controller.hb_x
                global_error_y = y_goal - ebot_controller.hb_y
                global_error_theta = theta_goal - ebot_controller.hb_theta

                # (Calculate error in body frame)
                # But for Controller outputs robot velocity in robot_body frame, 
                # i.e. velocity are define is in x, y of the robot frame, 
                # Notice: the direction of z axis says the same in global and body frame
                # therefore the errors will have have to be calculated in body frame.
                # 
                # This is probably the crux of Task 1, figure this out and rest should be fine.
                body_error_x =  global_error_x*cos(ebot_controller.hb_theta) + global_error_y*sin(ebot_controller.hb_theta)
                body_error_y = -global_error_x*cos(ebot_controller.hb_theta) + global_error_y*sin(ebot_controller.hb_theta)
                body_error_theta = global_error_theta

                # if(abs(body_error_x) < tolerance and abs(body_error_y) < tolerance and abs(body_error_theta) < tolerance):
                #     goal_reached = True
                # Finally implement a P controller 
                # to react to the error with velocities in x, y and theta.
                msg = Twist()
                msg.linear.x = ebot_controller.Kp * (global_error_x)
                msg.linear.y = ebot_controller.Kp * (global_error_y)
                msg.angular.z = ebot_controller.Kp * (global_error_theta)

                # Safety Check
                # make sure the velocities are within a range.
                # for now since we are in a simulator and we are not dealing with actual physical limits on the system 
                # we may get away with skipping this step. But it will be very necessary in the long run.

                # Clamp velocity to min and max values
                msg.linear.x = max(ebot_controller.min_linear_vel, min(ebot_controller.max_linear_vel, msg.linear.x))
                msg.linear.y = max(ebot_controller.min_linear_vel, min(ebot_controller.max_linear_vel, msg.linear.y))
                msg.angular.z = max(ebot_controller.min_angular_vel, min(ebot_controller.max_angular_vel, msg.angular.z))

                ebot_controller.publisher_.publish(msg)

                if(abs(body_error_x) < ebot_controller.linear_tolerance and abs(body_error_y) < ebot_controller.linear_tolerance and abs(body_error_theta) < ebot_controller.angular_tolerance):
                    #If Condition is up to you
                    msg = Twist()
                    msg.linear.x = 0.0
                    msg.linear.y = 0.0
                    msg.angular.z = 0.0
                    ebot_controller.publisher_.publish(msg)
                    
                    ############     DO NOT MODIFY THIS       #########
                    ebot_controller.index += 1
                    if ebot_controller.flag == 1 :
                        ebot_controller.index = 0
                    ebot_controller.send_request(ebot_controller.index)
                    ####################################################

        # Spin once to process callbacks
        rclpy.spin_once(ebot_controller)
    
    # Destroy the node and shut down ROS
    ebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
