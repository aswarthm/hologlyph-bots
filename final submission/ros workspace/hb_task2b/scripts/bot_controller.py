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
# Functions:        __init__, goalCallBack, arucoCb, pauseCallBack, inverse_kinematics, goalCallBack, convert_to_2d, get_goal, map, publish_force_vectors, goal_reached, stop_bot, pen_position, normalize_velocity, allBotsHome, timerCb
# Nodes:		
#                   Subs: [ 'hb_bot_{self.bot_id}/goal', '/detected_aruco_{self.bot_id}' ]
#                   Pubs: [ "/hb_bot_{self.bot_id}/rear_wheel_force", "/hb_bot_{self.bot_id}/left_wheel_force", "/hb_bot_{self.bot_id}/right_wheel_force" ]


################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.msg import Goal, Float1D    

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D    
import numpy as np  
from std_msgs.msg import Bool
from std_srvs.srv import Empty

from geometry_msgs.msg import Vector3


isSimulator = False
istask5b = False



# bot_id = 2

bot_ids = [1, 2, 3]

bot_done = { # if all are 1 then end run
    1: 0,
    2: 0,
    3: 0
}

bot_is_home = {
    1: 0,
    2: 0,
    3: 0
}

bot_home_flag = 0

class HBController(Node):
    '''
        Function Name: __init__

        Input : self--> HBController,bot_id-->id of the bot

        Output: None

        Logic: initialises each bot with its id

        Example call: hb_controller.__init__(2)
    '''
    def __init__(self, bot_id):
        self.bot_id = bot_id
        super().__init__(f'hb_controller{self.bot_id}')
        ##disable#self.get_logger().info(f"{self.bot_id} id controller start")

        self.create_timer(0.1, self.timerCb)

        self.goalsReceived = False        
        self.index = 0
        self.contour_index = 0
        self.flag = 0
        self.locationReceived = False
        self.pause = True
        self.end_of_run = False

        

        # Initialise the required variables
        self.bot_x_goals = []
        self.bot_y_goals = []
        self.bot_theta_goal = 0.0

        self.bot_home = {
            1: [397.0, 455.0],
            2: [243.0, 448.0],
            3: [93.0, 440.0]
        }


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
                                                    f"pen{self.bot_id}_pose",
                                                    self.arucoCb,
                                                    10)
        
        self.subscription_pause = self.create_subscription(Bool,
                                                           f"hb_bot_{self.bot_id}/pause",
                                                           self.pauseCallBack,
                                                           10)
        

        if(isSimulator):
            self.rear_wheel_publisher = self.create_publisher(Wrench,
                                                            f"/hb_bot_{self.bot_id}/rear_wheel_force",
                                                            10)
            self.left_wheel_publisher = self.create_publisher(Wrench,
                                                            f"/hb_bot_{self.bot_id}/left_wheel_force",
                                                            10)        
            self.right_wheel_publisher = self.create_publisher(Wrench,
                                                            f"/hb_bot_{self.bot_id}/right_wheel_force",
                                                            10)
            
        else:
            self.cmd_vel_publisher = self.create_publisher(Vector3,
                                                            f"/hb_bot_{self.bot_id}/cmd_vell",
                                                            10)


        self.pen_down_publisher = self.create_publisher(Bool, 
                                                        f"/pen{self.bot_id}_down",
                                                        10)
        



        self.hb_x = 250.0
        self.hb_y = 250.0
        self.hb_theta = 0.0

        if(isSimulator):
            self.k_mult = 40.0
            self.kp = 0.1*self.k_mult #1.5 # 5.5 gives 78
            self.ka = 1.8*self.k_mult #2.8 #1.8

            self.linear_tolerance = 10 #4.5 # linear tolerance
            self.angular_tolerance = math.radians(4) # degree tolerance
        else:
            self.k_mult = 60.0 #40.0
            self.kp = 0.1*self.k_mult #1.5 # 5.5 gives 78
            self.ka = 2.8*self.k_mult #2.8 #1.8

            self.linear_tolerance = 10.0 #4.5 # linear tolerance
            self.angular_tolerance = math.radians(8) # degree tolerance

        self.left_force = 0.0
        self.right_force = 0.0
        self.rear_force = 0.0

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

    '''
        Function Name: pauseCallBack

        Input : self--> HBController,msg-->

        Output: None

        Logic: Pauses the bot

        Example call: hb_controller.pauseCallBack(10)
    '''
    def pauseCallBack(self, msg):
        self.pause = msg.data
        # self.get_logger().info(f"{self.bot_id} paused")

        '''
        Function Name: arucoCb

        Input : self--> HBController, msg--> buffer size of messages

        Output: None

        Logic: Callback function when robot's position changes. Updates local variables with current x,y,theta

        Example call: hb_controller.arucoCb(10)
    '''
    def arucoCb(self, msg):
        if(self.bot_id == 2):
            pass
            # self.get_logger().info(str(msg))
        if(self.locationReceived == False):
            self.locationReceived = True

        self.hb_x = msg.x
        self.hb_y = msg.y
        if(istask5b):
            self.hb_theta = msg.theta - math.radians(90)
        else:
            self.hb_theta = msg.theta #- math.radians(90)


        '''
        Function Name: inverse_kinematics

        Input : self--> HBController, velocity--> numpy.ndarray in the format [theta, x, y]

        Output: force:numpy.ndarray in the format [rear_wheel, left_wheel, right_wheel]

        Logic: Calculate inverse kinematics for the desired velocity

        Example call: hb_controller.inverse_kinematics(velocity)
    '''
    def inverse_kinematics(self, velocity):
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

        '''
        Function Name: goalCallBack

        Input : self--> HBController, msg--> Goal list of bot containing x, y and theta

        Output: None

        Logic: Callback function when few goals are received. Updates array with goals to be reached
               Sleeps for a certian amount of time to prevent collision

        Example call: hb_controller.goalCallBack([goal_x, goal_y, goal_theta, end_of_list])
    '''
    def goalCallBack(self, msg):
        if(self.goalsReceived == False):
            self.bot_x_goals = self.convert_to_2d(msg.x)
            self.bot_y_goals = self.convert_to_2d(msg.y)

            # self.bot_x_goals = [x].append(msg.x)
            # self.bot_y_goals = [y].append(msg.y)
            self.bot_theta_goal = msg.theta

            self.goalsReceived = True

            # time.sleep(self.bot_id*3) #add some delay before starting to prevent collision

            # for i in range(5*self.bot_id):
            #     time.sleep(0.4)

    '''
        Function Name: convert_to_2d

        Input : self--> HBController, msg--> Goal list of bot containing x, y and theta

        Output: message in 2d format

        Logic: converts message to 2d format by extracting data from each row

        Example call: hb_controller.convert_to_2d(msg.x))
    '''

    def convert_to_2d(self, msg):
        return [row.data for row in msg]
    
        '''
        Function Name: get_goal

        Input : self--> HBController

        Output: [goal_x, goal_y, goal_theta, end_of_list]

        Logic: Returns the next goal for the bot and if all the goals have been reached i.e. end of list

        Example call: hb_controller.get_goal()
    '''
    def get_goal(self):
        goal_x = self.bot_x_goals[self.contour_index][self.index]
        goal_y = self.bot_y_goals[self.contour_index][self.index]
        goal_theta = self.bot_theta_goal

        end_of_list = self.index >= (len(self.bot_x_goals[self.contour_index]) - 1)

        self.end_of_run = self.contour_index >= (len(self.bot_x_goals) - 1) and end_of_list

        return [goal_x, goal_y, goal_theta, end_of_list]

    '''
        Function Name: map

        Input : self--> HBController,value-->value to be mapped,leftMin-->minimum input value, leftMax-->maximum input value, rightMin-->minimum output value,rightMax-->maximum output value

        Output: 

        Logic: To convert controller values to servo values

        Example call: hb_controller.map(force[0], -100.0, 100.0, 0.0, 180.0)
    '''   
    def map(self, value, leftMin, leftMax, rightMin, rightMax):
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return 180 - (rightMin + (valueScaled * rightSpan))
    
        '''
        Function Name: publish_force_vectors

        Input : self--> HBController, force--> numpy.ndarray in the format [rear, left, right]

        Output: None

        Logic: Publishes the forces to appropriate wheels, i.e. [rear, left, right]

        Example call: hb_controller.publish_force_vectors(force)
    ''' 
    def publish_force_vectors(self, force):

        if(isSimulator):
            #for simulator
            force_rear = Wrench()
            force_left = Wrench()
            force_right = Wrench()

            force_rear.force.y = force[0]
            force_left.force.y = force[1]
            force_right.force.y = force[2]


            self.rear_wheel_publisher.publish(force_rear)
            self.left_wheel_publisher.publish(force_left)
            self.right_wheel_publisher.publish(force_right)
        else:
            #for hardware
            cmd_vel = Vector3()


            cmd_vel.x = self.map(force[0], -100.0, 100.0, 0.0, 180.0)
            cmd_vel.y = self.map(force[1], -100.0, 100.0, 0.0, 180.0)
            cmd_vel.z = self.map(force[2], -100.0, 100.0, 0.0, 180.0)
            
            self.cmd_vel_publisher.publish(cmd_vel)


        if(self.bot_id == 3):
            pass
            # self.get_logger().info(f"{force[0]} {force[1]} {force[2]}")
    
        '''
        Function Name: goal_reached

        Input : self--> HBController, frame--> numpy.ndarray in the format [error_theta, error_x, error_y]

        Output: True if bot has reached goal, False otherwise

        Logic: Checks if the bot is within acceptable limits, which tells if the bot has reached its goal

        Example call: hb_controller.goal_reached(frame)
    ''' 
    def goal_reached(self, frame):
        error_theta = frame[0]

        error_linear = math.sqrt(math.pow(frame[1], 2) + math.pow(frame[2], 2))
        ##disable#self.get_logger().info(str(error_linear))

        if(abs(error_theta) < self.angular_tolerance and abs(error_linear) < self.linear_tolerance):
        # if(abs(error_theta) < self.angular_tolerance and abs(frame[1]) < self.linear_tolerance and abs(frame[2]) < self.linear_tolerance):
            return True
        
        return False
    
        '''
        Function Name: stop_bot

        Input : self--> HBController

        Output: None

        Logic: Stops the bot after reaching goal, or in case of an emergency

        Example call: hb_controller.stop_bot()
    '''
    def stop_bot(self):

        # bot_done[self.bot_id] = 1
        self.publish_force_vectors(np.array([0.0, 0.0, 0.0]))
        # time.sleep(2)
        # self.publish_force_vectors(np.array([0.0, 0.0, 0.0]))
    '''
        Function Name: pen_position

        Input : self--> HBController,pos-->

        Output: None

        Logic: sets the pen to touch the surface when required to draw otherwise lifts the pen up

        Example call: hb_controller.stop_bot()
    '''  
    def pen_position(self, pos):

        msg = Bool()
        msg.data = pos=="DOWN" #true = pendown, false=penup
        self.pen_down_publisher.publish(msg)
        

        '''
        Function Name: normalize_velocity

        Input : self--> HBController, velocity-->velocity to be normalised

        Output: normalised velocity list

        Logic: Makes sure the velocities are within acceptable range to prevent unpredictable behaviour

        Example call: hb_controller.normalize_velocity(velocity)
    ''' 
    def normalize_velocity(self, velocity):
        max_vel = 100.0

        velocity[1] = max(-max_vel, min(velocity[1], max_vel))
        velocity[2] = max(-max_vel, min(velocity[2], max_vel))
        
        return velocity

    
    '''
        Function Name: allBotsHome

        Input : self--> HBController

        Output: True if all bots reach home else False

        Logic: Returns False if any bot is not at home position, otherwise returns True

        Example call: hb_controller.allBotsHome()
    ''' 
    def allBotsHome(self):
        global bot_home_flag

        sum = 0

        for i in bot_ids:
            sum += bot_is_home[i]
        
        return sum/3
        
        if(bot_home_flag == 0):
            bot_home_flag = 1
            # time.sleep((self.bot_id)*3)

        return True

    
        '''
        Function Name: timerCb

        Input : self--> HBController

        Output: None

        Logic: Callback function to run main loop of the controller, simplifies multi threading

        Example call: hb_controller.timerCb()
    '''   
    def timerCb(self):
        global bot_home_flag
        
        if(self.pause == True):
            self.stop_bot()
            return

        # Check if the goals have been received
        if self.goalsReceived == True and self.locationReceived == True:
            try:
                # response from the service call
                response = self.get_goal()
            except Exception as e:
                #self.get_logger().info('Goal call failed %r' % (e,))
                pass
            else:
                #########           GOAL POSE             #########
                x_goal      = response[0] #+ 250.0
                y_goal      = response[1] #+ 250.0
                theta_goal  = response[2]
                self.flag = response[3]
                ####################################################

                # self.get_logger().info(f'{x_goal} {y_goal} {math.degrees(theta_goal)}')
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

                    if(bot_home_flag == 0):
                        if(self.index == 0):
                            bot_is_home[self.bot_id] = 1
                            self.stop_bot()
                            self.pen_position("UP")
                            while(self.allBotsHome() != 1.0):
                                pass
                            else:
                                if(self.bot_id == 3):
                                    time.sleep(8)#12
                                else:
                                    time.sleep(8)
                            self.get_logger().info(f"{self.bot_id}index 0")

                        if(self.index == 1):
                            bot_is_home[self.bot_id] = 2
                            self.stop_bot()
                            self.pen_position("UP")
                            while(self.allBotsHome() != 2.0):
                                pass
                            else:
                                if(self.bot_id == 3.0):
                                    time.sleep(8)#12
                                else:
                                    time.sleep(8)
                            self.get_logger().info(f"{self.bot_id}index 1")
                            bot_home_flag = 1

                    if(self.index == 3):
                        #do pendown
                        self.pen_position("DOWN")
                        self.get_logger().info(f"{self.bot_id}index 2")
                        # self.get_logger().info(f"{self.bot_id} {self.index}")

                    
                    if(self.flag == 1):
                        # self.goalsReceived = False
                        self.stop_bot()
                        #do penup
                        self.pen_position("UP")
                        # self.destroy_node()
                    
                    if(self.end_of_run):
                        bot_done[self.bot_id] = 1
                                                                
                    # if(self.finished_run()):
                    #     request = Empty.Request()
                    #     self.client.call_async(request)

                    ############     DO NOT MODIFY THIS       #########
                    self.index += 1
                    if self.flag == 1 :
                        self.contour_index += 1
                        self.index = 0
                    ####################################################


# class StopService(Node):
#     def __init__(self):
#         super().__init__(f'stop_service_node_start')
#         self.client = self.create_client(Empty, 'Stop_Flag')
#         self.create_timer(0.1, self.timerCb)
    
#     def finished_run(self):

#         for i in bot_ids:
#             if(bot_done[i] == 0):
#                 return False
            
#         return True
    
#     def timerCb(self):
#         if(self.finished_run()):
#             request = Empty.Request()
#             self.client.call_async(request)
#             self.get_logger().info("Finished Run")

#             self.destroy_node()
                        

class StopService(Node):
    '''
        Function Name: __init__

        Input : self-->Stopservice

        Output: None

        Logic: creates a client for Stop_flag service

        Example call: stop_serv.__init__()
    '''
    def __init__(self):
        super().__init__(f'stop_service_node_start')
        self.create_timer(0.1, self.timerCb)

    def blank(self, request, response):
        return response
    '''
        Function Name: finished_run

        Input : self-->Stopservice

        Output: Returns True if all bots are done,else False

        Logic: iterates over the bot_ids and checks the corresponding value in the bot_done dictionary. If any bot is not done (i.e., bot_done[i] == 0), it returns False. If all bots are done, it returns True.

        Example call: self.finished_run()
    '''    
    def finished_run(self):

        for i in bot_ids:
            if(bot_done[i] == 0):
                return False
            
        return True
    '''
        Function Name: timerCb

        Input : self-->Stopservice

        Output: None

        Logic: callback method for the timer. If finished_run() returns True, it creates a request for the ‘Stop_Flag’ service, sends the request asynchronously

        Example call: self.timerCb()
    '''
    def timerCb(self):
        if(self.finished_run()):
            self.srv = self.create_service(Empty, 'Stop_Flag', self.blank)
            self.get_logger().info("Finished Run")

            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()

    hb_controller_1 = HBController(bot_id=1)
    hb_controller_2 = HBController(bot_id=2)
    hb_controller_3 = HBController(bot_id=3)


    executor.add_node(hb_controller_1)
    executor.add_node(hb_controller_2)
    executor.add_node(hb_controller_3)


    stop_serv = StopService()

    executor.add_node(stop_serv)

    try:
        executor.spin()

    finally:
        executor.shutdown()

        hb_controller_1.destroy_node()
        hb_controller_2.destroy_node()
        hb_controller_3.destroy_node()

        stop_serv.destroy_node()
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