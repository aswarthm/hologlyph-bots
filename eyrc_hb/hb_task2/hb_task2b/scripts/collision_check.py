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
# Filename:		    collision_check.py
# Functions:		[__init__(self),publishBotPauseState(self, bot_id),collision(self, bot_1_pos, bot_2_pos),getIDtoPause(self, bot_1_id, bot_2_id),poseCallBack(self, msg, bot_id),penDownCallBack(self, msg, bot_id),main(args=None)]
# Nodes:		
#                   Subs: [ 'hb_bot_{self.bot_id}/goal', '/detected_aruco_{self.bot_id}' ]
#                   Pubs: [ "/hb_bot_{self.bot_id}/rear_wheel_force", "/hb_bot_{self.bot_id}/left_wheel_force", "/hb_bot_{self.bot_id}/right_wheel_force" ]
# Global Variables: None

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
from std_msgs.msg import Bool
from std_srvs.srv import Empty

from geometry_msgs.msg import Vector3
from functools import partial



class CollisionChecker(Node):
    '''
        Function Name: __init__

        Input : self--> CollisionChecker

        Output: None

        Logic: initialises each bot with its id and initializes the corresponding entries in the dictionaries and creates a publisher for pause messages and two subscribers for pose and pen down messages. 

        Example call: self.__init__()
    '''
    def __init__(self):
        super().__init__(f'collision_checker_node_start')

        self.bot_ids = [1, 2, 3]
        self.paused = {}

        self.bot_pos = {}
        self.bot_pen_down = {}
        self.bot_pause_pub = {}
        
        self.distance_threshold = 65.0
        for i in self.bot_ids:
            
            self.bot_pos[i] = [0.0, 0.0, 0.0]
            self.paused[i] = True
            self.bot_pen_down[i] = False
            self.bot_pause_pub[i] = self.create_publisher(Bool,
                                                          f"hb_bot_{i}/pause",
                                                          10)
            self.poseSubscriber = self.create_subscription(Pose2D,
                                                           f"pen{i}_pose",
                                                           partial(self.poseCallBack, bot_id=i),
                                                           10)
            self.penDownSubscriber = self.create_subscription(Bool,
                                                          f"/pen{i}_down",
                                                          partial(self.penDownCallBack, bot_id = 1),
                                                          10)
                
            
    '''
        Function Name: publishBotPauseState

        Input : self--> CollisionChecker,bot_id--> id of bot to be paused

        Output: None

        Logic: publishes the pause state of a specific bot

        Example call: self.publishBotPauseState(bot_id)
    ''' 
    def publishBotPauseState(self, bot_id):
        msg = Bool()
        msg.data = self.paused[bot_id]
        self.bot_pause_pub[bot_id].publish(msg)
    '''
        Function Name: collision

        Input : self--> CollisionChecker,bot_1_pos--> position of one bot,bot_2_pos--> position of other bot

        Output: False if distance between 2 bots greater than treshold,else False

        Logic: Calculates euclidean distance between two bots and if they get closer than the treshold value returns True to indicate collision can happen

        Example call: self.publishBotPauseState(bot_id)
    '''    
    def collision(self, bot_1_pos, bot_2_pos):

        distance = math.sqrt(math.pow(bot_1_pos[0] - bot_2_pos[0], 2) + math.pow(bot_1_pos[1] - bot_2_pos[1], 2))
        if(distance > self.distance_threshold):
            return False
        
        return True
    '''
        Function Name: getIDtoPause

        Input : self--> CollisionChecker,bot_1_id--> id of one bot,bot_2_id--> id of other bot

        Output: id of the bot which has to be paused

        Logic: returns the id of the bot which hasn't done pen_down yet or the bot with lower id

        Example call: self.getIDtoPause(bot_1_id,bot_2_id)
    '''  
    def getIDtoPause(self, bot_1_id, bot_2_id):
        # self.get_logger().info(f"{self.bot_pen_down}")
        if(self.bot_pen_down[bot_1_id] == False):
            return bot_1_id
        elif(self.bot_pen_down[bot_2_id] == False):
            return bot_2_id
        else:
            return min(bot_1_id, bot_2_id)
    '''
        Function Name: poseCallBack

        Input : self--> CollisionChecker,msg-->array with values x,y,theta ,bot_id--> id of bot

        Output: None

        Logic: updates the position of the bots and checks for collisions

        Example call: partial(self.poseCallBack, bot_id=i)
    '''    
    def poseCallBack(self, msg, bot_id):
        self.bot_pos[bot_id] = [msg.x, msg.y, msg.theta]
        colliding_bots = set()

        # For each pair of bots, check if they are colliding
        for i in range(len(self.bot_ids)):
            for j in range(i+1, len(self.bot_ids)):
                bot_1_id, bot_2_id = self.bot_ids[i], self.bot_ids[j]
                bot_1_pos, bot_2_pos = self.bot_pos[bot_1_id], self.bot_pos[bot_2_id]
                if self.collision(bot_1_pos, bot_2_pos):
                    bot_to_pause = self.getIDtoPause(bot_1_id, bot_2_id)
                    if not self.paused[bot_to_pause]:
                        self.get_logger().info(f"{bot_1_id} and {bot_2_id} collision paused {bot_to_pause} {self.paused}")
                        self.paused[bot_to_pause] = True
                        self.publishBotPauseState(bot_to_pause)
                    colliding_bots.add(bot_1_id)
                    colliding_bots.add(bot_2_id)

        # Unpause bots that are not colliding with any other bot
        for bot_id in self.bot_ids:
            if bot_id not in colliding_bots and self.paused.get(bot_id, False):
                self.paused[bot_id] = False
                self.publishBotPauseState(bot_id)
    
    '''
        Function Name: penDownCallBack

        Input : self--> CollisionChecker,msg-->boolean value,bot_id--> id of bot

        Output: None

        Logic: Callback function to update the pen down state of the bot with the given bot_id based on the msg object

        Example call: partial(self.penDownCallBack, bot_id = 1)
    '''  
    def penDownCallBack(self, msg, bot_id):
        
        self.bot_pen_down[bot_id] = msg.data
     
        
'''
    Function Name: main

    Input : None

    Output: None

    Logic: create a multi-threaded executor and one instance to check for collision between the three bots

    Example call: main()
'''   

def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()

    collision_checker = CollisionChecker()

    executor.add_node(collision_checker)

    try:
        executor.spin()

    finally:
        executor.shutdown()

        collision_checker.destroy_node()
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
