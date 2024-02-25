#!/usr/bin/env python3
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
    def __init__(self):
        super().__init__(f'collision_checker_node_start')

        self.bot_ids = [1, 2, 3]
        self.paused = {}

        self.bot_pos = {}
        self.bot_pen_down = {}
        self.bot_pause_pub = {}
        
        self.distance_threshold = 50.0
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
                
            

    def publishBotPauseState(self, bot_id):
        msg = Bool()
        msg.data = self.paused[bot_id]
        self.bot_pause_pub[bot_id].publish(msg)

    def collision(self, bot_1_pos, bot_2_pos):

        distance = math.sqrt(math.pow(bot_1_pos[0] - bot_2_pos[0], 2) + math.pow(bot_1_pos[1] - bot_2_pos[1], 2))
        if(distance > self.distance_threshold):
            return False
        
        return True

    def getIDtoPause(self, bot_1_id, bot_2_id):
        # self.get_logger().info(f"{self.bot_pen_down}")
        if(self.bot_pen_down[bot_1_id] == False):
            return bot_1_id
        elif(self.bot_pen_down[bot_2_id] == False):
            return bot_2_id
        else:
            return min(bot_1_id, bot_2_id)

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
    

    def penDownCallBack(self, msg, bot_id):
        
        self.bot_pen_down[bot_id] = msg.data



        

        
        


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
