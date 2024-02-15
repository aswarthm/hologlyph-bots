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
        self.bot_pause_pub = {}
        
        self.distance_threshold = 100.0
        for i in self.bot_ids:
            
            self.bot_pos[i] = [0.0, 0.0, 0.0]
            self.paused[i] = True
            self.bot_pause_pub[i] = self.create_publisher(Bool,
                                                          f"hb_bot_{i}/pause",
                                                          10)
            self.poseSubscriber = self.create_subscription(Pose2D,
                                                           f"pen{i}_pose",
                                                           partial(self.poseCallBack, bot_id=i),
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

    def poseCallBack(self, msg, bot_id):

        self.bot_pos[bot_id] = [msg.x, msg.y, msg.theta]

        for bot_1_id, bot_1_pos in self.bot_pos.items():
            for bot_2_id, bot_2_pos in self.bot_pos.items():
                if(bot_1_id != bot_2_id):
                    if(self.collision(bot_1_pos, bot_2_pos) == True):
                        bot_id = min(bot_1_id, bot_2_id)
                        if(self.paused[bot_id] == False):
                            self.get_logger().info(f"{bot_1_id} and {bot_2_id} collision")
                            self.paused[bot_id] = True
                            self.publishBotPauseState(bot_id)
                    else:
                        if(self.paused[bot_1_id] == True):
                            self.paused[bot_1_id] = False
                            self.publishBotPauseState(bot_1_id)
                        if(self.paused[bot_2_id] == True):
                            self.paused[bot_2_id] = False
                            self.publishBotPauseState(bot_2_id)

        

        
        


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
