import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, SetPen
import time

class TurtlePublisher(Node):
    def __init__(self):
        super().__init__('turtle_publisher')
        self.subscription = self.create_subscription(Pose, 
                                                     '/turtle1/pose',
                                                     self.update_pose,
                                                     10)
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.pose = Pose()

        self.first_done = 0
        self.second_done = 0

        self.speed = 1.0
        self.radius_1 = 0.9
        self.radius_2 = -0.5
    
    def update_pose(self, pose:Pose):
        self.pose.x = pose.x
        self.pose.y = pose.y
        self.pose.theta = pose.theta
        #self.get_logger().info('pos=' + str(pose))
        self.move()
    
    def update_pose_2(self, pose:Pose):
        self.pose.x = pose.x
        self.pose.y = pose.y
        self.pose.theta = pose.theta
        #self.get_logger().info('pos=' + str(pose))
        self.move_2()
    
    def spawn(self):
        self.client = self.create_client(Spawn, '/spawn')
        while(not self.client.wait_for_service(1.0)):
            self.get_logger().warn("Waiting for spawn service...")
        request = Spawn.Request()
        request.name = "turtle2"
        request.x = self.pose.x
        request.y = self.pose.y
        request.theta = 0.0

        self.future = self.client.call_async(request)

        self.client = self.create_client(SetPen, '/turtle2/set_pen')
        while(not self.client.wait_for_service(1.0)):
            self.get_logger().warn("Waiting for set pen service")
        
        request = SetPen.Request()
        request.r = 200
        request.g = 200
        request.b = 230
        request.width = 5
        future = self.client.call_async(request)

        self.subscription = self.create_subscription(Pose,
                                                     '/turtle2/pose',
                                                     self.update_pose_2,
                                                     10)
        self.publisher_2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
    
    def move(self):
        if(self.first_done == 0):
            msg = Twist()
            msg.linear.x = self.speed# * (self.goal_x - self.pose.x)
            msg.angular.z = self.radius_1
            if(self.pose.theta >= -0.1 and self.pose.theta < 0.0):
                self.first_done = 1
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
                time.sleep(0.5)
                self.spawn()
            self.publisher_.publish(msg)
            self.get_logger().info('x ' + str(self.pose.x) + ' angle' + str(self.pose.theta))
            # if(self.first_done == 0):
    
    def move_2(self):
        if(self.second_done == 0):
            msg = Twist()
            msg.linear.x = self.speed# * (self.goal_x - self.pose.x)
            msg.angular.z = self.radius_2
            if(self.pose.theta <= 0.1 and self.pose.theta > 0.0):
                self.second_done = 1
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            self.publisher_2.publish(msg)
            self.get_logger().info('x ' + str(self.pose.x) + ' angle' + str(self.pose.theta))
            # if(self.first_done == 0):
        
def main(args=None):
    rclpy.init(args=args)

    turtle_publisher = TurtlePublisher()

    rclpy.spin(turtle_publisher)

    turtle_publisher.destroy_publisher()
    turtle_publisher.destroy_subscription()

    rclpy.shutdown()

if __name__ == 'main':
    main()


            