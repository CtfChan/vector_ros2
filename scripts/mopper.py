import random
import numpy as np
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time

#msgs
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

#srv
from vector_ros2_interfaces.srv import LiftHeight, HeadAngle


class Mopper(Node):
    def __init__(self):
        super().__init__('mopper')

        self.get_logger().info('Initializing mopper node')

        #proximity sub
        self.create_subscription(Float32, '/vector/proximity', self.proximity_callback)

        #odom sub
        self.create_subscription(Odometry, '/vector/odom', self.odom_callback)

        #cmd_vel pub
        self.cmd_vel_pub = self.create_publisher(Twist, '/vector/cmd_vel')

        #client 
        self.lift_client = self.create_client(LiftHeight, '/vector/lift_height')
        while not self.lift_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Lift Service not available. Waiting again ...')
        self.lift_req = LiftHeight.Request()
        self.drop_lift()

        #private variable
        self.prev_odom_time = None
        self.prev_odom = None

    def drop_lift(self):
        self.lift_req.desired_height = 0.0
        self.future = self.lift_client.call_async(self.lift_req)

    def odom_callback(self, msg):
        #we just want to reverse if we get stuck
        if (self.prev_odom is None):
            self.prev_odom = msg
            self.prev_odom_time = time.time()
            return 

        if ((time.time() - self.prev_odom_time) > 1.0):
            curr_pose = np.array((msg.pose.pose.position.x, msg.pose.pose.position.y))
            prev_pose = np.array((self.prev_odom.pose.pose.position.x, self.prev_odom.pose.pose.position.y))
            distance = np.linalg.norm(curr_pose - prev_pose)
            if distance < 0.01:
                self.get_logger().info('I might be stuck')

                twist = Twist()
                twist.linear.x = -0.1
                self.cmd_vel_pub.publish(twist)
                
                time.sleep(0.5)

                twist.linear.x = 0.0
                twist.angular.z = 2.0 if random.randint(0, 1) == 1 else -2.0
                self.cmd_vel_pub.publish(twist)


            self.prev_odom = msg
            self.prev_odom_time = time.time()


    def proximity_callback(self, msg):
        twist = Twist()
        #if obstructed turn on spot
        distance = msg.data
        if (distance > 0.05):
            twist.linear.x = 0.1
            self.get_logger().info('straight')
        else:
            twist.angular.z = 2.0 if random.randint(0, 1) == 1 else -2.0
            self.get_logger().info('turn')
            time.sleep(0.5)
            

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    #initialize ros stuff
    rclpy.init(args=args)
    mopper = Mopper()
    rclpy.spin(mopper)

    #destroy node explicitly
    mopper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


