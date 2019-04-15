import anki_vector
import time

import rclpy
from rclpy.node import Node

# from std_msgs.msg import Float32

from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Movement(Node):
    def __init__(self, async_robot):
        super().__init__('movement')
        self.async_robot = async_robot

        self.get_logger().info('Initializing movement node')

        #subscriber to cmd_vel
        self.cmd_vel_sub = self.create_subscription(Twist, '/vector/cmd_vel', self.cmd_vel_callback)
        self.cmd_vel_sub #prevent warning

        #imu pub

    def cmd_vel_callback(self, msg):
        self.get_logger().info('Got a cmd_vel msg')
        axle_length = 0.055 #5.5cm
        cmd_lin_vel = msg.linear.x
        cmd_ang_vel = msg.angular.z
        right_wheel_vel = cmd_lin_vel - (cmd_ang_vel*axle_length*0.5)
        left_wheel_vel = cmd_lin_vel + (cmd_ang_vel*axle_length*0.5)
        #give wheel velocities in mm/s
        self.async_robot.motors.set_wheel_motors(right_wheel_vel*1000.0, left_wheel_vel*1000.0)


def main(args=None):
    #create connection to vector 
    async_robot = anki_vector.AsyncRobot(enable_camera_feed=True)
    async_robot.connect()

    #initialize ros stuff
    rclpy.init(args=args)
    movement = Movement(async_robot)
    rclpy.spin(movement)

    #destroy node explicitly
    movement.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


