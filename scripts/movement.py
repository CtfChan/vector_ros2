import anki_vector
from anki_vector.behavior import MIN_HEAD_ANGLE, MAX_HEAD_ANGLE

import time
import numpy as np

import rclpy
from rclpy.node import Node

#msgs
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

#srv
from vector_ros2_interfaces.srv import LiftHeight, HeadAngle


class Movement(Node):
    def __init__(self, async_robot, odom_publish_rate=30):
        super().__init__('movement')
        self.async_robot = async_robot

        self.get_logger().info('Initializing movement node')

        #subscriber to cmd_vel
        self.cmd_vel_sub = self.create_subscription(Twist, '/vector/cmd_vel', self.cmd_vel_callback)
        self.cmd_vel_sub #prevent warning

        #odom pub
        self.odom_pub = self.create_publisher(Odometry, '/vector/odom')
        self.odom_timer = self.create_timer( 1.0/odom_publish_rate, self.odom_callback)

        #lift height srv   
        self.lift_height_srv = self.create_service(LiftHeight, '/vector/lift_height', self.lift_height_callback)

        #head angle srv
        self.head_angle_srv = self.create_service(HeadAngle, '/vector/head_angle', self.head_angle_callback)


        #private members
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.lift_height = 0.0
        self.head_angle = anki_vector.util.degrees(35.0)

    def head_angle_callback(self, request, response):
        if (MIN_HEAD_ANGLE.degrees > request.desired_angle):
            self.head_angle = MIN_HEAD_ANGLE
        elif (MAX_HEAD_ANGLE.degrees < request.desired_angle):
            self.head_angle = MAX_HEAD_ANGLE
        else:
            self.head_angle = anki_vector.util.degrees(request.desired_angle)
        self.async_robot.behavior.set_head_angle(self.head_angle)

        response.current_angle = self.head_angle.degrees
        return response

    def lift_height_callback(self, request, response):
        if (request.desired_height >= 0.0):
            self.async_robot.behavior.set_lift_height(request.desired_height)
            self.lift_height = request.desired_height
        
        response.current_height = self.lift_height
        return response


    def odom_callback(self):
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = self.async_robot.pose.position.x * 0.001
        odom.pose.pose.position.y = self.async_robot.pose.position.y * 0.001
        odom.pose.pose.position.z = self.async_robot.pose.position.z * 0.001
        q = self.async_robot.pose.rotation
        odom.pose.pose.orientation.x = q.q0
        odom.pose.pose.orientation.y = q.q1
        odom.pose.pose.orientation.z = q.q2
        odom.pose.pose.orientation.w = q.q3

        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.angular.z = self.angular_velocity

        self.odom_pub.publish(odom)


    def cmd_vel_callback(self, msg):
        axle_length = 0.055 #5.5cm
        cmd_lin_vel = msg.linear.x
        cmd_ang_vel = msg.angular.z
        right_wheel_vel = cmd_lin_vel - (cmd_ang_vel*axle_length*0.5)
        left_wheel_vel = cmd_lin_vel + (cmd_ang_vel*axle_length*0.5)
        #give wheel velocities in mm/s
        self.async_robot.motors.set_wheel_motors(right_wheel_vel*1000.0, left_wheel_vel*1000.0)

        self.linear_velocity = cmd_lin_vel
        self.angular_velocity = cmd_ang_vel


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


