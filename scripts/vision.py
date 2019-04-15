import anki_vector

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from std_msgs.msg import Float32


class Vision(Node):
    def __init__(self, async_robot, proximity_publish_rate=30.0):
        super().__init__('vision')
        self.async_robot = async_robot

        self.get_logger().info('Initializing vision node')

        #set up proximity callback
        self.proximity_pub = self.create_publisher(Float32, '/vector/proximity')
        timer_period = 1.0/proximity_publish_rate
        self.timer = self.create_timer(timer_period, self.proximity_callback)

        #TO DO set up image callback
        # self.image_publisher = self.create_publisher(Image, '/vector/image')


    def proximity_callback(self):
        proximity_data = self.async_robot.proximity.last_sensor_reading
        if proximity_data is not None:
            msg = Float32()
            msg.data = proximity_data.distance.distance_mm / 1000.0 #convert to meters
            self.proximity_pub.publish(msg)


def main(args=None):
    #create connection to vector 
    async_robot = anki_vector.AsyncRobot(enable_camera_feed=True)
    async_robot.connect()

    #initialize ros stuff
    rclpy.init(args=args)
    vision = Vision(async_robot)
    rclpy.spin(vision)

    #destroy node explicitly
    vision.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


