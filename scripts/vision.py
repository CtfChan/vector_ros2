import anki_vector

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from std_msgs.msg import String


class Vision(Node):
    def __init__(self, async_robot, proximity_publish_rate=10.0):
        super().__init__('vision')
        self.async_robot = async_robot

        #set up proximity callback
        self.proximity_publisher = self.create_publisher(String, '/vector/proximity')
        timer_period = 1.0/proximity_publish_rate
        self.timer = self.create_timer(timer_period, self.proximity_callback)

        #TO DO set up image callback

    def proximity_callback(self):
        self.get_logger().info('Proximity Callback')
        proximity_data = self.async_robot.proximity.last_sensor_reading
        if proximity_data is not None:
            print('Proximity distance: {0}, engine considers useful: {1}'.format(proximity_data.distance, proximity_data.is_valid))



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



# import rospy
# import anki_vector
# import cv_bridge
# import numpy

# from sensor_msgs.msg import Image

# class Camera(object):
#     def __init__(self, async_robot, publish_rate=10):
#         self.async_robot = async_robot
#         self.rate = rospy.Rate(publish_rate)
#         self.image_publisher = rospy.Publisher("~camera", Image, queue_size=1)
#         self.publish_camera_feed()

#     def publish_camera_feed(self):
#         bridge = cv_bridge.CvBridge()

#         while not rospy.is_shutdown():
#             image = bridge.cv2_to_imgmsg(numpy.asarray(self.async_robot.camera.latest_image), encoding="rgb8") # convert PIL.Image to ROS Image
#             self.image_publisher.publish(image)

#             # make sure to publish at required rate
#             self.rate.sleep()

# if __name__=="__main__":
#     rospy.init_node("camera")
#     async_robot = anki_vector.AsyncRobot(enable_camera_feed=True)
#     async_robot.connect()
#     Camera(async_robot)
# rospy.spin()