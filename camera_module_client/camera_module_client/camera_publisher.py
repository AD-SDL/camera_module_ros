#! /usr/bin/env python3
"""Camera node"""

import cv2  # OpenCV library
import rclpy  # Python Client Library for ROS 2
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from rclpy.node import Node  # Handles the creation of nodes
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from sensor_msgs.msg import Image  # Image is the message type
from wei_services.srv import WeiImage

from time import sleep
class CameraPublisherNode(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """

    def __init__(self, TEMP_NODE_NAME = "Camera_Publisher_Node"):

        """
        Class constructor to set up the Camera node
        """

        super().__init__(TEMP_NODE_NAME)
        node_name = self.get_name()


        self.declare_parameter('camera_number', 0)       # Declaring parameter so it is able to be retrieved from module_params.yaml file
        self.camera_number = self.get_parameter('camera_number').get_parameter_value().integer_value     # Renaming parameter to general form so it can be used for other nodes too
        self.get_logger().info("Received Camera Name: " + node_name + " Camera number: " + str(self.camera_number))

        # We will publish a message every 0.1 seconds
        timer_period = 0.05  # seconds
        # State publisher

        camera_cb_group = ReentrantCallbackGroup()

        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.camera_value = self.camera_number
        self.cam = cv2.VideoCapture(self.camera_value)
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        self.current_image=None

        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.cameraPub = self.create_publisher(Image, node_name + "/video_frames", 10)
        self.cameraPub_handler = self.create_timer(timer_period, callback = self.cameraCallback, callback_group = camera_cb_group)


    def cameraCallback(self):
        """Callback function.
        This function gets called every 0.1 seconds.
        """

        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.

        ret, frame = self.cam.read()
        if ret:
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            self.current_image = self.br.cv2_to_imgmsg(frame)
            self.cameraPub.publish(self.current_image)
        # Display the message on the console
        self.get_logger().info("Publishing video frame")

    def grabImage(self,response):
            response.img = self.current_image
            return response

def main(args=None):

    rclpy.init(args=args)
    try:
        camera_publisher_node = CameraPublisherNode()
        executor = MultiThreadedExecutor()
        executor.add_node(camera_publisher_node)

        try:
            camera_publisher_node.get_logger().info('Beginning client, shut down with CTRL-C')
            executor.spin()
        except KeyboardInterrupt:
            camera_publisher_node.get_logger().info('Keyboard interrupt, shutting down.\n')
        finally:
            executor.shutdown()
            camera_publisher_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
