#! /usr/bin/env python3
"""Camera node"""

import cv2  # OpenCV library
import rclpy  # Python Client Library for ROS 2
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from rclpy.node import Node  # Handles the creation of nodes
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from std_msgs.msg import String
from sensor_msgs.msg import Image  # Image is the message type

from time import sleep
class CameraPublisherNode(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """

    def __init__(self, NODE_NAME = "Camera_Publisher_Node"):

        """
        Class constructor to set up the Camera node
        """

        super().__init__(NODE_NAME)

        # We will publish a message every 0.1 seconds
        timer_period = 0.05  # seconds
        # State publisher

        camera_cb_group = ReentrantCallbackGroup()
        state_cb_group = ReentrantCallbackGroup()


        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.camera_value=0
        self.cam = cv2.VideoCapture(self.camera_value)
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        
        sleep(1)
        if self.cam:
            self.state="READY"
        else:
            self.state="ERROR"

        self.statePub = self.create_publisher(String, NODE_NAME + '/state', 10)
        self.stateTimer = self.create_timer(timer_period, self.stateCallback, callback_group = state_cb_group)
        # Initiate the Node class's constructor and give it a name

        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.cameraPub = self.create_publisher(Image, NODE_NAME + "/video_frames", 10)
        self.cameraPub_handler = self.create_timer(timer_period, callback = self.cameraCallback, callback_group = camera_cb_group)


    def stateCallback(self):
        '''
        Publishes the state to the 'state' topic. 
        '''
        if not self.cam:
            self.state = "Error"

        msg = String()
        msg.data = 'State: %s' % self.state
        self.statePub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        # TODO: Add a state check before assigning state 

    def cameraCallback(self):
        """Callback function.
        This function gets called every 0.1 seconds.
        """

        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.

        ret, frame = self.cam.read()
        if ret:
            self.state = "Busy"
            self.stateCallback()
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            self.cameraPub.publish(self.br.cv2_to_imgmsg(frame))

        # Display the message on the console
        self.get_logger().info("Publishing video frame")


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
