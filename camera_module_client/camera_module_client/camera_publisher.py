#! /usr/bin/env python3
"""Camera node"""

import cv2  # OpenCV library
import rclpy  # Python Client Library for ROS 2
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from rclpy.node import Node  # Handles the creation of nodes
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from sensor_msgs.msg import Image  # Image is the message type

from time import sleep

import cv2, queue, threading
class VideoCapture:

  def __init__(self, name):
    self.cap = cv2.VideoCapture(name)
    self.q = queue.Queue()
    t = threading.Thread(target=self._reader)
    t.daemon = True
    t.start()

  # read frames as soon as they are available, keeping only most recent one
  def _reader(self):
    while True:
      ret, frame = self.cap.read()
      if not ret:
        break
      if not self.q.empty():
        try:
          self.q.get_nowait()   # discard previous (unprocessed) frame
        except queue.Empty:
          pass
      self.q.put(frame)

  def read(self):
    return self.q.get()

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


        self.declare_parameter('camera_number', 0)       
        self.camera_number = self.get_parameter('camera_number').get_parameter_value().integer_value    
        self.get_logger().info("Received Camera Name: " + node_name + " Camera number: " + str(self.camera_number))

        # We will publish a message every 0.1 seconds
        timer_period =  0.1 # seconds
        # State publisher

        camera_cb_group = ReentrantCallbackGroup()

        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        if self.camera_number == 0:
           self.cam = VideoCapture(self.camera_number)
        else:
           url = 'rtsp://admin:123@rplcam' + str(self.camera_number) + '.cels.anl.gov:8554/profile1'
           self.cam = VideoCapture(url)
        # Used to convert between ROS and OpenCV images
        #self.cam.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        self.br = CvBridge()
        self.current_image=None

        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.cameraPub = self.create_publisher(Image, node_name + "/video_frames", 1)
        self.cameraPub_handler = self.create_timer(timer_period, callback = self.cameraCallback, callback_group = camera_cb_group)


    def cameraCallback(self):
        """Callback function.
        This function gets called every 0.1 seconds.
        """

        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret = True
        frame = self.cam.read()
        if ret:
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            frame =  cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.current_image = self.br.cv2_to_imgmsg(frame)
            sleep(0.05)
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
