
import cv2  # OpenCV library
import rclpy  # Python Client Library for ROS 2
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from rclpy.node import Node  # Handles the creation of nodes
from std_msgs.msg import String
from sensor_msgs.msg import Image  # Image is the message type
from rclpy.qos import qos_profile_sensor_data

def main(args=None):
    rclpy.init(args=None)
    node = rclpy.create_node('minimal_subscriber')
    cameraSub = node.create_subscription(Image, "/std_ns/camera_module/video_frames", save_image_callback, qos_profile_sensor_data)
    cameraSub
    # prevent unused variable warning

    rclpy.spin_once(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown() 

def save_image_callback(data):
    br = CvBridge()
    current_frame = br.imgmsg_to_cv2(data)
    if current_frame.any(): 
        cv2.imwrite("test.png", current_frame)

    # Display image
    cv2.imshow("camera", current_frame)
    cv2.waitKey(1)


if __name__ == "__main__":
    main()
