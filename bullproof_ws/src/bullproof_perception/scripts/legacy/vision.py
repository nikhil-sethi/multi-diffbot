import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Mouse callback function
def get_pixel_location(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONUP:
        print("Clicked pixel location (x, y):", x, y)

# Callback function for receiving the image from ROS topic
def image_callback(msg):
    # Convert ROS image message to OpenCV image
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # Display the image
    cv2.imshow("Image", cv_image)

    # Set the mouse callback function for the image window
    cv2.setMouseCallback("Image", get_pixel_location)

    # Wait for a key press
    cv2.waitKey(1)

# Initialize ROS node
rospy.init_node('image_pixel_location')

# Create a CV bridge object
bridge = CvBridge()

# Subscribe to the image topic
image_topic = "/webcam/image_sync"  # Replace with your image topic
rospy.Subscriber(image_topic, Image, image_callback)

# Keep the script running
rospy.spin()