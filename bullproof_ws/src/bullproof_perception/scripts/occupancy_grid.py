#!/usr/bin/env python3
import numpy as np
import cv2
import rospy
import os


# sensor_msgs/CameraInfo Message



def test_img_out():
    
    bridge = CvBridge()
    current_directory = os.getcwdb()

    image_pub = rospy.Publisher('webcam/image_raw/compressed', Image, queue_size=1)
    camera_info_pub = rospy.Publisher('webcam/image_raw/camera_info', CameraInfo, queue_size=1)
    # image_pub = rospy.Publisher('image_raw', String, queue_size=1)


    rospy.init_node('fake_camera')

    rate = rospy.Rate(1) # 1hz


    while not rospy.is_shutdown():

        file = os.path.join(current_directory,'/../images/apriltag_example_image.jpg')
        # '/home/dborstlap/unif/mdp/bullproof-tech/bullproof_ws/src/bullproof_perception/images/apriltag_example_image.jpg'
        cv_image = cv2.imread(file)

        # cv2.imshow('aa', cv_image)
        # cv2.waitKey(0)

        # img_to_pub = Image()
        # if type(cv_image)==np.ndarray:
        #     img_to_pub.data = cv_image
        #     img_to_pub.height = cv_image.shape[0]
        #     img_to_pub.width = cv_image.shape[1]

        
        rospy.loginfo('publishing image')
        # msg = Image()
        # try:
        # if type(cv_image)==np.ndarray:
        img_msg = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        # rospy.loginfo(img_msg)
        image_pub.publish(img_msg)

        cam_inf = CameraInfo()
        camera_info_pub.publish(cam_inf)

        
        # rospy.loginfo('done publishing image')

        rate.sleep()



if __name__ == '__main__':
    test_img_out()








