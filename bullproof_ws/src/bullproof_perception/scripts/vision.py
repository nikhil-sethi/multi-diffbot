#!/usr/bin/env python3
import numpy as np
import cv2
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


# class OccupancyMap:
# 	def __init__(self, name):
# 		#super.__init__(name)
# 		# self.lidar_sub = rospy.Subscriber('/scan',LaserScan, queue_size = 100)
# 		self.pub = rospy.Publisher('/occupancy_map', OccupancyGrid, queue_size=10)
# 		self.name = name
# 	def make_map(self):
# 		row = [0.]*7
# 		map = [row for i in range(5)]
# 		map[1][1] = 1
# 		map[-1][-3] = 1
# 		self.pub.publish(map)
# 		return map
		


# https://w3.cs.jmu.edu/spragunr/CS354_S15/labs/mapping/mapper.py



def img_to_occupancy(file, width=None, height=None):
    color_img = cv2.imread(file)
    grey_img = np.mean(color_img, axis=2)
    bitmap = np.round(-grey_img/255+1).astype(int)
    occupancy_map = bitmap*100

    # max pooling
    # if width and height:
        # M, N = occupancy_map.shape
        # K = width
        # L = height
        # MK = M // K
        # NL = N // L
        # occupancy_map = bitmap[:MK*K, :NL*L].reshape(MK, K, NL, L).max(axis=(1, 3))
        # # or
        # cv2.resize(occupancy_map, (height, width)).astype(int)

    return occupancy_map


def generate_occupancy_map():
    pub = rospy.Publisher('map', OccupancyGrid, queue_size=1) #, latch=True)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        rospy.loginfo('hello:' + str(rospy.Time.now()))
        # print(rospy.Time.now())
        
        grid_msg = OccupancyGrid()
        # header
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"
        
		# info
        # color_img = cv2.imread('bullproof_ws/src/bullproof-ros/scripts/top_view_black_white.jpeg')
        color_img = cv2.imread('src/bullproof-ros/images/top_view_black_white.jpeg')
        height, width = color_img.shape[:2] # in pixels (or squares. all the same.)
        # print('height=', height)
        # print('width=', width)

        grid_msg.info.resolution = 1 # Width of each grid square in meters
        grid_msg.info.width = width
        grid_msg.info.height = height

        grid = img_to_occupancy('src/bullproof-ros/images/top_view_black_white.jpeg', width=width, height=height) # downscaling does not work yet
        
        flat_grid = grid.reshape((grid.size)) # make into one long list
        
        grid_msg.data = list(flat_grid) # occupancy grid message must be list of integers between 0 and 100

        pub.publish(grid_msg)
        rate.sleep()














# import numpy as np
# import cv2
# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge



# def test_img_out():
    
#     bridge = CvBridge()

#     image_pub = rospy.Publisher('image_raw/compressed', Image, queue_size=1)

#     rospy.init_node('fake_camera')

#     rate = rospy.Rate(1) # 1hz


#     while not rospy.is_shutdown():
#         file = '../images/apriltag_example_image.jpg'
#         cv_image = cv2.imread(file)

#         # cv2.imshow('aa', cv_image)
#         # cv2.waitKey(0)

#         # img_to_pub = Image

#         # img_to_pub.data = cv_image
#         # img_to_pub.height = cv_image.shape[0]
#         # img_to_pub.width = cv_image.shape[1]

        
#         rospy.loginfo('publishing image')

#         try:
#             img_msg = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
#             image_pub.publish(img_msg)
#         except:
#             pass
        
#         rospy.loginfo('done publishing image')

#         rate.sleep()





# if __name__ == '__main__':
#     test_img_out()

