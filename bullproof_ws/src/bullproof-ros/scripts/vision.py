#!/usr/bin/env python3
import numpy as np

# grid = np.zeros((5,5),dtype=int)
# a = grid.tolist()
# print(type(a))
# print(' siizzeee = ', grid.size)
# flat_grid = grid.reshape((grid.size,)) * 100
# print(list(np.round(flat_grid)))
# print(' aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')

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


def generate_occupancy_map():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    pub2 = rospy.Publisher('map', OccupancyGrid, queue_size=10)#, latch=True)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        
        grid_msg = OccupancyGrid()
        # header
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"
        
		# info
        grid_msg.info.resolution = 1 # Width of each grid square in meters
        grid_msg.info.width = 10
        grid_msg.info.height = 10
        grid = np.zeros((10,10), dtype=int) # 10x10 grid with values between 0 and 1
        grid[1,1] = 100
        grid[5,8] = 100
        
        flat_grid = grid.reshape((grid.size)) # make into one long list
        
        grid_msg.data = list(flat_grid) # occupancy grid message must be list of integers between 0 and 100
        # grid_msg.data = grid.tolist() # occupancy grid message must be list of integers between 0 and 100

        rospy.loginfo(hello_str)
        
        pub.publish(hello_str)
        pub2.publish(grid_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        generate_occupancy_map()
    except rospy.ROSInterruptException:
        pass


