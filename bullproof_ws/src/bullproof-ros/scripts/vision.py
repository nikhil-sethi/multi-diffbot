#!/usr/bin/env python3
import rospy
# from sensor_msgs import LaserScan
# from geometry_msgs import Twist
from nav_msgs import OccupancyGrid



class OccupancyMap:
	def __init__(self, name):
		#super.__init__(name)
		# self.lidar_sub = rospy.Subscriber('/scan',LaserScan, queue_size = 100)
		self.pub = rospy.Publisher('/occupancy_map', OccupancyGrid, queue_size=10)
		self.name = name
	def make_map(self):
		row = [0.]*7
		map = [row for i in range(5)]
		map[1][1] = 1
		map[-1][-3] = 1
		self.pub.publish(map)
		return map
		



if __name__ == '__main__':
	occupancymap = OccupancyMap('map1')
	map = occupancymap.make_map()



