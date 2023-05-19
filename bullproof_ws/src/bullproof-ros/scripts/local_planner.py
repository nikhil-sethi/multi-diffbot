#!/usr/bin/env python3
import rospy
from sensor_msgs import LaserScan
from geometry_msgs import Twist

class LocalPlanner:
	def __init__(self, name):
		#super.__init__(name)
		self.lidar_sub = rospy.Subscriber('/scan',LaserScan, queue_size = 100)
		self.control_pub = rospy.Publisher('')
		self.name = name
	def run(self):
		while rospy.isS

if __name__ == '__main__':
	local_planner = LocalPlanner('local_planner')
	local_planner.run()
