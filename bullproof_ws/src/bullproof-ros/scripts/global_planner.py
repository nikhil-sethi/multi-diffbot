#!/usr/bin/env python3
import rospy
from nav_msgs import OccupancyGrid
from vision import Generate_occupancy_map

class GlobalPlanner:
	def __init__(self, name):
		#super.__init__(name)
		self.sub = rospy.Subscriber('/occupancy_map',OccupancyGrid, queue_size = 20)
		self.name = name
	def visualize_grid(self):
		map = self.sub
		for row in map:
			print(row)

if __name__ == '__main__':
	global_planner = GlobalPlanner('global_planner')
	global_planner.run()

