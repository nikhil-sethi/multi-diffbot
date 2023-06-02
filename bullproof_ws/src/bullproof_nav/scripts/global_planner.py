#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import numpy as np
import cv2


# class GlobalPlanner:
# 	def __init__(self, name):
# 		#super.__init__(name)
# 		self.sub = rospy.Subscriber('/occupancy_map', OccupancyGrid, queue_size = 10)
# 		self.name = name
# 	def visualize_grid(self):
# 		map = self.sub
# 		a=1



# https://people.eng.unimelb.edu.au/pbeuchat/asclinic/software/ros_code_pub_and_sub_simple.html#add-the-subscriber-callback-function-for-when-messages-are-received-subscriber
# class map_subscriber:

#   def __init__(self):
#       # Initialise a subscriber
#       rospy.Subscriber('map', OccupancyGrid, self.subscriberCallback, queue_size=1)

#   # Implement the subscriber callback (what info/data it retrieves from the topic I think?)
#   def subscriberCallback(self, msg):
#       # Extract the data from the message
#       data = msg.data
#       # Display the data
#       rospy.loginfo("[MAPPER SUBSCRIBER!!! :D] received message with data = " + str(data))
#       return data
	

def visualize(map,height,width):
    map = np.reshape(map,(height,width)).astype(float)
    # for i, row in enumerate(map):
    #     print('this is row',i,':',row)

    cv2.imshow('image', map)
    cv2.waitKey(1)
    


def callback(msg):
    rospy.loginfo('bye' + str(rospy.Time.now()))
    map = msg.data
    info = msg.info
    width = info.width
    height = info.height
    visualize(map, height, width)
	
def get_occupancy_map():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('map', OccupancyGrid, callback)ccupancyGrid

    # print(' heeeeeerrrrrreeeeee')
    # print(sub)
    rospy.spin()
    



if __name__ == '__main__':
    get_occupancy_map()
    
    # rospy.init_node("sub_py_node")
    # rospy.loginfo("[SUBSCRIBER PY  NODE] namespace of node = " + rospy.get_namespace())
    # rospy.spin()
    # sub = map_subscriber()
    # sub.subscriberCallback()
    
	

