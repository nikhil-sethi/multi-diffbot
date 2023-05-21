#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import numpy as np


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
	

def visualize(map,width,height):
    map = np.reshape(map,(height,width))
    for i, row in enumerate(map):
        print('this is row',i,':',row)
    


def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
    map = msg.data
    width = msg.info.width
    height = msg.info.width
    visualize(map,width,height)
	
def get_occupancy_map():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('map', OccupancyGrid, callback)

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
    
	

