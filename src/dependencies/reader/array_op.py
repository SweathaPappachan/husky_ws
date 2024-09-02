#!/usr/bin/env python

import rospy
import numpy as np
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d

class BoundingBoxListener:
    def __init__(self):
        # Initialize an empty list to store x_min values
        self.x_min_list = []

        # Initialize the ROS node
        rospy.init_node('darknet_ros_3d_bounding_boxes_listener', anonymous=True)

        # Subscribe to the /darknet_ros_3d/bounding_boxes topic
        rospy.Subscriber('/darknet_ros_3d/bounding_boxes', BoundingBoxes3d, self.bounding_boxes_callback)
        
        # Keep the node running
        rospy.spin()

    def bounding_boxes_callback(self, msg):
        # Iterate through all bounding boxes in the message and store x_min values
        for bbox in msg.bounding_boxes:
            self.x_min_list.append(bbox.xmin)

        # Convert the list to a NumPy array
        x_min_array = np.array(self.x_min_list)

        # Output the NumPy array
        print(x_min_array )

if __name__ == '__main__':
    BoundingBoxListener()
