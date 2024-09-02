#!/usr/bin/env python

import rospy
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d

class BoundingBoxListener:
    def __init__(self):
        self.x_min_list = []
        self.y_min_list = []  # New list for y_min values
        self.y_max_list = []  # New list for y_max values
        self.max_size = 100  # Limit the size of the lists

        rospy.init_node('darknet_ros_3d_bounding_boxes_listener', anonymous=True)
        rospy.Subscriber('/darknet_ros_3d/bounding_boxes', BoundingBoxes3d, self.bounding_boxes_callback)
        
        rospy.spin()

    def bounding_boxes_callback(self, msg):
        if not msg.bounding_boxes:
            return  # Return if no bounding boxes are received

        for bbox in msg.bounding_boxes:
            self.x_min_list.append(bbox.xmin)
            self.y_min_list.append(bbox.ymin)  # Append ymin to the list
            self.y_max_list.append(bbox.ymax)  # Append ymax to the list

            # Limit the size of the x_min_list, y_min_list, and y_max_list
            if len(self.x_min_list) > self.max_size:
                self.x_min_list.pop(0)  # Remove the oldest entry
            if len(self.y_min_list) > self.max_size:
                self.y_min_list.pop(0)  # Remove the oldest entry
            if len(self.y_max_list) > self.max_size:
                self.y_max_list.pop(0)  # Remove the oldest entry

        # Print x_min, y_min, y_max, and dia values in the same line
        for x_min, y_min, y_max in zip(self.x_min_list, self.y_min_list, self.y_max_list):
            dia = y_max - y_min  # Calculate the difference
            print(f"x_min: {x_min:.2f}, y_min: {y_min:.2f}, y_max: {y_max:.2f}, dia: {dia:.2f}")  # Print each value as float

if __name__ == '__main__':
    BoundingBoxListener()
