#!/usr/bin/env python

import rospy
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d

def bounding_boxes_callback(msg):
    rospy.loginfo("Received BoundingBoxes3d message:")
    # Iterate through all bounding boxes in the message
    for bbox in msg.bounding_boxes:
        # Access the bounding box attributes
        # rospy.loginfo(f"Class: {bbox.class}")
        # rospy.loginfo(f"Probability: {bbox.probability}")
        rospy.loginfo(f"Bounding Box - x_min: {bbox.xmin}, y_min: {bbox.ymin}, z_min: {bbox.zmin}, "
                      f"x_max: {bbox.xmax}, y_max: {bbox.ymax}, z_max: {bbox.zmax}")
        # rospy.loginfo(f"Center - x: {bbox.center.x}, y: {bbox.center.y}, z: {bbox.center.z}")

def listener():
    # Initialize the ROS node
    rospy.init_node('darknet_ros_3d_bounding_boxes_listener', anonymous=True)
    
    # Subscribe to the /darknet_ros_3d/bounding_boxes topic
    rospy.Subscriber('/darknet_ros_3d/bounding_boxes', BoundingBoxes3d, bounding_boxes_callback)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    listener()