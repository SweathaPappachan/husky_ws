#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def load_trajectories(file_path):
    trajectories = np.load(file_path, allow_pickle=True)
    return trajectories

def select_trajectory(trajectories):
    num_trajectories = len(trajectories)
    rospy.loginfo(f"Select a trajectory number between 1 and {num_trajectories}:")
    
    trajectory_number = int(input(f"Enter trajectory number (0-{num_trajectories - 1}): "))
    
    if trajectory_number < 0 or trajectory_number >= num_trajectories:
        rospy.logwarn("Invalid trajectory number. Selecting trajectory 1 by default.")
        trajectory_number = 0
    
    rospy.loginfo(f"Selected trajectory {trajectory_number + 1}")
    return trajectories[trajectory_number]

def publish_trajectory(trajectory, frame_id):
    path_msg = Path()
    path_msg.header.frame_id = frame_id  # Ensure the frame_id is set here

    for point in trajectory:
        pose = PoseStamped()
        pose.header.frame_id = frame_id  # Set frame_id for each PoseStamped
        pose.pose.position.x = point[0] 
        pose.pose.position.y = point[1]
        path_msg.poses.append(pose)
    
    return path_msg

def trajectory_publisher():
    rospy.init_node('trajectory_publisher')
    
    # Set the file path and frame_id directly
    npy_file_path = "/home/tarun/husky_ws/src/husky_pure_pursuit/src/31_obs1_test.npy"  # Update with the correct file path
    frame_id = "odom"  # Set the desired frame_id here
    
    pub = rospy.Publisher('/trajectory', Path, queue_size=10)

    trajectories = load_trajectories(npy_file_path)
    selected_trajectory = select_trajectory(trajectories)
    
    # Publish the trajectory once
    path_msg = publish_trajectory(selected_trajectory, frame_id)
    path_msg.header.stamp = rospy.Time.now()
    pub.publish(path_msg)
    rospy.loginfo("Published trajectory.")

    rospy.spin()  # Keep the node alive

if __name__ == '__main__':
    try:
        trajectory_publisher()
    except rospy.ROSInterruptException:
        pass

