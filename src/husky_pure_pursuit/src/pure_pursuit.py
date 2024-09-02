#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
import math

L = 2.875  # Wheelbase of the vehicle
Kdd = 4.0  # Lookahead distance multiplier
min_lookahead = 0.5  # Minimum lookahead distance
target_velocity = 0.5  # Target velocity (m/s)
acceleration_rate = 0.1  # Rate of acceleration (m/s per cycle)
max_angular_velocity = 1.0  # Maximum angular velocity (rad/s)
waypoint_tolerance = 0.5  # Tolerance to consider the robot has reached the waypoint (meters)

class PurePursuit:
    def __init__(self):
        self.cx = []
        self.cy = []
        self.current_state = None
        self.local_plan = []  # Store the robot's actual positions

        rospy.Subscriber('/trajectory', Path, self.path_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.local_plan_pub = rospy.Publisher('/local_plan', Path, queue_size=10)

        rospy.loginfo("PurePursuit controller initialized.")

    def path_callback(self, msg):
        self.cx = [pose.pose.position.x for pose in msg.poses]
        self.cy = [pose.pose.position.y for pose in msg.poses]
        rospy.loginfo(f"Received trajectory with {len(self.cx)} waypoints.")

    def odom_callback(self, msg):
        self.current_state = self.extract_state(msg)
        if self.cx and self.cy:
            self.update_local_plan()
            self.compute_and_publish_cmd()

    def extract_state(self, odom):
        state = {
            'x': odom.pose.pose.position.x,
            'y': odom.pose.pose.position.y,
            'yaw': self.get_yaw_from_quaternion(odom.pose.pose.orientation),
            'v': np.sqrt(odom.twist.twist.linear.x**2 + odom.twist.twist.linear.y**2)
        }
        return state

    def get_yaw_from_quaternion(self, orientation):
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1])
        cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def compute_and_publish_cmd(self):
        if len(self.cx) < 1 or len(self.cy) < 1:
            rospy.logwarn("Trajectory not received yet or empty.")
            return

        ind, tx, ty = self.get_target_wp_index()

        # Check if we are at the last waypoint
        if ind >= len(self.cx) - 1:
            # Check if we're close enough to the last waypoint
            distance_to_last_wp = np.hypot(self.current_state['x'] - self.cx[-1], self.current_state['y'] - self.cy[-1])
            if distance_to_last_wp < waypoint_tolerance:
                rospy.loginfo("Reached final waypoint. Stopping the robot.")
                self.stop_robot()
                return

        rospy.loginfo(f"Target waypoint index: {ind}, tx={tx}, ty={ty}")

        ld = max(Kdd * self.current_state['v'], min_lookahead)  # Adjust lookahead distance
        alpha = math.atan2(ty - self.current_state['y'], tx - self.current_state['x']) - self.current_state['yaw']
        steer_angle = self.calc_steering_angle(alpha, ld)
        rospy.loginfo(f"Steering angle: {steer_angle} radians")

        # Set velocity with acceleration towards target velocity
        velocity_cmd = Twist()
        velocity_cmd.linear.x = min(self.current_state['v'] + acceleration_rate, target_velocity)

        # Apply maximum angular velocity limit
        velocity_cmd.angular.z = max(min(steer_angle, max_angular_velocity), -max_angular_velocity)
        self.cmd_pub.publish(velocity_cmd)
        rospy.loginfo(f"Published velocity command: linear.x={velocity_cmd.linear.x}, angular.z={velocity_cmd.angular.z}")

    def get_target_wp_index(self):
        # Find the closest waypoint
        dx = np.array(self.cx) - self.current_state['x']
        dy = np.array(self.cy) - self.current_state['y']
        dist = np.hypot(dx, dy)
        min_index = np.argmin(dist)

        # Look ahead a fixed number of waypoints or based on a distance threshold
        look_ahead_wps = 10  # Look ahead this many waypoints
        if min_index + look_ahead_wps < len(self.cx):
            target_index = min_index + look_ahead_wps
        else:
            target_index = len(self.cx) - 1

        tx = self.cx[target_index]
        ty = self.cy[target_index]
        return target_index, tx, ty

    def calc_steering_angle(self, alpha, ld):
        delta = math.atan2(2 * L * math.sin(alpha), ld)
        delta = max(min(delta, 1.0), -1.0)  # Clamp to [-1.0, 1.0]
        return delta

    def update_local_plan(self):
        # Record the current position
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = self.current_state['x']
        pose.pose.position.y = self.current_state['y']
        self.local_plan.append(pose)

        # Publish the local plan as a Path message
        path_msg = Path()
        path_msg.header.frame_id = "base_link"
        path_msg.header.stamp = rospy.Time.now()
        path_msg.poses = self.local_plan
        self.local_plan_pub.publish(path_msg)
        rospy.loginfo(f"Published local plan with {len(self.local_plan)} poses.")

    def stop_robot(self):
        """Publish a zero velocity to stop the robot."""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_pub.publish(stop_cmd)
        rospy.loginfo("Robot stopped.")

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_controller')
    pp = PurePursuit()
    rospy.spin()
