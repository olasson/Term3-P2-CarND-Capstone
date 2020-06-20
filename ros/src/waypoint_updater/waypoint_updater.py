#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32

import math
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''


# Constants
LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number Original value: 200
ROSPY_RATE = 50 #Rate for loop()
MAX_WP_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
       

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.waypoints_2D = None
        self.waypoints_tree = None
        
        self.pose = None
        
        self.stopline_wp_idx = -1
        self.stop_wp_idx = -1
        
        
        
        #rospy.spin()
        self.loop()
    
    
    def loop(self):
        """
        As described in Lesson 6: Project Programming a Real Self-Driving Car
        """
        rate = rospy.Rate(ROSPY_RATE)
        while not rospy.is_shutdown():
            if self.pose and self.waypoint_tree:
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()
                
        
    
    def get_closest_waypoint_idx(self):
        """
        As described in Lesson 6: Project Programming a Real Self-Driving Car
        """
        
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_waypoint_idx = self.waypoint_tree.query([x, y], 1)[1]
        
        # Extract coordinates ahead and behind car
        ahead_coord = self.waypoints_2D[closest_waypoint_idx]
        behind_coord = self.waypoints_2D[closest_waypoint_idx - 1]
        
        ahead_vect = np.array(ahead_coord)
        behind_vect = np.array(behind_coord)
        pos_vect = np.array([x, y])
        
        # The dot product checks if the closest waypoint is ahead or behind the car
        check_val = np.dot(ahead_vect - behind_vect, pos_vect - ahead_vect)
        
        # If the waypoint is behind us, take the closest index + 1, 
        # but modulo to avoid issues if the car is finishing a lap
        if check_val > 0:
            closest_waypoint_idx = (closest_waypoint_idx + 1) % len(self.waypoints_2D)
        return closest_waypoint_idx
    
    def publish_waypoints(self, closest_wp_idx):
        self.final_waypoints_pub.publish(self.generate_lane(closest_wp_idx))
    
    def generate_lane(self, closest_wp_idx):
        lane = Lane()
        #closest_wp_idx = self.get_closest_waypoint_idx()
        farthest_wp_idx = closest_wp_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_wp_idx : farthest_wp_idx]
        
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_wp_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_wp_idx)
            
        return lane
    
    def decelerate_waypoints(self, waypoints, closest_wp_idx):
        decelerated_waypoints = []
        for i, wp in enumerate(waypoints):
            wp_tmp = Waypoint()
            wp_tmp.pose = wp.pose
            
            stop_wp_idx = max(self.stopline_wp_idx - closest_wp_idx - 2, 0) 
            dist_wp = self.distance(waypoints, i, stop_wp_idx)
            vel = math.sqrt(2 * MAX_WP_DECEL * dist_wp)
            if vel < 1.:
                vel = 0.
                
            wp_tmp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            decelerated_waypoints.append(wp_tmp)
                    
        return decelerated_waypoints
        
    def pose_cb(self, msg):
        """
        Store the car's pose
        """
        self.pose = msg
   
    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2D:
            # Extract 2D waypoints for use in KDTree (as described in Lesson 6: Project Programming a Real Self-Driving Car)
            self.waypoints_2D = [[waypoint.pose.pose.position.x , waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2D)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
