#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import math

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # rospy.Subscriber('/traffic_waypoint ', Lane, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.all_waypoints = None
        self.all_waypoints_2D = None
        self.waypoint_KD_tree = None
        self.pose = None
        self.loop()
    
    def loop(self):
        rate = rospy.Rate(50)
        while rospy.is_shutdown() is False:
            if self.pose and self.all_waypoints:
                curr_closest_pos = self.get_closest_waypoint_pos()
                self.publish_waypoints(curr_closest_pos)
                rate.sleep()

    def publish_waypoints(self, closest_pos):
        lane = Lane()
        lane.header = self.all_waypoints.header
        lane.waypoints = self.all_waypoints.waypoints[closest_pos:closest_pos+LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)

    def get_closest_waypoint_pos(self):
        curr_x = self.pose.pose.position.x
        curr_y = self.pose.pose.position.y
        closest_pos = self.waypoint_KD_tree.query([curr_x, curr_y], 1)[1]

        closest_pos_coord = self.all_waypoints_2D[closest_pos]
        previous_pos_cord = self.all_waypoints_2D[closest_pos-1]

        closest_pos_coord = np.array(closest_pos_coord)
        previous_pos_cord = np.array(previous_pos_cord)
        current_pos_cord = np.array([curr_x, curr_y])

        dot_prod = np.dot(closest_pos_coord - previous_pos_cord, current_pos_cord-closest_pos_coord)
        if dot_prod > 0: #make sure closest waypoint is ahead of ego vehicle
            closest_pos = (closest_pos+1) % len(self.all_waypoints_2D)
        return closest_pos

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.all_waypoints = waypoints
        self.all_waypoints_2D = []
        if not self.all_waypoints_2D:
            for temp_waypoint in waypoints.waypoints:
                self.all_waypoints_2D.append([temp_waypoint.pose.pose.position.x, temp_waypoint.pose.pose.position.y])
            self.waypoint_KD_tree = KDTree(self.all_waypoints_2D)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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
