#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

from std_msgs.msg import Int32
from copy import deepcopy

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

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size = 1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size = 1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size = 1)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb, queue_size = 1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None # intialized only once, from publisher.
        self.next_waypoints = None
        self.tl_waypoint_idxs = None
        self.cur_pose = None

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        rospy.loginfo("got current_pose: in pose_cb()!")

        self.cur_pose = msg

        nearest_wp = self.get_closest_wp_index(msg.pose)

        lane = Lane()
        lane.waypoints = self.base_waypoints[nearest_wp:nearest_wp + LOOKAHEAD_WPS]
        lane.header.stamp = rospy.get_rostime()
        lane.header.frame_id = '/lane'

        self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        rospy.loginfo("got base_waypoints: in waypoints_cb()!")
        self.base_waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        rospy.loginfo("got traffic_waypoint: in traffic_cb()!")
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        rospy.loginfo("got obstacle_waypoint: in obstacle_cb()!")
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

    
    def get_closest_wp_index(self, pose):
        min_i = 0;

        bas_pos = self.base_waypoints[0].pose.pose.position
        min_d = self.square_distance(pose.position.x, pose.position.y, bas_pos.x, bas_pos.y)
        
        for (i, wp) in enumerate(self.base_waypoints):
            position = wp.pose.pose.position
            d = self.square_distance(pose.position.x, pose.position.y, position.x, position.y)
            if min_d > d:
                min_d = d
                min_i = i

        return min_i

    def square_distance(self, x1, y1, x2, y2):
        return (x2 - x1) ** 2.0 + (y2 - y1) ** 2.0

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
