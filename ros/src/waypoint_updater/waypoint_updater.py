#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import numpy as np
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

LOOKAHEAD_WPS = 50  # Number of waypoints we will publish.
MAX_DECEL = 0.5  # acceleration should not exceed 10 m/s^2 and jerk should not exceed 10 m/s^3.
CONST_DECEL = 1 / LOOKAHEAD_WPS  # to smooth braking, as discussed in video


class WaypointUpdater(object):
    def __init__(self):

        # start this ros node
        rospy.init_node('waypoint_updater')
        rospy.loginfo("Waypoint_updater node is initialized")

        # all the subscribing topics
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)  # not using it

        # publishes final_waypoints topic
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_lane = None
        self.pose = None
        self.stopline_wp_idx = -1
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        # publish the waypoints at a certain rate to the waypoint_follower
        self.spin()

    def spin(self):
        # the waypoint_follower (consumer) runs at ~50Hz in dbw_node, so less than that is expected
        # this gives controls over the publishing frequency
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                # get closest waypoint and publish waypoints after that
                # closest_waypoint_idx = self.get_closest_waypoint_idx()
                # self.publish_waypoints(closest_waypoint_idx)
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]  # take only indices

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        # if the closest waypoint is behind the car, take the next one
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        # get the final lane of ego and publish it
        lane = Lane()

        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS  # if the value is more than length of list, python wraps cuts it
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):  # no traffic lights
            lane.waypoints = base_waypoints
        else:  # traffic light detected
            lane.waypoints = self.generate_deceleration_waypoints(base_waypoints, closest_idx)

        return lane

    def generate_deceleration_waypoints(self, waypoints, closest_idx):
        # use some information from the base_waypoints and generate a new set of waypoints
        temp = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            # 2 or 3 waypoints back from the stop line in front of the traffic light so that front of car stops at line
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist) + (i * CONST_DECEL)  # to avoid steep decel by the sqrt function alone

            # for very low velocity, just stop
            if vel < 1.0:
                vel = 0.

            # to smooth out the deceleration, if the distance to the stop line is high, we keep the original velocity
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)

        return temp

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # self.base_waypoints = waypoints

        self.base_lane = waypoints

        # check that waypoints_2d has already been initialized before the subscriber is called
        # o/w there may be a race condition
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y]
                                 for waypoint in waypoints.waypoints]
            # find the closed waypoint nearest to
            self.waypoint_tree = KDTree(self.waypoints_2d)

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
        # Computes the distance between two waypoints in a list along the piecewise linear arc
        # connecting all waypoints between the two.
        dist = 0

        def dl(a, b):
            return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y)**2 + (a.z - b.z)**2)

        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
