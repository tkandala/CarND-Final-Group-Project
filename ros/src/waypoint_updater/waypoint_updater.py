#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        rospy.loginfo('pose_cb - x:%s y:%s z:%s', msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

        index = 0
        shortest_index = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        shortest_distance = dl(msg.pose.position, self.waypoints.waypoints[index].pose.pose.position)
        for wp in self.waypoints.waypoints:
            distance = dl(msg.pose.position, wp.pose.pose.position)
            if distance < shortest_distance and msg.pose.position < wp.pose.pose.position:
                shortest_distance = distance
                shortest_index = index
            index = index+1
        rospy.loginfo('shortest_distance = %s index = %s', distance, shortest_index)
        waypoints = []
        array_len = len(self.waypoints.waypoints)
        for i in range(LOOKAHEAD_WPS):
            if (shortest_index + i) < array_len:
                wp_add = self.waypoints.waypoints[i+shortest_index]
            else:
                wp_add = self.waypoints.waypoints[shortest_index+i-array_len]
            waypoints.append(wp_add)
            #rospy.loginfo('pose_cb - linear.x:%s linear.y:%s linear.z:%s array_len:%s',wp_add.twist.twist.linear.x, wp_add.twist.twist.linear.y, wp_add.twist.twist.linear.z, array_len)
        self.publish(waypoints)

    def publish(self, waypoints):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, waypoints):
#        for i in range(5):
#            position = waypoints.waypoints[i].pose.pose.position
#            rospy.loginfo('waypoints_cb - x:%s y:%s z:%s', position.x, position.y, position.z)
        # TODO: Implement
        self.waypoints = waypoints

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
