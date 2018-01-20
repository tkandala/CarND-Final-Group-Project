#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

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
IMPORTANT: queue_size=1 in rospy.Subscribe() is very imporant to test it with the simulator
and VM. See more detail here.
https://discussions.udacity.com/t/solved-stuck-at-steer-value-yawcontroller/499558/3
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
ACCEL = 0.5 # increment of 1 m/s at each waypoint
MAX_SPEED = 11.0 # m/s <=> 100km/h

class WaypointUpdater(object):
    def __init__(self):

	rospy.init_node('waypoint_updater')

        self.waypoints = None
        self.current_pose = None
        self.current_pose_idx = None
        self.next_red_traffic_light_idx = -1
        self.is_stopped = False
        self.is_cold_start = True

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)        

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        rospy.spin()

    def pose_cb(self, msg):

        self.current_pose = msg.pose
        self.publish()

    def publish(self):
        position = self.current_pose.position
        if self.waypoints_loaded:
            # Find the next LOOKAHEAD_WPS waypoints
            #rospy.loginfo(position)
            for a in range(len(self.waypoints)):
                if self.waypoints[a].pose.pose.position.x > position.x:
                    self.current_pose_idx = a - 1
                    #rospy.loginfo("current_pose_idx: %s and current_vel = %s", a-1, self.waypoints[a-1].twist.twist.linear.x)
                    break
                    #rospy.loginfo("current position idx: %s", self.current_pose_idx)
                    #rospy.loginfo("waypoint index: %s", a)
                    #self.final_waypoints.append(self.waypoints[a])
            lane = Lane()
            
            # Check if Traffic light was detected and plan waypoint accordingly
            if self.next_red_traffic_light_idx != -1 and not self.is_stopped:
                #rospy.loginfo("TL detected: %s and current position: %s", self.next_red_traffic_light_idx, position.x)
                #dist_to_light = self.distance(self.waypoints, self.current_pose_idx, self.next_red_traffic_light_idx)
                safe_distance = 10 #stop at a safe distance from the light say 10 waypoints from the light
                idx_to_stop = self.next_red_traffic_light_idx - safe_distance

                # may be a good thing will be to see if we have enough distance to stop before hitting high deceleration or jerk
                # but for now we will stop right before we reach the intersection and calculate deceleration based on that
                if self.current_pose_idx <= idx_to_stop:
                    #rospy.loginfo("is within safe distance to stop")
                    self.is_stopped = True
                    #calulate the velocity of the next waypoint based on deceleration until we reach the intersection
                    for idx in range(LOOKAHEAD_WPS):
                        next_waypoint_vel = self.waypoints[idx + self.current_pose_idx].twist.twist.linear.x - (idx + 1)*ACCEL
                        if next_waypoint_vel < 0 or self.is_cold_start:
                            next_waypoint_vel = -1
                        self.set_waypoint_velocity(self.waypoints, idx + self.current_pose_idx, next_waypoint_vel)
            elif self.next_red_traffic_light_idx == -1 and self.is_stopped:
                self.is_stopped = False
                self.is_cold_start = False
                #rospy.loginfo("TL green and current position: %s", self.current_pose_idx)
                for idx in range(LOOKAHEAD_WPS):
                    next_waypoint_vel = 1.1 + self.waypoints[idx+self.current_pose_idx].twist.twist.linear.x + (idx + 1)*ACCEL
                    if next_waypoint_vel > MAX_SPEED:
                        next_waypoint_vel = MAX_SPEED
                    self.set_waypoint_velocity(self.waypoints, idx + self.current_pose_idx, next_waypoint_vel)
                    #rospy.loginfo("setting vel for idx= %s to %s", self.current_pose_idx + idx, self.waypoints[self.current_pose_idx+idx].twist.twist.linear.x)

            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time(0)
            lane.waypoints = self.waypoints[self.current_pose_idx:self.current_pose_idx+LOOKAHEAD_WPS]
            self.final_waypoints_pub.publish(lane)
        else:
            rospy.logerr("Waypoints not loaded")
        
    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        self.waypoints_loaded = True

    def traffic_cb(self, msg):

        self.next_red_traffic_light_idx = msg.data

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
