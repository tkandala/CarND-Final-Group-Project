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
ACCEL = 1.0 # increment of 1 m/s at each waypoint
MAX_SPEED = 27.0 # m/s <=> 100km/h

class WaypointUpdater(object):
    def __init__(self):

	rospy.init_node('waypoint_updater')

        self.waypoints = None
        self.current_pose = None
        self.current_pose_idx = None

        

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)        

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        rospy.spin()

    def pose_cb(self, msg):
        if self.waypoints is None:
            return
        
        #rospy.loginfo("current pose: %s", msg)
        self.current_pose = msg.pose

        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        distances = [ dl(msg.pose.position, waypoint.pose.pose.position) for waypoint in self.waypoints ]
        increasing = distances[1] > distances[0]
        start_idx = None
        if increasing:
            start_idx = 0
        else:
            min_distance = 1e9
            min_idx = -1
            for (idx, distance) in enumerate(distances):
                if distance < min_distance:
                    min_distance = distance
                    min_idx = idx
                elif idx > 0 and idx - 1 == min_idx and distance > min_distance:
                    start_idx = idx
                    break
        self.current_pose_idx = start_idx - 1
        waypoints = self.waypoints[start_idx:start_idx+LOOKAHEAD_WPS]
        self.publish(waypoints)

    def publish(self, waypoints):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)
        
    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement

        #rospy.loginfo("Traffic light detected: %s", msg)

        next_traffic_light_idx = msg.data

        if next_traffic_light_idx != -1:
            # get current position index and calculate the distance to the red light for slowing down

            # distance to traffic light
            dist_to_light = self.distance(self.waypoints, self.current_pose_idx, next_traffic_light_idx)
            safe_distance = 10 #stop at a safe distance from the light say 10 waypoints from the light
            idx_to_stop = next_traffic_light_idx - safe_distance

            # may be a good thing will be to see if we have enough distance to stop before hitting high deceleration or jerk
            # but for now we will stop right before we reach the intersection and calculate deceleration based on that
            if self.current_pose_idx <= (next_traffic_light_idx - idx_to_stop):
                dist_to_stop = self.distance(self.waypoints, self.current_pose_idx, idx_to_stop)
                current_vel = self.get_waypoint_velocity(self.waypoints[self.current_pose_idx])
                req_dcln = -(current_vel * current_vel)/dist_to_stop

                #calulate the velocity of the next waypoint based on deceleration until we reach the intersection
                for idx in range(idx_to_stop - self.current_pose_idx):
                    start_idx = idx
                    dist_to_next_waypoint = self.distance(self.waypoints, start_idx, start_idx + 1)
                    next_waypoint_vel = (req_dcln * dist_to_next_waypoint / current_vel) + current_vel
                    self.set_waypoint_velocity(self.waypoints, start_idx + 1, next_waypoint_vel)

            else:
                current_vel = self.get_waypoint_velocity(self.waypoints[self.current_pose_idx])

                # Calulate the velocity of the next waypoint based on incremental speed
                # until we reach the max speed
                for wp in self.final_waypoints:
                    if current_vel > MAX_SPEED:                
                        current_vel = current_vel - ACCEL # min(MAX_SPEED, current_vel+ACCEL)
                    else:
                        current_vel = current_vel + ACCEL # max(MAX_SPEED, current_vel-ACCEL)
                    self.set_waypoint_velocity(self.waypoints, wp, current_vel) #Accelelerate to max speed

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
