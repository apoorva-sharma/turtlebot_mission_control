#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String, Int32
from geometry_msgs.msg import PoseStamped
import tf
import numpy as np

states = {'INIT':0,
          'HUMAN_CONTROL':1,
          'LOAD_MISSION':2,
          'EXECUTE_MISSION':3
          'DONE':4}

ctrlmodes = {'MOVING':0, 'LOOKING':1, 'STOP':2}


def pose_to_xyth(pose):
    th = tf.transformations.euler_from_quaternion((pose.orientation.x,
                                                   pose.orientation.y,
                                                   pose.orientation.z,
                                                   pose.orientation.w))[2]
    return [pose.position.x, pose.position.y, th]


class Supervisor:

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)
        self.trans_listener = tf.TransformListener()
        self.trans_broad = tf.TransformBroadcaster()

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)    # rviz "2D Nav Goal"

        self.waypoint_locations = {}    # dictionary that caches the most updated locations of each mission waypoint
        self.waypoint_offset = PoseStamped()
        self.waypoint_offset.pose.position.z = .4    # waypoint is located 40cm in front of the AprilTag, facing it
        quat = tf.transformations.quaternion_from_euler(0., np.pi/2, np.pi/2)
        self.waypoint_offset.pose.orientation.x = quat[0]
        self.waypoint_offset.pose.orientation.y = quat[1]
        self.waypoint_offset.pose.orientation.z = quat[2]
        self.waypoint_offset.pose.orientation.w = quat[3]

        self.goal = None

        self.state = states['INIT']

    def rviz_goal_callback(self, msg):
        self.goal = pose_to_xyth(msg.pose)    # example usage of the function pose_to_xyth (defined above)
        # this callback does nothing... yet!

    def update_waypoints(self):
        for tag_number in self.mission:
            try:
                self.waypoint_offset.header.frame_id = "/tag_{0}".format(tag_number)
                self.waypoint_locations[tag_number] = self.trans_listener.transformPose("/map", self.waypoint_offset)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

    def run(self):
        rate = rospy.Rate(1) # 1 Hz, change this to whatever you like
        while not rospy.is_shutdown():
            self.update_waypoints()

            # Inputs
            #  has_valid_path
            #  waypoint_locations

            # Internal States
            #  states
            #  mission

            # Outputs
            #  ctrlmode = fn(state, has_valid_path)
            #  goal = fn(state, mission, has_valid_path, waypoint_locations )



            # FILL ME IN!
            # STATE MACHINE HERE
            if self.state == states['INIT']:
                self.state = states['HUMAN_CONTROL']

            elif self.state == states['HUMAN_CONTROL']:
                if not self.has_valid_path:
                    self.ctrlmode = ctrlmodes['LOOKING']
                else:
                    self.ctrlmode = ctrlmodes['MOVING']

                if len(self.waypoint_locations) == WAYPOINTS_IN_MISSION:
                    self.state = states['LOAD_MISSION']

                         
            elif self.state == states['LOAD_MISSION']:
                if not self.has_valid_path:
                    self.goal = waypoint_locations[self.mission[0]]
                    self.mission = self.mission[1:]
                    self.state = states['EXECUTE_MISSION']

            elif self.state == states['EXECUTE_MISSION']:
                self.ctrlmode = ctrlmodes['MOVING']

                if self.has_valid_path:
                    self.state = states['LOAD_MISSION']


            elif self.state == states['DONE']:
                pass
            else:
                pass

            self.modePub.publish(self.ctrlmode)

            msg = Float32MultiArray()
            msg.data = self.goal
            self.navPub.publish(msg)

            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
