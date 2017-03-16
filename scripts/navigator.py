#!/usr/bin/env python

# Needs to know:
#  - map
#  - goal position
#  - robot's pose

# SUBSCRIBES TO: 
#  - /turtlebot_mission/nav_goal

# PUBLISHES:
#  - /turtlebot_mission/path
#  - /turtlebot_mission/position_goal

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import numpy as np
import matplotlib.pyplot as plt
import tf
from std_msgs.msg import Float32MultiArray, Bool
from astar import AStar, StochOccupancyGrid2D, StochOccupancyGrid2D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Navigator:

    def __init__(self):
        rospy.init_node('navigator', anonymous=True)

        self.plan_resolution = 0.25
        self.plan_horizon = 15

        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0,0]
        self.map_probs = []
        self.occupancy = None

        self.nav_sp = None

        self.path = []

        self.trans_listener = tf.TransformListener()

        rospy.Subscriber("map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("map_metadata", MapMetaData, self.map_md_callback)
        rospy.Subscriber("/turtlebot_mission/nav_goal", Float32MultiArray, self.nav_sp_callback)

        self.pose_sp_pub = rospy.Publisher('/turtlebot_mission/position_goal', Float32MultiArray, queue_size=10)
        self.nav_path_pub = rospy.Publisher('/turtlebot_mission/path_goal', Path, queue_size=10)
        self.has_valid_path = rospy.Publisher('/turtlebot_mission/valid_path', Bool, queue_size=10)

    def map_md_callback(self,msg):
        self.map_width = msg.width
        self.map_height = msg.height
        self.map_resolution = msg.resolution
        self.map_origin = (msg.origin.position.x,msg.origin.position.y)

    def map_callback(self,msg):
        self.map_probs = msg.data
        if self.map_width>0 and self.map_height>0 and len(self.map_probs)>0:
            self.occupancy = StochOccupancyGrid2D(self.map_resolution,
                                                  self.map_width,
                                                  self.map_height,
                                                  self.map_origin[0],
                                                  self.map_origin[1],
                                                  int(self.plan_resolution / self.map_resolution) * 2,
                                                  self.map_probs)

    def nav_sp_callback(self,msg):
        self.nav_sp = (msg.data[0],msg.data[1],msg.data[2])
        self.send_pose_sp()

    def send_pose_sp(self):
        try:
            (robot_translation,robot_rotation) = self.trans_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            self.has_robot_location = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            robot_translation = (0,0,0)
            robot_rotation = (0,0,0,1)
            self.has_robot_location = False

        if self.occupancy and self.has_robot_location and self.nav_sp:
            state_min = (-int(round(self.plan_horizon)), -int(round(self.plan_horizon)))
            state_max = (int(round(self.plan_horizon)), int(round(self.plan_horizon)))
            x_init = (round(robot_translation[0]/self.plan_resolution)*self.plan_resolution, 
                        round(robot_translation[1]/self.plan_resolution)*self.plan_resolution)
            x_goal = (round(self.nav_sp[0]/self.plan_resolution)*self.plan_resolution, round(self.nav_sp[1]/self.plan_resolution)*self.plan_resolution)
            
            if np.linalg.norm(np.array(x_goal) - np.array(x_init)) < 0.1:
                if np.linalg.norm(np.array(robot_translation[0:1]) - np.array(self.nav_sp[0:1])) < 0.1:
                    rospy.logwarn("I have reached my target" + str(self.nav_sp))
                    self.has_valid_path.publish(False)
                    return
                msg = Float32MultiArray()
                msg.data = self.nav_sp
                self.pose_sp_pub.publish(msg)
                self.has_valid_path.publish(True)
                return


            astar = AStar(state_min,state_max,x_init,x_goal,self.occupancy,self.plan_resolution)

            rospy.loginfo("Computing navigation plan")
            if astar.solve():
                
                # a naive path follower we could use

                # make angle the angle between point before and point after
                if(len(astar.path) > 2):
                    angle = np.arctan2( astar.path[2][1] - astar.path[0][1], 
                                        astar.path[2][0] - astar.path[0][0])
                else:
                    angle = self.nav_sp[2]

                pose_sp = (astar.path[1][0],astar.path[1][1],angle)
                msg = Float32MultiArray()
                msg.data = pose_sp
                self.pose_sp_pub.publish(msg)
                rospy.logwarn(robot_translation)
                # astar.plot_path()
                
                path_msg = Path()
                path_msg.header.frame_id = 'map'
                for state in astar.path:
                    pose_st = PoseStamped()
                    pose_st.pose.position.x = state[0]
                    pose_st.pose.position.y = state[1]
                    pose_st.header.frame_id = 'map'
                    path_msg.poses.append(pose_st)
                self.nav_path_pub.publish(path_msg)
                self.has_valid_path.publish(True)


            else:
                rospy.logwarn("Could not find path")
                self.has_valid_path.publish(False)

        if not self.occupancy:
            rospy.logwarn('No occupancy grid')



    def run(self):
        rospy.spin()

if __name__ == '__main__':
    nav = Navigator()
    nav.run()


