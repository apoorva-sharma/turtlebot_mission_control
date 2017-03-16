#!/usr/bin/env python

# FILL ME IN!


# Needs to know:
#  - goal position
#  - robot's pose

# SUBSCRIBES TO: 
#  - /turtlebot_mission/position_goal
#  - /turtlebot_mission/ctrl_mode

# PUBLISHES:
#  - /cmd_vel_mux/navi_input

# copy from hw1

import rospy
from std_msgs.msg import Float32MultiArray, Int32
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
import tf
import numpy as np

MOVING = 0
LOOKING = 1
STOP = 2

class Controller:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        rospy.Subscriber('/turtlebot_mission/position_goal', Float32MultiArray, self.GScallback )
        rospy.Subscriber('/turtlebot_mission/ctrl_mode', Int32, self.ctrl_callback)
        #rospy.Subscriber('/gazebo/model_states', ModelStates, self.MScallback)
        self.trans_listener = tf.TransformListener()
        self.pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.th_g = 0.0
        self.x_g = 0.0
        self.y_g = 0.0
        self.mode = 2


    def ctrl_callback(self, data):
        self.mode = data.data

    def GScallback(self, data):
        layout = data.layout
        arr = data.data
        self.x_g = arr[0]
        self.y_g = arr[1]
        self.th_g = arr[2]

        print "x:", self.x_g
        print "y:", self.y_g
        print "g:", self.th_g
    
    # def MScallback(self, data):
    #     pose = data.pose[data.name.index("mobile_base")]
    #     twist = data.twist[data.name.index("mobile_base")]
    #     self.x = pose.position.x
    #     self.y = pose.position.y
    #     quaternion = (
    #         pose.orientation.x,
    #         pose.orientation.y,
    #         pose.orientation.z,
    #         pose.orientation.w)
    #     euler = tf.transformations.euler_from_quaternion(quaternion)
    #     self.theta = euler[2]

    def wrapToPi(self,a):
        b = a
        for i in range(len(a)):
            if a[i] < -np.pi or a[i] > np.pi:
                b[i] = ((a[i]+np.pi) % (2*np.pi)) - np.pi
        return b

    def get_ctrl_output(self):
        x_g = self.x_g
        y_g = self.y_g
        th_g = self.th_g

        try:
            (translation,rotation) = self.trans_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rotation)
            self.x = translation[0]
            self.y = translation[1]
            self.theta = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # tf doesn't know where the robot is
            translation = (0,0,0)
            rotation = (0,0,0,1)

        # use self.x self.y and self.theta to compute the right control input here
        rho = np.sqrt((self.x-x_g)**2 + (self.y-y_g)**2)
        bearingToGoal = np.arctan2((y_g-self.y),(x_g-self.x))
        alpha, delta = self.wrapToPi([bearingToGoal - self.theta, bearingToGoal - th_g])

        # use these coordinates to comput control inputs
        k1 = 0.5
        k2 = 0.7
        k3 = 0.8

        cmd_x_dot = k1*rho*np.cos(alpha)
        cmd_theta_dot = k2*alpha + k1*np.sin(alpha)*np.cos(alpha)/alpha*(alpha + k3*delta)        

        # end of what you need to modify
        cmd = Twist()
        cmd.linear.x = cmd_x_dot
        cmd.angular.z = cmd_theta_dot
        return cmd

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            ctrl_output = Twist()
            rospy.logwarn(self.mode)
            if self.mode == MOVING:
                ctrl_output = self.get_ctrl_output()
            elif self.mode == LOOKING:
                ctrl_output.linear.x = 0.0
                ctrl_output.angular.z = 0.45 #maybe change this value to go faster

            self.pub.publish(ctrl_output)
            rate.sleep()

if __name__ == '__main__':
    ctrl = Controller()
    ctrl.run()

