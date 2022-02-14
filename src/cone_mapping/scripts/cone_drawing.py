#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import PoseStamped, Pose2D, PoseArray, Pose
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float32MultiArray

class Visualiser:
    def __init__(self):
        self.debug_visualizer = False
        self.fig, self.ax = plt.subplots()
        self.left_plot, = plt.plot([], [], '-o', color="#2b42d9")
        self.left_detections, = plt.plot([], [], 'x', color="#99a6ff")
        self.right_plot, = plt.plot([], [], '-o', color="#d9d02b")
        self.right_detections, = plt.plot([], [], 'x', color="#fffa9c")
        self.orange_plot, = plt.plot([], [], 'o', color="#ffa444")
        self.orange_detections, = plt.plot([], [], 'x', color="#ffd4a6")
        self.unknown_plot, = plt.plot([], [], 'o', color="#919191")
        self.car_plot, = plt.plot([], [], 'ro')
        self.left_x, self.left_y = [] , []
        self.left_detection_x, self.left_detection_y = [] , []
        self.right_x, self.right_y = [] , []
        self.right_detection_x, self.right_detection_y = [] , []
        self.orange_x, self.orange_y = [] , []
        self.orange_detection_x, self.orange_detection_y = [] , []
        self.unknow_x, self.unknown_y = [] , []
        self.car_x, self.car_y = [0] , [0]

    def plot_init(self):
        self.ax.set_xlim(0, 50)
        self.ax.set_ylim(-5, 5)
        #self.ax.invert_xaxis()
        return self.left_plot
    
    def left_cone_callback(self, msg):
        self.left_x = ([cone.position.x for cone in msg.poses])
        self.left_y = ([cone.position.y for cone in msg.poses])

    def right_cone_callback(self, msg):
        self.right_x = ([cone.position.x for cone in msg.poses])
        self.right_y = ([cone.position.y for cone in msg.poses])
    
    def orange_cone_callback(self, msg):
        self.orange_x = ([cone.position.x for cone in msg.poses])
        self.orange_y = ([cone.position.y for cone in msg.poses])

    def unknown_cone_callback(self, msg):
        self.unknow_x = ([cone.position.x for cone in msg.poses])
        self.unknown_y = ([cone.position.y for cone in msg.poses])

    def car_position_callback(self, msg):
        self.car_x.append(msg.pose.position.x)
        self.car_y.append(msg.pose.position.y)

    def cone_mapping_debugger(self, msg):
        for i in range(0, len(msg.data), 4):
            if int(msg.data[i + 3]) <= 1:
                self.orange_detection_x.append(self.car_x[-1] + msg.data[i + 0])
                self.orange_detection_y.append(self.car_y[-1] + msg.data[i + 1])

            elif int(msg.data[i + 3]) == 2:
                self.right_detection_x.append(self.car_x[-1] + msg.data[i + 0])
                self.right_detection_y.append(self.car_y[-1] + msg.data[i + 1])

            elif int(msg.data[i + 3]) == 3:
                self.left_detection_x.append(self.car_x[-1] + msg.data[i + 0])
                self.left_detection_y.append(self.car_y[-1] + msg.data[i + 1])
            

    def update_plot(self, frame):
        self.left_plot.set_data(self.left_x, self.left_y)
        self.right_plot.set_data(self.right_x,self.right_y)
        self.orange_plot.set_data(self.orange_x,self.orange_y)
        self.unknown_plot.set_data(self.unknow_x,self.unknown_y)
        self.car_plot.set_data(self.car_x,self.car_y)
        if self.debug_visualizer:
            self.left_detections.set_data(self.left_detection_x,self.left_detection_y)
            self.right_detections.set_data(self.right_detection_x,self.right_detection_y)
            self.orange_detections.set_data(self.orange_detection_x,self.orange_detection_y)
        return self.left_plot


rospy.init_node('realtime_visualizer')
vis = Visualiser()
sub = rospy.Subscriber('/cone_left', PoseArray, vis.left_cone_callback)
sub = rospy.Subscriber('/cone_right', PoseArray, vis.right_cone_callback)
sub = rospy.Subscriber('/cone_orange', PoseArray, vis.orange_cone_callback)
sub = rospy.Subscriber('/cone_unknown', PoseArray, vis.unknown_cone_callback)
sub = rospy.Subscriber('/pose_stamped', PoseStamped, vis.car_position_callback)

sub = rospy.Subscriber('/sensor_fusion/output', Float32MultiArray, vis.cone_mapping_debugger)

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show(block=True) 
