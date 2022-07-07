#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from state_estimator_pkg.srv import EstState, EstStateResponse

import copy
import math
from scipy.spatial.transform import Rotation
from datetime import datetime
import csv

# File Overview:
# - subscribes to Pose topic for each object tracked by Vicon
# - each subscriber has a callback function that stashes received pose in a global variable
# - controller requests state vector periodically by querying the EstState service
# - service returns the most recently collected poses from the Vicon topics

# Vicon state vector:
#   [Marmot x;
#   Marmot y;
#   Marmot theta;
#   Ped1 x;
#   Ped1 y;
#   Ped2 x;
#   ...]
# - length = 3 + 2*n_peds

class StateEstimator:
    current_veh_msg = []
    current_ped1_msg = []
    # current_ped2_msg = []
    # current_ped3_msg = []
    # current_ped4_msg = []

    record_hist = False
    saved_hist = False

    hist_veh_msg = []
    hist_ped1_msg = []
    # hist_ped2_msg = []
    # hist_ped3_msg = []
    # hist_ped4_msg = []

    def __init__(self):
        # TO-DO: change topic back to Marmot
        self.vrpn_sub_marmot_pose = rospy.Subscriber(
            "/car/vrpn_client_ros/vrpn_client_node/Wand/pose", 
            PoseStamped,
            callback=self.store_current_msg,
            callback_args=0,
            queue_size=1)

        self.vrpn_sub_ped1_pose = rospy.Subscriber(
            "/car/vrpn_client_ros/vrpn_client_node/ADCL_Ped1/pose", 
            PoseStamped,
            callback=self.store_current_msg,
            callback_args=1,
            queue_size=1)

        # self.vrpn_sub_ped2_pose = rospy.Subscriber(
        #     "/car/vrpn_client_ros/vrpn_client_node/ADCL_Ped2/pose", 
        #     PoseStamped,
        #     self.store_pose_msg,
        #     queue_size=1)

        # self.vrpn_sub_ped3_pose = rospy.Subscriber(
        #     "/car/vrpn_client_ros/vrpn_client_node/ADCL_Ped3/pose", 
        #     PoseStamped,
        #     self.store_pose_msg,
        #     queue_size=1)

        # self.vrpn_sub_ped4_pose = rospy.Subscriber(
        #     "/car/vrpn_client_ros/vrpn_client_node/ADCL_Ped4/pose", 
        #     PoseStamped,
        #     self.store_pose_msg,
        #     queue_size=1)

        self.est_state_srv = rospy.Service(
            "/car/state_estimator/get_est_state",
            EstState,
            self.estimate_state)

        return

    # subscriber callback function
    def store_current_msg(self, pose_msg, vrpn_object):
        if vrpn_object == 0:
            self.current_veh_msg = copy.deepcopy(pose_msg)
            if self.record_hist == True:
                self.hist_veh_msg.append(pose_msg)
        elif vrpn_object == 1:
            self.current_ped1_msg = copy.deepcopy(pose_msg)
            if self.record_hist == True:
                self.hist_ped1_msg.append(pose_msg)
        # elif vrpn_object == 2:
        #     self.current_ped2_msg = copy.deepcopy(pose_msg)
        #     if record_hist == True:
        #         hist_ped2_msg.append(pose_msg)
        # elif vrpn_object == 3:
        #     self.current_ped3_msg = copy.deepcopy(pose_msg)
        #     if record_hist == True:
        #         hist_ped3_msg.append(pose_msg)
        # elif vrpn_object == 4:
        #     self.current_ped4_msg = copy.deepcopy(pose_msg)
        #     if record_hist == True:
        #         hist_ped4_msg.append(pose_msg)
        
        return

    # function called by service
    def estimate_state(self, req):
        self.record_hist = req.record

        if self.record_hist == False and self.saved_hist == False and len(self.hist_veh_msg) > 0: 
            self.save_s_hist()

        current_state = [0]*(3 + 2*(1))
        current_state[0:3] = self.veh_state(self.current_veh_msg)
        current_state[3:5] = self.ped_state(self.current_ped1_msg)
        # current_state[5:7] = self.ped_state(self.current_ped2_msg)
        # current_state[7:9] = self.ped_state(self.current_ped3_msg)
        # current_state[9:11] = self.ped_state(self.current_ped4_msg)

        return EstStateResponse(current_state)

    # converts ROS PoseStamped message to regular (x,y,theta) variables
    def veh_state(self, pose_msg):
        ori = pose_msg.pose.orientation
        rot = Rotation.from_quat([ori.x, ori.y, ori.z, ori.w])
        rot_euler = rot.as_euler('xyz', degrees=False)
        theta = rot_euler[2]

        y_cal = -0.162  # m from Vicon model center to rear axis
        x = pose_msg.pose.position.x + y_cal*math.cos(theta)
        y = pose_msg.pose.position.y + y_cal*math.sin(theta)

        s = [x, y, theta]

        return s

    def ped_state(self, pose_msg):
        x = pose_msg.pose.position.x
        y = pose_msg.pose.position.y

        s = [x, y]

        return s

    # saves full Vicon history to csv file, called once at end of execution
    # TO-DO: manage histories from all objects
    #   - histories not necessarily same length, same time steps (hmm...)
    #   - see what each length is, might be close enough to not cause issues
    #   - probably easiest to save in separate files, pull together in analysis script
    def save_s_hist(self):
        # veh_secs = datetime.fromtimestamp(self.current_veh_msg.header.stamp.secs)  
        # veh_nsecs = datetime.fromtimestamp(self.current_veh_msg.header.stamp.nsecs)
        # ped1_secs = datetime.fromtimestamp(self.current_ped1_msg.header.stamp.secs)  
        # ped1_nsecs = datetime.fromtimestamp(self.current_ped1_msg.header.stamp.nsecs)

        # print(veh_secs)
        # print(veh_nsecs)
        # print(ped1_secs)
        # print(ped1_nsecs)

        # TO-DO: add datetime to file name
        # TO-DO: add time stamp to each entry

        # veh history
        f = open("/home/adcl/catkin_ws/src/marmot-ros/controller_pkg/histories/veh_hist_vrpn.csv", 'w')
        writer = csv.writer(f)
        for msg_k in self.hist_veh_msg:
            s_k = self.veh_state(msg_k)
            writer.writerow(s_k)
        f.close()

        # ped1 history
        f = open("/home/adcl/catkin_ws/src/marmot-ros/controller_pkg/histories/ped1_hist_vrpn.csv", 'w')
        writer = csv.writer(f)
        for msg_k in self.hist_ped1_msg:
            s_k = self.ped_state(msg_k)
            writer.writerow(s_k)
        f.close()
        
        self.saved_hist = True
        print("save complete")

        return

if __name__ == '__main__':
    try:
        rospy.init_node("state_estimator")
        StateEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass