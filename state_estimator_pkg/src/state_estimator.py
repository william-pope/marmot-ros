#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from state_estimator_pkg.srv import EstState, EstStateResponse

import copy
import math
from scipy.spatial.transform import Rotation
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
# - should be single state vector that is being continuously updated by each callback
#   - need to make sure two callbacks don't write at same time (think callbacks lock each other out?)
#   - could store in separate variables (as PoseStamped), combine into state vector when controller requests state
#   - 

class StateEstimator:
    global current_veh_msg
    global current_ped1_msg
    # global current_ped2_msg
    # global current_ped3_msg
    # global current_ped4_msg

    # global record
    # global saved
    # global pose_msg_hist

    def __init__(self):
        # global record
        # global saved
        # global pose_msg_hist
        
        # record = False
        # saved = False

        # pose_msg_hist = []

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
        global current_veh_msg
        global current_ped1_msg
        # global current_ped2_msg
        # global current_ped3_msg
        # global current_ped4_msg

        if vrpn_object == 0:
            current_veh_msg = copy.deepcopy(pose_msg)
        elif vrpn_object == 1:
            current_ped1_msg = copy.deepcopy(pose_msg)
        # elif vrpn_object == 2:
        #     current_ped2_msg = copy.deepcopy(pose_msg)
        # elif vrpn_object == 3:
        #     current_ped3_msg = copy.deepcopy(pose_msg)
        # elif vrpn_object == 4:
        #     current_ped4_msg = copy.deepcopy(pose_msg)

        # # recording state history (not required for main execution)
        # global record
        # global saved
        # global pose_msg_hist

        # if record == True:
        #     pose_msg_hist.append(pose_msg)
        # elif record == False and len(pose_msg_hist) > 0 and saved == False:
        #     print(len(pose_msg_hist))
        #     print("saving VRPN output")

        #     self.save_s_hist(pose_msg_hist)
        #     saved = True
        #     print("save complete")
        
        return

    # function called by service
    def estimate_state(self, req):
        global current_veh_msg
        global current_ped1_msg
        # global current_ped2_msg
        # global current_ped3_msg
        # global current_ped4_msg

        # global record
        # global pose_msg_hist

        # record = req.record

        current_state = [0]*(3 + 2*(1))
        current_state[0:3] = self.veh_state(current_veh_msg)
        current_state[3:5] = self.ped_state(current_ped1_msg)
        # current_state[5:7] = self.ped_state(current_ped2_msg)
        # current_state[7:9] = self.ped_state(current_ped3_msg)
        # current_state[9:11] = self.ped_state(current_ped4_msg)

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
    # def save_s_hist(self, pose_msg_hist):
    #     # TO-DO: add datetime to file name
    #     # TO-DO: add time stamp to each entry
    #     f = open("/home/adcl/catkin_ws/src/marmot-ros/controller_pkg/histories/s_hist_vrpn.csv", 'w')
    #     writer = csv.writer(f)

    #     for pose_msg_k in pose_msg_hist:
    #         s_k = self.msg_to_state(pose_msg_k)
    #         writer.writerow(s_k)

    #     f.close()
        
    #     return


if __name__ == '__main__':
    try:
        rospy.init_node("state_estimator")
        StateEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass