#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from state_estimator_pkg.srv import EstState, EstStateResponse

import copy
import math
from scipy.spatial.transform import Rotation
import csv

# TO-DO: need to save

class StateEstimator:
    global latest_pose_msg
    global record
    global saved
    global pose_msg_hist

    def __init__(self):
        global record
        global saved
        global pose_msg_hist
        
        record = False
        saved = False

        pose_msg_hist = []

        self.vrpn_sub_marmot_pose = rospy.Subscriber(
            "/car/vrpn_client_ros/vrpn_client_node/ADCL_Marmot/pose", 
            PoseStamped,
            self.store_pose_msg,
            queue_size=1)

        self.est_state_srv = rospy.Service(
            "/car/state_estimator/get_est_state",
            EstState,
            self.estimate_state)

        return

    # NOTE: records extra ~20 samples at beginning, just initialization time?
    def store_pose_msg(self, pose_msg):
        global latest_pose_msg
        global record
        global saved
        global pose_msg_hist

        latest_pose_msg = copy.deepcopy(pose_msg)

        if record == True:
            pose_msg_hist.append(pose_msg)
        elif record == False and len(pose_msg_hist) > 0 and saved == False:
            print(len(pose_msg_hist))
            print("saving VRPN output")

            self.save_s_hist(pose_msg_hist)
            saved = True
            print("save complete")
        
        return

    def estimate_state(self, req):
        global latest_pose_msg
        global record
        global pose_msg_hist

        record = req.record
        
            
        pose_msg = copy.deepcopy(latest_pose_msg)
        s = self.msg_to_state(pose_msg)

        return EstStateResponse(s[0], s[1], s[2])

    def msg_to_state(self, pose_msg):
        ori = pose_msg.pose.orientation
        rot = Rotation.from_quat([ori.x, ori.y, ori.z, ori.w])
        rot_euler = rot.as_euler('xyz', degrees=False)
        theta = rot_euler[2]

        y_cal = -0.162  # m from Vicon model center to rear axis
        x = pose_msg.pose.position.x + y_cal*math.cos(theta)
        y = pose_msg.pose.position.y + y_cal*math.sin(theta)

        s = [x, y, theta]

        return s

    def save_s_hist(self, pose_msg_hist):
        # TO-DO: add datetime to file name
        # TO-DO: add time stamp to each entry
        f = open("/home/adcl/catkin_ws/src/marmot-ros/controller_pkg/histories/s_hist_vrpn.csv", 'w')
        writer = csv.writer(f)

        for pose_msg_k in pose_msg_hist:
            s_k = self.msg_to_state(pose_msg_k)
            writer.writerow(s_k)

        f.close()
        
        return


if __name__ == '__main__':
    try:
        rospy.init_node("state_estimator")
        StateEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
