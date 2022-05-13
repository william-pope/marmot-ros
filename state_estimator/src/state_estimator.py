#!/usr/bin/env python

import numpy as np
import copy
import math
from scipy.spatial.transform import Rotation

import rospy
from geometry_msgs.msg import PoseStamped
from state_estimator.srv import EstState, EstStateResponse

# TO-DO: need to create .srv function

class StateEstimator:
    global latest_pose_msg

    def __init__(self):
        # TO-DO: change back to marmot pose
        self.vrpn_sub_marmot_pose = rospy.Subscriber("/car/vrpn_client_ros/vrpn_client_node/ADCL_Ped1/pose", 
            PoseStamped,
            self.store_pose_msg,
            queue_size=1)

        self.est_state_srv = rospy.Service("get_est_state",
            EstState,
            self.estimate_state)

        return

    def store_pose_msg(self, pose_msg):
        global latest_pose_msg
        latest_pose_msg = copy.deepcopy(pose_msg)
        return

    def estimate_state(self, req):
        b2a = 0.0889

        global latest_pose_msg
        pose_msg = copy.deepcopy(latest_pose_msg)

        s = np.zeros(3)

        ori = pose_msg.pose.orientation
        rot = Rotation.from_quat([ori.x, ori.y, ori.z, ori.w])
        rot_euler = rot.as_euler('xyz', degrees=False)
        theta = rot_euler[3] + np.pi/2

        x = pose_msg.pose.position.x - b2a*math.cos(theta)
        y = pose_msg.pose.position.y - b2a*math.sin(theta)

        # println("y: ", y)
        return EstStateResponse(x, y, theta)


if __name__ == '__main__':
    try:
        rospy.init_node("state_estimator")
        StateEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass