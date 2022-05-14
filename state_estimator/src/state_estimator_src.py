#!/usr/bin/env python

import copy
import math
from scipy.spatial.transform import Rotation

import rospy
from geometry_msgs.msg import PoseStamped
from state_estimator.srv import EstState, EstStateResponse


class StateEstimator:
    global latest_pose_msg

    def __init__(self):
        self.vrpn_sub_marmot_pose = rospy.Subscriber("/car/vrpn_client_ros/vrpn_client_node/ADCL_Marmot/pose", 
            PoseStamped,
            self.store_pose_msg,
            queue_size=1)

        self.est_state_srv = rospy.Service("/car/state_estimator/get_est_state",
            EstState,
            self.estimate_state)

        print("state_estimator node initialized")

        return

    def store_pose_msg(self, pose_msg):
        global latest_pose_msg
        latest_pose_msg = copy.deepcopy(pose_msg)
        return

    def estimate_state(self, req):
        y_cal = -0.162  # m from Vicon model center to rear axis

        global latest_pose_msg
        pose_msg = copy.deepcopy(latest_pose_msg)

        ori = pose_msg.pose.orientation
        rot = Rotation.from_quat([ori.x, ori.y, ori.z, ori.w])
        rot_euler = rot.as_euler('xyz', degrees=False)
        theta = rot_euler[2]

        x = pose_msg.pose.position.x + y_cal*math.cos(theta)
        y = pose_msg.pose.position.y + y_cal*math.sin(theta)

        return EstStateResponse(x, y, theta)


if __name__ == '__main__':
    try:
        rospy.init_node("state_estimator")
        StateEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
