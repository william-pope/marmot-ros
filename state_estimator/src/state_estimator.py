#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped


class StateEstimator:

    def __init__(self):
        self.pose_pub = rospy.Publisher("~inferred_pose", 
            PoseStamped, 
            queue_size=1)

        self.vicon_sub_ped1 = rospy.Subscriber("/car/vrpn_client_ros/vrpn_client_node/ADCL_Ped1/pose", 
            PoseStamped,
            self.callback)

    def callback(self, pose_msg):
        self.estimate_state(pose_msg)

    def estimate_state(self, pose_msg):
        self.pose_pub.publish(pose_msg)     # correct pose


if __name__ == '__main__':
    try:
        rospy.init_node("state_estimator")
        s = StateEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass