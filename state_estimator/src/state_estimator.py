#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped


class StateEstimator:

    def __init__(self):
        self.pose_pub = rospy.Publisher("~inferred_pose", 
            PoseStamped, 
            queue_size=1)

        # ISSUE: change back to marmot pose
        self.vrpn_sub_marmot_pose = rospy.Subscriber("/car/vrpn_client_ros/vrpn_client_node/ADCL_Ped1/pose", 
            PoseStamped,
            self.estimate_state,
            queue_size=1)

    def estimate_state(self, pose_msg):
        self.pose_pub.publish(pose_msg)


if __name__ == '__main__':
    try:
        rospy.init_node("state_estimator")
        s = StateEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass