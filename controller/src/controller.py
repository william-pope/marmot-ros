#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped

# end point:
#   - connected chain from vrpn->state_estimator->controller->ackermann_mux
#   - controller commands executed by car (manual input topics need to be empty)

#   - need to be able to interpret PoseStamped in controller context
#   - swap hat object for marmot object on vrpn
#   - want to publish commands based on vrpn measurement

# ISSUE: stopping and starting motion
#   - robot stops even when ped is >1, randomly starts again
#   - check if controls are still publishing

class Controller:

    def __init__(self):
        self.ctrl_pub = rospy.Publisher("/car/mux/ackermann_cmd_mux/input/controller", 
            AckermannDriveStamped, 
            queue_size=2)

        self.state_est_sub = rospy.Subscriber("/car/state_estimator/state_estimator/inferred_pose", 
            PoseStamped,
            self.callback)

    def callback(self, pose_msg):
        self.compute_action(pose_msg)

    def compute_action(self, pose_msg):
        x_ped = pose_msg.pose.position.x
        y_ped = pose_msg.pose.position.y
	theta_ped = pose_msg.pose.orientation

	#  print(theta_ped)

        if y_ped > 0.75:
            ctrl = [0.5,0.0]
        elif y_ped < -0.75:
            ctrl = [-0.5,-0.0]
        else:
            ctrl = [0.0,0.0]
            
        self.publish_ctrl(ctrl)

    def publish_ctrl(self, ctrl):
        dur = rospy.Duration(1.0)
        rate = rospy.Rate(10)
        start = rospy.Time.now()

        ctrl_msg = AckermannDriveStamped()
        ctrl_msg.header.stamp = rospy.Time.now()

        ctrl_msg.drive.speed = float(ctrl[0])
        ctrl_msg.drive.steering_angle = float(ctrl[1])

        print(ctrl_msg)
        self.ctrl_pub.publish(ctrl_msg)


if __name__ == '__main__':
    try:
        rospy.init_node("controller")
        c = Controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
