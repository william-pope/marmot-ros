#!/usr/bin/env python

import rospy
from std_msgs.msg import String

from ackermann_msgs.msg import AckermannDriveStamped
# from geometry_msgs.msg import Point, PoseStamped
# from std_msgs.msg import ColorRGBA, Empty
# from std_srvs.srv import Empty as SrvEmpty
# from visualization_msgs.msg import Marker

# import parameters
# import utils

class KeyControl:

    def __init__(self):
        self.ctrl_pub  = rospy.Publisher("/car/mux/ackermann_cmd_mux/input/manual", AckermannDriveStamped, queue_size=2)

        self.main()
        
        # self.CAR_NAME = rospy.get_param("~car_name", "/car")
        # if not self.CAR_NAME.endswith("/"):
        #     self.CAR_NAME += "/"

    def main(self):
        # self.run_plan("figure_8")
        # self.run_plan("speed_ramp")

        # ctrl = [0.25, -1]
        # while not rospy.is_shutdown():
            # self.publish_ctrl(ctrl)
        return

    def run_plan(self, plan_name):
        plan_file = "/home/adcl/catkin_ws/src/manual/src/plans/" + plan_name + ".txt"
        with open(plan_file) as file:
            plan = file.readlines()

        rospy.sleep(5.0)
        for c in plan:
            ctrl = c.split(",")

            self.publish_ctrl(ctrl)

    def publish_ctrl(self, ctrl):
        dur = rospy.Duration(1.0)
        rate = rospy.Rate(10)
        start = rospy.Time.now()

        ctrlmsg = AckermannDriveStamped()
        ctrlmsg.header.stamp = rospy.Time.now()

        ctrlmsg.drive.speed = float(ctrl[0])
        ctrlmsg.drive.steering_angle = float(ctrl[1])

        while rospy.Time.now() - start < dur:
            self.ctrl_pub.publish(ctrlmsg)
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node("key_control")
        kc = KeyControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass