#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped

plan_msg = """
please enter a number to select a driving plan:
1 - drag
2 - shuttle
3 - circle
4 - serpent

"""

class Preplan:

    def __init__(self):
        self.ctrl_pub = rospy.Publisher("/car/mux/ackermann_cmd_mux/input/manual", 
            AckermannDriveStamped, 
            queue_size=2)

        # checks op_mode
        while not rospy.is_shutdown():
            self.op_mode = rospy.get_param("/car/mode_select/op_mode")

            # ISSUE: several second long pause before executing plan
            if self.op_mode == "preplan_python":
                key_input = raw_input(plan_msg)

                if key_input == "1":
                    plan_name = "drag"

                elif key_input == "2":
                    plan_name = "shuttle"

                elif key_input == "3":
                    plan_name = "circle"

                elif key_input == "4":
                    plan_name = "serpent"

                else:
                    print("invalid input")
                    continue

                print(plan_name)
                # self.run_plan(plan_name)

                break
            else:
                continue

    def run_plan(self, plan_name):
        plan_file = "/home/adcl/catkin_ws/src/manual/src/plans/" + plan_name + ".txt"
        with open(plan_file) as file:
            plan = file.readlines()

        rospy.sleep(1.0)
        for c in plan:
            ctrl = c.split(",")
            self.publish_ctrl(ctrl)

    def publish_ctrl(self, ctrl):
        dur = rospy.Duration(1.0)
        rate = rospy.Rate(10)
        start = rospy.Time.now()

        ctrl_msg = AckermannDriveStamped()
        ctrl_msg.header.stamp = rospy.Time.now()

        ctrl_msg.drive.speed = float(ctrl[0])
        ctrl_msg.drive.steering_angle = float(ctrl[1])

        while rospy.Time.now()-start < dur:
            self.ctrl_pub.publish(ctrl_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node("preplan")
        p = Preplan()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass