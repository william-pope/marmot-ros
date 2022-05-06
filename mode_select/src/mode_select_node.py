#!/usr/bin/env python

import rospy
from std_msgs.msg import String

mode_msg = """
welcome to marmot ---

please enter a number to select operating mode:
1 - preplanned route (python)
2 - preplanned route (julia)
3 - keyboard control
4 - Vicon tracking

"""

class ModeSelect:

    def __init__(self):
        self.main()

    # ISSUE:runs "figure_8" even without being commanded to?
    def main(self):
        key_input = raw_input(mode_msg)

        if key_input == "1":
            print("preplanned route (python)")
            rospy.set_param("op_mode", "preplan_python")

        elif key_input == "2":
            print("preplanned route (julia)")
            rospy.set_param("op_mode", "preplan_julia")

        elif key_input == "3":
            print("keyboard control")
            rospy.set_param("op_mode", "key_control")

        elif key_input == "4":
            print("vicon tracking")
            rospy.set_param("op_mode", "vicon_track")

        else:
            print("invalid input")
            self.startup()

if __name__ == '__main__':
    try:
        rospy.init_node("mode_select")
        ui = ModeSelect()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass