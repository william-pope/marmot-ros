#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from controller_pkg.srv import AckPub, AckPubRequest, AckPubResponse

# TO-DO: what is dt required to have smooth actions fron VESC?

class AckPublisher:
    global latest_a_msg

    def __init__(self):
        global latest_a_msg

        latest_a_msg = AckPubRequest()
        latest_a_msg.a_v = 0.0
        latest_a_msg.a_phi = 0.0

        self.ack_pub_srv = rospy.Service(
            "/car/controller/act_ack_pub",
            AckPub,
            self.store_a_msg)

        self.ack_pub = rospy.Publisher(
            "/car/mux/ackermann_cmd_mux/input/controller", 
            AckermannDriveStamped, 
            queue_size=2)

        dt = 0.1
        rate = rospy.Rate(1/dt)
        while True:
            self.publish_to_ackermann(latest_a_msg)
            rate.sleep()

        return

    def store_a_msg(self, a_req):
        global latest_a_msg
        latest_a_msg = a_req

        return AckPubResponse(True)

    def publish_to_ackermann(self, a_msg): 
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()

        ack_msg.drive.speed = a_msg.a_v
        ack_msg.drive.steering_angle = a_msg.a_phi

        self.ack_pub.publish(ack_msg)

        return 


if __name__ == '__main__':
    try:
        rospy.init_node("ack_publisher")
        AckPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass