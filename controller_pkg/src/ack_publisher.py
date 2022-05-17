#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from controller_pkg.srv import AckPub, AckPubRequest, AckPubResponse

# - controller can send action whenever up to t1-, 
#       but needs to be free at t1+ to go back to POMDP

# - publisher will be occupied publishing ack commands,
#       can take short interrupt at t1- to take new action
# - could reverse hierarchy here so publisher queries controller (?)
# - service is synchronous, but doesn't trigger interrupt (I believe),
#       so controller will be waiting until service reads it
# - if publisher knows time, dt, t1-, can set it to take action at right time
# - there should be a better way to do this... pain

# - I think it's working :)
# - controller requests new action every 1.0 sec
# - publisher pushes action to ackermann every 0.1 sec until new command
# - system doesn;t react to changes in environment except at 1.0 sec recalculations

# - TO-DO: check if controller is free after requesting action
#   - should be able to print from controller.jl during publishing
#   - should see printing and motion from vehicle
#   - finished all prints before publishing -> at least means controller isn't stuck in service
#   - not really sure, printing isn't that reliable -> save time stamps instead

# - TO-DO: look at fine timing
#   - how long does it take publisher to respond to service/store msg?

class AckPublisher:
    global latest_a_msg
    latest_a_msg = AckPubRequest()
    latest_a_msg.a_v = 0.0
    latest_a_msg.a_phi = 0.0

    def __init__(self):
        global latest_a_msg

        self.ack_pub_srv = rospy.Service(
            "/car/controller/act_ack_pub",
            AckPub,
            self.store_a_msg)

        self.ack_pub = rospy.Publisher(
            "/car/mux/ackermann_cmd_mux/input/controller", 
            AckermannDriveStamped, 
            queue_size=2)

        while True:
            self.publish_to_ackermann(latest_a_msg)
            rospy.sleep(0.05)

        return

    # needs to be as fast as possible
    def store_a_msg(self, a_req):
        print("ack_pub: received new action")

        global latest_a_msg
        latest_a_msg = a_req

        return AckPubResponse(1)

    def publish_to_ackermann(self, a_msg): 
        # rate = rospy.Rate(10)

        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()

        ack_msg.drive.speed = a_msg.a_v
        ack_msg.drive.steering_angle = a_msg.a_phi

        print('ack_pub: publishing action: ' + str(a_msg.a_v) + ", " + str(a_msg.a_phi))
        # TO-DO: need to publish until new action received
        self.ack_pub.publish(ack_msg)

        return 


if __name__ == '__main__':
    try:
        rospy.init_node("ack_publisher")
        AckPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass