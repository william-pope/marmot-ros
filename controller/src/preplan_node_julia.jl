#!/usr/bin/env julia

# ISSUE: need to get HJB_function.jl, U/sg/veh.jld
using RobotOS

@rosimport ackermann_msgs.msg: AckermannDriveStamped
@rosimport geometry_msgs.msg: PoseStamped

rostypegen()
using .ackermann_msgs.msg
using .geometry_msgs.msg

function main()
    init_node("controller")
    ctrl_pub = Publisher{AckermannDriveStamped}
        ("/car/mux/ackermann_cmd_mux/input/manual",
        queue_size=2)

    # ISSUE: is callback called correctly here?
    state_est_sub = Subscriber{PoseStamped}
        ("/car/state_estimator/state_estimator/inferred_pose",
        run_control)

    # while(!is_shutdown())
end

# TO-DO: how to get U, sg, veh into this function?
function run_control(ctrl_pub, pose_msg)
    y = zeros(3)
    y[1] = pose_msg.pose.position.x
    y[2] = pose_msg.pose.position.x
    y[3] = pose_msg.pose.orientation

    a = optimal_action_HJB(y, U_HJB, sg, veh)

    publish_ctrl(ctrl_pub, a, dt)
end

function publish_ctrl(ctrl_pub, ctrl, dt)
    dur = Duration(dt)
    rate = Rate(10)
    start = get_rostime()

    ctrl_msg = AckermannDriveStamped()
    ctrl_msg.header.stamp = get_rostime()

    ctrl_msg.drive.speed = parse(Float64, ctrl[1])
    ctrl_msg.drive.steering_angle = parse(Float64, ctrl[2])

    while get_rostime()-start < dur
        publish(ctrl_pub, ctrl_msg)
        sleep(rate)
    end
end

main()