#!/usr/bin/env julia

# RobotOS.jl docs: https://jdlangs.github.io/RobotOS.jl/latest/

using RobotOS
using BSON: @load
using ReferenceFrameRotations

include("/home/adcl/Documents/marmot-algs/HJB-planner/HJB_functions.jl")

@rosimport ackermann_msgs.msg: AckermannDriveStamped
@rosimport geometry_msgs.msg: PoseStamped
@rosimport state_estimator.srv.EstState # correct?

rostypegen()
using .ackermann_msgs.msg
using .geometry_msgs.msg

# ISSUE: vrpn pose isn't updating when object moves
#   - still sends "new" measurements (not identical), but stuck at initial position

# TO-DO: need to propagate state forward one step when choosing new action

function main()
    @load "/home/adcl/Documents/marmot-algs/HJB-planner/U_HJB.bson" U_HJB
    @load "/home/adcl/Documents/marmot-algs/HJB-planner/sg.bson" sg # ISSUE
    @load "/home/adcl/Documents/marmot-algs/HJB-planner/veh.bson" veh

    init_node("controller")
    ctrl_pub = Publisher{AckermannDriveStamped}(
        "/car/mux/ackermann_cmd_mux/input/controller",
        queue_size=2)

    # ISSUE: state not updating
    #   - rostopic inferred_pose is correct
    #   - y = pose_msg in run_control() is wrong, stuck on initial reading
    #   - run_control() starts repeatedly as expected, but has wrong pose_msg somehow
    #   - issue with queue?
    #       - trying to work its way through buffer of missed messages?
    #       - looks like time slowly ticks up
    #       - storing all messages, need to throw out and only keep newest

    println("controller node initialized")

    spin()
end

function state_estimator_client()
    wait_for_service("get_est_state")
    get_est_state = ServiceProxy{EstState}("get_est_state")

    resp = get_est_state(1)

    s = [resp.x, resp.y, resp.theta]

    return s
end


function run_control(ctrl_pub, U_HJB, sg, veh)    
    dt = 1.0
    
    s = state_estimator_client()

    # TO-DO: implement new environment structure to hold T_set
    T_xy_set = [[-0.5 4.0];
                [0.5 4.0];
                [0.5 5.0];
                [-0.5 5.0]]

    T_theta_set = [[-pi, pi]]

    if in_target_set(s, T_xy_set, T_theta_set, veh) == false
        a = optimal_action_HJB(y, U_HJB, sg, veh)
    else
        a = [0.0, 0.0]
    end

    # println("a: ", a)
    publish_ctrl(ctrl_pub, a, dt)
end

function publish_ctrl(ctrl_pub, ctrl, dt)
    dur = Duration(dt)
    rate = Rate(10)
    start = get_rostime()

    ctrl_msg = AckermannDriveStamped()
    ctrl_msg.header.stamp = get_rostime()

    ctrl_msg.drive.speed = ctrl[1]
    ctrl_msg.drive.steering_angle = ctrl[2]

    while get_rostime()-start < dur
        publish(ctrl_pub, ctrl_msg)
        sleep(rate)
    end
end

main()