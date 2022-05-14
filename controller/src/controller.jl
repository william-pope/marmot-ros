#!/usr/bin/env julia

# RobotOS.jl docs: https://jdlangs.github.io/RobotOS.jl/latest/

using RobotOS
using BSON: @load
using ReferenceFrameRotations

include("/home/adcl/Documents/marmot-algs/HJB-planner/HJB_functions.jl")

@rosimport ackermann_msgs.msg: AckermannDriveStamped
@rosimport geometry_msgs.msg: PoseStamped
@rosimport state_estimator.srv: EstState

rostypegen()
using .ackermann_msgs.msg
using .geometry_msgs.msg
using .state_estimator.srv

# TO-DO: need to propagate state forward one step when choosing new action

function main()
    @load "/home/adcl/Documents/marmot-algs/HJB-planner/bson/U_HJB.bson" U_HJB
    @load "/home/adcl/Documents/marmot-algs/HJB-planner/bson/env.bson" env
    @load "/home/adcl/Documents/marmot-algs/HJB-planner/bson/veh.bson" veh

    init_node("controller")
    ctrl_pub = Publisher{AckermannDriveStamped}(
        "/car/mux/ackermann_cmd_mux/input/controller",
        queue_size=2)

    println("controller node initialized")

    end_run = false
    while end_run == false
        end_run = run_control(ctrl_pub, U_HJB, env, veh) 
    end
end

# NOTE: is this re-initializing the proxy every time? -> possibly, but doesn't seem to matter
function state_estimator_client()
    wait_for_service("/car/state_estimator/get_est_state")
    est_state_srv = ServiceProxy{EstState}("/car/state_estimator/get_est_state")

    req = EstStateRequest(1.0)
    resp = est_state_srv(req)

    s = [resp.x, resp.y, resp.theta]

    return s
end

# TO-DO: need to delegate publish_control to another node, so that this node can be calculating next action during dt
function run_control(ctrl_pub, U_HJB, env, veh)    
    dt = 0.1
    
    s = state_estimator_client()
    println("s: ", s)

    if in_target_set(s, env, veh) == false && in_workspace(s, env, veh) == true
        a = optimal_action_HJB(s, U_HJB, env, veh)

        a[1] = 0.5*a[1]  # TO-DO: change back
        end_run = false
    else
        a = [0.0, 0.0]
        end_run = true
    end

    println("a: ", a)
    println("")

    publish_ctrl(ctrl_pub, a, dt)

    return end_run
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