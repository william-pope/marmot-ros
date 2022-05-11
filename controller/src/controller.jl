#!/usr/bin/env julia

# RobotOS.jl docs: https://jdlangs.github.io/RobotOS.jl/latest/

using RobotOS
using JLD
using Rotations

include("/home/adcl/Documents/marmot-algs/HJB-planner/HJB_functions.jl")

@rosimport ackermann_msgs.msg: AckermannDriveStamped
@rosimport geometry_msgs.msg: PoseStamped

rostypegen()
using .ackermann_msgs.msg
using .geometry_msgs.msg

# - PoseStamped
#   - Header
#   - Pose
#       - Point
#           - x
#           - y
#           - z
#       - Quaternion
#           - x
#           - y
#           - z
#           - w

# ISSUE: run_conrol()
#   - rqt_graph looks good, controller node is connected to correct topics
#   - run_control() prints once (start of function), then nothing else
#   - bug inside the function that stops execution
#       - need to fix pose orientation using Rotations.jl

function main()
    U_HJB = load("/home/adcl/Documents/marmot-algs/HJB-planner/U_HJB.jld", "U_HJB")
    sg = load("/home/adcl/Documents/marmot-algs/HJB-planner/sg.jld", "sg")
    veh = load("/home/adcl/Documents/marmot-algs/HJB-planner/veh.jld", "veh")

    init_node("controller")
    ctrl_pub = Publisher{AckermannDriveStamped}(
        "/car/mux/ackermann_cmd_mux/input/controller",
        queue_size=2)

    # ISSUE: is callback called correctly here? (first time in Julia)
    state_est_sub = Subscriber{PoseStamped}(
        "/car/state_estimator/state_estimator/inferred_pose",
        run_control,
        (ctrl_pub, U_HJB, sg, veh,),
        queue_size=1)

    println("created controller.jl pubs and subs")

    spin()
end

# ISSUE: not being called -> subscriber not working? spin() blocking incorrectly?
function run_control(pose_msg::PoseStamped, ctrl_pub, U_HJB, sg, veh)
    println("run_control()")
    
    dt = 0.1
    
    y = zeros(3)
    y[1] = pose_msg.pose.position.x
    y[2] = pose_msg.pose.position.y

    println(y)

    # ISSUE: orientation is a 4 part (x,y,z,w) output, need to split into theta
    y[3] = pose_msg.pose.orientation    # ISSUE

    println(y)

    # To-DO: implement new structure to get rid of this
    T_xy_set = [[-0.5 4.0];
                [0.5 4.0];
                [0.5 5.0];
                [-0.5 5.0]]

    T_theta_set = [[-pi, pi]]


    if in_target_set(y, T_xy_set, T_theta_set, veh) == false
        a = optimal_action_HJB(y, U_HJB, sg, veh)
    else
        a = [0.0, 0.0]
    end

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