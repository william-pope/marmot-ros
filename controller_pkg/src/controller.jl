#!/usr/bin/env julia

using RobotOS
using BSON: @load, @save
using Dates

include("/home/adcl/Documents/marmot-algs/HJB-planner/HJB_functions.jl")

@rosimport state_estimator_pkg.srv: EstState
@rosimport controller_pkg.srv: AckPub

rostypegen()
using .state_estimator_pkg.srv
using .controller_pkg.srv

# TO-DO: need to propagate state forward one step when choosing new action

function main()
    @load "/home/adcl/Documents/marmot-algs/HJB-planner/bson/U_HJB.bson" U_HJB
    @load "/home/adcl/Documents/marmot-algs/HJB-planner/bson/env.bson" env
    @load "/home/adcl/Documents/marmot-algs/HJB-planner/bson/veh.bson" veh

    init_node("controller")

    Dt = 0.2
    rate = Rate(1/Dt)

    end_run = false
    a_k = [0.0, 0.0]
    s_hist = []
    a_hist = []

    while end_run == false
        a_k1, end_run, s_hist, a_hist = controller(a_k, s_hist, a_hist, U_HJB, env, veh) 
        a_k = deepcopy(a_k1)

        sleep(rate)
    end

    ack_publisher_client([0.0, 0.0])
    state_estimator_client(false)

    # TO-DO: add datetime to file name
    # TO-DO: add time stamp to each entry
    @save "/home/adcl/catkin_ws/src/marmot-ros/controller_pkg/histories/s_hist.bson" s_hist
    @save "/home/adcl/catkin_ws/src/marmot-ros/controller_pkg/histories/a_hist.bson" a_hist

    # need to add time stamps (can do later)
end

function state_estimator_client(record)
    wait_for_service("/car/state_estimator/get_est_state")
    est_state_srv = ServiceProxy{EstState}("/car/state_estimator/get_est_state")

    s_resp = est_state_srv(EstStateRequest(record))

    s = [s_resp.x, s_resp.y, s_resp.theta]

    return s
end

function ack_publisher_client(a)
    wait_for_service("/car/controller/act_ack_pub")
    ack_pub_srv = ServiceProxy{AckPub}("/car/controller/act_ack_pub")

    a_req = AckPubRequest(a[1], a[2])
    resp = ack_pub_srv(a_req)
end

function controller(a_k, s_hist, a_hist, U_HJB, env, veh)  
    ack_publisher_client(a_k)

    s_k = state_estimator_client(true)
    
    # NOTE: should be propagating state/particles here

    if in_target_set(s_k, env, veh) == false && in_workspace(s_k, env, veh) == true
        a_k1 = optimal_action_HJB(s_k, U_HJB, env, veh)

        a_k1[1] = 0.6*a_k1[1]  # TO-DO: change back
        end_run = false
    else
        a_k1 = [0.0, 0.0]
        end_run = true
    end

    println("controller: s_k: ", s_k)
    println("controller: a_k: ", a_k)

    push!(s_hist, s_k)
    push!(a_hist, a_k)

    return a_k1, end_run, s_hist, a_hist
end

main()