#!/usr/bin/env julia

# RobotOS.jl docs: https://jdlangs.github.io/RobotOS.jl/latest/

using RobotOS
using BSON: @load

include("/home/adcl/Documents/marmot-algs/HJB-planner/HJB_functions.jl")

@rosimport state_estimator_pkg.srv: EstState
@rosimport controller_pkg.srv: AckPub

rostypegen()
using .state_estimator_pkg.srv
using .controller_pkg.srv

# TO-DO: modify srv msg to include execution time

# TO-DO: need to propagate state forward one step when choosing new action

function main()
    @load "/home/adcl/Documents/marmot-algs/HJB-planner/bson/U_HJB.bson" U_HJB
    @load "/home/adcl/Documents/marmot-algs/HJB-planner/bson/env.bson" env
    @load "/home/adcl/Documents/marmot-algs/HJB-planner/bson/veh.bson" veh

    init_node("controller")

    end_run = false
    while end_run == false
        end_run = run_controller(U_HJB, env, veh) 
    end
end

# NOTE: is this re-initializing the proxy every time? -> possibly, but doesn't seem to matter
function state_estimator_client()
    wait_for_service("/car/state_estimator/get_est_state")

    # see if this can be moved up to main()
    est_state_srv = ServiceProxy{EstState}("/car/state_estimator/get_est_state")

    req = EstStateRequest(1.0)
    resp = est_state_srv(req)

    s = [resp.x, resp.y, resp.theta]

    return s
end

function ack_publisher_client(a)
    wait_for_service("/car/controller/act_ack_pub")

    # see if this can be moved up to main()
    ack_pub_srv = ServiceProxy{AckPub}("/car/controller/act_ack_pub")

    a_req = AckPubRequest(a[1], a[2])
    resp = ack_pub_srv(a_req)
end

# TO-DO: need to delegate publish_control to another node, so that this node can be calculating next action during dt
function run_controller(U_HJB, env, veh)    
    dt = 1.0    # TO-DO: change back
    
    s = state_estimator_client()
    # println("s: ", s)

    if in_target_set(s, env, veh) == false && in_workspace(s, env, veh) == true
        a = optimal_action_HJB(s, U_HJB, env, veh)

        a[1] = 0.3*a[1]  # TO-DO: change back
        end_run = false
    else
        a = [0.0, 0.0]
        end_run = true
    end

    println("controller: a: ", a)
    println("controller: requested new action")

    # this command will take as long as the service callback -> don't put sleep in the callback
    ack_publisher_client(a)
    for i in 1:40
        println("controller: free $i")
    end

    sleep(dt)

    return end_run
end

main()