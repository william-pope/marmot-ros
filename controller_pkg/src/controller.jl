#!/usr/bin/env julia

using RobotOS
using BSON: @load, @save
using Dates

include("/home/adcl/Documents/marmot-algs/HJB-planner/HJB_generator_functions.jl")

@rosimport state_estimator_pkg.srv: EstState
@rosimport controller_pkg.srv: AckPub

rostypegen()
using .state_estimator_pkg.srv
using .controller_pkg.srv

# TO-DO: need to set "close to zero" commands to actually be zero (due to floating point errors) (think this causes grinding)

function main()
    init_node("controller")

    # parameters ---
    @load "/home/adcl/Documents/marmot-algs/HJB-planner/bson/U_HJB.bson" U_HJB
    @load "/home/adcl/Documents/marmot-algs/HJB-planner/bson/env.bson" env
    @load "/home/adcl/Documents/marmot-algs/HJB-planner/bson/veh.bson" veh

    Dt = 0.5
    rate = Rate(1/Dt)

    # execution ---
    end_run = false
    a_k = [0.0, 0.0]
    s_hist = []
    a_hist = []

    # main loop
    while end_run == false
        # 1: publishes current action to ESC
        ack_publisher_client(a_k)
        # println("\ncontroller: a_k: ", a_k)

        # 2: receives current state from Vicon
        s_k = state_estimator_client(true)
        # println("controller: s_k: ", s_k)

        # 3: calculates next action
        a_k1, end_run = controller(s_k, a_k, Dt, car_EoM, env, veh) 
        
        # stores state/action history
        push!(s_hist, s_k)
        push!(a_hist, a_k)

        # passes new action to next loop
        a_k = deepcopy(a_k1)

        # 4: sleeps for remainder of Dt loop
        sleep(rate)
    end

    # sends [0,0] action to stop vehicle, ending run
    ack_publisher_client([0.0, 0.0])
    state_estimator_client(false)

    # saving ---
    # TO-DO: add datetime to file name
    # TO-DO: add time stamp to each entry
    @save "/home/adcl/catkin_ws/src/marmot-ros/controller_pkg/histories/s_hist.bson" s_hist
    @save "/home/adcl/catkin_ws/src/marmot-ros/controller_pkg/histories/a_hist.bson" a_hist
end

# TO-DO: modify this function to fit AR-DESPOT solver (main site of modifications)
#   - most code in main.jl loop
#   - action(planner, b) takes current belief and uses POMDP planner/solver to find best action
#       - planner = POMDPs.solve(solver, pomdp)
#       - solver = DESPOTSolver(..., ARDESPOT.IndependentBounds(lower, upper, ...), ...)
#           - lower = ARDESPOT.DefaultPolicyLB(FunctionPolicy(calculate_lower_bound_policy_pomdp_planning)))
#           - upper = calculate_upper_bound_policy_pomdp_planning
function controller(s_k, a_k, Dt, EoM::Function, env::Environment, veh::Vehicle)  
    # propagates state to next time step given current action
    # TO-DO: replace this with belief updater (?)
    s_k1 = deepcopy(s_k)

    # runs tree search to find best action at next time step
    if in_target_set(s_k1, env, veh) == false && in_workspace(s_k1, env, veh) == true
        # TO-DO: replace this with POMDP function
        a_k1 = [0.0, 0.0]
        end_run = false
    else
        a_k1 = [0.0, 0.0]
        end_run = true
    end
    # println("controller: a_k1: ", a_k1)

    return a_k1, end_run
end

function state_estimator_client(record)
    wait_for_service("/car/state_estimator/get_est_state")
    est_state_srv = ServiceProxy{EstState}("/car/state_estimator/get_est_state")

    resp = est_state_srv(EstStateRequest(record))

    return resp.state
end

function ack_publisher_client(a)
    wait_for_service("/car/controller/act_ack_pub")
    ack_pub_srv = ServiceProxy{AckPub}("/car/controller/act_ack_pub")

    a_req = AckPubRequest(a[1], a[2])
    resp = ack_pub_srv(a_req)
end

main()
