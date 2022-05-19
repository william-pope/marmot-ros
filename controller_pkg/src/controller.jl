#!/usr/bin/env julia

using RobotOS
using BSON: @load, @save
using Dates

include("/home/adcl/Documents/marmot-algs/HJB-planner/HJB_functions.jl")
include("/home/adcl/Documents/marmot-algs/MCTS-planner/MCTS_functions.jl")

@rosimport state_estimator_pkg.srv: EstState
@rosimport controller_pkg.srv: AckPub

rostypegen()
using .state_estimator_pkg.srv
using .controller_pkg.srv

# TO-DO: need to propagate state forward one step when choosing new action

function main()
    init_node("controller")

    # parameters ---
    @load "/home/adcl/Documents/marmot-algs/HJB-planner/bson/U_HJB.bson" U_HJB
    @load "/home/adcl/Documents/marmot-algs/HJB-planner/bson/env.bson" env
    @load "/home/adcl/Documents/marmot-algs/HJB-planner/bson/veh.bson" veh
    V_HJB = -U_HJB

    sims = 36
    w_max = 1
    d_max = 10
    c_UCB = 8
    slv = Solver(sims, w_max, d_max, c_UCB)

    S = [[-10.0, 10.0],
    [-10.0, 10.0],
    [-pi, pi]]

    A_v = [-0.751, 1.0]
    A_phi = [-0.5, 0.0, 0.5]
    A = vec([[a_v, a_phi] for a_phi in A_phi, a_v in A_v])

    gamma = 0.95

    std_v = 0
    std_phi = 0

    Dt = 0.5
    rate = Rate(1/Dt)

    # execution ---
    end_run = false
    a_k = [0.0, 0.0]
    s_hist = []
    a_hist = []

    while end_run == false
        a_k1, end_run, s_hist, a_hist = controller(a_k, Dt, s_hist, a_hist, S, A, V_HJB, slv, car_EoM, env, veh) 
        a_k = deepcopy(a_k1)

        sleep(rate)
    end

    ack_publisher_client([0.0, 0.0])
    state_estimator_client(false)

    # saving ---
    # TO-DO: add datetime to file name
    # TO-DO: add time stamp to each entry
    @save "/home/adcl/catkin_ws/src/marmot-ros/controller_pkg/histories/s_hist.bson" s_hist
    @save "/home/adcl/catkin_ws/src/marmot-ros/controller_pkg/histories/a_hist.bson" a_hist
end

function controller(a_k, Dt, s_hist, a_hist, S, A, V_HJB, slv::Solver, EoM::Function, env::Environment, veh::Vehicle)  
    # sends current action to publisher
    ack_publisher_client(a_k)

    # received current state from estimator
    s_k = state_estimator_client(true)

    println("controller: s_k: ", s_k)
    println("controller: a_k: ", a_k)
    
    # propagates state to next time step
    s_k1 = gen_state(s_k, a_k, 0.0, 0.0, Dt, EoM, veh)

    # runs MCTS to compute action for next time step
    if in_target_set(s_k1, env, veh) == false && in_workspace(s_k1, env, veh) == true
        a_k1 = continuous_MCTS(s_k1, V_HJB, Dt, S, A, gamma, 0.0, 0.0, slv, EoM, env, veh)  # Q: how long will this take? want anytime

        a_k1[1] = 0.5*a_k1[1]  # TO-DO: change back
        end_run = false
    else
        a_k1 = [0.0, 0.0]
        end_run = true
    end

    # stores state/action history
    push!(s_hist, s_k)
    push!(a_hist, a_k)

    return a_k1, end_run, s_hist, a_hist
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

main()