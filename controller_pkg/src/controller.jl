#!/usr/bin/env julia

using RobotOS
using BSON: @load, @save
using Dates

include("/home/adcl/Documents/marmot-algs/HJB-planner/HJB_generator_functions.jl")
include("/home/adcl/Documents/marmot-algs/HJB-planner/HJB_planner_functions.jl")

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

    Dt = 0.1
    rate = Rate(1/Dt)

    Am = [[a_v,a_phi] for a_v in [-veh.c_vb, veh.c_vf], a_phi in [-veh.c_phi, 0.0, veh.c_phi]]
    A = reshape(Am, (length(Am),1))
    sort!(A, dims=1)

    # execution ---
    end_run = false
    a_k = [0.0, 0.0]
    s_hist = []
    a_hist = []

    step = 1

    # main loop
    while end_run == false
        # 1: publishes current action to ESC
        ack_publisher_client(a_k)
        println("\ncontroller: a_k: ", a_k)

        # 2: receives current state from Vicon
        s_k = state_estimator_client(true)
        println("controller: s_k: ", s_k)

        # 3: calculates next action
        a_k1, end_run = controller(s_k, a_k, Dt, U_HJB, A, obstacle_mat, car_EoM, env, veh)

        # stores state/action history
        push!(s_hist, s_k)
        push!(a_hist, a_k)

        # passes new action to next loop
        a_k = deepcopy(a_k1)

        if step >= 2*60*4
            end_run = true
        end
        step += 1

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
function controller(s_k, a_k, Dt, U, A, O, EoM::Function, env::Environment, veh::Vehicle)
    # propagates state to next time step given current action
    # TO-DO: replace this with belief updater (?)
    s_k1 = runge_kutta_4(s_k, a_k, Dt, EoM, veh)

    # runs tree search to find best action at next time step
    if in_target_set(s_k1, env, veh) == false && in_workspace(s_k1, env, veh) == true
        # TO-DO: replace this with POMDP function
        a_k1 = HJB_action(s_k1, U, A, O, env, veh)
        println("a_k1: ", a_k1)
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


function new_main()
    init_node("controller")

    # parameters ---
    @load "/home/adcl/Documents/marmot-algs/HJB-planner/bson/U_HJB.bson" U_HJB
    @load "/home/adcl/Documents/marmot-algs/HJB-planner/bson/env.bson" env
    @load "/home/adcl/Documents/marmot-algs/HJB-planner/bson/veh.bson" veh

    Dt = 0.1
    rate = Rate(1/Dt)

    Am = [[a_v,a_phi] for a_v in [-veh.c_vb, veh.c_vf], a_phi in [-veh.c_phi, 0.0, veh.c_phi]]
    A = reshape(Am, (length(Am),1))
    sort!(A, dims=1)

    # initial tracking ---
    o_kn1 = []
    for _ in 1:5
        o_k = state_estimator_client(true)

        if isempty(o_kn1) == false
            # update belief

        end

        o_kn1 = deepcopy(o_k)

        sleep(rate)
    end

    # moving ---
    end_run = false
    o_kn1 = []
    a_k = [0.0, 0.0]
    o_hist = []
    a_hist = []

    step = 1

    # main loop
    while end_run == false
        # 1: publishes current action to ESC
        ack_publisher_client(a_k)
        println("\ncontroller: a_k: ", a_k)

        # 2: receives current state from Vicon
        o_k = state_estimator_client(true)
        println("controller: o_k: ", o_k)

        # 3: runs online POMDP planner
        # - 3.a: updates belief from new observation, old observation, old belief
        # update belief
        # convert to DESPOT belief format

        # - 3.b: runs DESPOT and returns action
        # ...
        a_k1, end_run = controller(o_k, a_k, Dt, U_HJB, A, obstacle_mat, car_EoM, env, veh)

        # stores state/action history
        push!(o_hist, o_k)
        push!(a_hist, a_k)

        # passes new action to next loop
        o_kn1 = deepcopy(o_k)
        a_k = deepcopy(a_k1)

        if step >= 2*60*4
            end_run = true
        end
        step += 1

        # 4: sleeps for remainder of Dt loop
        sleep(rate)
    end

    # sends [0,0] action to stop vehicle, ending run
    ack_publisher_client([0.0, 0.0])
    state_estimator_client(false)

    # saving ---
    # TO-DO: add datetime to file name
    # TO-DO: add time stamp to each entry
    @save "/home/adcl/catkin_ws/src/marmot-ros/controller_pkg/histories/o_hist.bson" o_hist
    @save "/home/adcl/catkin_ws/src/marmot-ros/controller_pkg/histories/a_hist.bson" a_hist

    rand_noise_generator_for_solver = MersenneTwister(rand_noise_generator_seed_for_solver)
    env = generate_ASPEN_environment_no_obstacles(10, rand_noise_generator_for_env)
    env.humans = Array{human_state,1}()

    #TO DO:
    #Include appropriate header files
    #Update vehicle position from
    #Update Cart Lidar Data from Vicon data
    env_right_now = deepcopy(env)

    #Create POMDP for env_right_now
    #POMDP_Planner_2D_action_space <: POMDPs.POMDP{POMDP_state_2D_action_space,Int,Array{location,1}}
    # discount_factor::Float64; pedestrian_distance_threshold::Float64; pedestrian_collision_penalty::Float64;
    # obstacle_distance_threshold::Float64; obstacle_collision_penalty::Float64; goal_reward_distance_threshold::Float64;
    # cart_goal_reached_distance_threshold::Float64; goal_reward::Float64; max_cart_speed::Float64; world::experiment_environment
    golfcart_2D_action_space_pomdp = POMDP_Planner_2D_action_space(0.97,0.5,-100.0,2.0,-100.0,0.0,1.0,1000.0,4.0,env_right_now)
    discount(p::POMDP_Planner_2D_action_space) = p.discount_factor
    isterminal(::POMDP_Planner_2D_action_space, s::POMDP_state_2D_action_space) = is_terminal_state_pomdp_planning(s,location(-100.0,-100.0));
    actions(m::POMDP_Planner_2D_action_space,b) = get_actions_non_holonomic(b)

    solver = DESPOTSolver(bounds=IndependentBounds(DefaultPolicyLB(FunctionPolicy(calculate_lower_bound_policy_pomdp_planning_2D_action_space),max_depth=100),
                            calculate_upper_bound_value_pomdp_planning_2D_action_space, check_terminal=true),K=50,D=100,T_max=0.5, tree_in_info=true,
                            rng = rand_noise_generator_for_solver)

    planner = POMDPs.solve(solver, golfcart_2D_action_space_pomdp);

    b = POMDP_2D_action_space_state_distribution(golfcart_2D_action_space_pomdp.world,current_belief)
    a, info = action_info(planner, b)
end
