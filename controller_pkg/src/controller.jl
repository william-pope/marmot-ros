#!/usr/bin/env julia

using RobotOS
using BSON: @load, @save
using Dates

algs_path = "/home/adcl/Documents/marmot-algs/"
include(algs_path * "HJB-planner/HJB_generator_functions.jl")
include(algs_path * "HJB-planner/HJB_planner_functions.jl")

han_path = "/home/adcl/Documents/human_aware_navigation/src/"
include(han_path * "environment.jl")
include(han_path * "utils.jl")
include(han_path * "two_d_action_space_pomdp.jl")
include(han_path * "belief_tracker.jl")
include(han_path * "new_main_2d_action_space_pomdp.jl")

# ROS connections:
#   - to state_updater node as CLIENT to state_updater service
#   - to ack_publisher node as CLIENT of ack_publisher service

@rosimport state_updater_pkg.srv: UpdateState
@rosimport controller_pkg.srv: AckPub

rostypegen()
using .state_updater_pkg.srv
using .controller_pkg.srv

# (?): these 3 lines are duplicated, can delete?
discount(p::POMDP_Planner_2D_action_space) = p.discount_factor
isterminal(::POMDP_Planner_2D_action_space, s::POMDP_state_2D_action_space) = is_terminal_state_pomdp_planning(s,location(-100.0,-100.0));
actions(m::POMDP_Planner_2D_action_space,b) = get_actions_non_holonomic(b)

# call state_updater service as client
function state_updater_client(record)
    wait_for_service("/car/state_updater/get_state_update")
    update_state_srv = ServiceProxy{UpdateState}("/car/state_updater/get_state_update")

    resp = update_state_srv(UpdateStateRequest(record))

    return resp.state
end

# call ack_publisher service as client
function ack_publisher_client(a)
    wait_for_service("/car/controller/act_ack_pub")
    ack_pub_srv = ServiceProxy{AckPub}("/car/controller/act_ack_pub")

    a_req = AckPubRequest(a[1], a[2])
    resp = ack_pub_srv(a_req)
end

function pomdp2ros_action(a_pomdp, v_kn1, Dt, veh_L)
    # define limits
    v_min = 0.0
    v_max = 1.0
    phi_max = 0.475
    
    # calculate velocity
    Dv = a_pomdp[2]
    v_k = v_kn1 + Dv
    clamp!(v_k, v_min, v_max)
    
    # calculate steering angle
    arc_length = v_k*Dt
    if(arc_length != 0)
        phi_k = atan(a_pomdp[1] * veh_L/arc_length)
        clamp!(phi_k, -phi_max, phi_max)
    else
        phi_k = 0.0
    end

    return [v_k, phi_k]
end

function main()
    # initialize ROS controller node
    init_node("controller")

    # define environment
    rand_noise_generator_for_solver = MersenneTwister(100)
    env_k = generate_ASPEN_environment_no_obstacles(0, rand_noise_generator_for_solver)
    # env.humans = Array{human_state,1}()

    # define POMDP
    golfcart_2D_action_space_pomdp = POMDP_Planner_2D_action_space(0.97, 0.1, -100.0, 2.0, -100.0, 0.0, 1.0, 100.0, 2.0, 0.5, env_k)

    discount(p::POMDP_Planner_2D_action_space) = p.discount_factor
    isterminal(::POMDP_Planner_2D_action_space, s::POMDP_state_2D_action_space) = is_terminal_state_pomdp_planning(s,location(-100.0,-100.0));
    actions(m::POMDP_Planner_2D_action_space,b) = get_actions_non_holonomic(b)

    solver = DESPOTSolver(bounds=IndependentBounds(DefaultPolicyLB(FunctionPolicy(calculate_lower_bound_policy_pomdp_planning_2D_action_space),max_depth=100),
                            calculate_upper_bound_value_pomdp_planning_2D_action_space, check_terminal=true),K=50,D=100,T_max=0.3, tree_in_info=true,
                            rng = rand_noise_generator_for_solver)

    planner = POMDPs.solve(solver, golfcart_2D_action_space_pomdp)

    # initialize utilities
    end_run = false    
    o_hist = []
    a_hist = []
    
    max_plan_steps = 2*60*4
    planning_Dt = 0.5
    planning_rate = Rate(1/planning_Dt)
  
    plan_step = 1

    while end_run == false
        println("\n--- --- ---")
        println("k = ", plan_step)
        println("t_k = ", Dates.now())

        # 1: publish current action to ESC
        ack_publisher_client(a_ros)

        # 2.a: receive current state and belief
        obs_k = state_updater_client(true)
        belief_dist_k = belief_updater_client(true)     # TO-DO: need to convert belief_array to actual belief object

        # 3: update environment
        env_k.cart.x = obs_k.state[1]
        env_k.cart.y = obs_k.state[2]
        env_k.cart.theta = obs_k.state[3]
        env_k.cart.v = a_ros[1]
        
        ped_states_k = Array{human_state,1}()
        ped_id = 1.0
        for i in 4:2:length(obs_k.state)
            ped = human_state(obs_k.state[i], obs_k.state[i+1], 1.0, env_k.goals[1], ped_id)
            ped_id += 1
            push!(ped_states_k, ped)
        end

        env_k.complete_cart_lidar_data = ped_states_k
        env_k.cart_lidar_data = ped_states_k

        dist_to_goal = sqrt((env_k.cart.x - env_k.cart.goal.x)^2 + (env_k.cart.y - env_k.cart.goal.y)^2)
        if ((plan_step >= max_plan_steps) || (dist_to_goal <= 0.5))
            end_run = true
        end
        
        # 4: calculate action for next cycle with POMDP solver
        state_k = [env_k.cart.x, env_k.cart.y, env_k.cart.theta]
        params_k = [env_k.cart.v, env_k.cart.L, a[2]]

        env_k1 = deepcopy(env_k)
        env_k1.cart.x, env_k1.cart.y, env_k1.cart.theta = last.(get_intermediate_points(state_k, planning_Dt, params_k));

        b = POMDP_2D_action_space_state_distribution(env_k1, belief_k)        # creates struct
        a_pomdp, info = action_info(planner, b)                               # (?): what is "info"?
        a_ros = pomdp2ros_action(a_pomdp, env_k1.cart.v, planning_Dt, env_k1.cart.L)
        
        println("vehicle state @ t_k1: ", [env_k1.cart.x, env_k1.cart.y, env_k1.cart.theta])
        println("POMDP action @ t_k1: ", a_pomdp)
        println("ROS action @ t_k1: ", a_ros)

        # 4: book-keeping
        push!(obs_hist, obs_k.state)
        push!(belief_hist, belief_k)
        push!(action_hist, a_ros)
        plan_step += 1

        #5: sleep for remainder of planning loop
        sleep(planning_rate)
    end

    # send [0,0] action to stop vehicle
    ack_publisher_client([0.0, 0.0])
    state_updater_client(false)

    # save state and action history
    @save "/home/adcl/catkin_ws/src/marmot-ros/controller_pkg/histories/o_hist.bson" o_hist
    @save "/home/adcl/catkin_ws/src/marmot-ros/controller_pkg/histories/a_hist.bson" a_hist
end

main()

#=
Changes:
1) Delta Theta Angle to Steering Angle
2) Delta velocity to actual velocity
3) Change [0.0,0.0] to sudden brake action
4) Include sudden brake action
=#


#  # receive initial observation from Vicon, convert to POMDP objects
#  initial_observation = state_updater_client(true)
#  env_right_now.cart.x = initial_observation[1]
#  env_right_now.cart.y = initial_observation[2]
#  env_right_now.cart.theta = initial_observation[3]

#  current_pedestrian_states = Array{human_state,1}()
#  pedestrian_id = 1.0
#  for i in 4:2:length(initial_observation)
#      pedestrian = human_state(initial_observation[i], initial_observation[i+1], 1.0, env_right_now.goals[1], pedestrian_id)
#      pedestrian_id += 1
#      push!(current_pedestrian_states, pedestrian)
#  end

#  env_right_now.complete_cart_lidar_data = current_pedestrian_states
#  env_right_now.cart_lidar_data = current_pedestrian_states

#  # calculate initial belief based on observation
#  initial_belief_over_complete_cart_lidar_data = update_belief([], env_right_now.goals, [], env_right_now.complete_cart_lidar_data)
#  initial_belief = get_belief_for_selected_humans_from_belief_over_complete_lidar_data(initial_belief_over_complete_cart_lidar_data,
#                                                      env_right_now.complete_cart_lidar_data, env_right_now.cart_lidar_data)


# belief_update_time_step = 0.5

#  #Get the first action
#  b = POMDP_2D_action_space_state_distribution(env_right_now,current_belief)
#  action, info = action_info(planner, b)
#  a = pomdp2ros_action(action, env_right_now.cart.v, planning_Dt, env_right_now.cart.L)

# # t=0.0 -> t=0.5 sec
# sleep(belief_update_time_step)
# new_observation = state_updater_client(true)
# new_pedestrian_states = Array{human_state,1}()
# pedestrian_id = 1.0
# for i in 4:2:length(initial_observation)
#     new_pedestrian = human_state(new_observation[i],new_observation[i+1],1.0,env_right_now.goals[1],pedestrian_id)
#     pedestrian_id += 1
#     push!(new_pedestrian_states,new_pedestrian)
# end

# env_right_now.complete_cart_lidar_data = new_pedestrian_states
# env_right_now.cart_lidar_data = new_pedestrian_states
# current_belief_over_complete_cart_lidar_data = update_belief(initial_belief_over_complete_cart_lidar_data, env_right_now.goals, current_pedestrian_states, new_pedestrian_states)
# current_belief = get_belief_for_selected_humans_from_belief_over_complete_lidar_data(current_belief_over_complete_cart_lidar_data,
#                                                     env_right_now.complete_cart_lidar_data, env_right_now.cart_lidar_data)
# current_pedestrian_states = new_pedestrian_states

# # t=0.5 -> t=1.0 sec
# sleep(belief_update_time_step)
# new_observation = state_updater_client(true)
# new_pedestrian_states = Array{human_state,1}()
# pedestrian_id = 1.0
# for i in 4:2:length(initial_observation)
#     new_pedestrian = human_state(new_observation[i],new_observation[i+1],1.0,env_right_now.goals[1],pedestrian_id)
#     pedestrian_id += 1
#     push!(new_pedestrian_states,new_pedestrian)
# end

# env_right_now.complete_cart_lidar_data = new_pedestrian_states
# env_right_now.cart_lidar_data = new_pedestrian_states
# current_belief_over_complete_cart_lidar_data = update_belief(current_belief_over_complete_cart_lidar_data, env_right_now.goals,
#                                                         current_pedestrian_states, new_pedestrian_states)
# current_belief = get_belief_for_selected_humans_from_belief_over_complete_lidar_data(current_belief_over_complete_cart_lidar_data,
#                                                     env_right_now.complete_cart_lidar_data, env_right_now.cart_lidar_data)
# current_pedestrian_states = new_pedestrian_states