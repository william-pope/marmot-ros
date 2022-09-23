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

#=
POMDP action set: Delta_v = {+1.0, +0.0, -1.0} m/s, Delta_theta = {+45, +30, +15, +0, -15, -30, -45} deg
ROS action set: v = [-0.75, 1.5] m/s, phi = [-0.475, 0.475] rad
Dt = 0.5 sec
v_kn1 = ... m/s
=#
function pomdp2ros_action(action, v_kn1, Dt, veh_L)
    # velocity
    Dv = action[2]
    v_k = clamp(v_kn1 + Dv, 0.0, 1.0)
    arc_length = v_k*Dt

    # steering angle
    # max_sa = 0.06353163608639502
    #max_sa = 0.475
    max_sa = Inf
    if(arc_length!=0)
        steer_angle = clamp(atan(veh_L*action[1]/arc_length), -max_sa, max_sa)
    else
        steer_angle = 0.0
    end

    return [v_k,steer_angle]
end


function main()
    # initialize ROS controller node
    init_node("controller")

    # define environment
    rand_noise_generator_for_solver = MersenneTwister(100)
    env = generate_ASPEN_environment_no_obstacles(0, rand_noise_generator_for_solver)
    # env.humans = Array{human_state,1}()
    env_right_now = deepcopy(env)

    # initialize utilities
    belief_update_time_step = 0.5
    end_run = false    
    o_hist = []
    a_hist = []
    num_steps_so_far = 1
    MAX_NUM_STEPS = 2*60*4
    planning_Dt = 0.5
    planning_rate  = Rate(1/planning_Dt)

    # define POMDP
    # golfcart_2D_action_space_pomdp = POMDP_Planner_2D_action_space(0.97,0.2,-100.0,2.0,-100.0,0.0,GLOBAL_RADIUS_AROUND_GOAL,100.0,2.0,GLOBAL_TIME_STEP,env_right_now)
    golfcart_2D_action_space_pomdp = POMDP_Planner_2D_action_space(0.97,0.1,-100.0,2.0,-100.0,0.0,1.0,100.0,2.0,0.5,env_right_now)
    discount(p::POMDP_Planner_2D_action_space) = p.discount_factor
    isterminal(::POMDP_Planner_2D_action_space, s::POMDP_state_2D_action_space) = is_terminal_state_pomdp_planning(s,location(-100.0,-100.0));
    actions(m::POMDP_Planner_2D_action_space,b) = get_actions_non_holonomic(b)
    solver = DESPOTSolver(bounds=IndependentBounds(DefaultPolicyLB(FunctionPolicy(calculate_lower_bound_policy_pomdp_planning_2D_action_space),max_depth=100),
                            calculate_upper_bound_value_pomdp_planning_2D_action_space, check_terminal=true),K=50,D=100,T_max=0.3, tree_in_info=true,
                            rng = rand_noise_generator_for_solver)
    planner = POMDPs.solve(solver, golfcart_2D_action_space_pomdp);

    # receive initial observation from Vicon, convert to POMDP objects
    initial_observation = state_updater_client(true)
    env_right_now.cart.x = initial_observation[1]
    env_right_now.cart.y = initial_observation[2]
    env_right_now.cart.theta = initial_observation[3]

    current_pedestrian_states = Array{human_state,1}()
    pedestrian_id = 1.0
    for i in 4:2:length(initial_observation)
        pedestrian = human_state(initial_observation[i], initial_observation[i+1], 1.0, env_right_now.goals[1], pedestrian_id)
        pedestrian_id += 1
        push!(current_pedestrian_states, pedestrian)
    end

    env_right_now.complete_cart_lidar_data = current_pedestrian_states
    env_right_now.cart_lidar_data = current_pedestrian_states

    # calculate initial belief based on observation
    initial_belief_over_complete_cart_lidar_data = update_belief([], env_right_now.goals, [], env_right_now.complete_cart_lidar_data)
    initial_belief = get_belief_for_selected_humans_from_belief_over_complete_lidar_data(initial_belief_over_complete_cart_lidar_data,
                                                        env_right_now.complete_cart_lidar_data, env_right_now.cart_lidar_data)

    # (?): Does vehicle need to pause at the beginning? Can't it just start moving using a uniform belief?

    # t=0.0 -> t=0.5 sec
    sleep(belief_update_time_step)
    new_observation = state_updater_client(true)
    new_pedestrian_states = Array{human_state,1}()
    pedestrian_id = 1.0
    for i in 4:2:length(initial_observation)
        new_pedestrian = human_state(new_observation[i],new_observation[i+1],1.0,env_right_now.goals[1],pedestrian_id)
        pedestrian_id += 1
        push!(new_pedestrian_states,new_pedestrian)
    end

    env_right_now.complete_cart_lidar_data = new_pedestrian_states
    env_right_now.cart_lidar_data = new_pedestrian_states
    current_belief_over_complete_cart_lidar_data = update_belief(initial_belief_over_complete_cart_lidar_data, env_right_now.goals, current_pedestrian_states, new_pedestrian_states)
    current_belief = get_belief_for_selected_humans_from_belief_over_complete_lidar_data(current_belief_over_complete_cart_lidar_data,
                                                        env_right_now.complete_cart_lidar_data, env_right_now.cart_lidar_data)
    current_pedestrian_states = new_pedestrian_states

    # t=0.5 -> t=1.0 sec
    sleep(belief_update_time_step)
    new_observation = state_updater_client(true)
    new_pedestrian_states = Array{human_state,1}()
    pedestrian_id = 1.0
    for i in 4:2:length(initial_observation)
        new_pedestrian = human_state(new_observation[i],new_observation[i+1],1.0,env_right_now.goals[1],pedestrian_id)
        pedestrian_id += 1
        push!(new_pedestrian_states,new_pedestrian)
    end

    env_right_now.complete_cart_lidar_data = new_pedestrian_states
    env_right_now.cart_lidar_data = new_pedestrian_states
    current_belief_over_complete_cart_lidar_data = update_belief(current_belief_over_complete_cart_lidar_data, env_right_now.goals,
                                                            current_pedestrian_states, new_pedestrian_states)
    current_belief = get_belief_for_selected_humans_from_belief_over_complete_lidar_data(current_belief_over_complete_cart_lidar_data,
                                                        env_right_now.complete_cart_lidar_data, env_right_now.cart_lidar_data)
    current_pedestrian_states = new_pedestrian_states

    # NOTE: planner still needs info on pedestrian positions for collision checking, etc.
    #   - so probably still need to query state_updater once per loop for that info
    #   - belief_updater service should only return the probability distribution for the current belief

    #Get the first action
    b = POMDP_2D_action_space_state_distribution(env_right_now,current_belief)
    action, info = action_info(planner, b)
    a = pomdp2ros_action(action, env_right_now.cart.v, planning_Dt, env_right_now.cart.L)
    println("HG**********")
    println(Dates.now())
    println("**************")
    #Let the while loop begin!
    while end_run == false
        println("**************")

        # 1: publishes current action to ESC
        println(Dates.now())
        ack_publisher_client(a)
        println("\ncontroller: a_k: ", a)

        # 2: receives next observation from Vicon
        println(Dates.now())
        new_observation = state_updater_client(true)
        println("controller: o_k: ", new_observation)
        env_right_now.cart.x = new_observation[1]
        env_right_now.cart.y = new_observation[2]
        env_right_now.cart.theta = new_observation[3]
        env_right_now.cart.v = a[1]
        new_pedestrian_states = Array{human_state,1}()
        pedestrian_id = 1.0
        for i in 4:2:length(initial_observation)
            new_pedestrian = human_state(new_observation[i],new_observation[i+1],1.0,env_right_now.goals[1],pedestrian_id)
            pedestrian_id += 1
            push!(new_pedestrian_states,new_pedestrian)
        end

        # # TO-DO: finish this function
        # #   - (?): issue with passing large env_right_now struct?
        # function ros2pomdp_observation(ros_observation, env_right_now)
        #     env_right_now.cart.x = new_observation[1]
        #     env_right_now.cart.y = new_observation[2]
        #     env_right_now.cart.theta = new_observation[3]
        #     env_right_now.cart.v = a[1]

        #     return 
        # end

        # 3: update environment and the belief
        println(Dates.now())
        env_right_now.complete_cart_lidar_data = new_pedestrian_states
        env_right_now.cart_lidar_data = new_pedestrian_states
        current_belief_over_complete_cart_lidar_data = update_belief(current_belief_over_complete_cart_lidar_data, env_right_now.goals,
                                                                current_pedestrian_states, new_pedestrian_states)
        current_belief = get_belief_for_selected_humans_from_belief_over_complete_lidar_data(current_belief_over_complete_cart_lidar_data,
                                                            env_right_now.complete_cart_lidar_data, env_right_now.cart_lidar_data)
        current_pedestrian_states = new_pedestrian_states

        # stores state/action history
        println(Dates.now())
        push!(o_hist, new_observation)
        push!(a_hist, a)

        dist_to_goal = sqrt( (env_right_now.cart.x - env_right_now.cart.goal.x)^2 + (env_right_now.cart.y-env_right_now.cart.goal.y)^2 )
        if ((num_steps_so_far >= MAX_NUM_STEPS) || (dist_to_goal<=0.5))
            end_run = true
        end
        num_steps_so_far += 1

        # 4: obtain the action for next cycle
        println(Dates.now())
        initial_state = [env_right_now.cart.x,env_right_now.cart.y,env_right_now.cart.theta]
        extra_parameters = [env_right_now.cart.v, env_right_now.cart.L, a[2]]
        x,y,theta = get_intermediate_points(initial_state, 0.5, extra_parameters);
        env_next_step = deepcopy(env_right_now)
        env_next_step.cart.x, env_next_step.cart.y, env_next_step.cart.theta = last(x), last(y), last(theta)

        b = POMDP_2D_action_space_state_distribution(env_next_step,current_belief)
        action, info = action_info(planner, b)
        println("POMDP action @ t_k1: ", action)
        a = pomdp2ros_action(action, env_next_step.cart.v, planning_Dt, env_next_step.cart.L)
        println("vehicle state @ t_k1: ", [env_next_step.cart.x, env_next_step.cart.y, env_next_step.cart.theta])
        println("ROS action @ t_k1: ", a)
        
        # 5: sleeps for remainder of Dt loop
        println(Dates.now())
        sleep(planning_rate)
    end

    # sends [0,0] action to stop vehicle
    ack_publisher_client([0.0, 0.0])
    state_updater_client(false)

    # saves state and action history
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