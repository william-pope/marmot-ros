#!/usr/bin/env julia

using RobotOS

han_path = "/home/adcl/Documents/human_aware_navigation/src/"
include(han_path * "environment.jl")
include(han_path * "utils.jl")
include(han_path * "two_d_action_space_pomdp.jl")
include(han_path * "belief_tracker.jl")
include(han_path * "new_main_2d_action_space_pomdp.jl")

# File Overview:
# - requests state from state_updater, uses to update and store current belief internally (10 Hz)
# - returns belief to controller when requested (2 Hz)

# - should have a main() script to run belief updates as master
# - should have global variables to store most recent distribution
# - need to set up new service for get_belief_update
# - need to attach to existing get_state_update service

@rosimport belief_estimator_pkg.srv: UpdateBelief
@rosimport state_updater_pkg.srv: UpdateState

rostypegen()
using .belief_updater_pkg.srv
using .state_updater_pkg.srv

# set up request to state_updater service
function state_updater_client(record)
    wait_for_service("/car/state_updater/get_state_update")
    update_state_srv = ServiceProxy{UpdateState}("/car/state_updater/get_state_update")

    resp = update_state_srv(UpdateStateRequest(record))

    return resp.state
end

        

        self.update_state_srv = rospy.Service(
            "/car/state_updater/get_state_update",
            UpdateState,
            self.update_state)

        return

    

    # function called by service
    def update_state(self, req):
        self.record_hist = req.record

        if self.record_hist == False and self.saved_hist == False and len(self.hist_veh_msg) > 0: 
            self.save_s_hist()

        current_state = [0]*(3 + 2*(0))
        current_state[0:3] = self.veh_state(self.current_veh_msg)
        current_state[3:5] = self.ped_state(self.current_ped1_msg)
        # current_state[5:7] = self.ped_state(self.current_ped2_msg)
        # current_state[7:9] = self.ped_state(self.current_ped3_msg)
        # current_state[9:11] = self.ped_state(self.current_ped4_msg)

        return UpdateStateResponse(current_state)