clc
clear
import MDP .*
%% Parameter definition
% host car velicity
    % v_host = ABS_VehicleSpeed;
% distance from host car to lead car
    % d_lead = Objects_Longitudial_Dist_Obj0; 
% lead car velocity
    % v_lead = ABS_VehicleSpeed + Objects_Relative_Velocity_Obj0;
time = 0 : 1 : 60;
v_lead = getVLead(time);
state_1_min = 0;
state_1_max = 30;
state_1_gap = 0.5;
state_2_min = 0;
state_2_max = 100;
state_2_gap = 2;
action_min = -5;
action_max = 5;
action_gap = 0.5;
initial_state = [40,70];
limits = struct('state_1_min', state_1_min, 'state_1_max', state_1_max, 'state_1_gap', state_1_gap, ...
                              'state_2_min', state_2_min, 'state_2_max', state_2_max, 'state_2_gap', state_2_gap);
% get state and action set from vector(v_host) and vector(d_lead);
[states, actions] = getState(state_1_min, state_1_max, state_1_gap, state_2_min, state_2_max, state_2_gap, action_min, action_max, action_gap);
% envoirment noise
noise = 0.2;
%discount factor
discount = 0.9;
% value iteration exit condition
exit_condition = 0.1;

%% start a MDP
M = MDP(states, actions , discount, limits, v_lead);
%% if Q_mat is not converge, just run follow command
M = M.updateQvalue(10000);
save('iter_30000.mat')
