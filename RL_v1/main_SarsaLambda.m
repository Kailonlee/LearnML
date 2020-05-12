clc
clear
import Envoirment .*
import SarsaLambda .*
%% ready for RL
% host car velicity
% v_host = ABS_VehicleSpeed;
% distance from host car to lead car
% d_lead = Objects_Longitudial_Dist_Obj0;
% lead car velocity
% v_lead = ABS_VehicleSpeed + Objects_Relative_Velocity_Obj0;
%====================
% create leading car velocity data
T = 1; %unit : s
time = 0 : 1 : 60;
v_lead = getVLead(time);
%====================
% state limits
state_1_min = 0;
state_1_max = 30;
state_1_gap = 2;
state_2_min = 0;
state_2_max = 100;
state_2_gap = 5;
limits = struct('state_1_min', state_1_min, 'state_1_max', state_1_max, 'state_1_gap', state_1_gap, ...
                      'state_2_min', state_2_min, 'state_2_max', state_2_max, 'state_2_gap', state_2_gap);
%====================
% action limits
action_min = -4;
action_max = 4;
action_gap = 0.5;
%====================
% create states and action cell;
[states, actions] = getState(state_1_min, state_1_max, state_1_gap, state_2_min, state_2_max, state_2_gap, action_min, action_max, action_gap);
%% Parameter definition
discount = 0.8;
alpha = 0.02;
gama = 0.9;
epsilon = 0.2;
lambda = 0.9;
observation_num = 2;
episode_num = 20000;
initial_state = '[20 30]';
%% start envoirment
env = Envoirment(v_lead, 0,1,T,observation_num, limits);
%% start Sarsa
salam = SarsaLambda(states, actions, alpha, gama, epsilon, lambda);
try
    for epi = 1:1:episode_num
        env = env.reset();
        current_state = initial_state;
        action_to_take = salam.getEpsilonAction(current_state);
        while 1
            fprintf('-----------------------------------\n');
            fprintf('[step] %d \n', env.step);
            env = env.update();
            if ~env.status
                fprintf('[break] sensor data over \n');
                break;
            end
            [state_prime, reward, done] = env.takeStep(current_state, action_to_take);
            action_to_take_prime = salam.getEpsilonAction(state_prime);
            %for display
            %===========================================
            current_state_ = eval(current_state);
            fprintf('[State] %.2fm/s %dm \n', current_state_(1),current_state_(2));
            fprintf('[V_lead] %.2fm/s \n', env.v_lead(env.step));
            fprintf('[Reward] %.2f \n', reward);
            %===========================================
            salam = salam.updateLearn(current_state, state_prime, action_to_take, action_to_take_prime, reward);
            if done
                break;
            end
            current_state = state_prime;
            action_to_take = action_to_take_prime;
        end
        fprintf('[Episode]: No.%d \n', epi);
    end
catch
    sendWechat('主人，你的程序跑崩了！！！');
end
sendWechat(['一共迭代了 ', num2str(epi), ' 次']);



