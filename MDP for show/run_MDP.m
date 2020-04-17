clc
clear
import MDP .*
%% Parameter definition
% host car velicity
v_host = [10,15,20,25,30,35,40,45,50];
% distance from host car to lead car
d_lead = [10,20,30,40,50,60,70,80,90,100]; 
% action set
action = {'speed_up', 'slow_down'};
initial_state = [40,70];
% get state set from vector(v_host) and vector(d_lead);
state = getState(v_host, d_lead);
% envoirment noise
noise = 0.2;
%discount factor
discount = 0.9;
% value iteration exit condition
exit_condition = 0.1;

%% start a MDP
M = MDP(state, action, initial_state, noise, discount, exit_condition);

%% for a certain state with a given episode number, calculate and display total return of all possible episode
% current_state = [30,30];
% total_returns = 0;
% episode_num = 10;
% for i = 1:1:episode_num
%     total_returns = total_returns + M.runEpisode(current_state, i);
% end
% fprintf('Total return = %d in %d episodes', total_returns, episode_num);

%% running value iteration, get optimal policy
M=M.valueIteration();
fprintf('value iteration number: %d \n', M.iter_num);
len_of_state = length(M.states.state);
optimal_action = zeros(len_of_state,1);
x_state = cell(1,len_of_state);
for i = 1:1:len_of_state
    optimal_action = M.computeAction(M.states.state(i,:));
    fprintf('State: %d m/s  %d m , Optimal Action: %s \n', M.states.state(i,2),M.states.state(i,3), optimal_action);
    x_state{1,i} = num2str(M.states.state(i,2:3)); 
end

%% draw bar
bar(M.states.state(:,1),M.states.state_value);
title('Optimal Value Function for Every State');
set(gca, 'XTick', M.states.state(:,1));
set(gca, 'XTickLabel', x_state);
rotateticklabel(gca,45);

