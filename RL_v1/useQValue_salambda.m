clc
clear
close all
load('D:\5_Work\MDP\MDP_test\RL\mat_data\SarsaLambda\20000.mat');
count = 1;
env = env.reset();
% current_state = initial_state;
current_state = '[10 15]';
while 1
    fprintf('-----------------------------------\n');
    fprintf('[step] %d \n', env.step);
    env = env.update();
    if ~env.status
        fprintf('[break] sensor data over \n');
        break;
    end
    action_to_take = salam.getOptimalAction(current_state);
    [state_prime, reward, done] = env.takeStep(current_state, action_to_take);
    %for display
    %===========================================
    current_state_ = eval(current_state);
    fprintf('[State] %.2fm/s %dm \n', current_state_(1),current_state_(2));
    fprintf('[V_lead] %.2fm/s \n', env.v_lead(env.step));
    fprintf('[Reward] %.2f \n', reward);
    %===========================================
    if done
        fprintf('[break] next state is illegal \n');
        break;
    end
    current_state = state_prime;
    state_for_plot(count, 1)= current_state_(1);
    state_for_plot(count, 2)= current_state_(2);
    count = count+1;
end
%% plot
subplot(121)
plot(env.v_lead, 'LineWidth', 2)
hold on
plot(state_for_plot(:,1), 'LineWidth', 2);
title('Following Speed')
xlabel('Time(s)');
ylabel('Speed(m/s)');
legend('speed of leading car', 'speed of host car');
subplot(122)
plot(env.v_lead .* 1.5, 'LineWidth', 2)
hold on
plot(state_for_plot(:,2), 'LineWidth', 2);
title('Following Distance')
xlabel('Time(s)');
ylabel('Distance(m)');
legend('best distance', 'actual distance');
% figure
% x=action_min: action_gap: action_max;
% y=1:1:length(M.Q_mat);
% [X, Y]=meshgrid(x,y);
% Z=M.Q_mat;
% surf(X,Y,Z)
