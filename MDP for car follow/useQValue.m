% load('iter_20000.mat');
%%
M = M.resetFakeSensor();
M = M.getRandInitialState();
fprintf('initial state %.2fm/s %dm \n', M.initial_state.data(1), M.initial_state.data(2));
current_state = M.initial_state;
while 1
    M = M.updateFakeSensor();
    if ~M.sensor.status
        fprintf('[break] sensor data over \n');
        break;
    end
    if (~M.isLegal(current_state))
        fprintf('[break] current state is illegal \n');
        break
    end
%     if (M.isAbsorb(current_state))
%         fprintf('[break] next state is Absorb \n');
%         break
%     end
    fprintf('[State] current state %.2fm/s %dm \n', current_state.data(1), current_state.data(2));
    fprintf('[V_lead] %.1fm/s \n', M.v_lead);
    [action_to_take] = M.computeAction(current_state);
    fprintf('[Optimal Action] action to take %.1f \n', action_to_take.data(1));
    next_state = M.getNextState(current_state, action_to_take);
    if isempty(next_state)
        fprintf('[break] next state is empty \n');
        break
    end
    reward =  M.getReward(current_state, next_state);
    fprintf('[Reward] %d \n', reward);
    fprintf('[Step] %d \n', M.sensor.step);
    state_1_plot(M.sensor.step) = current_state.data(1);
    state_2_plot(M.sensor.step) = current_state.data(2);
    current_state = next_state;
end
subplot(121)
plot(M.v_lead_total, 'LineWidth', 2)
hold on
plot(state_1_plot, 'LineWidth', 2);
title('Following Speed')
xlabel('Time(s)');
ylabel('Speed(m/s)');
legend('speed of leading car', 'speed of host car');
subplot(122)
plot(M.v_lead_total * 1.5, 'LineWidth', 2)
hold on
plot(state_2_plot, 'LineWidth', 2);
title('Following Distance')
xlabel('Time(s)');
ylabel('Distance(m)');
legend('best distance', 'actual distance');

