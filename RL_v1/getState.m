function [states, actions] = getState(state_1_min, state_1_max, state_1_gap, state_2_min, state_2_max, state_2_gap, action_min, action_max, action_gap)
state_1_num = (state_1_max - state_1_min) / state_1_gap + 1;
state_2_num = (state_2_max - state_2_min) / state_2_gap + 1;
action_num = (action_max - action_min) / action_gap +1;
state_1 = zeros(state_1_num, 1);
state_2 = zeros(state_2_num, 1);
states = cell(state_1_num * state_2_num, 1);
actions = cell(action_num, 1);
count = 1;
for s = state_1_min : state_1_gap : state_1_max
    state_1(count, 1) = roundn(state_1_min + state_1_gap * (count -1), -1);
    count  = count +1;
end
count = 1;
for s = state_2_min : state_2_gap : state_2_max
    state_2(count, 1) = state_2_min + state_2_gap * (count -1);
    count  = count +1;
end
% states = {state_1, state_2};
count = 1;
for i = 1:1:length(state_1)
    for j = 1:1:length(state_2)
        states{count}= mat2str([state_1(i), state_2(j)]);
        count = count +1;
    end
end
count = 1;
for a = action_min: action_gap : action_max
    actions{count} = num2str(action_min + action_gap * (count - 1));
    count = count + 1;
end

end