function [state] = getState(state_1, state_2)
state = zeros(length(state_1) * length(state_2), 2);
count = 1;
for i = 1:1:length(state_1)
    for j = 1:1:length(state_2)
        state(count,1) = count;
        state(count, 2:3) = [state_1(i), state_2(j)];
        count = count +1;
    end
end
end