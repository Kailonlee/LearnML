classdef MDP
    properties
        % envoirment (Pss)
        noise;
        % state
        states; %struct {state(state seq, state), state_value}
        initial_state;% temp no use
        % action
        actions;
        % discount factor
        discount;
        % iteration exit condition
        exit_condition;
        %value iteration number
        iter_num;
        %         current_state;
        %         current_action;
        %         next_state;
    end
    
    methods
        %构造函数
        function item = MDP(state, action, initial_state, noise, discount, exit_condition)
            item.states.state = state;
            item.states.state_value = zeros(length(state),1);
            item.actions = action;
            item.noise = noise;
            item.discount = discount;
            item.exit_condition = exit_condition;
            item.initial_state = initial_state;
        end
        
        % 判断可能的状态是否在取值范围内
        function [flag] = isLegal(obj, current_state)
            if current_state(1,2) >= min(obj.states.state(:,2)) && current_state(1,2) <= max(obj.states.state(:,2))...
                    && current_state(1,3) >= min(obj.states.state(:,3)) && current_state(1,3) <= max(obj.states.state(:,3))
                %                 && next_state(2,1) >= v_host(1) && next_state(2,1) <= v_host(end) ...
                %                     && next_state(2,2) >= d_lead(1) && next_state(2,2) <= d_lead(end)
                flag = 1;
            else
                flag = 0;
            end
        end
        
        %获取下一个状态和对应概率
        function [next_states] = getStateProb(obj, current_state, current_action)
            next_states = cell(2,2);
            if  strcmp(current_action, 'speed_up') % %%
                next_state(1,2) = current_state(1,2) + 5;%速度加5；
                next_state(1,3) =  current_state(1,3) - 10;%距离缩小10
                prob(1) = 1 - obj.noise;
                
                next_state(2,2) = current_state(1,2) + 5;%速度加5；
                next_state(2,3) =  current_state(1,3);%距离不变（前车可能加速）
                prob(2) = obj.noise;
                
            elseif  strcmp(current_action, 'slow_down')
                next_state(1,2) = current_state(1,2) - 5;%速度减5；
                next_state(1,3) =  current_state(1,3) + 10;%距离缩小10
                prob(1) = 1 - obj.noise;
                
                next_state(2,2) = current_state(1,2) - 5;%速度减5；
                next_state(2,3) = current_state(1,3);%距离不变（前车可能加速）
                prob(2) = obj.noise;
            end

            if(~(obj.isLegal(next_state(1,:))) && ~(obj.isLegal(next_state(2,:)))) %('next_states is illegal!')
                next_states = [];
            elseif (~(obj.isLegal(next_state(1,:))))
                next_state(2,1) = obj.getStateSeq(next_state(2,2:3));
                prob(2) = 1;%第二个状态概率为1
                next_states(2,:) = {next_state(2,:), prob(2)};
                next_states(1,:) = [];
            elseif ~(obj.isLegal(next_state(2,:)))
                next_state(1,1) = obj.getStateSeq(next_state(1,2:3));
                prob(1) = 1;%第二个状态概率为1
                next_states(1,:) = {next_state(1,:), prob(1)};
                next_states(2,:) = [];
            else
                next_state(1,1) = obj.getStateSeq(next_state(1,2:3));
                next_state(2,1) = obj.getStateSeq(next_state(2,2:3));
                next_states{1,1} = next_state(1,:);
                next_states{2,1} = next_state(2,:);
                next_states{1,2} = prob(1);
                next_states{2,2} = prob(2);
            end
            

        end
        
        %获取某一状态下可能的动作
        function [possible_action] = getPossibleAction(obj, current_state)
            if ~obj.isLegal(current_state)
                possible_action = [];
            else
                possible_action = obj.actions;
            end
        end
        
        %在某一状态下随机获取一个动作
        function [action_to_take] = getAction(obj, current_state)
            possible_action = obj.getPossibleAction(current_state);
            if isempty(possible_action)
                fprintf('This state %d m/s %d m has no legal action', current_state(1,2),current_state(1,3));
                action_to_take = [];
            else
                action_to_take = char(possible_action(randi(2)));% policy是随机的
            end
        end
        
        %随机执行可能的动作，返回未来可能的状态和获得的奖励
        function [next_state, reward] = takeAction(obj, current_state, action_to_take)
            ra = randsrc(1,1,1:100) / 100; %按照noise的概率到达下一状态
            next_states = cell2mat(obj.getStateProb(current_state, action_to_take));
            reward = obj.getReward(current_state, action_to_take);
            [row,~] = size(next_states);
            if row == 2  %如果下一时刻有两种可能状态，按照Pss选择下一个状态
                if ra > obj.noise
                    next_state = next_states(1,1:3);
                else
                    next_state = next_states(2,1:3);
                end
            elseif row < 2
                next_state = next_states(1,1:3);
            end
        end
        
        %获取某状态下的reward
        function [reward] = getReward(obj, current_state, current_action)
            vel = current_state(1,2);
            dist = current_state(1,3);
            %%%加入前车速度 传感输入
            if vel > 30 && dist < 30 %危险
                reward = -100;
            elseif vel > 30 && dist == 30 %不安全
                reward = -50;
            elseif vel > 30 && dist > 30 %安全
                reward = 0;
            elseif vel == 30 && dist < 30 % 不安全
                reward = -50;
            elseif vel == 30 && dist == 30 % 优秀
                reward = 100;
            elseif vel == 30 && dist > 30 % 安全
                reward = 0;
            elseif vel < 30 && dist < 30 %安全
                reward = 0;
            elseif vel < 30 && dist == 30 %安全
                reward = 50;
            elseif vel < 30 && dist > 30%距离过大
                reward = 0;
            end
        end
        
        %运行单个状态的episode，返回单个episode的return
        function [returns] = runEpisode(obj, current_state, episode_num)
            seq = obj.getStateSeq(current_state);
            current_state(1,2:3) = current_state;
            current_state(1,1) = seq; %添加状态的序号
            returns = 0;
            count = 0;
            while 1
                action_to_take = obj.getAction(current_state);
                if isempty(obj.getStateProb(current_state, action_to_take)) %直到下一时刻的状态非法，退出循环
                    fprintf('Next state is illegal!, episode number = %d, return = %d \n', episode_num, returns);
                    return;
                else
                    [next_state, reward] = obj.takeAction(current_state, action_to_take);
                    current_action = action_to_take;
                    fprintf('No.%d Current state: vel=%d m/s, dist = %d m  Action to take: %s   Next state: vel=%d m/s, dist=%d m  Reward = %d\n', ...
                        count+1,current_state(1,2),current_state(1,3), current_action, next_state(1,2), next_state(1,3), reward);
                    returns = returns +reward * (obj.discount ^ count);
                    current_state = next_state;
                    count = count + 1;
                end
            end
        end

        %获取状态的序号
        function [seq] = getStateSeq(obj, next_state)
            for i = 1:1:length(obj.states.state)
                if next_state(1) == obj.states.state(i,2)
                    if next_state(2) == obj.states.state(i,3)
                        seq = i;
                        break
                    end
                end
            end
        end
        
        %获取某个状态的value
        function state_value = getStateValue(obj,current_state)
            seq = current_state(1,1);
            state_value = obj.states.state_value(seq);
        end
        
        %计算每个状态的q value
        function [Q_value] = computeQValue(obj, current_state, current_action)
            next_states = obj.getStateProb(current_state, current_action);
            if isempty(next_states)
                Q_value = null(1);
                return;
            end
            prob = cell2mat(next_states(:,2));
            next_state = cell2mat(next_states(:,1));
            Q_value = 0;
            [row,~] = size(next_state);
            reward = obj.getReward(current_state, current_action);
            for i = 1:1:row
                Q_value = Q_value + prob(i) * (reward + obj.discount * obj.getStateValue(next_state(i,:)));
            end
        end
        
        %按照当前状态的Q_value，选择使得value最大的action
        function [optimal_action]=computeAction(obj, current_state)
            possible_action = obj.getPossibleAction(current_state);
            flag = true;
            for i = 1:1:length(possible_action)
                Q_value = obj.computeQValue(current_state, possible_action(i));
                if isempty(Q_value)
                    continue;
                end
                if flag %给max_Q_value赋值的flag
                    max_Q_value = Q_value;
                    optimal_action = char(possible_action(i));
                    flag = false;
                end
                if Q_value > max_Q_value  %update
                    max_Q_value = Q_value;
                    optimal_action = char(possible_action(i));
                end
            end
        end
        
        %值迭代求每个状态的最优value
        function obj = valueIteration(obj)
            obj.iter_num = 0;%迭代次数
            iter_this_value = zeros(length(obj.states.state_value),1);
            while 1
                obj.iter_num = obj.iter_num +1;
                iter_pre_value = obj.states.state_value;
                delta = 0;
                for i = 1:1:length(obj.states.state)
                    current_state = obj.states.state(i,:);
                    possible_action = obj.getPossibleAction(current_state);
                    if isempty(possible_action)
                        continue;
                    end
                    for j = 1:1:length(possible_action)
                        Q_value = obj.computeQValue(obj.states.state(i,:), possible_action(j));
                        if isempty(Q_value)
                            continue;
                        end
                        if j == 1
                            max_Q_value = Q_value;
                        end
                        if Q_value > max_Q_value
                            max_Q_value = Q_value;
                        end
                    end
                    iter_this_value(i) = max_Q_value;%每次迭代中，每个状态的value是max_Q_value，每次迭代都更新
                    error = abs(iter_pre_value(i) - iter_this_value(i));%当前迭代的value与前一时刻迭代的value的差值
                    delta = max(delta, error);%选择所有差值中最大的一个作为标准
                end
                % 修改当前时刻所有状态的value
                obj.states.state_value = iter_this_value;
                if delta < obj.exit_condition
                    break
                end  
            end%end while
        end%end function
        
    end %end method
end %end classdef
