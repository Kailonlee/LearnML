classdef MDP
    properties
        % fake sensor info, update every step
        sensor = struct('step', 0, 'status', 1);
        v_lead_total;
        v_lead;
        v_lead_prime;
        % sample time
        T = 1;
        % envoirment (Pss)
        noise;
        % state
        states; %struct {state(state seq, state), state_value}
        initial_state = struct('seq',[], 'data',[]);
        num_of_state;
        limits = struct('state_1_min', [], 'state_1_max', [], 'state_1_gap', [], ...
                              'state_2_min', [], 'state_2_max', [], 'state_2_gap', []);
        v_set;
        d_set;
        v_set_prime;
        d_set_prime;
        d_min_safe = 5;
        % action
        actions;
        num_of_action;
        % Matrix Q
        Q_mat;
        % discount factor
        discount;
        % learning rate
        alpha = 0.5;
        % Epsilon greedy
        epsilon = 0.2;
        % value iteration exit condition
        exit_condition;
        %value iteration number
        iter_num;
        %         current_state;
        %         current_action;
        %         next_state;
    end
    
    methods
        %���캯��
        function obj = MDP(states, actions,discount, limits, sensor)
            obj.states = states;
            obj.actions = actions;
            obj.v_lead_total = sensor;
            obj.limits  = limits;
            obj.discount = discount;
            obj.num_of_state = length(states);
            obj.num_of_action = length(actions);
            obj.Q_mat = zeros(obj.num_of_state, obj.num_of_action);
        end
        
        % ���µ�ǰ������
        function [obj] = updateFakeSensor(obj)
            if obj.sensor.step <= (length(obj.v_lead_total) -2)
                obj.v_lead = obj.v_lead_total(obj.sensor.step +1);
                obj.v_lead_prime = obj.v_lead_total(obj.sensor.step + 2);
                obj.sensor.step = obj.sensor.step + 1;
            else
                obj.sensor.status = 0;
            end
        end
        
        function [obj] = resetFakeSensor(obj)
            obj.sensor.step = 0;
            obj.sensor.status = 1;
        end
        
        % ��ȡ�����ʼ״̬
        function [obj] = getRandInitialState(obj)
            rand_state_seq = randi(obj.num_of_state);
            obj.initial_state = obj.states(rand_state_seq);
        end
        
        % Q-learning, updating
        function [obj] = updateQvalue(obj, episode_num)
            for epi = 1:1: episode_num 
                obj = obj.resetFakeSensor();
                obj = obj.getRandInitialState(); %������³�ʼ״̬
                current_state = obj.initial_state;
                for i = 1 : 1 : obj.num_of_state
                    obj = obj.updateFakeSensor();
                    if ~obj.sensor.status
                        fprintf('[break] sensor data over \n');
                        break;
                    end
                    if (~obj.isLegal(current_state))
                        fprintf('[break] current state is illegal \n');
                        break
                    end
                    if (obj.isAbsorb(current_state))
                        fprintf('[break] current state is Absorb \n');
                        break
                    end
                    fprintf('[State] current state %.2fm/s %dm \n', current_state.data(1), current_state.data(2));
                    fprintf('[V_lead] %.1fm/s \n', obj.v_lead);
                    choose = randi(100) / 100;
                    % epsilon-greedy
                    if choose > obj.epsilon
                        %������ѡ���Ŷ���
                        [action_to_take] = obj.computeAction(current_state);
                        fprintf('[Optimal Action] action to take %.1f \n', action_to_take.data(1));
                    else
                        %̽�������ѡ����
                        [action_to_take] = obj.actions(randi(obj.num_of_action));
                        fprintf('[Random Action] action to take %.1f \n', action_to_take.data(1));
                    end
                    next_state = obj.getNextState(current_state, action_to_take);
                    if isempty(next_state)
                        fprintf('[break] next state is empty \n');
                        break
                    end
                    reward =  obj.getReward(current_state, next_state);
                    fprintf('[Reward] %d \n', reward);
                    fprintf('[Step] %d \n', obj.sensor.step);
                    s_seq = current_state.seq;
                    a_seq = action_to_take.seq;
                    obj.Q_mat(s_seq, a_seq) = obj.Q_mat(s_seq , a_seq) * (1 - obj.alpha) + obj.alpha * (reward + obj.discount * obj.getMaxQValue(next_state));
                    current_state = next_state;
                end
                fprintf('episode: No.%d \n', epi);
            end
            fprintf('episode_num:%d is done \n', episode_num);
        end
        
        % ��ȡĳ״̬������Q_value
        function [max_Q_value] = getMaxQValue(obj, next_state)
            for i = 1:1:obj.num_of_action
                if i == 1
                    max_Q_value = obj.Q_mat(next_state.seq, i);
                end
                if  obj.Q_mat(next_state.seq,i) > max_Q_value
                    max_Q_value = obj.Q_mat(next_state.seq,i);
                end
            end  
        end
        
        % ��ȡĳ״̬��ִ��ĳ������Q_value
        function [Q_value] = getQValue(obj, current_state, current_action)
            Q_value = obj.Q_mat(current_state.seq, current_action.seq);
        end
        
        % �жϿ��ܵ�״̬�Ƿ���ȡֵ��Χ��
        function [flag] = isLegal(obj, current_state)
            v_host_ = current_state.data(1);
            d_lead_ = current_state.data(2);
            if v_host_ >= obj.limits.state_1_min && v_host_ <= obj.limits.state_1_max ...
                    && d_lead_ >= obj.limits.state_2_min && d_lead_ <= obj.limits.state_2_max
                flag = 1;
            else
                flag = 0;
            end
        end
        
        % �ж��Ƿ���������״̬���ȶ�����������
        function [flag] = isAbsorb(obj, current_state)
            v_host_ = current_state.data(1);
            d_lead_ = current_state.data(2);
            d_best = roundn(1.5 * obj.v_lead, -1); %Ϊ�˺�״̬ƥ�䣬���ﱣ��һλС��������Ϊ0.1
            
            if abs(v_host_ - obj.v_lead) <= 0.5 && abs(d_lead_ - d_best) <= 5 %�ȶ���������
                flag = 1;
            else
                flag = 0;
            end
        end
        
        % predict next state and probability with the Model
        function [next_state] = getNextState(obj, current_state, current_action)
%             if obj.isAbsorb(current_state)
%                 next_state = current_state;
%                 return
%             end
            acc_seq = current_action.seq;
            acc = current_action.data;
            v_host_ = current_state.data(1);
            d_lead_ = current_state.data(2);
            dist_gap = obj.limits.state_2_gap;
            v_host = v_host_ + acc * obj.T; 
            d_lead = d_lead_ + dist_gap * round((obj.v_lead - v_host) * obj.T / dist_gap); %����ǰ���ٶ��ȶ�
            next_state.data = [roundn(v_host, -1), round(d_lead)];
            if ~obj.isLegal(next_state)
                next_state = [];
                return;
            end  
            try
            next_state.seq = obj.getStateSeq(next_state.data); 
            catch
                fprintf('current_state: %d , %d \n', current_state.data(1), current_state.data(2));
                fprintf('next_state: %d , %d\n', next_state.data(1), next_state.data(2));
                disp(obj.sensor.step);
            end
%             if  strcmp(current_action, 'Rapid acceleration')
%                 acc = obj.acc_ra;
%             elseif strcmp(current_action,'acceleration')
%                 acc = obj.acc_a;
%             elseif strcmp(current_action,'hold on')
%                 acc = obj.acc_ho;
%             elseif strcmp(current_action,'deceleration')
%                 acc = obj.acc_d;
%             elseif strcmp(current_action, 'Abrupt deceleration')
%                 acc = obj.acc_ad;
%             end
        end
        
        %��ȡĳһ״̬�¿��ܵĶ���
        function [possible_action] = getPossibleAction(obj, current_state)
            if ~obj.isLegal(current_state)
                possible_action = [];
            else
                possible_action = obj.actions;
            end
        end
        
        %��ĳһ״̬�������ȡһ������
        function [action_to_take] = getAction(obj, current_state)
            possible_action = obj.getPossibleAction(current_state);
            if isempty(possible_action)
                fprintf('This state %d m/s %d m has no legal action', current_state(1,2),current_state(1,3));
                action_to_take = [];
            else
                action_to_take = char(possible_action(randi(2)));% policy�������
            end
        end
        
        %���ִ�п��ܵĶ���������δ�����ܵ�״̬�ͻ�õĽ���
        function [next_state, reward] = takeAction(obj, current_state, action_to_take)
            ra = randsrc(1,1,1:100) / 100; %����noise�ĸ��ʵ�����һ״̬
            next_states = cell2mat(obj.getStateProb(current_state, action_to_take));
            reward = obj.getReward(current_state, action_to_take);
            [row,~] = size(next_states);
            if row == 2  %�����һʱ�������ֿ���״̬������Pssѡ����һ��״̬
                if ra > obj.noise
                    next_state = next_states(1,1:3);
                else
                    next_state = next_states(2,1:3);
                end
            elseif row < 2
                next_state = next_states(1,1:3);
            end
        end
        
        %��ȡĳ״̬�µ�reward
        function [reward] = getReward(obj, current_state, next_state)
            v_host = current_state.data(1);
            d_lead = current_state.data(2);
            v_host_prime = next_state.data(1);
            d_lead_prime = next_state.data(2);
            d_best = roundn(1.5 * obj.v_lead, -1);
            d_best_prime = roundn(1.5 * obj.v_lead_prime, -1);
            v_rel = obj.v_lead - v_host;%����ٶ�
            v_rel_prime = obj.v_lead_prime - v_host_prime;%��һʱ������ٶ�
            obj.v_set = obj.v_lead;
            obj.v_set_prime = obj.v_lead_prime;
            obj.d_set = d_best;  %ֻ��ǰ������ʱ������Ѿ���
            obj.d_set_prime = d_best_prime;
            delta_d = obj.d_set - d_lead;
            delta_d_prime = obj.d_set - d_lead_prime;
            reward = 0;
            if abs(v_rel) <= 0.5 && abs(d_lead - d_best) <= 2
                reward = 100;
            elseif v_rel > 0.5 %����ײ����
                if d_lead > 2    
                    delta_v = abs(obj.v_set - v_host);
                    delta_v_prime = abs(obj.v_set_prime - v_host_prime);
                    if v_host_prime >= v_host
                        r1 = 10;
                    else
                        r1 = -10;
                    end
                    if  delta_v > delta_v_prime && delta_v_prime < 0.5 && abs(delta_d_prime) < 2 %��Ϊ��һʱ���ٶ�ƫ���С����
                        r2 = 100;
                    elseif abs(delta_d_prime) < abs(delta_d)
                        r2 = 10;
                    else
                        r2 = -10;
                    end
                    reward = r1+r2;
                else
                    reward = -10;
                end
            elseif v_rel < -0.5 %����ײ���գ���������
                if delta_d >= 0 %��ǰ����С�����ֵ�� bad, �ж��Ƿ���٣�����good, ����bad
                    if v_host_prime < v_host  && abs(delta_d_prime) < abs(delta_d)  %��һʱ�̼��٣�������ã� good
                           reward = 10;
                    else  %bad bad
                           reward = -10;
                    end
                else %��ǰ����������ֵ, good�� �ж��Ƿ����, ����good�� ����bad
                    if v_host_prime > v_host && abs(delta_d_prime) < abs(delta_d)
                        reward = 10;
                    else
                        reward = 0; %�������
                    end
                end
%                 if v_rel_prime < 0    %��һʱ������ٶ�С��0  bad
%                     % �жϱ������޼��٣����޼��� bad bad
%                     if v_host > v_host_prime && abs(delta_d_prime) < abs(delta_d)  %not too bad
%                         reward = 10;
%                     else  %bad bad
%                         reward = -10;
%                     end
%                 else                       %��һʱ������ٶȴ���0  good
%                     if abs(delta_d) <= 2  && v_host_prime >= v_host  % ��ǰ�������ţ�Ӧ�ü���
%                         reward = 10;
%                     elseif abs(delta_d) <= 2  && v_host_prime < v_host
%                         reward = 0;
%                     elseif delta_d < -2 && v_host_prime >= v_host
%                         reward = 10;
%                     elseif delta_d < -2 && v_host_prime < v_host
%                         reward = 0;
%                     elseif delta_d > 2 && v_host_prime >= v_host
%                         reward = -10;
%                     else
%                         reward = 0;
%                     end
%                 end
            end
%             if v_rel > 0          %ǰ������
%                 delta_v = abs(obj.v_set - v_host_prime);
%                 if v_host_prime >= v_host
%                     r1 = 10;
%                 else
%                     r1 = -10;
%                 end
%                 if delta_v >= 0 || delta_v < 0.5
%                     r2 = 100;
%                 else
%                     r2 = -1;
%                 end
%                 reward = r1+r2;
%             elseif v_rel <= 0    %ǰ������
%                 obj.d_set = d_best;  %ֻ��ǰ������ʱ������Ѿ���
%                 delta_d = obj.d_set - d_lead_prime;
%                 if v_rel_prime < 0    %��һʱ������ٶ�С��0
%                     delta_v  = obj.v_set - v_host_prime;
%                 else                        %��һʱ������ٶȴ���0
%                     delta_v  = v_host - obj.v_lead;
%                 end
%                 if delta_d > obj.d_min_safe &&  delta_v > 0 && delta_v < 1
%                     reward = 100;
%                 elseif delta_d < obj.d_min_safe && delta_v < 0
%                     reward = -1;
%                 elseif delta_d < -10 && delta_v < 3
%                     reward = -10;
%                 end
%             end
        end
        
        %���е���״̬��episode�����ص���episode��return
        function [returns] = runEpisode(obj, current_state, episode_num)
            seq = obj.getStateSeq(current_state);
            current_state(1,2:3) = current_state;
            current_state(1,1) = seq; %���״̬�����
            returns = 0;
            count = 0;
            while 1
                action_to_take = obj.getAction(current_state);
                if isempty(obj.getStateProb(current_state, action_to_take)) %ֱ����һʱ�̵�״̬�Ƿ����˳�ѭ��
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

        %��ȡ״̬�����
        function [seq] = getStateSeq(obj, next_state)
            for i = 1:1:obj.num_of_state
                if obj.states(i).data(1) == next_state(1)
                    if obj.states(i).data(2) == next_state(2)
                        seq = obj.states(i).seq;
                        return
                    end
                end
            end
        end
        
        %��ȡĳ��״̬��value
        function state_value = getStateValue(obj,current_state)
            seq = current_state(1,1);
            state_value = obj.states.state_value(seq);
        end
        
        %����ÿ��״̬��q value
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
        
        %���յ�ǰ״̬��Q_value��ѡ��ʹ��value����action
        function [optimal_action]=computeAction(obj, current_state)
            possible_action = obj.getPossibleAction(current_state);
            for i = 1:1:length(possible_action)
                current_action = possible_action(i);
                Q_value = obj.getQValue(current_state, current_action);
                if i == 1
                    max_Q_value = Q_value;
                    optimal_action = current_action;
                end
                if Q_value > max_Q_value  %update
                    max_Q_value = Q_value;
                    optimal_action = current_action;
                end
            end
        end
        
        %ֵ������ÿ��״̬������value
        function obj = valueIteration(obj)
            obj.iter_num = 0;%��������
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
                    iter_this_value(i) = max_Q_value;%ÿ�ε����У�ÿ��״̬��value��max_Q_value��ÿ�ε���������
                    error = abs(iter_pre_value(i) - iter_this_value(i));%��ǰ������value��ǰһʱ�̵�����value�Ĳ�ֵ
                    delta = max(delta, error);%ѡ�����в�ֵ������һ����Ϊ��׼
                end
                % �޸ĵ�ǰʱ������״̬��value
                obj.states.state_value = iter_this_value;
                if delta < obj.exit_condition
                    break
                end  
            end%end while
        end%end function
        
    end %end method
end %end classdef
