classdef RL
    properties
        states;
        state_num;
        actions;
        alpha;
        gama;
        epsilon;
        Q_table;
    end
    
    methods(Abstract)
        updateLearn(obj);
    end
    
    methods
        function obj = RL(states_, actions_, alpha_, gama_, epsilon_)
            obj.states = states_;
            obj.actions = actions_;
            obj.alpha = alpha_;
            obj.gama = gama_;
            obj.epsilon = epsilon_;
            row = length(states_);
            column = length(actions_);
            obj.state_num = eval(['length(',states_{1,1},')']);
            sz = [row, column];
            valType = cell(column, 1);
            for i = 1:1:column
                valType{i,1} = 'double';
            end
            obj.Q_table = table('Size', sz, 'VariableTypes', valType, 'VariableNames', actions_, 'RowNames', states_);
        end
        
        function obj = checkState(obj, state)
            if  isempty(find(strcmp(obj.Q_table.Properties.RowNames, state), 1))
                new_line = array2table(zeros(1, length(obj.actions)), 'VariableNames', obj.actions);
                obj.Q_table(end+1,:) = new_line;
                obj.Q_table.Properties.RowNames{end} = state;
            end
        end
        
        function [action_to_take] = getEpsilonAction(obj, state)
            obj = obj.checkState(state);
            choose = randi(100) / 100;
            if choose >= obj.epsilon
                [Q_max, Q_index] = max(obj.Q_table{state,:});
                action_to_take = obj.Q_table.Properties.VariableNames{Q_index};
                fprintf('[Optimal Action] %s \n', action_to_take);
            else
                action_num = length(obj.actions);
                action_to_take = obj.actions{randi(action_num)};
                fprintf('[Random Action] %s \n', action_to_take);
            end
        end
        
        function [action_to_take] = getDecayEpsilonAction(obj, state, epi_num)
            obj = obj.checkState(state);
            choose = randi(100) / 100;
            decay_epsilon = 1 / epi_num; % epsilon decays with the increase of episode number
            % when the epi_num approched infinity, epsilon decays to 0.
            % at this time the possibility of choosing the optimal action is the greatest
            if choose >= decay_epsilon
                [Q_max, Q_index] = max(obj.Q_table{state,:});
                action_to_take = obj.Q_table.Properties.VariableNames{Q_index};
                fprintf('[Optimal Action] %s \n', action_to_take);
            else
                action_num = length(obj.actions);
                action_to_take = obj.actions{randi(action_num)};
                fprintf('[Random Action] %s \n', action_to_take);
            end
        end
        
        function [action_to_take] = getOptimalAction(obj, state)
            obj = obj.checkState(state);
            [Q_max, Q_index] = max(obj.Q_table{state,:});
            action_to_take = obj.Q_table.Properties.VariableNames{Q_index};
            fprintf('[Optimal Action] %s \n', action_to_take);
        end
        
    end
end