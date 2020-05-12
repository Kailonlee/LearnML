classdef SarsaLambda < RL
    properties
        lambda;
        E_table;
    end
    methods
        function obj = SarsaLambda(states_, actions_, alpha_, gama_, epsilon_, lambda_)
            obj = obj@RL(states_, actions_, alpha_, gama_, epsilon_);
            obj.E_table = obj.Q_table;
            obj.lambda = lambda_;
        end
        
        function obj = checkState(obj, state)
            if  isempty(find(strcmp(obj.Q_table.Properties.RowNames, state), 1))
                new_line = array2table(zeros(1, length(obj.actions)), 'VariableNames', obj.actions);
                obj.Q_table(end+1,:) = new_line;
                obj.Q_table.Properties.RowNames{end} = state;
                obj.E_table(end+1,:) = new_line;
                obj.E_table.Properties.RowNames{end} = state;
            end
        end
        
        function [obj] = updateLearn(obj, state, state_prime, action, action_prime, reward)
            obj = obj.checkState(state_prime);
            if strcmp(state_prime, 'end')
                q_target = reward;
            else
                q_target = reward + obj.gama * obj.Q_table{state_prime, action_prime};
            end
            q_predict = obj.Q_table{state, action};
            q_error = q_target - q_predict;
            obj.E_table{state, action} = obj.E_table{state, action} + 1;
            %update total Q_table and E_table
            obj.Q_table.Variables = obj.Q_table.Variables + obj.alpha * q_error .* obj.E_table.Variables;
            obj.E_table.Variables = obj.gama * obj.lambda .* obj.E_table.Variables;
        end
    end
end