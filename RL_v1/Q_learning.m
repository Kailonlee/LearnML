classdef Q_learning < RL
    properties
        
    end
    methods
        function obj = Q_learning(states_, actions_, alpha_, gama_, epsilon_)
            obj = obj@RL(states_, actions_, alpha_, gama_, epsilon_);
        end
        
        function [obj] = updateLearn(obj, state, state_prime, action, reward)
            obj = obj.checkState(state_prime);
            if strcmp(state_prime, 'end')
                obj.Q_table{state, action} = obj.Q_table{state, action} + obj.alpha * (reward - obj.Q_table{state, action});
            else
                obj.Q_table{state, action}= obj.Q_table{state, action} + obj.alpha * (reward + obj.gama * max(obj.Q_table{state_prime, :}) - obj.Q_table{state, action});
            end
        end
    end
end