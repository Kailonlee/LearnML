classdef Envoirment
    properties
        step;
        status;
        v_lead;
        T;
        observation_num;
        limits;
    end
    methods
        function obj = Envoirment( v_lead_, step_, status_, T_, ob_num, limits_)
            obj.step = step_;
            obj.status = status_;
            obj.v_lead = v_lead_;
            obj.T = T_;
            obj.observation_num = ob_num;
            obj.limits = limits_;
        end
        
        function obj = reset(obj)
            obj.step = 1;
            obj.status = 1;
        end
        
        function obj = update(obj)
            if obj.step < (length(obj.v_lead) - 1)
                obj.step = obj.step + 1;
            else
                obj.status = 0;
            end
        end
        
        function flag = isLegal(obj, state)
            state_ = eval(state);
            for i = 1:1:obj.observation_num
                if state_(i) >= eval(['obj.limits.state_',num2str(i),'_min']) ...
                        && state_(i) <= eval(['obj.limits.state_',num2str(i),'_max'])
                    flag = 1;
                else
                    flag = 0;
                end
            end
        end
        function [state_prime, reward, done] = takeStep(obj, state, action)
            state_prime = obj.getNextState(state, action);
            if obj.isLegal(state_prime)
                [reward, done, state_prime] = obj.getReward(state, state_prime, action);
            else % if state out of limits
                state_prime_ = eval(state_prime);
                fprintf('[Warn] next state %.2fm/s %.2fm is out of limits\n', state_prime_(1), state_prime_(2))
                state_prime = 'end';
                reward = -100;
                done = true;               
            end
        end
        
        function [state_prime] = getNextState(obj, state, action)
            acc = eval(action);
            state_ = eval(state);
            v_host = state_(1);
            d_lead = state_(2);
            v_host_prime = round(v_host + acc * obj.T);
            v_lead_prime = obj.v_lead(obj.step+1);
            d_lead_prime = round(d_lead + (v_lead_prime - v_host_prime) * obj.T); 
            state_prime = mat2str([v_host_prime, d_lead_prime]);
            
        end
        
        function [reward, done, state_prime] = getReward(obj, state, state_prime, action)
            % when next state out of limits or next state is absorbing,
            % return done is true
            done = false;
            state_ = eval(state);
            state_prime_ = eval(state_prime);
            v_host = state_(1);
            d_lead = state_(2);
            v_host_prime = state_prime_(1);
            d_lead_prime = state_prime_(2);
            v_lead_ = obj.v_lead(obj.step);
            v_lead_prime = obj.v_lead(obj.step+1);
            d_best = round(1.5 * v_lead_);
            d_best_prime = round(1.5 * v_lead_prime);
            v_rel = round(v_lead_ - v_host);
            v_rel_prime =round(v_lead_prime - v_host_prime);
            delta_d = round(d_best - d_lead);
            delta_d_prime = round(d_best_prime - d_lead_prime);
            if v_rel == 0 && delta_d ==0 && v_rel_prime == 0 && delta_d_prime == 0
                state_prime_ = eval(state_prime);
                fprintf('[Congratulation!] next state %.2fm/s %.2fm is absorbing\n', state_prime_(1), state_prime_(2))
                %%======================================================
                %%if you want iteration stops when state is absorbing, just uncomment following lines
                %                 done = true;
                %                 state_prime = 'end';
                %%======================================================
            end
            if  abs(v_rel_prime) == 0 && abs(delta_d_prime) == 0
                reward = 100;
            elseif d_lead_prime < 0.2 * d_best_prime %crash
                state_prime_ = eval(state_prime);
                fprintf('[Warn] next state %.2fm/s %.2fm is Crashing\n', state_prime_(1), state_prime_(2))
                reward = -100;
                done = true;
                state_prime = 'end';
            else
                reward = -abs(v_rel_prime) * obj.T - abs(delta_d_prime) - abs(eval(action)) * obj.T ^ 2;
            end
        end% end function
    end
end