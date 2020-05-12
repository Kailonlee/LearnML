function [v_lead] = getVLead(x)
% for i =1:1:5000
%     if  i <= point_1
%         v_lead(i,1) = v_1;
%     elseif i <= point_2
%         v_lead(i,1) = v_1 + (i - point_1) * v_gap;
%     elseif i <= point_3
%         v_lead(i,1) = v_2;
%     elseif i <= point_4
%         v_lead(i,1) = v_2 - (i - point_3) * v_gap;
%     else
%         v_lead(i,1) = v1;
%     end
% end
v_lead = 10 .* (x >= 0 & x < 10) +  x .* (x >= 10 & x < 20) ...
    + 20 .* (x >= 20 & x < 40) + (40 - 0.5*x) .* (x >= 40 & x < 50) + 15 .* (x >=50 & x<=60);
plot(x, v_lead, 'LineWidth', 2);
end