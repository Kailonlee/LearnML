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
% v_lead = 10 .* (x >= 0 & x < 10) +  10 .* (x >= 10 & x < 20) ...
%     + 10 .* (x >= 20 & x < 40) + 10 .* (x >= 40 & x < 50) + 10 .* (x >=50 & x<=60);
v_lead = (0.2*x+10) .* (x >= 0 & x <= 60);
plot(x, v_lead, 'LineWidth', 2);
end