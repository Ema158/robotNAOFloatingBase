% This file compute the term JQp*qp (31x1) which is used to calculate "qpp" from Qpp = JQ*qpp + JQp*qp, i.e. 
% qpp = inv(JQ)[Qpp - JQp*qpp]
function JQpqp = JQp_qp(robot)  
JQpqp = zeros(robot.joints,1);
[~, ~, JpCoMqp, ~, Jpi_qp] = VelAceCoMs_Frames(robot,zeros(30,1));% Podemos utilizar tambien ésta función usando qpp = 0 =)
JQpqp(25:27) = JpCoMqp(1:3);
end

