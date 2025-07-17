% This file compute the term JQp*qp (31x1) which is used to calculate "qpp" from Qpp = JQ*qpp + JQp*qp, i.e. 
% qpp = inv(JQ)[Qpp - JQp*qpp]
function [JQpqpR,JQpqpL] = JQp_qpFeet(robot)  
[~, ~, JpCoMqp, ~, Jpi_qp] = VelAceCoMs_FramesFB(robot,zeros(30,1));% Podemos utilizar tambien ésta función usando qpp = 0 =)
JQpqpR = Jpi_qp{8};
JQpqpL = Jpi_qp{15};
end

