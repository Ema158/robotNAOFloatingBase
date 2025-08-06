function q = desiredPosture(joints)
%% Assigning desired joint positions
baseDof = 6;
q = zeros(joints,1);

q(1) = 0.00;
q(2) = 0.00; 
q(3) = 0.01; 
q(4) = 0.0; 
q(5) = 0.0;
q(6) = 0.0;
%
q(1+baseDof) = 0;
q(2+baseDof) = 0; 
q(3+baseDof) = -0.6; %-0.6 -0.9
q(4+baseDof) = 0.7;  %0.7 1.3
q(5+baseDof) = -0.1; %-0.1 -0.4
q(6+baseDof) = 0;

q(7+baseDof) = 0;
q(8+baseDof) = 0;
q(9+baseDof) = -0.2; %-0.2 -0.9
q(10+baseDof) = 0.8; %0.8 1.3
q(11+baseDof) = -0.6; %-0.6 -0.4
q(12+baseDof) = 0;
%
q(13+baseDof) = 1.6; %1.6
q(14+baseDof) = 0;
q(15+baseDof) = 0; %1.6
q(16+baseDof) = 0; %0.2
q(17+baseDof) = 0.0;

q(18+baseDof) = -1.6; %-1.6
q(19+baseDof) = 0;
q(20+baseDof) = 0; %1.6
q(21+baseDof) = 0; %-0.2
q(22+baseDof) = 0.0;

q(23+baseDof) = 0;
q(24+baseDof) = 0;

% q(1) = 0.0254;
% q(2) = -0.0529; 
% q(3) = 0.0189; 
% q(4) = 0.1837; 
% q(5) = 0.0703;
% q(6) = -0.0122;
% %
% q(1+baseDof) = 0.0409;
% q(2+baseDof) = 0.0415; 
% q(3+baseDof) = -0.5583; %-0.6 -0.9
% q(4+baseDof) = 0.6508;  %0.7 1.3
% q(5+baseDof) = -0.1883; %-0.1 -0.4
% q(6+baseDof) = -0.227;
% 
% q(7+baseDof) = -0.0126;
% q(8+baseDof) = -0.0406;
% q(9+baseDof) = -0.2166; %-0.2 -0.9
% q(10+baseDof) = 0.7977; %0.8 1.3
% q(11+baseDof) = -0.6016; %-0.6 -0.4
% q(12+baseDof) = -0.0017;
% %
% q(13+baseDof) = 1.6; %1.6
% q(14+baseDof) = 0;
% q(15+baseDof) = 0; %1.6
% q(16+baseDof) = 0; %0.2
% q(17+baseDof) = 0.0;
% 
% q(18+baseDof) = -1.6; %-1.6
% q(19+baseDof) = 0;
% q(20+baseDof) = 0; %1.6
% q(21+baseDof) = 0; %-0.2
% q(22+baseDof) = 0.0;
% 
% q(23+baseDof)=0;
% q(24+baseDof)=0;
end