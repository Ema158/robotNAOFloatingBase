function [ XD ] = dynam_FloatingBaseTest1(t,X)
%%Simple support dynamic model (zero dynamics) for the robot model
%   State variable (x, y, xp, yp)
global robot gait_parameters
% ==============================================================================================
global contB 
% Dynamics
%  ---------------------
qfp = X(7:12);
global OutOfWorkSpace
if isempty(OutOfWorkSpace)
    [qfpp, ~, ~, ~, ~, ~, ~] = Desired_qfpp_FloatingBaseTest1(robot,X,gait_parameters,t);
else
    qfpp = zeros(12,1);
    fprintf('Iteration %d. CoM OUT of WORKSPACE!. Essential model NOT computed.  \n',contB);
end

% Output
XD = [qfp; qfpp];
contB = contB + 1;
end