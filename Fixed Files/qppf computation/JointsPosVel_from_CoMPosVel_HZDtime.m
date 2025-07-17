function [q,qp,JPhi,JQ,Jd] = JointsPosVel_from_CoMPosVel_HZDtime(qf,qfp,robot,gait_parameters,t)
% Computation of Joints positions and velocities from CoM position and VELOCITIES
% Posiciones
%-----------------
q = InvGeometricHZDtime(qf,robot,gait_parameters,t);
%-------------------------
robot = robot_move(robot,q);

Case = 'velocity';
% "Case", "gait_parameters", "qfp" and "x" are used inside "OptionDesiredTrajectory"
OptionDesiredTrajectory;

% Velocities
%-----------------
%Jacobian (dQ/dq)
aux = zeros(3,30);
aux(:,4:6) = eye(3);
JQ = [J_state_vTest1(robot);robot.J_CoM;aux];
%Matrix Jd
Jd = [dhd_Phi;    
        1, 0, 0, 0, 0, 0, zeros(1,phiDim);
        0, 1, 0, 0, 0, 0, zeros(1,phiDim);
        0, 0, 1, 0, 0, 0, zeros(1,phiDim);
        0, 0, 0, 1, 0, 0, zeros(1,phiDim);
        0, 0, 0, 0, 1, 0, zeros(1,phiDim);
        0, 0, 0, 0, 0, 1, zeros(1,phiDim)];
JPhi = JQ\Jd;

%-------------------------
qp = JPhi*Phip;
%-------------------------